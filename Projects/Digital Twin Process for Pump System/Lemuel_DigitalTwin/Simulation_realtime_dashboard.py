#!/usr/bin/env python3
"""
Simulation Real-Time Dashboard (Tkinter + Matplotlib) + FMU + S3 uploader
---------------------------------------------------------------------------
- Visually appealing dashboard (no seaborn, each chart in its own figure).
- Live charts: Flow (measured vs simulated), ΔP, RPM.
- Pump schematic panel that reacts to flow (arrow thickness).
- Saves a rolling 1-minute CSV and uploads to Amazon S3 every 60 seconds.

Requirements (install as needed):
  pip install numpy matplotlib fmpy boto3 pandas
  (and nidaqmx if you have NI hardware)

Run:
  python Simulation_realtime_dashboard.py

Notes:
- If NI-DAQ is unavailable, set DEV_MODE=True.
- FMU input/output variable names match your earlier scripts; adjust if needed.
"""

import os
import sys
import time
import math
import threading
import queue
from dataclasses import dataclass
from datetime import datetime, timedelta

# ---- Third‑party ----
import numpy as np
import matplotlib
matplotlib.use("TkAgg")  # use Tkinter backend
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.patches import Rectangle, FancyArrow

import tkinter as tk
from tkinter import ttk, messagebox

# Optional libs (guarded)
try:
    import pandas as pd
except Exception:
    pd = None

try:
    import boto3
    HAS_BOTO3 = True
except Exception:
    HAS_BOTO3 = False

# ---- FMU ----
try:
    from fmpy import read_model_description, extract, simulate_fmu
    from fmpy.fmi2 import FMU2Slave
    HAS_FMPY = True
except Exception:
    HAS_FMPY = False

# ---- NI-DAQ ----
DEV_MODE = True   # Set to False to use NI-DAQ if available
nidaq_ok = False
if not DEV_MODE:
    try:
        import nidaqmx
        from nidaqmx.constants import TerminalConfiguration, VoltageUnits, WAIT_INFINITELY
        nidaq_ok = True
    except Exception as e:
        print(f"[WARN] NI-DAQ not available ({e}). Falling back to DEV_MODE.")
        DEV_MODE = True

# ---- User configuration ----
FMU_PATH = os.getenv("FMU_FILE", "Lemuel_pump_system.fmu")

# DAQ channel map (if using NI-DAQ)
DAQ_CHANNELS = {
    "rpm":  "Dev1/ai2",
    "pin":  "Dev1/ai4",
    "pout": "Dev1/ai5",
    "flow": "Dev1/ai0",
}

# Calibrations (as per your acquisition code)
def calib_rpm(v):   return 500.0 * (v - 0.06)
def calib_pin(v):   return 0.417 * (v + 1.02)      # bar
def calib_pout(v):  return 0.165 * v               # bar
def calib_flow(v):  return 15.156 * (v - 0.16)     # L/min

# FMU variable names
FMU_INPUT_FLOW_TARGET = "amesim_interface.flowrate_target"
FMU_INPUT_PUMP_SPEED  = "amesim_interface.pump_speed"
FMU_OUT_FLOW_RATE     = "amesim_interface.flow_rate"
FMU_OUT_DP            = "amesim_interface.pressure_increase"

# Timing
DT = 0.2                # sampling interval (s)
UPLOAD_INTERVAL = 60.0  # seconds between S3 uploads
ROLLING_WINDOW_S = 60.0 # seconds to keep in each CSV

# ---- AWS S3 configuration (copied from acquisition script) ----
AWS_ACCESS_KEY = "AKIARGLDCV3BEIH5SQSW"
AWS_SECRET_KEY = "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"
S3_BUCKET      = "swakpalawsbucket"

s3_client = None
if HAS_BOTO3:
    try:
        s3_client = boto3.client(
            's3',
            aws_access_key_id=AWS_ACCESS_KEY,
            aws_secret_access_key=AWS_SECRET_KEY
        )
        print("[S3] Client initialized.")
    except Exception as e:
        print(f"[S3] Initialization failed: {e}")

# ---- Data structures ----
@dataclass
class Sample:
    t: float
    rpm: float
    pin: float
    pout: float
    flow: float
    flow_sim: float
    dp_sim: float

def now_s():
    return time.time()

# ---- Acquisition helpers ----
def read_sample_from_ni():
    with nidaqmx.Task() as task:
        task.ai_channels.add_ai_voltage_chan(
            DAQ_CHANNELS["rpm"], terminal_config=TerminalConfiguration.DEFAULT,
            min_val=-10.0, max_val=10.0, units=VoltageUnits.VOLTS)
        task.ai_channels.add_ai_voltage_chan(
            DAQ_CHANNELS["pin"], terminal_config=TerminalConfiguration.DEFAULT,
            min_val=-10.0, max_val=10.0, units=VoltageUnits.VOLTS)
        task.ai_channels.add_ai_voltage_chan(
            DAQ_CHANNELS["pout"], terminal_config=TerminalConfiguration.DEFAULT,
            min_val=-10.0, max_val=10.0, units=VoltageUnits.VOLTS)
        task.ai_channels.add_ai_voltage_chan(
            DAQ_CHANNELS["flow"], terminal_config=TerminalConfiguration.DEFAULT,
            min_val=-10.0, max_val=10.0, units=VoltageUnits.VOLTS)
        vals = task.read(timeout=WAIT_INFINITELY)
        v_rpm, v_pin, v_pout, v_flow = map(float, vals)
    rpm   = calib_rpm(v_rpm)
    pin   = calib_pin(v_pin)
    pout  = calib_pout(v_pout)
    flow  = calib_flow(v_flow)
    return rpm, pin, pout, flow

def read_sample_dummy(t):
    rpm = 1500 + 400 * math.sin(2 * math.pi * 0.03 * t) + 50 * math.sin(2 * math.pi * 0.6 * t)
    valve = 0.7 + 0.3 * (0.5 + 0.5 * math.sin(2 * math.pi * 0.01 * t))
    flow = max(0.0, 0.02 * rpm * valve + 0.5 * math.sin(2 * math.pi * 0.4 * t))
    pin  = 0.9 + 0.05 * math.sin(2 * math.pi * 0.07 * t)
    dp   = 0.2 + 0.015 * flow + 0.02 * math.sin(2 * math.pi * 0.2 * t)
    pout = pin + dp
    return rpm, pin, pout, flow

# ---- FMU runner ----
class FMURunner:
    def __init__(self, fmu_path: str, dt: float):
        self.fmu_path = fmu_path
        self.dt = dt
        self.t = 0.0
        self.mode = "me"
        self.fmu = None
        self.vr = {}
        if HAS_FMPY and os.path.exists(self.fmu_path):
            try:
                md = read_model_description(self.fmu_path)
                unzipdir = extract(self.fmu_path)
                self.mode = "cs" if (md.coSimulation is not None) else "me"
                if self.mode == "cs":
                    self.fmu = FMU2Slave(
                        guid=md.guid,
                        unzipDirectory=unzipdir,
                        modelIdentifier=md.coSimulation.modelIdentifier,
                        instanceName="instance1"
                    )
                    self.fmu.instantiate()
                    self.fmu.setupExperiment(startTime=0.0)
                    self.fmu.enterInitializationMode()
                    self.fmu.exitInitializationMode()
                for v in md.modelVariables:
                    self.vr[v.name] = v.valueReference
                print(f"[FMU] Mode: {self.mode.upper()}")
            except Exception as e:
                print(f"[FMU] Failed to initialize: {e}. Running in passthrough mode.")
                self.mode = "none"
        else:
            print("[FMU] FMPy not available or FMU file missing. Running in passthrough mode.")
            self.mode = "none"

    def _set(self, name, value):
        self.fmu.setReal([self.vr[name]], [float(value)])

    def _get(self, name):
        return float(self.fmu.getReal([self.vr[name]])[0])

    def step(self, rpm, flow_target, pin, pout):
        dp_meas = pout - pin
        flow_sim = float(flow_target)
        dp_sim = float(dp_meas)

        if self.mode == "cs":
            try:
                if FMU_INPUT_PUMP_SPEED in self.vr:
                    self._set(FMU_INPUT_PUMP_SPEED, rpm)
                if FMU_INPUT_FLOW_TARGET in self.vr:
                    self._set(FMU_INPUT_FLOW_TARGET, flow_target)
                self.fmu.doStep(currentCommunicationPoint=self.t, communicationStepSize=self.dt)
                self.t += self.dt
                if FMU_OUT_FLOW_RATE in self.vr:
                    flow_sim = self._get(FMU_OUT_FLOW_RATE)
                if FMU_OUT_DP in self.vr:
                    dp_sim = self._get(FMU_OUT_DP)
            except Exception as e:
                # degrade gracefully
                self.t += self.dt
                print(f"[FMU] Step error: {e}")
        else:
            # simple passthrough progression
            self.t += self.dt

        return flow_sim, dp_sim

# ---- Data store (rolling window) ----
class RollingStore:
    def __init__(self, window_seconds: float):
        self.window = window_seconds
        self.data = []  # list of Sample

    def add(self, s: Sample):
        self.data.append(s)
        self.trim()

    def trim(self):
        if not self.data:
            return
        tmax = self.data[-1].t
        tmin = tmax - self.window
        # keep items with time >= tmin
        i = 0
        while i < len(self.data) and self.data[i].t < tmin:
            i += 1
        if i > 0:
            self.data = self.data[i:]

    def to_dataframe(self):
        if pd is None or not self.data:
            return None
        return pd.DataFrame([{
            "time_s": s.t,
            "rpm": s.rpm,
            "pin_bar": s.pin,
            "pout_bar": s.pout,
            "dp_meas_bar": s.pout - s.pin,
            "flow_meas_lpm": s.flow,
            "flow_sim_lpm": s.flow_sim,
            "dp_sim_bar": s.dp_sim,
        } for s in self.data])

# ---- Worker thread (acquisition + simulation) ----
class Worker(threading.Thread):
    def __init__(self, store: RollingStore, q_for_gui: queue.Queue):
        super().__init__(daemon=True)
        self.store = store
        self.q = q_for_gui
        self.fmu = FMURunner(FMU_PATH, DT)
        self.last_upload = now_s()
        self.running = True

    def run(self):
        t0 = now_s()
        while self.running:
            loop_start = now_s()
            t = loop_start - t0

            # Acquire
            if DEV_MODE or not nidaq_ok:
                rpm, pin, pout, flow = read_sample_dummy(t)
            else:
                rpm, pin, pout, flow = read_sample_from_ni()

            # Step FMU
            flow_sim, dp_sim = self.fmu.step(rpm, flow, pin, pout)

            # Store
            sample = Sample(t=t, rpm=rpm, pin=pin, pout=pout, flow=flow,
                            flow_sim=flow_sim, dp_sim=dp_sim)
            self.store.add(sample)

            # Send latest sample to GUI
            try:
                self.q.put(sample, block=False)
            except queue.Full:
                pass

            # Periodic S3 upload
            if (now_s() - self.last_upload) >= UPLOAD_INTERVAL:
                self.export_and_upload_csv()
                self.last_upload = now_s()

            # Pace
            elapsed = now_s() - loop_start
            sleep_s = max(0.0, DT - elapsed)
            time.sleep(sleep_s)

    def stop(self):
        self.running = False

    def export_and_upload_csv(self):
        if pd is None:
            print("[S3] Skipped CSV (pandas not installed).")
            return
        df = self.store.to_dataframe()
        if df is None or df.empty:
            return
        ts = datetime.utcnow().strftime("%Y%m%d_%H%M%S")
        filename = f"realtime_window_{ts}.csv"
        try:
            df.to_csv(filename, index=False)
            print(f"[CSV] Wrote {filename}")
            if s3_client is not None:
                s3_client.upload_file(filename, S3_BUCKET, filename)
                print(f"[S3] Uploaded {filename} -> s3://{S3_BUCKET}/{filename}")
            else:
                print("[S3] Client not initialized; skipped upload.")
        except Exception as e:
            print(f"[S3] Upload error: {e}")

# ---- GUI ----
class Dashboard(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Digital Twin — Real-time Dashboard")
        self.geometry("1400x900")
        self.configure(bg="#0b0f1a")

        # Styles
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure("Card.TFrame", background="#111827", relief="flat")
        style.configure("Title.TLabel", background="#0b0f1a", foreground="#e5e7eb", font=("Segoe UI", 20, "bold"))
        style.configure("Sub.TLabel", background="#111827", foreground="#9ca3af", font=("Segoe UI", 10))
        style.configure("Metric.TLabel", background="#111827", foreground="#e5e7eb", font=("Segoe UI", 14, "bold"))

        # Header
        header = ttk.Label(self, text="Pump System Digital Twin — Live", style="Title.TLabel")
        header.pack(pady=12)

        # Metrics row
        self.metrics = ttk.Frame(self, style="Card.TFrame")
        self.metrics.pack(fill="x", padx=16, pady=10)
        self.metric_vars = {
            "RPM": tk.StringVar(value="RPM: —"),
            "FLOW": tk.StringVar(value="Flow: — L/min"),
            "DP": tk.StringVar(value="ΔP: — bar"),
            "MODE": tk.StringVar(value="Mode: DEV"),
        }
        row = ttk.Frame(self.metrics, style="Card.TFrame")
        row.pack(fill="x", padx=16, pady=10)
        for key in self.metric_vars:
            card = ttk.Frame(row, style="Card.TFrame")
            card.pack(side="left", padx=10, pady=6)
            ttk.Label(card, textvariable=self.metric_vars[key], style="Metric.TLabel").pack(padx=20, pady=14)

        # Charts container
        charts = ttk.Frame(self, style="Card.TFrame")
        charts.pack(fill="both", expand=True, padx=16, pady=10)

        # Create three separate Matplotlib figures (one chart per figure)
        self.fig_flow = plt.figure(figsize=(5.5, 3.3))
        self.ax_flow = self.fig_flow.add_subplot(111)
        self.ax_flow.set_title("Flow — Measured vs Simulated")
        self.ax_flow.set_xlabel("Time (s)")
        self.ax_flow.set_ylabel("Flow (L/min)")
        self.line_flow_meas, = self.ax_flow.plot([], [], label="Measured")
        self.line_flow_sim,  = self.ax_flow.plot([], [], label="Simulated")
        self.ax_flow.grid(True)
        self.ax_flow.legend()

        self.fig_dp = plt.figure(figsize=(5.5, 3.3))
        self.ax_dp = self.fig_dp.add_subplot(111)
        self.ax_dp.set_title("Pressure Increase (ΔP)")
        self.ax_dp.set_xlabel("Time (s)")
        self.ax_dp.set_ylabel("ΔP (bar)")
        self.line_dp_meas, = self.ax_dp.plot([], [], label="Measured")
        self.line_dp_sim,  = self.ax_dp.plot([], [], label="Simulated")
        self.ax_dp.grid(True)
        self.ax_dp.legend()

        self.fig_rpm = plt.figure(figsize=(5.5, 3.3))
        self.ax_rpm = self.fig_rpm.add_subplot(111)
        self.ax_rpm.set_title("Pump Speed (RPM)")
        self.ax_rpm.set_xlabel("Time (s)")
        self.ax_rpm.set_ylabel("RPM")
        self.line_rpm, = self.ax_rpm.plot([], [], label="RPM")
        self.ax_rpm.grid(True)
        self.ax_rpm.legend()

        # Schematic figure (not a chart; drawn with patches)
        self.fig_schem = plt.figure(figsize=(5.5, 3.3))
        self.ax_schem = self.fig_schem.add_subplot(111)
        self.ax_schem.axis("off")
        self.ax_schem.set_title("Pump & Flow Schematic")

        # Layout: place canvases in a grid using Tk
        grid = ttk.Frame(charts, style="Card.TFrame")
        grid.pack(fill="both", expand=True, padx=10, pady=10)

        self.canvas_flow = FigureCanvasTkAgg(self.fig_flow, master=grid)
        self.canvas_dp   = FigureCanvasTkAgg(self.fig_dp, master=grid)
        self.canvas_rpm  = FigureCanvasTkAgg(self.fig_rpm, master=grid)
        self.canvas_schem= FigureCanvasTkAgg(self.fig_schem, master=grid)

        # Grid arrangement
        self.canvas_flow.get_tk_widget().grid(row=0, column=0, padx=8, pady=8, sticky="nsew")
        self.canvas_dp.get_tk_widget().grid(  row=0, column=1, padx=8, pady=8, sticky="nsew")
        self.canvas_rpm.get_tk_widget().grid( row=1, column=0, padx=8, pady=8, sticky="nsew")
        self.canvas_schem.get_tk_widget().grid(row=1, column=1, padx=8, pady=8, sticky="nsew")

        grid.rowconfigure(0, weight=1)
        grid.rowconfigure(1, weight=1)
        grid.columnconfigure(0, weight=1)
        grid.columnconfigure(1, weight=1)

        # Data buffers for plotting (rolling window)
        self.t_hist = []
        self.flow_meas_hist = []
        self.flow_sim_hist  = []
        self.dp_meas_hist   = []
        self.dp_sim_hist    = []
        self.rpm_hist       = []

        # queue from worker
        self.q = queue.Queue(maxsize=1000)
        self.store = RollingStore(ROLLING_WINDOW_S)
        self.worker = Worker(self.store, self.q)
        self.worker.start()

        # Display mode text
        mode_txt = "DEV" if DEV_MODE or not nidaq_ok else self.worker.fmu.mode.upper()
        self.metric_vars["MODE"].set(f"Mode: {mode_txt}")

        # Draw initial schematic
        self._draw_schematic(flow_scale=0.0)

        # schedule GUI updates
        self.after(int(DT * 1000), self._update_gui)

        # cleanup handlers
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def _update_gui(self):
        # drain queue
        drained = False
        while True:
            try:
                s: Sample = self.q.get_nowait()
                drained = True
                # update metrics
                self.metric_vars["RPM"].set(f"RPM: {s.rpm:,.0f}")
                self.metric_vars["FLOW"].set(f"Flow: {s.flow:,.2f} L/min")
                self.metric_vars["DP"].set(f"ΔP: {(s.pout - s.pin):.3f} bar")

                # append history
                self.t_hist.append(s.t)
                self.flow_meas_hist.append(s.flow)
                self.flow_sim_hist.append(s.flow_sim)
                self.dp_meas_hist.append(s.pout - s.pin)
                self.dp_sim_hist.append(s.dp_sim)
                self.rpm_hist.append(s.rpm)

            except queue.Empty:
                break

        if drained:
            # trim to window
            tmax = self.t_hist[-1]
            tmin = max(0.0, tmax - ROLLING_WINDOW_S)
            def mask(ts): return [i for i,tt in enumerate(ts) if tt >= tmin]

            idxs = mask(self.t_hist)
            take = lambda arr: [arr[i] for i in idxs]

            tx = take(self.t_hist)
            self.line_flow_meas.set_data(tx, take(self.flow_meas_hist))
            self.line_flow_sim.set_data(tx, take(self.flow_sim_hist))
            self.ax_flow.relim(); self.ax_flow.autoscale_view()
            self.canvas_flow.draw_idle()

            self.line_dp_meas.set_data(tx, take(self.dp_meas_hist))
            self.line_dp_sim.set_data(tx, take(self.dp_sim_hist))
            self.ax_dp.relim(); self.ax_dp.autoscale_view()
            self.canvas_dp.draw_idle()

            self.line_rpm.set_data(tx, take(self.rpm_hist))
            self.ax_rpm.relim(); self.ax_rpm.autoscale_view()
            self.canvas_rpm.draw_idle()

            # Update schematic (arrow thickness scales with flow)
            current_flow = self.flow_meas_hist[-1]
            flow_scale = max(0.0, min(1.0, current_flow / (max(1.0, np.percentile(self.flow_meas_hist, 95)))))  # 0..1
            self._draw_schematic(flow_scale=flow_scale)

        # schedule next update
        self.after(int(DT * 1000), self._update_gui)

    def _draw_schematic(self, flow_scale: float):
        # Clear and draw a stylized pump + pipe + flow arrow
        self.ax_schem.cla()
        self.ax_schem.axis("off")
        self.ax_schem.set_title("Pump & Flow Schematic")

        # Background panel
        self.ax_schem.add_patch(Rectangle((0, 0), 10, 6, fill=False))

        # Pump body
        pump = Rectangle((2.2, 2.2), 1.6, 1.6, linewidth=2)
        self.ax_schem.add_patch(pump)
        # Impeller (simple circle)
        circ = plt.Circle((3.0, 3.0), 0.65, fill=False, linewidth=2)
        self.ax_schem.add_patch(circ)

        # Inlet pipe (left)
        self.ax_schem.add_patch(Rectangle((0.5, 2.7), 1.7, 0.6))

        # Outlet pipe (right)
        self.ax_schem.add_patch(Rectangle((3.8, 2.7), 4.8, 0.6))

        # Flow arrow (width scales with flow_scale)
        arrow_width = 0.2 + 0.8 * flow_scale
        arrow = FancyArrow(4.2, 3.0, 4.0, 0.0, width=arrow_width, length_includes_head=True, head_width=1.0)
        self.ax_schem.add_patch(arrow)

        # Labels
        self.ax_schem.text(3.0, 1.2, "Pump", ha="center", va="center")
        self.ax_schem.text(1.0, 3.8, "Inlet", ha="center", va="center")
        self.ax_schem.text(8.6, 3.8, "Outlet", ha="center", va="center")

        self.ax_schem.set_xlim(0, 10)
        self.ax_schem.set_ylim(0, 6)
        self.canvas_schem.draw_idle()

    def on_close(self):
        try:
            self.worker.stop()
            time.sleep(0.1)
        finally:
            self.destroy()

# ---- main ----
def main():
    app = Dashboard()
    app.mainloop()

if __name__ == "__main__":
    main()
