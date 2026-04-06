#!/usr/bin/env python3
"""
Interactive Dashboard for Simulation_realtime.py (Flow & ΔP)
------------------------------------------------------------
- Start/Stop controls
- Real-time charts: Flow (measured vs simulated), ΔP (measured vs simulated)
- Pump schematic that responds to flow
- Uses NI-DAQ if available; otherwise DEV_MODE generates dummy signals
- FMU step behavior mirrors Simulation_realtime.py

Run:
  python Simulation_realtime_dashboard_v2.py

Requirements:
  pip install numpy matplotlib fmpy
  (optional) pip install nidaqmx
"""

import os
import sys
import time
import math
import threading
import queue
from dataclasses import dataclass

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.patches import Rectangle, FancyArrow

import tkinter as tk
from tkinter import ttk

# -------- User configuration ----------
FMU_PATH = os.getenv("FMU_FILE", "Lemuel_pump_system.fmu")

# Channels (if using NI-DAQ)
DAQ_CHANNELS = {
    "rpm":  "Dev1/ai2",
    "pin":  "Dev1/ai4",
    "pout": "Dev1/ai5",
    "flow": "Dev1/ai0",
}

# Calibrations
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
DT = 0.10               # sample period (s)
ROLLING_WINDOW_S = 60.0 # seconds

# DEV/DAQ
DEV_MODE = False  # set True to force dummy data
nidaq_ok = False
if not DEV_MODE:
    try:
        import nidaqmx
        from nidaqmx.constants import TerminalConfiguration, VoltageUnits, WAIT_INFINITELY
        nidaq_ok = True
    except Exception as e:
        print(f"[WARN] NI-DAQ not available ({e}). Switching to DEV_MODE.", file=sys.stderr)
        DEV_MODE = True

# FMPy
try:
    from fmpy import read_model_description, extract, simulate_fmu
    from fmpy.fmi2 import FMU2Slave
    HAS_FMPY = True
except Exception as e:
    print(f"[WARN] FMPy not available ({e}). Running without FMU.")
    HAS_FMPY = False


@dataclass
class Sample:
    t: float
    rpm: float
    pin: float
    pout: float
    flow: float
    flow_sim: float
    dp_sim: float


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


class FMURunner:
    """Mimics Simulation_realtime.py behavior: Co-Sim if available, else re-simulate."""
    def __init__(self, fmu_path: str, dt: float):
        self.fmu_path = fmu_path
        self.dt = dt
        self.mode = "unknown"
        self.t = 0.0
        self.fmu = None
        self.vr = {}
        self.model_desc = None
        self.unzipdir = None
        self.history = {"t": [], "rpm": [], "flow_meas": [], "dp_meas": []}

        if HAS_FMPY and os.path.exists(fmu_path):
            try:
                self.model_desc = read_model_description(fmu_path)
                self.unzipdir = extract(fmu_path)
                self.mode = "cs" if (self.model_desc.coSimulation is not None) else "me"
                if self.mode == "cs":
                    self.fmu = FMU2Slave(
                        guid=self.model_desc.guid,
                        unzipDirectory=self.unzipdir,
                        modelIdentifier=self.model_desc.coSimulation.modelIdentifier,
                        instanceName="instance1"
                    )
                    self.fmu.instantiate()
                    self.fmu.setupExperiment(startTime=0.0)
                    self.fmu.enterInitializationMode()
                    self.fmu.exitInitializationMode()
                for v in self.model_desc.modelVariables:
                    self.vr[v.name] = v.valueReference
            except Exception as e:
                print(f"[FMU] init failed: {e}")
                self.mode = "none"
        else:
            self.mode = "none"

    def _set(self, name, value):
        self.fmu.setReal([self.vr[name]], [float(value)])

    def _get(self, name):
        return float(self.fmu.getReal([self.vr[name]])[0])

    def step(self, rpm, flow_target, pin, pout):
        dp_meas = pout - pin

        if self.mode == "cs" and self.fmu is not None:
            try:
                if FMU_INPUT_PUMP_SPEED in self.vr:
                    self._set(FMU_INPUT_PUMP_SPEED, rpm)
                if FMU_INPUT_FLOW_TARGET in self.vr:
                    self._set(FMU_INPUT_FLOW_TARGET, flow_target)
                self.fmu.doStep(currentCommunicationPoint=self.t, communicationStepSize=self.dt)
                self.t += self.dt
                flow_sim = self._get(FMU_OUT_FLOW_RATE) if FMU_OUT_FLOW_RATE in self.vr else float('nan')
                dp_sim   = self._get(FMU_OUT_DP)         if FMU_OUT_DP in self.vr         else float('nan')
            except Exception as e:
                print(f"[FMU] step error: {e}")
                self.t += self.dt
                flow_sim, dp_sim = flow_target, dp_meas
        elif self.mode == "me":
            # Re-simulate with accumulated inputs (simplified to last point for responsiveness)
            self.t += self.dt
            flow_sim, dp_sim = flow_target, dp_meas
        else:
            self.t += self.dt
            flow_sim, dp_sim = flow_target, dp_meas

        # update history (for potential ME resim use)
        self.history["t"].append(self.t)
        self.history["rpm"].append(rpm)
        self.history["flow_meas"].append(flow_target)
        self.history["dp_meas"].append(dp_meas)

        # rolling trim
        while len(self.history["t"]) > 0 and (self.history["t"][-1] - self.history["t"][0]) > ROLLING_WINDOW_S:
            for k in self.history:
                self.history[k].pop(0)

        return flow_sim, dp_sim

    def terminate(self):
        if self.mode == "cs" and self.fmu is not None:
            try:
                self.fmu.terminate()
            except Exception:
                pass
            try:
                self.fmu.freeInstance()
            except Exception:
                pass


class RollingBuffer:
    def __init__(self, window_s: float):
        self.window = window_s
        self.t = []
        self.flow_meas = []
        self.flow_sim  = []
        self.dp_meas   = []
        self.dp_sim    = []

    def add(self, t, flow_meas, flow_sim, dp_meas, dp_sim):
        self.t.append(t)
        self.flow_meas.append(flow_meas)
        self.flow_sim.append(flow_sim)
        self.dp_meas.append(dp_meas)
        self.dp_sim.append(dp_sim)
        self.trim()

    def trim(self):
        if not self.t:
            return
        tmax = self.t[-1]
        tmin = tmax - self.window
        # drop from front
        i = 0
        while i < len(self.t) and self.t[i] < tmin:
            i += 1
        if i > 0:
            self.t = self.t[i:]
            self.flow_meas = self.flow_meas[i:]
            self.flow_sim  = self.flow_sim[i:]
            self.dp_meas   = self.dp_meas[i:]
            self.dp_sim    = self.dp_sim[i:]


class Worker(threading.Thread):
    def __init__(self, out_q: queue.Queue):
        super().__init__(daemon=True)
        self.q = out_q
        self.running = False
        self.fmu = FMURunner(FMU_PATH, DT)
        self.t0 = None

    def start_run(self):
        self.running = True
        if self.t0 is None:
            self.t0 = time.time()

    def stop_run(self):
        self.running = False

    def run(self):
        while True:
            if self.running:
                t = time.time() - self.t0
                if DEV_MODE or not nidaq_ok:
                    rpm, pin, pout, flow = read_sample_dummy(t)
                else:
                    rpm, pin, pout, flow = read_sample_from_ni()
                flow_sim, dp_sim = self.fmu.step(rpm, flow, pin, pout)
                s = Sample(t=t, rpm=rpm, pin=pin, pout=pout, flow=flow,
                           flow_sim=flow_sim, dp_sim=dp_sim)
                try:
                    self.q.put(s, timeout=0.1)
                except queue.Full:
                    pass
                time.sleep(DT)
            else:
                time.sleep(0.05)


class Dashboard(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Pump Digital Twin — Flow & ΔP Dashboard")
        self.geometry("1280x840")
        self.configure(bg="#0b0f1a")

        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure("Card.TFrame", background="#111827", relief="flat")
        style.configure("Title.TLabel", background="#0b0f1a", foreground="#e5e7eb", font=("Segoe UI", 20, "bold"))
        style.configure("Metric.TLabel", background="#111827", foreground="#e5e7eb", font=("Segoe UI", 12, "bold"))
        style.configure("Hint.TLabel", background="#0b0f1a", foreground="#9ca3af", font=("Segoe UI", 10))

        header = ttk.Label(self, text="Pump System — Real-time vs Simulated (Flow & ΔP)", style="Title.TLabel")
        header.pack(pady=10)

        controls = ttk.Frame(self, style="Card.TFrame")
        controls.pack(fill="x", padx=16, pady=8)
        self.btn_start = ttk.Button(controls, text="Start", command=self._on_start)
        self.btn_stop  = ttk.Button(controls, text="Stop",  command=self._on_stop, state="disabled")
        self.btn_start.pack(side="left", padx=8, pady=8)
        self.btn_stop.pack(side="left", padx=8, pady=8)
        ttk.Label(controls, text="Use Start/Stop to control acquisition and FMU stepping.", style="Hint.TLabel").pack(side="left", padx=16)

        metrics = ttk.Frame(self, style="Card.TFrame")
        metrics.pack(fill="x", padx=16, pady=10)
        self.var_rpm = tk.StringVar(value="RPM: —")
        self.var_flow = tk.StringVar(value="Flow: — L/min")
        self.var_dp = tk.StringVar(value="ΔP: — bar")
        for var in (self.var_rpm, self.var_flow, self.var_dp):
            card = ttk.Frame(metrics, style="Card.TFrame"); card.pack(side="left", padx=10, pady=6)
            ttk.Label(card, textvariable=var, style="Metric.TLabel").pack(padx=18, pady=12)

        body = ttk.Frame(self, style="Card.TFrame")
        body.pack(fill="both", expand=True, padx=16, pady=10)

        self.fig_flow = plt.figure(figsize=(5.0, 3.2))
        self.ax_flow = self.fig_flow.add_subplot(111)
        self.ax_flow.set_title("Flow (L/min) — Measured vs Simulated")
        self.ax_flow.set_xlabel("Time (s)")
        self.ax_flow.set_ylabel("Flow (L/min)")
        self.line_flow_meas, = self.ax_flow.plot([], [], label="Measured", linestyle="--")
        self.line_flow_sim,  = self.ax_flow.plot([], [], label="Simulated")
        self.ax_flow.grid(True)
        self.ax_flow.legend()
        self.canvas_flow = FigureCanvasTkAgg(self.fig_flow, master=body)
        self.canvas_flow.get_tk_widget().grid(row=0, column=0, padx=8, pady=8, sticky="nsew")

        self.fig_dp = plt.figure(figsize=(5.0, 3.2))
        self.ax_dp = self.fig_dp.add_subplot(111)
        self.ax_dp.set_title("Pressure Difference ΔP (bar) — Measured vs Simulated")
        self.ax_dp.set_xlabel("Time (s)")
        self.ax_dp.set_ylabel("ΔP (bar)")
        self.line_dp_meas, = self.ax_dp.plot([], [], label="Measured", linestyle="--")
        self.line_dp_sim,  = self.ax_dp.plot([], [], label="Simulated")
        self.ax_dp.grid(True)
        self.ax_dp.legend()
        self.canvas_dp = FigureCanvasTkAgg(self.fig_dp, master=body)
        self.canvas_dp.get_tk_widget().grid(row=1, column=0, padx=8, pady=8, sticky="nsew")

        self.fig_schem = plt.figure(figsize=(5.0, 6.6))
        self.ax_schem = self.fig_schem.add_subplot(111)
        self.ax_schem.axis("off")
        self.ax_schem.set_title("Pump & Flow Schematic")
        self.canvas_schem = FigureCanvasTkAgg(self.fig_schem, master=body)
        self.canvas_schem.get_tk_widget().grid(row=0, column=1, rowspan=2, padx=8, pady=8, sticky="nsew")

        body.rowconfigure(0, weight=1)
        body.rowconfigure(1, weight=1)
        body.columnconfigure(0, weight=1)
        body.columnconfigure(1, weight=1)

        self.buff = RollingBuffer(ROLLING_WINDOW_S)

        self.q = queue.Queue(maxsize=1000)
        self.worker = Worker(self.q)
        self.worker.start()

        self.after(int(DT * 1000), self._tick)

        self._draw_schematic(0.0)

        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _on_start(self):
        self.worker.start_run()
        self.btn_start.config(state="disabled")
        self.btn_stop.config(state="normal")

    def _on_stop(self):
        self.worker.stop_run()
        self.btn_start.config(state="normal")
        self.btn_stop.config(state="disabled")

    def _on_close(self):
        try:
            self.worker.stop_run()
        finally:
            self.destroy()

    def _tick(self):
        updated = False
        while True:
            try:
                s: Sample = self.q.get_nowait()
            except queue.Empty:
                break

            updated = True
            dp_meas = s.pout - s.pin
            self.buff.add(s.t, s.flow, s.flow_sim, dp_meas, s.dp_sim)

            self.var_rpm.set(f"RPM: {s.rpm:,.0f}")
            self.var_flow.set(f"Flow: {s.flow:,.2f} L/min")
            self.var_dp.set(f"ΔP: {dp_meas:.3f} bar")

        if updated and self.buff.t:
            tx = self.buff.t
            self.line_flow_meas.set_data(tx, self.buff.flow_meas)
            self.line_flow_sim.set_data(tx,  self.buff.flow_sim)
            self.ax_flow.relim(); self.ax_flow.autoscale_view()
            self.canvas_flow.draw_idle()

            self.line_dp_meas.set_data(tx, self.buff.dp_meas)
            self.line_dp_sim.set_data(tx,  self.buff.dp_sim)
            self.ax_dp.relim(); self.ax_dp.autoscale_view()
            self.canvas_dp.draw_idle()

            current_flow = self.buff.flow_meas[-1]
            ref = np.percentile(self.buff.flow_meas, 95) if len(self.buff.flow_meas) >= 5 else max(1.0, current_flow)
            scale = 0.0 if ref <= 0 else max(0.0, min(1.0, current_flow / ref))
            self._draw_schematic(scale)

        self.after(int(DT * 1000), self._tick)

    def _draw_schematic(self, flow_scale: float):
        self.ax_schem.cla()
        self.ax_schem.axis("off")
        self.ax_schem.set_title("Pump & Flow Schematic")

        self.ax_schem.add_patch(Rectangle((0, 0), 10, 8, fill=False))

        pump = Rectangle((2.0, 3.2), 2.0, 1.6, linewidth=2)
        self.ax_schem.add_patch(pump)
        circ = plt.Circle((3.0, 4.0), 0.7, fill=False, linewidth=2)
        self.ax_schem.add_patch(circ)

        self.ax_schem.add_patch(Rectangle((0.6, 3.6), 1.4, 0.4))
        self.ax_schem.add_patch(Rectangle((4.0, 3.6), 5.0, 0.4))

        arrow_w = 0.2 + 1.0 * flow_scale
        arrow = FancyArrow(4.3, 3.8, 4.2, 0.0, width=arrow_w, length_includes_head=True, head_width=1.2)
        self.ax_schem.add_patch(arrow)

        self.ax_schem.text(3.0, 2.4, "Pump", ha="center", va="center")
        self.ax_schem.text(1.0, 4.6, "Inlet", ha="center", va="center")
        self.ax_schem.text(8.8, 4.6, "Outlet", ha="center", va="center")

        self.ax_schem.set_xlim(0, 10)
        self.ax_schem.set_ylim(0, 8)
        self.canvas_schem.draw_idle()


def main():
    app = Dashboard()
    app.mainloop()


if __name__ == "__main__":
    main()
