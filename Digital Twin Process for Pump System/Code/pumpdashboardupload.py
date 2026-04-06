#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Digital Twin Pump Dashboard — Real-time vs Simulation with per-minute CSV + S3 upload

- Pure Python: Tkinter + Matplotlib (+ boto3, numpy, fmpy used by your Simulation_realtime.py)
- Uses your existing FMU runner & IO from Simulation_realtime.py if available.
- Overlays Measured vs Simulated Flow and ΔP.
- Buttons: Start, Stop, Save CSV Now, Save + Upload Now.
- Every minute: append CSV locally and upload to S3 automatically.

Run:
  python pump_dashboard.py
"""

import os
import sys
import time
import csv
import math
from pathlib import Path
from datetime import datetime

# Make sure Tkinter backend is used before importing tkagg canvas
import matplotlib
matplotlib.use("TkAgg")

import numpy as np
import tkinter as tk
from tkinter import ttk, messagebox
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
import boto3

# -----------------------------------------------------------------------------
# S3 — hardcoded credentials and smart region detection
# -----------------------------------------------------------------------------
BUCKET_NAME = "bbdgtwin"

_AWS_ACCESS_KEY_ID     = "AKIAZZZS2DNDHROGADJZ"
_AWS_SECRET_ACCESS_KEY = "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"

def _make_s3_client():
    """
    Build an S3 client; detect bucket region to avoid PermanentRedirect/301.
    """
    base = boto3.client(
        "s3",
        aws_access_key_id=_AWS_ACCESS_KEY_ID,
        aws_secret_access_key=_AWS_SECRET_ACCESS_KEY,
        region_name="us-east-1",  # works for GetBucketLocation even for non-us-east-1 buckets
    )
    try:
        loc = base.get_bucket_location(Bucket=BUCKET_NAME).get("LocationConstraint")
        # AWS returns None for us-east-1
        region = loc or "us-east-1"
    except Exception:
        # If we can't determine, fall back to us-east-1 and let boto3 handle it
        region = "us-east-1"
    return boto3.client(
        "s3",
        aws_access_key_id=_AWS_ACCESS_KEY_ID,
        aws_secret_access_key=_AWS_SECRET_ACCESS_KEY,
        region_name=region,
    )

S3 = _make_s3_client()

# -----------------------------------------------------------------------------
# Try to use your existing model/IO from Simulation_realtime.py
# -----------------------------------------------------------------------------
HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.append(str(HERE))

DEV_FALLBACK = False

try:
    from Simulation_realtime import (
        FMURunner, read_sample_dummy, read_sample_from_ni,
        DEV_MODE, DT, FMU_PATH
    )
    try:
        from Simulation_realtime import nidaq_ok  # optional
        NIDAQ_OK = bool(nidaq_ok)
    except Exception:
        NIDAQ_OK = False
except Exception as e:
    # Minimal fallback so the dashboard still runs without your file — uses synthetic signals only.
    DEV_FALLBACK = True
    DT = 0.10
    FMU_PATH = "Lemuel_pump_system.fmu"

    class FMURunner:
        def __init__(self, fmu_path: str, dt: float):
            self.dt = dt
            self.mode = "fallback"
            self.t = 0.0
            self.history = {k: [] for k in
                ("t","rpm_meas","flow_meas","dp_meas","flow_sim","dp_sim")}
        def terminate(self):
            pass
        def step(self, rpm, flow_target, pin, pout):
            # "Simulated" outputs follow measurement with a slight lag/noise
            dp_meas = pout - pin
            flow_sim = 0.9*flow_target + 0.1*math.sin(2*math.pi*0.7*self.t)
            dp_sim   = 0.9*dp_meas     + 0.05*math.sin(2*math.pi*0.5*self.t)
            self.t += self.dt
            h = self.history
            h["t"].append(self.t)
            h["rpm_meas"].append(rpm)
            h["flow_meas"].append(flow_target)
            h["dp_meas"].append(dp_meas)
            h["flow_sim"].append(flow_sim)
            h["dp_sim"].append(dp_sim)
            return flow_sim, dp_sim

    def read_sample_dummy(t):
        rpm = 1500 + 400 * math.sin(2*math.pi*0.03*t) + 50 * math.sin(2*math.pi*0.6*t)
        valve = 0.7 + 0.3 * (0.5 + 0.5 * math.sin(2*math.pi*0.01*t))
        flow = max(0.0, 0.02 * rpm * valve + 0.5 * math.sin(2*math.pi*0.4*t))
        pin = 0.9 + 0.05 * math.sin(2*math.pi*0.07*t)
        dp  = 0.2 + 0.015*flow + 0.02 * math.sin(2*math.pi*0.2*t)
        pout = pin + dp
        return rpm, pin, pout, flow

    def read_sample_from_ni():
        # fallback always uses dummy
        t = time.time() % 1000.0
        return read_sample_dummy(t)

    DEV_MODE = True
    NIDAQ_OK = False

# -----------------------------------------------------------------------------
# App config and constants
# -----------------------------------------------------------------------------
DATA_DIR = Path("pump_data"); DATA_DIR.mkdir(exist_ok=True)
CSV_HEADER = ["timestamp_iso","t_epoch","rpm","flow_meas","flow_sim","dp_meas","dp_sim"]
PLOT_WINDOW_S = 60.0  # seconds shown in rolling plot window

# -----------------------------------------------------------------------------
# Dashboard
# -----------------------------------------------------------------------------
class PumpDashboard:
    def __init__(self, root):
        self.root = root
        root.title("⚙️ Digital Twin Pump — Real-time vs Simulation")
        root.configure(bg="#0f172a")  # dark engineering theme

        style = ttk.Style()
        try: style.theme_use("clam")
        except Exception: pass
        style.configure("TFrame", background="#0f172a")
        style.configure("TLabel", background="#0f172a", foreground="#e2e8f0", font=("Segoe UI", 10))
        style.configure("Header.TLabel", font=("Segoe UI Semibold", 14))
        style.configure("KPI.TLabel", font=("Consolas", 12))
        style.configure("TButton", font=("Segoe UI", 10), padding=6)

        # Header
        top = ttk.Frame(root); top.pack(fill="x", padx=12, pady=(12,6))
        ttk.Label(top, text="Digital Twin — Real-time overlay (Flow & ΔP)", style="Header.TLabel").pack(side="left")
        fmu_label = os.path.basename(FMU_PATH) if FMU_PATH else "—"
        s3_state = f"S3: ready ({BUCKET_NAME})"
        self.status_lbl = ttk.Label(top, text=f"Idle  |  FMU: {fmu_label}  |  {s3_state}")
        self.status_lbl.pack(side="right")

        # Figure
        mid = ttk.Frame(root); mid.pack(fill="both", expand=True, padx=12, pady=6)
        fig = Figure(figsize=(10.8, 6.2), dpi=100)
        self.ax1 = fig.add_subplot(211)
        self.ax2 = fig.add_subplot(212, sharex=self.ax1)
        for ax in (self.ax1, self.ax2):
            ax.set_facecolor("#0b1220"); ax.grid(True, alpha=0.25); ax.tick_params(colors="#94a3b8")
            ax.spines[:].set_color("#334155")
        self.ax1.set_ylabel("Flow (L/min)", color="#e2e8f0")
        self.ax2.set_xlabel("Time (s)", color="#e2e8f0"); self.ax2.set_ylabel("ΔP (bar)", color="#e2e8f0")

        (self.flow_meas_line,) = self.ax1.plot([], [], linestyle="--", label="Measured Flow")
        (self.flow_sim_line,)  = self.ax1.plot([], [], label="FMU Flow Rate")
        self.ax1.legend(facecolor="#0b1220", labelcolor="#e2e8f0")
        (self.dp_meas_line,) = self.ax2.plot([], [], linestyle="--", label="Measured ΔP")
        (self.dp_sim_line,)  = self.ax2.plot([], [], label="FMU ΔP")
        self.ax2.legend(facecolor="#0b1220", labelcolor="#e2e8f0")

        canvas = FigureCanvasTkAgg(fig, master=mid)
        canvas_widget = canvas.get_tk_widget()
        canvas_widget.pack(fill="both", expand=True)
        self.canvas = canvas
        NavigationToolbar2Tk(canvas, mid)  # optional toolbar

        # Controls & KPIs
        bottom = ttk.Frame(root); bottom.pack(fill="x", padx=12, pady=(6,12))
        btns = ttk.Frame(bottom); btns.pack(side="left")
        self.start_btn = ttk.Button(btns, text="Start", command=self.start)
        self.stop_btn  = ttk.Button(btns, text="Stop", command=self.stop, state="disabled")
        self.save_btn  = ttk.Button(btns, text="Save CSV Now", command=self.save_now)
        self.upload_btn= ttk.Button(btns, text="Save + Upload Now", command=self.save_and_upload_now)
        for b in (self.start_btn, self.stop_btn, self.save_btn, self.upload_btn):
            b.pack(side="left", padx=4)

        kpis = ttk.Frame(bottom); kpis.pack(side="right")
        self.kpi_rpm  = ttk.Label(kpis, text="RPM: —", style="KPI.TLabel");  self.kpi_rpm.pack(side="left", padx=8)
        self.kpi_flow = ttk.Label(kpis, text="Flow (M/S): —/—", style="KPI.TLabel"); self.kpi_flow.pack(side="left", padx=8)
        self.kpi_dp   = ttk.Label(kpis, text="ΔP (M/S): —/—", style="KPI.TLabel"); self.kpi_dp.pack(side="left", padx=8)

        # Runtime state
        self.running = False
        self.t0 = None
        self.fmu_runner = None

        # History (from FMU runner)
        self.t_hist = []
        self.flow_meas = []; self.flow_sim = []
        self.dp_meas = [];   self.dp_sim = []

        # Minute logging
        self._active_minute_id = None
        self._minute_rows = []

        # Clean shutdown
        root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---------------- Run control ----------------
    def start(self):
        if self.running:
            return
        if self.fmu_runner is None:
            try:
                self.fmu_runner = FMURunner(FMU_PATH, DT)
            except Exception as e:
                messagebox.showerror("FMU error", f"Could not start FMU:\n{e}")
                return
        self.running = True
        self.start_btn.config(state="disabled")
        self.stop_btn.config(state="normal")
        mode = getattr(self.fmu_runner, "mode", "unknown").upper()
        self.status_lbl.config(text=f"Running  |  FMU mode: {mode}  |  S3: ready")
        self.t0 = time.time()
        self._tick()

    def stop(self):
        self.running = False
        self.start_btn.config(state="normal")
        self.stop_btn.config(state="disabled")
        self.status_lbl.config(text="Stopped")
        self._flush_minute_to_csv(upload=False)

    def on_close(self):
        try:
            self._flush_minute_to_csv(upload=True)
        except Exception:
            pass
        try:
            if self.fmu_runner:
                self.fmu_runner.terminate()
        except Exception:
            pass
        self.root.destroy()

    # ---------------- Core loop ----------------
    def _tick(self):
        if not self.running:
            return
        loop_start = time.time()
        t = loop_start - (self.t0 or loop_start)

        # Read live sample
        if (not DEV_MODE) and 'NIDAQ_OK' in globals() and NIDAQ_OK:
            try:
                rpm, pin, pout, flow = read_sample_from_ni()
            except Exception:
                rpm, pin, pout, flow = read_sample_dummy(t)
        else:
            rpm, pin, pout, flow = read_sample_dummy(t)

        # Simulate one step
        flow_sim, dp_sim = self.fmu_runner.step(rpm=rpm, flow_target=flow, pin=pin, pout=pout)
        dp_meas = (pout - pin)

        # Retrieve history from runner (it already appended this step)
        tw = np.array(self.fmu_runner.history["t"], dtype=float)
        self.flow_meas = list(self.fmu_runner.history["flow_meas"])
        self.flow_sim  = list(self.fmu_runner.history["flow_sim"])
        self.dp_meas   = list(self.fmu_runner.history["dp_meas"])
        self.dp_sim    = list(self.fmu_runner.history["dp_sim"])

        # Update plots (rolling window)
        if len(tw):
            tmax = float(tw[-1])
            tmin = max(0.0, tmax - PLOT_WINDOW_S)
            idx = tw >= tmin
            self.ax1.set_xlim(max(0.0, tmin), tmax + 0.01)
            self.flow_meas_line.set_data(tw[idx], np.array(self.flow_meas)[idx])
            self.flow_sim_line.set_data (tw[idx], np.array(self.flow_sim)[idx])
            self.dp_meas_line.set_data  (tw[idx], np.array(self.dp_meas)[idx])
            self.dp_sim_line.set_data   (tw[idx], np.array(self.dp_sim)[idx])
            for ax in (self.ax1, self.ax2):
                ax.relim(); ax.autoscale_view()
            self.canvas.draw_idle()

        # KPI labels
        self.kpi_rpm.config(text=f"RPM: {rpm:7.1f}")
        self.kpi_flow.config(text=f"Flow (M/S): {flow:6.2f} / {flow_sim:6.2f}")
        self.kpi_dp.config(text=f"ΔP (M/S): {dp_meas:6.3f} / {dp_sim:6.3f}")

        # Per-sample row & minute roll (/60)
        now_epoch = time.time()
        minute_id = int(now_epoch // 60)
        if self._active_minute_id is None:
            self._active_minute_id = minute_id
        elif minute_id != self._active_minute_id:
            self._flush_minute_to_csv(upload=True)
            self._active_minute_id = minute_id

        self._minute_rows.append([
            datetime.fromtimestamp(now_epoch).isoformat(timespec="seconds"),
            f"{now_epoch:.3f}", f"{rpm:.3f}",
            f"{flow:.5f}", f"{flow_sim:.5f}",
            f"{dp_meas:.5f}", f"{dp_sim:.5f}",
        ])

        # Pace loop to DT
        elapsed = time.time() - loop_start
        delay_ms = max(1, int((DT - elapsed) * 1000))
        self.root.after(delay_ms, self._tick)

    # ---------------- CSV / S3 helpers ----------------
    def _minute_filename(self, minute_id: int) -> Path:
        ts = datetime.fromtimestamp(minute_id * 60)
        return DATA_DIR / f"pump_{ts:%Y%m%d_%H%M}.csv"

    def _flush_minute_to_csv(self, upload: bool):
        if not self._minute_rows or self._active_minute_id is None:
            return
        fn = self._minute_filename(self._active_minute_id)
        new_file = not fn.exists()
        with fn.open("a", newline="") as f:
            w = csv.writer(f)
            if new_file:
                w.writerow(CSV_HEADER)
            w.writerows(self._minute_rows)
        self._minute_rows = []
        if upload:
            try:
                S3.upload_file(str(fn), BUCKET_NAME, fn.name)
                self.status_lbl.config(text=f"Uploaded {fn.name} → s3://{BUCKET_NAME}/{fn.name}")
            except Exception as e:
                self.status_lbl.config(text="Upload failed")
                messagebox.showwarning("S3 upload failed", f"{e}")

    def save_now(self):
        if self._active_minute_id is None:
            messagebox.showinfo("Save CSV", "No data yet.")
            return
        self._flush_minute_to_csv(upload=False)
        messagebox.showinfo("Save CSV", "Current minute saved to CSV.")

    def save_and_upload_now(self):
        if self._active_minute_id is None:
            messagebox.showinfo("Upload", "No data yet.")
            return
        # Save current minute snapshot (do not clear rows so collection continues)
        fn = self._minute_filename(self._active_minute_id)
        new_file = not fn.exists()
        with fn.open("a", newline="") as f:
            w = csv.writer(f)
            if new_file:
                w.writerow(CSV_HEADER)
            w.writerows(self._minute_rows)
        try:
            S3.upload_file(str(fn), BUCKET_NAME, fn.name)
            self.status_lbl.config(text=f"Uploaded {fn.name} → s3://{BUCKET_NAME}/{fn.name}")
            messagebox.showinfo("Upload", f"Saved and uploaded: {fn.name}")
        except Exception as e:
            self.status_lbl.config(text="Upload failed")
            messagebox.showwarning("S3 upload failed", f"{e}")

# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
if __name__ == "__main__":
    root = tk.Tk()
    app = PumpDashboard(root)
    root.mainloop()
