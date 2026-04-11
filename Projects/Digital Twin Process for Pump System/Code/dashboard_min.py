# dashboard_min.py
# Digital Twin Pump Dashboard — minimal, no dash_bootstrap_components needed.

import os, time, math
from collections import deque
import numpy as np
import dash
from dash import dcc, html, Output, Input
import plotly.graph_objs as go

# ---- Config ----
FMU_PATH = os.getenv("FMU_FILE", "/mnt/data/Lemuel_pump_system.fmu")
DT = 0.10           # s between updates
ROLLING_SEC = 60.0  # window
MAX_POINTS = int(3 * 60 / DT)  # ~3 minutes buffer

# FMU variable names (edit to match your FMU)
FMU_INPUT_FLOW_TARGET = "amesim_interface.flowrate_target"
FMU_INPUT_PUMP_SPEED  = "amesim_interface.pump_speed"
FMU_OUT_FLOW_RATE     = "amesim_interface.flow_rate"
FMU_OUT_DP            = "amesim_interface.pressure_increase"

# ---- Try FMPy (graceful fallback if not installed) ----
FMPY_OK = True
try:
    from fmpy import read_model_description, extract, simulate_fmu
    from fmpy.fmi2 import FMU2Slave
except Exception:
    FMPY_OK = False

# ---- Dummy acquisition (runs everywhere) ----
def read_sample_dummy(t):
    rpm = 1500 + 350 * math.sin(2*math.pi*0.03*t) + 40 * math.sin(2*math.pi*0.6*t)
    valve = 0.7 + 0.25 * (0.5 + 0.5 * math.sin(2 * math.pi * 0.01 * t))
    flow = max(0.0, 0.02 * rpm * valve + 0.4 * math.sin(2 * math.pi * 0.4 * t))
    pin  = 0.9 + 0.05 * math.sin(2 * math.pi * 0.07 * t)
    dp   = 0.22 + 0.015 * flow + 0.02 * math.sin(2 * math.pi * 0.2 * t)
    pout = pin + dp
    return rpm, pin, pout, flow

# ---- FMU runner (CS if available; simple ME fallback; else disabled) ----
class FMURunner:
    def __init__(self, path, dt):
        self.dt = float(dt)
        self.t = 0.0
        self.available = FMPY_OK and os.path.exists(path)
        self.mode = "none"
        self.fmu = None
        self.vr = {}
        self.path = path
        self.t_hist, self.flow_hist, self.rpm_hist = [], [], []

        if not self.available:
            return
        try:
            md = read_model_description(path)
            unzip = extract(path)
            self.vr = {v.name: v.valueReference for v in md.modelVariables}
            if md.coSimulation is not None:
                self.mode = "cs"
                self.fmu = FMU2Slave(
                    guid=md.guid, unzipDirectory=unzip,
                    modelIdentifier=md.coSimulation.modelIdentifier, instanceName="inst1"
                )
                self.fmu.instantiate()
                self.fmu.setupExperiment(startTime=0.0)
                self.fmu.enterInitializationMode()
                self._set(FMU_INPUT_FLOW_TARGET, 0.0)
                self._set(FMU_INPUT_PUMP_SPEED,  0.0)
                self.fmu.exitInitializationMode()
            else:
                self.mode = "me"
        except Exception:
            self.available = False
            self.mode = "none"
            self.fmu = None

    def _set(self, name, val):
        if self.fmu and name in self.vr:
            try: self.fmu.setReal([self.vr[name]], [float(val)])
            except: pass

    def _get(self, name):
        if not self.fmu or name not in self.vr: return float("nan")
        try: return float(self.fmu.getReal([self.vr[name]])[0])
        except: return float("nan")

    def step(self, rpm, flow_target):
        if not self.available:
            # FMU disabled: echo measured with simple mapping
            return float(flow_target), 0.22 + 0.015 * float(flow_target)

        if self.mode == "cs":
            self._set(FMU_INPUT_PUMP_SPEED, rpm)
            self._set(FMU_INPUT_FLOW_TARGET, flow_target)
            try:
                self.fmu.doStep(currentCommunicationPoint=self.t, communicationStepSize=self.dt)
                self.t += self.dt
                return self._get(FMU_OUT_FLOW_RATE), self._get(FMU_OUT_DP)
            except Exception:
                return float("nan"), float("nan")

        # ME fallback: re-simulate to current time with accumulated inputs
        try:
            self.t += self.dt
            self.t_hist.append(self.t)
            self.flow_hist.append(float(flow_target))
            self.rpm_hist.append(float(rpm))
            arr = np.zeros(len(self.t_hist), dtype=[('time','f8'),
                                                    (FMU_INPUT_FLOW_TARGET,'f8'),
                                                    (FMU_INPUT_PUMP_SPEED,'f8')])
            arr['time'] = np.array(self.t_hist)
            arr[FMU_INPUT_FLOW_TARGET] = np.array(self.flow_hist)
            arr[FMU_INPUT_PUMP_SPEED]  = np.array(self.rpm_hist)
            res = simulate_fmu(self.path, start_time=0.0, stop_time=self.t, input=arr)
            fr = res[FMU_OUT_FLOW_RATE][-1] if FMU_OUT_FLOW_RATE in res.dtype.names else float("nan")
            dp = res[FMU_OUT_DP][-1]         if FMU_OUT_DP in res.dtype.names         else float("nan")
            return float(fr), float(dp)
        except Exception:
            return float("nan"), float("nan")

# ---- History buffer ----
class History:
    def __init__(self, nmax):
        self.t = deque(maxlen=nmax)
        self.flow_m = deque(maxlen=nmax)
        self.flow_s = deque(maxlen=nmax)
        self.dp_m   = deque(maxlen=nmax)
        self.dp_s   = deque(maxlen=nmax)
        self.rpm    = deque(maxlen=nmax)
    def push(self, t, flow_m, flow_s, dp_m, dp_s, rpm):
        self.t.append(t); self.flow_m.append(flow_m); self.flow_s.append(flow_s)
