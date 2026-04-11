# app.py
# Digital Twin Pump — Real-time Dashboard (Flow & ΔP: measured vs simulated)
# Runs in VS Code. Start/Stop buttons, rolling plots, engineering theme.
#
# Requirements:
#   pip install dash dash-bootstrap-components fmpy numpy
#   (optional for hardware) pip install nidaqmx
#
# Notes:
# - Uses FMU Co-Simulation if available, otherwise falls back to re-simulating
#   with accumulated inputs (Model Exchange compatible).
# - If NI-DAQ isn't available, it auto-switches to DEV_MODE dummy signals.
# - Set FMU path via env var FMU_FILE or edit FMU_PATH below.

import os
import math
import base64
import time
import numpy as np

from fmpy import read_model_description, extract, simulate_fmu
from fmpy.fmi2 import FMU2Slave

import dash
from dash import html, dcc, callback, Output, Input, State, no_update
import dash_bootstrap_components as dbc

# ----------------------------- Configuration -----------------------------
FMU_PATH = os.getenv("FMU_FILE", "/mnt/data/Lemuel_pump_system.fmu")

# FMU variable names (adapt if needed)
FMU_INPUT_FLOW_TARGET = "amesim_interface.flowrate_target"
FMU_INPUT_PUMP_SPEED  = "amesim_interface.pump_speed"
FMU_OUT_FLOW_RATE     = "amesim_interface.flow_rate"
FMU_OUT_DP            = "amesim_interface.pressure_increase"

DT = 0.10           # seconds between updates (~10 Hz)
ROLLING_WINDOW = 60 # seconds shown in plots
MAX_POINTS = int(3 * 60 / DT)  # cap ~3 minutes

# Images (optional). Put your images in the same folder or adjust paths.
IMG_PATHS = [
    "/mnt/data/pumpsystem.jpg",     # your lab / pump photo
    "/mnt/data/DG.png"              # architecture/diagram
]

# ----------------------------- Hardware (optional) -----------------------
DEV_MODE = False
nidaq_ok = False
try:
    import nidaqmx
    from nidaqmx.constants import TerminalConfiguration, VoltageUnits, WAIT_INFINITELY
    nidaq_ok = True
except Exception:
    DEV_MODE = True  # fallback

# Calibrations (same as your scripts)
def calib_rpm(v):   return 500.0 * (v - 0.06)
def calib_pin(v):   return 0.417 * (v + 1.02)      # bar
def calib_pout(v):  return 0.165 * v               # bar
def calib_flow(v):  return 15.156 * (v - 0.16)     # L/min

def read_sample_from_ni():
    with nidaqmx.Task() as task:
        task.ai_channels.add_ai_voltage_chan("Dev1/ai2", terminal_config=TerminalConfiguration.DEFAULT,
                                             min_val=-10.0, max_val=10.0, units=VoltageUnits.VOLTS)
        task.ai_channels.add_ai_voltage_chan("Dev1/ai4", terminal_config=TerminalConfiguration.DEFAULT,
                                             min_val=-10.0, max_val=10.0, units=VoltageUnits.VOLTS)
        task.ai_channels.add_ai_voltage_chan("Dev1/ai5", terminal_config=TerminalConfiguration.DEFAULT,
                                             min_val=-10.0, max_val=10.0, units=VoltageUnits.VOLTS)
        task.ai_channels.add_ai_voltage_chan("Dev1/ai0", terminal_config=TerminalConfiguration.DEFAULT,
                                             min_val=-10.0, max_val=10.0, units=VoltageUnits.VOLTS)
        v_rpm, v_pin, v_pout, v_flow = map(float, task.read(timeout=WAIT_INFINITELY))
    return calib_rpm(v_rpm), calib_pin(v_pin), calib_pout(v_pout), calib_flow(v_flow)

def read_sample_dummy(t):
    # Reasonable synthetic dynamics for demos
    rpm = 1500 + 400 * math.sin(2 * math.pi * 0.03 * t) + 50 * math.sin(2 * math.pi * 0.6 * t)
    valve = 0.7 + 0.3 * (0.5 + 0.5 * math.sin(2 * math.pi * 0.01 * t))
    flow = max(0.0, 0.02 * rpm * valve + 0.5 * math.sin(2 * math.pi * 0.4 * t))
    pin  = 0.9 + 0.05 * math.sin(2 * math.pi * 0.07 * t)
    dp   = 0.2 + 0.015 * flow + 0.02 * math.sin(2 * math.pi * 0.2 * t)
    pout = pin + dp
    return rpm, pin, pout, flow

# ----------------------------- FMU runner -----------------------------
class FMURunner:
    def __init__(self, fmu_path, dt):
        self.dt = dt
        self.fmu_path = fmu_path
        self.model_desc = read_model_description(fmu_path)
        self.unzipdir = extract(fmu_path)
        self.mode = "cs" if self.model_desc.coSimulation is not None else "me"
        self.t = 0.0

        self.vr = {v.name: v.valueReference for v in self.model_desc.modelVariables}

        self.fmu = None
        if self.mode == "cs":
            self.fmu = FMU2Slave(guid=self.model_desc.guid,
                                 unzipDirectory=self.unzipdir,
                                 modelIdentifier=self.model_desc.coSimulation.modelIdentifier,
                                 instanceName="instance1")
            self.fmu.instantiate()
            self.fmu.setupExperiment(startTime=0.0)
            self.fmu.enterInitializationMode()
            self.fmu.exitInitializationMode()

        self.history = {k: [] for k in ["t","flow_meas","dp_meas","flow_sim","dp_sim","rpm_meas"]}

    def _set_input_cs(self, name, value):
        if name in self.vr:
            self.fmu.setReal([self.vr[name]], [float(value)])

    def _get_real_cs(self, name):
        return float(self.fmu.getReal([self.vr[name]])[0]) if name in self.vr else float("nan")

    def step(self, rpm, flow_target, pin, pout):
        dp_meas = pout - pin

        if self.mode == "cs":
            self._set_input_cs(FMU_INPUT_PUMP_SPEED, rpm)
            self._set_input_cs(FMU_INPUT_FLOW_TARGET, flow_target)
            self.fmu.doStep(currentCommunicationPoint=self.t, communicationStepSize=self.dt)
            self.t += self.dt
            flow_sim = self._get_real_cs(FMU_OUT_FLOW_RATE)
            dp_sim   = self._get_real_cs(FMU_OUT_DP)
        else:
            # Accumulate & re-simulate
            t_vec = np.array(self.history["t"] + [self.t + self.dt], dtype=float)
            if len(t_vec) == 1:
                t_vec[0] = self.dt
            rpm_series  = np.array(self.history["rpm_meas"]  + [rpm], dtype=float)
            flow_series = np.array(self.history["flow_meas"] + [flow_target], dtype=float)

            inputs = np.zeros(t_vec.shape, dtype=[('time', 'f8'),
                                                  (FMU_INPUT_FLOW_TARGET, 'f8'),
                                                  (FMU_INPUT_PUMP_SPEED,  'f8')])
            inputs['time'] = t_vec
            inputs[FMU_INPUT_FLOW_TARGET] = flow_series
            inputs[FMU_INPUT_PUMP_SPEED]  = rpm_series

            res = simulate_fmu(self.fmu_path, start_time=0.0, stop_time=float(inputs['time'][-1]), input=inputs)
            flow_sim = res[FMU_OUT_FLOW_RATE][-1] if FMU_OUT_FLOW_RATE in res.dtype.names else float("nan")
            dp_sim   = res[FMU_OUT_DP][-1]         if FMU_OUT_DP in res.dtype.names         else float("nan")
            self.t = float(inputs['time'][-1])

        # Log & trim
        for k, v in [
            ("t", self.t), ("flow_meas", flow_target), ("dp_meas", dp_meas),
            ("flow_sim", flow_sim), ("dp_sim", dp_sim), ("rpm_meas", rpm)
        ]:
            self.history[k].append(v)

        if len(self.history["t"]) > MAX_POINTS:
            for k in self.history:
                self.history[k] = self.history[k][-MAX_POINTS:]

        return flow_sim, dp_sim

    def terminate(self):
        if self.mode == "cs" and self.fmu is not None:
            try: self.fmu.terminate()
            except: pass
            try: self.fmu.freeInstance()
            except: pass

# Singleton-style FMU object for callback use
_fmu = None
_t0 = None

def get_fmu():
    global _fmu, _t0
    if _fmu is None:
        _fmu = FMURunner(FMU_PATH, DT)
        _t0 = time.time()
    return _fmu

# ----------------------------- Helpers -----------------------------
def b64_image(path):
    try:
        with open(path, "rb") as f:
            return "data:image/png;base64," + base64.b64encode(f.read()).decode("ascii")
    except Exception:
        return None

def make_figure(t, y_meas, y_sim, y_label):
    # Keep last 60 s
    if len(t) == 0:
        return {"data": [], "layout": {"template": "plotly_white"}}
    tmax = t[-1]
    tmin = max(0.0, tmax - ROLLING_WINDOW)
    idx = [i for i, ti in enumerate(t) if ti >= tmin]

    import plotly.graph_objs as go
    fig = go.Figure()
    fig.add_trace(go.Scatter(x=[t[i] for i in idx], y=[y_meas[i] for i in idx],
                             mode="lines", name=f"Measured {y_label}", line=dict(dash="dash")))
    fig.add_trace(go.Scatter(x=[t[i] for i in idx], y=[y_sim[i] for i in idx],
                             mode="lines", name=f"FMU {y_label}"))
    fig.update_layout(
        template="plotly_white",
        margin=dict(l=40, r=20, t=10, b=30),
        xaxis_title="Time (s)",
        yaxis_title=y_label,
        legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1.0),
        paper_bgcolor="#0b132b", plot_bgcolor="#0b132b",
        font=dict(color="#e0e6f1"),
        xaxis=dict(gridcolor="#2a3456"), yaxis=dict(gridcolor="#2a3456"),
    )
    return fig

# ----------------------------- Dash UI -----------------------------
app = dash.Dash(__name__, external_stylesheets=[dbc.themes.CYBORG])
app.title = "Digital Twin Pump — Dashboard"

topbar = dbc.Navbar(
    dbc.Container([
        html.Div("Digital Twin Pump System", className="navbar-brand fw-bold"),
        html.Div("Real-time vs Simulated (FMU)", className="text-muted"),
    ]),
    color="dark", dark=True, className="mb-3", style={"borderBottom": "2px solid #1b263b"}
)

controls = dbc.Card(
    dbc.CardBody([
        html.H5("Controls", className="card-title"),
        dbc.ButtonGroup([
            dbc.Button("Start", id="btn-start", color="success"),
            dbc.Button("Stop", id="btn-stop", color="danger", className="ms-2"),
        ]),
        html.Hr(),
        html.Small([
            "FMU: ", html.Code(os.path.basename(FMU_PATH)),
            " | Mode: ", html.Span(id="mode-badge", className="ms-1 badge rounded-pill text-bg-secondary"),
            html.Br(), "Input: NI-DAQ" if (nidaq_ok and not DEV_MODE) else "Input: Dummy (DEV_MODE)"
        ]),
        html.Div(id="hidden-init", style={"display":"none"})  # kicks FMU init once
    ]),
    className="shadow-sm"
)

gauges = dbc.Card(
    dbc.CardBody([
        html.H5("Live Values", className="card-title"),
        dbc.Row([
            dbc.Col(html.Div([html.Div("Flow (L/min)", className="text-muted small"),
                              html.H3(id="val-flow", className="fw-bold")]), md=4),
            dbc.Col(html.Div([html.Div("ΔP (bar)", className="text-muted small"),
                              html.H3(id="val-dp", className="fw-bold")]), md=4),
            dbc.Col(html.Div([html.Div("RPM", className="text-muted small"),
                              html.H3(id="val-rpm", className="fw-bold")]), md=4),
        ])
    ]),
    className="shadow-sm"
)

graphs = dbc.Card(
    dbc.CardBody([
        html.H5("Signals", className="card-title"),
        dcc.Graph(id="graph-flow", config={"displayModeBar": False}, style={"height":"38vh"}),
        dcc.Graph(id="graph-dp",   config={"displayModeBar": False}, style={"height":"38vh"}),
    ]),
    className="shadow-sm"
)

images = []
for p in IMG_PATHS:
    uri = b64_image(p)
    if uri:
        images.append(html.Img(src=uri, style={"width":"100%", "borderRadius":"12px", "marginBottom":"12px"}))

right_col = dbc.Card(dbc.CardBody([
    html.H5("Pump System", className="card-title"),
    *(images or [html.Div("Add images to IMG_PATHS to show your hardware/diagram.", className="text-muted")])
]), className="shadow-sm")

app.layout = html.Div([
    topbar,
    dbc.Container([
        dbc.Row([
            dbc.Col(dbc.Stack([controls, gauges, graphs], gap=3), md=8),
            dbc.Col(right_col, md=4),
        ])
    ], fluid=True),
    dcc.Interval(id="tick", interval=int(DT*1000), disabled=True),
    dcc.Store(id="store-history", data=None),
])

# ----------------------------- Callbacks -----------------------------
@callback(Output("tick", "disabled"),
          Input("btn-start", "n_clicks"),
          Input("btn-stop", "n_clicks"),
          prevent_initial_call=True)
def start_stop(n_start, n_stop):
    ctx = dash.callback_context
    if not ctx.triggered:
        return True
    trig = ctx.triggered[0]["prop_id"].split(".")[0]
    return False if trig == "btn-start" else True

@callback(
    Output("store-history", "data"),
    Output("graph-flow", "figure"),
    Output("graph-dp", "figure"),
    Output("val-flow", "children"),
    Output("val-dp", "children"),
    Output("val-rpm", "children"),
    Output("mode-badge", "children"),
    Input("tick", "n_intervals"),
    State("store-history", "data"),
    prevent_initial_call=False
)
def update_dashboard(n, hist):
    # Init FMU and history on first call
    fmu = get_fmu()
    mode_label = fmu.mode.upper()

    if hist is None:
        hist = fmu.history
    else:
        # point history to FMU's internal (keeps things in one place)
        fmu.history = hist

    # Acquire one sample (DAQ or dummy)
    t_now = time.time() - _t0
    if DEV_MODE or not nidaq_ok:
        rpm, pin, pout, flow = read_sample_dummy(t_now)
    else:
        rpm, pin, pout, flow = read_sample_from_ni()

    # Use measured flow as target input to FMU
    flow_sim, dp_sim = fmu.step(rpm=rpm, flow_target=flow, pin=pin, pout=pout)

    t   = fmu.history["t"]
    f_m = fmu.history["flow_meas"]
    f_s = fmu.history["flow_sim"]
    d_m = fmu.history["dp_meas"]
    d_s = fmu.history["dp_sim"]

    fig_flow = make_figure(t, f_m, f_s, "Flow (L/min)")
    fig_dp   = make_figure(t, d_m, d_s, "ΔP (bar)")

    last_flow = f"{(f_m[-1] if f_m else 0):.2f}"
    last_dp   = f"{(d_m[-1] if d_m else 0):.3f}"
    last_rpm  = f"{rpm:.0f}"

    return fmu.history, fig_flow, fig_dp, last_flow, last_dp, last_rpm, mode_label

# Force FMU initialization once so mode badge shows up before first tick
@callback(Output("hidden-init", "children"), Input("hidden-init", "id"))
def _init_once(_):
    _ = get_fmu()
    return ""

# ----------------------------- Run -----------------------------
if __name__ == "__main__":
    print(f"Launching dashboard — FMU: {FMU_PATH}")
    app.run_server(debug=False, host="127.0.0.1", port=8050)
