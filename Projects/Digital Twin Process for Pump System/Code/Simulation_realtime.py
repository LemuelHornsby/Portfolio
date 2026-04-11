#!/usr/bin/env python3
"""
Digital Twin Pump System — Real-time acquisition + FMU simulation + live plots

Requirements:
  pip install numpy pandas matplotlib fmpy nidaqmx

Notes:
- If the FMU supports Co-Simulation, we run step-by-step with FMU2Slave (preferred).
- Otherwise, we re-run simulate_fmu() on the accumulated inputs (works with Model Exchange FMUs too).
- If nidaqmx isn't available or no hardware is connected, set DEV_MODE = True to run with dummy signals.
"""

import os
import sys
import time
import math
from dataclasses import dataclass

import numpy as np
import matplotlib.pyplot as plt

# ---------- User configuration ----------
FMU_PATH = os.getenv("FMU_FILE", "Lemuel_pump_system.fmu")

# NI-DAQ channels (as in your DAQ code)
DAQ_CHANNELS = {
    "rpm":  "Dev1/ai2",
    "pin":  "Dev1/ai4",
    "pout": "Dev1/ai5",
    "flow": "Dev1/ai0",
}

# Calibrations (copied from your DAQ script)
def calib_rpm(v):   return 500.0 * (v - 0.06)
def calib_pin(v):   return 0.417 * (v + 1.02)      # bar
def calib_pout(v):  return 0.165 * v               # bar
def calib_flow(v):  return 15.156 * (v - 0.16)     # L/min

# FMU variable names (edit if your FMU uses slightly different names)
FMU_INPUT_FLOW_TARGET = "amesim_interface.flowrate_target"
FMU_INPUT_PUMP_SPEED  = "amesim_interface.pump_speed"
FMU_OUT_FLOW_RATE     = "amesim_interface.flow_rate"
FMU_OUT_DP            = "amesim_interface.pressure_increase"

# Timing
DT = 0.10             # seconds between samples (10 Hz)
PLOT_WINDOW_S = 60.0  # show last N seconds in the plot window (rolling)
MAX_POINTS = 1800     # safety cap for buffers (~3 minutes at 10 Hz)

# Development fallback if no NI-DAQ
DEV_MODE = False  # set True to run without hardware (generates dummy signals)
# ----------------------------------------


# Try NI-DAQ import unless DEV_MODE
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
from fmpy import read_model_description, extract, simulate_fmu
from fmpy.fmi2 import FMU2Slave


@dataclass
class Sample:
    t: float
    rpm: float
    pin: float
    pout: float
    flow: float

def read_sample_from_ni():
    """Read one sample from NI-DAQ and apply calibrations."""
    with nidaqmx.Task() as task:
        # Add channels in a fixed order: rpm, pin, pout, flow
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

        vals = task.read(timeout=WAIT_INFINITELY)  # list in same order
        v_rpm, v_pin, v_pout, v_flow = map(float, vals)

    rpm   = calib_rpm(v_rpm)
    pin   = calib_pin(v_pin)
    pout  = calib_pout(v_pout)
    flow  = calib_flow(v_flow)
    return rpm, pin, pout, flow

def read_sample_dummy(t):
    """Generate plausible dummy signals if hardware is unavailable."""
    # Simulated RPM ramp + ripple
    rpm = 1500 + 400 * math.sin(2 * math.pi * 0.03 * t) + 50 * math.sin(2 * math.pi * 0.6 * t)
    # Flow roughly proportional to RPM with valve effect
    valve = 0.7 + 0.3 * (0.5 + 0.5 * math.sin(2 * math.pi * 0.01 * t))
    flow = max(0.0, 0.02 * rpm * valve + 0.5 * math.sin(2 * math.pi * 0.4 * t))
    # Pressures: pin baseline, pout depends on flow; add noise
    pin  = 0.9 + 0.05 * math.sin(2 * math.pi * 0.07 * t)
    dp   = 0.2 + 0.015 * flow + 0.02 * math.sin(2 * math.pi * 0.2 * t)
    pout = pin + dp
    return rpm, pin, pout, flow


class FMURunner:
    """Handles both Co-Sim (FMU2Slave) and fallback simulate_fmu() modes."""
    def __init__(self, fmu_path: str, dt: float):
        self.fmu_path = fmu_path
        self.dt = dt
        self.mode = "unknown"
        self.model_desc = read_model_description(fmu_path)
        self.unzipdir = extract(fmu_path)
        self.fmu = None
        self.t = 0.0

        # Pick mode
        if self.model_desc.coSimulation is not None:
            self.mode = "cs"  # Co-Simulation
        else:
            self.mode = "me"  # Model Exchange fallback via simulate_fmu

        if self.mode == "cs":
            self.fmu = FMU2Slave(guid=self.model_desc.guid,
                                 unzipDirectory=self.unzipdir,
                                 modelIdentifier=self.model_desc.coSimulation.modelIdentifier,
                                 instanceName="instance1")
            self.fmu.instantiate()
            self.fmu.setupExperiment(startTime=0.0)
            self.fmu.enterInitializationMode()
            self.fmu.exitInitializationMode()

        # Cache value references for speed (optional)
        self.vr = {}
        for v in self.model_desc.modelVariables:
            self.vr[v.name] = v.valueReference

        self.history = {
            "t": [],
            "flow_meas": [],
            "dp_meas": [],
            "flow_sim": [],
            "dp_sim": [],
            "rpm_meas": []
        }

    def _set_input_cs(self, name, value):
        # All inputs here are Real
        self.fmu.setReal([self.vr[name]], [float(value)])

    def _get_output_cs(self, name):
        return float(self.fmu.getReal([self.vr[name]])[0])

    def step(self, rpm, flow_target, pin, pout):
        """Advance simulation by one dt using the latest measurements as inputs."""
        dp_meas = pout - pin

        if self.mode == "cs":
            # Set inputs at current time
            if FMU_INPUT_PUMP_SPEED in self.vr:
                self._set_input_cs(FMU_INPUT_PUMP_SPEED, rpm)
            if FMU_INPUT_FLOW_TARGET in self.vr:
                self._set_input_cs(FMU_INPUT_FLOW_TARGET, flow_target)

            # Do one step
            self.fmu.doStep(currentCommunicationPoint=self.t, communicationStepSize=self.dt)
            self.t += self.dt

            # Read outputs
            flow_sim = self._get_output_cs(FMU_OUT_FLOW_RATE) if FMU_OUT_FLOW_RATE in self.vr else np.nan
            dp_sim   = self._get_output_cs(FMU_OUT_DP)         if FMU_OUT_DP in self.vr         else np.nan

        else:
            # Fallback: re-simulate from 0..t with all accumulated inputs
            # Build inputs from history + this new point
            t_vec = np.array(self.history["t"] + [self.t], dtype=float)
            if len(t_vec) == 1 or t_vec[-1] <= t_vec[-2]:
                # ensure strictly increasing time
                if len(t_vec) == 1:
                    t_vec[0] = 0.0
                else:
                    t_vec[-1] = t_vec[-2] + self.dt

            rpm_series  = np.array(self.history["rpm_meas"] + [rpm], dtype=float)
            flow_series = np.array(self.history["flow_meas"] + [flow_target], dtype=float)  # using measured flow as target

            inputs = np.zeros(t_vec.shape, dtype=[('time', 'f8'),
                                                  (FMU_INPUT_FLOW_TARGET, 'f8'),
                                                  (FMU_INPUT_PUMP_SPEED,  'f8')])
            inputs['time'] = t_vec - t_vec[0]
            inputs[FMU_INPUT_FLOW_TARGET] = flow_series
            inputs[FMU_INPUT_PUMP_SPEED]  = rpm_series

            res = simulate_fmu(self.fmu_path, start_time=0.0, stop_time=float(inputs['time'][-1]),
                               input=inputs)

            # Extract last simulated outputs
            flow_sim = res[FMU_OUT_FLOW_RATE][-1] if FMU_OUT_FLOW_RATE in res.dtype.names else np.nan
            dp_sim   = res[FMU_OUT_DP][-1]         if FMU_OUT_DP in res.dtype.names         else np.nan

            # Advance time
            self.t = float(inputs['time'][-1])

        # Save to history
        self.history["t"].append(self.t)
        self.history["flow_meas"].append(flow_target)
        self.history["rpm_meas"].append(rpm)
        self.history["dp_meas"].append(dp_meas)
        self.history["flow_sim"].append(flow_sim)
        self.history["dp_sim"].append(dp_sim)

        # Trim history to window
        self._trim_history()

        return flow_sim, dp_sim

    def _trim_history(self):
        if len(self.history["t"]) <= MAX_POINTS:
            return
        # keep last MAX_POINTS
        for k in self.history:
            self.history[k] = self.history[k][-MAX_POINTS:]

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


def main():
    print("Initializing FMU…")
    fmu_runner = FMURunner(FMU_PATH, DT)
    print(f"FMU mode: {fmu_runner.mode.upper()} (path: {FMU_PATH})")

    print("Starting acquisition + live plotting… (Ctrl+C to stop)")
    plt.ion()
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    fig.suptitle("Digital Twin — Real-time vs Simulated")

    # Lines
    flow_meas_line, = ax1.plot([], [], label="Measured Flow (L/min)", linestyle="--")
    flow_sim_line,  = ax1.plot([], [], label="FMU Flow Rate")
    ax1.set_ylabel("Flow (L/min)")
    ax1.grid(True)
    ax1.legend()

    dp_meas_line, = ax2.plot([], [], label="Measured ΔP (bar)", linestyle="--")
    dp_sim_line,  = ax2.plot([], [], label="FMU Pressure Increase")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("ΔP (bar)")
    ax2.grid(True)
    ax2.legend()

    t0 = time.time()
    try:
        while True:
            loop_start = time.time()
            t = loop_start - t0

            # Read one sample
            if DEV_MODE or not nidaq_ok:
                rpm, pin, pout, flow = read_sample_dummy(t)
            else:
                rpm, pin, pout, flow = read_sample_from_ni()

            # Use measured flow as "target" to drive FMU (adjust if your FMU expects valve position or a setpoint)
            flow_sim, dp_sim = fmu_runner.step(rpm=rpm, flow_target=flow, pin=pin, pout=pout)

            # Update plots with rolling window
            tw = np.array(fmu_runner.history["t"], dtype=float)
            if len(tw) > 0:
                tmax = tw[-1]
                tmin = max(0.0, tmax - PLOT_WINDOW_S)
                idx  = tw >= tmin

                ax1.set_xlim(max(0.0, tmin), tmax + 0.01)

                flow_meas_line.set_data(tw[idx], np.array(fmu_runner.history["flow_meas"])[idx])
                flow_sim_line.set_data(tw[idx],  np.array(fmu_runner.history["flow_sim"])[idx])

                dp_meas_line.set_data(tw[idx],   np.array(fmu_runner.history["dp_meas"])[idx])
                dp_sim_line.set_data(tw[idx],    np.array(fmu_runner.history["dp_sim"])[idx])

                # Auto-scale y each time (lightweight)
                for ax in (ax1, ax2):
                    ax.relim()
                    ax.autoscale_view()

            plt.pause(0.001)  # let matplotlib process GUI events

            # Pace loop to DT
            elapsed = time.time() - loop_start
            to_sleep = DT - elapsed
            if to_sleep > 0:
                time.sleep(to_sleep)

    except KeyboardInterrupt:
        print("\nStopping…")
    finally:
        fmu_runner.terminate()
        plt.ioff()
        plt.show()


if __name__ == "__main__":
    main()
