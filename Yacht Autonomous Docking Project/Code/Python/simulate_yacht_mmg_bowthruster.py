# simulate_yacht_mmg.py
import socket
import json
import time
import math
from typing import Dict

from mmg_setup_yacht_bowthruster import make_yacht_params, clamp, wrap_pi


def body_to_earth(u: float, v: float, psi: float):
    # Earth frame x,y where x aligns Unity +X, y aligns Unity +Z
    x_dot = u * math.cos(psi) - v * math.sin(psi)
    y_dot = u * math.sin(psi) + v * math.cos(psi)
    return x_dot, y_dot


def step_layer3_mmg(state: Dict, control: Dict, params: Dict, dt: float) -> Dict:
    """
    3-DOF MMG-like dynamics in Unity convention:
      u:+X, v:+Z, r:+Y
    """
    x, y, psi = state["x"], state["y"], state["psi"]
    u, v, r = state["u"], state["v"], state["r"]

    throttle = clamp(control["throttle"], -1.0, 1.0)
    delta = math.radians(clamp(control["rudder_angle_deg"],
                               -params["rudder"].max_deg,
                               params["rudder"].max_deg))

    m = params["m"]
    Iz = params["Iz"]
    xG = params["xG"]

    Xu_dot = params["Xu_dot"]
    Yv_dot = params["Yv_dot"]
    Nr_dot = params["Nr_dot"]

    # Forces
    Xh, Yh, Nh = params["hull_forces"](u, v, r)
    Xp = params["prop_thrust"](u, throttle)
    XR, YR = params["rudder_forces"](u, v, r, delta, throttle)

    # ✅ CG-relative moment arm (fixes spin-in-place tendency)
    NR = (params["rudder"].xR - xG) * YR


    # --- Bow thruster (dormant unless control['bow_thruster'] is provided) ---
    # Convention: +bow_thruster => lateral force in body +Z (sway +)
    bow_cmd = float(control.get("bow_thruster", 0.0))
    if "bow_thruster_forces" in params:
        Y_bt, N_bt = params["bow_thruster_forces"](bow_cmd, xG)
    else:
        Y_bt, N_bt = 0.0, 0.0
    X = Xh + Xp + XR
    Y = Yh + YR + Y_bt
    N = Nh + NR + N_bt
    # Effective masses
    m_x = m - Xu_dot
    m_y = m - Yv_dot
    I_z = Iz - Nr_dot

    # 3DOF coupling
    u_dot = (X / m_x) + v * r
    v_dot = (Y / m_y) - u * r
    r_dot = (N / I_z)

    # Semi-implicit integration
    u += u_dot * dt
    v += v_dot * dt
    r += r_dot * dt

    psi = wrap_pi(psi + r * dt)
    x_dot, y_dot = body_to_earth(u, v, psi)
    x += x_dot * dt
    y += y_dot * dt

    return {"x": x, "y": y, "psi": psi, "u": u, "v": v, "r": r}


def run_tcp(host="127.0.0.1", port=5005, dt=0.05):
    print(f"[Python] Waiting for Unity on {host}:{port}...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind((host, port))
    sock.listen(1)
    conn, addr = sock.accept()
    print("[Python] Unity connected:", addr)

    # ✅ Better baseline tuning to ensure "rudder=0 => straight"
    tune = {
        "lin_hull": 2.5, #increse to damp surge oscillations
        "nl_hull": 1.0,      # ↑ more yaw moment for turning
        "added": 1.0,        # ↑ more added mass for stability
        "thrust": 4.0, # increase to boost acceleration
        "rudder": 1.0, # vary to adjust turning radius
        "yaw_damp": 1.0,      # ↑ damp yaw to prevent spin-out
        "sway_damp": 1.6,     # ↓ allow v to develop in turns
        "Xuu_scale": 1.0,     # optional: reduces top speed if needed
    }
    params = make_yacht_params(tune=tune)

    state = {"x": 0.0, "y": 0.0, "psi": 0.0, "u": 0.0, "v": 0.0, "r": 0.0}

    throttle = 0.8 # constant throttle

    # Toggle this for quick validation:
    # False: always rudder=0 (pure surge)
    # True: apply rudder=35 after 10s (turning)
    RUDDER_TEST_MODE = True
    t = 0.0

    print("[Python] Simulation running...")
    while True:
        if not RUDDER_TEST_MODE:
            rudder_deg = 0.0
        else:
            rudder_deg = 0.0 if t < 10.0 else 35.0

        control = {"throttle": throttle, "rudder_angle_deg": rudder_deg, "bow_thruster": 0.0} # example bow thruster command
        state = step_layer3_mmg(state, control, params, dt)


        # --- DIAGNOSTIC: drift angle + yaw sign ---
        u = state["u"]
        v = state["v"]
        r = state["r"]
        psi = state["psi"]

        # Drift angle (beta) in degrees
        beta_deg = math.degrees(math.atan2(v, max(abs(u), 1e-6)))
        psi_deg = math.degrees(psi)

        # Print once per second to avoid spam
        if int(t) != int(t - dt):
            print(
                f"[DBG] t={t:6.1f}s | "
                f"psi={psi_deg:7.2f} deg | "
                f"beta={beta_deg:7.2f} deg | "
                f"u={u:5.2f} m/s | "
                f"v={v:5.2f} m/s | "
                f"r={r:7.4f} rad/s"
            )

        # ptint state for debugging
        print(f"t={t:.2f}s -> x={state['x']:.2f}, y={state['y']:.2f}, psi={state['psi']:.2f}, u={state['u']:.2f}, v={state['v']:.2f}, r={state['r']:.2f}")

        # send to Unity
        msg = {
            "x": state["x"],
            "y": state["y"],
            "psi": -state["psi"],
            "u": state["u"],
            "v": state["v"],
            "r": state["r"],
            "throttle": throttle,
            "rudder_angle_deg": rudder_deg,
            "bow_thruster": float(control.get("bow_thruster", 0.0)),

        }
        conn.sendall((json.dumps(msg) + "\n").encode("utf-8"))

        time.sleep(dt)
        t += dt


if __name__ == "__main__":
    run_tcp()
