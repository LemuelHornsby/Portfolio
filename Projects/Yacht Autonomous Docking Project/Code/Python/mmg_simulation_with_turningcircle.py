import socket
import json
import time
import math
import csv

from mmg_yachdynamics_model_with_turning_circle import mmg_step

HOST = "127.0.0.1"
PORT = 5005

print(f"[Python] Waiting for Unity on {HOST}:{PORT}...")
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind((HOST, PORT))
sock.listen(1)
conn, addr = sock.accept()
print("[Python] Unity connected:", addr)

state = {
    "x": 0.0,
    "y": 0.0,
    "psi": 0.0,
    "u": 0.0,
    "v": 0.0,
    "r": 0.0,
}

params = {
    # Physical yacht parameters (from YachtDynamics)
    "m": 366770,
    "Iz": 3209238,
    "L": 40.0,
    "Ar": 4.0, #changed from 2.5 to 4.0 to increase maneuverability
    "xr": -17.0,

    # Added mass
    "m_added": 0.3 * 366770,
    "Iz_added": 0.05 * 366770 * 40.0**2,

    # Damping
    "Du": 24000.0,
    "Dv": 20000.0,
    "Dr": 500000.0,

    # Thrust & rudder
    "K_thrust": 60000.0,
    "K_rudder": 200.0 * 4.0,

    # Wind (north, east)
    "wind": (0.0, 0.0),  # Earth frame wind
    "K_wind_x": 300.0,
    "K_wind_y": 500.0,
    "K_wind_n": 1000.0,

    # Simple layer 1/2 params
    "U_max": 4.0,
    "tau_u": 20.0,
    "I_z": 3.2e6,
    "k_N": 1.0e7,
    "d_N": 5.0e6,
}

dt = 0.05

# 1, 2, 3, or 4 (4 = turning circle mode)
layer = 4

# Base control
throttle = 0.8      # 0..1
rudder_angle = 0.0  # degrees

# Turning circle timing state (used only for layer 4)
t = 0.0
turning_started = False

print("[Python] MMG simulation running...")

# CSV logging setup
csv_path = "mmg_turningcircle_log.csv"
fieldnames = [
    "time_s",
    "x_m",
    "y_m",
    "psi_deg",
    "u_mps",
    "v_mps",
    "r_degps",
    "speed_mps",
    "throttle",
    "rudder_deg",
]

with open(csv_path, mode="w", newline="", encoding="utf-8") as csvfile:
     writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
     writer.writeheader()

     try:
         while True:
             # --------------------------------------------------
             # Layer 4: Turning circle timing logic
             #  - 0–30 s: rudder = 0°
             #  - >30 s: rudder = 35°
             # --------------------------------------------------
             if layer == 4:
                 if t < 30.0:
                     rudder_angle = 0.0
                 else:
                     if not turning_started:
                         print("[MMG] Turning Circle Maneuver Started (rudder = 35°)")
                         turning_started = True
                     rudder_angle = 35.0  # starboard turn; use -35.0 for port

             control = {
                 "throttle": throttle,
                 "rudder_angle": rudder_angle,
             }

             state = mmg_step(state, control, params, dt, layer=layer)

             # Derived metrics for clean logging
             psi_deg = math.degrees(state["psi"])
             r_degps = math.degrees(state["r"])
             speed_mps = math.sqrt(state["u"] ** 2 + state["v"] ** 2)

             # Write row to CSV
             writer.writerow({
                 "time_s": round(t, 3),
                 "x_m": state["x"],
                 "y_m": state["y"],
                 "psi_deg": psi_deg,
                 "u_mps": state["u"],
                 "v_mps": state["v"],
                 "r_degps": r_degps,
                 "speed_mps": speed_mps,
                 "throttle": throttle,
                 "rudder_deg": rudder_angle,
             })
             csvfile.flush()

             # Debug print (optional)
             print(
                 f"t={t:.2f}s | x={state['x']:.2f} m, y={state['y']:.2f} m, "
                 f"psi={psi_deg:.2f}°, u={state['u']:.3f} m/s, v={state['v']:.3f} m/s, r={r_degps:.3f} °/s"
             )

             # Send to Unity
             msg = {
                 "x": state["x"],
                 "y": state["y"],
                 "psi": state["psi"],
                 "throttle": throttle,
                 "rudder_angle": rudder_angle,
                 "u": state["u"],
                 "v": state["v"],
                 "r": state["r"],
             }
             conn.sendall((json.dumps(msg) + "\n").encode("utf-8"))

             time.sleep(dt)
             t += dt
     finally:
         try:
             conn.close()
         except Exception:
             pass
         try:
             sock.close()
         except Exception:
             pass
         print(f"[Python] CSV log saved to {csv_path}")