import argparse
import json
import math
import socket
import time
from typing import Dict, Tuple

from mmg_setup_yacht_bowthruster import make_yacht_params, clamp, wrap_pi
from simulate_yacht_mmg_bowthruster import step_layer3_mmg


def load_marina_and_entities(
    scenario_json: str,
    marina_name: str,
    obstacle_id: int,
    goal_id: int,
) -> Tuple[Dict, Dict]:
    with open(scenario_json, "r", encoding="utf-8") as f:
        data = json.load(f)

    marina = next((m for m in data["marinas"] if m.get("name") == marina_name), None)
    if marina is None:
        raise ValueError(f"Marina '{marina_name}' not found in {scenario_json}")

    obstacle = next((o for o in marina.get("static_obstacles", []) if int(o.get("id", -1)) == obstacle_id), None)
    if obstacle is None:
        raise ValueError(f"Obstacle id {obstacle_id} not found in marina {marina_name}")

    goal = next((g for g in marina.get("dock_targets", []) if int(g.get("id", -1)) == goal_id), None)
    if goal is None:
        raise ValueError(f"Dock goal id {goal_id} not found in marina {marina_name}")

    return obstacle, goal


def unit(dx: float, dy: float) -> Tuple[float, float]:
    n = math.hypot(dx, dy)
    if n < 1e-9:
        return 1.0, 0.0
    return dx / n, dy / n


def normalize_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def project_along_cross(px: float, py: float, ox: float, oy: float, tx: float, ty: float) -> Tuple[float, float]:
    ex = px - ox
    ey = py - oy
    along = ex * tx + ey * ty
    cross = ex * (-ty) + ey * tx
    return along, cross


def run_demo(
    scenario_json: str,
    marina_name: str,
    obstacle_id: int,
    goal_id: int,
    host: str,
    port: int,
    dt: float,
    pass_side: str,
):
    obstacle, goal = load_marina_and_entities(scenario_json, marina_name, obstacle_id, goal_id)

    start_ref_x, start_ref_y = 0.0, 100.0
    obs_x, obs_y = float(obstacle["x"]), float(obstacle["y"])
    goal_x, goal_y = float(goal["x"]), float(goal["y"])

    tx, ty = unit(goal_x - start_ref_x, goal_y - start_ref_y)
    lateral_sign = 1.0 if pass_side.lower() == "left" else -1.0

    safety_allowance_m = 14.0
    obstacle_radius_m = float(obstacle.get("r", 12.0))
    keepout_radius_m = obstacle_radius_m + 25.0 + safety_allowance_m
    obs_yaw = float(obstacle.get("yaw", 0.0))
    obs_sx = float(obstacle.get("sx", 2.0 * obstacle_radius_m))
    obs_sy = float(obstacle.get("sy", 2.0 * obstacle_radius_m))
    ownship_long_margin = 36.0
    ownship_lat_margin = 28.0
    ellipse_a = 0.5 * obs_sx + ownship_long_margin + safety_allowance_m
    ellipse_b = 0.5 * obs_sy + ownship_lat_margin + safety_allowance_m
    hard_inner_radius_m = keepout_radius_m - 4.0
    orbit_radius_m = keepout_radius_m + 18.0
    prepare_start_dist_m = keepout_radius_m + 120.0
    avoid_start_dist_m = keepout_radius_m + 98.0
    clear_release_m = orbit_radius_m + 28.0
    behind_distance = 95.0
    spawn_x = obs_x - tx * behind_distance
    spawn_y = obs_y - ty * behind_distance
    spawn_psi = math.atan2(goal_y - spawn_y, goal_x - spawn_x)

    tune = {
        "lin_hull": 2.5,
        "nl_hull": 1.0,
        "added": 1.0,
        "thrust": 4.0,
        "rudder": 1.0,
        "yaw_damp": 1.0,
        "sway_damp": 1.6,
        "Xuu_scale": 1.0,
    }
    params = make_yacht_params(tune=tune)

    state = {
        "x": spawn_x,
        "y": spawn_y,
        "psi": spawn_psi,
        "u": 0.0,
        "v": 0.0,
        "r": 0.0,
    }

    print(f"[Python] Demo setup: marina={marina_name}, obstacle={obstacle_id}, goal={goal_id}")
    print(f"[Python] Obstacle at ({obs_x:.1f}, {obs_y:.1f}), goal at ({goal_x:.1f}, {goal_y:.1f})")
    print(f"[Python] Spawn behind obstacle at ({spawn_x:.1f}, {spawn_y:.1f}), pass_side={pass_side}")
    print(f"[Python] Safety allowance = {safety_allowance_m:.1f} m")
    print(f"[Python] Obstacle r={obstacle_radius_m:.1f} m, keepout r={keepout_radius_m:.1f} m")
    print(f"[Python] Ellipse barrier a={ellipse_a:.1f} m, b={ellipse_b:.1f} m, yaw={math.degrees(obs_yaw):.1f} deg")

    print(f"[Python] Waiting for Unity on {host}:{port}...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind((host, port))
    sock.listen(1)
    conn, addr = sock.accept()
    print("[Python] Unity connected:", addr)

    t = 0.0
    phase = "approach"
    prev_rudder_cmd = 0.0
    prev_bow_cmd = 0.0
    prev_heading_cmd = spawn_psi

    rudder_rate_limit_dps_approach = 1.8
    rudder_rate_limit_dps_avoid = 1.0
    rudder_rate_limit_dps_recover = 2.8
    bow_rate_limit_per_s_approach = 0.12
    bow_rate_limit_per_s_avoid = 0.18
    bow_rate_limit_per_s_recover = 0.35
    heading_rate_limit_dps_approach = 1.2
    heading_rate_limit_dps_avoid = 0.8
    heading_rate_limit_dps_recover = 2.2
    approach_slow_band_m = 35.0

    nx, ny = -ty, tx
    side_offset = orbit_radius_m + 8.0
    side_wp_x = obs_x - tx * 8.0 + lateral_sign * side_offset * nx
    side_wp_y = obs_y - ty * 8.0 + lateral_sign * side_offset * ny
    front_wp_x = obs_x + tx * 32.0 + lateral_sign * side_offset * nx
    front_wp_y = obs_y + ty * 32.0 + lateral_sign * side_offset * ny

    while True:
        dxg = goal_x - state["x"]
        dyg = goal_y - state["y"]
        dist_goal = math.hypot(dxg, dyg)

        along, cross = project_along_cross(state["x"], state["y"], obs_x, obs_y, tx, ty)
        dist_obs = math.hypot(state["x"] - obs_x, state["y"] - obs_y)
        d_side = math.hypot(state["x"] - side_wp_x, state["y"] - side_wp_y)
        d_front = math.hypot(state["x"] - front_wp_x, state["y"] - front_wp_y)
        angle_obs = math.atan2(state["y"] - obs_y, state["x"] - obs_x)
        heading_to_goal = math.atan2(dyg, dxg)

        if lateral_sign > 0.0:
            orbit_target_angle = angle_obs + math.radians(34.0)
        else:
            orbit_target_angle = angle_obs - math.radians(34.0)
        orbit_target_x = obs_x + orbit_radius_m * math.cos(orbit_target_angle)
        orbit_target_y = obs_y + orbit_radius_m * math.sin(orbit_target_angle)
        heading_to_orbit = math.atan2(orbit_target_y - state["y"], orbit_target_x - state["x"])

        # Ellipse proximity metric (q=1 on boundary, q<1 inside).
        dxo_pre = state["x"] - obs_x
        dyo_pre = state["y"] - obs_y
        c_pre = math.cos(obs_yaw)
        s_pre = math.sin(obs_yaw)
        lx_pre = c_pre * dxo_pre + s_pre * dyo_pre
        ly_pre = -s_pre * dxo_pre + c_pre * dyo_pre
        ell_q_pre = (lx_pre / max(ellipse_a, 1e-6)) ** 2 + (ly_pre / max(ellipse_b, 1e-6)) ** 2

        if phase == "approach" and (dist_obs < prepare_start_dist_m or ell_q_pre < 7.5):
            phase = "prepare"
        elif phase == "prepare" and (dist_obs < avoid_start_dist_m or ell_q_pre < 6.0):
            phase = "avoid"
        elif phase == "avoid" and d_front < 16.0:
            phase = "recover"
        elif phase == "recover" and dist_obs > clear_release_m and along > 75.0 and abs(cross) < 8.0:
            phase = "track"

        if phase == "approach":
            slow_enter_dist = keepout_radius_m + (approach_slow_band_m + 20.0)
            slow_full_dist = keepout_radius_m + approach_slow_band_m
            if dist_obs > slow_enter_dist:
                throttle_cmd = 0.54
            elif dist_obs > slow_full_dist:
                throttle_cmd = 0.42
            else:
                throttle_cmd = 0.30

            desired_approach_heading = heading_to_goal
            e_psi = wrap_pi(desired_approach_heading - state["psi"])
            rudder_cmd = math.degrees(clamp(0.55 * e_psi, -math.radians(7.0), math.radians(7.0)))
            bow_cmd = 0.0

        elif phase == "prepare":
            prepare_target_heading = math.atan2(side_wp_y - state["y"], side_wp_x - state["x"])
            e_psi = wrap_pi(prepare_target_heading - state["psi"])
            throttle_cmd = 0.22
            rudder_cmd = math.degrees(clamp(0.38 * e_psi, -math.radians(5.0), math.radians(5.0)))
            bow_cmd = 0.0

        elif phase == "avoid":
            if d_side > 12.0:
                desired_psi = math.atan2(side_wp_y - state["y"], side_wp_x - state["x"])
            else:
                desired_psi = math.atan2(front_wp_y - state["y"], front_wp_x - state["x"])
            e_psi = wrap_pi(desired_psi - state["psi"])

            radial_err = keepout_radius_m - dist_obs
            ell_proximity = max(0.0, 1.35 - ell_q_pre)
            if ell_q_pre < 1.8:
                throttle_cmd = 0.09
            elif ell_q_pre < 2.6:
                throttle_cmd = 0.12
            else:
                throttle_cmd = 0.16
            rudder_raw = 0.50 * e_psi + 0.08 * lateral_sign * ell_proximity + 0.0010 * lateral_sign * max(0.0, radial_err)
            rudder_cmd = math.degrees(clamp(rudder_raw, -math.radians(6.0), math.radians(6.0)))
            bow_base = 0.015 + 0.02 * ell_proximity + 0.0008 * max(0.0, radial_err)
            bow_cmd = clamp(lateral_sign * bow_base, -1.0, 1.0)

            if dist_obs < hard_inner_radius_m:
                escape_heading = math.atan2(state["y"] - obs_y, state["x"] - obs_x) + lateral_sign * math.radians(35.0)
                e_psi_escape = wrap_pi(escape_heading - state["psi"])
                throttle_cmd = 0.10
                rudder_cmd = math.degrees(clamp(0.60 * e_psi_escape, -math.radians(7.0), math.radians(7.0)))
                bow_cmd = clamp(lateral_sign * 0.16, -1.0, 1.0)

        elif phase == "recover":
            desired_cross = 0.0
            e_cross = desired_cross - cross
            desired_psi = heading_to_goal
            e_psi = wrap_pi(desired_psi - state["psi"])

            throttle_cmd = 0.52
            rudder_raw = 0.020 * e_cross + 0.70 * e_psi
            rudder_cmd = math.degrees(clamp(rudder_raw, -math.radians(10.0), math.radians(10.0)))
            bow_cmd = clamp(-0.012 * cross, -0.28, 0.28)

        else:
            desired_psi = math.atan2(dyg, dxg)
            e_psi = wrap_pi(desired_psi - state["psi"])

            throttle_cmd = 0.30 if dist_goal > 70.0 else 0.24
            rudder_cmd = math.degrees(clamp(0.60 * e_psi, -math.radians(9.0), math.radians(9.0)))
            bow_cmd = 0.0

        control = {
            "throttle": clamp(throttle_cmd, -1.0, 1.0),
            "rudder_angle_deg": clamp(rudder_cmd, -35.0, 35.0),
            "bow_thruster": clamp(bow_cmd, -1.0, 1.0),
        }

        # Heading-command slew limit to avoid abrupt turn-in behavior.
        if phase == "approach":
            desired_heading_cmd = desired_approach_heading
        elif phase == "prepare":
            desired_heading_cmd = math.atan2(side_wp_y - state["y"], side_wp_x - state["x"])
        elif phase == "avoid":
            desired_heading_cmd = math.atan2(side_wp_y - state["y"], side_wp_x - state["x"]) if d_side > 12.0 else math.atan2(front_wp_y - state["y"], front_wp_x - state["x"])
        else:
            desired_heading_cmd = heading_to_goal

        if phase in ("prepare", "avoid"):
            heading_rate_limit_dps = heading_rate_limit_dps_avoid
        elif phase == "recover":
            heading_rate_limit_dps = heading_rate_limit_dps_recover
        else:
            heading_rate_limit_dps = heading_rate_limit_dps_approach

        max_heading_step = math.radians(heading_rate_limit_dps) * dt
        heading_delta = wrap_pi(desired_heading_cmd - prev_heading_cmd)
        heading_delta = clamp(heading_delta, -max_heading_step, max_heading_step)
        heading_cmd = wrap_pi(prev_heading_cmd + heading_delta)
        prev_heading_cmd = heading_cmd

        heading_err_cmd = wrap_pi(heading_cmd - state["psi"])
        rudder_from_heading = math.degrees(clamp(0.40 * heading_err_cmd, -math.radians(5.0), math.radians(5.0)))
        blend = 0.35 if phase in ("prepare", "avoid") else 0.25
        control["rudder_angle_deg"] = (1.0 - blend) * control["rudder_angle_deg"] + blend * rudder_from_heading

        # Actuator rate limits for smoother, more ship-like response.
        if phase in ("prepare", "avoid"):
            rudder_rate_limit_dps = rudder_rate_limit_dps_avoid
            bow_rate_limit_per_s = bow_rate_limit_per_s_avoid
        elif phase == "recover":
            rudder_rate_limit_dps = rudder_rate_limit_dps_recover
            bow_rate_limit_per_s = bow_rate_limit_per_s_recover
        else:
            rudder_rate_limit_dps = rudder_rate_limit_dps_approach
            bow_rate_limit_per_s = bow_rate_limit_per_s_approach

        max_rudder_step = rudder_rate_limit_dps * dt
        max_bow_step = bow_rate_limit_per_s * dt

        control["rudder_angle_deg"] = clamp(
            control["rudder_angle_deg"],
            prev_rudder_cmd - max_rudder_step,
            prev_rudder_cmd + max_rudder_step,
        )
        control["bow_thruster"] = clamp(
            control["bow_thruster"],
            prev_bow_cmd - max_bow_step,
            prev_bow_cmd + max_bow_step,
        )

        prev_rudder_cmd = control["rudder_angle_deg"]
        prev_bow_cmd = control["bow_thruster"]

        state = step_layer3_mmg(state, control, params, dt)

        # Hard non-penetration barrier for pose-driven Unity updates:
        # keeps the vessel center outside an obstacle-aligned ellipse.
        dxo = state["x"] - obs_x
        dyo = state["y"] - obs_y
        c_yaw = math.cos(obs_yaw)
        s_yaw = math.sin(obs_yaw)

        # world -> obstacle local
        lx = c_yaw * dxo + s_yaw * dyo
        ly = -s_yaw * dxo + c_yaw * dyo

        ell_q = (lx / max(ellipse_a, 1e-6)) ** 2 + (ly / max(ellipse_b, 1e-6)) ** 2
        barrier_hit = ell_q < 1.0
        barrier_engaged = ell_q < 1.30
        if barrier_engaged:
            if abs(lx) < 1e-9 and abs(ly) < 1e-9:
                # pick outward point on starboard/port side consistent with bypass direction
                proj_lx = 0.0
                proj_ly = lateral_sign * ellipse_b * 1.05
            else:
                target_q = 1.10 if ell_q >= 1.0 else 1.0
                scale = math.sqrt(target_q / max(ell_q, 1e-9))
                proj_lx = lx * scale
                proj_ly = ly * scale

            # local -> world projected position
            state["x"] = obs_x + c_yaw * proj_lx - s_yaw * proj_ly
            state["y"] = obs_y + s_yaw * proj_lx + c_yaw * proj_ly

            # Local outward normal of ellipse at projected point.
            n_lx = proj_lx / max(ellipse_a * ellipse_a, 1e-9)
            n_ly = proj_ly / max(ellipse_b * ellipse_b, 1e-9)
            n_ln = math.hypot(n_lx, n_ly)
            if n_ln < 1e-9:
                n_lx, n_ly = 1.0, 0.0
            else:
                n_lx /= n_ln
                n_ly /= n_ln

            # normal to world
            nx_b = c_yaw * n_lx - s_yaw * n_ly
            ny_b = s_yaw * n_lx + c_yaw * n_ly

            # Remove inward normal velocity component to avoid re-entering next step.
            vr_in = state["u"] * nx_b + state["v"] * ny_b
            if vr_in < 0.0:
                state["u"] -= vr_in * nx_b
                state["v"] -= vr_in * ny_b
            if ell_q < 1.20:
                state["u"] += 0.05 * nx_b
                state["v"] += 0.05 * ny_b

            # Do not force yaw here; keep turning governed by controller for bow-led behavior.

        if int(t) != int(t - dt):
            print(
                f"[DBG] phase={phase:8s} t={t:6.1f}s "
                f"pos=({state['x']:7.1f},{state['y']:7.1f}) "
                f"u={state['u']:4.2f} v={state['v']:5.2f} r={state['r']:6.3f} "
                f"d_obs={dist_obs:6.1f} keepout={keepout_radius_m:5.1f} ell_q={ell_q:4.2f} hit={int(barrier_hit)} d_side={d_side:5.1f} d_front={d_front:5.1f} along={along:6.1f} cross={cross:6.1f} "
                f"thr={control['throttle']:4.2f} rud={control['rudder_angle_deg']:6.1f} bt={control['bow_thruster']:5.2f}"
            )

        msg = {
            "x": state["x"],
            "y": state["y"],
            "psi": -state["psi"],
            "u": state["u"],
            "v": state["v"],
            "r": state["r"],
            "throttle": control["throttle"],
            "rudder_angle": control["rudder_angle_deg"],
            "rudder_angle_deg": control["rudder_angle_deg"],
            "bow_thruster": control["bow_thruster"],
            "demo_phase": phase,
            "obstacle_x": obs_x,
            "obstacle_y": obs_y,
            "goal_x": goal_x,
            "goal_y": goal_y,
        }
        conn.sendall((json.dumps(msg) + "\n").encode("utf-8"))

        time.sleep(dt)
        t += dt


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Unity demo: place yacht behind obstacle and maneuver around it using throttle+rudder+bow thruster.")
    parser.add_argument("--scenario-json", default="marinas_export.json")
    parser.add_argument("--marina-name", default="MarinaRoot1")
    parser.add_argument("--obstacle-id", type=int, default=3)
    parser.add_argument("--goal-id", type=int, default=0)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5005)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--pass-side", choices=["left", "right"], default="left")
    args = parser.parse_args()

    run_demo(
        scenario_json=args.scenario_json,
        marina_name=args.marina_name,
        obstacle_id=args.obstacle_id,
        goal_id=args.goal_id,
        host=args.host,
        port=args.port,
        dt=args.dt,
        pass_side=args.pass_side,
    )
