import argparse
import csv
import os

from stable_baselines3 import PPO

from drl_avoid_env import YachtAvoidRecoverEnv


def main():
    parser = argparse.ArgumentParser(description="Run one deterministic diagnostic rollout and log phase behavior.")
    parser.add_argument("--scenario-json", default="marinas_export.json")
    parser.add_argument("--marina-name", default="MarinaRoot1")
    parser.add_argument("--obstacle-id", type=int, default=3)
    parser.add_argument("--goal-id", type=int, default=0)
    parser.add_argument("--model-dir", required=True)
    parser.add_argument("--model-name", default="final_model.zip")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--max-steps", type=int, default=2000)
    parser.add_argument("--trajectory-csv", default=None)
    args = parser.parse_args()

    env = YachtAvoidRecoverEnv(
        scenario_json=args.scenario_json,
        marina_name=args.marina_name,
        obstacle_id=args.obstacle_id,
        goal_id=args.goal_id,
        dt=args.dt,
        max_steps=args.max_steps,
        seed=args.seed,
        curriculum_stage=3,
    )
    model = PPO.load(os.path.join(args.model_dir, args.model_name))

    obs, info = env.reset(seed=args.seed)
    prev_phase = info.get("phase", "approach")
    t = 0.0
    rows = []
    print(f"[DIAG] start phase={prev_phase}")

    terminated = False
    truncated = False
    final_info = {}
    while not terminated and not truncated:
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, step_info = env.step(action)
        phase = step_info.get("phase", prev_phase)
        if phase != prev_phase:
            print(f"[DIAG] t={t:7.2f}s phase {prev_phase} -> {phase}")
            prev_phase = phase

        rows.append(
            {
                "t": t,
                "phase": phase,
                "action_throttle": float(action[0]),
                "action_rudder_norm": float(action[1]),
                "action_bow": float(action[2]),
                "control_throttle": float(step_info.get("control_throttle", 0.0)),
                "control_rudder_angle_deg": float(step_info.get("control_rudder_angle_deg", 0.0)),
                "control_bow_thruster": float(step_info.get("control_bow_thruster", 0.0)),
                "x": env.state["x"],
                "y": env.state["y"],
                "psi": env.state["psi"],
                "u": env.state["u"],
                "v": env.state["v"],
                "r": env.state["r"],
                "reward": reward,
            }
        )
        final_info = step_info
        t += args.dt

    print(
        "[DIAG] done "
        f"success={final_info.get('success', False)} "
        f"collision={final_info.get('collision', False)} "
        f"timeout={final_info.get('timeout', False)} "
        f"steps={final_info.get('steps', 0)} "
        f"return={final_info.get('episode_return', 0.0):.3f}"
    )

    if args.trajectory_csv:
        os.makedirs(os.path.dirname(args.trajectory_csv), exist_ok=True)
        with open(args.trajectory_csv, "w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=list(rows[0].keys()) if rows else ["t", "phase"])
            writer.writeheader()
            writer.writerows(rows)
        print(f"[DIAG] wrote trajectory: {args.trajectory_csv}")


if __name__ == "__main__":
    main()
