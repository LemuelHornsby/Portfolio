import argparse
import json
import os
from statistics import mean

import numpy as np
from stable_baselines3 import PPO

from drl_avoid_env import YachtAvoidRecoverEnv


def main():
    parser = argparse.ArgumentParser(description="Evaluate trained avoid+recover PPO policy.")
    parser.add_argument("--scenario-json", default="marinas_export.json")
    parser.add_argument("--marina-name", default="MarinaRoot1")
    parser.add_argument("--obstacle-id", type=int, default=3)
    parser.add_argument("--goal-id", type=int, default=0)
    parser.add_argument("--model-dir", required=True)
    parser.add_argument("--model-name", default="final_model.zip")
    parser.add_argument("--episodes", type=int, default=200)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--max-steps", type=int, default=2000)
    parser.add_argument("--curriculum-stage", type=int, default=0)
    parser.add_argument("--output-json", default=None)
    parser.add_argument("--episodes-json", default=None)
    args = parser.parse_args()

    model_path = os.path.join(args.model_dir, args.model_name)
    if not os.path.exists(model_path):
        raise FileNotFoundError(f"Model file not found: {model_path}")

    env = YachtAvoidRecoverEnv(
        scenario_json=args.scenario_json,
        marina_name=args.marina_name,
        obstacle_id=args.obstacle_id,
        goal_id=args.goal_id,
        dt=args.dt,
        max_steps=args.max_steps,
        seed=args.seed,
        curriculum_stage=args.curriculum_stage,
    )
    model = PPO.load(model_path)

    rows = []
    for ep in range(args.episodes):
        obs, _ = env.reset(seed=args.seed + ep)
        done = False
        truncated = False
        info = {}
        while not done and not truncated:
            action, _ = model.predict(obs, deterministic=True)
            obs, _, done, truncated, info = env.step(action)
        reached_avoid = bool(info.get("reached_avoid", False))
        reached_recover = bool(info.get("reached_recover", False))
        milestone_success = reached_avoid and reached_recover
        rows.append(
            {
                "episode": ep,
                "success": milestone_success,
                "env_success": bool(info.get("success", False)),
                "collision": bool(info.get("collision", False)),
                "timeout": bool(info.get("timeout", False)),
                "steps": int(info.get("steps", 0)),
                "episode_return": float(info.get("episode_return", 0.0)),
                "reached_avoid": reached_avoid,
                "reached_recover": reached_recover,
            }
        )

    successes = [r for r in rows if r["success"]]
    collisions = [r for r in rows if r["collision"]]
    timeouts = [r for r in rows if r["timeout"]]

    metrics = {
        "episodes": args.episodes,
        "success_rate": len(successes) / max(args.episodes, 1),
        "collision_rate": len(collisions) / max(args.episodes, 1),
        "timeout_rate": len(timeouts) / max(args.episodes, 1),
        "avg_steps": float(mean(r["steps"] for r in rows)) if rows else 0.0,
        "avg_return": float(mean(r["episode_return"] for r in rows)) if rows else 0.0,
        "p95_steps": float(np.percentile([r["steps"] for r in rows], 95)) if rows else 0.0,
        "avoid_reached_rate": float(mean(1.0 if r["reached_avoid"] else 0.0 for r in rows)) if rows else 0.0,
        "recover_reached_rate": float(mean(1.0 if r["reached_recover"] else 0.0 for r in rows)) if rows else 0.0,
        "env_success_rate": float(mean(1.0 if r["env_success"] else 0.0 for r in rows)) if rows else 0.0,
        "curriculum_stage": int(args.curriculum_stage),
    }

    print(json.dumps(metrics, indent=2))

    if args.output_json:
        os.makedirs(os.path.dirname(args.output_json), exist_ok=True)
        with open(args.output_json, "w", encoding="utf-8") as file:
            json.dump(metrics, file, indent=2)
    if args.episodes_json:
        os.makedirs(os.path.dirname(args.episodes_json), exist_ok=True)
        with open(args.episodes_json, "w", encoding="utf-8") as file:
            json.dump(rows, file, indent=2)


if __name__ == "__main__":
    main()
