import argparse
import json
import os
from dataclasses import asdict

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.vec_env import DummyVecEnv, VecMonitor

from drl_avoid_common import AllowanceConfig
from drl_avoid_env import YachtAvoidRecoverEnv


def build_env(args):
    return YachtAvoidRecoverEnv(
        scenario_json=args.scenario_json,
        marina_name=args.marina_name,
        obstacle_id=args.obstacle_id,
        goal_id=args.goal_id,
        dt=args.dt,
        max_steps=args.max_steps,
        seed=args.seed,
        curriculum_stage=0,
    )


def main():
    parser = argparse.ArgumentParser(description="Train PPO for yacht obstacle avoid+recover phase.")
    parser.add_argument("--scenario-json", default="marinas_export.json")
    parser.add_argument("--marina-name", default="MarinaRoot1")
    parser.add_argument("--obstacle-id", type=int, default=3)
    parser.add_argument("--goal-id", type=int, default=0)
    parser.add_argument("--model-dir", required=True)
    parser.add_argument("--timesteps", type=int, default=1000000)
    parser.add_argument("--init-model-dir", default=None)
    parser.add_argument("--init-model-name", default="final_model.zip")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--max-steps", type=int, default=2000)
    parser.add_argument("--n-envs", type=int, default=20)
    args = parser.parse_args()

    os.makedirs(args.model_dir, exist_ok=True)

    def _thunk(rank: int):
        def _make():
            env = build_env(args)
            env.reset(seed=args.seed + rank)
            return env

        return _make

    vec_env = DummyVecEnv([_thunk(i) for i in range(args.n_envs)])
    vec_env = VecMonitor(vec_env)

    if args.init_model_dir:
        init_model_path = os.path.join(args.init_model_dir, args.init_model_name)
        if not os.path.isfile(init_model_path):
            raise FileNotFoundError(f"Init model not found: {init_model_path}")
        model = PPO.load(init_model_path, env=vec_env, seed=args.seed)
        print(f"Loaded initial model from: {init_model_path}")
    else:
        model = PPO(
            "MlpPolicy",
            vec_env,
            learning_rate=3e-4,
            n_steps=2048,
            batch_size=256,
            n_epochs=10,
            gamma=0.995,
            gae_lambda=0.95,
            clip_range=0.2,
            ent_coef=0.005,
            vf_coef=0.5,
            max_grad_norm=0.5,
            verbose=1,
            seed=args.seed,
            tensorboard_log=os.path.join(args.model_dir, "tb"),
        )

    checkpoint_cb = CheckpointCallback(
        save_freq=max(10000 // max(args.n_envs, 1), 1),
        save_path=os.path.join(args.model_dir, "checkpoints"),
        name_prefix="ppo_avoid_recover",
    )
    model.learn(total_timesteps=args.timesteps, callback=[checkpoint_cb], progress_bar=True)

    model_path = os.path.join(args.model_dir, "final_model")
    model.save(model_path)

    metadata = {
        "model_path": model_path,
        "scenario_json": args.scenario_json,
        "marina_name": args.marina_name,
        "obstacle_id": args.obstacle_id,
        "goal_id": args.goal_id,
        "seed": args.seed,
        "timesteps": args.timesteps,
        "init_model_dir": args.init_model_dir,
        "init_model_name": args.init_model_name,
        "dt": args.dt,
        "max_steps": args.max_steps,
        "n_envs": args.n_envs,
        "allowances": asdict(AllowanceConfig()),
        "curriculum": "disabled_demo_style_single_distribution",
    }
    with open(os.path.join(args.model_dir, "train_config.json"), "w", encoding="utf-8") as file:
        json.dump(metadata, file, indent=2)

    print(f"Training complete. Saved model to: {model_path}.zip")


if __name__ == "__main__":
    main()
