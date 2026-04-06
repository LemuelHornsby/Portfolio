import math
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from drl_avoid_common import AllowanceConfig, build_allowances, load_marina_entities, project_along_cross, unit
from mmg_setup_yacht_bowthruster import clamp, make_yacht_params, wrap_pi
from simulate_yacht_mmg_bowthruster import step_layer3_mmg


@dataclass
class CurriculumConfig:
    stage: int = 0


class YachtAvoidRecoverEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(
        self,
        scenario_json: str = "marinas_export.json",
        marina_name: str = "MarinaRoot1",
        obstacle_id: int = 3,
        goal_id: int = 0,
        dt: float = 0.05,
        max_steps: int = 2000,
        seed: Optional[int] = None,
        curriculum_stage: int = 0,
    ):
        super().__init__()
        _, self.obstacle, self.goal = load_marina_entities(scenario_json, marina_name, obstacle_id, goal_id)
        self.allow_cfg = AllowanceConfig()
        self.allowances = build_allowances(self.obstacle, self.allow_cfg)

        self.curriculum = CurriculumConfig(stage=curriculum_stage)
        self.dt = dt
        self.max_steps = max_steps
        self.rng = np.random.default_rng(seed)

        self.start_ref_x = 0.0
        self.start_ref_y = 100.0
        self.obs_x = float(self.obstacle["x"])
        self.obs_y = float(self.obstacle["y"])
        self.goal_x = float(self.goal["x"])
        self.goal_y = float(self.goal["y"])

        self.tx, self.ty = unit(self.goal_x - self.start_ref_x, self.goal_y - self.start_ref_y)
        self.nominal_heading = math.atan2(self.ty, self.tx)
        self.obs_yaw = float(self.obstacle.get("yaw", 0.0))
        self.obs_sx = float(self.obstacle.get("sx", 2.0 * float(self.obstacle.get("r", 12.0))))
        self.obs_sy = float(self.obstacle.get("sy", 2.0 * float(self.obstacle.get("r", 12.0))))

        self.params = make_yacht_params(
            tune={
                "lin_hull": 2.5,
                "nl_hull": 1.0,
                "added": 1.0,
                "thrust": 4.0,
                "rudder": 1.0,
                "yaw_damp": 1.0,
                "sway_damp": 1.6,
                "Xuu_scale": 1.0,
            }
        )

        self.action_space = spaces.Box(
            low=np.array([-1.0, -1.0, -1.0], dtype=np.float32),
            high=np.array([1.0, 1.0, 1.0], dtype=np.float32),
            dtype=np.float32,
        )
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(18,), dtype=np.float32)

        self.state: Dict[str, float] = {}
        self.phase = "approach"
        self.steps = 0
        self.episode_return = 0.0
        self.prev_along = 0.0
        self.prev_d_front = 0.0
        self.prev_abs_cross = 0.0
        self.prev_ell_q = 1.0
        self.avoid_steps = 0
        self.prev_abs_epsi = 0.0
        self.avoid_bow_sign = 1.0
        self.pass_side_sign = 1.0
        self.avoid_r_integral = 0.0
        self.avoid_turn_sign = 0.0
        self.recover_steps = 0
        self.recover_start_abs_epsi = math.pi
        self.recover_best_abs_epsi = math.pi
        self._phase_reached = {"avoid": False, "recover": False}
        self.min_forward_throttle_approach = 0.28
        self.min_forward_throttle_avoid = 0.16
        self.min_forward_throttle_recover = 0.12
        self.near_obstacle_band_m = 20.0
        self.near_obstacle_max_throttle = 0.24
        self.safety_rudder_min_deg = 18.0
        self.safety_bow_min = 0.50
        self.avoid_target_cross_ratio = 0.65
        self.avoid_guidance_rudder_min_deg = 32.0
        self.avoid_guidance_bow_min = 0.95
        self.avoid_guidance_max_throttle = 0.20
        self.terminate_on_recover_milestone = True

        self.side_wp_x = 0.0
        self.side_wp_y = 0.0
        self.front_wp_x = 0.0
        self.front_wp_y = 0.0
        self.obstacle_bow_along = 0.0

    def set_curriculum_stage(self, stage: int):
        self.curriculum.stage = 0

    def _compute_waypoints(self):
        nx, ny = -self.ty, self.tx
        side_offset = self.allowances["orbit_radius_m"] + self.allow_cfg.side_wp_offset_extra_m
        self.side_wp_x = self.obs_x - self.tx * self.allow_cfg.side_wp_back_along_m + self.pass_side_sign * side_offset * nx
        self.side_wp_y = self.obs_y - self.ty * self.allow_cfg.side_wp_back_along_m + self.pass_side_sign * side_offset * ny
        self.front_wp_x = self.obs_x + self.tx * self.allow_cfg.front_wp_forward_along_m + self.pass_side_sign * side_offset * nx
        self.front_wp_y = self.obs_y + self.ty * self.allow_cfg.front_wp_forward_along_m + self.pass_side_sign * side_offset * ny

        c_obs = math.cos(self.obs_yaw)
        s_obs = math.sin(self.obs_yaw)
        route_in_obs_x = self.tx * c_obs + self.ty * s_obs
        route_in_obs_y = -self.tx * s_obs + self.ty * c_obs
        self.obstacle_bow_along = abs(route_in_obs_x) * (0.5 * self.obs_sx) + abs(route_in_obs_y) * (0.5 * self.obs_sy)

    def _phase_transition(self, metrics: Dict[str, float]):
        prev_phase = self.phase
        if self.phase == "approach" and metrics["dist_obs"] < self.allowances["avoid_start_dist_m"]:
            self.phase = "avoid"
            self._phase_reached["avoid"] = True
        elif self.phase == "avoid" and (
            (metrics["passed_obstacle"] > 0.5 and metrics["along"] > (self.obstacle_bow_along + 4.0))
            or metrics["d_front"] < (self.allow_cfg.front_wp_reach_m + 5.0)
            or (
                metrics["along"] > (self.obstacle_bow_along - 2.0)
                and metrics["dist_obs"] > (1.05 * self.allowances["keepout_radius_m"])
            )
            or (
                (self.pass_side_sign * metrics["cross"]) > (0.30 * self.allowances["orbit_radius_m"])
                and metrics["dist_obs"] > (1.10 * self.allowances["keepout_radius_m"])
                and metrics["ell_q"] > 1.20
            )
        ):
            self.phase = "recover"
            self._phase_reached["recover"] = True
        elif (
            self.phase == "recover"
            and metrics["dist_obs"] > self.allowances["clear_release_m"]
            and metrics["along"] > 75.0
            and abs(metrics["e_psi_nominal"]) < math.radians(2.5)
            and abs(self.state["r"]) < 0.02
        ):
            self.phase = "track"
        return prev_phase != self.phase

    def _compute_metrics(self) -> Dict[str, float]:
        along, cross = project_along_cross(self.state["x"], self.state["y"], self.obs_x, self.obs_y, self.tx, self.ty)
        dist_obs = math.hypot(self.state["x"] - self.obs_x, self.state["y"] - self.obs_y)
        d_front = math.hypot(self.state["x"] - self.front_wp_x, self.state["y"] - self.front_wp_y)
        heading_to_front = math.atan2(self.front_wp_y - self.state["y"], self.front_wp_x - self.state["x"])
        e_psi_front = wrap_pi(heading_to_front - self.state["psi"])
        e_psi_nominal = wrap_pi(self.nominal_heading - self.state["psi"])
        stern_along = along - self.allow_cfg.stern_to_cg_m * (
            math.cos(self.state["psi"]) * self.tx + math.sin(self.state["psi"]) * self.ty
        )
        passed_obstacle = 1.0 if stern_along > (self.obstacle_bow_along + self.allow_cfg.passed_margin_m) else 0.0

        dxo = self.state["x"] - self.obs_x
        dyo = self.state["y"] - self.obs_y
        c_yaw = math.cos(self.obs_yaw)
        s_yaw = math.sin(self.obs_yaw)
        lx = c_yaw * dxo + s_yaw * dyo
        ly = -s_yaw * dxo + c_yaw * dyo
        ellipse_a = self.allowances["ellipse_a"]
        ellipse_b = self.allowances["ellipse_b"]
        ell_q = (lx / max(ellipse_a, 1e-6)) ** 2 + (ly / max(ellipse_b, 1e-6)) ** 2

        return {
            "along": along,
            "cross": cross,
            "dist_obs": dist_obs,
            "d_front": d_front,
            "e_psi_front": e_psi_front,
            "e_psi_nominal": e_psi_nominal,
            "passed_obstacle": passed_obstacle,
            "ell_q": ell_q,
        }

    def _obs(self, metrics: Dict[str, float]) -> np.ndarray:
        phase_one_hot = [
            1.0 if self.phase == "approach" else 0.0,
            1.0 if self.phase == "avoid" else 0.0,
            1.0 if self.phase == "recover" else 0.0,
            1.0 if self.phase == "track" else 0.0,
        ]
        obs = np.array(
            [
                self.state["x"],
                self.state["y"],
                self.state["psi"],
                self.state["u"],
                self.state["v"],
                self.state["r"],
                metrics["along"],
                metrics["cross"],
                metrics["dist_obs"],
                metrics["d_front"],
                metrics["e_psi_front"],
                metrics["e_psi_nominal"],
                metrics["ell_q"],
                metrics["passed_obstacle"],
                *phase_one_hot,
            ],
            dtype=np.float32,
        )
        return obs

    def _reward(self, metrics: Dict[str, float], transitioned: bool) -> float:
        reward = 0.0
        reward += 0.03 * (metrics["along"] - self.prev_along)
        front_progress = self.prev_d_front - metrics["d_front"]
        abs_cross = abs(metrics["cross"])
        cross_progress = abs_cross - self.prev_abs_cross
        ell_margin_progress = metrics["ell_q"] - self.prev_ell_q

        if metrics["ell_q"] < 1.0:
            reward -= 120.0
        elif metrics["ell_q"] < 1.15:
            reward -= 22.0 * (1.15 - metrics["ell_q"]) / 0.15
        elif metrics["dist_obs"] < self.allowances["keepout_radius_m"]:
            reward -= 2.5 * (self.allowances["keepout_radius_m"] - metrics["dist_obs"]) / max(self.allowances["keepout_radius_m"], 1e-6)
        elif metrics["ell_q"] < 1.25:
            reward -= 8.0 * (1.25 - metrics["ell_q"]) / 0.25

        if self.phase in ("avoid", "recover"):
            reward += 1.6 * ell_margin_progress

        if self.phase == "avoid":
            along_progress = metrics["along"] - self.prev_along
            reward += 0.16 * front_progress
            reward += 0.20 * along_progress
            reward += 1.0 * metrics["passed_obstacle"]
            reward += 0.24 * cross_progress

            if along_progress < 0.0:
                reward -= 0.28 * abs(along_progress)

            if self.state["u"] < 0.0:
                reward -= 0.35 * abs(self.state["u"])

            if metrics["along"] < -20.0:
                reward -= 0.01 * abs(metrics["along"] + 20.0)

            desired_signed_cross = self.pass_side_sign * (0.55 * self.allowances["orbit_radius_m"])
            signed_cross = self.pass_side_sign * metrics["cross"]
            side_error = max(0.0, desired_signed_cross - signed_cross)
            reward -= 0.02 * side_error

            if metrics["along"] < (self.obstacle_bow_along - 8.0) and abs_cross < (0.35 * self.allowances["orbit_radius_m"]):
                reward -= 0.12 * ((0.35 * self.allowances["orbit_radius_m"]) - abs_cross)
            reward -= 0.003 * float(self.avoid_steps)

        if self.phase == "recover":
            along_progress = metrics["along"] - self.prev_along
            reward += 0.12 * along_progress
            reward += 0.18 * front_progress
            heading_improve = self.prev_abs_epsi - abs(metrics["e_psi_nominal"])
            reward += 2.5 * heading_improve
            reward -= 0.5 * abs(metrics["e_psi_nominal"])
            reward -= 0.15 * abs(self.state["r"])
            if self._phase_reached["recover"]:
                reward += 1.0 * max(0.0, ell_margin_progress)
                if self.recover_steps > int(0.8 / max(self.dt, 1e-6)) and metrics["ell_q"] < 1.08:
                    reward -= 2.0 * (1.08 - metrics["ell_q"]) / 0.08
            if abs(self.avoid_turn_sign) > 0.5 and abs(self.state["r"]) > 0.003:
                recover_turn_sign = 1.0 if self.state["r"] > 0.0 else -1.0
                if recover_turn_sign == -self.avoid_turn_sign:
                    reward += 0.06
                else:
                    reward -= 0.08

        if transitioned:
            if self.phase == "avoid":
                reward += 4.0
            elif self.phase == "recover":
                reward += 45.0
            elif self.phase == "track":
                reward += 35.0
        return reward

    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        super().reset(seed=seed)
        if seed is not None:
            self.rng = np.random.default_rng(seed)

        self.pass_side_sign = 1.0
        spawn_jitter_along = float(self.rng.uniform(-3.0, 3.0))
        spawn_jitter_cross = float(self.rng.uniform(-2.0, 2.0))

        spawn_x = self.obs_x - self.tx * (self.allow_cfg.spawn_behind_distance_m + spawn_jitter_along) + spawn_jitter_cross * (-self.ty)
        spawn_y = self.obs_y - self.ty * (self.allow_cfg.spawn_behind_distance_m + spawn_jitter_along) + spawn_jitter_cross * (self.tx)
        spawn_psi = math.atan2(self.goal_y - spawn_y, self.goal_x - spawn_x)

        self.state = {"x": spawn_x, "y": spawn_y, "psi": spawn_psi, "u": 0.0, "v": 0.0, "r": 0.0}
        self.phase = "approach"
        self.steps = 0
        self.episode_return = 0.0
        self.prev_along = 0.0
        self.prev_d_front = 0.0
        self.avoid_steps = 0
        self.prev_abs_epsi = 0.0
        self.avoid_r_integral = 0.0
        self.avoid_turn_sign = 0.0
        self.recover_steps = 0
        self.recover_start_abs_epsi = math.pi
        self.recover_best_abs_epsi = math.pi
        self._phase_reached = {"avoid": False, "recover": False}
        self._compute_waypoints()

        metrics = self._compute_metrics()
        self.prev_along = metrics["along"]
        self.prev_d_front = metrics["d_front"]
        self.prev_abs_cross = abs(metrics["cross"])
        self.prev_ell_q = metrics["ell_q"]
        self.prev_abs_epsi = abs(metrics["e_psi_nominal"])
        self.avoid_bow_sign = self.pass_side_sign
        return self._obs(metrics), {
            "phase": self.phase,
            "curriculum_stage": 0,
        }

    def step(self, action: np.ndarray):
        self.steps += 1
        metrics_pre = self._compute_metrics()
        action = np.asarray(action, dtype=np.float32)
        raw_throttle = float(clamp(float(action[0]), -1.0, 1.0))
        throttle = raw_throttle
        if self.phase in ("approach", "avoid", "recover"):
            if self.phase in ("avoid", "recover"):
                throttle = 0.5 * (raw_throttle + 1.0)
            else:
                throttle = max(0.0, raw_throttle)
            if metrics_pre["dist_obs"] > self.allowances["keepout_radius_m"]:
                min_forward_throttle = self.min_forward_throttle_avoid
                if self.phase == "approach":
                    min_forward_throttle = self.min_forward_throttle_approach
                elif self.phase == "recover":
                    min_forward_throttle = self.min_forward_throttle_recover
                throttle = max(min_forward_throttle, throttle)
        rudder = float(clamp(float(action[1]) * 35.0, -35.0, 35.0))
        bow = float(clamp(float(action[2]), -1.0, 1.0))

        if self.phase in ("avoid", "recover"):
            near_obstacle_limit = self.allowances["keepout_radius_m"] + self.near_obstacle_band_m
            if metrics_pre["dist_obs"] < near_obstacle_limit:
                throttle = min(throttle, self.near_obstacle_max_throttle)
                if (abs(metrics_pre["cross"]) < (0.80 * self.allowances["orbit_radius_m"])) or (metrics_pre["ell_q"] < 1.25):
                    if self.pass_side_sign > 0.0:
                        rudder = max(rudder, self.safety_rudder_min_deg)
                        bow = max(bow, self.safety_bow_min)
                    else:
                        rudder = min(rudder, -self.safety_rudder_min_deg)
                        bow = min(bow, -self.safety_bow_min)

        if self.phase == "avoid":
            signed_cross = self.pass_side_sign * metrics_pre["cross"]
            target_signed_cross = self.avoid_target_cross_ratio * self.allowances["orbit_radius_m"]
            before_front_clear = metrics_pre["along"] < (self.obstacle_bow_along + 6.0)
            if before_front_clear and (signed_cross < target_signed_cross):
                throttle = min(throttle, self.avoid_guidance_max_throttle)
                if self.pass_side_sign > 0.0:
                    rudder = max(rudder, self.avoid_guidance_rudder_min_deg)
                    bow = max(bow, self.avoid_guidance_bow_min)
                else:
                    rudder = min(rudder, -self.avoid_guidance_rudder_min_deg)
                    bow = min(bow, -self.avoid_guidance_bow_min)

        if self.phase in ("avoid", "recover") and metrics_pre["ell_q"] < 1.15:
            throttle = min(throttle, 0.08)
            if self.pass_side_sign > 0.0:
                rudder = 35.0
                bow = 1.0
            else:
                rudder = -35.0
                bow = -1.0

        if self._phase_reached["recover"] and self.phase == "recover":
            recover_established = self.recover_steps >= int(0.5 / max(self.dt, 1e-6))
            if recover_established and metrics_pre["ell_q"] < 1.12:
                if metrics_pre["ell_q"] < 1.06:
                    throttle = min(throttle, 0.06)
                    shield_rudder = 35.0 * self.pass_side_sign
                    shield_bow = 1.0 * self.pass_side_sign
                else:
                    throttle = min(throttle, 0.12)
                    shield_rudder = 22.0 * self.pass_side_sign
                    shield_bow = 0.60 * self.pass_side_sign

                if shield_rudder > 0.0:
                    rudder = max(rudder, shield_rudder)
                    bow = max(bow, shield_bow)
                else:
                    rudder = min(rudder, shield_rudder)
                    bow = min(bow, shield_bow)

        control = {
            "throttle": throttle,
            "rudder_angle_deg": rudder,
            "bow_thruster": bow,
        }

        self.state = step_layer3_mmg(self.state, control, self.params, self.dt)
        metrics = self._compute_metrics()
        transitioned = self._phase_transition(metrics)
        if self.phase == "avoid":
            self.avoid_steps += 1
            self.avoid_r_integral += self.state["r"] * self.dt
            self.recover_steps = 0
        elif self.phase == "recover":
            if transitioned:
                self.avoid_turn_sign = 1.0 if self.avoid_r_integral >= 0.0 else -1.0
                if abs(self.avoid_r_integral) < 1e-6:
                    self.avoid_turn_sign = self.pass_side_sign
                self.recover_start_abs_epsi = abs(metrics["e_psi_nominal"])
                self.recover_best_abs_epsi = abs(metrics["e_psi_nominal"])
                self.recover_steps = 0
            else:
                self.recover_steps += 1
                self.recover_best_abs_epsi = min(self.recover_best_abs_epsi, abs(metrics["e_psi_nominal"]))
            self.avoid_steps = 0
        else:
            self.avoid_steps = 0
            self.recover_steps = 0

        terminated = False
        truncated = False
        success = False
        crossed_keepout = metrics["dist_obs"] < self.allowances["keepout_radius_m"]
        crossed_ellipse = metrics["ell_q"] < 1.0
        collision = crossed_keepout or crossed_ellipse
        timeout = self.steps >= self.max_steps

        in_front_or_passed = (
            (metrics["passed_obstacle"] > 0.5)
            or (metrics["along"] > (self.obstacle_bow_along + 2.0))
            or (metrics["d_front"] < (self.allow_cfg.front_wp_reach_m + 6.0))
        )
        recover_stable = self.recover_steps >= int(1.5 / max(self.dt, 1e-6))
        recover_progress = self.recover_start_abs_epsi - abs(metrics["e_psi_nominal"])
        heading_recovering = (recover_progress > math.radians(1.5)) or (abs(metrics["e_psi_nominal"]) < math.radians(20.0))
        opposite_turn_ok = (
            abs(self.avoid_turn_sign) < 0.5
            or abs(self.state["r"]) < 0.003
            or (1.0 if self.state["r"] > 0.0 else -1.0) == -self.avoid_turn_sign
        )
        clear_of_obstacle = (
            metrics["dist_obs"] >= self.allowances["keepout_radius_m"]
            and metrics["ell_q"] >= 1.0
        )
        realistic_recover_done = in_front_or_passed and recover_stable and heading_recovering and opposite_turn_ok and clear_of_obstacle

        milestone_success = self._phase_reached["avoid"] and self._phase_reached["recover"]
        milestones_success = milestone_success if self.terminate_on_recover_milestone else (milestone_success and realistic_recover_done)

        if self.phase == "track" or milestones_success:
            terminated = True
            success = True
        if collision:
            terminated = True
        if timeout:
            truncated = True

        reward = self._reward(metrics, transitioned)
        if success:
            reward += 30.0
        if collision:
            reward -= 200.0
        if timeout and not self._phase_reached["recover"]:
            reward -= 180.0

        self.episode_return += reward
        self.prev_along = metrics["along"]
        self.prev_d_front = metrics["d_front"]
        self.prev_abs_cross = abs(metrics["cross"])
        self.prev_ell_q = metrics["ell_q"]
        self.prev_abs_epsi = abs(metrics["e_psi_nominal"])

        info = {
            "phase": self.phase,
            "success": success,
            "collision": collision,
            "collision_keepout": crossed_keepout,
            "collision_ellipse": crossed_ellipse,
            "timeout": timeout,
            "episode_return": self.episode_return,
            "steps": self.steps,
            "reached_avoid": self._phase_reached["avoid"],
            "reached_recover": self._phase_reached["recover"],
            "dist_obs": metrics["dist_obs"],
            "e_psi_nominal": metrics["e_psi_nominal"],
            "along": metrics["along"],
            "cross": metrics["cross"],
            "d_front": metrics["d_front"],
            "ell_q": metrics["ell_q"],
            "curriculum_stage": self.curriculum.stage,
            "control_throttle": throttle,
            "control_rudder_angle_deg": rudder,
            "control_bow_thruster": bow,
        }
        return self._obs(metrics), float(reward), terminated, truncated, info
