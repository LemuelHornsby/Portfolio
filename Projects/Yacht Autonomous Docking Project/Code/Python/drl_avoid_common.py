import json
import math
from dataclasses import dataclass
from typing import Dict, Tuple


@dataclass
class AllowanceConfig:
    safety_allowance_m: float = 14.0
    ownship_long_margin: float = 36.0
    ownship_lat_margin: float = 28.0
    keepout_pad_m: float = 25.0
    keepout_hard_delta_m: float = 4.0
    orbit_extra_m: float = 18.0
    avoid_start_extra_m: float = 80.0
    clear_release_extra_m: float = 28.0
    spawn_behind_distance_m: float = 145.0
    side_wp_offset_extra_m: float = 8.0
    side_wp_back_along_m: float = 8.0
    front_wp_forward_along_m: float = 32.0
    front_wp_reach_m: float = 16.0
    stern_to_cg_m: float = 11.0
    passed_margin_m: float = 2.0


def load_marina_entities(
    scenario_json: str,
    marina_name: str,
    obstacle_id: int,
    goal_id: int,
) -> Tuple[Dict, Dict, Dict]:
    with open(scenario_json, "r", encoding="utf-8") as file:
        data = json.load(file)
    marina = next((m for m in data["marinas"] if m.get("name") == marina_name), None)
    if marina is None:
        raise ValueError(f"Marina '{marina_name}' not found in {scenario_json}")

    obstacle = next((o for o in marina.get("static_obstacles", []) if int(o.get("id", -1)) == obstacle_id), None)
    if obstacle is None:
        raise ValueError(f"Obstacle id {obstacle_id} not found in marina {marina_name}")

    goal = next((g for g in marina.get("dock_targets", []) if int(g.get("id", -1)) == goal_id), None)
    if goal is None:
        raise ValueError(f"Goal id {goal_id} not found in marina {marina_name}")

    return data, obstacle, goal


def unit(dx: float, dy: float) -> Tuple[float, float]:
    norm = math.hypot(dx, dy)
    if norm < 1e-9:
        return 1.0, 0.0
    return dx / norm, dy / norm


def project_along_cross(px: float, py: float, ox: float, oy: float, tx: float, ty: float) -> Tuple[float, float]:
    ex = px - ox
    ey = py - oy
    along = ex * tx + ey * ty
    cross = ex * (-ty) + ey * tx
    return along, cross


def build_allowances(obstacle: Dict, cfg: AllowanceConfig) -> Dict[str, float]:
    obstacle_r = float(obstacle.get("r", 12.0))
    obs_sx = float(obstacle.get("sx", 2.0 * obstacle_r))
    obs_sy = float(obstacle.get("sy", 2.0 * obstacle_r))

    keepout = obstacle_r + cfg.keepout_pad_m + cfg.safety_allowance_m
    hard_inner = keepout - cfg.keepout_hard_delta_m
    orbit = keepout + cfg.orbit_extra_m
    avoid_start = keepout + cfg.avoid_start_extra_m
    clear_release = orbit + cfg.clear_release_extra_m

    ellipse_a = 0.5 * obs_sx + cfg.ownship_long_margin + cfg.safety_allowance_m
    ellipse_b = 0.5 * obs_sy + cfg.ownship_lat_margin + cfg.safety_allowance_m

    return {
        "keepout_radius_m": keepout,
        "hard_inner_radius_m": hard_inner,
        "orbit_radius_m": orbit,
        "avoid_start_dist_m": avoid_start,
        "clear_release_m": clear_release,
        "ellipse_a": ellipse_a,
        "ellipse_b": ellipse_b,
    }
