from __future__ import annotations

from pace_bench.tasks.stage_prompt import uniform_suffix_for_task

import importlib.util
import os
from typing import Any, Dict, List


_STAGES_DIR = os.path.dirname(os.path.abspath(__file__))
_ENV_SPEC = importlib.util.spec_from_file_location(
    "c03_environment_stages", os.path.join(_STAGES_DIR, "environment.py")
)
_ENV = importlib.util.module_from_spec(_ENV_SPEC)
_ENV_SPEC.loader.exec_module(_ENV)


def _replace_once(text: str, old: str, new: str, *, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise ValueError(f"C-03 prompt update for {label} expected one match, found {count}")
    return text.replace(old, new, 1)


def _fmt_slots(slots: List) -> str:
    return ", ".join(f"[{int(lo)}, {int(hi)}]" for lo, hi in slots)


def _fmt_obstacles(obstacles: List) -> str:
    if not obstacles:
        return "none"
    return "; ".join(
        f"at ({float(x):.1f}, {float(y):.1f}, {float(hw):.1f}, {float(hh):.1f})"
        for x, y, hw, hh in obstacles
    )


def _fmt_ice(zones: List) -> str:
    if not zones:
        return "none"
    entries = []
    for shape, _friction in zones:
        x, y, hw, hh = shape
        entries.append(
            f"centered at ({float(x):.1f}, {float(y):.2f}) m with half-size "
            f"({float(hw):.1f}, {float(hh):.2f}) m"
        )
    return "Low-friction surfaces " + " and ".join(entries) + " are present in the corridor"


def _fmt_moving(terrain: Dict[str, Any]) -> str:
    first = terrain.get("moving_obstacle", _ENV.MOVING_OBSTACLE)
    second = terrain.get("moving_obstacle_2", _ENV.MOVING_OBSTACLE_2)
    entries = []
    if first is not None:
        entries.append(f"({float(first[0]):.1f}, {float(first[1]):.1f})")
    if second is not None:
        entries.append(f"({float(second[0]):.1f}, {float(second[1]):.1f})")
    if not entries:
        return "none"
    joined = " and ".join(entries)
    return (
        f"Kinematic boxes oscillating horizontally in the corridor at {joined}; "
        "query `sandbox.get_terrain_obstacles()` for real-time positions"
    )


def update_task_description_for_visible_changes(
    base_description: str,
    target_terrain_config: Dict[str, Any],
    base_terrain_config: Dict[str, Any],
    target_physics_config: Dict[str, Any] | None = None,
    base_physics_config: Dict[str, Any] | None = None,
    *,
    stage: Dict[str, Any] | None = None,
) -> str:
    del target_physics_config, base_physics_config, stage
    terrain = dict(target_terrain_config or {})
    base = dict(base_terrain_config or {})
    text = base_description

    defaults = {
        "max_thrust_magnitude": _ENV.MAX_THRUST_MAGNITUDE,
        "spawn_x": 11.0,
        "spawn_y": 1.35,
        "ground_y_top": _ENV.DEFAULT_GROUND_Y_TOP,
        "activation_required_steps": _ENV.ACTIVATION_REQUIRED_STEPS,
        "activation_zone_x_min": _ENV.ACTIVATION_ZONE_X_MIN,
        "activation_zone_x_max": _ENV.ACTIVATION_ZONE_X_MAX,
        "impulse_budget": _ENV.IMPULSE_BUDGET,
        "rendezvous_distance": _ENV.RENDEZVOUS_DISTANCE_DEFAULT,
        "rendezvous_rel_speed": _ENV.RENDEZVOUS_REL_SPEED_DEFAULT,
        "rendezvous_heading_tolerance_deg": _ENV.RENDEZVOUS_HEADING_TOLERANCE_DEG_DEFAULT,
        "heading_reference_min_target_speed": _ENV.HEADING_REFERENCE_MIN_TARGET_SPEED,
        "track_distance": _ENV.TRACK_DISTANCE_DEFAULT,
        "cooldown_threshold": _ENV.COOLDOWN_THRESHOLD,
        "cooldown_max_thrust": _ENV.COOLDOWN_MAX_THRUST,
        "cooldown_steps": _ENV.COOLDOWN_STEPS,
    }

    def values(name: str):
        original = base.get(name, defaults[name])
        return original, terrain.get(name, original)

    original, current = values("max_thrust_magnitude")
    if current != original:
        text = _replace_once(
            text,
            f"max {original:g} N",
            f"max {current:g} N (originally {original:g} N in the source environment)",
            label="maximum thrust",
        )

    original_x, current_x = values("spawn_x")
    original_y, current_y = values("spawn_y")
    if (current_x, current_y) != (original_x, original_y):
        text = _replace_once(
            text,
            f"Spawns at ({original_x:.1f}, {original_y:.2f}) m (x, y)",
            f"Spawns at ({current_x:.1f}, {current_y:.2f}) m (x, y) "
            f"(originally ({original_x:.1f}, {original_y:.2f}) m in the source environment)",
            label="spawn position",
        )

    original, current = values("ground_y_top")
    if current != original:
        text = _replace_once(
            text,
            f"Top surface at y = {original:.1f} m",
            f"Top surface at y = {current:.1f} m "
            f"(originally {original:.1f} m in the source environment)",
            label="ground height",
        )

    default_ice = list(_ENV.ICE_ZONES)
    original_ice = base.get("ice_zones", default_ice)
    current_ice = terrain.get("ice_zones", original_ice)
    if current_ice != original_ice:
        text = _replace_once(
            text,
            f"- **Ice patches**: {_fmt_ice(original_ice)}.",
            f"- **Ice patches**: {_fmt_ice(current_ice)} "
            f"(originally {_fmt_ice(original_ice)} in the source environment).",
            label="ice-patch geometry",
        )

    default_obstacles = list(_ENV.OBSTACLES)
    original_obstacles = base.get("obstacles", default_obstacles)
    current_obstacles = terrain.get("obstacles", original_obstacles)
    if current_obstacles != original_obstacles:
        text = _replace_once(
            text,
            f"- **Static boxes**: {_fmt_obstacles(original_obstacles)}.",
            f"- **Static boxes**: {_fmt_obstacles(current_obstacles)} "
            f"(originally {_fmt_obstacles(original_obstacles)} in the source environment).",
            label="static obstacles",
        )

    original_moving = _fmt_moving(base)
    current_moving = _fmt_moving(terrain)
    if current_moving != original_moving:
        text = _replace_once(
            text,
            f"- **Moving obstacles**: {original_moving}.",
            f"- **Moving obstacles**: {current_moving} "
            f"(originally {original_moving} in the source environment).",
            label="moving obstacles",
        )

    original, current = values("activation_required_steps")
    original_lo, current_lo = values("activation_zone_x_min")
    original_hi, current_hi = values("activation_zone_x_max")
    if (current, current_lo, current_hi) != (original, original_lo, original_hi):
        old = (
            f"at least {int(original)} consecutive steps with seeker x in "
            f"[{original_lo:.1f}, {original_hi:.1f}] m"
        )
        new = (
            f"at least {int(current)} consecutive steps with seeker x in "
            f"[{current_lo:.1f}, {current_hi:.1f}] m "
            f"(originally {int(original)} steps in [{original_lo:.1f}, {original_hi:.1f}] m "
            "in the source environment)"
        )
        text = _replace_once(text, old, new, label="activation gate")

    original_p1 = base.get("slots_phase1", _ENV.SLOTS_PHASE1)
    original_p2 = base.get("slots_phase2", _ENV.SLOTS_PHASE2)
    current_p1 = terrain.get("slots_phase1", original_p1)
    current_p2 = terrain.get("slots_phase2", original_p2)
    if current_p1 != original_p1 or current_p2 != original_p2:
        old = f"Phase 1 {_fmt_slots(original_p1)}; phase 2 {_fmt_slots(original_p2)} simulation steps"
        new = (
            f"Phase 1 {_fmt_slots(current_p1)}; phase 2 {_fmt_slots(current_p2)} simulation steps "
            f"(originally phase 1 {_fmt_slots(original_p1)}; phase 2 {_fmt_slots(original_p2)} "
            "in the source environment)"
        )
        text = _replace_once(text, old, new, label="rendezvous slots")

    original, current = values("impulse_budget")
    if current != original:
        text = _replace_once(
            text,
            f"limited to {original:.0f} N·s",
            f"limited to {current:.0f} N·s "
            f"(originally {original:.0f} N·s in the source environment)",
            label="impulse budget",
        )

    original, current = values("rendezvous_distance")
    if current != original:
        text = _replace_once(
            text,
            f"distance to target ≤ {original:.1f} m",
            f"distance to target ≤ {current:.1f} m "
            f"(originally {original:.1f} m in the source environment)",
            label="rendezvous distance",
        )

    original, current = values("rendezvous_rel_speed")
    if current != original:
        text = _replace_once(
            text,
            f"relative speed < {original:g} m/s",
            f"relative speed < {current:g} m/s "
            f"(originally {original:g} m/s in the source environment)",
            label="rendezvous relative speed",
        )

    original, current = values("rendezvous_heading_tolerance_deg")
    if current != original:
        text = _replace_once(
            text,
            f"heading within {original:.1f}° of the reference direction",
            f"heading within {current:.1f}° of the reference direction "
            f"(originally {original:.1f}° in the source environment)",
            label="heading tolerance",
        )

    original, current = values("heading_reference_min_target_speed")
    if current != original:
        text = _replace_once(
            text,
            f"target speed ≥ {original:g} m/s",
            f"target speed ≥ {current:g} m/s "
            f"(originally {original:g} m/s in the source environment)",
            label="heading reference threshold",
        )

    original, current = values("track_distance")
    if current != original:
        text = _replace_once(
            text,
            f"Maintain distance <= {original:g} m",
            f"Maintain distance <= {current:g} m "
            f"(originally {original:g} m in the source environment)",
            label="tracking distance",
        )

    original_t, current_t = values("cooldown_threshold")
    original_m, current_m = values("cooldown_max_thrust")
    original_s, current_s = values("cooldown_steps")
    if (current_t, current_m, current_s) != (original_t, original_m, original_s):
        old = (
            f"Exceeding {original_t:.0f} N thrust triggers a cooldown; during the next "
            f"{int(original_s)} steps, maximum thrust is reduced to {original_m:.0f} N"
        )
        new = (
            f"Exceeding {current_t:.0f} N thrust triggers a cooldown; during the next "
            f"{int(current_s)} steps, maximum thrust is reduced to {current_m:.0f} N "
            f"(originally {original_t:.0f} N, {int(original_s)} steps, and "
            f"{original_m:.0f} N in the source environment)"
        )
        text = _replace_once(text, old, new, label="cooldown regime")

    return text


def update_success_criteria_for_visible_changes(
    base_success_criteria: str,
    target_terrain_config: Dict[str, Any],
    base_terrain_config: Dict[str, Any],
    target_physics_config: Dict[str, Any] | None = None,
    base_physics_config: Dict[str, Any] | None = None,
    *,
    stage: Dict[str, Any] | None = None,
) -> str:
    del target_physics_config, base_physics_config, stage
    terrain = dict(target_terrain_config or {})
    base = dict(base_terrain_config or {})
    text = base_success_criteria

    def values(name: str, default):
        original = base.get(name, default)
        return original, terrain.get(name, original)

    ast0, ast1 = values("activation_required_steps", _ENV.ACTIVATION_REQUIRED_STEPS)
    az00, az01 = values("activation_zone_x_min", _ENV.ACTIVATION_ZONE_X_MIN)
    az10, az11 = values("activation_zone_x_max", _ENV.ACTIVATION_ZONE_X_MAX)
    if (ast1, az01, az11) != (ast0, az00, az10):
        old = f"≥{int(ast0)} consecutive steps with seeker x ∈ [{az00:.1f}, {az10:.1f}] m"
        new = (
            f"≥{int(ast1)} consecutive steps with seeker x ∈ [{az01:.1f}, {az11:.1f}] m "
            f"(originally {int(ast0)} steps in [{az00:.1f}, {az10:.1f}] m in the source environment)"
        )
        text = _replace_once(text, old, new, label="success activation gate")

    rz00, rz01 = values("rendezvous_zone_x_min", _ENV.RENDEZVOUS_ZONE_X_MIN)
    rz10, rz11 = values("rendezvous_zone_x_max", _ENV.RENDEZVOUS_ZONE_X_MAX)
    if (rz01, rz11) != (rz00, rz10):
        text = _replace_once(
            text,
            f"seeker x ∈ [{rz00:.1f}, {rz10:.1f}] m",
            f"seeker x ∈ [{rz01:.1f}, {rz11:.1f}] m "
            f"(originally [{rz00:.1f}, {rz10:.1f}] m in the source environment)",
            label="success rendezvous zone",
        )

    rd0, rd1 = values("rendezvous_distance", _ENV.RENDEZVOUS_DISTANCE_DEFAULT)
    if rd1 != rd0:
        text = _replace_once(
            text,
            f"distance to **true** target ≤ {rd0:.1f} m",
            f"distance to **true** target ≤ {rd1:.1f} m "
            f"(originally {rd0:.1f} m in the source environment)",
            label="success rendezvous distance",
        )

    rs0, rs1 = values("rendezvous_rel_speed", _ENV.RENDEZVOUS_REL_SPEED_DEFAULT)
    if rs1 != rs0:
        text = _replace_once(
            text,
            f"relative speed < {rs0:g} m/s",
            f"relative speed < {rs1:g} m/s "
            f"(originally {rs0:g} m/s in the source environment)",
            label="success relative speed",
        )

    ht0, ht1 = values(
        "rendezvous_heading_tolerance_deg", _ENV.RENDEZVOUS_HEADING_TOLERANCE_DEG_DEFAULT
    )
    if ht1 != ht0:
        text = _replace_once(
            text,
            f"heading within {ht0:.1f}° of target velocity direction",
            f"heading within {ht1:.1f}° of target velocity direction "
            f"(originally {ht0:.1f}° in the source environment)",
            label="success heading tolerance",
        )

    href0, href1 = values(
        "heading_reference_min_target_speed", _ENV.HEADING_REFERENCE_MIN_TARGET_SPEED
    )
    if href1 != href0:
        text = _replace_once(
            text,
            f"target speed ≥ {href0:g} m/s",
            f"target speed ≥ {href1:g} m/s "
            f"(originally {href0:g} m/s in the source environment)",
            label="success heading reference threshold",
        )

    track0, track1 = values("track_distance", _ENV.TRACK_DISTANCE_DEFAULT)
    if track1 != track0:
        text = _replace_once(
            text,
            f"Maintain distance <= {track0:g} m",
            f"Maintain distance <= {track1:g} m "
            f"(originally {track0:g} m in the source environment)",
            label="success tracking distance",
        )

    impulse0, impulse1 = values("impulse_budget", _ENV.IMPULSE_BUDGET)
    if impulse1 != impulse0:
        text = _replace_once(
            text,
            f"must not exceed **{impulse0:.0f} N·s**",
            f"must not exceed **{impulse1:.0f} N·s** "
            f"(originally {impulse0:.0f} N·s in the source environment)",
            label="success impulse budget",
        )
        text = _replace_once(
            text,
            f"**Fuel budget**: {impulse0:.0f} N·s",
            f"**Fuel budget**: {impulse1:.0f} N·s "
            f"(originally {impulse0:.0f} N·s in the source environment)",
            label="fuel-budget detail",
        )

    ct0, ct1 = values("cooldown_threshold", _ENV.COOLDOWN_THRESHOLD)
    cm0, cm1 = values("cooldown_max_thrust", _ENV.COOLDOWN_MAX_THRUST)
    cs0, cs1 = values("cooldown_steps", _ENV.COOLDOWN_STEPS)
    if (ct1, cm1, cs1) != (ct0, cm0, cs0):
        old = (
            f"Exceeding {ct0:.0f} N thrust triggers cooldown; max thrust reduced to "
            f"{cm0:.0f} N for {int(cs0)} steps"
        )
        new = (
            f"Exceeding {ct1:.0f} N thrust triggers cooldown; max thrust reduced to "
            f"{cm1:.0f} N for {int(cs1)} steps "
            f"(originally {ct0:.0f} N, {cm0:.0f} N, and {int(cs0)} steps "
            "in the source environment)"
        )
        text = _replace_once(text, old, new, label="success cooldown regime")

    return text


UNIFORM_SUFFIX = uniform_suffix_for_task("C_03")


def get_c03_curriculum_stages() -> List[Dict[str, Any]]:
    return [
        {
            "stage_id": "Stage-1",
            "title": "Threshold-Locked Shadow Pursuit",
            "mutation_description": "A narrow safe-thrust envelope, sensor shadow, and split rendezvous windows force a deliberate observe-activate-coast-intercept cycle; one aggressive correction causes a long near-zero-thrust lockout.",
            "task_description_suffix": uniform_suffix_for_task("C_03"),
            "terrain_config": {
                "impulse_budget": 3900.0,
                "target_speed": 1.7,
                "target_change_interval": 3.8,
                "rendezvous_distance": 3.6,
                "rendezvous_rel_speed": 2.0,
                "rendezvous_heading_tolerance_deg": 96.0,
                "max_thrust_magnitude": 76.0,
                "cooldown_threshold": 28.0,
                "cooldown_max_thrust": 0.8,
                "cooldown_steps": 900,
                "track_distance": 7.2,
                "blind_zone_x_min": 9.5,
                "blind_zone_x_max": 15.8,
                "speed_blind_threshold_mps": 0.55,
                "spawn_x": 17.2,
                "spawn_y": 1.55,
                "slots_phase1": [[3500, 3580], [4300, 4380]],
                "slots_phase2": [[6100, 6180], [6900, 6980]],
            },
            "physics_config": {
                "linear_damping": 0.35,
                "angular_damping": 1.0,
                "gravity": (0.0, -0.4),
            },
        },
        {
            "stage_id": "Stage-2",
            "title": "Moving-Barrier Shadow Intercept",
            "mutation_description": "The threshold lock and sensor shadow persist while dual moving barriers cross the transfer lane, the target turns faster, and split slots require obstacle-aware observation, coasting, and braking.",
            "task_description_suffix": uniform_suffix_for_task("C_03"),
            "terrain_config": {
                "ground_friction": 0.0,
                "impulse_budget": 6500.0,
                "spawn_x": 17.2,
                "spawn_y": 1.55,
                "target_speed": 2.2,
                "target_change_interval": 3.0,
                "rendezvous_rel_speed": 2.25,
                "rendezvous_heading_tolerance_deg": 160.0,
                "track_distance": 12.5,
                "max_thrust_magnitude": 125.0,
                "cooldown_threshold": 80.0,
                "cooldown_max_thrust": 2.0,
                "cooldown_steps": 1000,
                "blind_zone_x_min": 12.5,
                "blind_zone_x_max": 17.4,
                "speed_blind_threshold_mps": 0.65,
                "slots_phase1": [[3300, 3400], [3950, 4050]],
                "slots_phase2": [[5900, 6000], [6550, 6650]],
                "obstacles": [],
                "ice_zones": [],
                "moving_obstacle": (14.2, 2.65, 0.4, 0.25),
                "moving_obstacle_2": (16.6, 2.55, 0.35, 0.25),
            },
            "physics_config": {
                "linear_damping": 0.15,
                "angular_damping": 1.0,
                "gravity": (2.0, -3.0),
            },
        },
        {
            "stage_id": "Stage-3",
            "title": "Alternating-Obstacle Blind Chase",
            "mutation_description": "A wider shadow, two-sided obstacle geometry, a longer weak-thrust lockout, and high-frequency stochastic turns force memory-based prediction with explicit corridor and collision management.",
            "task_description_suffix": uniform_suffix_for_task("C_03"),
            "terrain_config": {
                "ground_friction": 0.02,
                "impulse_budget": 17000.0,
                "rendezvous_rel_speed": 2.0,
                "rendezvous_heading_tolerance_deg": 120.0,
                "max_thrust_magnitude": 200.0,
                "cooldown_threshold": 171.0,
                "cooldown_max_thrust": 2.0,
                "cooldown_steps": 1100,
                "blind_zone_x_min": 12.5,
                "blind_zone_x_max": 17.0,
                "speed_blind_threshold_mps": 4.0,
                "track_distance": 10.5,
                "target_speed": 2.0,
                "target_change_interval": 0.7,
                "slots_phase1": [[3700, 3880], [4200, 4380]],
                "slots_phase2": [[6200, 6380], [6700, 6880]],
                "spawn_x": 11.0,
                "spawn_y": 1.55,
                "ice_zones": [],
                "moving_obstacle": None,
                "moving_obstacle_2": None,
                "obstacles": [(7.5, 1.3, 0.3, 0.3), (20.5, 1.3, 0.3, 0.3)],
            },
            "physics_config": {
                "linear_damping": 1.4,
                "angular_damping": 1.5,
                "gravity": (-3.5, -9.5),
            },
        },
        {
            "stage_id": "Stage-4",
            "title": "Compound Lockout Gauntlet",
            "mutation_description": "The widest shadow, dual moving barriers, static edge traps, fastest target changes, strictest rendezvous geometry, smallest budget, and longest lockout combine into a predictive multi-window gauntlet.",
            "task_description_suffix": uniform_suffix_for_task("C_03"),
            "terrain_config": {
                "impulse_budget": 7000.0,
                "ground_friction": 0.005,
                "cooldown_threshold": 58.0,
                "cooldown_max_thrust": 8.0,
                "cooldown_steps": 1200,
                "rendezvous_distance": 3.2,
                "rendezvous_rel_speed": 1.0,
                "rendezvous_heading_tolerance_deg": 30.0,
                "track_distance": 9.2,
                "target_speed": 1.0,
                "target_change_interval": 0.6,
                "blind_zone_x_min": 15.0,
                "blind_zone_x_max": 17.5,
                "speed_blind_threshold_mps": 1.2,
                "slots_phase1": [[3650, 3880], [4150, 4410]],
                "slots_phase2": [[6150, 6380], [6650, 6910]],
                "spawn_x": 14.0,
                "spawn_y": 1.45,
                "obstacles": [],
                "ice_zones": [],
                "moving_obstacle": None,
                "moving_obstacle_2": None,
            },
            "physics_config": {
                "linear_damping": 2.8,
                "angular_damping": 4.0,
                "gravity": (0.0, -10.0),
            },
        },
    ]
