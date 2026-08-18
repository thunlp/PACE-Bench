from pace_bench.tasks.stage_prompt import uniform_suffix_for_task

from typing import Any, Dict, List, Optional, Tuple

import re

from pace_bench.tasks.categories.Category5_Cybernetics_Control.C_04 import environment as c04_env

_DEFAULT_WALLS: Dict[int, Tuple[float, float, float, float]] = {
    0: (0.0, 0.0, 20.0, 0.5),
    1: (0.0, 2.5, 20.0, 0.5),
    2: (0.0, 0.0, 0.5, 3.0),
    3: (20.0, 0.0, 0.5, 3.0),
    4: (5.0, 0.0, 0.2, 1.0),
    5: (9.0, 1.8, 0.2, 1.2),
    6: (14.0, 1.8, 0.2, 1.2),

}

_DEFAULT_PHYSICS = {
    "control_lag_steps": 0,
    "structural_impulse_scale_k": float(c04_env.STRUCTURAL_IMPULSE_SCALE_K),
    "fluid_drag_x_min": -999.0,
    "fluid_drag_x_max": -999.0,
    "fluid_drag_coeff": 0.0,
    "turbulence_intensity": 0.0,
    "control_reversal_x_min": -999.0,
    "control_reversal_x_max": -999.0,
    "magnetic_floor_y_max": -999.0,
    "magnetic_floor_force": 0.0,
    "current_force_back": 0.0,
    "shear_wind_gradient": 0.0,
    "shear_wind_reference_y": float(c04_env.SHEAR_WIND_REFERENCE_Y),
    "oneway_force_right": float(c04_env.ONEWAY_FORCE_RIGHT),
    "lock_gate_fx": float(c04_env.LOCK_GATE_FX),
    "wind_oscillation_amp": float(c04_env.WIND_OSCILLATION_AMP),
    "wind_oscillation_omega": float(c04_env.WIND_OSCILLATION_OMEGA),
    "slip_friction": float(c04_env.SLIP_FRICTION),
    "max_steps": int(c04_env.MAX_STEPS),
    "lock_gate_x_min": float(c04_env.LOCK_GATE_X_MIN),
    "lock_gate_x_max": float(c04_env.LOCK_GATE_X_MAX),
    "activation_x_min": float(c04_env.ACTIVATION_X_MIN),
    "activation_x_max": float(c04_env.ACTIVATION_X_MAX),
    "backward_fx_threshold": float(c04_env.BACKWARD_FX_THRESHOLD),
    "backward_speed_max": float(c04_env.BACKWARD_SPEED_MAX),
    "backward_steps_required": int(c04_env.BACKWARD_STEPS_REQUIRED),

}

_DEFAULT_TERRAIN_DELAY = {
    "whisker_delay_steps": 0,
    "position_delay_steps": 0,
    "oneway_x": float(c04_env.ONEWAY_X),

}

_MAZE_INNER_WALL_LABELS = ["internal wall 1", "internal wall 2", "internal wall 3"]

def _labeled_inner_walls_subset(terrain: Dict[str, Any] | None, indices: List[int]) -> str:
    raw = _effective_walls_subset(terrain, indices)
    parts = raw.split("; ")
    labels = _MAZE_INNER_WALL_LABELS
    return "; ".join(f"{labels[i]} {parts[i]}" for i in range(len(parts)))

def _gravity_y(pc: Optional[Dict[str, Any]]) -> float:
    if not pc:
        return -9.8
    g = pc.get("gravity", -9.8)
    if isinstance(g, (list, tuple)):
        return float(g[1])
    return float(g)

def _merge_physics(pc: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    out = dict(_DEFAULT_PHYSICS)
    if pc:
        pc = dict(pc)
        pc.pop("task_description", None)
        if "structural_impulse_scale_k" not in pc and "collision_velocity_limit" in pc:
            pc["structural_impulse_scale_k"] = float(pc["collision_velocity_limit"])
        pc.pop("collision_velocity_limit", None)
        out.update(pc)
    return out

def _terrain_delays(tc: Optional[Dict[str, Any]]) -> Tuple[int, int]:
    t = tc or {}
    wd = int(t.get("whisker_delay_steps", _DEFAULT_TERRAIN_DELAY["whisker_delay_steps"]))
    pd = int(t.get("position_delay_steps", _DEFAULT_TERRAIN_DELAY["position_delay_steps"]))
    return wd, pd

def _effective_walls_subset(terrain: Dict[str, Any] | None, indices: List[int]) -> str:
    w = dict(_DEFAULT_WALLS)
    overrides = (terrain or {}).get("wall_overrides") or {}
    for k, v in overrides.items():
        w[int(k)] = tuple(float(x) for x in v)
    parts = [w[i] for i in indices]
    return "; ".join(f"({t[0]:.1f}, {t[1]:.1f}, {t[2]:.1f}, {t[3]:.1f})" for t in parts)

def _blind_active(lo: float, hi: float) -> bool:
    return lo > -500.0 and hi > -500.0 and lo < hi

def _fmt_impulse_ns(v: float) -> str:
    iv = int(round(v))
    if abs(v - iv) < 1e-6:
        return f"{iv} N·s"
    return f"{v:.1f} N·s"

def _configs_differ_from_base(
    tt: Optional[Dict[str, Any]],
    tp_raw: Optional[Dict[str, Any]],
    bt: Dict[str, Any],
    bp_merged: Dict[str, Any],
    bp_raw: Optional[Dict[str, Any]] = None,

) -> bool:
    tt = tt or {}
    tp_m = _merge_physics(tp_raw)
    bp_r = bp_raw or {}
    if _gravity_y(tp_raw) != _gravity_y(bp_r):
        return True
    if _effective_walls_subset(tt, list(range(7))) != _effective_walls_subset(bt, list(range(7))):
        return True
    blo, bhi = float(bt.get("whisker_blind_front_x_lo", -999.0)), float(bt.get("whisker_blind_front_x_hi", -999.0))
    tlo, thi = float(tt.get("whisker_blind_front_x_lo", -999.0)), float(tt.get("whisker_blind_front_x_hi", -999.0))
    base_blind = "none" if not _blind_active(blo, bhi) else f"x in [{blo:.1f}, {bhi:.1f}] m"
    tgt_blind = "none" if not _blind_active(tlo, thi) else f"x in [{tlo:.1f}, {thi:.1f}] m"
    if tgt_blind != base_blind:
        return True
    wdb, pdb = _terrain_delays(bt)
    wdt, pdt = _terrain_delays(tt)
    if wdb != wdt or pdb != pdt:
        return True
    if float(tt.get("oneway_x", _DEFAULT_TERRAIN_DELAY["oneway_x"])) != float(
        bt.get("oneway_x", _DEFAULT_TERRAIN_DELAY["oneway_x"])
    ):
        return True
    if int(tp_m["control_lag_steps"]) != int(bp_merged["control_lag_steps"]):
        return True
    if float(tp_m["structural_impulse_scale_k"]) != float(bp_merged["structural_impulse_scale_k"]):
        return True
    if int(tp_m.get("max_steps", _DEFAULT_PHYSICS["max_steps"])) != int(
        bp_merged.get("max_steps", _DEFAULT_PHYSICS["max_steps"])
    ):
        return True
    if int(tp_m.get("backward_steps_required", _DEFAULT_PHYSICS["backward_steps_required"])) != int(
        bp_merged.get("backward_steps_required", _DEFAULT_PHYSICS["backward_steps_required"])
    ):
        return True
    if float(tp_m.get("backward_fx_threshold", _DEFAULT_PHYSICS["backward_fx_threshold"])) != float(
        bp_merged.get("backward_fx_threshold", _DEFAULT_PHYSICS["backward_fx_threshold"])
    ):
        return True
    if float(tp_m.get("backward_speed_max", _DEFAULT_PHYSICS["backward_speed_max"])) != float(
        bp_merged.get("backward_speed_max", _DEFAULT_PHYSICS["backward_speed_max"])
    ):
        return True
    for key in (
        "fluid_drag_x_min",
        "fluid_drag_x_max",
        "fluid_drag_coeff",
        "turbulence_intensity",
        "control_reversal_x_min",
        "control_reversal_x_max",
        "magnetic_floor_y_max",
        "magnetic_floor_force",
        "current_force_back",
        "shear_wind_gradient",
        "shear_wind_reference_y",
        "oneway_force_right",
        "lock_gate_fx",
        "lock_gate_x_min",
        "lock_gate_x_max",
        "activation_x_min",
        "activation_x_max",
        "wind_oscillation_amp",
        "wind_oscillation_omega",
        "slip_friction",
    ):
        if float(tp_m.get(key, _DEFAULT_PHYSICS[key])) != float(bp_merged.get(key, _DEFAULT_PHYSICS[key])):
            return True
    return False

_UNIFORM_SUFFIX_GRAVITY_BULLET = ""

_UNIFORM_SUFFIX_BULLETS_REST = uniform_suffix_for_task("C_04")

def _uniform_suffix(include_gravity_mutation: bool) -> str:
    del include_gravity_mutation
    return uniform_suffix_for_task("C_04")

def _build_environmental_anomalies_suffix_curriculum_union() -> str:
    return _uniform_suffix(include_gravity_mutation=False).strip()

UNIFORM_SUFFIX = uniform_suffix_for_task("C_04")

MUTATED_SUCCESS_CRITERIA_POINTER = """
---
**Mutated environment:** The **Possible Environment Variations** section at the end of the Task Environment lists physical channels that may differ from the source environment; apply that notice when solving this stage.
"""

def update_task_description_for_visible_changes(
    base_description: str,
    target_terrain_config: Dict[str, Any],
    base_terrain_config: Dict[str, Any],
    target_physics_config: Dict[str, Any] = None,
    base_physics_config: Dict[str, Any] = None,

) -> str:
    description = base_description
    bt = base_terrain_config or {}
    tt = target_terrain_config or {}
    bp = _merge_physics(base_physics_config)
    tp = _merge_physics(target_physics_config)
    ws_base_03 = _effective_walls_subset(bt, [0, 1, 2, 3])
    ws_tgt_03 = _effective_walls_subset(tt, [0, 1, 2, 3])
    if ws_tgt_03 != ws_base_03:
        pat = r"(- \*\*Maze outer shell \(indices 0–3; lower-left x, y, width, height in m\)\*\*: )(.*?)(\.\n|$)"
        labels = ["floor", "ceiling", "left wall", "right wall"]
        base_parts = ws_base_03.split("; ")
        tgt_parts = ws_tgt_03.split("; ")
        base_labeled = "; ".join(f"{label} {part}" for label, part in zip(labels, base_parts))
        tgt_labeled = "; ".join(f"{label} {part}" for label, part in zip(labels, tgt_parts))
        description, match_count = re.subn(
            pat,
            lambda m: f"{m.group(1)}{tgt_labeled} (originally {base_labeled} in the source environment){m.group(3)}",
            description,
            count=1,
        )
        if match_count != 1:
            raise ValueError(
                "C_04 stages: maze outer-shell prompt replacement count was not one"
            )
    ws_base_46 = _effective_walls_subset(bt, [4, 5, 6])
    ws_tgt_46 = _effective_walls_subset(tt, [4, 5, 6])
    if ws_tgt_46 != ws_base_46:
        pat = r"(- \*\*Maze walls \(indices 4-6; lower-left x, y, width, height in m\)\*\*: )(.*?)(\.\n|$)"
        tgt_labeled_46 = _labeled_inner_walls_subset(tt, [4, 5, 6])
        base_labeled_46 = _labeled_inner_walls_subset(bt, [4, 5, 6])
        description, match_count = re.subn(
            pat,
            lambda m: f"{m.group(1)}{tgt_labeled_46} (originally {base_labeled_46} in the source environment){m.group(3)}",
            description,
            count=1,
        )
        if match_count != 1:
            raise ValueError(
                "C_04 stages: inner-wall prompt replacement count was not one"
            )
    tk, bk = float(tp["structural_impulse_scale_k"]), float(bp["structural_impulse_scale_k"])
    if tk != bk:
        am = float(c04_env.AGENT_MASS)
        imp_t, imp_b = tk * am, bk * am
        pat = r"(- \*\*Structural impulse limit\*\*: )(.*?)(\.\n|$)"
        replacement = (
            "Failure occurs when collision normal impulse exceeds "
            f"{_fmt_impulse_ns(imp_t)} (originally "
            f"{_fmt_impulse_ns(imp_b)} in the source environment)"
        )
        description, match_count = re.subn(
            pat, lambda m: f"{m.group(1)}{replacement}{m.group(3)}",
            description, count=1,
        )
        if match_count != 1:
            raise ValueError("C_04 stages: structural-limit prompt replacement count was not one")
    return description.strip()

def update_success_criteria_for_visible_changes(
    base_success_criteria: str,
    target_terrain_config: Dict[str, Any],
    base_terrain_config: Dict[str, Any],
    target_physics_config: Dict[str, Any] = None,
    base_physics_config: Dict[str, Any] = None,

) -> str:
    bt = base_terrain_config or {}
    tt = target_terrain_config or {}
    bp = _merge_physics(base_physics_config)
    tp = _merge_physics(target_physics_config)
    bp_raw = base_physics_config or {}
    tp_raw = target_physics_config or {}
    criteria = base_success_criteria
    am = float(c04_env.AGENT_MASS)
    tk, bk = float(tp["structural_impulse_scale_k"]), float(bp["structural_impulse_scale_k"])
    if tk != bk:
        imp_t, imp_b = tk * am, bk * am
        pat = r"(3\. \*\*Survival\*\*: )([^\n]+)"
        new_survival = (
            "Stay below the structural impulse limit: "
            f"**{_fmt_impulse_ns(imp_t)}** at this stage; originally "
            f"**{_fmt_impulse_ns(imp_b)}** in the source environment."
        )
        criteria, match_count = re.subn(
            pat, lambda m: f"{m.group(1)}{new_survival}", criteria, count=1
        )
        if match_count != 1:
            raise ValueError(
                "C_04 stages: structural success-criteria replacement count was not one"
            )
    if _configs_differ_from_base(tt, tp_raw, bt, bp, bp_raw):
        criteria = criteria.rstrip() + "\n" + MUTATED_SUCCESS_CRITERIA_POINTER.strip()
    return criteria

def get_source_base_physics_config() -> Dict[str, Any]:
    return dict(_merge_physics(None))

def get_source_base_terrain_config() -> Dict[str, Any]:
    return {}

def get_c04_curriculum_stages() -> List[Dict[str, Any]]:
    return [
        {
            "stage_id": "Stage-1",
            "title": "Controller Adaptation I",
            "mutation_description": "One or more listed environmental properties differ from the source environment.",
            "terrain_config": {},
            "physics_config": {
                "control_lag_steps": 25,
                "structural_impulse_scale_k": 12.0,
                "magnetic_floor_y_max": 1.6,
                "magnetic_floor_force": -60.0,
            },
            "task_description_suffix": uniform_suffix_for_task("C_04"),
        },
        {
            "stage_id": "Stage-2",
            "title": "Altered Passage Geometry",
            "mutation_description": "The visible inner-wall geometry differs; other listed properties may also differ.",
            "terrain_config": {
                "whisker_blind_front_x_lo": 5.0,
                "whisker_blind_front_x_hi": 13.0,
                "wall_overrides": {
                    "5": (9.0, 0.0, 0.2, 2.0),
                    "6": (14.0, 0.0, 0.2, 2.0),
                },
            },
            "physics_config": {},
            "task_description_suffix": uniform_suffix_for_task("C_04"),
        },
        {
            "stage_id": "Stage-3",
            "title": "Altered Passage Adaptation",
            "mutation_description": "The visible inner-wall geometry and exposed constraints differ; other listed properties may also differ.",
            "terrain_config": {
                "wall_overrides": {
                    "4": (5.0, 0.0, 0.2, 1.7),
                    "5": (9.0, 1.7, 0.2, 0.6),
                }
            },
            "physics_config": {
                "fluid_drag_x_min": 6.0,
                "fluid_drag_x_max": 14.0,
                "fluid_drag_coeff": 1.0,
                "turbulence_intensity": 100.0,
                "structural_impulse_scale_k": 65.0,
            },
            "task_description_suffix": uniform_suffix_for_task("C_04"),
        },
        {
            "stage_id": "Stage-4",
            "title": "Controller Adaptation IV",
            "mutation_description": "One or more listed environmental properties and exposed constraints differ from the source environment.",
            "terrain_config": {},
            "physics_config": {
                "control_reversal_x_min": 0.0,
                "control_reversal_x_max": 20.0,
                "magnetic_floor_y_max": 1.5,
                "magnetic_floor_force": -80.0,
                "turbulence_intensity": 150.0,
                "control_lag_steps": 0,
                "structural_impulse_scale_k": 50.0,
            },
            "task_description_suffix": uniform_suffix_for_task("C_04"),
        },
    ]
