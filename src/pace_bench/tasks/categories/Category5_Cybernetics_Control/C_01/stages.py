from __future__ import annotations

from pace_bench.tasks.stage_prompt import uniform_suffix_for_task

import importlib.util

import math

import os

import re

from typing import Any, Dict, List, Optional, Tuple

_env_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "environment.py")

_env_spec = importlib.util.spec_from_file_location(
    "davinci_category5_c01_environment_stages_only",
    _env_path,

)

_env_mod = importlib.util.module_from_spec(_env_spec)

assert _env_spec.loader is not None

_env_spec.loader.exec_module(_env_mod)

_BASE_POLE_START_ANGLE = _env_mod.DEFAULT_POLE_START_ANGLE

_BASE_POLE_LENGTH = _env_mod.POLE_LENGTH

_BASE_TRACK_CENTER_X = _env_mod.TRACK_CENTER_X

_BASE_MAX_STEPS = _env_mod.MAX_STEPS

_BASE_CART_MASS = _env_mod.CART_MASS

_BASE_POLE_MASS = _env_mod.POLE_MASS

_BASE_SAFE_HALF_RANGE = _env_mod.SAFE_HALF_RANGE

_BASE_SENSOR_DELAY_ANGLE_STEPS = _env_mod.DEFAULT_SENSOR_DELAY_ANGLE_STEPS

_BASE_SENSOR_DELAY_OMEGA_STEPS = _env_mod.DEFAULT_SENSOR_DELAY_OMEGA_STEPS

_BASE_CART_FORCE_LIMIT_NEWTONS = _env_mod.CART_FORCE_LIMIT_NEWTONS

_BASE_CART_RAIL_CENTER_Y = _env_mod.CART_RAIL_CENTER_Y

_BASE_BALANCE_ANGLE_DEG = _env_mod.BALANCE_ANGLE_DEG

def _fmt_track_center_m(x: float) -> str:
    xf = float(x)
    if math.isclose(xf, round(xf), rel_tol=0.0, abs_tol=1e-6):
        return f"{int(round(xf))}m"
    return f"{xf:.1f}m"

def _fmt_track_center_num(x: float) -> str:
    xf = float(x)
    if math.isclose(xf, round(xf), rel_tol=0.0, abs_tol=1e-6):
        return str(int(round(xf)))
    return f"{xf:.1f}"

def _scalar_physics_differs(a: float, b: float) -> bool:
    return not math.isclose(float(a), float(b), rel_tol=1e-12, abs_tol=1e-9)

def _task_sensor_delay_angle_line(target: int, old: int) -> str:
    base = f"- **Sensor reporting (angle)**: {target} simulation steps of delay from true state"
    if target == _BASE_SENSOR_DELAY_ANGLE_STEPS:
        return base + "."
    orig_suffix = f" (originally {old} simulation steps in the source environment)"
    return base + orig_suffix + "."

def _task_sensor_delay_omega_line(target: int, old: int) -> str:
    base = f"- **Sensor reporting (angular velocity)**: {target} simulation steps of delay from true state"
    if target == _BASE_SENSOR_DELAY_OMEGA_STEPS:
        return base + "."
    orig_suffix = f" (originally {old} simulation steps in the source environment)"
    return base + orig_suffix + "."

def _parse_task_center_x(description: str) -> Optional[float]:
    m = re.search(r"center x\s*=\s*(\d+\.?\d*)m", description)
    return float(m.group(1)) if m else None

def _parse_task_safe_half(description: str) -> Optional[float]:
    m = re.search(r"safe range ±(\d+\.?\d*)m inclusive", description)
    return float(m.group(1)) if m else None

def _parse_task_episode_steps(description: str) -> Optional[int]:
    m = re.search(r"- \*\*Episode length\*\*: At most (\d+) simulation steps", description)
    return int(m.group(1)) if m else None

def _parse_success_track(description: str) -> Optional[Tuple[float, float]]:
    m = re.search(
        r"\*\*Track Limits\*\*: Cart remains within the safe zone \(\|x - (\d+\.?\d*)\| ≤ (\d+\.?\d*)m",
        description,
    )
    if not m:
        return None
    return float(m.group(1)), float(m.group(2))

def _parse_success_episode_steps(description: str) -> Optional[int]:
    m = re.search(r"3\. \*\*Episode length\*\*: At most (\d+) simulation steps", description)
    return int(m.group(1)) if m else None

def _success_track_limits_line(target_track_center: float, target_safe_range: float) -> str:
    core = f"|x - {target_track_center:.1f}| ≤ {target_safe_range:.1f}m"
    if math.isclose(target_track_center, _BASE_TRACK_CENTER_X, rel_tol=0.0, abs_tol=1e-6) and math.isclose(
        target_safe_range, _BASE_SAFE_HALF_RANGE, rel_tol=0.0, abs_tol=1e-6
    ):
        return f"2. **Track Limits**: Cart remains within the safe zone ({core})."
    base_cx = _fmt_track_center_num(_BASE_TRACK_CENTER_X)
    return (
        f"2. **Track Limits**: Cart remains within the safe zone (|x - {target_track_center:g}| ≤ {target_safe_range:g}m) "
        f"(originally |x - {_BASE_TRACK_CENTER_X:g}| ≤ {_BASE_SAFE_HALF_RANGE:g}m in the source environment)."
    )

def _parse_task_cart_mass(description: str) -> Optional[float]:
    m = re.search(r"\*\*Cart\*\*: [Aa] body of mass (\d+\.?\d*) kg", description)
    return float(m.group(1)) if m else None

def _parse_task_pole_mass(description: str) -> Optional[float]:
    m = re.search(r"\*\*Pole\*\*: Mass (\d+\.?\d*) kg", description)
    return float(m.group(1)) if m else None

def _parse_task_pole_length_m(description: str) -> Optional[float]:
    m = re.search(r"\*\*Length\*\*: (\d+\.?\d*)m", description)
    return float(m.group(1)) if m else None

def _parse_task_actuator_limit_n(description: str) -> Optional[float]:
    m = re.search(r"\*\*Actuator Limit\*\*: The cart force is limited to ±(\d+(?:\.\d+)?)\s*N", description)
    return float(m.group(1)) if m else None

def _parse_task_rail_y(description: str) -> Optional[float]:
    m = re.search(
        r"horizontal track at y=(\d+\.?\d*)m(?: \(originally [\d.]+m in the source environment\))?",
        description,
    )
    return float(m.group(1)) if m else None

def _parse_task_sensor_angle_delay(description: str) -> Optional[int]:
    m = re.search(
        r"\*\*Sensor reporting \(angle\)\*\*: (\d+) simulation steps of delay from true state",
        description,
    )
    return int(m.group(1)) if m else None

def _parse_task_sensor_omega_delay(description: str) -> Optional[int]:
    m = re.search(
        r"\*\*Sensor reporting \(angular velocity\)\*\*: (\d+) simulation steps of delay from true state",
        description,
    )
    return int(m.group(1)) if m else None

def _verify_task_description_sync(
    description: str,
    target_track_center: float,
    target_safe_range: float,
    target_max_steps: int,
    target_cart_mass: float,
    target_pole_mass: float,
    target_pole_length: float,
    target_cart_force_limit: float,
    target_sensor_delay_angle: int,
    target_sensor_delay_omega: int,
    target_cart_rail_center_y: float,

) -> None:
    pc = _parse_task_center_x(description)
    if pc is not None and not math.isclose(pc, target_track_center, rel_tol=0.0, abs_tol=1e-6):
        raise RuntimeError(
        )
    ps = _parse_task_safe_half(description)
    if ps is not None and not math.isclose(ps, target_safe_range, rel_tol=0.0, abs_tol=1e-6):
        raise RuntimeError(
        )
    pe = _parse_task_episode_steps(description)
    if pe is not None and pe != target_max_steps:
        raise RuntimeError(
        )
    pcm = _parse_task_cart_mass(description)
    if pcm is not None and not math.isclose(pcm, target_cart_mass, rel_tol=0.0, abs_tol=1e-6):
        raise RuntimeError(
        )
    ppm = _parse_task_pole_mass(description)
    if ppm is not None and not math.isclose(ppm, target_pole_mass, rel_tol=0.0, abs_tol=1e-6):
        raise RuntimeError(
        )
    plen = _parse_task_pole_length_m(description)
    if plen is not None and not math.isclose(plen, target_pole_length, rel_tol=0.0, abs_tol=1e-6):
        raise RuntimeError(
        )
    pact = _parse_task_actuator_limit_n(description)
    if pact is not None and not math.isclose(pact, target_cart_force_limit, rel_tol=0.0, abs_tol=1e-6):
        raise RuntimeError(
        )
    psda = _parse_task_sensor_angle_delay(description)
    if psda is not None and psda != target_sensor_delay_angle:
        raise RuntimeError(
        )
    psdw = _parse_task_sensor_omega_delay(description)
    if psdw is not None and psdw != target_sensor_delay_omega:
        raise RuntimeError(
        )
    pry = _parse_task_rail_y(description)
    if pry is not None and not math.isclose(pry, target_cart_rail_center_y, rel_tol=0.0, abs_tol=1e-6):
        raise RuntimeError(
        )
    _require_task_description_parses(description)

def _require_task_description_parses(description: str) -> None:
    checks = [
        (_parse_task_center_x(description), "track center (center x=…m)"),
        (_parse_task_safe_half(description), "safe half-range (safe range ±…m inclusive)"),
        (_parse_task_episode_steps(description), "episode length line"),
        (_parse_task_cart_mass(description), "cart mass line"),
        (_parse_task_pole_mass(description), "pole mass line"),
        (_parse_task_pole_length_m(description), "pole length line"),
        (_parse_task_actuator_limit_n(description), "actuator limit line"),
        (_parse_task_sensor_angle_delay(description), "sensor angle delay line"),
        (_parse_task_sensor_omega_delay(description), "sensor omega delay line"),
        (_parse_task_rail_y(description), "cart rail y (horizontal track at y=…m)"),
    ]
    for val, label in checks:
        if val is None:
            raise RuntimeError(f"C-01 prompt sync: could not parse {label} from task_description.")

def _parse_success_actuator_limit_n(description: str) -> Optional[float]:
    m = re.search(r"- \*\*Actuator\*\*: Cart force must not exceed ±(\d+(?:\.\d+)?)\s*N", description)
    return float(m.group(1)) if m else None

def _verify_success_criteria_sync(
    description: str,
    target_max_steps: int,
    target_track_center: float,
    target_safe_range: float,
    target_cart_force_limit: float,

) -> None:
    pt = _parse_success_episode_steps(description)
    if pt is not None and pt != target_max_steps:
        raise RuntimeError(
        )
    ptr = _parse_success_track(description)
    if ptr is not None:
        pc, ps = ptr
        if not math.isclose(pc, target_track_center, rel_tol=0.0, abs_tol=1e-6) or not math.isclose(
            ps, target_safe_range, rel_tol=0.0, abs_tol=1e-6
        ):
            raise RuntimeError(
            )
    sa = _parse_success_actuator_limit_n(description)
    if sa is not None and not math.isclose(sa, target_cart_force_limit, rel_tol=0.0, abs_tol=1e-6):
        raise RuntimeError(
        )
    _require_success_criteria_parses(description)

def _require_success_criteria_parses(description: str) -> None:
    if _parse_success_episode_steps(description) is None:
        raise RuntimeError("C-01 prompt sync: could not parse episode length from success_criteria.")
    if _parse_success_track(description) is None:
        raise RuntimeError("C-01 prompt sync: could not parse track limits from success_criteria.")
    if _parse_success_actuator_limit_n(description) is None:
        raise RuntimeError("C-01 prompt sync: could not parse actuator limit from success_criteria.")

def update_task_description_for_visible_changes(
    base_description: str,
    target_terrain_config: Dict[str, Any],
    base_terrain_config: Dict[str, Any],
    target_physics_config: Dict[str, Any] = None,
    base_physics_config: Dict[str, Any] = None,
    *,
    stage: Dict[str, Any] = None,

) -> str:
    description = base_description
    base_physics_config = dict(base_physics_config or {})
    target_physics_config = target_physics_config or {}
    if stage is not None:
        target_physics_config = dict(stage.get("physics_config") or {})
    else:
        target_physics_config = dict(target_physics_config)
    target_track_center = float(target_physics_config.get("track_center_x", _BASE_TRACK_CENTER_X))
    target_max_steps = int(target_physics_config.get("max_steps", _BASE_MAX_STEPS))
    target_cart_mass = float(target_physics_config.get("cart_mass", _BASE_CART_MASS))
    target_pole_mass = float(target_physics_config.get("pole_mass", _BASE_POLE_MASS))
    target_safe_range = float(target_physics_config.get("safe_half_range", _BASE_SAFE_HALF_RANGE))
    target_pole_length = float(target_physics_config.get("pole_length", _BASE_POLE_LENGTH))
    target_pole_start_angle = float(target_physics_config.get("pole_start_angle", _BASE_POLE_START_ANGLE))
    target_sensor_delay_angle = int(target_physics_config.get("sensor_delay_angle_steps", _BASE_SENSOR_DELAY_ANGLE_STEPS))
    target_sensor_delay_omega = int(target_physics_config.get("sensor_delay_omega_steps", _BASE_SENSOR_DELAY_OMEGA_STEPS))
    target_cart_force_limit = float(
        target_physics_config.get("cart_force_limit_newtons", _BASE_CART_FORCE_LIMIT_NEWTONS)
    )
    target_cart_rail_center_y = float(
        target_physics_config.get("cart_rail_center_y", _BASE_CART_RAIL_CENTER_Y)
    )
    target_balance_angle_deg = float(
        target_physics_config.get("balance_angle_deg", _BASE_BALANCE_ANGLE_DEG)
    )
    display_base_pole_start_angle = float(base_physics_config.get("pole_start_angle", _BASE_POLE_START_ANGLE))
    parsed_cx = _parse_task_center_x(description)
    if parsed_cx is not None and not math.isclose(parsed_cx, target_track_center, rel_tol=0.0, abs_tol=1e-6):
        center_pat = re.compile(r"(center x=)(\d+\.?\d*)(m, safe range)")
        if center_pat.search(description):
            description = center_pat.sub(
                rf"center x={_fmt_track_center_m(target_track_center)} (originally {_fmt_track_center_m(_BASE_TRACK_CENTER_X)} in the source environment)\g<3>",
                description,
                count=1,
            )
    parsed_safe = _parse_task_safe_half(description)
    if parsed_safe is not None and not math.isclose(parsed_safe, target_safe_range, rel_tol=0.0, abs_tol=1e-6):
        safe_flex = re.compile(r"(safe range ±)(\d+\.?\d*)(m inclusive)")
        if safe_flex.search(description):
            description = safe_flex.sub(
                f"safe range ±{target_safe_range:g}m inclusive (originally ±{_BASE_SAFE_HALF_RANGE:g}m in the source environment)",
                description,
                count=1,
            )
    parsed_ep = _parse_task_episode_steps(description)
    if parsed_ep is not None and parsed_ep != target_max_steps:
        ep_pat = re.compile(r"- \*\*Episode length\*\*: At most \d+ simulation steps.*")
        if ep_pat.search(description):
            if target_max_steps == _BASE_MAX_STEPS:
                repl = f"- **Episode length**: At most {target_max_steps} simulation steps (must hold balance until the end)."
            else:
                repl = f"- **Episode length**: At most {target_max_steps} simulation steps (originally {parsed_ep} in the source environment, must hold balance until the end)."
            description = ep_pat.sub(repl, description, count=1)
        else:
            ep_fallback = re.compile(r"- \*\*Episode length\*\*: At most \d+ simulation steps.*")
            if ep_fallback.search(description):
                if target_max_steps == _BASE_MAX_STEPS:
                    repl_fb = f"- **Episode length**: At most {target_max_steps} simulation steps (must hold balance until the end)."
                else:
                    repl_fb = f"- **Episode length**: At most {target_max_steps} simulation steps (originally {parsed_ep} in the source environment, must hold balance until the end)."
                description = ep_fallback.sub(repl_fb, description, count=1)
    parsed_ry = _parse_task_rail_y(description)
    if parsed_ry is not None and not math.isclose(
        parsed_ry, target_cart_rail_center_y, rel_tol=0.0, abs_tol=1e-6
    ):
        rail_flex = re.compile(r"(horizontal track at y=)(\d+\.?\d*)(m)")
        if rail_flex.search(description):
            if math.isclose(target_cart_rail_center_y, _BASE_CART_RAIL_CENTER_Y):
                description = rail_flex.sub(rf"\g<1>{target_cart_rail_center_y:g}m", description, count=1)
            else:
                description = rail_flex.sub(
                    rf"\g<1>{target_cart_rail_center_y:g}m (originally {_BASE_CART_RAIL_CENTER_Y:g}m in the source environment)",
                    description,
                    count=1,
                )
    cart_flex = re.compile(r"(mass )(\d+\.?\d*)( kg)")
    cm = cart_flex.search(description)
    if cm and not math.isclose(float(cm.group(2)), target_cart_mass, rel_tol=0.0, abs_tol=1e-6):
        if math.isclose(target_cart_mass, _BASE_CART_MASS):
            description = cart_flex.sub(rf"\g<1>{target_cart_mass:g} kg", description, count=1)
        else:
            description = cart_flex.sub(
                rf"\g<1>{target_cart_mass:g} kg (originally {_BASE_CART_MASS:g} kg in the source environment)",
                description,
                count=1,
            )
    pole_flex = re.compile(r"(Mass )(\d+\.?\d*)( kg, width)")
    pm = pole_flex.search(description)
    if pm and not math.isclose(float(pm.group(2)), target_pole_mass, rel_tol=0.0, abs_tol=1e-6):
        if math.isclose(target_pole_mass, _BASE_POLE_MASS):
            description = pole_flex.sub(rf"\g<1>{target_pole_mass:g} kg", description, count=1)
        else:
            description = pole_flex.sub(
                rf"\g<1>{target_pole_mass:g} kg (originally {_BASE_POLE_MASS:g} kg in the source environment)",
                description,
                count=1,
            )
    len_flex = re.compile(r"(\*\*Length\*\*: )(\d+\.?\d*)(m\.)")
    lm = len_flex.search(description)
    if lm and not math.isclose(float(lm.group(2)), target_pole_length, rel_tol=0.0, abs_tol=1e-6):
        if math.isclose(target_pole_length, _BASE_POLE_LENGTH):
            description = len_flex.sub(rf"\g<1>{target_pole_length:.1f}m.", description, count=1)
        else:
            description = len_flex.sub(
                rf"\g<1>{target_pole_length:.1f}m (originally {_BASE_POLE_LENGTH:.1f}m in the source environment).",
                description,
                count=1,
            )
    if _scalar_physics_differs(target_pole_start_angle, display_base_pole_start_angle):
        ang_deg_new = math.degrees(target_pole_start_angle)
        upright_pat = r"Initially upright \(angle = 0° or 0rad\)\."
        mutated_ang_pat = re.compile(r"(Initially \(angle = )(\d+\.?\d*)(° or )(\d+\.?\d*)(rad\)\.)")
        if abs(display_base_pole_start_angle) < 1e-5 and re.search(upright_pat, description):
            replacement = f"Initially (angle = {ang_deg_new}° or {ang_deg_new}rad) (originally 0° or 0rad in the source environment)."
            description = re.sub(upright_pat, replacement, description, count=1)
        else:
            am = mutated_ang_pat.search(description)
            if am and math.isclose(float(am.group(2)), float(display_base_pole_start_angle), rel_tol=0.0, abs_tol=1e-5):
                replacement = f"{am.group(1)}{ang_deg_new}{am.group(3)}{ang_deg_new}{am.group(5)}"
                description = mutated_ang_pat.sub(replacement, description, count=1)
    sd_ang_pat = re.compile(r"- \*\*Sensor reporting \(angle\)\*\*: (\d+) simulation steps of delay from true state[^.]*\.")
    sam = sd_ang_pat.search(description)
    if sam and int(sam.group(1)) != target_sensor_delay_angle:
        old_ang = int(sam.group(1))
        description = sd_ang_pat.sub(
            _task_sensor_delay_angle_line(target_sensor_delay_angle, old_ang),
            description,
            count=1,
        )
    sd_om_pat = re.compile(r"- \*\*Sensor reporting \(angular velocity\)\*\*: (\d+) simulation steps of delay from true state[^.]*\.")
    som = sd_om_pat.search(description)
    if som and int(som.group(1)) != target_sensor_delay_omega:
        old_om = int(som.group(1))
        description = sd_om_pat.sub(
            _task_sensor_delay_omega_line(target_sensor_delay_omega, old_om),
            description,
            count=1,
        )
    act_task = re.compile(r"(cart force is limited to ±)(\d+\.?\d*)(N)")
    am_act = act_task.search(description)
    if am_act:
        cur_fl = float(am_act.group(2))
        if not math.isclose(cur_fl, target_cart_force_limit, rel_tol=0.0, abs_tol=1e-6):
            fn = int(target_cart_force_limit) if float(target_cart_force_limit).is_integer() else target_cart_force_limit
            ob = int(_BASE_CART_FORCE_LIMIT_NEWTONS) if float(_BASE_CART_FORCE_LIMIT_NEWTONS).is_integer() else _BASE_CART_FORCE_LIMIT_NEWTONS
            if math.isclose(target_cart_force_limit, _BASE_CART_FORCE_LIMIT_NEWTONS):
                description = act_task.sub(rf"\g<1>{fn}N.", description, count=1)
            else:
                description = act_task.sub(
                    rf"\g<1>{fn}N. (originally {ob}N in the source environment)",
                    description,
                    count=1,
                )
    if not math.isclose(target_balance_angle_deg, _BASE_BALANCE_ANGLE_DEG):
        bal_d = int(target_balance_angle_deg) if float(target_balance_angle_deg).is_integer() else target_balance_angle_deg
        ob_d = int(_BASE_BALANCE_ANGLE_DEG) if float(_BASE_BALANCE_ANGLE_DEG).is_integer() else _BASE_BALANCE_ANGLE_DEG

        bal_pat = re.compile(r"\|angle\| <= (\d+(?:\.\d+)?)°")
        description = bal_pat.sub(
            f"|angle| <= {bal_d}° (originally {ob_d}° in the source environment)",
            description,
        )

        grad_pat = re.compile(r"\((\d+(?:\.\d+)?)°, 90°, lock-in count")
        description = grad_pat.sub(
            f"({bal_d}° (originally {ob_d}° in source env), 90°, lock-in count",
            description,
        )
    _verify_task_description_sync(
        description,
        target_track_center,
        target_safe_range,
        target_max_steps,
        target_cart_mass,
        target_pole_mass,
        target_pole_length,
        target_cart_force_limit,
        target_sensor_delay_angle,
        target_sensor_delay_omega,
        target_cart_rail_center_y,
    )
    return description

def update_success_criteria_for_visible_changes(
    base_success_criteria: str,
    target_terrain_config: Dict[str, Any],
    base_terrain_config: Dict[str, Any],
    target_physics_config: Dict[str, Any] = None,
    base_physics_config: Dict[str, Any] = None,
    *,
    stage: Dict[str, Any] = None,

) -> str:
    description = base_success_criteria
    base_physics_config = dict(base_physics_config or {})
    target_physics_config = target_physics_config or {}
    if stage is not None:
        target_physics_config = dict(stage.get("physics_config") or {})
    else:
        target_physics_config = dict(target_physics_config)
    target_max_steps = int(target_physics_config.get("max_steps", _BASE_MAX_STEPS))
    target_track_center = float(target_physics_config.get("track_center_x", _BASE_TRACK_CENTER_X))
    target_safe_range = float(target_physics_config.get("safe_half_range", _BASE_SAFE_HALF_RANGE))
    target_sensor_delay_angle = int(target_physics_config.get("sensor_delay_angle_steps", _BASE_SENSOR_DELAY_ANGLE_STEPS))
    target_sensor_delay_omega = int(target_physics_config.get("sensor_delay_omega_steps", _BASE_SENSOR_DELAY_OMEGA_STEPS))
    target_cart_force_limit = float(
        target_physics_config.get("cart_force_limit_newtons", _BASE_CART_FORCE_LIMIT_NEWTONS)
    )
    target_balance_angle_deg_sc = float(
        target_physics_config.get("balance_angle_deg", _BASE_BALANCE_ANGLE_DEG)
    )
    display_base_steps = int(base_physics_config.get("max_steps", _BASE_MAX_STEPS))
    display_base_center = float(base_physics_config.get("track_center_x", _BASE_TRACK_CENTER_X))
    display_base_safe = float(base_physics_config.get("safe_half_range", _BASE_SAFE_HALF_RANGE))
    display_base_sensor_delay_angle = int(base_physics_config.get("sensor_delay_angle_steps", _BASE_SENSOR_DELAY_ANGLE_STEPS))
    display_base_sensor_delay_omega = int(base_physics_config.get("sensor_delay_omega_steps", _BASE_SENSOR_DELAY_OMEGA_STEPS))
    parsed_sc_steps = _parse_success_episode_steps(description)
    if parsed_sc_steps is not None and parsed_sc_steps != target_max_steps:
        sc_ep_pat = re.compile(r"3\. \*\*Episode length\*\*: At most \d+ simulation steps\.?")
        if sc_ep_pat.search(description):
            description = sc_ep_pat.sub(
                lambda m: (
                    f"3. **Episode length**: At most {target_max_steps} simulation steps"
                    if target_max_steps == _BASE_MAX_STEPS
                    else (
                        f"3. **Episode length**: At most {target_max_steps} simulation steps "
                        f"(originally {display_base_steps} in the source environment)"
                    )
                ),
                description,
                count=1,
            )
    elif target_max_steps != display_base_steps:
        sc_ep_pat = re.compile(r"3\. \*\*Episode length\*\*: At most \d+ simulation steps\.?")
        if sc_ep_pat.search(description):
            if target_max_steps == _BASE_MAX_STEPS:
                description = sc_ep_pat.sub(
                    lambda m: f"3. **Episode length**: At most {target_max_steps} simulation steps",
                    description,
                    count=1,
                )
            else:
                description = sc_ep_pat.sub(
                    lambda m: (
                        f"3. **Episode length**: At most {target_max_steps} simulation steps "
                        f"(originally {display_base_steps} in the source environment)"
                    ),
                    description,
                    count=1,
                )
    parsed_tr = _parse_success_track(description)
    if parsed_tr is not None:
        pc, ps = parsed_tr
        if not math.isclose(pc, target_track_center, rel_tol=0.0, abs_tol=1e-6) or not math.isclose(
            ps, target_safe_range, rel_tol=0.0, abs_tol=1e-6
        ):
            track_any = re.compile(r"2\. \*\*Track Limits\*\*: Cart remains within the safe zone .*")
            if track_any.search(description):
                description = track_any.sub(
                    _success_track_limits_line(target_track_center, target_safe_range),
                    description,
                    count=1,
                )
    sc_act = re.compile(r"(\*\*Actuator\*\*: Cart force must not exceed ±)(\d+(?:\.\d+)?)\s*N")
    am_sca = sc_act.search(description)
    if am_sca:
        cur_fl = float(am_sca.group(2))
        if not math.isclose(cur_fl, target_cart_force_limit, rel_tol=0.0, abs_tol=1e-6):
            fn = int(target_cart_force_limit) if float(target_cart_force_limit).is_integer() else target_cart_force_limit
            ob = int(_BASE_CART_FORCE_LIMIT_NEWTONS) if float(_BASE_CART_FORCE_LIMIT_NEWTONS).is_integer() else _BASE_CART_FORCE_LIMIT_NEWTONS
            if math.isclose(target_cart_force_limit, _BASE_CART_FORCE_LIMIT_NEWTONS):
                description = sc_act.sub(rf"\g<1>{fn}N.", description, count=1)
            else:
                description = sc_act.sub(
                    rf"\g<1>{fn}N. (originally {ob}N in the source environment)",
                    description,
                    count=1,
                )
    if not math.isclose(target_balance_angle_deg_sc, _BASE_BALANCE_ANGLE_DEG):
        bal_d = int(target_balance_angle_deg_sc) if float(target_balance_angle_deg_sc).is_integer() else target_balance_angle_deg_sc
        ob_d = int(_BASE_BALANCE_ANGLE_DEG) if float(_BASE_BALANCE_ANGLE_DEG).is_integer() else _BASE_BALANCE_ANGLE_DEG

        bal_sc_pat = re.compile(r"\|angle\| <= (\d+(?:\.\d+)?)°")
        description = bal_sc_pat.sub(
            f"|angle| <= {bal_d}° (originally {ob_d}° in the source environment)",
            description,
        )
    _verify_success_criteria_sync(
        description, target_max_steps, target_track_center, target_safe_range, target_cart_force_limit
    )
    return description

def _visible_physics_config(
    target_physics_config: Optional[Dict[str, Any]],
    stage: Optional[Dict[str, Any]],
) -> Dict[str, Any]:
    """Return target settings without exposing hidden dynamics in prompt text."""
    if stage is not None:
        return dict(stage.get("physics_config") or {})
    return dict(target_physics_config or {})


def _replace_once(text: str, pattern: str, replacement: str, label: str) -> str:
    updated, count = re.subn(pattern, replacement, text, count=1)
    if count != 1:
        raise RuntimeError(f"C-01 prompt sync: expected exactly one {label}, found {count}.")
    return updated


# These later definitions intentionally supersede the legacy broad synchronizers above.
# Grading/build constraints and directly visible geometry/state are synchronized;
# masses, gravity, and sensor latency remain undisclosed.
def update_task_description_for_visible_changes(
    base_description: str,
    target_terrain_config: Dict[str, Any],
    base_terrain_config: Dict[str, Any],
    target_physics_config: Optional[Dict[str, Any]] = None,
    base_physics_config: Optional[Dict[str, Any]] = None,
    *,
    stage: Optional[Dict[str, Any]] = None,
) -> str:
    del target_terrain_config, base_terrain_config, base_physics_config
    description = base_description
    config = _visible_physics_config(target_physics_config, stage)

    track_center = float(config.get("track_center_x", _BASE_TRACK_CENTER_X))
    safe_half = float(config.get("safe_half_range", _BASE_SAFE_HALF_RANGE))
    rail_y = float(config.get("cart_rail_center_y", _BASE_CART_RAIL_CENTER_Y))
    max_steps = int(config.get("max_steps", _BASE_MAX_STEPS))
    force_limit = float(config.get("cart_force_limit_newtons", _BASE_CART_FORCE_LIMIT_NEWTONS))
    balance_deg = float(config.get("balance_angle_deg", _BASE_BALANCE_ANGLE_DEG))
    pole_length = float(config.get("pole_length", _BASE_POLE_LENGTH))
    pole_start_angle = float(config.get("pole_start_angle", _BASE_POLE_START_ANGLE))

    if not math.isclose(track_center, _BASE_TRACK_CENTER_X):
        description = _replace_once(
            description,
            rf"center x={_BASE_TRACK_CENTER_X:g}m",
            f"center x={track_center:g}m (source center {_BASE_TRACK_CENTER_X:g}m)",
            "track center",
        )
    if not math.isclose(safe_half, _BASE_SAFE_HALF_RANGE):
        description = _replace_once(
            description,
            rf"safe range ±{_BASE_SAFE_HALF_RANGE:g}m inclusive",
            f"safe range ±{safe_half:g}m inclusive (source half-range ±{_BASE_SAFE_HALF_RANGE:g}m)",
            "safe half-range",
        )
    if not math.isclose(rail_y, _BASE_CART_RAIL_CENTER_Y):
        description = _replace_once(
            description,
            rf"horizontal track at y={_BASE_CART_RAIL_CENTER_Y:g}(?:\.0+)?m",
            f"horizontal track at y={rail_y:g}m (source y={_BASE_CART_RAIL_CENTER_Y:g}m)",
            "rail height",
        )
    if not math.isclose(pole_length, _BASE_POLE_LENGTH):
        description = _replace_once(
            description,
            rf"visible length {_BASE_POLE_LENGTH:g}(?:\.0+)?m",
            (
                f"visible length {pole_length:g}m "
                f"(originally {_BASE_POLE_LENGTH:g}m in the source environment)"
            ),
            "pole length",
        )
    if not math.isclose(pole_start_angle, _BASE_POLE_START_ANGLE):
        angle_degrees = math.degrees(pole_start_angle)
        description = _replace_once(
            description,
            r"starts upright \(angle = 0° or 0rad\)",
            (
                f"starts at angle {angle_degrees:g}° ({pole_start_angle:g} rad) "
                "(originally upright at 0°/0 rad in the source environment)"
            ),
            "pole initial angle",
        )
    if max_steps != _BASE_MAX_STEPS:
        description = _replace_once(
            description,
            rf"At most {_BASE_MAX_STEPS} simulation steps",
            f"At most {max_steps} simulation steps (source limit {_BASE_MAX_STEPS})",
            "episode length",
        )
    if not math.isclose(force_limit, _BASE_CART_FORCE_LIMIT_NEWTONS):
        description = _replace_once(
            description,
            rf"limited to ±{_BASE_CART_FORCE_LIMIT_NEWTONS:g}N",
            f"limited to ±{force_limit:g}N (source limit ±{_BASE_CART_FORCE_LIMIT_NEWTONS:g}N)",
            "actuator limit",
        )
    if not math.isclose(balance_deg, _BASE_BALANCE_ANGLE_DEG):
        description = re.sub(
            rf"\|angle\| <= {_BASE_BALANCE_ANGLE_DEG:g}°",
            f"|angle| <= {balance_deg:g}° (source threshold {_BASE_BALANCE_ANGLE_DEG:g}°)",
            description,
        )
    return description


def update_success_criteria_for_visible_changes(
    base_success_criteria: str,
    target_terrain_config: Dict[str, Any],
    base_terrain_config: Dict[str, Any],
    target_physics_config: Optional[Dict[str, Any]] = None,
    base_physics_config: Optional[Dict[str, Any]] = None,
    *,
    stage: Optional[Dict[str, Any]] = None,
) -> str:
    del target_terrain_config, base_terrain_config, base_physics_config
    criteria = base_success_criteria
    config = _visible_physics_config(target_physics_config, stage)

    track_center = float(config.get("track_center_x", _BASE_TRACK_CENTER_X))
    safe_half = float(config.get("safe_half_range", _BASE_SAFE_HALF_RANGE))
    max_steps = int(config.get("max_steps", _BASE_MAX_STEPS))
    force_limit = float(config.get("cart_force_limit_newtons", _BASE_CART_FORCE_LIMIT_NEWTONS))
    balance_deg = float(config.get("balance_angle_deg", _BASE_BALANCE_ANGLE_DEG))

    if not math.isclose(track_center, _BASE_TRACK_CENTER_X) or not math.isclose(
        safe_half, _BASE_SAFE_HALF_RANGE
    ):
        criteria = _replace_once(
            criteria,
            rf"\|x - {_BASE_TRACK_CENTER_X:g}\| ≤ {_BASE_SAFE_HALF_RANGE:g}m",
            (
                f"|x - {track_center:g}| ≤ {safe_half:g}m "
                f"(source zone |x - {_BASE_TRACK_CENTER_X:g}| ≤ {_BASE_SAFE_HALF_RANGE:g}m)"
            ),
            "success track limits",
        )
    if max_steps != _BASE_MAX_STEPS:
        criteria = _replace_once(
            criteria,
            rf"At most {_BASE_MAX_STEPS} simulation steps",
            f"At most {max_steps} simulation steps (source limit {_BASE_MAX_STEPS})",
            "success episode length",
        )
    if not math.isclose(force_limit, _BASE_CART_FORCE_LIMIT_NEWTONS):
        criteria = _replace_once(
            criteria,
            rf"must not exceed ±{_BASE_CART_FORCE_LIMIT_NEWTONS:g}N",
            f"must not exceed ±{force_limit:g}N (source limit ±{_BASE_CART_FORCE_LIMIT_NEWTONS:g}N)",
            "success actuator limit",
        )
    if not math.isclose(balance_deg, _BASE_BALANCE_ANGLE_DEG):
        criteria = re.sub(
            rf"\|angle\| <= {_BASE_BALANCE_ANGLE_DEG:g}°",
            f"|angle| <= {balance_deg:g}° (source threshold {_BASE_BALANCE_ANGLE_DEG:g}°)",
            criteria,
        )
    return criteria


UNIFORM_SUFFIX = uniform_suffix_for_task("C_01")

def curriculum_stages() -> List[Dict[str, Any]]:
    return [
        {
            "stage_id": "Stage-1",
            "title": "Curriculum stage 1",
            "task_description_suffix": uniform_suffix_for_task("C_01"),
            "physics_config": {
                "track_center_x": 50.0,
                "pole_start_angle": 0.16580627893946121,
                "sensor_delay_angle_steps": 10,
                "sensor_delay_omega_steps": 10,
            },
        },
        {
            "stage_id": "Stage-2",
            "title": "Curriculum stage 2",
            "task_description_suffix": uniform_suffix_for_task("C_01"),
            "physics_config": {
                "track_center_x": 50.0,
                "cart_force_limit_newtons": 0.5,
                "max_steps": 2000,
                "gravity": 12.0,
                "sensor_delay_angle_steps": 10,
                "sensor_delay_omega_steps": 10,
                "pole_start_angle": 0.003,
                "safe_half_range": 0.10,
            },
        },
        {
            "stage_id": "Stage-3",
            "title": "Curriculum stage 3",
            "task_description_suffix": uniform_suffix_for_task("C_01"),
            "physics_config": {
                "track_center_x": 50.0,
                "gravity": 22.0,
                "sensor_delay_angle_steps": 3,
                "sensor_delay_omega_steps": 3,
                "cart_force_limit_newtons": 4.0,
                "pole_mass": 5.0,
                "cart_mass": 3.0,
                "safe_half_range": 0.2,
                "pole_length": 0.7,
                "max_steps": 1000,
                "pole_start_angle": 0.0004,
            },
        },
        {
            "stage_id": "Stage-4",
            "title": "Curriculum stage 4",
            "task_description_suffix": uniform_suffix_for_task("C_01"),
            "physics_config": {
                "track_center_x": 50.0,
                "gravity": 36.0,
                "sensor_delay_angle_steps": 3,
                "sensor_delay_omega_steps": 3,
                "cart_force_limit_newtons": 50.0,
                "pole_mass": 12.5,
                "cart_mass": 1.95,
                "safe_half_range": 0.145,
                "pole_length": 0.5,
                "max_steps": 480,
                "pole_start_angle": 0.00006,
                "cart_rail_center_y": 6.0,
            },
        },
    ]

def get_stages():
    curriculum = curriculum_stages()
    result = []
    for s in curriculum:
        pid = s["stage_id"]
        num = pid.split("-")[1]
        result.append({
            "name": pid,
            "description": s.get("title", pid),
            "build_fn": f"build_agent_stage_{num}",
            "action_fn": f"agent_action_stage_{num}",
            "config_overrides": s.get("physics_config", {}),
            "terrain_config": s.get("terrain_config", {}) or {},
            "task_description_suffix": s.get("task_description_suffix", "") or "",
        })
    return result
