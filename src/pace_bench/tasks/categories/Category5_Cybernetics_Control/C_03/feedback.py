import math
from typing import Any, Dict, List, Optional


def _number(metrics: Dict[str, Any], key: str) -> Optional[float]:
    value = metrics.get(key)
    if value is None:
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _integer(metrics: Dict[str, Any], key: str) -> Optional[int]:
    number = _number(metrics, key)
    return None if number is None else int(number)


def _shown(value: Optional[float], digits: int = 3) -> str:
    return "unavailable" if value is None else f"{value:.{digits}f}"


def _status(metrics: Dict[str, Any], key: str) -> str:
    if key not in metrics:
        return "unavailable"
    return "YES" if bool(metrics.get(key)) else "NO"


def format_task_metrics(metrics: Dict[str, Any]) -> List[str]:
    if not isinstance(metrics, dict):
        return ["**Diagnostics unavailable**: evaluator metrics were not a mapping."]
    if metrics.get("error"):
        return [f"**Evaluation error**: {metrics['error']}"]

    success = bool(metrics.get("success", False))
    failed = bool(metrics.get("failed", False))
    outcome = "SUCCESS" if success else ("FAILED" if failed else "IN PROGRESS")
    step = _integer(metrics, "step_count")
    max_steps = _integer(metrics, "max_steps")
    step_text = "unavailable" if step is None else str(step)
    if max_steps is not None:
        step_text += f"/{max_steps}"
    headline = f"**{outcome}** at step {step_text}"
    if failed and metrics.get("failure_reason"):
        headline += f" — {metrics['failure_reason']}"
    parts = [headline]

    finite_keys = (
        "seeker_x",
        "seeker_y",
        "seeker_vx",
        "seeker_vy",
        "distance_to_target",
        "relative_speed",
    )
    invalid = [
        key
        for key in finite_keys
        if key in metrics and _number(metrics, key) is None
    ]
    if bool(metrics.get("numerical_nonfinite_detected", False)) or invalid:
        detail = ", ".join(invalid) if invalid else "tracked state"
        parts.append(f"Numerics: NON-FINITE ({detail}).")
        return parts

    sx = _number(metrics, "seeker_x")
    sy = _number(metrics, "seeker_y")
    vx = _number(metrics, "seeker_vx")
    vy = _number(metrics, "seeker_vy")
    if None not in (sx, sy, vx, vy):
        parts.append(
            f"State: p=({sx:.3f},{sy:.3f}) m; v=({vx:+.3f},{vy:+.3f}) m/s."
        )

    count = _integer(metrics, "rendezvous_count")
    required = _integer(metrics, "required_rendezvous_count")
    capture_fields = []
    if count is not None and required is not None:
        capture_fields.append(f"rendezvous={count}/{required}")
    distance = _number(metrics, "distance_to_target")
    distance_limit = _number(metrics, "rendezvous_distance")
    if distance is not None and distance_limit is not None:
        capture_fields.append(
            f"d={distance:.3f}/≤{distance_limit:.3f} m "
            f"(margin {distance_limit - distance:+.3f})"
        )
    relative_speed = _number(metrics, "relative_speed")
    relative_limit = _number(metrics, "rendezvous_rel_speed")
    if relative_speed is not None and relative_limit is not None:
        capture_fields.append(
            f"rel-v={relative_speed:.3f}/<{relative_limit:.3f} m/s "
            f"(margin {relative_limit - relative_speed:+.3f})"
        )
    heading_error = _number(metrics, "heading_error_deg")
    heading_limit = _number(metrics, "heading_tolerance_deg")
    if heading_error is not None and heading_limit is not None:
        capture_fields.append(
            f"heading={heading_error:.2f}/≤{heading_limit:.2f}° "
            f"(margin {heading_limit - heading_error:+.2f})"
        )
    rz_lo = _number(metrics, "rendezvous_zone_x_min")
    rz_hi = _number(metrics, "rendezvous_zone_x_max")
    if sx is not None and rz_lo is not None and rz_hi is not None:
        capture_fields.append(
            f"zone-x={sx:.3f} in [{rz_lo:.3f},{rz_hi:.3f}] "
            f"(margin {min(sx - rz_lo, rz_hi - sx):+.3f} m)"
        )
    if capture_fields:
        parts.append(
            "Capture now (conditions count only in slots): "
            + "; ".join(capture_fields)
            + "."
        )

    activation = metrics.get("activation_achieved")
    activation_text = (
        "unavailable"
        if activation is None
        else ("ACHIEVED" if bool(activation) else "NOT ACHIEVED")
    )
    current_streak = _integer(metrics, "activation_current_consecutive_steps")
    best_streak = _integer(metrics, "activation_max_consecutive_steps")
    required_streak = _integer(metrics, "activation_required_steps")
    achieved_step = _integer(metrics, "activation_achieved_step")
    parts.append(
        f"Activation: {activation_text}; streak current/best/required="
        f"{current_streak if current_streak is not None else 'unavailable'}/"
        f"{best_streak if best_streak is not None else 'unavailable'}/"
        f"{required_streak if required_streak is not None else 'unavailable'}"
        + (
            f"; achieved step={achieved_step}."
            if achieved_step is not None
            else "."
        )
    )

    slot_samples = []
    for phase in (1, 2):
        distance_key = f"phase{phase}_best_distance"
        if distance_key not in metrics:
            slot_samples.append(f"P{phase}=unavailable")
            continue
        best_distance = _number(metrics, distance_key)
        if best_distance is None:
            slot_samples.append(f"P{phase}=no sample")
            continue
        best_step = _integer(metrics, f"phase{phase}_best_step")
        best_relative = _number(metrics, f"phase{phase}_best_relative_speed")
        best_heading = _number(metrics, f"phase{phase}_best_heading_error_deg")
        slot_samples.append(
            f"P{phase}@{best_step if best_step is not None else '?'}: "
            f"d={best_distance:.3f} m, rel-v={_shown(best_relative)} m/s, "
            f"heading={_shown(best_heading, 2)}°"
        )
    parts.append("Closest slot samples: " + "; ".join(slot_samples) + ".")

    impulse_used = _number(metrics, "impulse_used")
    impulse_budget = _number(metrics, "impulse_budget")
    thrust = _number(metrics, "last_applied_thrust_magnitude")
    normal_cap = _number(metrics, "max_thrust_magnitude")
    cooldown_cap = _number(metrics, "cooldown_max_thrust")
    cooldown_left = _integer(metrics, "cooldown_remaining_steps")
    control_fields = []
    if impulse_used is not None and impulse_budget is not None:
        control_fields.append(
            f"impulse={impulse_used:.2f}/{impulse_budget:.2f} N·s "
            f"(margin {impulse_budget - impulse_used:+.2f}, out={_status(metrics, 'out_of_fuel')})"
        )
    if thrust is not None and normal_cap is not None:
        active_cap = (
            cooldown_cap
            if cooldown_left is not None
            and cooldown_left > 0
            and cooldown_cap is not None
            else normal_cap
        )
        control_fields.append(
            f"thrust={thrust:.2f}/{active_cap:.2f} N active cap; "
            f"cooldown={cooldown_left if cooldown_left is not None else 'unavailable'} steps"
        )
    if control_fields:
        parts.append("Control constraints: " + "; ".join(control_fields) + ".")

    c_lo = _number(metrics, "corridor_x_lo")
    c_hi = _number(metrics, "corridor_x_hi")
    corridor_tolerance = _number(metrics, "corridor_violation_tolerance")
    safety_fields = []
    if sx is not None and c_lo is not None and c_hi is not None:
        safety_fields.append(
            f"corridor=[{c_lo:.3f},{c_hi:.3f}] m, "
            f"margin={min(sx - c_lo, c_hi - sx):+.3f} m, "
            f"allowance={_shown(corridor_tolerance)} m, "
            f"violation={_status(metrics, 'corridor_violation')}"
        )
    obstacle_limit = _number(metrics, "obstacle_penetration_limit")
    safety_fields.append(
        f"obstacle collision={_status(metrics, 'obstacle_collision')}, "
        f"limit={_shown(obstacle_limit)} m penetration"
    )
    parts.append("Safety: " + "; ".join(safety_fields) + ".")

    track_limit = _number(metrics, "track_distance")
    peak_track = _number(metrics, "post_rendezvous_peak_distance")
    peak_track_step = _integer(metrics, "post_rendezvous_peak_distance_step")
    if track_limit is not None and peak_track is not None:
        parts.append(
            f"Tracking: peak d={peak_track:.3f} m at step "
            f"{peak_track_step if peak_track_step is not None else 'unavailable'}; "
            f"limit={track_limit:.3f} m; margin={track_limit - peak_track:+.3f} m."
        )

    chronology = []
    rendezvous_steps = metrics.get("rendezvous_steps")
    if isinstance(rendezvous_steps, list):
        chronology.append(
            "rendezvous steps="
            + (",".join(str(value) for value in rendezvous_steps[:2]) or "none")
        )
    else:
        chronology.append("rendezvous steps=unavailable")
    events = metrics.get("failure_events")
    if isinstance(events, list) and events:
        event_chunks = []
        for event in events[-4:]:
            if not isinstance(event, dict):
                continue
            if event.get("type") == "activation_achieved":
                continue
            chunk = f"{event.get('step', '?')}:{event.get('type', 'event')}"
            if event.get("detail"):
                chunk += f" ({event['detail']})"
            event_chunks.append(chunk)
        chronology.append(
            "events=" + ("; ".join(event_chunks) or "none beyond activation")
        )
    elif isinstance(events, list):
        chronology.append("events=none")
    else:
        chronology.append("events=unavailable")
    parts.append("Timeline: " + "; ".join(chronology) + ".")

    peak_speed = _number(metrics, "peak_seeker_speed")
    peak_acceleration = _number(metrics, "peak_acceleration")
    if peak_speed is not None and peak_acceleration is not None:
        parts.append(
            f"Numerics: finite; peak speed={peak_speed:.3f} m/s; "
            f"peak acceleration={peak_acceleration:.3f} m/s²."
        )
    elif all(key in metrics and _number(metrics, key) is not None for key in finite_keys):
        parts.append("Numerics: finite; peak-motion data unavailable.")
    else:
        parts.append("Numerics: unavailable.")
    return parts


def get_improvement_suggestions(
    metrics: Dict[str, Any],
    score: float,
    success: bool,
    failed: bool,
    failure_reason: Optional[str],
    error: Optional[str],
) -> List[str]:
    del metrics, score, success, failed, failure_reason, error
    return []
