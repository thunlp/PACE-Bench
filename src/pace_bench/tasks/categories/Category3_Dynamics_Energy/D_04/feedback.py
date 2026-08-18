"""Deterministic, non-leaking feedback for D-04."""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional


def _number(value: Any) -> Optional[float]:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _fmt(value: Any, digits: int = 3) -> str:
    result = _number(value)
    return "unavailable" if result is None else f"{result:.{digits}f}"


def _outcome(metrics: Dict[str, Any]) -> str:
    step = metrics.get("step_count", "unavailable")
    maximum = metrics.get("max_steps")
    step_text = f"{step}/{maximum}" if maximum is not None else str(step)
    if metrics.get("success"):
        return f"**Outcome**: SUCCESS at step {step_text}."
    if metrics.get("failed"):
        reason = metrics.get("failure_reason") or "failure reason unavailable"
        return f"**Outcome**: FAILURE at step {step_text} — {reason}."
    return f"**Outcome**: IN PROGRESS at step {step_text}."


def format_task_metrics(metrics: Dict[str, Any]) -> List[str]:
    if not isinstance(metrics, dict) or not metrics:
        return ["**Outcome**: unavailable — no metrics were provided."]
    lines = [
        "## D-04 Swing Feedback",
        _outcome(metrics),
        "### Observed motion",
        f"- Seat position: ({_fmt(metrics.get('seat_x'))}, {_fmt(metrics.get('seat_y'))}) m.",
        f"- Seat velocity: ({_fmt(metrics.get('seat_vx'))}, {_fmt(metrics.get('seat_vy'))}) m/s; "
        f"speed {_fmt(metrics.get('seat_speed'))} m/s.",
        f"- Peak observed height: {_fmt(metrics.get('max_seat_y_reached'))} m; "
        f"target y≥{_fmt(metrics.get('target_y_min'))} m and x in "
        f"[{_fmt(metrics.get('target_x_min'))}, {_fmt(metrics.get('target_x_max'))}] m.",
        f"- Height progress: {_fmt(metrics.get('progress_pct'), 1)}%; "
        f"vertical target margin {_fmt(metrics.get('vertical_margin_to_target'))} m.",
        "### Apex observations",
    ]
    events = metrics.get("apex_events")
    if isinstance(events, list) and events:
        for event in events:
            if isinstance(event, (list, tuple)) and len(event) >= 4:
                lines.append(
                    f"- Step {event[0]}: x={_fmt(event[1])} m, y={_fmt(event[2])} m, "
                    f"speed={_fmt(event[3])} m/s."
                )
    else:
        lines.append("- No apex event was recorded.")
    lines.extend(
        [
            "### Actuator observations",
            f"- Calls: {metrics.get('force_calls', 'unavailable')}; applied "
            f"{metrics.get('force_applied_count', 'unavailable')}; suppressed "
            f"{metrics.get('force_suppressed_count', 'unavailable')}; force-clamped "
            f"{metrics.get('force_clamped_count', 'unavailable')}; impulse-clamped "
            f"{metrics.get('impulse_clamped_count', 'unavailable')}.",
            f"- Delivery ratio: {_fmt(metrics.get('force_delivery_pct'), 1)}%; "
            f"suppressed in dead zone {metrics.get('force_suppressed_deadzone', 'unavailable')} times and "
            f"by directional fault {metrics.get('force_suppressed_fault', 'unavailable')} times.",
            f"- Force aligned with velocity in {_fmt(metrics.get('phase_alignment_pct'), 1)}% of sampled calls; "
            f"anti-aligned in {_fmt(metrics.get('phase_antialigned_pct'), 1)}%.",
            "### Public constraints",
            f"- Structure mass: {_fmt(metrics.get('structure_mass'))} / "
            f"{_fmt(metrics.get('max_structure_mass'))} kg.",
            f"- Pump-force limit: {_fmt(metrics.get('max_pump_force'))} N per axis; "
            f"impulse limit {_fmt(metrics.get('max_impulse'))} N·s per axis.",
            "### Data health",
        ]
    )
    bad = []
    for key in ("seat_x", "seat_y", "seat_vx", "seat_vy", "seat_speed", "max_seat_y_reached"):
        value = metrics.get(key)
        if value is not None and _number(value) is None:
            bad.append(key)
    if bad:
        lines.append("- Non-finite observations: " + ", ".join(bad) + ".")
    elif metrics.get("extreme_velocity_detected"):
        lines.append("- Extreme velocity was observed; the simulation may be numerically unstable.")
    else:
        lines.append("- No numerical anomaly reported.")
    return lines


def get_improvement_suggestions(
    metrics: Dict[str, Any],
    score: float,
    success: bool,
    failed: bool,
    failure_reason: str = None,
    error: str = None,
) -> List[str]:
    """Suggestions are reserved for code/runtime repair, not solution prescriptions."""
    if error:
        return [f"- Resolve the reported execution error: {error}"]
    return []
