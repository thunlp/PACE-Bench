"""Deterministic feedback for D-05 without hidden-parameter disclosure."""

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
        return f"**Outcome**: SUCCESS at step {step_text}; shell broken."
    if metrics.get("failed"):
        reason = metrics.get("failure_reason") or "failure reason unavailable"
        return f"**Outcome**: FAILURE at step {step_text} — {reason}."
    return f"**Outcome**: IN PROGRESS at step {step_text}."


def format_task_metrics(metrics: Dict[str, Any]) -> List[str]:
    if not isinstance(metrics, dict) or not metrics:
        return ["**Outcome**: unavailable — no metrics were provided."]
    lines = [
        "## D-05 Hammer Feedback",
        _outcome(metrics),
        "### Terminal hammer observation",
        f"- Position: ({_fmt(metrics.get('hammer_x'))}, {_fmt(metrics.get('hammer_y'))}) m.",
        f"- Velocity: ({_fmt(metrics.get('velocity_x'))}, {_fmt(metrics.get('velocity_y'))}) m/s; "
        f"speed {_fmt(metrics.get('speed'))} m/s; angular velocity "
        f"{_fmt(metrics.get('angular_velocity'))} rad/s.",
        f"- Peak observed speed: {_fmt(metrics.get('peak_speed'))} m/s; "
        f"peak agent kinetic energy {_fmt(metrics.get('peak_kinetic_energy'))} J at step "
        f"{metrics.get('peak_ke_step', 'unavailable')}.",
        "### Slot observations",
        f"- Visible gap: y=[{_fmt(metrics.get('slot_gap_y_low'))}, "
        f"{_fmt(metrics.get('slot_gap_y_high'))}] m.",
    ]
    if metrics.get("slot_entry_step") is not None:
        lines.append(
            f"- Entry at step {metrics.get('slot_entry_step')}: hammer y="
            f"{_fmt(metrics.get('slot_entry_hammer_y'))} m; observed bar y="
            f"{_fmt(metrics.get('slot_entry_bar_y'))} m."
        )
    else:
        lines.append("- No slot entry was recorded.")
    lines.append("### Contact chronology")
    events = metrics.get("contact_events")
    if isinstance(events, list) and events:
        for event in events:
            if isinstance(event, dict):
                lines.append(
                    f"- Step {event.get('step', 'unavailable')}: contact with "
                    f"{event.get('obstacle', 'unavailable')} at hammer position "
                    f"({_fmt(event.get('hammer_x'))}, {_fmt(event.get('hammer_y'))}) m."
                )
    else:
        lines.append("- No obstacle contact was recorded.")
    lines.extend(
        [
            "### Public constraints",
            f"- Structure mass: {_fmt(metrics.get('structure_mass'))} / strictly less than "
            f"{_fmt(metrics.get('max_structure_mass'))} kg.",
            f"- Build zone: x=[{_fmt(metrics.get('build_zone_x_min'))}, "
            f"{_fmt(metrics.get('build_zone_x_max'))}], y=[{_fmt(metrics.get('build_zone_y_min'))}, "
            f"{_fmt(metrics.get('build_zone_y_max'))}] m.",
            f"- Maximum observed shell-joint force: {_fmt(metrics.get('max_shell_joint_force'))} N; "
            f"visible break threshold {_fmt(metrics.get('shell_break_force'))} N.",
            "### Data health",
        ]
    )
    bad = []
    for key in ("hammer_x", "hammer_y", "velocity_x", "velocity_y", "speed", "peak_speed"):
        value = metrics.get(key)
        if value is not None and _number(value) is None:
            bad.append(key)
    if bad:
        lines.append("- Non-finite observations: " + ", ".join(bad) + ".")
    errors = metrics.get("observation_errors")
    if isinstance(errors, list) and errors:
        lines.extend(f"- Observation error: {error}" for error in errors)
    if not bad and not (isinstance(errors, list) and errors):
        lines.append("- No numerical or observation errors reported.")
    return lines


def get_improvement_suggestions(
    metrics: Dict[str, Any],
    score: float,
    success: bool,
    failed: bool,
    failure_reason: str = None,
    error: str = None,
) -> List[str]:
    if error:
        return [f"- Resolve the reported execution error: {error}"]
    return []
