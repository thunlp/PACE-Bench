"""Deterministic, provenance-safe feedback for D-02."""

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
    number = _number(value)
    return "unavailable" if number is None else f"{number:.{digits}f}"


def _outcome(metrics: Dict[str, Any]) -> str:
    step = metrics.get("step_count", "unavailable")
    max_steps = metrics.get("max_steps")
    step_text = f"{step}/{max_steps}" if max_steps is not None else str(step)
    if metrics.get("success"):
        return f"**Outcome**: SUCCESS at step {step_text}."
    if metrics.get("failed"):
        reason = metrics.get("failure_reason") or "failure reason unavailable"
        return f"**Outcome**: FAILURE at step {step_text} — {reason}."
    return f"**Outcome**: IN PROGRESS at step {step_text}."


def _slot_lines(metrics: Dict[str, Any]) -> List[str]:
    definitions = metrics.get("slot_definitions")
    approaches = metrics.get("slot_closest_approach")
    if not isinstance(definitions, list) or not definitions:
        return ["- Slot observations: unavailable."]
    if not isinstance(approaches, dict):
        approaches = {}
    lines: List[str] = []
    for definition in definitions:
        if not isinstance(definition, dict):
            continue
        number = definition.get("slot_num", "?")
        x_min = _fmt(definition.get("x_min"), 1)
        x_max = _fmt(definition.get("x_max"), 1)
        floor = _fmt(definition.get("floor_y"), 1)
        ceiling = _fmt(definition.get("ceil_y"), 1)
        approach = approaches.get(f"slot_{number}")
        if not isinstance(approach, dict):
            lines.append(
                f"- Slot {number} (x=[{x_min}, {x_max}], gap=[{floor}, {ceiling}] m): not observed."
            )
            continue
        floor_margin = _number(approach.get("floor_margin"))
        ceiling_margin = _number(approach.get("ceil_margin"))
        if floor_margin is None or ceiling_margin is None:
            status = "margin unavailable"
        elif floor_margin >= 0 and ceiling_margin >= 0:
            status = "clear"
        else:
            status = "collision margin"
        lines.append(
            f"- Slot {number}: {status}; floor margin {_fmt(floor_margin)} m, "
            f"ceiling margin {_fmt(ceiling_margin)} m at step {approach.get('step', 'unavailable')}."
        )
    return lines or ["- Slot observations: unavailable."]


def _health_lines(metrics: Dict[str, Any]) -> List[str]:
    lines: List[str] = []
    nonfinite = []
    for key in ("jumper_x", "jumper_y", "jumper_vx", "jumper_vy", "angular_velocity", "angle"):
        value = metrics.get(key)
        if value is not None and _number(value) is None:
            nonfinite.append(key)
    if nonfinite:
        lines.append("- Non-finite observations: " + ", ".join(nonfinite) + ".")
    errors = metrics.get("observation_errors")
    if isinstance(errors, list) and errors:
        lines.extend(f"- Observation error: {error}" for error in errors)
    if not lines:
        lines.append("- No numerical or observation errors reported.")
    return lines


def format_task_metrics(metrics: Dict[str, Any]) -> List[str]:
    """Format only evaluator-provided evidence; never infer hidden physics."""
    if not isinstance(metrics, dict) or not metrics:
        return ["**Outcome**: unavailable — no metrics were provided."]

    lines = ["## D-02 Jumper Feedback", _outcome(metrics)]
    lines.extend(
        [
            "### Observed terminal state",
            f"- Position: ({_fmt(metrics.get('jumper_x'))}, {_fmt(metrics.get('jumper_y'))}) m.",
            f"- Velocity: ({_fmt(metrics.get('jumper_vx'))}, {_fmt(metrics.get('jumper_vy'))}) m/s; "
            f"speed {_fmt(metrics.get('jumper_speed'))} m/s.",
            f"- Horizontal progress: {_fmt(metrics.get('progress'), 1)}%; "
            f"distance to platform {_fmt(metrics.get('distance_from_platform'))} m.",
            "### Public constraints",
            f"- Structure mass: {_fmt(metrics.get('structure_mass'), 2)} / "
            f"{_fmt(metrics.get('max_structure_mass'), 2)} kg.",
            f"- Build zone: x=[{_fmt(metrics.get('build_zone_x_min'), 1)}, "
            f"{_fmt(metrics.get('build_zone_x_max'), 1)}], "
            f"y=[{_fmt(metrics.get('build_zone_y_min'), 1)}, "
            f"{_fmt(metrics.get('build_zone_y_max'), 1)}] m.",
            "### Slot evidence",
        ]
    )
    lines.extend(_slot_lines(metrics))
    lines.extend(
        [
            "### Temporal evidence",
            f"- Peak speed: {_fmt(metrics.get('peak_speed'))} m/s at step "
            f"{metrics.get('peak_speed_step', 'unavailable')}.",
            f"- Peak angular speed: {_fmt(metrics.get('peak_angular_vel'))} rad/s at step "
            f"{metrics.get('peak_angular_vel_step', 'unavailable')}.",
        ]
    )
    events = metrics.get("trajectory_events")
    if isinstance(events, list) and events:
        for event in events:
            if isinstance(event, dict):
                lines.append(
                    f"- Step {event.get('step', 'unavailable')}: {event.get('event', 'unavailable')}."
                )
    else:
        lines.append("- Trajectory events: none recorded.")
    lines.append("### Data health")
    lines.extend(_health_lines(metrics))
    return lines


def get_improvement_suggestions(
    metrics: Dict[str, Any],
    score: float,
    success: bool,
    failed: bool,
    failure_reason: Optional[str] = None,
    error: Optional[str] = None,
) -> List[str]:
    del metrics, score, success, failed, failure_reason
    if error:
        return [f"- Resolve the reported execution error: {error}"]
    return []
