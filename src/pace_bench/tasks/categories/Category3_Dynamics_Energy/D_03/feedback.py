"""Deterministic feedback for D-03 using only public constraints and observations."""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional


def _number(value: Any) -> Optional[float]:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _fmt(value: Any, digits: int = 2) -> str:
    number = _number(value)
    return "unavailable" if number is None else f"{number:.{digits}f}"


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


def _event_lines(metrics: Dict[str, Any]) -> List[str]:
    events: List[tuple[int, str]] = []
    for event in metrics.get("zone_crossings") or []:
        if isinstance(event, dict):
            step = int(_number(event.get("step")) or 0)
            events.append((step, f"zone {event.get('from_zone', '?')} → {event.get('to_zone', '?')} at x={_fmt(event.get('x'))} m"))
    for event in metrics.get("gate_arrival_events") or []:
        if isinstance(event, dict):
            step = int(_number(event.get("step")) or 0)
            state = "open" if event.get("gate_open") else "closed"
            events.append((step, f"reached gate {event.get('gate', '?')} while {state}"))
    for event in metrics.get("gate_collision_details") or []:
        if isinstance(event, dict):
            step = int(_number(event.get("step")) or 0)
            events.append((step, f"collided with gate {event.get('gate', '?')}"))
    if not events:
        return ["- No events recorded."]
    return [f"- Step {step}: {text}." for step, text in sorted(events, key=lambda item: item[0])]


def format_task_metrics(metrics: Dict[str, Any]) -> List[str]:
    if not isinstance(metrics, dict) or not metrics:
        return ["**Outcome**: unavailable — no metrics were provided."]
    lines = [
        "## D-03 Phase-Locked Gate Feedback",
        _outcome(metrics),
        "### Observed motion",
        f"- Cart x={_fmt(metrics.get('x'))} m, velocity=({_fmt(metrics.get('vx'))}, "
        f"{_fmt(metrics.get('vy'))}) m/s, speed={_fmt(metrics.get('speed'))} m/s.",
        f"- Target: x≥{_fmt(metrics.get('target_x_min'))} m with speed in "
        f"[{_fmt(metrics.get('target_speed_min'))}, {_fmt(metrics.get('target_speed_max'))}] m/s.",
        f"- Speed trap at x={_fmt(metrics.get('speed_trap_x'))} m: observed speed "
        f"{_fmt(metrics.get('speed_trap_actual_speed'))} m/s; minimum "
        f"{_fmt(metrics.get('speed_trap_min'))} m/s.",
        f"- Checkpoint at x={_fmt(metrics.get('checkpoint_11_x'))} m: observed speed "
        f"{_fmt(metrics.get('checkpoint_11_actual_speed'))} m/s; band "
        f"[{_fmt(metrics.get('checkpoint_11_speed_min'))}, "
        f"{_fmt(metrics.get('checkpoint_11_speed_max'))}] m/s.",
        "### Public build constraints",
        f"- Beams: {metrics.get('beam_count', 'unavailable')} / allowed "
        f"[{metrics.get('min_beam_count', 'unavailable')}, {metrics.get('max_beam_count', 'unavailable')}].",
        f"- Structure mass: {_fmt(metrics.get('structure_mass'))} / "
        f"{_fmt(metrics.get('max_structure_mass'))} kg.",
        "### Event chronology",
    ]
    lines.extend(_event_lines(metrics))
    lines.append("### Data health")
    bad = []
    for key in ("x", "speed", "vx", "vy", "peak_speed"):
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
    failure_reason: Optional[str] = None,
    error: Optional[str] = None,
) -> List[str]:
    del metrics, score, success, failed, failure_reason
    if error:
        return [f"- Resolve the reported execution error: {error}"]
    return []
