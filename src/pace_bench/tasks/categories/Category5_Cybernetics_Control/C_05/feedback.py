"""Deterministic, observation-only feedback for C-05."""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional


def _number(value: Any) -> Optional[float]:
    if isinstance(value, bool):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return result if math.isfinite(result) else None


def _fmt(value: Any, digits: int = 3) -> str:
    number = _number(value)
    return "unavailable" if number is None else f"{number:.{digits}f}"


def _int_text(value: Any) -> str:
    number = _number(value)
    return "unavailable" if number is None else str(int(number))


def _outcome(metrics: Dict[str, Any]) -> str:
    step = _int_text(metrics.get("step_count"))
    maximum = _int_text(metrics.get("max_steps"))
    where = f"step {step}/{maximum}"
    if metrics.get("success"):
        return f"- Outcome: SUCCESS at {where}."
    if metrics.get("failed"):
        reason = metrics.get("failure_reason") or "failure reason unavailable"
        return f"- Outcome: FAILURE at {where} — {reason}."
    return f"- Outcome: IN PROGRESS at {where}."


def _window_line(metrics: Dict[str, Any], prefix: str) -> str:
    visited = bool(metrics.get(f"{prefix[0]}_visited"))
    elapsed = _number(metrics.get(f"steps_since_last_{prefix[0]}"))
    limit_key = "temporal_window_A_to_B" if prefix == "A→B" else "temporal_window_B_to_C"
    limit = _number(metrics.get(limit_key))
    if not visited:
        return f"- {prefix} recency: source zone not yet visited."
    if elapsed is None or limit is None:
        return f"- {prefix} recency: unavailable."
    return (
        f"- {prefix} recency: {int(elapsed)}/{int(limit)} steps; "
        f"remaining margin {int(limit - elapsed)} steps."
    )


def format_task_metrics(metrics: Dict[str, Any]) -> List[str]:
    if not isinstance(metrics, dict) or not metrics:
        return ["## C-05 Logic Lock Feedback", "- Outcome: unavailable — no metrics provided."]

    triggered = metrics.get("triggered_switches")
    if not isinstance(triggered, list):
        triggered = []
    sequence = " → ".join(str(item) for item in triggered) or "none"
    cooldown = _number(metrics.get("cooldown_remaining"))
    cooldown_total = _number(metrics.get("cooldown_total"))
    barrier_remaining = _number(metrics.get("barrier_steps_until_open"))
    dwell = _number(metrics.get("steps_in_current_zone"))
    dwell_required = _number(metrics.get("steps_required_to_trigger"))
    speed = _number(metrics.get("speed"))
    speed_limit = _number(metrics.get("speed_cap_inside"))
    force = _number(metrics.get("applied_force_magnitude"))
    force_limit = _number(metrics.get("force_limit_inside"))
    recent_y = _number(metrics.get("agent_y_max_recent"))
    required_y = _number(metrics.get("required_max_y_c"))

    lines = [
        "## C-05 Logic Lock Feedback",
        _outcome(metrics),
        "### Sequence timeline",
        f"- Triggered: {sequence}; next required: {metrics.get('next_required') or 'none'}; "
        f"wrong order={bool(metrics.get('wrong_order'))}.",
        (
            f"- Cooldown: {_int_text(cooldown)}/{_int_text(cooldown_total)} steps remaining/total; "
            f"barrier active={bool(metrics.get('barrier_active'))}, "
            f"steps until open={_int_text(barrier_remaining)}."
        ),
        _window_line(metrics, "A→B"),
        _window_line(metrics, "B→C"),
        "### Current observable state",
        f"- Position: ({_fmt(metrics.get('agent_x'))}, {_fmt(metrics.get('agent_y'))}) m; "
        f"velocity: ({_fmt(metrics.get('agent_vx'))}, {_fmt(metrics.get('agent_vy'))}) m/s.",
        f"- Distance to next zone boundary: {_fmt(metrics.get('distance_to_next_zone'))} m; "
        f"inside next zone={bool(metrics.get('inside_next_required_zone'))}.",
        f"- Measured repulsion at agent: ({_fmt(metrics.get('repulsion_fx'))}, "
        f"{_fmt(metrics.get('repulsion_fy'))}) N; magnitude "
        f"{_fmt(metrics.get('repulsion_magnitude'))} N.",
        "### Live constraint margins",
    ]

    if speed is not None and speed_limit is not None:
        lines.append(
            f"- Speed: {speed:.3f}/{speed_limit:.3f} m/s; margin {speed_limit - speed:+.3f} m/s."
        )
    else:
        lines.append("- Speed limit state: unavailable.")
    if force is not None and force_limit is not None:
        lines.append(
            f"- Controller force: {force:.3f}/{force_limit:.3f} N; margin {force_limit - force:+.3f} N."
        )
    else:
        lines.append("- Controller force-limit state: unavailable.")
    if dwell is not None and dwell_required is not None:
        lines.append(
            f"- Current dwell progress: {int(dwell)}/{int(dwell_required)} consecutive steps; "
            f"remaining {max(0, int(dwell_required - dwell))}."
        )
    if recent_y is not None and required_y is not None:
        lines.append(
            f"- Retained altitude maximum: {recent_y:.3f}/{required_y:.3f} m required for C; "
            f"margin {recent_y - required_y:+.3f} m."
        )

    reset_pairs = (
        ("zone exit", "dwell_reset_zone_change"),
        ("speed", "dwell_reset_speed"),
        ("force", "dwell_reset_force"),
        ("temporal", "dwell_blocked_temporal"),
        ("altitude", "dwell_blocked_altitude"),
        ("cooldown", "dwell_blocked_cooldown"),
    )
    lines.append(
        "- Reset/block counts: "
        + ", ".join(f"{label}={_int_text(metrics.get(key))}" for label, key in reset_pairs)
        + "."
    )

    invalid = []
    for key in (
        "agent_x", "agent_y", "agent_vx", "agent_vy", "speed",
        "applied_force_magnitude", "repulsion_fx", "repulsion_fy",
        "repulsion_magnitude", "distance_to_next_zone",
    ):
        if key in metrics and metrics.get(key) is not None and _number(metrics.get(key)) is None:
            invalid.append(key)
    lines.append("### Numerical health")
    lines.append(
        "- Invalid or non-finite observations: " + ", ".join(invalid) + "."
        if invalid
        else "- All reported numeric observations are finite."
    )
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
