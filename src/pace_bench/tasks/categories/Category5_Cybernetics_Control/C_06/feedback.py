"""Compact, deterministic, non-privileged feedback for C-06."""

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


def _fmt(value: Any, digits: int = 4) -> str:
    number = _number(value)
    return "unavailable" if number is None else f"{number:.{digits}f}"


def _integer(value: Any) -> str:
    number = _number(value)
    return "unavailable" if number is None else str(int(number))


def _outcome(metrics: Dict[str, Any]) -> str:
    step = _integer(metrics.get("step_count"))
    maximum = _integer(metrics.get("max_steps"))
    if metrics.get("success"):
        return f"- Outcome: SUCCESS at step {step}/{maximum}."
    if metrics.get("failed"):
        reason = metrics.get("failure_reason") or "failure reason unavailable"
        return f"- Outcome: FAILURE at step {step}/{maximum} — {reason}."
    return f"- Outcome: IN PROGRESS at step {step}/{maximum}."


def _target_events(metrics: Dict[str, Any]) -> str:
    raw = metrics.get("target_change_events")
    if not isinstance(raw, list) or not raw:
        return "- Target changes observed: none so far."
    parts = []
    for event in raw:
        if not isinstance(event, dict):
            continue
        parts.append(
            f"step {_integer(event.get('step'))}: "
            f"{_fmt(event.get('from'))}→{_fmt(event.get('to'))} rad/s"
        )
    return "- Target changes observed: " + ("; ".join(parts) if parts else "unavailable") + "."


def format_task_metrics(
    metrics: Dict[str, Any],
    previous_metrics: Optional[Dict[str, Any]] = None,
) -> List[str]:
    del previous_metrics
    if not isinstance(metrics, dict) or not metrics:
        return ["## C-06 Governor Feedback", "- Outcome: unavailable — no metrics provided."]

    step = _number(metrics.get("step_count"))
    regulation_start = _number(metrics.get("regulation_start_step"))
    if step is None or regulation_start is None:
        phase = "unavailable"
    elif step < regulation_start:
        phase = f"startup; {int(regulation_start - step)} steps until regulation"
    else:
        phase = f"regulation; {int(step - regulation_start)} scored steps elapsed"

    sensed = _number(metrics.get("wheel_angular_velocity"))
    target = _number(metrics.get("target_speed"))
    reported_error = _number(metrics.get("reported_speed_error"))
    mean_error = _number(metrics.get("mean_speed_error"))
    mean_limit = _number(metrics.get("mean_speed_error_threshold"))
    stall_count = _number(metrics.get("stall_count"))
    stall_limit = _number(metrics.get("stall_steps_threshold"))
    stall_speed = _number(metrics.get("stall_speed_threshold"))

    lines = [
        "## C-06 Governor Feedback",
        _outcome(metrics),
        "### Timeline",
        f"- Phase: {phase}.",
        _target_events(metrics),
        f"- First below-stall observation step: {_integer(metrics.get('first_stall_step'))}; "
        f"maximum consecutive below-stall count: {_integer(metrics.get('maximum_stall_count'))}.",
        "### Sensor and command observations",
        f"- Delayed speed readout: {_fmt(sensed)} rad/s; current target: {_fmt(target)} rad/s; "
        f"absolute reported error: {_fmt(reported_error)} rad/s.",
        f"- Peak absolute reported error through this step: "
        f"{_fmt(metrics.get('peak_reported_speed_error'))} rad/s.",
        f"- Requested motor torque on the latest step: "
        f"{_fmt(metrics.get('commanded_torque'))} N·m.",
        "### Published grading state",
    ]
    if mean_error is not None and mean_limit is not None:
        lines.append(
            f"- Regulation-phase scoring mean |ω−target|: "
            f"{mean_error:.4f}/{mean_limit:.4f} rad/s; margin "
            f"{mean_limit - mean_error:+.4f} rad/s."
        )
    else:
        lines.append("- Regulation-phase scoring mean: unavailable.")
    if stall_count is not None and stall_limit is not None and stall_speed is not None:
        lines.append(
            f"- Current consecutive stall count: {int(stall_count)}/{int(stall_limit)} "
            f"at the published |ω|<{stall_speed:.4f} rad/s condition; remaining "
            f"{int(stall_limit - stall_count)} steps."
        )
    else:
        lines.append("- Stall counter state: unavailable.")

    invalid = []
    for key in (
        "wheel_angular_velocity",
        "target_speed",
        "reported_speed_error",
        "mean_speed_error",
        "commanded_torque",
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
    failure_reason: Optional[str] = None,
    error: Optional[str] = None,
) -> List[str]:
    if error:
        return [f"- Resolve the reported execution error: {error}"]
    return []
