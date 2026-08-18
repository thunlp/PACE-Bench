"""Deterministic, observation-only feedback for D-06."""

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


def _ball_lines(metrics: Dict[str, Any]) -> List[str]:
    positions = metrics.get("per_ball_positions")
    speeds = metrics.get("per_ball_speeds")
    caught = metrics.get("per_ball_caught")
    if not isinstance(positions, dict) or not positions:
        return ["- Per-ball observations unavailable."]
    if not isinstance(speeds, dict):
        speeds = {}
    if not isinstance(caught, dict):
        caught = {}
    lines = []
    for index in sorted(positions, key=lambda value: int(value)):
        position = positions.get(index)
        if not isinstance(position, (list, tuple)) or len(position) < 2:
            lines.append(f"- Ball {int(index) + 1}: position unavailable.")
            continue
        status = "caught" if caught.get(index, False) else "uncaught"
        lines.append(
            f"- Ball {int(index) + 1}: {status}; position "
            f"({_fmt(position[0])}, {_fmt(position[1])}) m; speed "
            f"{_fmt(speeds.get(index))} m/s."
        )
    return lines


def format_task_metrics(metrics: Dict[str, Any]) -> List[str]:
    if not isinstance(metrics, dict) or not metrics:
        return ["**Outcome**: unavailable — no metrics were provided."]
    lines = [
        "## D-06 Sequential Catch Feedback",
        _outcome(metrics),
        f"- Caught: {metrics.get('balls_caught_count', 'unavailable')} / "
        f"{metrics.get('balls_required_count', 'unavailable')} balls.",
        f"- Target box: x=[{_fmt(metrics.get('target_x_min'))}, "
        f"{_fmt(metrics.get('target_x_max'))}], y=[{_fmt(metrics.get('target_y_min'))}, "
        f"{_fmt(metrics.get('target_y_max'))}] m; caught-speed threshold "
        f"{_fmt(metrics.get('caught_speed_threshold'))} m/s.",
        f"- Pit rule: y<{_fmt(metrics.get('pit_y_threshold'))} m while speed>"
        f"{_fmt(metrics.get('pit_speed_threshold'))} m/s.",
        "### Per-ball observations",
    ]
    lines.extend(_ball_lines(metrics))
    lines.extend(
        [
            "### Sequence observations",
            f"- Approach line x<{_fmt(metrics.get('approach_x_m'))} m; "
            f"sequential violation: {bool(metrics.get('sequential_violation'))}.",
        ]
    )
    sequence = metrics.get("sequential_detail")
    if isinstance(sequence, list) and sequence:
        for entry in sequence:
            if isinstance(entry, dict):
                predecessors = entry.get("predecessors_uncaught")
                predecessor_text = "none"
                if isinstance(predecessors, list) and predecessors:
                    predecessor_text = ", ".join(
                        str(item.get("predecessor_idx", "?"))
                        for item in predecessors if isinstance(item, dict)
                    )
                lines.append(
                    f"- Ball {entry.get('ball_idx', '?')} crossed at step "
                    f"{entry.get('approach_step', 'unavailable')} with speed "
                    f"{_fmt(entry.get('speed_at_approach'))} m/s; uncaught predecessors: "
                    f"{predecessor_text}."
                )
    lines.extend(
        [
            "### Public structural constraints",
            f"- Beams: {metrics.get('beam_count', 'unavailable')}; joints: "
            f"{metrics.get('joint_count', 'unavailable')}.",
            f"- Structure mass: {_fmt(metrics.get('structure_mass'))} / strictly less than "
            f"{_fmt(metrics.get('max_structure_mass'))} kg.",
            f"- Peak observed joint force: {_fmt(metrics.get('peak_joint_force'))} N; "
            f"peak limit {_fmt(metrics.get('max_joint_force_limit'))} N; fatigue limit "
            f"{_fmt(metrics.get('joint_fatigue_threshold'))} N.",
            f"- Structure smashed: {bool(metrics.get('structure_smashed'))}; pit failure: "
            f"{bool(metrics.get('pit_failure'))}.",
            "### Data health",
        ]
    )
    bad = []
    for key in ("ball_x", "ball_y", "ball_vx", "ball_vy", "ball_speed", "peak_joint_force"):
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
