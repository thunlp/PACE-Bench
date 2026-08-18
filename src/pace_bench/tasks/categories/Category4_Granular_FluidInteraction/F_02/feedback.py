"""Deterministic, observation-only feedback for F-02."""

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
    maximum = metrics.get("max_steps")
    step_text = f"{step}/{maximum}" if maximum is not None else str(step)
    if metrics.get("success"):
        return f"**Outcome**: SUCCESS at step {step_text}."
    if metrics.get("failed"):
        reason = metrics.get("failure_reason") or "failure reason unavailable"
        return f"**Outcome**: FAILURE at step {step_text} — {reason}."
    return f"**Outcome**: IN PROGRESS at step {step_text}."


def _event_lines(metrics: Dict[str, Any]) -> List[str]:
    events = metrics.get("zone_crossing_events")
    if not isinstance(events, list) or not events:
        return ["- No zone-boundary crossing has been observed."]
    lines = []
    for event in events[-12:]:
        if not isinstance(event, dict):
            lines.append("- Malformed crossing observation.")
            continue
        lines.append(
            f"- Step {event.get('step', 'unavailable')}: "
            f"{event.get('zone', 'unknown boundary')} at front x="
            f"{_fmt(event.get('front_x'))} m, lowest y={_fmt(event.get('lowest_y'))} m."
        )
    return lines


def _joint_lines(metrics: Dict[str, Any]) -> List[str]:
    lines = [
        f"- Integrity: {'lost' if metrics.get('structure_broken') else 'intact'}; "
        f"joints {metrics.get('joint_count', 'unavailable')} / initial "
        f"{metrics.get('initial_joint_count', 'unavailable')}."
    ]
    events = metrics.get("joint_failure_events")
    if isinstance(events, list) and events:
        lines.append(f"- Observed joint failures: {len(events)}.")
        for event in events[-8:]:
            if isinstance(event, dict):
                lines.append(
                    f"  - Step {event.get('step', 'unavailable')}: reaction force "
                    f"{_fmt(event.get('reaction_force'))} N at bodies "
                    f"{event.get('body_a_idx', '?')}–{event.get('body_b_idx', '?')}."
                )
    samples = metrics.get("joint_force_samples")
    if isinstance(samples, list) and samples:
        values = [
            _number(sample.get("reaction_force"))
            for sample in samples
            if isinstance(sample, dict)
        ]
        values = [value for value in values if value is not None]
        if values:
            lines.append(f"- Highest sampled joint reaction: {max(values):.3f} N.")
    return lines


def _body_lines(metrics: Dict[str, Any]) -> List[str]:
    observations = metrics.get("body_observations")
    if not isinstance(observations, list) or not observations:
        return ["- Component observations unavailable."]
    lines = []
    for body in observations[:24]:
        if not isinstance(body, dict):
            lines.append("- Malformed component observation.")
            continue
        lines.append(
            f"- Body {body.get('body_idx', '?')}: position "
            f"({_fmt(body.get('x'))}, {_fmt(body.get('y'))}) m; velocity "
            f"({_fmt(body.get('vx'))}, {_fmt(body.get('vy'))}) m/s; "
            f"in water={bool(body.get('in_water'))}."
        )
    if len(observations) > 24:
        lines.append(f"- {len(observations) - 24} additional components omitted.")
    return lines


def format_task_metrics(metrics: Dict[str, Any]) -> List[str]:
    if not isinstance(metrics, dict) or not metrics:
        return ["**Outcome**: unavailable — no metrics were provided."]
    lines = [
        "## F-02 Amphibian Feedback",
        _outcome(metrics),
        "### Progress observations",
        f"- Vehicle front x={_fmt(metrics.get('vehicle_front_x'))} m; target "
        f"x={_fmt(metrics.get('target_x'))} m; progress {_fmt(metrics.get('progress'))}%.",
        f"- Lowest vehicle point y={_fmt(metrics.get('vehicle_lowest_y'))} m; "
        f"sink threshold y={_fmt(metrics.get('sink_y_threshold'))} m.",
        f"- Front-component velocity: ({_fmt(metrics.get('velocity_x'))}, "
        f"{_fmt(metrics.get('velocity_y'))}) m/s; speed-cap events "
        f"{metrics.get('speed_cap_count', 'unavailable')} at "
        f"{_fmt(metrics.get('speed_cap_limit'))} m/s.",
        "### Public design constraints",
        f"- Structure mass: {_fmt(metrics.get('structure_mass'))} / "
        f"{_fmt(metrics.get('max_structure_mass'))} kg.",
        f"- Thrust cooldown: {metrics.get('thrust_cooldown_steps', 'unavailable')} steps.",
    ]
    violations = metrics.get("constraint_violations")
    if isinstance(violations, list) and violations:
        lines.append(f"- Design violations: {len(violations)}.")
        lines.extend(f"  - {violation}" for violation in violations)
    else:
        lines.append("- No design violations reported.")
    lines.append("### Structural observations")
    lines.extend(_joint_lines(metrics))
    lines.append("### Crossing observations")
    lines.extend(_event_lines(metrics))
    lines.append("### Component observations")
    lines.extend(_body_lines(metrics))
    lines.append("### Data health")
    issues = metrics.get("numerical_issues")
    errors = metrics.get("observation_errors")
    if isinstance(issues, list) and issues:
        lines.extend(f"- Numerical issue: {issue}" for issue in issues)
    if isinstance(errors, list) and errors:
        lines.extend(f"- Observation error: {error}" for error in errors)
    if not (isinstance(issues, list) and issues) and not (
        isinstance(errors, list) and errors
    ):
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
