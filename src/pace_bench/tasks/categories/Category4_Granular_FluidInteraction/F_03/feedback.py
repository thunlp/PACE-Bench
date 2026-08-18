"""Deterministic, observation-only feedback for F-03."""

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


def _carry_lines(metrics: Dict[str, Any]) -> List[str]:
    log = metrics.get("carry_log")
    if not isinstance(log, list) or not log:
        return ["- Carry observations unavailable."]
    valid = [entry for entry in log if isinstance(entry, dict)]
    if not valid:
        return ["- Carry observations were malformed."]
    counts = [_number(entry.get("carried")) for entry in valid]
    counts = [count for count in counts if count is not None]
    peak = max(counts) if counts else None
    active = sum(1 for count in counts if count > 0)
    dumping = sum(1 for entry in valid if entry.get("over_hopper") and entry.get("dumping"))
    return [
        f"- Observed peak carried particles: {_fmt(peak, 0)}; carrying observed in "
        f"{active}/{len(valid)} recorded samples; dumping posture over hopper in "
        f"{dumping} samples."
    ]


def _break_lines(metrics: Dict[str, Any]) -> List[str]:
    events = metrics.get("joint_break_events")
    if not isinstance(events, list) or not events:
        return ["- No joint break was observed."]
    lines = [f"- Joint breaks observed: {len(events)}."]
    for event in events[-8:]:
        if isinstance(event, dict):
            lines.append(
                f"  - Step {event.get('step', 'unavailable')}: reaction force "
                f"{_fmt(event.get('force_N'))} N, reaction torque "
                f"{_fmt(event.get('torque_Nm'))} N·m."
            )
    return lines


def format_task_metrics(metrics: Dict[str, Any]) -> List[str]:
    if not isinstance(metrics, dict) or not metrics:
        return ["**Outcome**: unavailable — no metrics were provided."]
    lines = [
        "## F-03 Excavator Feedback",
        _outcome(metrics),
        "### Transfer observations",
        f"- Hopper count: {metrics.get('particles_in_truck', 'unavailable')} / "
        f"{metrics.get('min_particles_in_hopper', 'unavailable')} required, from "
        f"{metrics.get('initial_particle_count', 'unavailable')} initial particles.",
        f"- Particle distribution: {metrics.get('particles_in_pit', 'unavailable')} "
        f"in pit; {metrics.get('particles_escaped', 'unavailable')} outside pit and hopper.",
    ]
    lines.extend(_carry_lines(metrics))
    lines.extend(
        [
            "### Mechanism observations",
            f"- Scoop position: ({_fmt(metrics.get('agent_x'))}, "
            f"{_fmt(metrics.get('agent_y'))}) m; velocity "
            f"({_fmt(metrics.get('velocity_x'))}, {_fmt(metrics.get('velocity_y'))}) m/s; "
            f"angle {_fmt(metrics.get('bucket_angle_rad'))} rad.",
            f"- Arm joint angle: {_fmt(metrics.get('arm_joint_angle_rad'))} rad; "
            f"arm body position ({_fmt(metrics.get('arm_x'))}, "
            f"{_fmt(metrics.get('arm_y'))}) m.",
            f"- Scoop trajectory envelope: x=[{_fmt(metrics.get('scoop_traj_x_min'))}, "
            f"{_fmt(metrics.get('scoop_traj_x_max'))}], y=["
            f"{_fmt(metrics.get('scoop_traj_y_min'))}, "
            f"{_fmt(metrics.get('scoop_traj_y_max'))}] m.",
            f"- Peak observed motion: scoop speed {_fmt(metrics.get('peak_body_speed'))} "
            f"m/s; angular speed {_fmt(metrics.get('peak_angular_velocity'))} rad/s; "
            f"particle speed {_fmt(metrics.get('max_particle_speed'))} m/s.",
            "### Public constraints",
            f"- Structure mass: {_fmt(metrics.get('structure_mass'))} / "
            f"{_fmt(metrics.get('max_structure_mass'))} kg; joints "
            f"{metrics.get('joint_count', 'unavailable')}; integrity "
            f"{'lost' if metrics.get('structure_broken') else 'intact'}.",
            f"- Time limit: {_fmt(metrics.get('max_time_seconds'))} s; build zone "
            f"x=[{_fmt(metrics.get('build_zone_x_min'))}, "
            f"{_fmt(metrics.get('build_zone_x_max'))}], y=["
            f"{_fmt(metrics.get('build_zone_y_min'))}, "
            f"{_fmt(metrics.get('build_zone_y_max'))}] m.",
        ]
    )
    violations = metrics.get("constraint_violations")
    if isinstance(violations, list) and violations:
        lines.append(f"- Design violations: {len(violations)}.")
        lines.extend(f"  - {violation}" for violation in violations)
    else:
        lines.append("- No design violations reported.")
    lines.append("### Structural loads")
    lines.append(
        f"- Peak observed joint reaction: {_fmt(metrics.get('peak_joint_force'))} N, "
        f"{_fmt(metrics.get('peak_joint_torque'))} N·m."
    )
    lines.extend(_break_lines(metrics))
    lines.append("### Data health")
    errors = metrics.get("observation_errors")
    invalid = []
    for key in (
        "agent_x", "agent_y", "velocity_x", "velocity_y", "structure_mass",
        "peak_body_speed", "max_particle_speed",
    ):
        if key in metrics and metrics.get(key) is not None and _number(metrics.get(key)) is None:
            invalid.append(key)
    if invalid:
        lines.append("- Non-finite observations: " + ", ".join(invalid) + ".")
    if isinstance(errors, list) and errors:
        lines.extend(f"- Observation error: {error}" for error in errors)
    if not invalid and not (isinstance(errors, list) and errors):
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
