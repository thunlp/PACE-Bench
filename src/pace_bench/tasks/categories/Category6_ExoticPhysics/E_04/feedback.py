from __future__ import annotations

import math
from typing import Any, Dict, List, Optional


def _finite(value: Any) -> Optional[float]:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    value = float(value)
    return value if math.isfinite(value) else None


def _number(metrics: Dict[str, Any], key: str) -> Optional[float]:
    return _finite(metrics.get(key))


def _fmt(value: Optional[float], digits: int = 3) -> str:
    return "unavailable" if value is None else f"{value:.{digits}f}"


def _event_line(label: str, event: Any) -> str:
    if not isinstance(event, dict):
        return f"{label}: unavailable."
    utilization = _finite(event.get("utilization"))
    step = _finite(event.get("step"))
    load = _finite(event.get("load"))
    limit = _finite(event.get("limit"))
    anchor_x = _finite(event.get("anchor_x"))
    anchor_y = _finite(event.get("anchor_y"))
    joint_type = str(event.get("joint_type", "unknown"))
    if utilization is None:
        return f"{label}: unavailable."
    margin = (1.0 - utilization) * 100.0
    return (
        f"{label}: {utilization * 100.0:.2f}% utilization "
        f"({margin:+.2f}% margin), {joint_type} joint at "
        f"({_fmt(anchor_x)}, {_fmt(anchor_y)}) m, step {_fmt(step, 0)}, "
        f"measured load {_fmt(load, 6)} against {_fmt(limit, 6)}."
    )


def _broken_joint_lines(events: Any) -> List[str]:
    if not isinstance(events, list) or not events:
        return ["Joint failures: none recorded."]
    lines = [f"Joint failures recorded: {len(events)}."]
    ordered = sorted(
        (event for event in events if isinstance(event, dict)),
        key=lambda event: _finite(event.get("break_step")) or math.inf,
    )
    displayed = ordered
    if len(ordered) > 6:
        displayed = ordered[:3] + ordered[-2:]
        lines.append(
            f"Showing the first 3 and last 2 events; "
            f"{len(ordered) - 5} intermediate event(s) omitted."
        )
    for index, event in enumerate(displayed, start=1):
        step = _finite(event.get("break_step"))
        anchor_x = _finite(event.get("anchor_x"))
        anchor_y = _finite(event.get("anchor_y"))
        force = _finite(event.get("force_at_break"))
        torque = _finite(event.get("torque_at_break"))
        force_limit = _finite(event.get("force_limit_at_break"))
        torque_limit = _finite(event.get("torque_limit_at_break"))
        lines.append(
            f"{index}. {event.get('joint_type', 'unknown')} joint at "
            f"({_fmt(anchor_x)}, {_fmt(anchor_y)}) m broke at step "
            f"{_fmt(step, 0)}; force {_fmt(force, 6)}/{_fmt(force_limit, 6)} N; "
            f"torque {_fmt(torque, 6)}/{_fmt(torque_limit, 6)} N·m."
        )
    return lines


def _active_peak_lines(per_joint: Any) -> List[str]:
    if not isinstance(per_joint, dict) or not per_joint:
        return ["Active-joint peak measurements: unavailable."]
    entries = [entry for entry in per_joint.values() if isinstance(entry, dict)]
    force_entries = sorted(
        entries,
        key=lambda entry: _finite(entry.get("force")) or -math.inf,
        reverse=True,
    )[:3]
    torque_entries = sorted(
        entries,
        key=lambda entry: _finite(entry.get("torque")) or -math.inf,
        reverse=True,
    )[:3]

    def describe(entry: Dict[str, Any], key: str, unit: str) -> str:
        return (
            f"{entry.get('type', 'unknown')} at "
            f"({_fmt(_finite(entry.get('anchor_x')))}, "
            f"{_fmt(_finite(entry.get('anchor_y')))}) m: "
            f"{_fmt(_finite(entry.get(key)), 6)} {unit}"
        )

    lines = ["Largest active-joint force peaks:"]
    lines.extend(f"- {describe(entry, 'force', 'N')}" for entry in force_entries)
    lines.append("Largest active-joint torque peaks:")
    lines.extend(
        f"- {describe(entry, 'torque', 'N·m')}" for entry in torque_entries
    )
    return lines


def format_task_metrics(metrics: Dict[str, Any]) -> List[str]:
    if not metrics:
        return ["**Evaluation data**: unavailable"]
    if "error" in metrics:
        return [f"**Evaluator error**: {metrics.get('error')}"]

    critical_keys = (
        "structure_mass",
        "max_structure_mass",
        "max_joint_reaction_force",
        "max_joint_reaction_torque",
        "peak_body_speed",
    )
    invalid = [
        key
        for key in critical_keys
        if key in metrics and _number(metrics, key) is None
    ]
    if invalid:
        return [
            "### Evaluation state",
            "Non-finite or invalid measured fields: " + ", ".join(invalid),
        ]

    if metrics.get("success"):
        outcome = "success"
    elif metrics.get("failed"):
        outcome = "failure"
    else:
        outcome = "in progress"

    lines = ["### Outcome", f"Status: {outcome}."]
    reason = metrics.get("failure_reason")
    if reason:
        lines.append(f"Recorded reason: {reason}.")
    violations = metrics.get("design_violations")
    if isinstance(violations, list) and violations:
        lines.append("Recorded design violations: " + "; ".join(map(str, violations)) + ".")

    step = _number(metrics, "step_count")
    max_steps = _number(metrics, "max_steps")
    sim_time = _number(metrics, "simulation_time_s")
    lines.extend(
        [
            "",
            "### Duration and topology",
            (
                f"Step: {_fmt(step, 0)}/{_fmt(max_steps, 0)}; "
                f"simulation time: {_fmt(sim_time)} s."
            ),
            (
                f"Beams: {_fmt(_number(metrics, 'beam_count'), 0)} "
                f"(minimum {_fmt(_number(metrics, 'min_beams'), 0)}); "
                f"joints: {_fmt(_number(metrics, 'joint_count'), 0)}/"
                f"{_fmt(_number(metrics, 'initial_joint_count'), 0)} "
                f"(minimum {_fmt(_number(metrics, 'min_joints'), 0)}); "
                f"pivot joints: {_fmt(_number(metrics, 'pivot_joint_count'), 0)}."
            ),
            (
                f"Observed beam-center span: "
                f"[{_fmt(_number(metrics, 'span_min_x'))}, "
                f"{_fmt(_number(metrics, 'span_max_x'))}] m; required reach: "
                f"x <= {_fmt(_number(metrics, 'required_span_left_x'))} m and "
                f"x >= {_fmt(_number(metrics, 'required_span_right_x'))} m."
            ),
            (
                "Beam centers outside build zone: "
                f"{_fmt(_number(metrics, 'out_of_zone_beam_count'), 0)}."
            ),
            "",
            "### Mass envelope",
            (
                f"Current mass: {_fmt(_number(metrics, 'structure_mass'), 6)} kg; "
                f"recorded peak: {_fmt(_number(metrics, 'peak_structure_mass'), 6)} kg; "
                f"limit: {_fmt(_number(metrics, 'max_structure_mass'), 3)} kg."
            ),
        ]
    )
    first_mass_step = _number(metrics, "first_mass_violation_step")
    lines.append(
        "First mass-limit event: "
        + (
            f"step {first_mass_step:.0f}."
            if first_mass_step is not None
            else "none recorded."
        )
    )

    lines.extend(["", "### Joint integrity"])
    lines.extend(_broken_joint_lines(metrics.get("joints_ever_broken")))
    closest = metrics.get("closest_joint_margin_events")
    closest = closest if isinstance(closest, dict) else {}
    lines.append(_event_line("Closest force-limit event", closest.get("force")))
    lines.append(_event_line("Closest torque-limit event", closest.get("torque")))
    lines.append(
        f"All-time measured reaction peaks: "
        f"{_fmt(_number(metrics, 'max_joint_reaction_force'), 6)} N and "
        f"{_fmt(_number(metrics, 'max_joint_reaction_torque'), 6)} N·m."
    )
    lines.append(
        f"Effective limits at final sample: "
        f"{_fmt(_number(metrics, 'effective_joint_force_limit'), 6)} N and "
        f"{_fmt(_number(metrics, 'effective_joint_torque_limit'), 6)} N·m."
    )
    lines.extend(["", "### Spatial load observations"])
    lines.extend(_active_peak_lines(metrics.get("per_joint_peaks")))
    lines.append(
        f"Peak measured body speed: "
        f"{_fmt(_number(metrics, 'peak_body_speed'), 6)} m/s."
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
    del score, success, failed, failure_reason
    if error:
        return [f"- Execution error: {error[:200]}"]
    if not metrics:
        return []
    if metrics.get("success"):
        return ["- Full-duration joint integrity was recorded."]
    events = metrics.get("joints_ever_broken")
    if isinstance(events, list) and events:
        first = min(
            (
                _finite(event.get("break_step"))
                for event in events
                if isinstance(event, dict)
            ),
            default=None,
            key=lambda value: math.inf if value is None else value,
        )
        if first is not None:
            return [f"- The first recorded joint failure occurred at step {first:.0f}."]
    mass_step = _number(metrics, "first_mass_violation_step")
    if mass_step is not None:
        return [f"- The first recorded mass-limit event occurred at step {mass_step:.0f}."]
    return ["- The recorded run did not satisfy all completion conditions."]
