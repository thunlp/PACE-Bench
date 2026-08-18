"""Compact, deterministic, non-privileged feedback for E-06."""

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


def _ratio_line(
    label: str,
    value: Any,
    limit: Any,
    unit: str,
) -> str:
    measured = _number(value)
    threshold = _number(limit)
    if measured is None or threshold is None:
        return f"- {label}: unavailable."
    status = "PASS" if measured <= threshold else "FAIL"
    return (
        f"- {label}: {measured:.3f}/{threshold:.3f} {unit} ({status}); "
        f"margin {threshold - measured:+.3f} {unit}."
    )


def _event_timeline(metrics: Dict[str, Any]) -> List[str]:
    events = metrics.get("failure_event_timeline")
    if not isinstance(events, list) or not events:
        return ["- Failure events: none recorded."]
    entries = []
    for event in events[:6]:
        if not isinstance(event, dict):
            continue
        step = _integer(event.get("step"))
        position = (
            f"({_fmt(event.get('pos_x'))}, {_fmt(event.get('pos_y'))}) m"
        )
        if event.get("event_type") == "joint_failure":
            entries.append(
                f"step {step}: joint {event.get('fail_type', 'failure')} at "
                f"{position}, measured peak "
                f"{_fmt(event.get('peak_force'))} N/"
                f"{_fmt(event.get('peak_torque'))} N·m and damage "
                f"{_fmt(event.get('damage'))} pts"
            )
        else:
            entries.append(
                f"step {step}: beam destruction "
                f"({event.get('fail_reason', 'unknown')}) at {position}"
            )
    return [
        "- Failure chronology: "
        + ("; ".join(entries) if entries else "unavailable")
        + (f"; {len(events) - 6} later event(s) omitted" if len(events) > 6 else "")
        + "."
    ]


def _hotspots(metrics: Dict[str, Any]) -> str:
    records = metrics.get("per_joint_stress_data")
    if not isinstance(records, list) or not records:
        return "- Joint-stress locations: unavailable."
    force_limit = _number(metrics.get("joint_break_force"))
    torque_limit = _number(metrics.get("joint_break_torque"))
    damage_limit = _number(metrics.get("damage_limit"))
    ranked = []
    for record in records:
        if not isinstance(record, dict):
            continue
        ratios = []
        for value, limit in (
            (_number(record.get("peak_force")), force_limit),
            (_number(record.get("peak_torque")), torque_limit),
            (_number(record.get("damage")), damage_limit),
        ):
            if value is not None and limit is not None and limit > 0.0:
                ratios.append(value / limit)
        ranked.append((max(ratios, default=0.0), record))
    ranked.sort(key=lambda item: item[0], reverse=True)
    entries = []
    for ratio, record in ranked[:3]:
        entries.append(
            f"({_fmt(record.get('anchor_x'))}, {_fmt(record.get('anchor_y'))}) m: "
            f"F={_fmt(record.get('peak_force'))} N, "
            f"T={_fmt(record.get('peak_torque'))} N·m, "
            f"damage={_fmt(record.get('damage'))} pts, "
            f"worst utilization={ratio * 100.0:.1f}%"
        )
    return "- Highest measured joint utilizations: " + "; ".join(entries) + "."


def format_task_metrics(
    metrics: Dict[str, Any],
    previous_metrics: Optional[Dict[str, Any]] = None,
) -> List[str]:
    del previous_metrics
    if not isinstance(metrics, dict) or not metrics:
        return ["## E-06 Endurance Feedback", "- Outcome: unavailable — no metrics provided."]
    if metrics.get("error"):
        return [
            "## E-06 Endurance Feedback",
            f"- Outcome: evaluation error — {metrics.get('error')}.",
        ]

    initial_mass = _number(metrics.get("initial_structure_mass"))
    mass_limit = _number(metrics.get("max_structure_mass"))
    initial_anchors = _number(metrics.get("initial_ground_anchor_count"))
    required_anchors = _number(metrics.get("required_ground_anchor_count"))
    initial_min_x = _number(metrics.get("initial_min_x"))
    initial_max_x = _number(metrics.get("initial_max_x"))
    initial_max_y = _number(metrics.get("initial_max_y"))
    left_required = _number(metrics.get("span_x_left_required"))
    right_required = _number(metrics.get("span_x_right_required"))
    height_required = _number(metrics.get("minimum_height_required"))

    lines = [
        "## E-06 Endurance Feedback",
        _outcome(metrics),
        "### Failure chronology",
        (
            f"- First joint loss: step "
            f"{_integer(metrics.get('first_joint_fail_step'))} at "
            f"{metrics.get('first_joint_fail_pos') or 'unavailable'}, type "
            f"{metrics.get('first_joint_fail_type') or 'unavailable'}."
        ),
        (
            f"- First beam loss: step "
            f"{_integer(metrics.get('first_body_fail_step'))} at "
            f"{metrics.get('first_body_fail_pos') or 'unavailable'}, reason "
            f"{metrics.get('first_body_fail_reason') or 'unavailable'}."
        ),
        *_event_timeline(metrics),
        (
            f"- Loss totals: {_integer(metrics.get('total_joints_removed'))} "
            f"joint(s), {_integer(metrics.get('total_bodies_destroyed'))} "
            f"beam(s)."
        ),
        "### Spatial and build margins",
    ]

    if None not in (initial_min_x, initial_max_x, initial_max_y):
        left_margin = left_required - initial_min_x if left_required is not None else None
        right_margin = initial_max_x - right_required if right_required is not None else None
        height_margin = initial_max_y - height_required if height_required is not None else None
        lines.append(
            f"- Initial beam-center extent: x=[{initial_min_x:.3f}, "
            f"{initial_max_x:.3f}] m, max y={initial_max_y:.3f} m; "
            f"left/right/height margins "
            f"{_fmt(left_margin)}/{_fmt(right_margin)}/{_fmt(height_margin)} m."
        )
    else:
        lines.append("- Initial beam-center extent and span margins: unavailable.")
    lines.append(
        f"- Current active-beam extent: x=[{_fmt(metrics.get('current_min_x'))}, "
        f"{_fmt(metrics.get('current_max_x'))}] m, max y="
        f"{_fmt(metrics.get('current_max_y'))} m; runtime span diagnostic "
        f"{'PASS' if metrics.get('span_check_passed') else 'OUTSIDE INITIAL TARGET'}."
    )

    lines.extend(
        [
            "### Load, damage, and rotation",
            _ratio_line(
                "Peak measured joint reaction force",
                metrics.get("max_joint_force"),
                metrics.get("joint_break_force"),
                "N",
            ),
            _ratio_line(
                "Peak measured joint reaction torque",
                metrics.get("max_joint_torque"),
                metrics.get("joint_break_torque"),
                "N·m",
            ),
            _ratio_line(
                "Maximum accumulated joint damage",
                metrics.get("max_joint_damage"),
                metrics.get("damage_limit"),
                "pts",
            ),
            _hotspots(metrics),
            _ratio_line(
                "Peak measured beam angular speed",
                metrics.get("peak_body_angvel"),
                metrics.get("beam_angvel_thresh"),
                "rad/s",
            ),
            (
                f"- Worst current consecutive high-spin count: "
                f"{_integer(metrics.get('worst_spin_consec_steps'))}/"
                f"{_integer(metrics.get('beam_angvel_tolerance_steps'))} "
                f"simulation step(s), at "
                f"{metrics.get('worst_spin_body_pos') or 'unavailable'}."
            ),
            "### Constraint profile",
        ]
    )
    if initial_mass is not None and mass_limit is not None:
        lines.append(
            f"- Initial mass: {initial_mass:.6f}/{mass_limit:.6f} kg "
            f"({'PASS' if initial_mass <= mass_limit else 'FAIL'}); margin "
            f"{mass_limit - initial_mass:+.6f} kg."
        )
    else:
        lines.append("- Initial mass constraint: unavailable.")
    if initial_anchors is not None and required_anchors is not None:
        lines.append(
            f"- Initial ground-anchor count: {int(initial_anchors)}/"
            f"{int(required_anchors)} "
            f"({'PASS' if initial_anchors == required_anchors else 'FAIL'}); "
            f"allowed x=[{_fmt(metrics.get('allowed_anchor_x_min'))}, "
            f"{_fmt(metrics.get('allowed_anchor_x_max'))}] m."
        )
    else:
        lines.append("- Ground-anchor constraint: unavailable.")
    lines.append(
        f"- Initial span/height design check: "
        f"{'PASS' if metrics.get('initial_span_check_passed') else 'FAIL'}; "
        f"integrity: {'FAIL' if metrics.get('structure_broken') else 'PASS'}; "
        f"intact joints/beams {_integer(metrics.get('joint_count'))}/"
        f"{_integer(metrics.get('body_count'))} from "
        f"{_integer(metrics.get('initial_joint_count'))}/"
        f"{_integer(metrics.get('initial_body_count'))}."
    )
    violations = metrics.get("constraint_violations")
    if isinstance(violations, list) and violations:
        lines.append(
            "- Build-time violations: "
            + "; ".join(str(item) for item in violations[:6])
            + "."
        )
    lines.extend(
        [
            "### Energy applicability",
            (
                "- No energy budget is part of E-06 grading; measured joint "
                "loads, accumulated damage, and beam rotation are the applicable "
                "endurance diagnostics."
            ),
            "### Numerical health",
        ]
    )

    invalid = []
    for key in (
        "step_count",
        "initial_structure_mass",
        "structure_mass",
        "max_joint_force",
        "max_joint_torque",
        "max_joint_damage",
        "peak_body_angvel",
        "current_min_x",
        "current_max_x",
        "current_max_y",
    ):
        if key in metrics and metrics.get(key) is not None and _number(metrics.get(key)) is None:
            invalid.append(key)
    lines.append(
        "- Invalid or non-finite observations: " + ", ".join(invalid) + "."
        if invalid
        else "- All reported scalar observations are finite."
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
    del metrics, score, success, failed, failure_reason
    if error:
        return [f"- Resolve the reported execution error: {error}"]
    return []
