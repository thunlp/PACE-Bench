"""Compact, deterministic, non-privileged feedback for E-01."""

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


def _margin(value: Any, label: str) -> str:
    margin = _number(value)
    if margin is None:
        return f"{label}: unavailable"
    status = "PASS" if margin >= 0.0 else "FAIL"
    return f"{label}: {status}, margin {margin:+.3f} m"


def _failure_timeline(metrics: Dict[str, Any]) -> List[str]:
    lines = [
        f"- First fixture-boundary exit: "
        f"step {_integer(metrics.get('first_out_of_bounds_step'))}.",
        f"- First observed joint-count loss: "
        f"step {_integer(metrics.get('first_structure_break_step'))}.",
    ]
    events = metrics.get("joint_failure_events")
    if not isinstance(events, list) or not events:
        lines.append("- Measured joint-break events: none recorded.")
        return lines

    summaries: List[str] = []
    for event in events[:4]:
        if not isinstance(event, dict):
            continue
        summaries.append(
            f"step {_integer(event.get('step'))} at "
            f"({_fmt(event.get('anchor_x'))}, {_fmt(event.get('anchor_y'))}) m, "
            f"measured reaction {_fmt(event.get('force_at_break'))} N"
        )
    lines.append(
        "- Measured joint-break chronology: "
        + ("; ".join(summaries) if summaries else "unavailable")
        + (f"; {len(events) - 4} later event(s) omitted" if len(events) > 4 else "")
        + "."
    )
    return lines


def format_task_metrics(
    metrics: Dict[str, Any],
    previous_metrics: Optional[Dict[str, Any]] = None,
) -> List[str]:
    del previous_metrics
    if not isinstance(metrics, dict) or not metrics:
        return ["## E-01 Containment Feedback", "- Outcome: unavailable — no metrics provided."]

    mass = _number(metrics.get("structure_mass"))
    mass_limit = _number(metrics.get("max_structure_mass"))
    beam_count = _number(metrics.get("beam_count"))
    beam_limit = _number(metrics.get("max_beam_count"))
    joint_count = _number(metrics.get("joint_count"))
    initial_joints = _number(metrics.get("initial_joint_count"))

    lines = [
        "## E-01 Containment Feedback",
        _outcome(metrics),
        "### Event chronology",
        *_failure_timeline(metrics),
        "### Spatial state",
        (
            f"- Dynamic-fixture extent: x=[{_fmt(metrics.get('body_x_min'))}, "
            f"{_fmt(metrics.get('body_x_max'))}] m, "
            f"y=[{_fmt(metrics.get('body_y_min'))}, "
            f"{_fmt(metrics.get('body_y_max'))}] m."
        ),
        (
            f"- Arena bounds: x=[{_fmt(metrics.get('arena_x_min'))}, "
            f"{_fmt(metrics.get('arena_x_max'))}] m, "
            f"y=[{_fmt(metrics.get('arena_y_min'))}, "
            f"{_fmt(metrics.get('arena_y_max'))}] m; "
            f"worst observed fixture margin "
            f"{_fmt(metrics.get('minimum_arena_margin'))} m."
        ),
        "- "
        + "; ".join(
            (
                _margin(
                    metrics.get("forbidden_zone_min_margin"),
                    "nearest forbidden-zone center",
                ),
                _margin(
                    metrics.get("obstacle_zone_min_margin"),
                    "nearest obstacle-zone center",
                ),
                _margin(
                    metrics.get("build_zone_tightest_margin"),
                    "tightest build-time beam-center",
                ),
            )
        )
        + ".",
        "### Constraint and load state",
    ]

    if mass is not None and mass_limit is not None:
        lines.append(
            f"- Structure mass: {mass:.6f}/{mass_limit:.6f} kg; "
            f"margin {mass_limit - mass:+.6f} kg."
        )
    else:
        lines.append("- Structure mass constraint: unavailable.")
    if beam_count is not None and beam_limit is not None:
        lines.append(
            f"- Beam count: {int(beam_count)}/{int(beam_limit)}; "
            f"margin {int(beam_limit - beam_count):+d}."
        )
    else:
        lines.append("- Beam-count constraint: unavailable.")
    if joint_count is not None and initial_joints is not None:
        lines.append(
            f"- Intact agent joints: {int(joint_count)}/{int(initial_joints)}; "
            f"observed losses {int(initial_joints - joint_count)}."
        )
    else:
        lines.append("- Agent-joint integrity: unavailable.")

    latest = metrics.get("latest_joint_force_summary")
    if isinstance(latest, dict):
        lines.append(
            f"- Latest measured joint reactions: maximum "
            f"{_fmt(latest.get('max_force'), 6)} N across "
            f"{_integer(latest.get('joint_count_at_step'))} joint(s) at "
            f"step {_integer(latest.get('step'))}; rollout peak "
            f"{_fmt(metrics.get('peak_reaction_force_ever'), 6)} N."
        )
    else:
        lines.append(
            f"- Latest joint-reaction summary: unavailable; rollout peak "
            f"{_fmt(metrics.get('peak_reaction_force_ever'), 6)} N."
        )

    lines.extend(
        [
            "### Energy and numerical health",
            (
                f"- Kinetic energy: initial "
                f"{_fmt(metrics.get('kinetic_energy_initial'))} J, current "
                f"{_fmt(metrics.get('kinetic_energy_current'))} J, observed peak "
                f"{_fmt(metrics.get('kinetic_energy_peak'))} J."
            ),
            (
                f"- Observed peak body speed: "
                f"{_fmt(metrics.get('peak_body_velocity'))} m/s."
            ),
        ]
    )

    invalid = []
    for key in (
        "step_count",
        "structure_mass",
        "beam_count",
        "joint_count",
        "body_x_min",
        "body_x_max",
        "body_y_min",
        "body_y_max",
        "peak_reaction_force_ever",
        "peak_body_velocity",
        "kinetic_energy_current",
    ):
        if key in metrics and metrics.get(key) is not None and _number(metrics.get(key)) is None:
            invalid.append(key)
    lines.append(
        "- Invalid or non-finite reported fields: " + ", ".join(invalid) + "."
        if invalid
        else "- All reported numeric observations are finite."
    )
    violations = metrics.get("constraint_violations")
    if isinstance(violations, list) and violations:
        lines.append(
            "- Build-time violations: "
            + "; ".join(str(item) for item in violations[:5])
            + "."
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
