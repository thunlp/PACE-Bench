"""Compact, deterministic, non-prescriptive feedback for E-05."""

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


def _event_line(metrics: Dict[str, Any]) -> str:
    events = metrics.get("temporal_events")
    if not isinstance(events, list) or not events:
        return "- Recorded path events: none."
    entries = []
    for event in events[:5]:
        if not isinstance(event, dict):
            continue
        raw_name = str(event.get("event", "event"))
        name = {
            "ceiling_entry": "upper diagnostic band entry",
            "ground_entry": "ground band entry",
            "pit_entry": "forbidden pit entry",
            "progress_plateau_detected": "forward-progress plateau detected",
        }.get(raw_name, raw_name.replace("_", " "))
        entries.append(
            f"step {_integer(event.get('step'))}: {name} at "
            f"({_fmt(event.get('body_x'))}, {_fmt(event.get('body_y'))}) m"
        )
    return (
        "- Recorded path chronology: "
        + ("; ".join(entries) if entries else "unavailable")
        + (f"; {len(events) - 5} later event(s) omitted" if len(events) > 5 else "")
        + "."
    )


def format_task_metrics(
    metrics: Dict[str, Any],
    previous_metrics: Optional[Dict[str, Any]] = None,
) -> List[str]:
    del previous_metrics
    if not isinstance(metrics, dict) or not metrics:
        return ["## E-05 Navigation Feedback", "- Outcome: unavailable — no metrics provided."]
    if metrics.get("error"):
        return [
            "## E-05 Navigation Feedback",
            f"- Outcome: evaluation error — {metrics.get('error')}.",
        ]

    target_x_margin = _number(metrics.get("target_x_margin"))
    target_y_margin = _number(metrics.get("target_y_margin"))
    pit_margin = _number(metrics.get("pit_zone_margin"))
    thrust = _number(metrics.get("thrust_magnitude"))
    thrust_cap = _number(metrics.get("max_thrust"))
    clipped = _number(metrics.get("thrust_clipped_steps"))
    step = _number(metrics.get("step_count"))
    progress = _number(metrics.get("progress_x"))
    progress_pct = progress * 100.0 if progress is not None else None

    lines = [
        "## E-05 Navigation Feedback",
        _outcome(metrics),
        "### Event chronology",
        f"- First target-zone entry: step "
        f"{_integer(metrics.get('first_target_entry_step'))}; first pit entry: "
        f"step {_integer(metrics.get('first_pit_entry_step'))}.",
        f"- First upper diagnostic-band entry: step "
        f"{_integer(metrics.get('first_ceiling_entry_step'))}; first ground-band "
        f"entry: step {_integer(metrics.get('first_ground_entry_step'))}.",
        _event_line(metrics),
        (
            f"- First detected 300-step forward-progress plateau: step "
            f"{_integer(metrics.get('progress_plateau_end_step'))}, at x="
            f"{_fmt(metrics.get('progress_plateau_x'))} m; longest measured "
            f"plateau run {_integer(metrics.get('progress_plateau_duration'))} steps."
        ),
        "### Spatial and motion state",
        (
            f"- Body center: ({_fmt(metrics.get('body_x'))}, "
            f"{_fmt(metrics.get('body_y'))}) m; velocity "
            f"({_fmt(metrics.get('velocity_x'))}, "
            f"{_fmt(metrics.get('velocity_y'))}) m/s."
        ),
        (
            f"- Distance to target rectangle: "
            f"{_fmt(metrics.get('dist_to_target'))} m; target-axis margins "
            f"x={_fmt(target_x_margin)} m, y={_fmt(target_y_margin)} m "
            f"(negative means outside)."
        ),
        (
            f"- Forward progress: {_fmt(progress_pct, 1)}%; "
            f"maximum observed x={_fmt(metrics.get('max_x_reached'))} m."
        ),
        (
            f"- Forbidden-pit margin: {_fmt(pit_margin)} m "
            f"({'PASS' if pit_margin is not None and pit_margin >= 0.0 else 'FAIL' if pit_margin is not None else 'unavailable'}); "
            f"observed path extent x=[{_fmt(metrics.get('min_body_x'))}, "
            f"{_fmt(metrics.get('max_body_x'))}] m, "
            f"y=[{_fmt(metrics.get('min_body_y'))}, "
            f"{_fmt(metrics.get('max_body_y'))}] m."
        ),
        "### Loads and actuation",
        (
            f"- Current measured net field force: "
            f"({_fmt(metrics.get('net_magnetic_force_x'))}, "
            f"{_fmt(metrics.get('net_magnetic_force_y'))}) N; observed peak "
            f"magnitude {_fmt(metrics.get('peak_magnetic_force_magnitude'))} N."
        ),
    ]

    if thrust is not None and thrust_cap is not None:
        lines.append(
            f"- Applied thrust magnitude: {thrust:.3f}/{thrust_cap:.3f} N; "
            f"live capacity margin {thrust_cap - thrust:+.3f} N."
        )
    else:
        lines.append("- Applied thrust and live engine-cap comparison: unavailable.")
    lines.append(
        f"- Peak requested/applied thrust: "
        f"{_fmt(metrics.get('peak_requested_thrust'))}/"
        f"{_fmt(metrics.get('peak_applied_thrust'))} N; clipped-command steps "
        f"{_integer(clipped)}, first at step "
        f"{_integer(metrics.get('first_thrust_clip_step'))}."
    )
    lines.extend(
        [
            (
                f"- Public gravity load on the body: "
                f"({_fmt(metrics.get('gravity_force_x'))}, "
                f"{_fmt(metrics.get('gravity_force_y'))}) N; hover-equivalent "
                f"magnitude {_fmt(metrics.get('required_hover_thrust'))} N."
            ),
            "### Energy observations",
            (
                f"- Current kinetic energy: "
                f"{_fmt(metrics.get('kinetic_energy'))} J; accumulated thrust "
                f"work {_fmt(metrics.get('cumulative_thrust_work'))} J; "
                f"accumulated field work "
                f"{_fmt(metrics.get('cumulative_magnetic_work'))} J."
            ),
            "### Published constraints",
            (
                f"- Target rectangle: "
                f"{'PASS' if metrics.get('reached_target') else 'NOT YET'}; "
                f"forbidden pit: {'FAIL' if metrics.get('in_pit_zone') else 'PASS'}."
            ),
            (
                f"- Applied-thrust cap enforcement: "
                f"{'PASS' if thrust is not None and thrust_cap is not None and thrust <= thrust_cap + 1e-9 else 'unavailable'}; "
                f"step budget remaining "
                f"{_integer(max(0.0, (_number(metrics.get('max_steps')) or 0.0) - (step or 0.0)))}."
            ),
            "### Numerical health",
        ]
    )

    invalid = []
    for key in (
        "body_x",
        "body_y",
        "velocity_x",
        "velocity_y",
        "speed",
        "dist_to_target",
        "pit_zone_margin",
        "net_magnetic_force_x",
        "net_magnetic_force_y",
        "thrust_magnitude",
        "kinetic_energy",
    ):
        if key in metrics and metrics.get(key) is not None and _number(metrics.get(key)) is None:
            invalid.append(key)
    if invalid:
        lines.append("- Invalid or non-finite observations: " + ", ".join(invalid) + ".")
    else:
        lines.append(
            f"- All reported numeric observations are finite; peak speed "
            f"{_fmt(metrics.get('max_speed'))} m/s and peak vertical "
            f"acceleration {_fmt(metrics.get('peak_vertical_accel'))} m/s²."
        )
    lines.append(
        f"- Measured velocity reversals: "
        f"{_integer(metrics.get('velocity_reversal_count_x'))} horizontal, "
        f"{_integer(metrics.get('velocity_reversal_count_y'))} vertical."
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
