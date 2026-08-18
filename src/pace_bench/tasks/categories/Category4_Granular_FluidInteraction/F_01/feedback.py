"""Deterministic, observation-only feedback for F-01."""

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


def _constraint_lines(metrics: Dict[str, Any]) -> List[str]:
    violations = metrics.get("constraint_violations")
    if not isinstance(violations, list) or not violations:
        return ["- Design validation: no reported violations."]
    lines = [f"- Design validation: {len(violations)} violation(s)."]
    lines.extend(f"  {index}. {value}" for index, value in enumerate(violations, 1))
    return lines


def _coverage_lines(metrics: Dict[str, Any]) -> List[str]:
    coverage = metrics.get("beam_coverage_envelope")
    if not isinstance(coverage, dict) or not coverage:
        return ["- Strip coverage observations unavailable."]
    lines = []
    for name in ("left", "middle", "right"):
        data = coverage.get(name)
        if not isinstance(data, dict):
            lines.append(f"- {name.title()} strip: unavailable.")
            continue
        lines.append(
            f"- {name.title()} strip: {data.get('beam_count', 'unavailable')} beams; "
            f"covered vertical span {_fmt(data.get('coverage_span'))} m."
        )
    return lines


def _joint_lines(metrics: Dict[str, Any]) -> List[str]:
    lines = [
        f"- Structure intact: {not bool(metrics.get('structure_broken'))}; "
        f"joints {metrics.get('joint_count', 'unavailable')} / initial "
        f"{metrics.get('initial_joint_count', 'unavailable')}.",
        f"- Weld limit: {_fmt(metrics.get('joint_force_limit'))} N for "
        f"{metrics.get('joint_break_consecutive_steps', 'unavailable')} consecutive steps.",
    ]
    events = metrics.get("joint_break_events")
    if isinstance(events, list) and events:
        lines.append(f"- Observed weld failures: {len(events)}.")
        for event in events[:10]:
            if not isinstance(event, dict):
                lines.append("  - Malformed weld-failure observation.")
                continue
            anchor = event.get("anchor")
            if isinstance(anchor, (list, tuple)) and len(anchor) >= 2:
                anchor_text = f"({_fmt(anchor[0])}, {_fmt(anchor[1])})"
            else:
                anchor_text = "unavailable"
            lines.append(
                f"  - Step {event.get('step', 'unavailable')}: anchor {anchor_text}; "
                f"reaction force {_fmt(event.get('force'))} N."
            )
    peaks = metrics.get("joint_peak_forces")
    if isinstance(peaks, list) and peaks:
        finite_peaks = [
            _number(item.get("peak_force"))
            for item in peaks
            if isinstance(item, dict)
        ]
        finite_peaks = [value for value in finite_peaks if value is not None]
        if finite_peaks:
            lines.append(f"- Highest observed weld reaction: {max(finite_peaks):.3f} N.")
    return lines


def format_task_metrics(metrics: Dict[str, Any]) -> List[str]:
    if not isinstance(metrics, dict) or not metrics:
        return ["**Outcome**: unavailable — no metrics were provided."]
    lines = [
        "## F-01 Dam Feedback",
        _outcome(metrics),
        "### Containment observations",
        f"- Leakage: {_fmt(metrics.get('leakage_rate_percent'))}% / "
        f"{_fmt(metrics.get('leakage_limit_percent'))}% limit; containment "
        f"{_fmt(metrics.get('containment_percent'))}%.",
        f"- Particles: {metrics.get('initial_particle_count', 'unavailable')} initial; "
        f"{_fmt(metrics.get('leaked_particle_count'))} weighted leaked; "
        f"{metrics.get('current_particle_count', 'unavailable')} currently active.",
        "### Structural observations",
        f"- Beams: {metrics.get('beam_count', 'unavailable')} within required "
        f"[{metrics.get('min_beam_count', 'unavailable')}, "
        f"{metrics.get('max_beam_count', 'unavailable')}].",
        f"- Mass: {_fmt(metrics.get('structure_mass'))} / "
        f"{_fmt(metrics.get('max_structure_mass'))} kg.",
    ]
    lines.extend(_joint_lines(metrics))
    lines.append("### Build-strip observations")
    lines.extend(_coverage_lines(metrics))
    lines.append("### Design validation")
    lines.extend(_constraint_lines(metrics))
    lines.append("### Data health")
    health = metrics.get("numerical_health_warnings")
    errors = metrics.get("observation_errors")
    if isinstance(health, list) and health:
        lines.append(f"- Numerical warnings: {len(health)}.")
        lines.extend(f"  - {entry}" for entry in health[:10])
    if isinstance(errors, list) and errors:
        lines.append(f"- Observation errors: {len(errors)}.")
        lines.extend(f"  - {entry}" for entry in errors[:10])
    if not (isinstance(health, list) and health) and not (
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
