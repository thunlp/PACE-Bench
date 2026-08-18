from __future__ import annotations

import math

from typing import Any, Dict, List


def _dict(value: Any) -> Dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _list(value: Any) -> List[Any]:
    return value if isinstance(value, list) else []


def _number(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return result if math.isfinite(result) else None


def _fmt(value: Any, decimals: int = 2) -> str:
    number = _number(value)
    if number is None:
        return "unavailable"
    text = f"{number:.{decimals}f}".rstrip("0").rstrip(".")
    return text if text else "0"


def _count(value: Any) -> str:
    number = _number(value)
    return str(int(number)) if number is not None and number.is_integer() else _fmt(value)


def _append_timeline(parts: List[str], metrics: Dict[str, Any]) -> None:
    parts.append("### Timeline")
    step = _number(metrics.get("step_count"))
    maximum = _number(metrics.get("max_steps"))
    progress = _number(metrics.get("progress_pct"))
    line = f"- Snapshot: step {_count(step)}"
    if maximum is not None:
        line += f"/{_count(maximum)}"
    if progress is not None:
        line += f" ({_fmt(progress, 1)}%)"
    parts.append(line)

    active = _number(metrics.get("spawned_particle_count"))
    planned = _number(metrics.get("planned_total_particle_count"))
    if active is not None:
        line = f"- Active particles: {_count(active)}"
        if planned is not None:
            line += f"/{_count(planned)} planned"
        parts.append(line)

    event_by_wave: Dict[int, Dict[str, Any]] = {}
    for raw_event in _list(metrics.get("wave_events")):
        event = _dict(raw_event)
        wave = _number(event.get("wave"))
        if wave is not None:
            event_by_wave[int(wave)] = event
    for wave, schedule_key in ((2, "second_wave_step"), (3, "third_wave_step")):
        event = event_by_wave.get(wave)
        if event:
            line = f"- Wave {wave}: triggered at step {_count(event.get('step'))}"
            spawned = _number(event.get("spawned_count"))
            active_after = _number(event.get("active_count"))
            if spawned is not None:
                line += f", spawned {_count(spawned)}"
            if active_after is not None:
                line += f", active afterward {_count(active_after)}"
            parts.append(line)
            continue
        schedule = _number(metrics.get(schedule_key))
        if schedule is not None:
            state = "pending" if step is None or step < schedule else "not recorded"
            parts.append(f"- Wave {wave}: {state} at scheduled step {_count(schedule)}")


def _append_classification(parts: List[str], metrics: Dict[str, Any]) -> None:
    parts.append("### Classification snapshot")
    purity = _number(metrics.get("purity_percent"))
    target = _number(metrics.get("min_purity_percent"))
    if purity is not None:
        line = f"- Purity: {_fmt(purity, 1)}%"
        if target is not None:
            margin = purity - target
            line += f" against {_fmt(target, 1)}% target (margin {_fmt(margin, 1)} pp)"
        parts.append(line)

    class_rows = (
        ("Small", "small_in_small_zone", "small"),
        ("Medium", "medium_in_medium_zone", "medium"),
        ("Large", "large_in_large_zone", "large"),
    )
    y_stats = _dict(metrics.get("particle_y_stats"))
    correct_values: List[float] = []
    for display, correct_key, stats_key in class_rows:
        correct = _number(metrics.get(correct_key))
        stats = _dict(y_stats.get(stats_key))
        class_total = _number(stats.get("count"))
        if correct is not None:
            correct_values.append(correct)
            line = f"- {display}: {_count(correct)} correctly classified"
            if class_total is not None:
                line += f"/{_count(class_total)} active"
            parts.append(line)

    active = _number(metrics.get("spawned_particle_count"))
    if active is not None and len(correct_values) == 3:
        correct_total = sum(correct_values)
        parts.append(
            f"- Total: {_count(correct_total)}/{_count(active)} correct; "
            f"{_count(max(0.0, active - correct_total))} misrouted"
        )

    contamination = metrics.get("contaminated")
    if isinstance(contamination, bool):
        feed_y = _number(metrics.get("feed_y_min"))
        suffix = f" below feed y={_fmt(feed_y)} m" if feed_y is not None else ""
        parts.append(
            f"- Cross-zone contamination diagnostic{suffix}: "
            f"{'detected' if contamination else 'none'} (not a separate score gate)"
        )


def _append_observations(parts: List[str], metrics: Dict[str, Any]) -> None:
    parts.append("### Observable particle state")
    zones = _dict(metrics.get("zone_boundaries"))
    zone_values = [
        _number(zones.get("small_zone_y_max")),
        _number(zones.get("medium_zone_y_min")),
        _number(zones.get("medium_zone_y_max")),
        _number(zones.get("large_zone_y_min")),
    ]
    if all(value is not None for value in zone_values):
        small_max, medium_min, medium_max, large_min = zone_values
        parts.append(
            f"- Zone boundaries: small y<{_fmt(small_max)}; "
            f"medium {_fmt(medium_min)}≤y<{_fmt(medium_max)}; "
            f"large y≥{_fmt(large_min)}"
        )

    y_stats = _dict(metrics.get("particle_y_stats"))
    velocity_stats = _dict(metrics.get("particle_velocity_stats"))
    emitted = False
    for label in ("small", "medium", "large"):
        ys = _dict(y_stats.get(label))
        vs = _dict(velocity_stats.get(label))
        count = _number(ys.get("count"))
        if count is None:
            count = _number(vs.get("count"))
        if count is None:
            continue
        emitted = True
        line = f"- {label.capitalize()} ({_count(count)} active)"
        if count > 0 and any(_number(ys.get(key)) is not None for key in ("min", "median", "max")):
            line += (
                f": y min/median/max={_fmt(ys.get('min'))}/"
                f"{_fmt(ys.get('median'))}/{_fmt(ys.get('max'))} m"
            )
        if count > 0 and any(_number(vs.get(key)) is not None for key in ("median", "max")):
            line += (
                f"; speed median/max={_fmt(vs.get('median'))}/"
                f"{_fmt(vs.get('max'))} m/s"
            )
        parts.append(line)
    if not emitted:
        parts.append("- Particle position and speed summaries unavailable")


def _append_constraints(parts: List[str], metrics: Dict[str, Any]) -> None:
    parts.append("### Construction and integrity")
    mass = _number(metrics.get("structure_mass"))
    max_mass = _number(metrics.get("max_structure_mass"))
    if mass is not None and max_mass is not None:
        parts.append(
            f"- Structure mass: {_fmt(mass, 3)}/{_fmt(max_mass, 3)} kg "
            f"(margin {_fmt(max_mass - mass, 3)} kg)"
        )
    beams = _number(metrics.get("beam_count"))
    max_beams = _number(metrics.get("max_beams"))
    if beams is not None and max_beams is not None:
        parts.append(
            f"- Beam count: {_count(beams)}/{_count(max_beams)} "
            f"(margin {_count(max_beams - beams)})"
        )

    margins: List[float] = []
    violations = 0
    for raw_margin in _list(metrics.get("beam_build_zone_margins")):
        record = _dict(raw_margin)
        for key in ("vertex_x_margin", "vertex_y_margin"):
            margin = _number(record.get(key))
            if margin is not None:
                margins.append(margin)
                if margin < 0:
                    violations += 1
    if margins:
        parts.append(
            f"- Build-zone footprint: worst vertex margin {_fmt(min(margins), 4)} m; "
            f"{violations} negative margin(s)"
        )
    elif beams == 0:
        parts.append("- Build-zone footprint: no beams placed")

    broken = metrics.get("structure_broken")
    joints = _number(metrics.get("joint_count"))
    initial_joints = _number(metrics.get("initial_joint_count"))
    if isinstance(broken, bool):
        line = f"- Structure integrity: {'lost' if broken else 'intact'}"
        if joints is not None:
            line += f"; joints {_count(joints)}"
            if initial_joints is not None:
                line += f"/{_count(initial_joints)} initial"
        break_step = _number(metrics.get("structure_break_step"))
        if break_step is not None:
            line += f"; first detected at step {_count(break_step)}"
        parts.append(line)


def _append_numerical(parts: List[str], metrics: Dict[str, Any]) -> None:
    health = _dict(metrics.get("numerical_health"))
    if not health:
        return
    parts.append("### Numerical snapshot")
    nan_flag = health.get("nan_detected") is True
    inf_flag = health.get("inf_detected") is True
    extreme_count = _number(health.get("extreme_velocity_count"))
    if extreme_count is None:
        extreme_count = float(len(_list(health.get("extreme_velocity_events"))))
    parts.append(
        f"- Non-finite state flags: NaN={'yes' if nan_flag else 'no'}, "
        f"Inf={'yes' if inf_flag else 'no'}"
    )
    parts.append(f"- Current active particles above 100 m/s: {_count(extreme_count)}")


def format_task_metrics(metrics: Dict[str, Any]) -> List[str]:
    """Format only observable outcomes and declared task constraints."""
    if not isinstance(metrics, dict):
        return ["**Evaluation error**: metrics must be a dictionary"]
    if "error" in metrics:
        return [
            "## F_04 evaluation error",
            f"- Error: {metrics.get('error')}",
            f"- Step: {_count(metrics.get('step_count'))}",
        ]

    violations = _list(metrics.get("constraint_violations"))
    if violations:
        parts = [
            "## F_04 build-phase result",
            "- Status: failed before simulation",
            f"- Step: {_count(metrics.get('step_count'))}",
        ]
        parts.extend(f"- Constraint violation: {violation}" for violation in violations)
        return parts

    parts: List[str] = ["## F_04 evaluation trace"]
    if metrics.get("failed") is True:
        parts.append(f"- Status: failed — {metrics.get('failure_reason') or 'reason unavailable'}")
    elif metrics.get("success") is True:
        parts.append("- Status: success")
    else:
        parts.append("- Status: simulation in progress")

    for appender in (
        _append_timeline,
        _append_classification,
        _append_observations,
        _append_constraints,
        _append_numerical,
    ):
        parts.append("")
        appender(parts, metrics)
    return parts


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
