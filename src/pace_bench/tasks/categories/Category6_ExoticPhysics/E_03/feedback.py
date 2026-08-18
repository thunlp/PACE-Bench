from __future__ import annotations

import math
from typing import Any, Dict, List, Optional


def _number(value: Any) -> Optional[float]:
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return result


def _finite(value: Any) -> Optional[float]:
    result = _number(value)
    return result if result is not None and math.isfinite(result) else None


def _fmt(value: Any, decimals: int = 3) -> str:
    result = _number(value)
    if result is None:
        return "N/A"
    if not math.isfinite(result):
        return "non-finite"
    return f"{result:.{decimals}f}"


def _status(value: Any) -> str:
    return "PASS" if bool(value) else "FAIL"


def _zone_margin(x, y, bounds):
    x_min, x_max, y_min, y_max = bounds
    if not all(value is not None for value in bounds):
        return "bounds unavailable"
    details: List[str] = []
    if x is None:
        details.append("x unavailable")
    elif x < x_min:
        details.append(f"{x_min - x:.3f} m before x-range")
    elif x > x_max:
        details.append(f"{x - x_max:.3f} m beyond x-range")
    else:
        details.append("x in range")
    if y is None:
        details.append("y unavailable")
    elif y < y_min:
        details.append(f"{y_min - y:.3f} m below y-range")
    elif y > y_max:
        details.append(f"{y - y_max:.3f} m above y-range")
    else:
        details.append("y in range")
    return ", ".join(details)


def _objective_bounds(metrics, prefix):
    return tuple(
        _finite(metrics.get(f"{prefix}_{suffix}"))
        for suffix in ("x_lo", "x_hi", "y_lo", "y_hi")
    )


def _target_bounds(metrics):
    return tuple(
        _finite(metrics.get(key))
        for key in (
            "target_x_min",
            "target_x_max",
            "target_y_min",
            "target_y_max",
        )
    )


def _event_line(label: str, event: Any) -> str:
    if not isinstance(event, dict) or not bool(event.get("entered")):
        return f"- {label}: not entered"
    entry_step = event.get("entry_step", "N/A")
    x = _fmt(event.get("x_at_entry"))
    y = _fmt(event.get("y_at_entry"))
    speed = _fmt(event.get("speed_at_entry"))
    line = (
        f"- {label}: entered at step {entry_step}, position=({x}, {y}) m, "
        f"speed={speed} m/s"
    )
    if bool(event.get("exited")):
        line += (
            f"; first exit at step {event.get('exit_step', 'N/A')}, "
            f"speed={_fmt(event.get('speed_at_exit'))} m/s"
        )
    return line


def _format_outcome(metrics: Dict[str, Any]) -> List[str]:
    step = metrics.get("step_count", "N/A")
    limit = metrics.get("max_steps", "N/A")
    if bool(metrics.get("success")):
        outcome = "PASS"
    elif bool(metrics.get("failed")):
        outcome = "FAIL"
    else:
        outcome = "INCOMPLETE"
    lines = [
        "## E-03 Slippery World — measured run report",
        f"**Outcome:** {outcome} at step {step} / {limit}",
    ]
    reason = metrics.get("failure_reason")
    if reason:
        lines.append(f"**Recorded stop reason:** {reason}")
    return lines + [""]


def _format_chronology(metrics: Dict[str, Any]) -> List[str]:
    zones = metrics.get("zone_forensics")
    zones = zones if isinstance(zones, dict) else {}
    lines = [
        "### Objective chronology",
        _event_line("Checkpoint Alpha", zones.get("checkpoint_a")),
        _event_line("Checkpoint Beta", zones.get("checkpoint_b")),
        _event_line("Final target", zones.get("target_zone")),
    ]
    furthest_x = _finite(zones.get("furthest_x"))
    if furthest_x is not None:
        lines.append(
            f"- Furthest measured x: {furthest_x:.3f} m at step "
            f"{zones.get('furthest_x_step', 'N/A')}"
        )
    return lines + [""]


def _format_objective_proximity(metrics: Dict[str, Any]) -> List[str]:
    x = _finite(metrics.get("sled_x"))
    y = _finite(metrics.get("sled_y"))
    vx = _finite(metrics.get("velocity_x"))
    vy = _finite(metrics.get("velocity_y"))
    lines = [
        "### Terminal state and objective proximity",
        (
            f"- Position: ({_fmt(x)}, {_fmt(y)}) m; velocity: "
            f"({_fmt(vx)}, {_fmt(vy)}) m/s; speed: "
            f"{_fmt(metrics.get('velocity_magnitude'))} m/s"
        ),
    ]
    objectives = (
        (
            "Checkpoint Alpha",
            "checkpoint_a_reached",
            _objective_bounds(metrics, "checkpoint_a"),
        ),
        (
            "Checkpoint Beta",
            "checkpoint_b_reached",
            _objective_bounds(metrics, "checkpoint_b"),
        ),
        ("Final target", "reached_target", _target_bounds(metrics)),
    )
    for label, status_key, bounds in objectives:
        lines.append(
            f"- {label}: {_status(metrics.get(status_key))}; "
            f"terminal margin: {_zone_margin(x, y, bounds)}"
        )

    zones = metrics.get("zone_forensics")
    closest = (
        zones.get("closest_objective_distance")
        if isinstance(zones, dict)
        else None
    )
    if isinstance(closest, dict):
        for key, label in (
            ("checkpoint_a", "Alpha"),
            ("checkpoint_b", "Beta"),
            ("target_zone", "target"),
        ):
            record = closest.get(key)
            if isinstance(record, dict):
                lines.append(
                    f"- Closest measured distance to {label}: "
                    f"{_fmt(record.get('distance'))} m at step "
                    f"{record.get('step', 'N/A')}"
                )
    distance = _finite(metrics.get("distance_to_target"))
    progress = _finite(metrics.get("progress_pct"))
    if distance is not None or progress is not None:
        lines.append(
            f"- Terminal target distance: {_fmt(distance)} m; "
            f"x-progress from start to target entrance: {_fmt(progress, 1)}%"
        )
    return lines + [""]


def _format_action_effect(metrics: Dict[str, Any]) -> List[str]:
    thrust = metrics.get("thrust_forensics")
    if not isinstance(thrust, dict):
        return ["### Command/effect observations", "- No thrust measurements.", ""]
    lines = [
        "### Command and motion observations",
        (
            f"- Last commanded force: ({_fmt(thrust.get('commanded_fx'))}, "
            f"{_fmt(thrust.get('commanded_fy'))}) N; magnitude "
            f"{_fmt(thrust.get('commanded_magnitude'))} N"
        ),
        (
            f"- Largest commanded magnitude observed in this run: "
            f"{_fmt(thrust.get('peak_commanded_thrust'))} N"
        ),
    ]
    lines.append(
        f"- Terminal measured velocity: "
        f"({_fmt(metrics.get('velocity_x'))}, "
        f"{_fmt(metrics.get('velocity_y'))}) m/s"
    )
    near_peak = thrust.get("near_running_peak_command_steps")
    total = thrust.get("total_steps")
    if isinstance(near_peak, (int, float)) and isinstance(total, (int, float)):
        lines.append(
            f"- Commands within 98% of the running command peak: "
            f"{int(near_peak)} / {int(total)} steps"
        )
    return lines + [""]


def _format_constraints(metrics: Dict[str, Any]) -> List[str]:
    step = _finite(metrics.get("step_count"))
    limit = _finite(metrics.get("max_steps"))
    time_ok = step is not None and limit is not None and step <= limit
    rows = [
        ("Checkpoint Alpha entered", metrics.get("checkpoint_a_reached")),
        ("Checkpoint Beta entered after Alpha", metrics.get("checkpoint_b_reached")),
        ("Both checkpoints completed in order", metrics.get("checkpoint_reached")),
        ("Target entered after both checkpoints", metrics.get("reached_target")),
        ("Within simulation-step limit", time_ok),
    ]
    lines = [
        "### Constraint status",
        "| Constraint | Status |",
        "|---|---|",
    ]
    lines.extend(f"| {label} | {_status(value)} |" for label, value in rows)
    return lines + [""]


def _format_health(metrics: Dict[str, Any]) -> List[str]:
    lines = ["### Numerical and motion health"]
    health = metrics.get("numerical_health")
    if isinstance(health, dict):
        non_finite = health.get("non_finite_fields")
        if bool(health.get("all_finite")):
            lines.append("- Evaluator state fields checked as finite.")
        elif isinstance(non_finite, list) and non_finite:
            lines.append(
                "- Non-finite evaluator fields: "
                + ", ".join(str(item) for item in non_finite)
            )
        else:
            lines.append("- Numerical-health check did not complete.")
    else:
        lines.append("- Numerical-health data unavailable.")

    zones = metrics.get("zone_forensics")
    if isinstance(zones, dict):
        lines.append(
            f"- Peak measured speed: "
            f"{_fmt(zones.get('peak_systemic_speed'))} m/s"
        )
    stuck = metrics.get("stuck_forensics")
    if isinstance(stuck, dict):
        lines.append(
            f"- Longest consecutive near-zero-speed interval: "
            f"{stuck.get('longest_stuck_duration', 'N/A')} steps, first at "
            f"step {stuck.get('longest_stuck_start_step', 'N/A')} near "
            f"({_fmt(stuck.get('longest_stuck_x'))}, "
            f"{_fmt(stuck.get('longest_stuck_y'))}) m"
        )
        lines.append(
            f"- Near-zero-speed interval active at termination: "
            f"{bool(stuck.get('still_stuck_at_end'))}"
        )
    return lines + [""]


def format_task_metrics(metrics: Dict[str, Any]) -> List[str]:
    if not isinstance(metrics, dict) or not metrics:
        return ["**No metrics available.**"]
    if "error" in metrics:
        return [f"**Evaluator state:** {metrics.get('error')}"]
    lines: List[str] = []
    for formatter in (
        _format_outcome,
        _format_chronology,
        _format_objective_proximity,
        _format_action_effect,
        _format_constraints,
        _format_health,
    ):
        lines.extend(formatter(metrics))
    return lines


def get_improvement_suggestions(
    metrics: Dict[str, Any],
    score: float,
    success: bool,
    failed: bool,
    failure_reason: Optional[str] = None,
    error: Optional[str] = None,
) -> List[str]:
    return []
