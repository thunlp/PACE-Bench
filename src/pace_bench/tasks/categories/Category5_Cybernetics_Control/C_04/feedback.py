import math

from typing import Any, Dict, List


def reset_feedback_state():
    return None


def _number(value: Any):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _fmt(value: Any, decimals: int = 2) -> str:
    number = _number(value)
    return f"{number:.{decimals}f}" if number is not None else "N/A"


def _non_finite_paths(value: Any, path: str = "metrics") -> List[str]:
    bad: List[str] = []
    if isinstance(value, dict):
        for key, nested in value.items():
            bad.extend(_non_finite_paths(nested, f"{path}.{key}"))
    elif isinstance(value, (list, tuple)):
        for index, nested in enumerate(value):
            bad.extend(_non_finite_paths(nested, f"{path}[{index}]"))
    elif isinstance(value, float) and not math.isfinite(value):
        bad.append(path)
    return bad


def _section_outcome(metrics: Dict[str, Any]) -> List[str]:
    error = metrics.get("error") or metrics.get("error_message")
    if error:
        return ["### Outcome", "- ERROR", f"- Execution evidence: {error}"]
    success = metrics.get("success") is True
    failed = metrics.get("failed") is True
    reason = metrics.get("failure_reason")
    status = "FAIL" if failed or reason else "PASS" if success else "INCOMPLETE"
    parts = ["### Outcome", f"- {status}"]
    if reason:
        parts.append(f"- Decisive evidence: {reason}")
    elif success:
        parts.append("- Decisive evidence: unlock and exit-dwell requirements were satisfied.")
    else:
        parts.append("- Decisive evidence: no terminal result was reported.")
    return parts


def _section_chronology(metrics: Dict[str, Any]) -> List[str]:
    parts = ["\n### 1. Chronology"]
    step = metrics.get("step_count")
    maximum = metrics.get("max_steps")
    if step is not None:
        parts.append(f"- Evaluation ended at step {step}/{maximum}.")
    else:
        parts.append("- Evaluation end step unavailable.")

    timeline = metrics.get("diagnostic_timeline")
    if not isinstance(timeline, dict):
        parts.append("- Transport event timeline unavailable.")
        return parts
    events = []
    for key, label in (
        ("first_activation_entry_step", "first activation-zone entry"),
        ("first_unlock_step", "unlock achieved"),
        ("first_exit_entry_step", "first geometric exit-zone entry"),
        ("first_all_whiskers_max_step", "first simultaneous max-range whisker report"),
        ("destruction_step", "structural destruction"),
    ):
        event_step = timeline.get(key)
        if event_step is not None:
            events.append((int(event_step), label))
    qualified_exit = metrics.get("first_qualified_exit_step")
    if qualified_exit is not None:
        events.append((int(qualified_exit), "first post-unlock exit-dwell step"))
    hold_completion = metrics.get("exit_hold_completion_step")
    if hold_completion is not None:
        events.append((int(hold_completion), "exit-dwell requirement first satisfied"))
    if events:
        for event_step, label in sorted(events):
            parts.append(f"- Step {event_step}: {label}.")
    else:
        parts.append("- No tracked milestone was observed.")

    parts.append(
        f"- Furthest reported x: {_fmt(timeline.get('max_reported_x_m'))} m "
        f"at step {timeline.get('max_reported_x_step', 'N/A')}."
    )
    parts.append(
        f"- Closest exit-zone distance: {_fmt(timeline.get('closest_exit_distance_m'))} m "
        f"at step {timeline.get('closest_exit_distance_step', 'N/A')}."
    )
    return parts


def _section_spatial(metrics: Dict[str, Any]) -> List[str]:
    parts = ["\n### 2. Spatial and sensor state"]
    x = _number(metrics.get("agent_x"))
    y = _number(metrics.get("agent_y"))
    vx = _number(metrics.get("agent_vx"))
    vy = _number(metrics.get("agent_vy"))
    speed = math.hypot(vx, vy) if vx is not None and vy is not None else None
    parts.append(
        f"- Reported pose: ({_fmt(x)},{_fmt(y)}) m; velocity "
        f"({_fmt(vx)},{_fmt(vy)}) m/s; speed {_fmt(speed)} m/s."
    )
    parts.append(
        f"- Exit bounds: x>={_fmt(metrics.get('exit_x_min'))} m, "
        f"y=[{_fmt(metrics.get('exit_y_min'))},{_fmt(metrics.get('exit_y_max'))}] m; "
        f"current deficits dx={_fmt(metrics.get('distance_to_exit_x'))} m, "
        f"dy={_fmt(metrics.get('distance_y_to_exit_band'))} m."
    )
    parts.append(
        f"- Activation x-band: [{_fmt(metrics.get('activation_x_min'))},"
        f"{_fmt(metrics.get('activation_x_max'))}] m."
    )

    wall_map = metrics.get("wall_clearance_map")
    if isinstance(wall_map, dict) and x is not None:
        candidates = []
        for wall in wall_map.get("walls", []):
            position = wall.get("position") or {}
            wall_x = _number(position.get("x_min"))
            if wall_x is not None:
                candidates.append((abs(wall_x - x), wall))
        if candidates:
            _, wall = min(candidates, key=lambda item: item[0])
            position = wall.get("position") or {}
            relative = wall.get("agent_relative") or {}
            gaps = wall.get("gaps") or {}
            clearance = wall.get("clearance_needed_m") or {}
            parts.append(
                f"- Nearest tracked inner wall #{wall.get('wall_index', 'N/A')}: "
                f"x=[{_fmt(position.get('x_min'))},{_fmt(position.get('x_max'))}] m, "
                f"y=[{_fmt(position.get('y_min'))},{_fmt(position.get('y_max'))}] m; "
                f"forward distance={_fmt(relative.get('distance_to_wall_x'))} m; "
                f"boundary-to-wall gaps above/below={_fmt((gaps.get('above') or {}).get('size_m'))}/"
                f"{_fmt((gaps.get('below') or {}).get('size_m'))} m; "
                f"center-height deficits above/below="
                f"{_fmt(clearance.get('to_pass_above'))}/"
                f"{_fmt(clearance.get('to_pass_below'))} m."
            )

    max_range = metrics.get("whisker_max_range")
    readings = [
        metrics.get("whisker_front"), metrics.get("whisker_up"),
        metrics.get("whisker_down"),
    ]
    parts.append(
        f"- Whiskers front/up/down: {_fmt(readings[0])}/{_fmt(readings[1])}/"
        f"{_fmt(readings[2])} m (maximum range {_fmt(max_range)} m)."
    )
    numeric_readings = [_number(value) for value in readings]
    numeric_max = _number(max_range)
    if numeric_max is not None and all(value is not None for value in numeric_readings):
        all_at_max = all(abs(value - numeric_max) <= 1e-6 for value in numeric_readings)
        parts.append(f"- All three current whisker readings at maximum: {all_at_max}.")
    return parts


def _format_unlock_condition(condition: Dict[str, Any]) -> str:
    name = str(condition.get("name", "unnamed"))
    state = "PASS" if condition.get("pass") is True else "FAIL"
    value = condition.get("value")
    if isinstance(condition.get("zone"), (list, tuple)) and len(condition["zone"]) >= 2:
        zone = condition["zone"]
        numeric_value = _number(value)
        lower = _number(zone[0])
        upper = _number(zone[1])
        outside = None
        if None not in (numeric_value, lower, upper):
            outside = max(lower - numeric_value, numeric_value - upper, 0.0)
        detail = (
            f"value {_fmt(value)}; allowed [{_fmt(zone[0])},{_fmt(zone[1])}]; "
            f"distance outside {_fmt(outside)}"
        )
    else:
        limit = condition.get("limit")
        detail = (
            f"value {_fmt(value)}; strict upper limit {_fmt(limit)}; "
            f"signed excess {_fmt(condition.get('margin'))}"
        )
    return f"- {name}: {state} — {detail}."


def _section_constraints(metrics: Dict[str, Any]) -> List[str]:
    parts = ["\n### 3. Constraint profile"]
    unlock = metrics.get("unlock_condition_status")
    if isinstance(unlock, dict):
        parts.append(
            f"- Unlock state: {'PASS' if metrics.get('unlocked') is True else 'FAIL'}; "
            f"current streak {unlock.get('consecutive_count', 'N/A')}/"
            f"{unlock.get('required_consecutive', 'N/A')} steps; "
            f"maximum observed streak {(metrics.get('diagnostic_timeline') or {}).get('max_unlock_condition_streak', 'N/A')}."
        )
        for condition in unlock.get("conditions", []):
            if isinstance(condition, dict):
                parts.append(_format_unlock_condition(condition))
    else:
        parts.append("- Unlock-condition measurements unavailable.")

    current_dwell = _number(metrics.get("consecutive_steps_in_exit"))
    max_dwell = _number(metrics.get("max_consecutive_steps_in_exit"))
    required_dwell = _number(metrics.get("consecutive_exit_steps_required"))
    if current_dwell is not None and required_dwell is not None:
        achieved_dwell = max_dwell is not None and max_dwell >= required_dwell
        state = "PASS" if achieved_dwell else "FAIL"
        historical_margin = (
            f"{max_dwell - required_dwell:+.0f}"
            if max_dwell is not None else "N/A"
        )
        parts.append(
            f"- Exit dwell: {state} — current {int(current_dwell)}/{int(required_dwell)} steps; "
            f"maximum observed {int(max_dwell) if max_dwell is not None else 'N/A'}; "
            f"historical margin {historical_margin} steps."
        )

    timeline = metrics.get("diagnostic_timeline") or {}
    peak_impulse = _number(timeline.get("peak_collision_impulse_ns"))
    impulse_limit = _number(metrics.get("structural_impulse_limit_ns"))
    if peak_impulse is not None and impulse_limit is not None:
        state = "PASS" if peak_impulse <= impulse_limit else "FAIL"
        parts.append(
            f"- Structural impulse: {state} — peak {peak_impulse:.2f}/{impulse_limit:.2f} N·s "
            f"at step {timeline.get('peak_collision_impulse_step', 'N/A')}; "
            f"headroom {impulse_limit - peak_impulse:+.2f} N·s."
        )
    parts.append(f"- Agent destroyed: {bool(metrics.get('agent_destroyed', False))}.")

    step = _number(metrics.get("step_count"))
    maximum = _number(metrics.get("max_steps"))
    if step is not None and maximum is not None:
        parts.append(
            f"- Step limit: {int(step)}/{int(maximum)} used; "
            f"{max(0, int(maximum - step))} steps remaining at termination."
        )
    return parts


def _section_actuation(metrics: Dict[str, Any]) -> List[str]:
    parts = ["\n### 4. Actuation and stability"]
    ledger = metrics.get("force_ledger")
    if isinstance(ledger, dict):
        requested = ledger.get("requested_force") or {}
        evaluated = ledger.get("unlock_evaluated_force") or {}
        parts.append(
            f"- Latest requested force: ({_fmt(requested.get('fx'))},"
            f"{_fmt(requested.get('fy'))}) N; force used by the unlock check: "
            f"({_fmt(evaluated.get('fx'))},{_fmt(evaluated.get('fy'))}) N."
        )
        req_fx = _number(requested.get("fx"))
        req_fy = _number(requested.get("fy"))
        eval_fx = _number(evaluated.get("fx"))
        eval_fy = _number(evaluated.get("fy"))
        if None not in (req_fx, req_fy, eval_fx, eval_fy):
            mismatch = math.hypot(req_fx - eval_fx, req_fy - eval_fy)
            parts.append(f"- Requested/evaluated command mismatch: {mismatch:.2f} N.")
    else:
        parts.append("- Command measurements unavailable.")

    timeline = metrics.get("diagnostic_timeline")
    if isinstance(timeline, dict):
        parts.append(
            f"- Peak requested force magnitude: {_fmt(timeline.get('peak_requested_force_n'))} N "
            f"at step {timeline.get('peak_requested_force_step', 'N/A')}."
        )
        parts.append(
            f"- Peak speed: {_fmt(timeline.get('max_speed_mps'))} m/s at step "
            f"{timeline.get('max_speed_step', 'N/A')}."
        )
    return parts


def _section_numerical_health(metrics: Dict[str, Any]) -> List[str]:
    bad = _non_finite_paths(metrics)
    if not bad:
        return ["\n### 5. Numerical health", "- All reported numeric metrics are finite."]
    shown = ", ".join(bad[:8])
    suffix = f" (+{len(bad) - 8} more)" if len(bad) > 8 else ""
    return [
        "\n### 5. Numerical health",
        f"- {len(bad)} non-finite numeric value(s): {shown}{suffix}.",
    ]


def format_task_metrics(metrics: Dict[str, Any]) -> List[str]:
    if not metrics:
        return ["**No metrics available**"]
    parts = _section_outcome(metrics)
    diagnostic_keys = {
        "agent_x", "step_count", "unlock_condition_status", "force_ledger",
        "constraint_violations", "diagnostic_timeline",
    }
    if not diagnostic_keys.intersection(metrics):
        parts.extend([
            "\n### Diagnostics",
            "- Task metrics are unavailable; refer to the execution error reported above.",
        ])
        parts.extend(_section_numerical_health(metrics))
        return parts
    parts.extend(_section_chronology(metrics))
    parts.extend(_section_spatial(metrics))
    parts.extend(_section_constraints(metrics))
    parts.extend(_section_actuation(metrics))
    parts.extend(_section_numerical_health(metrics))
    return parts


def get_improvement_suggestions(
    metrics: Dict[str, Any],
    score: float,
    success: bool,
    failed: bool,
    failure_reason: str = None,
    error: str = None,

) -> List[str]:
    return []
