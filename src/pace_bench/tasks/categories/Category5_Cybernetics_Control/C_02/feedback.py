import math
from typing import Any, Dict, List, Optional


def _finite(value: Any) -> bool:
    try:
        return value is not None and math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _number(value: Any, digits: int = 3) -> str:
    if not _finite(value):
        return "unavailable" if value is None else str(value)
    return f"{float(value):.{digits}f}"


def _constraint_value(value: Any) -> str:
    if isinstance(value, (list, tuple)) and len(value) == 2:
        return f"[{_number(value[0], 2)}, {_number(value[1], 2)}]"
    return _number(value) if isinstance(value, (int, float)) else str(value)


def _format_constraints(metrics: Dict[str, Any]) -> List[str]:
    profile = metrics.get("constraint_profile")
    if not isinstance(profile, list) or not profile:
        return ["Constraint profile unavailable."]

    valid = [item for item in profile if isinstance(item, dict)]
    failures = [item for item in valid if str(item.get("status")) == "FAIL"]
    pending = [item for item in valid if str(item.get("status")) == "PENDING"]
    passes = [item for item in valid if str(item.get("status")) == "PASS"]

    def render(item: Dict[str, Any]) -> str:
        status = str(item.get("status", "UNKNOWN"))
        name = str(item.get("name", "unnamed constraint"))
        value = _constraint_value(item.get("value"))
        limit = _constraint_value(item.get("limit"))
        margin = item.get("margin")
        margin_text = f", margin {_number(margin)}" if _finite(margin) else ""
        return (
            f"- {status} — {name}: value {value}; "
            f"limit {limit}{margin_text}."
        )

    lines = [render(item) for item in failures]
    ranked_passes = sorted(
        (
            item
            for item in passes
            if _finite(item.get("margin")) and _finite(item.get("limit"))
        ),
        key=lambda item: abs(float(item["margin"]))
        / max(abs(float(item["limit"])), 1e-12),
    )
    keep_count = 2 if not failures else 1
    retained_passes = ranked_passes[:keep_count]
    lines.extend(render(item) for item in retained_passes)

    if pending:
        pending_text = "; ".join(
            f"{item.get('name', 'unnamed')} (limit {_constraint_value(item.get('limit'))})"
            for item in pending
        )
        lines.append(f"- PENDING — {pending_text}.")
    retained_ids = {id(item) for item in retained_passes}
    summarized_passes = [
        str(item.get("name", "unnamed"))
        for item in passes
        if id(item) not in retained_ids
    ]
    if summarized_passes:
        lines.append("- PASS — " + "; ".join(summarized_passes) + ".")
    if not lines:
        lines.append("No valid constraint entries were available.")
    return lines


def _format_events(metrics: Dict[str, Any]) -> List[str]:
    events: List[tuple[int, str]] = []
    transit = metrics.get("corridor_transit")
    if isinstance(transit, dict):
        entry_step = transit.get("entry_step")
        if isinstance(entry_step, int):
            events.append(
                (
                    entry_step,
                    "corridor entry at "
                    f"(x={_number(transit.get('entry_x'), 2)}, "
                    f"y={_number(transit.get('entry_y'), 2)}) m",
                )
            )
        violation_step = transit.get("violation_step")
        if isinstance(violation_step, int):
            events.append(
                (
                    violation_step,
                    f"corridor {transit.get('violation_kind', 'unknown')} violation at "
                    f"(x={_number(transit.get('violation_x'), 2)}, "
                    f"y={_number(transit.get('violation_y'), 2)}) m",
                )
            )
        exit_step = transit.get("exit_step")
        if isinstance(exit_step, int):
            events.append(
                (
                    exit_step,
                    "corridor exit at "
                    f"(x={_number(transit.get('exit_x'), 2)}, "
                    f"y={_number(transit.get('exit_y'), 2)}) m",
                )
            )
    landing_step = metrics.get("landing_step")
    if isinstance(landing_step, int):
        events.append(
            (
                landing_step,
                f"touchdown with |vy|={_number(abs(float(metrics['landing_vy'])), 3) if _finite(metrics.get('landing_vy')) else 'unavailable'} m/s, "
                f"angle={_number(math.degrees(float(metrics['landing_angle'])), 2) if _finite(metrics.get('landing_angle')) else 'unavailable'} deg",
            )
        )
    events.sort(key=lambda item: item[0])
    if not events:
        return ["No corridor-entry, corridor-exit, violation, or touchdown event was recorded."]
    return [f"- Step {step}: {description}." for step, description in events]


def _format_state(metrics: Dict[str, Any]) -> List[str]:
    lines = [
        "- Terminal lander state: "
        f"position=({_number(metrics.get('lander_x'), 2)}, {_number(metrics.get('lander_y'), 2)}) m; "
        f"velocity=({_number(metrics.get('lander_vx'), 2)}, {_number(metrics.get('lander_vy'), 2)}) m/s; "
        f"angle={_number(math.degrees(float(metrics['lander_angle'])), 2) if _finite(metrics.get('lander_angle')) else 'unavailable'} deg; "
        f"angular velocity={_number(metrics.get('lander_angular_velocity'), 3)} rad/s."
    ]
    transit = metrics.get("corridor_transit")
    if isinstance(transit, dict) and transit.get("entered"):
        lines.append(
            "- Corridor center trajectory range: "
            f"y=[{_number(transit.get('min_y_in_corridor'), 2)}, "
            f"{_number(transit.get('max_y_in_corridor'), 2)}] m; permitted hull-corner bounds "
            f"y=[{_number(metrics.get('barrier_y_top'), 2)}, "
            f"{_number(metrics.get('barrier_y_bottom'), 2)}] m."
        )
    if metrics.get("landed"):
        lines.append(
            "- Touchdown horizontal geometry: "
            f"hull ground-contact edge x=[{_number(metrics.get('landing_x_lo'), 2)}, "
            f"{_number(metrics.get('landing_x_hi'), 2)}] m; platform zone x="
            f"[{_number(metrics.get('zone_x_min'), 2)}, "
            f"{_number(metrics.get('zone_x_max'), 2)}] m."
        )
    remaining = metrics.get("remaining_fuel")
    total = metrics.get("total_fuel_impulse")
    used = (
        float(total) - float(remaining)
        if _finite(total) and _finite(remaining)
        else None
    )
    lines.append(
        f"- Impulse budget: used {_number(used)} N·s; "
        f"remaining {_number(remaining)} N·s of {_number(total)} N·s; "
        f"minimum at touchdown {_number(metrics.get('min_fuel_remaining_at_landing'))} N·s."
    )
    return lines


def _format_control_profile(metrics: Dict[str, Any]) -> List[str]:
    actuation = metrics.get("actuation_diagnostics")
    motion = metrics.get("motion_diagnostics")
    lines: List[str] = []
    if isinstance(actuation, dict) and actuation:
        thrust_steps = actuation.get("thrust_saturation_steps")
        torque_steps = actuation.get("torque_saturation_steps")
        lines.append(
            "- Applied thrust: peak "
            f"{_number(actuation.get('peak_abs_applied_thrust'))} N / "
            f"{_number(metrics.get('max_thrust'))} N limit; "
            f"saturated for {_number(thrust_steps, 0)} steps"
            + (
                f" from step {actuation.get('first_thrust_saturation_step')}"
                if actuation.get("first_thrust_saturation_step") is not None
                else ""
            )
            + "."
        )
        lines.append(
            "- Applied steering torque: peak "
            f"{_number(actuation.get('peak_abs_applied_torque'))} N·m / "
            f"{_number(metrics.get('max_torque'))} N·m limit; "
            f"saturated for {_number(torque_steps, 0)} steps"
            + (
                f" from step {actuation.get('first_torque_saturation_step')}"
                if actuation.get("first_torque_saturation_step") is not None
                else ""
            )
            + "."
        )
    else:
        lines.append("- Applied-actuation history unavailable.")

    if isinstance(motion, dict) and motion:
        vx = motion.get("abs_vx") if isinstance(motion.get("abs_vx"), dict) else {}
        vy = motion.get("abs_vy") if isinstance(motion.get("abs_vy"), dict) else {}
        omega = (
            motion.get("abs_angular_velocity")
            if isinstance(motion.get("abs_angular_velocity"), dict)
            else {}
        )
        lines.append(
            "- Motion peaks: "
            f"|vx|={_number(vx.get('value'))} m/s at step {vx.get('step', 'unavailable')}; "
            f"|vy|={_number(vy.get('value'))} m/s at step {vy.get('step', 'unavailable')}; "
            f"|ω|={_number(omega.get('value'))} rad/s at step "
            f"{omega.get('step', 'unavailable')}."
        )
    else:
        lines.append("- Motion-peak history unavailable.")
    return lines


def _format_numerical_health(metrics: Dict[str, Any]) -> str:
    checked = (
        "lander_x",
        "lander_y",
        "lander_vx",
        "lander_vy",
        "lander_angle",
        "lander_angular_velocity",
        "height_above_ground",
        "remaining_fuel",
    )
    invalid = [key for key in checked if metrics.get(key) is not None and not _finite(metrics.get(key))]
    if invalid:
        return "Non-finite values: " + ", ".join(invalid) + "."
    return "All available terminal state values are finite."


def format_task_metrics(metrics: Dict[str, Any]) -> List[str]:
    if not isinstance(metrics, dict):
        return ["## C-02 Lander", "Metrics unavailable: expected a dictionary."]
    if metrics.get("error"):
        return ["## C-02 Lander", f"Evaluation error: {metrics['error']}"]

    success = bool(metrics.get("success"))
    failed = bool(metrics.get("failed"))
    outcome = "SUCCESS" if success else "FAILED" if failed else "IN PROGRESS"
    reason = metrics.get("failure_reason")
    step = metrics.get("step_count", "unavailable")
    horizon = metrics.get("episode_horizon")
    progress = f"step {step}/{horizon}" if horizon is not None else f"step {step}"
    parts = [f"## C-02 Lander — {outcome}", f"- {progress}."]
    if reason:
        parts.append(f"- Decisive result: {reason}.")

    parts.extend(["", "### Event chronology"])
    parts.extend(_format_events(metrics))
    parts.extend(["", "### State and spatial margins"])
    parts.extend(_format_state(metrics))
    parts.extend(["", "### Control and motion profile"])
    parts.extend(_format_control_profile(metrics))
    parts.extend(["", "### Constraint profile"])
    parts.extend(_format_constraints(metrics))
    parts.extend(["", "### Numerical health", _format_numerical_health(metrics)])
    return parts


def get_improvement_suggestions(
    metrics: Dict[str, Any],
    score: float,
    success: bool,
    failed: bool,
    failure_reason: Optional[str] = None,
    error: Optional[str] = None,
) -> List[str]:
    if error:
        return [f"Execution error: {error}"]
    return []
