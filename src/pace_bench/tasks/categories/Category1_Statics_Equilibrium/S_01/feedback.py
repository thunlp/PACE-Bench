from typing import Any, Dict, List, Optional

import math


def _f(value: Any) -> Optional[float]:
    if value is None:
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _step(value: Any) -> Optional[int]:
    number = _f(value)
    return int(number) if number is not None else None


def _fmt_pos(position: Any) -> str:
    if not isinstance(position, (list, tuple)) or len(position) < 2:
        return "(unavailable)"
    x = _f(position[0])
    y = _f(position[1])
    if x is None or y is None:
        return "(unavailable)"
    return f"({x:.2f}, {y:.2f}) m"


def _anchor_position(record: Dict[str, Any], prefix: str = "") -> Any:
    anchor_a = record.get(f"{prefix}anchor_a_pos")
    anchor_b = record.get(f"{prefix}anchor_b_pos")
    if not isinstance(anchor_a, (list, tuple)) or len(anchor_a) < 2:
        return None
    if not isinstance(anchor_b, (list, tuple)) or len(anchor_b) < 2:
        return anchor_a
    ax = _f(anchor_a[0])
    ay = _f(anchor_a[1])
    bx = _f(anchor_b[0])
    by = _f(anchor_b[1])
    if None in (ax, ay, bx, by):
        return anchor_a
    return ((ax + bx) / 2.0, (ay + by) / 2.0)


def _moment_signature(metrics: Dict[str, Any]) -> tuple[Any, ...]:
    return (
        _step(metrics.get("step_count")),
        bool(metrics.get("success")),
        bool(metrics.get("failed")),
        _f(metrics.get("vehicle_x")),
        metrics.get("failure_reason"),
    )


def _format_outcome(metrics: Dict[str, Any]) -> List[str]:
    step_count = _step(metrics.get("step_count"))
    samples = _step(metrics.get("evaluation_sample_count"))
    detail = []
    if samples is not None:
        detail.append(f"{samples} evaluator samples")
    if step_count is not None:
        detail.append(f"terminal step {step_count}")
    suffix = f" — {', '.join(detail)}" if detail else ""
    return [f"## Passage and terminal state{suffix}"]


def _format_progress(metrics: Dict[str, Any]) -> List[str]:
    parts: List[str] = []
    vehicle_x = _f(metrics.get("vehicle_x"))
    vehicle_y = _f(metrics.get("vehicle_y"))
    target_x = _f(metrics.get("target_x"))
    start_x = _f(metrics.get("vehicle_start_x"))
    best_x = _f(metrics.get("best_vehicle_x"))
    best_step = _step(metrics.get("best_vehicle_x_step"))
    best_y = _f(metrics.get("best_vehicle_y_at_progress"))
    best_is_terminal = (
        best_x is not None
        and vehicle_x is not None
        and abs(best_x - vehicle_x) < 1e-9
        and best_step == _step(metrics.get("step_count"))
    )
    if vehicle_x is not None and vehicle_y is not None and not best_is_terminal:
        terminal = f"- Terminal chassis: ({vehicle_x:.2f}, {vehicle_y:.2f}) m"
        if target_x is not None:
            terminal += f"; target margin {vehicle_x - target_x:+.2f} m"
        parts.append(terminal)
    if best_x is not None:
        label = "Best/terminal chassis" if best_is_terminal else "Best sampled progress"
        best_line = f"- {label}: x={best_x:.2f} m"
        if best_y is not None:
            best_line += f", y={best_y:.2f} m"
        if best_step is not None:
            best_line += f" at step {best_step}"
        if target_x is not None:
            best_line += (
                f"; target x≥{target_x:.2f}, deficit "
                f"{max(0.0, target_x - best_x):.2f} m"
            )
        if start_x is not None and target_x is not None and target_x > start_x:
            progress = 100.0 * (best_x - start_x) / (target_x - start_x)
            best_line += f" ({progress:.1f}% displacement)"
        parts.append(best_line)

    velocity_x = _f(metrics.get("velocity_x"))
    velocity_y = _f(metrics.get("velocity_y"))
    angle = _f(metrics.get("normalized_angle"))
    angular_velocity = _f(metrics.get("angular_velocity"))
    state = []
    if velocity_x is not None and velocity_y is not None:
        state.append(
            f"velocity=({velocity_x:.2f}, {velocity_y:.2f}) m/s"
        )
    if angle is not None:
        state.append(f"tilt={math.degrees(angle):.1f}°")
    if angular_velocity is not None:
        state.append(f"angular velocity={angular_velocity:.3f} rad/s")
    if state:
        parts.append("- Terminal motion: " + "; ".join(state))
    if not parts:
        parts.append("- Spatial state unavailable")
    return parts


def _build_zone_result(metrics: Dict[str, Any]) -> Optional[tuple[bool, str]]:
    bounds = [
        _f(metrics.get("build_zone_x_min")),
        _f(metrics.get("build_zone_x_max")),
        _f(metrics.get("build_zone_y_min")),
        _f(metrics.get("build_zone_y_max")),
    ]
    positions = metrics.get("body_creation_positions")
    if any(value is None for value in bounds) or not isinstance(positions, list):
        return None
    x_min, x_max, y_min, y_max = bounds
    invalid = []
    worst_margin = math.inf
    worst_index = None
    worst_position = None
    for index, position in enumerate(positions):
        if not isinstance(position, (list, tuple)) or len(position) < 2:
            invalid.append(index)
            continue
        x = _f(position[0])
        y = _f(position[1])
        if x is None or y is None:
            invalid.append(index)
            continue
        margin = min(x - x_min, x_max - x, y - y_min, y_max - y)
        if margin < worst_margin:
            worst_margin = margin
            worst_index = index
            worst_position = (x, y)
        if margin < 0.0:
            invalid.append(index)
    status = not invalid
    text = (
        f"build {len(invalid)}/{len(positions)} centers out, "
        f"x[{x_min:.1f},{x_max:.1f}] y[{y_min:.1f},{y_max:.1f}]"
    )
    if worst_index is not None and worst_position is not None:
        text += (
            f", min Δ{worst_margin:+.2f} m at B{worst_index} "
            f"{_fmt_pos(worst_position)}"
        )
    return status, text


def _format_constraints(metrics: Dict[str, Any]) -> List[str]:
    rows: List[tuple[bool, str]] = []
    mass = _f(metrics.get("structure_mass"))
    mass_limit = _f(metrics.get("max_structure_mass"))
    if mass is not None and mass_limit is not None:
        margin = mass_limit - mass
        rows.append(
            (margin >= 0.0, f"mass {mass:.2f}≤{mass_limit:.2f} kg, Δ{margin:+.2f}")
        )

    build_zone = _build_zone_result(metrics)
    if build_zone is not None:
        rows.append(build_zone)

    joint_count = _step(metrics.get("joint_count"))
    initial_joint_count = _step(metrics.get("initial_joint_count"))
    failure_events = metrics.get("joint_failure_events")
    event_count = len(failure_events) if isinstance(failure_events, list) else None
    if joint_count is not None and initial_joint_count is not None:
        broken = bool(metrics.get("structure_broken"))
        text = (
            f"integrity {joint_count}/{initial_joint_count} active, "
            f"{max(0, initial_joint_count - joint_count)} lost"
        )
        if event_count is not None:
            text += f", {event_count} events"
        rows.append((not broken, text))

    fail_y = _f(metrics.get("fail_zone_y"))
    min_vehicle_y = _f(metrics.get("min_vehicle_y"))
    if min_vehicle_y is not None and fail_y is not None:
        margin = min_vehicle_y - fail_y
        first_step = _step(metrics.get("first_chassis_fail_zone_sample_step"))
        minimum_step = _step(metrics.get("min_vehicle_y_step"))
        text = f"chassis y_min {min_vehicle_y:.2f}>{fail_y:.2f} m"
        minimum_x = _f(metrics.get("min_vehicle_x_at_min_y"))
        if minimum_x is not None:
            text += f" at ({minimum_x:.2f},{min_vehicle_y:.2f})"
        if minimum_step is not None:
            text += f" s{minimum_step}"
        text += f", Δ{margin:+.2f}"
        if first_step is not None and first_step != minimum_step:
            text += f", first fail s{first_step}"
        rows.append((margin > 0.0, text))

    min_structure_y = _f(metrics.get("min_structure_y"))
    if min_structure_y is not None and fail_y is not None:
        margin = min_structure_y - fail_y
        first_step = _step(metrics.get("first_structure_fail_zone_sample_step"))
        minimum_step = _step(metrics.get("min_structure_y_step"))
        minimum_position = (
            metrics.get("min_structure_x_at_min_y"),
            min_structure_y,
        )
        text = f"structure y_min {min_structure_y:.2f}>{fail_y:.2f} m"
        body_index = _step(metrics.get("min_structure_body_index"))
        if body_index is not None:
            text += f" B{body_index}"
        if _f(minimum_position[0]) is not None:
            text += f" at {_fmt_pos(minimum_position)}"
        if minimum_step is not None:
            text += f" s{minimum_step}"
        text += f", Δ{margin:+.2f}"
        if first_step is not None and first_step != minimum_step:
            text += f", first fail s{first_step}"
        rows.append((margin > 0.0, text))

    acceleration = _f(metrics.get("max_vertical_accel_seen"))
    if acceleration is None:
        acceleration = _f(metrics.get("max_vertical_accel"))
    acceleration_limit = _f(metrics.get("max_vertical_acceleration_limit"))
    if acceleration is not None and acceleration_limit is not None:
        margin = acceleration_limit - acceleration
        peak_step = _step(metrics.get("max_vertical_accel_step"))
        text = (
            f"|a_y| peak {acceleration:.2f}≤{acceleration_limit:.2f} "
            f"m/s², Δ{margin:+.2f}"
        )
        if peak_step is not None:
            text += f" s{peak_step}"
        rows.append((margin >= 0.0, text))

    maximum_streak = _step(metrics.get("max_high_angular_velocity_count"))
    streak_limit = _step(metrics.get("unstable_threshold_limit"))
    angular_limit = _f(metrics.get("max_angular_velocity_limit"))
    stability_start = _step(metrics.get("stability_check_start_step"))
    if maximum_streak is not None and streak_limit is not None and angular_limit is not None:
        text = (
            f"ω streak {maximum_streak}<{streak_limit} samples "
            f"(|ω|>{angular_limit:.2f} rad/s"
        )
        if stability_start is not None:
            text += f" after s{stability_start}"
        text += ")"
        first_high_step = _step(metrics.get("first_high_angular_velocity_step"))
        if first_high_step is not None:
            text += f", first post-start over-limit s{first_high_step}"
        rows.append((maximum_streak < streak_limit, text))

    peak_angle = _f(metrics.get("max_abs_angle"))
    flip_limit = _f(metrics.get("flip_angle_limit_rad"))
    if peak_angle is not None and flip_limit is not None:
        margin = flip_limit - peak_angle
        peak_step = _step(metrics.get("max_abs_angle_step"))
        text = (
            f"|tilt| peak {math.degrees(peak_angle):.1f}°≤"
            f"{math.degrees(flip_limit):.1f}°, Δ{math.degrees(margin):+.1f}°"
        )
        if peak_step is not None:
            text += f" s{peak_step}"
        rows.append((margin >= 0.0, text))

    airborne_rotation = _f(metrics.get("max_airborne_rotation_seen"))
    if airborne_rotation is None:
        airborne_rotation = _f(metrics.get("airborne_rotation_accumulated"))
    airborne_limit = _f(metrics.get("max_airborne_rotation_limit"))
    if airborne_rotation is not None and airborne_limit is not None:
        margin = airborne_limit - airborne_rotation
        peak_step = _step(metrics.get("max_airborne_rotation_step"))
        text = (
            f"air rotation {math.degrees(airborne_rotation):.1f}°≤"
            f"{math.degrees(airborne_limit):.1f}°, Δ{math.degrees(margin):+.1f}°"
        )
        if peak_step is not None:
            text += f" s{peak_step}"
        rows.append((margin >= 0.0, text))

    parts = ["## Constraint profile"]
    failed_rows = [text for passed, text in rows if not passed]
    passed_rows = [text for passed, text in rows if passed]
    parts.extend(f"- [FAIL] {text}" for text in failed_rows)
    if passed_rows:
        parts.append("- [PASS] " + "; ".join(passed_rows))
    if not rows:
        parts.append("- Constraint measurements unavailable")
    return parts


def _limit_for_record(
    record: Dict[str, Any], metrics: Dict[str, Any], channel: str
) -> Optional[float]:
    direct = _f(record.get(f"{channel}_limit"))
    if direct is not None:
        return direct
    role = "anchor" if record.get("is_anchor") else "joint"
    return _f(metrics.get(f"{role}_max_{channel}_limit"))


def _stress_entries(metrics: Dict[str, Any]) -> List[Dict[str, Any]]:
    entries = []
    sources = [
        (metrics.get("joint_failure_events"), True),
        (metrics.get("joint_stress_summary"), False),
    ]
    for records, failed in sources:
        if not isinstance(records, list):
            continue
        for sequence, record in enumerate(records):
            if not isinstance(record, dict):
                continue
            force = _f(record.get("peak_force"))
            torque = _f(record.get("peak_torque"))
            force_limit = _limit_for_record(record, metrics, "force")
            torque_limit = _limit_for_record(record, metrics, "torque")
            force_util = force / force_limit if force is not None and force_limit and force_limit > 0 else None
            torque_util = torque / torque_limit if torque is not None and torque_limit and torque_limit > 0 else None
            utilizations = [
                (name, value)
                for name, value in (("force", force_util), ("torque", torque_util))
                if value is not None
            ]
            if not utilizations:
                continue
            governing_channel, governing_utilization = max(
                utilizations, key=lambda item: item[1]
            )
            prefix = f"peak_{governing_channel}_"
            entries.append(
                {
                    "joint_id": record.get("joint_id", record.get("joint_idx", sequence)),
                    "is_anchor": bool(record.get("is_anchor")),
                    "failed": failed,
                    "force": force,
                    "force_limit": force_limit,
                    "force_util": force_util,
                    "torque": torque,
                    "torque_limit": torque_limit,
                    "torque_util": torque_util,
                    "governing_channel": governing_channel,
                    "governing_utilization": governing_utilization,
                    "peak_step": _step(record.get(f"{prefix}step", record.get("step"))),
                    "position": _anchor_position(record, prefix),
                }
            )
    return sorted(entries, key=lambda entry: entry["governing_utilization"], reverse=True)


def _format_loads(metrics: Dict[str, Any]) -> List[str]:
    entries = _stress_entries(metrics)
    parts = ["## Peak joint loads"]
    if not entries:
        parts.append("- No finite joint stress measurements available")
        return parts
    failed_count = sum(1 for entry in entries if entry["failed"])
    parts.append(
        f"- Top 3 of {len(entries)} joints; {failed_count} failed:"
    )
    for rank, entry in enumerate(entries[:3], start=1):
        role = "anchor" if entry["is_anchor"] else "structural"
        lifecycle = "failed" if entry["failed"] else "active"
        channel = entry["governing_channel"]
        actual = entry[channel]
        limit = entry[f"{channel}_limit"]
        margin = limit - actual
        force_text = "unavailable"
        if entry["force"] is not None and entry["force_limit"] is not None:
            force_text = (
                f"{entry['force']:.2f}/{entry['force_limit']:.2f}="
                f"{100.0 * entry['force_util']:.1f}%"
            )
        torque_text = "unavailable"
        if entry["torque"] is not None and entry["torque_limit"] is not None:
            torque_text = (
                f"{entry['torque']:.3f}/{entry['torque_limit']:.3f}="
                f"{100.0 * entry['torque_util']:.1f}%"
            )
        where = _fmt_pos(entry["position"])
        when = (
            f"s{entry['peak_step']}"
            if entry["peak_step"] is not None
            else "step unavailable"
        )
        parts.append(
            f"- #{rank} J{entry['joint_id']} {role}/{lifecycle} {when} {where}: "
            f"F {force_text}; T {torque_text}; {channel} Δ{margin:+.3f}"
        )
    return parts


def _format_chronology(metrics: Dict[str, Any]) -> List[str]:
    timeline: List[tuple[int, int, str]] = []
    failure_events = metrics.get("joint_failure_events")
    if isinstance(failure_events, list) and failure_events:
        valid_events = [
            event
            for event in failure_events
            if isinstance(event, dict) and _step(event.get("step")) is not None
        ]
        if valid_events:
            valid_events.sort(key=lambda event: _step(event.get("step")))
            first = valid_events[0]
            first_step = _step(first.get("step"))
            role = "anchor" if first.get("is_anchor") else "structural"
            timeline.append(
                (
                    first_step,
                    0,
                    f"first joint failure J{first.get('joint_id', '?')} "
                    f"{role} {_fmt_pos(_anchor_position(first))}",
                )
            )
            last_step = _step(valid_events[-1].get("step"))
            anchor_count = sum(1 for event in valid_events if event.get("is_anchor"))
            structural_count = len(valid_events) - anchor_count
            x_values = [
                _f(position[0])
                for event in valid_events
                if (position := _anchor_position(event)) is not None
            ]
            x_values = [value for value in x_values if value is not None]
            span = f"steps {first_step}–{last_step}"
            if x_values:
                span += f", anchor x=[{min(x_values):.2f}, {max(x_values):.2f}] m"
            cascade_text = (
                f"{len(valid_events)} total ({anchor_count} anchor, "
                f"{structural_count} structural), {span}"
            )
        else:
            cascade_text = f"{len(failure_events)} malformed or untimed failure records"
    else:
        cascade_text = "no joint failure events"

    for key, priority, label in (
        ("first_chassis_fail_zone_sample_step", 1, "chassis first at/below fail plane"),
        ("first_structure_fail_zone_sample_step", 2, "structure first at/below fail plane"),
    ):
        event_step = _step(metrics.get(key))
        if event_step is not None:
            timeline.append((event_step, priority, label))

    terminal_step = _step(metrics.get("step_count"))
    if terminal_step is not None:
        terminal_text = "terminal evaluation"
        if metrics.get("success"):
            terminal_text += " (passage success)"
        timeline.append((terminal_step, 9, terminal_text))

    parts = ["## Chronology"]
    if timeline:
        parts.append(
            "- "
            + " → ".join(
                f"s{event_step} {label}"
                for event_step, _, label in sorted(timeline)
            )
        )
    parts.append(f"- Joint failures: {cascade_text}")
    return parts


def _format_numerical_health(metrics: Dict[str, Any]) -> List[str]:
    required = (
        "vehicle_x",
        "vehicle_y",
        "velocity_x",
        "velocity_y",
        "angular_velocity",
        "normalized_angle",
        "structure_mass",
        "max_vertical_accel",
    )
    missing = []
    invalid = []
    for key in required:
        if key not in metrics or metrics.get(key) is None:
            missing.append(key)
            continue
        try:
            value = float(metrics.get(key))
        except (TypeError, ValueError):
            invalid.append(f"{key}={metrics.get(key)!r}")
            continue
        if not math.isfinite(value):
            invalid.append(f"{key}={value}")
    if invalid:
        return ["## Numerical health: FAIL", "- Non-finite or non-numeric: " + ", ".join(invalid)]
    if missing:
        return ["## Numerical health: PARTIAL", "- Missing core fields: " + ", ".join(missing)]
    return ["## Numerical health: PASS — core evaluator values are finite"]


def format_task_metrics(
    metrics: Dict[str, Any],
    prev_metrics: Optional[Dict[str, Any]] = None,

) -> List[str]:
    if not metrics:
        return ["## Outcome: metrics unavailable"]
    if (
        prev_metrics is not None
        and _moment_signature(metrics) == _moment_signature(prev_metrics)
    ):
        step_count = _step(metrics.get("step_count"))
        return [
            "## Outcome: state unchanged"
            + (f" at evaluator sample step {step_count}" if step_count is not None else "")
        ]
    parts: List[str] = []
    parts.extend(_format_outcome(metrics))
    parts.extend(_format_progress(metrics))
    parts.extend(_format_chronology(metrics))
    parts.extend(_format_constraints(metrics))
    parts.extend(_format_loads(metrics))
    parts.extend(_format_numerical_health(metrics))
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
        return ["- Code execution failed. Review the error details above."]
    return []
