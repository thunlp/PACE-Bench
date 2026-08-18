"""Compact, deterministic feedback for F-05."""

from typing import Any, Dict, List

import math


def _number(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _integer(value: Any) -> int | None:
    try:
        return int(value)
    except (TypeError, ValueError, OverflowError):
        return None


def _ratio_line(label: str, value: Any, limit: Any, unit: str) -> str | None:
    measured = _number(value)
    maximum = _number(limit)
    if measured is None:
        return None
    if maximum is None:
        return f"- {label}: {measured:.2f} {unit} (no finite break limit)"
    if maximum <= 0.0:
        return f"- {label}: {measured:.2f} {unit}; configured limit is {maximum:.2f}"
    percent = measured / maximum * 100.0
    margin = maximum - measured
    status = "within" if margin >= 0.0 else "over"
    return (
        f"- {label}: {measured:.2f}/{maximum:.2f} {unit} ({percent:.1f}%); "
        f"{abs(margin):.2f} {unit} {status} limit"
    )


def _numerical_warnings(metrics: Dict[str, Any]) -> List[str]:
    public_numeric_keys = (
        "boat_angle_rad",
        "boat_angle_deg",
        "peak_abs_boat_angle_rad",
        "peak_abs_boat_angle_deg",
        "boat_x",
        "boat_y",
        "structure_mass",
        "cargo_lowest_y",
        "cargo_loss_margin",
        "joint_peak_force_N",
        "joint_peak_torque_Nm",
        "peak_angular_velocity_rad_s",
    )
    warnings = []
    for key in public_numeric_keys:
        value = metrics.get(key)
        if value is None:
            continue
        try:
            if not math.isfinite(float(value)):
                warnings.append(f"{key} is non-finite ({value})")
        except (TypeError, ValueError):
            warnings.append(f"{key} is non-numeric ({value})")
    return warnings


def format_task_metrics(metrics: Dict[str, Any]) -> List[str]:
    """Format only measured outcomes and published constraints.

    Hidden forcing coefficients and other invisible simulator parameters are
    intentionally ignored even if a legacy caller includes them in ``metrics``.
    """

    if not isinstance(metrics, dict) or not metrics:
        return ["(No evaluation metrics available.)"]

    violations = metrics.get("constraint_violations")
    if isinstance(violations, list) and violations:
        lines = ["### Build Rejected"]
        lines.extend(f"- {item}" for item in violations)
        reason = metrics.get("failure_reason")
        if reason:
            lines.append(f"- Reason: {reason}")
        return lines

    success = metrics.get("success") is True
    failed = metrics.get("failed") is True
    outcome = "SUCCESS" if success else "FAILED" if failed else "INCOMPLETE"
    lines = ["### Evaluation Summary", f"- Outcome: {outcome}"]

    step = _integer(metrics.get("step_count"))
    max_steps = _integer(metrics.get("max_steps"))
    if step is not None and max_steps is not None and max_steps > 0:
        lines.append(f"- Progress: step {step}/{max_steps} ({step / max_steps * 100.0:.1f}%)")
    elif step is not None:
        lines.append(f"- Progress: step {step}")
    reason = metrics.get("failure_reason")
    if reason:
        lines.append(f"- Reason: {reason}")

    grace = _integer(metrics.get("cargo_grace_steps"))
    loss_plane = _number(metrics.get("cargo_water_y"))
    initial = _integer(metrics.get("initial_cargo_count"))
    lost = _integer(metrics.get("cargo_lost_count"))
    if lost is None:
        lost = _integer(metrics.get("cargo_in_water"))
    current_below = _integer(metrics.get("cargo_currently_below_loss_plane"))
    retained = _integer(metrics.get("cargo_retained"))
    first_loss = _integer(metrics.get("cargo_loss_first_step"))
    lowest_y = _number(metrics.get("cargo_lowest_y"))
    lowest_step = _integer(metrics.get("cargo_lowest_y_step"))

    retention_section_start = len(lines)
    lines.extend(["", "### Retention and Stability"])
    if initial is not None and lost is not None:
        if retained is None:
            retained = max(0, initial - lost)
        line = f"- Episode retention: {retained}/{initial}; unique breaches: {lost}"
        if loss_plane is not None:
            line += f" below y={loss_plane:.2f} m"
        if grace is not None:
            line += f" after step {grace}"
        lines.append(line)
    if current_below is not None:
        lines.append(f"- Currently below loss plane: {current_below}")
    if first_loss is not None:
        lines.append(f"- First post-grace cargo breach: step {first_loss}")
    if lowest_y is not None:
        line = f"- Lowest observed cargo center: y={lowest_y:.3f} m"
        if lowest_step is not None:
            line += f" at step {lowest_step}"
        if loss_plane is not None:
            margin = lowest_y - loss_plane
            line += f"; margin {margin:+.3f} m"
        lines.append(line)

    peak_roll = _number(metrics.get("peak_abs_boat_angle_deg"))
    roll_limit = _number(metrics.get("boat_max_angle_deg"))
    final_roll = _number(metrics.get("boat_angle_deg"))
    if peak_roll is not None:
        line = f"- Peak absolute roll: {peak_roll:.2f} deg"
        if roll_limit is not None:
            line += f" / {roll_limit:.2f} deg; margin {roll_limit - peak_roll:+.2f} deg"
        lines.append(line)
    if final_roll is not None:
        lines.append(f"- Final absolute roll: {abs(final_roll):.2f} deg")
    capsize_step = _integer(metrics.get("capsize_first_step"))
    if capsize_step is not None:
        lines.append(f"- First roll-limit crossing: step {capsize_step}")
    peak_omega = _number(metrics.get("peak_angular_velocity_rad_s"))
    if peak_omega is not None:
        lines.append(
            f"- Peak angular speed: {peak_omega:.3f} rad/s "
            f"({math.degrees(peak_omega):.2f} deg/s)"
        )
    if len(lines) == retention_section_start + 2:
        del lines[retention_section_start:]

    structure_section_start = len(lines)
    lines.extend(["", "### Structure and Constraints"])
    initial_joints = _integer(metrics.get("initial_joint_count"))
    joints = _integer(metrics.get("joint_count"))
    broken = _integer(metrics.get("broken_joint_count"))
    if joints is not None and initial_joints is not None:
        if broken is None:
            broken = max(0, initial_joints - joints)
        lines.append(
            f"- Welds: {joints}/{initial_joints} remain; {broken} broken"
        )
    first_break = _integer(metrics.get("first_joint_break_step"))
    if first_break is not None:
        lines.append(f"- First weld break: step {first_break}")

    force_line = _ratio_line(
        "Peak weld force",
        metrics.get("joint_peak_force_N"),
        metrics.get("joint_max_force_N"),
        "N",
    )
    if force_line:
        lines.append(force_line)
    torque_line = _ratio_line(
        "Peak weld torque",
        metrics.get("joint_peak_torque_Nm"),
        metrics.get("joint_max_torque_Nm"),
        "N*m",
    )
    if torque_line:
        lines.append(torque_line)

    mass = _number(metrics.get("structure_mass"))
    mass_limit = _number(metrics.get("max_structure_mass"))
    if mass is not None and mass_limit is not None:
        lines.append(
            f"- Structure mass: {mass:.2f}/{mass_limit:.2f} kg; "
            f"margin {mass_limit - mass:+.2f} kg"
        )

    zone_values = tuple(
        _number(metrics.get(key))
        for key in (
            "build_zone_x_min",
            "build_zone_x_max",
            "build_zone_y_min",
            "build_zone_y_max",
        )
    )
    if all(value is not None for value in zone_values):
        x_min, x_max, y_min, y_max = zone_values
        lines.append(
            f"- Build zone: x=[{x_min:.2f}, {x_max:.2f}], "
            f"y=[{y_min:.2f}, {y_max:.2f}] m"
        )
    floor_margin = _number(metrics.get("lowest_beam_y_floor_margin"))
    if floor_margin is not None:
        lines.append(f"- Minimum beam-footprint y margin: {floor_margin:+.3f} m")

    milestones = metrics.get("cargo_retention_milestones")
    if isinstance(milestones, dict) and milestones:
        ordered = []
        for key, value in milestones.items():
            step_key = _integer(key)
            retained_value = _integer(value)
            if step_key is not None and retained_value is not None:
                ordered.append((step_key, retained_value))
        if ordered:
            ordered.sort()
            lines.append(
                "- Retention milestones: "
                + ", ".join(f"{step}:{count}" for step, count in ordered)
            )
    if len(lines) == structure_section_start + 2:
        del lines[structure_section_start:]

    warnings = _numerical_warnings(metrics)
    if warnings:
        lines.extend(["", "### Numerical Health"])
        lines.extend(f"- Warning: {warning}" for warning in warnings)
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
