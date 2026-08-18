from typing import Any, Dict, List

import math


def _number(value: Any):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _f(value: Any, decimals: int = 2) -> str:
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


def _zone_text(bounds: Dict[str, Any]) -> str:
    if not isinstance(bounds, dict):
        return "bounds unavailable"
    if "y_threshold" in bounds:
        return f"y>{_f(bounds.get('y_threshold'), 1)} m"
    return (
        f"x=[{_f(bounds.get('x_min'), 1)},{_f(bounds.get('x_max'), 1)}], "
        f"y=[{_f(bounds.get('y_min'), 1)},{_f(bounds.get('y_max'), 1)}] m"
    )


def _section_outcome(metrics: Dict[str, Any]) -> List[str]:
    success = metrics.get("success") is True
    failed = metrics.get("failed") is True
    reason = metrics.get("failure_reason")
    if success:
        status = "PASS"
    elif failed or reason:
        status = "FAIL"
    else:
        status = "INCOMPLETE"
    parts = ["### Outcome", f"- {status}"]
    if reason:
        parts.append(f"- Reason: {reason}")
    elif success:
        parts.append("- Reason: all evaluated success conditions were satisfied.")
    else:
        parts.append("- Reason: no terminal result was reported.")
    return parts


def _section_constraints(metrics: Dict[str, Any]) -> List[str]:
    parts = ["\n### 1. Constraint profile"]
    violations = metrics.get("constraint_violations")
    if isinstance(violations, list) and violations:
        parts.extend(f"- Build constraint: FAIL — {violation}" for violation in violations)
        mass = _number(metrics.get("structure_mass"))
        limit = _number(metrics.get("max_structure_mass"))
        if mass is not None and limit is not None:
            parts.append(f"- Mass measured/limit: {mass:.2f}/{limit:.2f} kg")
        parts.append(
            "- Allowed component-center bounds: "
            f"x=[{_f(metrics.get('build_zone_x_min'), 1)},{_f(metrics.get('build_zone_x_max'), 1)}], "
            f"y=[{_f(metrics.get('build_zone_y_min'), 1)},{_f(metrics.get('build_zone_y_max'), 1)}] m "
            f"with {_f(metrics.get('build_zone_tolerance'), 1)} m tolerance."
        )
        parts.append(
            "- Runtime delivery, integrity, and force usage were not evaluated after rejection; "
            f"configured delivery threshold={_f(metrics.get('min_delivery_ratio_percent'), 1)}%, "
            f"force cap={_f(metrics.get('force_budget'), 1)} N per step."
        )
        parts.append(f"- Simulation rejected at step {metrics.get('step_count', 'N/A')}.")
        return parts

    delivery = _number(metrics.get("delivery_ratio_percent"))
    minimum = _number(metrics.get("min_delivery_ratio_percent"))
    if delivery is not None and minimum is not None:
        margin = delivery - minimum
        state = "PASS" if margin >= 0.0 else "FAIL"
        parts.append(
            f"- Delivery: {state} — {delivery:.1f}% measured / {minimum:.1f}% required "
            f"(margin {margin:+.1f} percentage points)."
        )
    else:
        parts.append("- Delivery: measurement or threshold unavailable.")

    mass = _number(metrics.get("structure_mass"))
    mass_limit = _number(metrics.get("max_structure_mass"))
    if mass is not None and mass_limit is not None:
        margin = mass_limit - mass
        state = "PASS" if margin >= 0.0 else "FAIL"
        parts.append(
            f"- Mass: {state} — {mass:.2f}/{mass_limit:.2f} kg "
            f"(headroom {margin:+.2f} kg)."
        )

    if "structure_broken" in metrics:
        if metrics.get("structure_broken"):
            parts.append("- Integrity: FAIL — at least one initial joint was lost.")
        else:
            parts.append("- Integrity: PASS — initial joint count was preserved.")
    build_keys = (
        "build_zone_x_min", "build_zone_x_max",
        "build_zone_y_min", "build_zone_y_max", "build_zone_tolerance",
    )
    if all(_number(metrics.get(key)) is not None for key in build_keys):
        parts.append(
            "- Build/anchoring: PASS at initialization; component centers were within "
            f"x=[{_f(metrics.get('build_zone_x_min'), 1)},{_f(metrics.get('build_zone_x_max'), 1)}], "
            f"y=[{_f(metrics.get('build_zone_y_min'), 1)},{_f(metrics.get('build_zone_y_max'), 1)}] m "
            f"plus {_f(metrics.get('build_zone_tolerance'), 1)} m tolerance, and at least one joint existed."
        )
    else:
        parts.append("- Build/anchoring: status unavailable.")
    return parts


def _section_spatial(metrics: Dict[str, Any]) -> List[str]:
    parts = ["\n### 2. Spatial diagnostics"]
    initial = _number(metrics.get("initial_particle_count"))
    active = _number(metrics.get("particle_active_count"))
    if initial is not None and active is not None:
        parts.append(
            f"- Active/lost particles: {int(active)}/{int(initial)} active, "
            f"{int(initial - active)} lost."
        )
    parts.append(
        "- Distribution: "
        f"source={metrics.get('particles_in_source', 'N/A')}, "
        f"build={metrics.get('particles_in_build_zone', 'N/A')}, "
        f"target={metrics.get('particles_in_target', 'N/A')}."
    )
    parts.append(
        "- Target bounds: "
        f"x=[{_f(metrics.get('target_x_min'), 1)},{_f(metrics.get('target_x_max'), 1)}], "
        f"y=[{_f(metrics.get('target_y_min'), 1)},{_f(metrics.get('target_y_max'), 1)}] m."
    )

    if active is not None and active <= 0:
        parts.append("- No active particles remained; centroid and closest-particle position are unavailable.")
        return parts

    mean_x = _number(metrics.get("particle_mean_x"))
    mean_y = _number(metrics.get("particle_mean_y"))
    max_x = _number(metrics.get("particle_max_x"))
    if mean_x is not None and mean_y is not None:
        parts.append(f"- Active-particle centroid: ({mean_x:.2f},{mean_y:.2f}) m.")
    if max_x is not None:
        parts.append(f"- Rightmost active particle: x={max_x:.2f} m.")

    closest = _number(metrics.get("closest_particle_distance_to_target"))
    position = metrics.get("closest_particle_position")
    if closest is None or closest < 0.0:
        parts.append("- Closest active particle: unavailable.")
    elif closest == 0.0:
        parts.append("- Closest active particle: inside the target bounds.")
    else:
        position_text = ""
        if isinstance(position, (list, tuple)) and len(position) >= 2:
            position_text = f" at ({_f(position[0])},{_f(position[1])}) m"
        parts.append(f"- Closest active particle: {closest:.2f} m from target{position_text}.")
    return parts


def _section_hazards(metrics: Dict[str, Any]) -> List[str]:
    parts = ["\n### 3. Hazard interaction"]
    counts = metrics.get("hazard_losses")
    if not isinstance(counts, dict):
        parts.append("- Hazard measurements unavailable.")
        return parts
    bounds = metrics.get("hazard_zone_bounds") or {}
    positions = metrics.get("hazard_loss_positions") or {}
    loss_names = ("pit1", "pit2", "pit3", "out_of_bounds", "floor")
    total_lost = sum(int(counts.get(name, 0) or 0) for name in loss_names)
    initial = metrics.get("initial_particle_count", "N/A")
    parts.append(f"- Hazard-attributed losses: {total_lost}/{initial} particles.")
    for name in ("pit1", "pit2", "pit3"):
        count = int(counts.get(name, 0) or 0)
        if count <= 0:
            continue
        first_position = ""
        recorded = positions.get(name) or []
        if recorded and isinstance(recorded[0], (list, tuple)) and len(recorded[0]) >= 2:
            first_position = f"; first loss at ({_f(recorded[0][0])},{_f(recorded[0][1])}) m"
        parts.append(
            f"- {name}: {count} lost in {_zone_text(bounds.get(name, {}))}{first_position}."
        )
    if counts.get("out_of_bounds", 0):
        parts.append(f"- Out of bounds: {int(counts['out_of_bounds'])} lost.")
    if counts.get("floor", 0):
        parts.append(f"- Below floor: {int(counts['floor'])} lost.")
    parts.append(
        "- Exposure (particle-steps): "
        f"headwind={int(counts.get('headwind', 0) or 0)} "
        f"({_zone_text(bounds.get('headwind', {}))}); "
        f"gravity well={int(counts.get('gravwell', 0) or 0)} "
        f"({_zone_text(bounds.get('gravwell', {}))})."
    )
    return parts


def _section_actuation(metrics: Dict[str, Any]) -> List[str]:
    parts = ["\n### 4. Actuation and motion"]
    budget = _number(metrics.get("force_budget"))
    peak = _number(metrics.get("force_budget_peak_used"))
    last = _number(metrics.get("force_budget_last_used"))
    if budget is not None and peak is not None:
        parts.append(
            f"- Peak commanded force: {peak:.2f}/{budget:.2f} N per step "
            f"({peak / budget * 100.0:.1f}% used, {budget - peak:+.2f} N headroom)."
            if budget > 0.0
            else f"- Peak commanded force: {peak:.2f} N; configured budget is {budget:.2f} N."
        )
    if last is not None:
        parts.append(f"- Final simulated step commanded force: {last:.2f} N.")
    if budget is None:
        parts.append("- Force-use measurements unavailable.")

    zone_stats = metrics.get("zone_velocity_stats") or {}
    zone_order = (
        "source", "build_pre_pit3", "pit3_zone", "build_mid",
        "pit1_zone", "pit2_zone", "build_post_pit2", "target",
    )
    occupied = []
    for name in zone_order:
        stats = zone_stats.get(name)
        if not isinstance(stats, dict) or not stats.get("count"):
            continue
        occupied.append(
            f"{name}: n={int(stats['count'])}, "
            f"mean velocity=({_f(stats.get('mean_vx'))},{_f(stats.get('mean_vy'))}) m/s"
        )
    if occupied:
        parts.append("- Occupied-zone motion: " + "; ".join(occupied) + ".")
    elif not isinstance(metrics.get("zone_velocity_stats"), dict):
        parts.append("- Occupied-zone motion: velocity measurements unavailable.")
    else:
        parts.append("- Occupied-zone motion: no active particles in tracked transport zones.")
    return parts


def _section_timeline(metrics: Dict[str, Any]) -> List[str]:
    parts = ["\n### 5. Timeline"]
    step = metrics.get("step_count")
    maximum = metrics.get("max_steps")
    if step is not None:
        parts.append(f"- Evaluation ended at step {step}/{maximum}.")
    else:
        parts.append("- Evaluation end step unavailable.")
    timeline = metrics.get("transport_timeline")
    if not isinstance(timeline, dict):
        parts.append("- Transport milestones unavailable.")
        return parts
    milestone_labels = (
        ("first_source_exit_step", "First source exit"),
        ("first_build_zone_entry_step", "First build-zone entry"),
        ("first_target_entry_step", "First target entry"),
    )
    for key, label in milestone_labels:
        observed = timeline.get(key)
        parts.append(f"- {label}: " + (f"step {observed}." if observed is not None else "not observed."))
    peak_count = timeline.get("peak_particles_in_target")
    peak_step = timeline.get("peak_particles_in_target_step")
    if peak_count is not None:
        suffix = f" at step {peak_step}" if peak_step is not None else ""
        parts.append(f"- Peak simultaneous target occupancy: {peak_count} particles{suffix}.")

    first_exposures = timeline.get("first_hazard_exposure_steps") or {}
    exposure_events = [
        f"{name}=step {event_step}"
        for name, event_step in first_exposures.items()
        if event_step is not None
    ]
    if exposure_events:
        parts.append("- First hazard exposures: " + ", ".join(exposure_events) + ".")
    first_losses = timeline.get("first_hazard_loss_steps") or {}
    loss_events = [
        f"{name}=step {event_step}"
        for name, event_step in first_losses.items()
        if event_step is not None
    ]
    if loss_events:
        parts.append("- First hazard losses: " + ", ".join(loss_events) + ".")
    return parts


def _section_numerical_health(metrics: Dict[str, Any]) -> List[str]:
    bad = _non_finite_paths(metrics)
    if not bad:
        return ["\n### 6. Numerical health", "- All reported numeric metrics are finite."]
    displayed = ", ".join(bad[:8])
    suffix = f" (+{len(bad) - 8} more)" if len(bad) > 8 else ""
    return [
        "\n### 6. Numerical health",
        f"- {len(bad)} non-finite numeric value(s): {displayed}{suffix}.",
    ]


def format_task_metrics(metrics: Dict[str, Any]) -> List[str]:
    if not metrics:
        return ["**No evaluation metrics available.**"]
    parts = _section_outcome(metrics)
    diagnostic_keys = {
        "constraint_violations", "delivery_ratio_percent", "structure_mass",
        "particle_active_count", "hazard_losses", "force_budget", "step_count",
    }
    if not diagnostic_keys.intersection(metrics):
        parts.extend([
            "\n### Diagnostics",
            "- Task metrics are unavailable; refer to the execution error reported above.",
        ])
        parts.extend(_section_numerical_health(metrics))
        return parts
    parts.extend(_section_constraints(metrics))
    violations = metrics.get("constraint_violations")
    if not (isinstance(violations, list) and violations):
        parts.extend(_section_spatial(metrics))
        parts.extend(_section_hazards(metrics))
        parts.extend(_section_actuation(metrics))
        parts.extend(_section_timeline(metrics))
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
    if error:
        return [f"- Resolve the reported execution error: {error}"]
    return []
