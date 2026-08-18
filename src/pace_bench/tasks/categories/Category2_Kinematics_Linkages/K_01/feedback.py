from typing import Dict, Any, List, Optional

import math

def _is_finite_number(v: Any) -> bool:
    if v is None:
        return False
    try:
        f = float(v)
        return math.isfinite(f)
    except (TypeError, ValueError):
        return False

def _fmt(v: Any, decimals: int = 3) -> str:
    try:
        f = float(v)
        if not math.isfinite(f):
            return str(v)
        return f"{f:.{decimals}f}"
    except (TypeError, ValueError):
        return str(v)

def _is_likely_wheel_entry(e: Dict) -> bool:
    return isinstance(e, dict) and e.get("type") == "wheel" and "radius" in e

def _is_likely_beam_entry(e: Dict) -> bool:
    return isinstance(e, dict) and e.get("type") == "beam"

def _tier(v: float) -> str:
    if v is None:
        return "UNKNOWN"
    if v > 80:
        return "CRITICAL"
    if v > 50:
        return "ELEVATED"
    return "NOMINAL"

def _dim1_temporal_chronology(metrics: Dict[str, Any]) -> List[str]:
    parts: List[str] = []
    parts.append("### 1. Temporal\n")
    events: List[str] = []
    start_x = metrics.get("initial_x", 10.0)
    if _is_finite_number(start_x):
        events.append(f"Spawn: x={float(start_x):.2f} m")
    cs = metrics.get("first_collapse_step")
    if cs is not None and _is_finite_number(cs):
        events.append(f"Step {int(float(cs))}: collapse (y < threshold)")
    bz = metrics.get("first_bz_violation_step")
    if bz is not None and _is_finite_number(bz):
        events.append(f"Step {int(float(bz))}: build-zone violation")
    if metrics.get("joint_limit_hit") is True:
        events.append("Joint angle limit reached")
    sc = metrics.get("step_count", 0)
    wx = metrics.get("walker_x")
    wy = metrics.get("walker_y")
    if _is_finite_number(sc) and _is_finite_number(wx) and _is_finite_number(wy):
        sci = int(float(sc))
        dist = _fmt(metrics.get("distance_traveled", 0), 3)
        events.append(f"Step {sci} terminal: ({_fmt(wx, 2)}, {_fmt(wy, 2)}) m, Δx={dist} m")
    if not events:
        parts.append("No timeline events recorded.\n")
    elif len(events) == 1:
        parts.append(f"{events[0]}\n")
    else:
        parts.append(" →  ".join(events) + "\n")
    return parts

def _dim2_spatial_diagnostics(metrics: Dict[str, Any]) -> List[str]:
    parts: List[str] = []
    parts.append("### 2. Spatial\n")
    wx = metrics.get("walker_x")
    wy = metrics.get("walker_y")
    if _is_finite_number(wx) and _is_finite_number(wy):
        parts.append(f"Torso: ({_fmt(wx, 3)}, {_fmt(wy, 3)}) m")
    collapse_margin = metrics.get("collapse_margin")
    min_torso_h = float(metrics.get("min_torso_height", 1.2))
    if _is_finite_number(collapse_margin):
        cm = float(collapse_margin)
        if cm >= 0:
            parts.append(f" | Collapse margin: {cm:+.3f} m (>{min_torso_h:.2f})")
            if cm < 0.10:
                parts.append(f" ⚠️ {cm*100:.1f} cm from failure")
        else:
            parts.append(f" | COLLAPSED: {cm:.3f} m below {min_torso_h:.2f} threshold")
    gc = metrics.get("ground_contact_margin")
    ground_y = float(metrics.get("ground_y", 1.0))
    if _is_finite_number(gc):
        gcv = float(gc)
        parts.append(f" | Ground clearance: {gcv:+.3f} m")
        if 0 <= gcv < 0.05:
            parts.append(" ⚠️ near contact")
        elif gcv < 0:
            parts.append(" ⚠️ below ground")
    target_x = metrics.get("target_x")
    if _is_finite_number(target_x) and _is_finite_number(wx):
        gap = float(target_x) - float(wx)
        parts.append(f" | To target x={float(target_x):.1f}: {gap:.1f} m remaining")
    parts.append("\n")
    wheel_clr = metrics.get("wheel_ground_clearances", [])
    if wheel_clr:
        touching_count = sum(1 for wc in wheel_clr if wc.get("touching_ground", False))
        total_wheels = len(wheel_clr)
        parts.append(f"Wheels: {touching_count}/{total_wheels} in ground contact")
        if touching_count == 0 and total_wheels > 0:
            parts.append(" ⚠️ no traction")
        if 0 < touching_count < total_wheels:
            parts.append("  —  ")
            touching_indices = [wc.get("body_index", "?") for wc in wheel_clr if wc.get("touching_ground")]
            parts.append(f"contact: #{', #'.join(str(i) for i in touching_indices)}")
        parts.append("\n")
    max_x = metrics.get("max_x_reached")
    if _is_finite_number(max_x) and _is_finite_number(wx):
        if float(max_x) > float(wx):
            regression = float(max_x) - float(wx)
            parts.append(f"⚠️ Regression: peak x={_fmt(max_x, 3)} m → fell back by {regression:.3f} m\n")
    return parts

def _dim3_load_distribution(metrics: Dict[str, Any]) -> List[str]:
    parts: List[str] = []
    parts.append("### 3. Load & Stress\n")
    mass = metrics.get("structure_mass")
    max_mass = metrics.get("max_structure_mass")
    if _is_finite_number(mass) and _is_finite_number(max_mass) and float(max_mass) > 0:
        mass_util = float(mass) / float(max_mass) * 100
        margin = float(max_mass) - float(mass)
        tier = _tier(mass_util)
        parts.append(
            f"Mass: {_fmt(mass, 3)} / {_fmt(max_mass, 2)} kg "
            f"({mass_util:.1f}%, margin {margin:+.3f} kg) — {tier}\n"
        )
    body_details = metrics.get("body_details", [])
    nw = sum(1 for b in body_details if _is_likely_wheel_entry(b))
    nbeam = sum(1 for b in body_details if _is_likely_beam_entry(b))
    nj = metrics.get("num_joints", 0)
    parts.append(f"Components: {nw} wheel(s), {nbeam} beam(s), {nj} joint(s)\n")
    per_joint = metrics.get("per_joint_details", [])
    if not per_joint:
        max_angle = metrics.get("max_joint_angle_abs")
        j_lo = metrics.get("default_joint_lower_limit")
        j_hi = metrics.get("default_joint_upper_limit")
        if _is_finite_number(max_angle):
            parts.append(f"Peak |joint angle|: {float(max_angle):.3f} rad")
            if _is_finite_number(j_lo) and _is_finite_number(j_hi):
                rng = float(j_hi) - float(j_lo)
                if rng > 0:
                    util = abs(float(max_angle)) / rng * 100
                    parts.append(f" ({util:.1f}% of range) — {_tier(util)}\n")
        return parts
    records = []
    for j in per_joint:
        if j.get("type") != "revolute":
            continue
        angle = j.get("current_angle", 0.0)
        lo = j.get("lower_limit")
        hi = j.get("upper_limit")
        angle_util = None
        if lo is not None and hi is not None:
            rng = float(hi) - float(lo)
            if rng > 0:
                angle_util = abs(float(angle)) / rng * 100.0
        torque_util = None
        motor_torque = j.get("motor_torque")
        max_torque = j.get("max_torque", 0)
        if motor_torque is not None and _is_finite_number(max_torque) and float(max_torque) > 0:
            torque_util = abs(float(motor_torque)) / float(max_torque) * 100.0
        motor_active = bool(j.get("motor_enabled", False)) and float(j.get("motor_speed", 0)) != 0
        records.append({
            "index": j.get("index", "?"),
            "anchor": (j.get("anchor_x", 0), j.get("anchor_y", 0)),
            "angle": angle,
            "angle_util": angle_util,
            "torque_util": torque_util,
            "motor_active": motor_active,
            "motor_speed": j.get("motor_speed", 0),
            "max_torque": j.get("max_torque", 0),
        })
    active_count = sum(1 for r in records if r["motor_active"])
    n_revolute = len(records)
    all_nominal = all(
        (r["angle_util"] is None or r["angle_util"] <= 50) and
        (r["torque_util"] is None or r["torque_util"] <= 50)
        for r in records
    )
    any_active = active_count > 0
    if all_nominal and not any_active:
        parts.append(f"Joints ({n_revolute} revolute): all IDLE, all NOMINAL\n")
    else:
        notable = [r for r in records if (
            (r["angle_util"] is not None and r["angle_util"] > 50) or
            (r["torque_util"] is not None and r["torque_util"] > 50) or
            r["motor_active"]
        )]
        if notable:
            parts.append(f"Joints ({n_revolute} revolute, {active_count} active, {len(notable)} notable):\n")
            for rec in notable:
                idx = rec["index"]
                ax, ay = rec["anchor"]
                au = rec["angle_util"]
                tu = rec["torque_util"]
                max_util = max(au if au is not None else 0, tu if tu is not None else 0)
                tier = _tier(max_util)
                detail = []
                if au is not None:
                    detail.append(f"angle {au:.1f}%")
                if tu is not None:
                    detail.append(f"torque {tu:.1f}%")
                if rec["motor_active"]:
                    detail.append(f"ON speed={_fmt(rec['motor_speed'], 1)}")
                else:
                    detail.append("IDLE")
                parts.append(
                    f"  - Joint #{idx} ({_fmt(ax, 2)}, {_fmt(ay, 2)}): "
                    f"{', '.join(detail)} — {tier}\n"
                )
        else:
            parts.append(f"Joints ({n_revolute} revolute): all NOMINAL ({active_count} active)\n")
    saturated = [r for r in records if r["torque_util"] is not None and r["torque_util"] > 90]
    if saturated:
        sat_indices = [str(r["index"]) for r in saturated]
        parts.append(
            f"⚠️ Torque saturation: {len(saturated)} joint(s) (#{', #'.join(sat_indices)}) "
            f"at >90% max — motor may be insufficient\n"
        )
    return parts

def _dim4_energy_power_flow(metrics: Dict[str, Any]) -> List[str]:
    parts: List[str] = []
    parts.append("### 4. Energy\n")
    per_joint = metrics.get("per_joint_details", [])
    active_motors = 0
    for j in per_joint:
        if j.get("type") != "revolute":
            continue
        if j.get("motor_enabled", False) and float(j.get("motor_speed", 0)) != 0:
            active_motors += 1
    if active_motors > 0:
        parts.append(f"Active motors: {active_motors}")
    else:
        parts.append("No active motors")
    vx = metrics.get("walker_vx")
    vy = metrics.get("walker_vy")
    if _is_finite_number(vx) and _is_finite_number(vy):
        speed = math.hypot(float(vx), float(vy))
        if float(vx) > 0.0001:
            direction = f"{_fmt(vx, 4)} forward"
        elif float(vx) < -0.0001:
            direction = f"{_fmt(vx, 4)} backward ⚠️"
        else:
            direction = "neutral"
        parts.append(f" | |v|={speed:.4f} m/s ({direction})")
        if speed < 0.0005 and active_motors > 0:
            parts.append(" ⚠️ stalled")
    parts.append("\n")
    return parts

def _dim5_constraint_profile(metrics: Dict[str, Any]) -> List[str]:
    parts: List[str] = []
    parts.append("### 5. Constraints\n")
    constraints: List[str] = []
    mass = metrics.get("structure_mass")
    max_mass = metrics.get("max_structure_mass")
    if _is_finite_number(mass) and _is_finite_number(max_mass) and float(max_mass) > 0:
        m = float(mass)
        mm = float(max_mass)
        passed = m <= mm
        status = "PASS" if passed else "FAIL"
        util = m / mm * 100
        constraints.append(f"{'✅' if passed else '❌'} Mass ({status}): {util:.1f}% of {mm:.1f} kg")
    collapse_margin = metrics.get("collapse_margin")
    min_h = float(metrics.get("min_torso_height", 1.2))
    if _is_finite_number(collapse_margin):
        cm = float(collapse_margin)
        passed = cm >= 0
        constraints.append(f"{'✅' if passed else '❌'} Height ({'PASS' if passed else 'FAIL'}): margin {cm:+.3f} m")
    bz_x_min = float(metrics.get("build_zone_x_min", 0.0))
    bz_x_max = float(metrics.get("build_zone_x_max", 50.0))
    bz_y_max = float(metrics.get("build_zone_y_max", 10.0))
    wx = metrics.get("walker_x")
    wy = metrics.get("walker_y")
    if _is_finite_number(wx) and _is_finite_number(wy):
        x_ok = bz_x_min <= float(wx) <= bz_x_max
        y_ok = min_h <= float(wy) <= bz_y_max
        bz_ok = x_ok and y_ok
        constraints.append(f"{'✅' if bz_ok else '❌'} Zone ({'PASS' if bz_ok else 'FAIL'}): "
                          f"({float(wx):.1f}, {float(wy):.1f}) in [{bz_x_min:.0f},{bz_x_max:.0f}]×[{min_h:.1f},{bz_y_max:.0f}]")
    joint_limit_hit = metrics.get("joint_limit_hit")
    max_angle_abs = metrics.get("max_joint_angle_abs")
    j_lo = metrics.get("default_joint_lower_limit")
    j_hi = metrics.get("default_joint_upper_limit")
    if _is_finite_number(j_lo) and _is_finite_number(j_hi):
        if joint_limit_hit is True:
            constraints.append(f"❌ Joint angles (FAIL): limit [{float(j_lo):.3f}, {float(j_hi):.3f}] hit")
        elif _is_finite_number(max_angle_abs):
            rng = float(j_hi) - float(j_lo)
            if rng > 0:
                util = abs(float(max_angle_abs)) / rng * 100
                constraints.append(f"{'⚠️ ' if util > 70 else '✅'} Joint angles ({'NEAR' if util > 70 else 'PASS'}): {util:.1f}% of range")
    steps = metrics.get("step_count")
    req_steps = metrics.get("min_simulation_steps_required")
    if _is_finite_number(steps) and _is_finite_number(req_steps) and float(req_steps) > 0:
        s = float(steps)
        rs = float(req_steps)
        passed = s >= rs
        pct = min(s / rs * 100, 100.0)
        constraints.append(f"{'✅' if passed else '❌'} Duration ({'PASS' if passed else 'FAIL'}): {int(s)}/{int(rs)} steps ({pct:.1f}%)")
    td = metrics.get("target_distance_val")
    dist = metrics.get("distance_traveled")
    if _is_finite_number(td) and _is_finite_number(dist):
        d = float(dist)
        tdd = float(td)
        passed = d >= tdd
        remaining = max(0.0, tdd - d)
        complete = max(0.0, min(100.0, d / tdd * 100)) if tdd > 0 else 0
        constraints.append(f"{'✅' if passed else '❌'} Distance ({'PASS' if passed else 'FAIL'}): "
                          f"{d:+.3f}/{tdd:.1f} m ({complete:.1f}%)")
    for c in constraints:
        parts.append(f"  {c}\n")
    env_parts = []
    mbf = metrics.get("max_body_friction")
    if _is_finite_number(mbf):
        env_parts.append(f"bf={float(mbf):.4f}")
    if _is_finite_number(j_lo) and _is_finite_number(j_hi):
        env_parts.append(f"jlim=[{float(j_lo):.4f},{float(j_hi):.4f}]")
    if env_parts:
        parts.append(f"  Env: {'  '.join(env_parts)}\n")
    return parts

def _dim6_numerical_health(metrics: Dict[str, Any]) -> List[str]:
    parts: List[str] = []
    parts.append("### 6. Health\n")
    issues: List[str] = []
    scalar_keys = [
        "walker_x", "walker_y", "walker_vx", "walker_vy",
        "distance_traveled", "max_x_reached", "min_torso_y",
        "progress", "step_count", "structure_mass", "max_structure_mass",
        "collapse_margin", "ground_contact_margin",
        "torso_angular_velocity", "max_joint_angle_abs",
    ]
    for k in scalar_keys:
        v = metrics.get(k)
        if v is None:
            continue
        try:
            f = float(v)
            if math.isnan(f):
                issues.append(f"NaN in `{k}`")
            elif math.isinf(f):
                issues.append(f"Inf in `{k}`")
        except (TypeError, ValueError):
            issues.append(f"Non-numeric `{k}`: {type(v).__name__}")
    vx = metrics.get("walker_vx")
    vy = metrics.get("walker_vy")
    if _is_finite_number(vx) and _is_finite_number(vy):
        speed = math.hypot(float(vx), float(vy))
        if speed > 100:
            issues.append(f"Extreme |v|={speed:.1f} m/s — possible solver divergence")
        elif speed > 50:
            issues.append(f"High |v|={speed:.1f} m/s")
    av = metrics.get("torso_angular_velocity")
    if _is_finite_number(av):
        if abs(float(av)) > 100:
            issues.append(f"Extreme |ω|={abs(float(av)):.1f} rad/s")
    nb = metrics.get("num_bodies", 0)
    if nb == 0:
        issues.append("Zero dynamic bodies — no mechanism built")
    per_joint = metrics.get("per_joint_details", [])
    for j in per_joint:
        ji = j.get("index", "?")
        for field in ("current_angle", "motor_torque", "motor_speed"):
            val = j.get(field)
            if val is not None:
                try:
                    f = float(val)
                    if math.isnan(f):
                        issues.append(f"NaN in joint #{ji} `{field}`")
                    elif math.isinf(f):
                        issues.append(f"Inf in joint #{ji} `{field}`")
                except (TypeError, ValueError):
                    pass
    if not issues:
        parts.append("✅ All values normal — no NaN, Inf, or solver divergence.\n")
    else:
        for issue in issues:
            parts.append(f"  ⚠️ {issue}\n")
    return parts

def format_task_metrics(metrics: Dict[str, Any]) -> List[str]:
    if not metrics:
        return ["**No task metrics available.**"]
    parts: List[str] = []
    try:
        critical_keys = (
            "walker_x", "walker_y", "distance_traveled", "max_x_reached",
            "min_torso_y", "progress", "step_count", "structure_mass",
            "max_structure_mass", "min_simulation_steps_required", "target_x",
        )
        for k in critical_keys:
            if k in metrics and not _is_finite_number(metrics[k]):
                parts.append(f"Outcome: INVALID METRICS — non-finite `{k}`={metrics[k]}")
                return parts
        if metrics.get("failed"):
            parts.append("Outcome: ❌ FAILED\n")
        elif metrics.get("success"):
            parts.append("Outcome: ✅ SUCCESS\n")
        else:
            parts.append("Outcome: ⚠️ INCOMPLETE\n")
        parts.extend(_dim1_temporal_chronology(metrics))
        parts.extend(_dim2_spatial_diagnostics(metrics))
        parts.extend(_dim3_load_distribution(metrics))
        parts.extend(_dim4_energy_power_flow(metrics))
        parts.extend(_dim5_constraint_profile(metrics))
        parts.extend(_dim6_numerical_health(metrics))
    except Exception as exc:
        parts = [f"Outcome: FEEDBACK ERROR — {type(exc).__name__}: {exc}"]
    if not parts:
        try:
            from pace_bench.evaluation.verification.diagnostics import (
                format_generic_execution_metrics,
            )
            parts = format_generic_execution_metrics(metrics)
        except Exception as exc:
            parts = [f"Outcome: GENERIC FEEDBACK ERROR — {type(exc).__name__}: {exc}"]
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
        return ["- Code execution failed. Review the execution error above."]
    return []
