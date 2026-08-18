from typing import Dict, Any, List, Optional

import math

def _f(val: Any) -> Optional[float]:
    if val is None:
        return None
    try:
        fv = float(val)
        return fv if math.isfinite(fv) else None
    except (TypeError, ValueError):
        return None

def _is_finite(val: Any) -> bool:
    return _f(val) is not None

def _margin(actual: Any, limit: Any) -> Optional[float]:
    a = _f(actual)
    l = _f(limit)
    if a is None or l is None:
        return None
    return a - l

def _pct(numer: Any, denom: Any) -> Optional[float]:
    n = _f(numer)
    d = _f(denom)
    if n is None or d is None or d == 0.0:
        return None
    return (n / d) * 100.0

def _fmt_val(val: Any, unit: str = "") -> str:
    v = _f(val)
    if v is None:
        return "n/a"
    return f"{v:.3f}{unit}"


def _int_or_default(val: Any, default: int = 0) -> int:
    value = _f(val)
    return int(value) if value is not None else default

def _changed(val: Any, prev_val: Any, tol: float = 1e-6) -> bool:
    v = _f(val)
    pv = _f(prev_val)
    if v is None or pv is None:
        return v is not None or pv is not None
    return abs(v - pv) > tol

def _is_new_run(metrics: Dict[str, Any], prev: Dict[str, Any]) -> bool:
    curr_step = _f(metrics.get("step_count"))
    prev_step = _f(prev.get("step_count"))
    if curr_step is not None and prev_step is not None and curr_step < prev_step:
        return True
    curr_init_y = _f(metrics.get("initial_object_y"))
    prev_init_y = _f(prev.get("initial_object_y"))
    if curr_init_y is not None and prev_init_y is not None and abs(curr_init_y - prev_init_y) > 0.001:
        return True
    return False

def _format_numerical_health(metrics: Dict[str, Any]) -> Optional[str]:
    issues: List[str] = []
    critical_keys = [
        "object_x", "object_y", "gripper_x", "gripper_y",
        "slider_body_y", "slider_translation", "min_finger_tip_y",
        "structure_mass", "object_mass",
    ]
    for k in critical_keys:
        v = metrics.get(k)
        if v is not None:
            try:
                fv = float(v)
                if not math.isfinite(fv):
                    issues.append(f"Non-finite: {k}={v}")
            except (TypeError, ValueError):
                issues.append(f"Non-numeric: {k}={v}")
    obj_vel = metrics.get("object_velocity") or {}
    speed = _f(obj_vel.get("speed")) if obj_vel else None
    peak_speed = _f(metrics.get("peak_object_speed"))
    if speed is not None and speed > 100.0:
        issues.append(f"Object speed {speed:.1f} m/s >100 — physics explosion")
    elif speed is not None and speed > 20.0:
        issues.append(f"Object speed {speed:.1f} m/s high (>20)")
    if peak_speed is not None and peak_speed > 100.0:
        issues.append(f"Peak speed {peak_speed:.1f} m/s — solver divergence")
    obj_y = _f(metrics.get("object_y"))
    if obj_y is not None and obj_y < -50.0:
        issues.append(f"Object y={obj_y:.1f}m far below world")
    elif obj_y is not None and obj_y < 0.0:
        issues.append(f"Object y={obj_y:.2f}m below ground (y=0)")
    obj_x = _f(metrics.get("object_x"))
    if obj_x is not None and abs(obj_x) > 50.0:
        issues.append(f"Object x={obj_x:.1f}m far outside expected range")
    angular_vel = _f(obj_vel.get("angular_vel")) if obj_vel else None
    if angular_vel is not None and abs(angular_vel) > 20.0:
        issues.append(f"Angular velocity {angular_vel:.1f} rad/s extreme")
    slider_trans = _f(metrics.get("slider_translation"))
    slider_comp = _f(metrics.get("slider_computed_translation"))
    if slider_trans is not None and slider_comp is not None:
        if abs(slider_trans - slider_comp) > 0.5 and slider_comp > 0.01:
            issues.append(
                f"Slider translation mismatch: API={slider_trans:.3f}m, body-delta={slider_comp:.3f}m"
            )
    min_obj_y = _f(metrics.get("min_object_y_seen"))
    if min_obj_y is not None and min_obj_y < -10.0:
        issues.append(f"Min object y={min_obj_y:.1f}m — catastrophic fall")
    observation_errors = metrics.get("observation_error_count", 0)
    if isinstance(observation_errors, (int, float)) and observation_errors > 0:
        detail = metrics.get("last_observation_error") or "details unavailable"
        issues.append(f"Observation API errors: {_int_or_default(observation_errors)}; last={detail}")
    if issues:
        return "## Numerical Health\n" + "\n".join(f"- ⚠️ {iss}" for iss in issues)
    return None

def _format_environment_context(metrics: Dict[str, Any]) -> List[str]:
    parts = ["## Environment Context"]
    obj_shape = metrics.get("object_shape")
    obj_w = _f(metrics.get("object_width"))
    obj_h = _f(metrics.get("object_height"))
    obj_r = _f(metrics.get("object_radius"))
    if obj_shape:
        line = f"- **Target object**: {obj_shape}"
        if obj_w is not None and obj_h is not None:
            line += f", {obj_w:.2f}×{obj_h:.2f}m"
        elif obj_r is not None:
            line += f", r={obj_r:.3f}m"
        parts.append(line)
    mass = _f(metrics.get("structure_mass"))
    max_mass = _f(metrics.get("max_structure_mass"))
    if mass is not None:
        line = f"- **Structure mass**: {mass:.2f}kg"
        if max_mass is not None and max_mass > 0:
            pct_v = mass / max_mass * 100.0
            line += f" / {max_mass:.2f}kg ({pct_v:.1f}%)"
        parts.append(line)
    step_cnt = _f(metrics.get("step_count"))
    time_step = _f(metrics.get("time_step"))
    if step_cnt is not None and time_step is not None and time_step > 0:
        sim_time = float(step_cnt) * time_step
        parts.append(f"- **Simulation**: {_int_or_default(step_cnt)} steps ({sim_time:.2f}s)")
    elif step_cnt is not None:
        parts.append(f"- **Simulation**: {_int_or_default(step_cnt)} steps")
    return parts

def _format_event_timeline(metrics: Dict[str, Any], prev_metrics: Optional[Dict[str, Any]] = None) -> Optional[List[str]]:
    timeline = metrics.get("event_timeline") or []
    prev_timeline = prev_metrics.get("event_timeline") or [] if prev_metrics else []
    prev_event_count = len(prev_timeline)
    new_events = timeline[prev_event_count:] if prev_metrics else timeline
    if not timeline:
        step_cnt = metrics.get("step_count", 0)
        obj_grasped = metrics.get("object_grasped", False)
        obj_fell = metrics.get("object_fell", False)
        contact_pts = metrics.get("object_contact_points", 0)
        lines = ["## Event Chronology", "- No events recorded; terminal snapshot only:"]
        lines.append(f"  Step {_int_or_default(step_cnt)}")
        if obj_grasped:
            lines.append("  Object grasped at some point")
        if obj_fell:
            min_y = metrics.get("min_object_y_seen", "?")
            lines.append(f"  Object fell (min y={_fmt_val(min_y, 'm')})")
        if contact_pts and _int_or_default(contact_pts) == 0 and not obj_grasped:
            lines.append("  Zero contact points at terminal snapshot")
        return lines
    parts = []
    if prev_metrics is not None and not new_events and prev_event_count > 0:
        total_contact = metrics.get("total_contact_steps", 0)
        lost_count = metrics.get("contact_lost_count", 0)
        persist = metrics.get("contact_persistence_steps", 0)
        prev_contact = prev_metrics.get("total_contact_steps", 0)
        prev_lost = prev_metrics.get("contact_lost_count", 0)
        if (total_contact != prev_contact or lost_count != prev_lost):
            parts.append(f"## Event Chronology (updated)")
            parts.append(
                f"- No new events since last moment. "
                f"Contact: {_int_or_default(total_contact)} steps "
                f"({_int_or_default(lost_count)} losses), "
                f"{_int_or_default(persist)} consecutive at end."
            )
            return parts
        return None
    if prev_metrics is not None:
        parts.append(f"## Event Chronology (+{len(new_events)} new events)")
    else:
        parts.append("## Event Chronology")
    parts.append(f"- **{len(timeline)} total events** ({len(new_events)} new this moment):")
    for ev in new_events:
        step = _int_or_default(ev.get("step", 0))
        event_type = ev.get("event", "unknown")
        obj_y = ev.get("object_y")
        obj_x = ev.get("object_x")
        if event_type == "contact_lost":
            pos_str = ""
            if _is_finite(obj_x) and _is_finite(obj_y):
                pos_str = f" at ({_f(obj_x):.2f}, {_f(obj_y):.2f})m"
            parts.append(f"  Step {step}: **CONTACT LOST**{pos_str}")
        elif event_type == "grasp_acquired":
            pos_str = ""
            if _is_finite(obj_x) and _is_finite(obj_y):
                pos_str = f" at ({_f(obj_x):.2f}, {_f(obj_y):.2f})m"
            parts.append(f"  Step {step}: **GRASP ACQUIRED**{pos_str}")
        elif event_type == "object_fell":
            min_y = ev.get("min_object_y_seen")
            min_h = ev.get("min_object_height")
            detail = ""
            if _is_finite(min_y):
                detail = f" | min y={_f(min_y):.2f}m"
                if _is_finite(min_h):
                    detail += f" (required >={_f(min_h):.2f}m)"
            parts.append(f"  Step {step}: **OBJECT FELL**{detail}")
        else:
            pos_str = ""
            if _is_finite(obj_x) and _is_finite(obj_y):
                pos_str = f" at ({_f(obj_x):.2f}, {_f(obj_y):.2f})m"
            parts.append(f"  Step {step}: {event_type}{pos_str}")
    total_contact = metrics.get("total_contact_steps", 0)
    lost_count = metrics.get("contact_lost_count", 0)
    persist = metrics.get("contact_persistence_steps", 0)
    if total_contact is not None and total_contact > 0:
        parts.append(
            f"- Contact: {_int_or_default(total_contact)} steps with contact, "
            f"{_int_or_default(lost_count)} losses, "
            f"{_int_or_default(persist)} consecutive at end"
        )
    return parts

def _format_spatial_diagnostics(metrics: Dict[str, Any], prev_metrics: Optional[Dict[str, Any]] = None) -> Optional[List[str]]:
    parts = ["## Spatial Diagnostics"]
    delta_mode = prev_metrics is not None
    has_any = False
    obj_x = _f(metrics.get("object_x"))
    obj_y = _f(metrics.get("object_y"))
    gx = _f(metrics.get("gripper_x"))
    gy = _f(metrics.get("gripper_y"))
    target_y = _f(metrics.get("target_object_y"))
    init_obj_y = _f(metrics.get("initial_object_y"))
    prev_obj_x = _f(prev_metrics.get("object_x")) if prev_metrics else None
    prev_obj_y = _f(prev_metrics.get("object_y")) if prev_metrics else None
    obj_moved = not delta_mode or _changed(obj_y, prev_obj_y, 0.01) or _changed(obj_x, prev_obj_x, 0.01)
    if obj_moved or not delta_mode:
        if obj_x is not None and obj_y is not None:
            line = f"- **Object**: ({obj_x:.2f}, {obj_y:.2f})m"
            if init_obj_y is not None:
                dy = obj_y - init_obj_y
                line += f" | Δy={dy:+.2f}m"
            if target_y is not None:
                margin_to_target = obj_y - target_y
                sev = "✓" if margin_to_target >= 0 else ("⚠" if margin_to_target >= -0.5 else "✗")
                line += f" | margin to y={target_y:.2f}: {margin_to_target:+.2f}m [{sev}]"
            parts.append(line)
            has_any = True
    prev_gx = _f(prev_metrics.get("gripper_x")) if prev_metrics else None
    prev_gy = _f(prev_metrics.get("gripper_y")) if prev_metrics else None
    gripper_moved = not delta_mode or _changed(gx, prev_gx, 0.01) or _changed(gy, prev_gy, 0.01)
    if gripper_moved or not delta_mode:
        if gx is not None and gy is not None:
            if obj_x is not None and obj_y is not None:
                dist = math.hypot(gx - obj_x, gy - obj_y)
                parts.append(f"- **Gripper**: ({gx:.2f}, {gy:.2f})m | sep={dist:.2f}m")
            else:
                parts.append(f"- **Gripper**: ({gx:.2f}, {gy:.2f})m")
            has_any = True
    slider_body_y = _f(metrics.get("slider_body_y"))
    slider_anchor_y = _f(metrics.get("slider_anchor_y"))
    slider_trans = _f(metrics.get("slider_translation"))
    slider_comp_trans = _f(metrics.get("slider_computed_translation"))
    slider_lo = _f(metrics.get("slider_lower_limit"))
    slider_hi = _f(metrics.get("slider_upper_limit"))
    slider_motor_spd = _f(metrics.get("slider_motor_speed"))
    prev_slider_y = _f(prev_metrics.get("slider_body_y")) if prev_metrics else None
    slider_moved = not delta_mode or _changed(slider_body_y, prev_slider_y, 0.01)
    if slider_body_y is not None and slider_moved:
        parts.append("- **Slider**:")
        if slider_anchor_y is not None:
            depth = slider_anchor_y - slider_body_y
            range_str = ""
            if slider_lo is not None and slider_hi is not None:
                range_total = slider_hi - slider_lo
                if range_total > 0 and slider_anchor_y is not None:
                    depth_val = slider_anchor_y - slider_body_y
                    used_pct = max(0.0, min(100.0, (depth_val - slider_lo) / range_total * 100.0))
                    margin_to_max = slider_hi - depth_val
                    sev = "CRITICAL" if margin_to_max < 0 else ("NEAR" if margin_to_max < 0.5 else "OK")
                    range_str = f" | {used_pct:.0f}% of [{slider_lo:.1f},{slider_hi:.1f}]m | margin to max {margin_to_max:+.2f}m [{sev}]"
            parts.append(f"  y={slider_body_y:.3f}m, depth={depth:.3f}m{range_str}")
            if slider_comp_trans is not None:
                t_part = f"  translation={slider_comp_trans:.3f}m"
                if slider_motor_spd is not None and abs(slider_motor_spd) > 0.001:
                    direction = "↓" if slider_motor_spd > 0 else "↑"
                    t_part += f" | motor={slider_motor_spd:.2f}m/s {direction}"
                elif slider_motor_spd is not None:
                    t_part += " | motor=stopped"
                parts.append(t_part)
            has_any = True
    min_finger_y = _f(metrics.get("min_finger_tip_y"))
    prev_min_fy = _f(prev_metrics.get("min_finger_tip_y")) if prev_metrics else None
    fingers_moved = not delta_mode or _changed(min_finger_y, prev_min_fy, 0.01)
    if min_finger_y is not None and fingers_moved:
        if obj_y is not None:
            vert_gap = min_finger_y - obj_y
            sev = "BELOW" if vert_gap < 0 else "ABOVE"
            parts.append(f"- **Fingers**: lowest tip y={min_finger_y:.3f}m | gap to obj={vert_gap:+.3f}m ({sev})")
        else:
            parts.append(f"- **Fingers**: lowest tip y={min_finger_y:.3f}m")
        has_any = True
    finger_states = metrics.get("finger_joint_states") or []
    prev_finger_states = prev_metrics.get("finger_joint_states") or [] if prev_metrics else []
    if finger_states:
        angles_str_parts = []
        near_any = False
        for i, st in enumerate(finger_states):
            ang = _f(st.get("angle_deg"))
            if ang is not None:
                lo = _f(st.get("lower_limit"))
                hi = _f(st.get("upper_limit"))
                lo_deg = math.degrees(lo) if lo is not None else None
                hi_deg = math.degrees(hi) if hi is not None else None
                near = False
                if lo_deg is not None and hi_deg is not None:
                    margin_lo = ang - lo_deg
                    margin_hi = hi_deg - ang
                    near = (margin_lo < 5.0 or margin_hi < 5.0)
                    if near:
                        near_any = True
                prev_ang = None
                if prev_finger_states and i < len(prev_finger_states):
                    prev_ang = _f(prev_finger_states[i].get("angle_deg"))
                ang_changed = not delta_mode or _changed(ang, prev_ang, 0.5)
                if ang_changed or near:
                    if near:
                        angles_str_parts.append(f"#{i+1}={ang:.1f}° [NEAR LIMIT]")
                    else:
                        angles_str_parts.append(f"#{i+1}={ang:.1f}°")
        if angles_str_parts:
            parts.append(f"- **Finger angles**: {', '.join(angles_str_parts)}")
            has_any = True
    plat_top = _f(metrics.get("platform_top_y"))
    if plat_top is not None and obj_y is not None:
        plat_margin = obj_y - plat_top
        if plat_margin <= 0.5:
            sev = "BELOW" if plat_margin <= 0 else "ABOVE"
            parts.append(f"- **Platform**: top y={plat_top:.2f}m | obj margin={plat_margin:+.2f}m [{sev}]")
            has_any = True
    bx_min = _f(metrics.get("build_zone_x_min"))
    bx_max = _f(metrics.get("build_zone_x_max"))
    by_min = _f(metrics.get("build_zone_y_min"))
    by_max = _f(metrics.get("build_zone_y_max"))
    if all(v is not None for v in [bx_min, bx_max, by_min, by_max]) and gx is not None and gy is not None:
        x_lo = gx - bx_min
        x_hi = bx_max - gx
        y_lo = gy - by_min
        y_hi = by_max - gy
        nearest = min(x_lo, x_hi, y_lo, y_hi)
        if nearest < 3.0:
            parts.append(
                f"- **Build zone** [{bx_min:.0f},{bx_max:.0f}]×[{by_min:.0f},{by_max:.0f}]m: "
                f"gripper margins x={x_lo:+.1f}/{x_hi:+.1f}, y={y_lo:+.1f}/{y_hi:+.1f}m"
            )
            has_any = True
    if not has_any:
        return ["## Spatial Diagnostics", "- No significant position changes from previous moment"]
    return parts

def _format_load_distribution(metrics: Dict[str, Any], prev_metrics: Optional[Dict[str, Any]] = None) -> Optional[List[str]]:
    parts = ["## Load & Motor Utilization"]
    finger_states = metrics.get("finger_joint_states") or []
    slider_force = _f(metrics.get("slider_motor_force"))
    slider_motor_spd = _f(metrics.get("slider_motor_speed"))
    slider_max_force = _f(metrics.get("slider_max_motor_force"))
    entries = []
    for i, st in enumerate(finger_states):
        torque = _f(st.get("motor_torque"))
        max_torque = _f(st.get("max_motor_torque"))
        angle = _f(st.get("angle_deg"))
        if torque is not None and max_torque is not None and max_torque > 0:
            pct_util = abs(torque) / max_torque * 100.0
            tier = "CRITICAL" if pct_util > 80.0 else ("ELEVATED" if pct_util > 50.0 else "NOMINAL")
            entries.append({
                "label": f"Finger #{i+1}",
                "pct": pct_util,
                "value": abs(torque),
                "limit": max_torque,
                "unit": "N·m",
                "tier": tier,
                "angle": angle,
            })
    if slider_force is not None and slider_max_force is not None and slider_max_force > 0:
        pct_util = abs(slider_force) / slider_max_force * 100.0
        tier = "CRITICAL" if pct_util > 80.0 else ("ELEVATED" if pct_util > 50.0 else "NOMINAL")
        entries.append({
            "label": "Slider",
            "pct": pct_util,
            "value": abs(slider_force),
            "limit": slider_max_force,
            "unit": "N",
            "tier": tier,
            "angle": None,
        })
    if not entries:
        parts.append("- No motor load data")
        return parts
    critical = [e for e in entries if e["tier"] == "CRITICAL"]
    elevated = [e for e in entries if e["tier"] == "ELEVATED"]
    nominal = [e for e in entries if e["tier"] == "NOMINAL"]
    summary_parts = []
    if critical:
        summary_parts.append(f"{len(critical)} CRITICAL (>80%)")
    if elevated:
        summary_parts.append(f"{len(elevated)} ELEVATED (50-80%)")
    if nominal:
        summary_parts.append(f"{len(nominal)} nominal (≤50%)")
    parts.append(f"- **Tiers**: {', '.join(summary_parts)}")
    non_nominal = critical + elevated
    if non_nominal:
        for e in non_nominal:
            mark = "⚠️" if e["tier"] == "CRITICAL" else "⚠"
            line = f"  {mark} {e['label']}: {e['pct']:.1f}% of {e['limit']:.1f}{e['unit']} ({e['value']:.2f}{e['unit']}) [{e['tier']}]"
            if e.get("angle") is not None:
                line += f" | {e['angle']:.1f}°"
            parts.append(line)
    else:
        max_pct = max(e["pct"] for e in entries)
        parts.append(f"  All motors nominal (≤50%), max {max_pct:.1f}%")
    max_ni = _f(metrics.get("contact_max_normal_impulse"))
    if max_ni is not None and max_ni > 0:
        from pace_bench.core.simulator import TIME_STEP
        normal_force = max_ni / TIME_STEP if TIME_STEP > 0 else 0.0
        parts.append(f"- **Contact**: peak normal ≈ {normal_force:.1f}N (impulse {max_ni:.3f}N·s)")
    return parts

def _format_energy_flow(metrics: Dict[str, Any]) -> List[str]:
    parts = ["## Energy & Velocity"]
    obj_vel = metrics.get("object_velocity") or {}
    peak_speed = _f(metrics.get("peak_object_speed"))
    vx = _f(obj_vel.get("vx")) if obj_vel else None
    vy = _f(obj_vel.get("vy")) if obj_vel else None
    speed = _f(obj_vel.get("speed")) if obj_vel else None
    angular_vel = _f(obj_vel.get("angular_vel")) if obj_vel else None
    has_any = False
    if vx is not None and vy is not None:
        parts.append(f"- **Velocity**: ({vx:.3f}, {vy:.3f}) m/s | |v|={speed:.3f} m/s" if speed is not None else f"- **Velocity**: ({vx:.3f}, {vy:.3f}) m/s")
        has_any = True
    if angular_vel is not None and abs(angular_vel) > 0.1:
        parts.append(f"- **Angular velocity**: {angular_vel:.3f} rad/s")
        has_any = True
    if peak_speed is not None and peak_speed > 50.0:
        parts.append(f"- ⚠️ **Peak speed**: {peak_speed:.1f} m/s — possible solver instability")
        has_any = True
    if not has_any:
        parts.append("- No velocity data")
    return parts

def _format_constraints(metrics: Dict[str, Any]) -> List[str]:
    parts = ["## Constraint Satisfaction Profile"]
    profile = metrics.get("constraint_profile") or []
    if not profile:
        profile = _build_constraint_profile_from_metrics(metrics)
    failed = [c for c in profile if c.get("status") == "FAIL"]
    near = [c for c in profile if c.get("status") == "NEAR"]
    passed = [c for c in profile if c.get("status") == "PASS"]
    parts.append(
        f"- **{len(failed)} FAILED, {len(near)} NEAR, {len(passed)} PASSED**"
    )
    for c in failed + near:
        status = c.get("status", "?")
        name = c.get("name", "?")
        value = c.get("value")
        limit = c.get("limit")
        margin_val = c.get("margin")
        unit = c.get("unit", "")
        pct_val = c.get("pct")
        mark = "❌" if status == "FAIL" else "⚠️"
        line = f"  {mark} {name}: {status}"
        if margin_val is not None and _is_finite(margin_val):
            line += f" | margin {_f(margin_val):+.2f}{unit}"
        if pct_val is not None and _is_finite(pct_val):
            line += f" | {_f(pct_val):.1f}%"
        if _is_finite(value) and _is_finite(limit):
            if unit:
                line += f" | {_f(value):.3f}{unit}/{_f(limit):.3f}{unit}"
        elif _is_finite(value):
            line += f" | value={_f(value):.3f}{unit}"
        if limit and not _is_finite(limit):
            line += f" | limit {limit}"
        parts.append(line)
    if passed:
        passed_names = ", ".join(c.get("name", "?") for c in passed)
        parts.append(f"  ✅ {passed_names}: all PASSED")
    return parts

def _build_constraint_profile_from_metrics(metrics: Dict[str, Any]) -> List[Dict[str, Any]]:
    profile = []
    sm = _f(metrics.get("structure_mass"))
    mm = _f(metrics.get("max_structure_mass"))
    if sm is not None and mm is not None:
        pct_v = (sm / mm * 100.0) if mm > 0 else 0.0
        profile.append({
            "name": "Mass budget", "status": "FAIL" if sm > mm else ("NEAR" if pct_v > 80.0 else "PASS"),
            "value": sm, "limit": mm, "margin": mm - sm, "unit": "kg", "pct": pct_v,
        })
    obj_y = _f(metrics.get("object_y"))
    target_y = _f(metrics.get("target_object_y"))
    if obj_y is not None and target_y is not None:
        m = obj_y - target_y
        profile.append({
            "name": "Target height", "status": "PASS" if m >= 0 else ("NEAR" if m >= -0.5 else "FAIL"),
            "value": obj_y, "limit": target_y, "margin": m, "unit": "m",
        })
    min_seen = _f(metrics.get("min_object_y_seen"))
    min_height = _f(metrics.get("min_object_height"))
    if metrics.get("object_fell") and min_seen is not None and min_height is not None:
        profile.append({
            "name": "Min height maintained", "status": "FAIL",
            "value": metrics.get("min_object_y_seen"),
            "limit": metrics.get("min_object_height"), "margin": None, "unit": "m",
        })
    elif min_seen is not None and min_height is not None:
        profile.append({
            "name": "Min height maintained", "status": "PASS",
            "value": metrics.get("min_object_y_seen"),
            "limit": metrics.get("min_object_height"), "margin": None, "unit": "m",
        })
    grasped = metrics.get("object_grasped", False)
    profile.append({
        "name": "Grasp", "status": "PASS" if grasped else "FAIL",
        "value": bool(grasped),
        "limit": "evaluator grasp proxy true", "margin": None, "unit": "",
    })
    held = metrics.get("steps_with_object_above_target", 0)
    req = _f(metrics.get("min_simulation_steps_required"))
    if req is not None and req > 0:
        pct_v = (held / req * 100.0) if isinstance(held, (int, float)) else 0.0
        profile.append({
            "name": "Sustain duration",
            "status": "PASS" if held >= req else ("NEAR" if pct_v >= 50.0 else "FAIL"),
            "value": held, "limit": req, "margin": held - req, "unit": "steps", "pct": pct_v,
        })
    fr = metrics.get("failure_reason")
    if fr:
        profile.append({
            "name": "Evaluator failure", "status": "FAIL",
            "value": None, "limit": None, "margin": None,
            "unit": "", "detail": str(fr),
        })
    return profile

def format_task_metrics(metrics: Dict[str, Any]) -> List[str]:
    if not metrics:
        return ["**No metrics available**"]
    prev = None
    first_moment = True
    parts: List[str] = []
    score = _f(metrics.get("score"))
    success = metrics.get("success", False)
    failed = metrics.get("failed", False)
    failure_reason = metrics.get("failure_reason", "")
    step_count = metrics.get("step_count", 0)
    status = "SUCCESS ✓" if success else ("FAILED ❌" if failed else "IN PROGRESS")
    score_text = f"{score:.1f}" if score is not None else "n/a"
    header = f"## Summary: {status} | score={score_text} | step={step_count}"
    parts.append(header)
    if failure_reason and str(failure_reason).strip():
        parts.append(f"**Failure**: {failure_reason}")
    health = _format_numerical_health(metrics)
    if health:
        parts.append(health)
    elif first_moment:
        parts.append("## Numerical Health\n- No anomalies detected in the available metrics ✓")
    if first_moment:
        parts.extend(_format_environment_context(metrics))
    events = _format_event_timeline(metrics, prev)
    if events is not None:
        parts.extend(events)
    spatial = _format_spatial_diagnostics(metrics, prev)
    if spatial is not None:
        parts.extend(spatial)
    loads = _format_load_distribution(metrics, prev)
    if loads is not None:
        parts.extend(loads)
    parts.extend(_format_energy_flow(metrics))
    parts.extend(_format_constraints(metrics))
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
