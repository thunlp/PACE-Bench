import math

from typing import Dict, Any, List

def _safe_float(v: Any, default: float = 0.0) -> float:
    try:
        x = float(v)
        return x if math.isfinite(x) else default
    except (TypeError, ValueError):
        return default

def _format_temporal_chronology(metrics: Dict[str, Any]) -> List[str]:
    lines: List[str] = []
    events = metrics.get("temporal_events")
    if not isinstance(events, list) or not events:
        fallback = _build_fallback_timeline(metrics)
        if not fallback:
            return lines
        events = fallback
    lines.append("## 1. Timeline")
    for ev in events:
        if not isinstance(ev, dict):
            continue
        event_name = ev.get("event", "unknown")
        step = ev.get("step", "?")
        detail = ev.get("detail", "")
        line = f"  [{event_name}] step {step}"
        if detail:
            line += f" — {detail}"
        lines.append(line)
        for sub_key in ("contact_to_motion_delay", "suspension_depth", "gap"):
            val = ev.get(sub_key)
            if val is not None:
                try:
                    fv = float(val)
                    if math.isfinite(fv):
                        lines.append(f"    {sub_key}: {fv:.3f}")
                except (TypeError, ValueError):
                    pass
    return lines

def _build_fallback_timeline(metrics: Dict[str, Any]) -> List[Dict[str, Any]]:
    events: List[Dict[str, Any]] = []
    fc = metrics.get("first_contact_step")
    if fc is not None:
        events.append({"event": "first_contact", "step": int(fc),
                        "detail": f"Pusher made contact at step {int(fc)}"})
    fm = metrics.get("first_object_motion_step")
    if fm is not None:
        delay = int(fm) - int(fc) if fc is not None else None
        d = f"Object first moved at step {int(fm)}"
        if delay is not None and delay > 0:
            d += f" (contact-to-motion delay: {delay} steps)"
        events.append({"event": "first_object_motion", "step": int(fm), "detail": d,
                        "contact_to_motion_delay": delay})
    fs = metrics.get("first_wheel_suspension_step")
    if fs is not None:
        depth = _safe_float(metrics.get("max_wheel_suspension_depth"))
        events.append({"event": "first_wheel_suspension", "step": int(fs),
                        "detail": f"Wheels suspended at step {int(fs)} (depth {depth:.3f}m)",
                        "suspension_depth": round(depth, 3)})
    fsp = metrics.get("first_wheel_spinning_step")
    if fsp is not None:
        events.append({"event": "first_wheel_spinning", "step": int(fsp),
                        "detail": f"Wheels spinning at step {int(fsp)}"})
    return events

def _format_spatial_diagnostics(metrics: Dict[str, Any]) -> List[str]:
    lines: List[str] = []
    lines.append("## 2. State")
    ground_y = _safe_float(metrics.get("ground_y"), default=float("nan"))
    ox = metrics.get("object_x")
    oy = metrics.get("object_y")
    if ox is not None:
        ox_f = _safe_float(ox)
        tx = metrics.get("target_object_x")
        tx_str = ""
        if tx is not None:
            tx_f = _safe_float(tx)
            if math.isfinite(tx_f):
                shortfall = tx_f - ox_f
                tx_str = f" (target {tx_f:.2f}m, shortfall {shortfall:+.2f}m)"
        oy_str = ""
        if oy is not None:
            oy_f = _safe_float(oy)
            oy_str = f", y={oy_f:.2f}m"
        lines.append(f"  Object: x={ox_f:.2f}m{oy_str}{tx_str}")
        dp = _safe_float(metrics.get("distance_pushed"))
        mdp = _safe_float(metrics.get("max_distance_pushed"))
        lines.append(f"  Displacement: {dp:.2f}m net, {mdp:.2f}m best")
    px = metrics.get("pusher_x")
    py = metrics.get("pusher_y")
    if px is not None:
        px_f = _safe_float(px)
        gap_f = _safe_float(metrics.get("pusher_object_gap"), default=float("nan"))
        gap_str = ""
        if math.isfinite(gap_f):
            if gap_f > 0.05:
                gap_str = f", {gap_f:.2f}m ahead of object"
            elif gap_f < -0.05:
                gap_str = f", {abs(gap_f):.2f}m behind object"
            else:
                gap_str = ", near contact"
        py_str = ""
        if py is not None and math.isfinite(ground_y):
            py_f = _safe_float(py)
            py_str = f", y={py_f:.2f}m ({py_f - ground_y:+.2f}m above ground)"
        elif py is not None:
            py_str = f", y={_safe_float(py):.2f}m"
        lines.append(f"  Pusher: x={px_f:.2f}m{py_str}{gap_str}")
    pa = metrics.get("pusher_angle")
    pt_max = metrics.get("max_pusher_tilt")
    tilt_limit = math.pi / 6
    tilt_parts = []
    if pa is not None:
        tilt_parts.append(f"{math.degrees(_safe_float(pa)):.1f}°")
    if pt_max is not None:
        pt_f = _safe_float(pt_max)
        tilt_parts.append(f"peak {math.degrees(pt_f):.1f}° (limit 30.0°, margin {tilt_limit - pt_f:+.3f}rad)")
    if tilt_parts:
        lines.append(f"  Tilt: {', '.join(tilt_parts)}")
    wca = metrics.get("wheel_contact_audit")
    if isinstance(wca, dict):
        ever = wca.get("ever_contacted", False)
        min_gap = wca.get("min_gap_to_ground")
        radii = wca.get("radii", [])
        positions = wca.get("positions", [])
        if radii and positions:
            contacting = []
            suspended = []
            for r, pos in zip(radii, positions):
                wx, wy = float(pos[0]), float(pos[1])
                wr = float(r)
                bottom_y = wy - wr
                g = bottom_y - ground_y if math.isfinite(ground_y) else float('inf')
                status = "CONTACT" if g <= 0.01 else "SUSPENDED"
                if g <= 0.01:
                    contacting.append((wx, wy, wr, g))
                else:
                    suspended.append((wx, wy, wr, g))
            n_total = len(radii)
            if contacting and not suspended:
                lines.append(f"  Wheels: {n_total} wheel(s), all in ground contact")
            elif suspended and not contacting:
                min_g = min(g for _, _, _, g in suspended) if suspended else 0.5
                lines.append(f"  Wheels: {n_total} wheel(s), all SUSPENDED "
                            f"(min gap {min_g:+.3f}m, never contacted ground)")
            else:
                lines.append(f"  Wheels: {len(contacting)} contacting, {len(suspended)} suspended "
                            f"(of {n_total} total)")
                for wx, wy, wr, g in contacting:
                    lines.append(f"    CONTACT: pos=({wx:.2f},{wy:.2f}) r={wr:.3f}")
                for wx, wy, wr, g in suspended:
                    lines.append(f"    SUSPENDED: pos=({wx:.2f},{wy:.2f}) r={wr:.3f} gap={g:.3f}m")
        elif min_gap is not None and math.isfinite(float(min_gap)):
            min_gap_f = float(min_gap)
            contact_str = "yes" if ever else "no"
            if not ever and min_gap_f > 0.01:
                lines.append(f"  Wheels: ever contacted={contact_str}, min gap {min_gap_f:+.3f}m")
    return lines

def _format_load_distribution(metrics: Dict[str, Any]) -> List[str]:
    lines: List[str] = []
    lines.append("## 3. Mechanics")
    mass = _safe_float(metrics.get("structure_mass"))
    max_mass = _safe_float(metrics.get("max_structure_mass"), default=float("inf"))
    if max_mass != float("inf") and max_mass > 0:
        util = mass / max_mass * 100.0
        margin_m = max_mass - mass
        severity = " — CRITICAL" if util > 100 else " — elevated" if util > 70 else ""
        lines.append(f"  Mass: {mass:.2f} / {max_mass:.2f} kg "
                    f"({util:.1f}% used, margin {margin_m:+.2f} kg){severity}")
    else:
        lines.append(f"  Mass: {mass:.2f} kg")
    ma = metrics.get("motor_actuation")
    if isinstance(ma, dict):
        ever = ma.get("ever_active", False)
        if ever:
            peak_sat = _safe_float(ma.get("peak_saturation"))
            cmd = ma.get("cmd_speeds", [])
            act = ma.get("actual_speeds", [])
            tor = ma.get("torques_used", [])
            parts = ["Motors: active"]
            if peak_sat > 0:
                parts.append(f"peak saturation {peak_sat * 100:.0f}%")
            if cmd and act:
                cmd_avg = sum(cmd) / len(cmd)
                act_avg = sum(act) / len(act)
                tor_max = max(tor) if tor else 0.0
                parts.append(f"speed cmd={cmd_avg:.1f} act={act_avg:.1f} rad/s")
                parts.append(f"peak torque {tor_max:.2f} N·m")
                if abs(act_avg) < 0.01 and abs(cmd_avg) > 0.1:
                    parts.append("STALLED — commanded but zero rotation")
            lines.append(f"  {', '.join(parts)}")
        else:
            lines.append("  Motors: never active")
    mwt = _safe_float(metrics.get("max_wheel_tangential_speed"))
    if mwt > 2.0:
        lines.append(f"  Wheel slip: peak tangential speed {mwt:.1f} m/s — wheels spinning")
    return lines

def _format_energy_flow(metrics: Dict[str, Any]) -> List[str]:
    lines: List[str] = []
    lines.append("## 4. Energy")
    et = metrics.get("energy_tracking")
    if not isinstance(et, dict):
        lines.append("  No energy data.")
        return lines
    chassis_ke = _safe_float(et.get("peak_chassis_ke"))
    motor_energy = _safe_float(et.get("cumulative_motor_energy_est"))
    if chassis_ke < 0.001 and motor_energy < 0.001:
        lines.append("  No chassis kinetic energy or motor work recorded.")
        return lines
    lines.append(f"  Peak chassis KE: {chassis_ke:.2f}J")
    lines.append(f"  Motor work (est): {motor_energy:.2f}J")
    return lines

def _format_constraint_profile(metrics: Dict[str, Any]) -> List[str]:
    lines: List[str] = []
    lines.append("## 5. Constraints")
    profile = metrics.get("constraint_profile")
    if not isinstance(profile, list) or not profile:
        profile = _build_constraint_fallback(metrics)
    if not profile:
        lines.append("  No constraint data.")
        return lines
    failed = [c for c in profile if c.get("status") == "FAIL"]
    passed = [c for c in profile if c.get("status") == "PASS"]
    if failed:
        lines.append("  **Failed:**")
        for c in failed:
            name = c.get("constraint", "?")
            cur = str(c.get("current", "—"))
            lim = str(c.get("limit", "—"))
            mar = str(c.get("margin", "—"))
            lines.append(f"  ❌ {name}: {cur} vs {lim} ({mar})")
    near = []
    for c in passed:
        name = c.get("constraint", "?")
        util = c.get("utilization_pct")
        if util is not None and util > 50:
            near.append(f"{name} ({util:.0f}% utilized)")
        else:
            mar_str = str(c.get("margin", ""))
            try:
                lim_str = str(c.get("limit", "0"))
                lim_parts = lim_str.split()
                if lim_parts:
                    lim_val = float(lim_parts[0])
                    if lim_val > 0 and mar_str.startswith("+"):
                        mar_val = float(mar_str.split()[0].replace("+", ""))
                        if mar_val / lim_val < 0.15:
                            near.append(f"{name} (margin {mar_val:.2f})")
            except (ValueError, IndexError):
                pass
    all_passed = [c.get("constraint", "?") for c in passed]
    all_failed = [c.get("constraint", "?") for c in failed]
    lines.append(f"  **Result**: {len(passed)}/{len(profile)} PASS, {len(failed)}/{len(profile)} FAIL")
    if all_failed:
        lines.append(f"  Failed: {', '.join(all_failed)}")
    if near:
        lines.append(f"  Near-limit: {', '.join(near)}")
    return lines

def _build_constraint_fallback(metrics: Dict[str, Any]) -> List[Dict[str, Any]]:
    profile: List[Dict[str, Any]] = []
    mass = _safe_float(metrics.get("structure_mass"))
    max_mass = _safe_float(metrics.get("max_structure_mass"), default=float("inf"))
    if max_mass != float("inf") and max_mass > 0:
        profile.append({
            "constraint": "Structure mass",
            "status": "PASS" if mass <= max_mass else "FAIL",
            "current": f"{mass:.2f} kg",
            "limit": f"{max_mass:.2f} kg",
            "margin": f"{max_mass - mass:+.2f} kg",
            "utilization_pct": round(mass / max_mass * 100.0, 1),
            "phase": "build-time",
        })
    ox = _safe_float(metrics.get("object_x"))
    tx = _safe_float(metrics.get("target_object_x"), default=float("nan"))
    if math.isfinite(tx):
        profile.append({
            "constraint": "Target distance",
            "status": "PASS" if ox >= tx else "FAIL",
            "current": f"{ox:.2f} m",
            "limit": f"{tx:.2f} m",
            "margin": f"{ox - tx:+.2f} m",
            "phase": "runtime",
        })
    if metrics.get("step_count") is not None and metrics.get("min_simulation_steps_required") is not None:
        sc = int(metrics["step_count"])
        min_sc = int(metrics["min_simulation_steps_required"])
        profile.append({
            "constraint": "Simulation time",
            "status": "PASS" if sc >= min_sc else "FAIL",
            "current": f"{sc} steps",
            "limit": f"{min_sc} steps",
            "margin": f"{sc - min_sc:+d} steps",
            "phase": "runtime",
        })
    oy_raw = metrics.get("object_y")
    if oy_raw is not None and math.isfinite(_safe_float(oy_raw, default=float("nan"))):
        oy = float(oy_raw)
        payload_limit = 0.50
        profile.append({
            "constraint": "Payload support (y > 0.5m)",
            "status": "PASS" if oy > payload_limit else "FAIL",
            "current": f"{oy:.2f} m",
            "limit": "0.50 m",
            "margin": f"{oy - payload_limit:+.2f} m",
            "phase": "runtime",
        })
    if metrics.get("max_pusher_tilt") is not None:
        tilt = _safe_float(metrics.get("max_pusher_tilt"), default=float("nan"))
        if math.isfinite(tilt):
            tilt_limit = math.pi / 6
            profile.append({
                "constraint": "Chassis tilt (< π/6 rad)",
                "status": "PASS" if tilt < tilt_limit else "FAIL",
                "current": f"{math.degrees(tilt):.1f}°",
                "limit": "30.0°",
                "margin": f"{tilt_limit - tilt:+.3f} rad",
                "phase": "runtime",
            })
    return profile

def _format_numerical_health(metrics: Dict[str, Any]) -> List[str]:
    lines: List[str] = []
    lines.append("## 6. Numerical")
    issues: List[str] = []
    ni = metrics.get("numerical_instability")
    if ni is True:
        issues.append("❌ Numerical instability — NaN/Inf detected")
    pbv = _safe_float(metrics.get("peak_body_velocity"))
    if pbv > 100.0:
        issues.append(f"⚠️ Extreme body velocity: {pbv:.0f} m/s")
    elif pbv > 50.0:
        issues.append(f"⚠️ Elevated body velocity: {pbv:.0f} m/s")
    obj_speed = math.sqrt(
        _safe_float(metrics.get("object_velocity_x")) ** 2 +
        _safe_float(metrics.get("object_velocity_y")) ** 2
    )
    if obj_speed > 100.0:
        issues.append(f"⚠️ Extreme object velocity: {obj_speed:.0f} m/s")
    mwt = _safe_float(metrics.get("max_wheel_tangential_speed"))
    if mwt > 50.0:
        issues.append(f"⚠️ Extreme wheel tangential speed: {mwt:.0f} m/s")
    for key in ("object_x", "object_y", "pusher_x", "pusher_y", "structure_mass"):
        if key in metrics and metrics[key] is not None:
            try:
                if not math.isfinite(float(metrics[key])):
                    issues.append(f"❌ Non-finite metric: {key}={metrics[key]}")
            except (TypeError, ValueError):
                issues.append(f"❌ Non-numeric metric: {key}={metrics[key]}")
    diagnostic_errors = metrics.get("diagnostic_error_count", 0)
    if isinstance(diagnostic_errors, (int, float)) and diagnostic_errors > 0:
        issues.append(
            f"⚠️ Diagnostic collection errors: {int(diagnostic_errors)}; "
            f"last={metrics.get('last_diagnostic_error') or 'details unavailable'}"
        )
    if not issues:
        lines.append("  ✅ No numerical anomalies")
    else:
        for issue in issues:
            lines.append(f"  {issue}")
    return lines

def format_task_metrics(metrics: Dict[str, Any]) -> List[str]:
    if not metrics or not isinstance(metrics, dict):
        return ["**No metrics available**"]
    error = metrics.get("error")
    if error:
        return [f"**Execution Error**: {error}"]
    parts: List[str] = []
    success = bool(metrics.get("success"))
    failed = bool(metrics.get("failed"))
    status = "SUCCESS ✓" if success else ("FAILED ✗" if failed else "IN PROGRESS")
    parts.append(f"## Outcome: {status}")
    if metrics.get("failure_reason"):
        parts.append(f"Failure: {metrics['failure_reason']}")
    parts.append("")
    try:
        parts.extend(_format_temporal_chronology(metrics))
    except Exception as e:
        parts.append(f"*[Timeline error: {e}]*")
    parts.append("")
    try:
        parts.extend(_format_spatial_diagnostics(metrics))
    except Exception as e:
        parts.append(f"*[State error: {e}]*")
    parts.append("")
    try:
        parts.extend(_format_load_distribution(metrics))
    except Exception as e:
        parts.append(f"*[Mechanics error: {e}]*")
    parts.append("")
    try:
        parts.extend(_format_energy_flow(metrics))
    except Exception as e:
        parts.append(f"*[Energy error: {e}]*")
    parts.append("")
    try:
        parts.extend(_format_constraint_profile(metrics))
    except Exception as e:
        parts.append(f"*[Constraints error: {e}]*")
    parts.append("")
    try:
        parts.extend(_format_numerical_health(metrics))
    except Exception as e:
        parts.append(f"*[Numerical error: {e}]*")
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
        return ["- Code execution failed. Review the reported exception and traceback."]
    return []
