from typing import Dict, Any, List, Optional

import math

def _is_finite(x: Any) -> bool:
    if x is None:
        return False
    try:
        f = float(x)
        return math.isfinite(f)
    except (TypeError, ValueError):
        return False

def _fmt_val(v: Any, decimals: int = 2) -> str:
    if not _is_finite(v):
        return "—"
    return f"{float(v):.{decimals}f}"

def _fmt_delta(old: Any, new: Any, decimals: int = 2) -> str:
    if not _is_finite(old) or not _is_finite(new):
        return f"{_fmt_val(old, decimals)} → {_fmt_val(new, decimals)}"
    d = float(new) - float(old)
    return f"{float(old):.{decimals}f} → {float(new):.{decimals}f} ({d:+.{decimals}f})"

def _format_events_full(metrics: Dict[str, Any]) -> List[str]:
    parts: List[str] = []
    parts.append("### Events\n")
    step_count = metrics.get("step_count")
    target_y = metrics.get("target_object_y")
    max_obj_y = metrics.get("max_object_y_reached")
    step_at_max_y = metrics.get("step_at_max_object_y")
    step_first_cross = metrics.get("step_at_first_above_target")
    step_last_cross = metrics.get("step_at_last_above_target")
    broken = metrics.get("structure_broken", False)
    joint_count = metrics.get("joint_count")
    initial_jc = metrics.get("initial_joint_count")
    max_lifter_y = metrics.get("max_lifter_y")
    max_lifter_y_step = metrics.get("max_lifter_y_at_step")
    vel_y_at_peak = metrics.get("vel_y_at_max_height")
    vel_y_first = metrics.get("vel_y_on_first_cross")
    obj_vel_y = metrics.get("object_velocity_y")
    obj_y = metrics.get("object_y")
    failed = metrics.get("failed", False)
    failure_reason = metrics.get("failure_reason", "")
    succeeded = metrics.get("success", False)
    joint_failure_events = metrics.get("joint_failure_events") or []
    if _is_finite(step_count):
        time_step = metrics.get("time_step")
        if _is_finite(time_step):
            sim_s = float(step_count) * float(time_step)
            parts.append(f"Step {int(step_count)} ({sim_s:.1f}s simulated)\n")
        else:
            parts.append(f"Step {int(step_count)}\n")
    events: List[str] = []
    lifting_thresh = metrics.get("lifting_threshold_m")
    initial_y = metrics.get("initial_object_y")
    if _is_finite(initial_y) and _is_finite(lifting_thresh):
        events.append(
            f"Initial y={float(initial_y):.2f} m "
            f"(lift threshold: y > {float(initial_y) + float(lifting_thresh):.2f} m)"
        )
    if step_first_cross is not None and _is_finite(step_first_cross):
        sc = int(step_first_cross)
        detail = f"Step {sc}: first reached target (y ≥ {_fmt_val(target_y)} m)"
        if vel_y_first is not None and _is_finite(vel_y_first):
            vyf = float(vel_y_first)
            direction = "rising" if vyf >= 0 else "falling"
            detail += f", vy={vyf:.1f} m/s ({direction})"
        events.append(detail)
    if _is_finite(max_lifter_y):
        step_str = f" (step {int(max_lifter_y_step)})" if max_lifter_y_step is not None else ""
        events.append(f"Lifter peak y={float(max_lifter_y):.2f} m{step_str}")
    if _is_finite(max_obj_y):
        step_str = f" (step {int(step_at_max_y)})" if step_at_max_y is not None else ""
        detail = f"Object peak y={float(max_obj_y):.2f} m{step_str}"
        if vel_y_at_peak is not None and _is_finite(vel_y_at_peak):
            vy_peak = float(vel_y_at_peak)
            direction = "rising" if vy_peak > 0.01 else ("near-apex" if abs(vy_peak) <= 0.01 else "descending")
            detail += f", vy={vy_peak:.1f} m/s ({direction})"
        events.append(detail)
    if broken and joint_failure_events:
        for i, jf in enumerate(joint_failure_events):
            jtype = jf.get("joint_type", "unknown")
            force = jf.get("force", 0.0)
            limit = jf.get("limit", float("inf"))
            pct_str = f" ({force / limit * 100.0:.0f}% of {limit:.0f} N)" if limit < float('inf') else ""
            events.append(f"Joint failure #{i + 1}: {jtype} — force {force:.0f} N{pct_str}")
    if broken and initial_jc is not None and joint_count is not None:
        lost = int(initial_jc) - int(joint_count)
        if lost > 0:
            events.append(f"Structure lost: {lost}/{int(initial_jc)} joints broken, {int(joint_count)} remain")
    if step_last_cross is not None and _is_finite(step_last_cross):
        detail = f"Step {int(step_last_cross)}: last above target"
        if step_count is not None and _is_finite(step_count):
            steps_since = int(step_count) - int(step_last_cross)
            if steps_since > 0:
                detail += f" ({steps_since} steps before end)"
        events.append(detail)
    if _is_finite(obj_y) and _is_finite(target_y):
        final_margin = float(obj_y) - float(target_y)
        vel_str = f", vy={float(obj_vel_y):.1f} m/s" if _is_finite(obj_vel_y) else ""
        events.append(f"Final: y={float(obj_y):.2f} m, margin {final_margin:+.2f} m vs target{vel_str}")
    if events:
        for i, ev in enumerate(events, 1):
            parts.append(f"  {i}. {ev}")
    else:
        parts.append("  (No events reconstructed)")
    return parts

def _format_spatial_full(metrics: Dict[str, Any]) -> List[str]:
    parts: List[str] = []
    parts.append("\n### Ceiling / Obstacle\n")
    ceiling_gap = metrics.get("ceiling_gap")
    max_body_width = metrics.get("max_body_width")
    max_lifter_y = metrics.get("max_lifter_y")
    if ceiling_gap:
        c_y = ceiling_gap.get("y")
        c_x_min = ceiling_gap.get("x_min")
        c_x_max = ceiling_gap.get("x_max")
        if all(v is not None for v in [c_y, c_x_min, c_x_max]):
            gap_width = float(c_x_max) - float(c_x_min)
            parts.append(f"Gap at y={float(c_y):.2f} m, x=[{float(c_x_min):.2f}, {float(c_x_max):.2f}], width={gap_width:.2f} m")
            if _is_finite(max_body_width):
                parts.append(f"  Widest created body: {float(max_body_width):.2f} m (gap width {gap_width:.2f} m)")
            if _is_finite(max_lifter_y):
                parts.append(f"  Observed lifter peak: y={float(max_lifter_y):.2f} m")
        else:
            parts.append("Ceiling configured but parameters incomplete.")
    else:
        parts.append("No ceiling obstacle.")
    obj_plat_h_off = metrics.get("obj_platform_h_offset")
    obj_plat_v_off = metrics.get("obj_platform_v_offset")
    parts.append("\n### Object-Platform Retention\n")
    items = []
    if obj_plat_h_off is not None and _is_finite(obj_plat_h_off):
        items.append(f"H-offset: {float(obj_plat_h_off):+.2f} m")
    if obj_plat_v_off is not None and _is_finite(obj_plat_v_off):
        items.append(f"V-offset: {float(obj_plat_v_off):+.2f} m")
    if items:
        parts.append(" | ".join(items))
    else:
        parts.append("Offset data unavailable.")
    obj_x = metrics.get("object_x")
    lx = metrics.get("lifter_x")
    if _is_finite(obj_x) and _is_finite(lx):
        h_sep = float(obj_x) - float(lx)
        parts.append(f"\n### Horizontal\n")
        parts.append(f"Object x={float(obj_x):.2f} m | Lifter x={float(lx):.2f} m | Separation: {h_sep:+.2f} m")
    return parts

def _format_loads_full(metrics: Dict[str, Any]) -> List[str]:
    parts: List[str] = []
    parts.append("\n### Joint Forces\n")
    joint_force_summary = metrics.get("joint_force_summary") or []
    max_jf_limit = metrics.get("max_joint_force_limit")
    has_limit = _is_finite(max_jf_limit) and float(max_jf_limit) < float('inf')
    peak_jf = metrics.get("peak_joint_reaction_force")
    broken = metrics.get("structure_broken", False)
    joint_count = metrics.get("joint_count")
    initial_jc = metrics.get("initial_joint_count")
    joint_failure_events = metrics.get("joint_failure_events") or []
    lim_v = float(max_jf_limit) if has_limit else None
    peak_v = float(peak_jf) if _is_finite(peak_jf) else None
    if not joint_force_summary and not joint_failure_events:
        parts.append("(No joint force data)\n")
        return parts
    n_joints = len(joint_force_summary)
    if has_limit and peak_v is not None:
        pct = peak_v / lim_v * 100.0
        tier = "CRITICAL" if pct >= 100 else ("ELEVATED" if pct >= 80 else "NOMINAL")
        parts.append(f"{n_joints} joint(s), limit {lim_v:.0f} N | peak {peak_v:.1f} N ({pct:.1f}%) — {tier}")
    elif peak_v is not None:
        parts.append(f"{n_joints} joint(s), no force limit | peak {peak_v:.1f} N")
    else:
        parts.append(f"{n_joints} joint(s)")
    if joint_force_summary and n_joints <= 3:
        j_parts = []
        for i, entry in enumerate(joint_force_summary, 1):
            pf_val = entry.get("peak_force", 0.0)
            pct_val = entry.get("pct_of_limit")
            if pct_val is not None:
                if pct_val >= 100:
                    tag = "CRITICAL"
                elif pct_val >= 80:
                    tag = "ELEVATED"
                else:
                    tag = ""
                detail = f"#{i}: {pf_val:.1f} N ({pct_val:.1f}%)"
                if tag:
                    detail += f" — {tag}"
                j_parts.append(detail)
            else:
                j_parts.append(f"#{i}: {pf_val:.1f} N")
        parts.append("  " + " | ".join(j_parts))
    elif joint_force_summary:
        elevated = [e for e in joint_force_summary if e.get("pct_of_limit", 0) and e["pct_of_limit"] >= 80]
        if elevated:
            for e in elevated:
                parts.append(f"  #{joint_force_summary.index(e) + 1}: {e['peak_force']:.1f} N ({e['pct_of_limit']:.1f}%) — ELEVATED")
        else:
            parts.append("  All joints NOMINAL")
    if len(joint_force_summary) >= 2:
        forces = [e.get("peak_force", 0.0) for e in joint_force_summary]
        max_f, min_f = max(forces), min(forces)
        if max_f > 0 and min_f > 0 and max_f / min_f > 3.0:
            parts.append(f"  ⚠ Stress concentration: max/min force ratio = {max_f / min_f:.1f}×")
    if broken and initial_jc is not None and joint_count is not None:
        lost = int(initial_jc) - int(joint_count)
        parts.append(f"  Joints lost: {lost}/{int(initial_jc)}")
    return parts

def _format_constraints_full(metrics: Dict[str, Any]) -> List[str]:
    parts: List[str] = []
    parts.append("\n### Constraints\n")
    target_y = metrics.get("target_object_y")
    max_obj_y = metrics.get("max_object_y_reached")
    broken = metrics.get("structure_broken", False)
    joint_count = metrics.get("joint_count")
    initial_jc = metrics.get("initial_joint_count")
    structure_mass = metrics.get("structure_mass")
    max_mass = metrics.get("max_structure_mass")
    steps_held = metrics.get("steps_with_object_above_target")
    req_steps = metrics.get("min_simulation_steps_required")
    ceiling_gap = metrics.get("ceiling_gap")
    max_body_width = metrics.get("max_body_width")
    max_lifter_y = metrics.get("max_lifter_y")
    max_jf_limit = metrics.get("max_joint_force_limit")
    peak_jf = metrics.get("peak_joint_reaction_force")
    succeeded = metrics.get("success", False)
    failed = metrics.get("failed", False)
    rows: List[tuple] = []
    if _is_finite(max_obj_y) and _is_finite(target_y):
        margin = float(max_obj_y) - float(target_y)
        status = "PASS" if margin >= 0 else "FAIL"
        rows.append(("Height", status, f"peak {float(max_obj_y):.2f} vs {float(target_y):.2f} m ({margin:+.2f} m)"))
    if broken:
        lost = (int(initial_jc) - int(joint_count)) if (initial_jc is not None and joint_count is not None) else "?"
        rows.append(("Structure", "FAIL", f"{lost} joint(s) lost, {joint_count}/{initial_jc} remain"))
    elif initial_jc is not None and joint_count is not None:
        rows.append(("Structure", "PASS", f"{int(joint_count)}/{int(initial_jc)} joints intact"))
    if _is_finite(steps_held) and _is_finite(req_steps):
        shortfall = max(0, int(req_steps) - int(steps_held))
        status = "PASS" if shortfall == 0 else "FAIL"
        pct = int(steps_held) / max(1, int(req_steps)) * 100.0
        rows.append(("Sustain", status, f"{int(steps_held)}/{int(req_steps)} steps ({pct:.1f}%)"))
    if _is_finite(structure_mass) and _is_finite(max_mass):
        margin = float(max_mass) - float(structure_mass)
        status = "PASS" if margin >= 0 else "FAIL"
        pct = float(structure_mass) / float(max_mass) * 100.0
        rows.append(("Mass", status, f"{float(structure_mass):.2f}/{float(max_mass):.1f} kg ({pct:.1f}%)"))
    if _is_finite(max_jf_limit) and float(max_jf_limit) < float('inf') and _is_finite(peak_jf):
        lim_v = float(max_jf_limit)
        pf = float(peak_jf)
        status = "PASS" if pf <= lim_v else "FAIL"
        rows.append(("Joint force", status, f"peak {pf:.1f} N / limit {lim_v:.0f} N ({pf / lim_v * 100.0:.1f}%)"))
    fr = metrics.get("failure_reason") or ""
    if "Design constraint violated" in fr:
        rows.append(("Build zone", "FAIL", fr))
    if rows:
        for name, status, detail in rows:
            marker = "[✓ PASS]" if status == "PASS" else ("[✗ FAIL]" if status == "FAIL" else "[? UNKN]")
            parts.append(f"  {marker} {name}: {detail}")
    else:
        parts.append("  (No constraint data)")
    verdict = "SUCCESS" if succeeded else ("FAILURE" if failed else "INCONCLUSIVE")
    parts.append(f"\nVerdict: {verdict}")
    warnings: List[str] = []
    if _is_finite(peak_jf) and _is_finite(max_jf_limit) and float(max_jf_limit) < float('inf'):
        pct_jf = float(peak_jf) / float(max_jf_limit) * 100.0
        if 70 <= pct_jf < 100:
            warnings.append(f"joint force {pct_jf:.0f}% of limit")
    if _is_finite(structure_mass) and _is_finite(max_mass):
        pct_mass = float(structure_mass) / float(max_mass) * 100.0
        if 70 <= pct_mass < 100:
            warnings.append(f"mass {pct_mass:.0f}% of budget")
    if warnings:
        parts.append(f"Near-limit passes: {'; '.join(warnings)}")
    return parts

def _format_health_full(metrics: Dict[str, Any]) -> List[str]:
    parts: List[str] = []
    parts.append("\n### Numerical\n")
    issues: List[str] = []
    core_keys = [
        "object_x", "object_y", "object_velocity_x", "object_velocity_y",
        "lifter_x", "lifter_y", "max_object_y_reached", "max_lifter_y",
        "peak_object_velocity_x", "peak_object_velocity_y",
        "structure_mass", "peak_joint_reaction_force",
        "height_gained", "progress",
    ]
    for key in core_keys:
        val = metrics.get(key)
        if val is not None and not _is_finite(val):
            issues.append(f"Non-finite '{key}': {val}")
    extreme_speed = 100.0
    vx = metrics.get("object_velocity_x")
    vy = metrics.get("object_velocity_y")
    peak_vx = metrics.get("peak_object_velocity_x")
    peak_vy = metrics.get("peak_object_velocity_y")
    if _is_finite(vx) and abs(float(vx)) > extreme_speed:
        issues.append(f"Extreme vx={float(vx):.1f} m/s")
    if _is_finite(vy) and abs(float(vy)) > extreme_speed:
        issues.append(f"Extreme vy={float(vy):.1f} m/s")
    if _is_finite(peak_vx) and float(peak_vx) > extreme_speed:
        issues.append(f"Extreme peak |vx|={float(peak_vx):.1f} m/s")
    if _is_finite(peak_vy) and float(peak_vy) > extreme_speed:
        issues.append(f"Extreme peak |vy|={float(peak_vy):.1f} m/s")
    obj_x = metrics.get("object_x")
    obj_y = metrics.get("object_y")
    if _is_finite(obj_x) and abs(float(obj_x)) > 1000.0:
        issues.append(f"Extreme x={float(obj_x):.1f} m — likely numerical runaway")
    if _is_finite(obj_y) and abs(float(obj_y)) > 10000.0:
        issues.append(f"Extreme y={float(obj_y):.1f} m — likely numerical runaway")
    structure_mass = metrics.get("structure_mass")
    if _is_finite(structure_mass) and float(structure_mass) < 0:
        issues.append(f"Negative mass: {float(structure_mass):.3f} kg")
    peak_jf = metrics.get("peak_joint_reaction_force")
    if _is_finite(peak_jf) and float(peak_jf) > 1e6:
        issues.append(f"Joint force {float(peak_jf):.1e} N > 1 MN — likely numerical artifact")
    diagnostic_errors = metrics.get("diagnostic_error_count", 0)
    if isinstance(diagnostic_errors, (int, float)) and diagnostic_errors > 0:
        issues.append(
            f"Diagnostic collection errors: {int(diagnostic_errors)}; "
            f"last={metrics.get('last_diagnostic_error') or 'details unavailable'}"
        )
    if issues:
        parts.append(f"⚠ {len(issues)} warning(s):")
        for issue in issues:
            parts.append(f"  • {issue}")
    else:
        parts.append("✓ Clean")
    return parts

def _format_full_report(metrics: Dict[str, Any]) -> List[str]:
    parts: List[str] = []
    parts.extend(_format_events_full(metrics))
    parts.extend(_format_spatial_full(metrics))
    parts.extend(_format_loads_full(metrics))
    parts.extend(_format_constraints_full(metrics))
    parts.extend(_format_health_full(metrics))
    max_y = metrics.get("max_object_y_reached")
    target_y = metrics.get("target_object_y")
    broken = metrics.get("structure_broken", False)
    if _is_finite(max_y) and _is_finite(target_y):
        reached = "reached" if float(max_y) >= float(target_y) else "did not reach"
        broken_s = " (structure broken)" if broken else ""
        parts.append(
            f"\n---\n**Summary**: Object peak y={float(max_y):.2f} m "
            f"({reached} target {float(target_y):.2f} m){broken_s}."
        )
    return parts

def _format_events_delta(metrics: Dict[str, Any], prev: Dict[str, Any]) -> List[str]:
    parts: List[str] = []
    parts.append("### New Events\n")
    target_y = metrics.get("target_object_y")
    max_obj_y = metrics.get("max_object_y_reached")
    prev_max_obj_y = prev.get("max_object_y_reached")
    step_at_max_y = metrics.get("step_at_max_object_y")
    prev_step_at_max_y = prev.get("step_at_max_object_y")
    step_first_cross = metrics.get("step_at_first_above_target")
    prev_step_first_cross = prev.get("step_at_first_above_target")
    step_last_cross = metrics.get("step_at_last_above_target")
    prev_step_last_cross = prev.get("step_at_last_above_target")
    broken = metrics.get("structure_broken", False)
    prev_broken = prev.get("structure_broken", False)
    joint_count = metrics.get("joint_count")
    initial_jc = metrics.get("initial_joint_count")
    prev_joint_count = prev.get("joint_count")
    max_lifter_y = metrics.get("max_lifter_y")
    prev_max_lifter_y = prev.get("max_lifter_y")
    max_lifter_y_step = metrics.get("max_lifter_y_at_step")
    vel_y_at_peak = metrics.get("vel_y_at_max_height")
    vel_y_first = metrics.get("vel_y_on_first_cross")
    obj_vel_y = metrics.get("object_velocity_y")
    obj_y = metrics.get("object_y")
    prev_obj_y = prev.get("object_y")
    joint_failure_events = metrics.get("joint_failure_events") or []
    prev_joint_failure_events = prev.get("joint_failure_events") or []
    step_count = metrics.get("step_count")
    new_events: List[str] = []
    has_any = False
    if step_first_cross is not None and _is_finite(step_first_cross):
        if prev_step_first_cross is None or not _is_finite(prev_step_first_cross):
            sc = int(step_first_cross)
            detail = f"Step {sc}: first reached target (y ≥ {_fmt_val(target_y)} m)"
            if vel_y_first is not None and _is_finite(vel_y_first):
                vyf = float(vel_y_first)
                direction = "rising" if vyf >= 0 else "falling"
                detail += f", vy={vyf:.1f} m/s ({direction})"
            new_events.append(detail)
            has_any = True
    if _is_finite(max_lifter_y):
        if not _is_finite(prev_max_lifter_y) or float(max_lifter_y) != float(prev_max_lifter_y):
            step_str = f" (step {int(max_lifter_y_step)})" if max_lifter_y_step is not None else ""
            new_events.append(f"Lifter peak y={float(max_lifter_y):.2f} m{step_str}")
            has_any = True
    if _is_finite(max_obj_y):
        prev_max = float(prev_max_obj_y) if _is_finite(prev_max_obj_y) else None
        if prev_max is None or float(max_obj_y) != prev_max:
            step_str = f" (step {int(step_at_max_y)})" if step_at_max_y is not None else ""
            detail = f"Object peak y={float(max_obj_y):.2f} m{step_str}"
            if vel_y_at_peak is not None and _is_finite(vel_y_at_peak):
                vy_peak = float(vel_y_at_peak)
                direction = "rising" if vy_peak > 0.01 else ("near-apex" if abs(vy_peak) <= 0.01 else "descending")
                detail += f", vy={vy_peak:.1f} m/s ({direction})"
            new_events.append(detail)
            has_any = True
    if len(joint_failure_events) > len(prev_joint_failure_events):
        new_failures = joint_failure_events[len(prev_joint_failure_events):]
        for jf in new_failures:
            jtype = jf.get("joint_type", "unknown")
            force = jf.get("force", 0.0)
            limit = jf.get("limit", float("inf"))
            pct_str = f" ({force / limit * 100.0:.0f}% of {limit:.0f} N)" if limit < float('inf') else ""
            new_events.append(f"Joint failure: {jtype} — force {force:.0f} N{pct_str}")
            has_any = True
    if broken and not prev_broken:
        if initial_jc is not None and joint_count is not None:
            lost = int(initial_jc) - int(joint_count)
            new_events.append(f"Structure broke: {lost}/{int(initial_jc)} joints lost, {int(joint_count)} remain")
            has_any = True
    if step_last_cross is not None and _is_finite(step_last_cross):
        prev_last = int(prev_step_last_cross) if (prev_step_last_cross is not None and _is_finite(prev_step_last_cross)) else None
        if prev_last is None or int(step_last_cross) != prev_last:
            detail = f"Step {int(step_last_cross)}: last above target"
            if step_count is not None and _is_finite(step_count):
                steps_since = int(step_count) - int(step_last_cross)
                if steps_since > 0:
                    detail += f" ({steps_since} steps before end)"
            new_events.append(detail)
            has_any = True
    if _is_finite(obj_y) and _is_finite(target_y):
        final_margin = float(obj_y) - float(target_y)
        vel_str = f", vy={float(obj_vel_y):.1f} m/s" if _is_finite(obj_vel_y) else ""
        new_events.append(f"Final: y={float(obj_y):.2f} m, margin {final_margin:+.2f} m vs target{vel_str}")
        has_any = True
    if has_any:
        for i, ev in enumerate(new_events, 1):
            parts.append(f"  {i}. {ev}")
    else:
        parts.append("  (No new events since previous moment)")
    return parts

def _format_deltas_section(metrics: Dict[str, Any], prev: Dict[str, Any]) -> List[str]:
    parts: List[str] = []
    parts.append("\n### Changes from Previous Moment\n")
    deltas: List[str] = []
    obj_y = metrics.get("object_y")
    prev_obj_y = prev.get("object_y")
    if _is_finite(obj_y) and _is_finite(prev_obj_y) and float(obj_y) != float(prev_obj_y):
        deltas.append(f"Object y: {_fmt_delta(prev_obj_y, obj_y)} m")
    max_obj_y = metrics.get("max_object_y_reached")
    prev_max_y = prev.get("max_object_y_reached")
    if _is_finite(max_obj_y) and _is_finite(prev_max_y) and float(max_obj_y) != float(prev_max_y):
        deltas.append(f"Peak object y: {_fmt_delta(prev_max_y, max_obj_y)} m")
    obj_x = metrics.get("object_x")
    prev_obj_x = prev.get("object_x")
    if _is_finite(obj_x) and _is_finite(prev_obj_x) and abs(float(obj_x) - float(prev_obj_x)) > 0.01:
        deltas.append(f"Object x: {_fmt_delta(prev_obj_x, obj_x)} m")
    lx = metrics.get("lifter_x")
    prev_lx = prev.get("lifter_x")
    if _is_finite(obj_x) and _is_finite(lx) and _is_finite(prev_obj_x) and _is_finite(prev_lx):
        sep = float(obj_x) - float(lx)
        prev_sep = float(prev_obj_x) - float(prev_lx)
        if abs(sep - prev_sep) > 0.01:
            deltas.append(f"Horizontal separation: {_fmt_delta(prev_sep, sep)} m")
    obj_vy = metrics.get("object_velocity_y")
    prev_vy = prev.get("object_velocity_y")
    if _is_finite(obj_vy) and _is_finite(prev_vy) and abs(float(obj_vy) - float(prev_vy)) > 0.1:
        deltas.append(f"Object vy: {_fmt_delta(prev_vy, obj_vy, 1)} m/s")
    max_lifter_y = metrics.get("max_lifter_y")
    prev_max_lifter_y = prev.get("max_lifter_y")
    if _is_finite(max_lifter_y) and _is_finite(prev_max_lifter_y) and abs(float(max_lifter_y) - float(prev_max_lifter_y)) > 0.01:
        deltas.append(f"Lifter peak y: {_fmt_delta(prev_max_lifter_y, max_lifter_y)} m")
    peak_jf = metrics.get("peak_joint_reaction_force")
    prev_peak_jf = prev.get("peak_joint_reaction_force")
    if _is_finite(peak_jf) and _is_finite(prev_peak_jf) and abs(float(peak_jf) - float(prev_peak_jf)) > 1.0:
        deltas.append(f"Joint peak force: {_fmt_delta(prev_peak_jf, peak_jf, 1)} N")
    broken = metrics.get("structure_broken", False)
    prev_broken = prev.get("structure_broken", False)
    if broken and not prev_broken:
        deltas.append("Structure broke since previous moment")
    if deltas:
        for d in deltas:
            parts.append(f"  • {d}")
    else:
        parts.append("  (No significant changes)")
    return parts

def _format_constraints_delta(metrics: Dict[str, Any], prev: Dict[str, Any]) -> List[str]:
    parts: List[str] = []
    parts.append("\n### Constraints\n")
    target_y = metrics.get("target_object_y")
    max_obj_y = metrics.get("max_object_y_reached")
    prev_max_y = prev.get("max_object_y_reached")
    broken = metrics.get("structure_broken", False)
    prev_broken = prev.get("structure_broken", False)
    steps_held = metrics.get("steps_with_object_above_target")
    req_steps = metrics.get("min_simulation_steps_required")
    prev_steps_held = prev.get("steps_with_object_above_target")
    ceiling_gap = metrics.get("ceiling_gap")
    max_body_width = metrics.get("max_body_width")
    prev_max_body_width = prev.get("max_body_width")
    max_lifter_y = metrics.get("max_lifter_y")
    prev_max_lifter_y = prev.get("max_lifter_y")
    max_jf_limit = metrics.get("max_joint_force_limit")
    peak_jf = metrics.get("peak_joint_reaction_force")
    prev_peak_jf = prev.get("peak_joint_reaction_force")
    succeeded = metrics.get("success", False)
    failed = metrics.get("failed", False)
    prev_failed = prev.get("failed", False)
    prev_succeeded = prev.get("success", False)
    changes: List[str] = []
    if _is_finite(max_obj_y) and _is_finite(target_y) and _is_finite(prev_max_y):
        cur_pass = float(max_obj_y) >= float(target_y)
        prev_pass = float(prev_max_y) >= float(target_y)
        if cur_pass != prev_pass:
            margin = float(max_obj_y) - float(target_y)
            arrow = "✗→✓" if cur_pass else "✓→✗"
            changes.append(f"Height: {arrow} (peak {float(max_obj_y):.2f} vs {float(target_y):.2f} m, margin {margin:+.2f} m)")
    if broken != prev_broken:
        if broken:
            changes.append("Structure: ✓→✗ (joints broke since previous moment)")
        else:
            changes.append("Structure: ✗→✓ (restored)")
    if _is_finite(steps_held) and _is_finite(req_steps) and _is_finite(prev_steps_held):
        cur_sustain = int(steps_held)
        prev_sustain = int(prev_steps_held)
        if cur_sustain != prev_sustain:
            shortfall = max(0, int(req_steps) - cur_sustain)
            status = "PASS" if shortfall == 0 else "FAIL"
            pct = cur_sustain / max(1, int(req_steps)) * 100.0
            changes.append(f"Sustain: [{status}] {cur_sustain}/{int(req_steps)} steps ({pct:.1f}%) (was {prev_sustain})")
    if ceiling_gap and _is_finite(max_body_width) and _is_finite(prev_max_body_width) and float(max_body_width) != float(prev_max_body_width):
        changes.append(f"Max body width: {_fmt_delta(prev_max_body_width, max_body_width)} m")
    if ceiling_gap and _is_finite(max_lifter_y) and _is_finite(prev_max_lifter_y) and abs(float(max_lifter_y) - float(prev_max_lifter_y)) > 0.01:
        changes.append(f"Ceiling vert: lifter peak {_fmt_delta(prev_max_lifter_y, max_lifter_y)} m")
    if _is_finite(max_jf_limit) and float(max_jf_limit) < float('inf') and _is_finite(peak_jf) and _is_finite(prev_peak_jf):
        lim_v = float(max_jf_limit)
        cur_pass = float(peak_jf) <= lim_v
        prev_pass = float(prev_peak_jf) <= lim_v
        if cur_pass != prev_pass:
            arrow = "✗→✓" if cur_pass else "✓→✗"
            changes.append(f"Joint force: {arrow} (peak {float(peak_jf):.1f} N / limit {lim_v:.0f} N)")
    if changes:
        for c in changes:
            parts.append(f"  {c}")
    else:
        parts.append("  (No constraint status changes)")
    verdict = "SUCCESS" if succeeded else ("FAILURE" if failed else "INCONCLUSIVE")
    prev_verdict = "SUCCESS" if prev_succeeded else ("FAILURE" if prev_failed else "INCONCLUSIVE")
    if verdict != prev_verdict:
        parts.append(f"\nVerdict: {prev_verdict} → {verdict}")
    else:
        parts.append(f"\nVerdict: {verdict}")
    return parts

def _format_health_delta(metrics: Dict[str, Any], prev: Dict[str, Any]) -> List[str]:
    parts: List[str] = []
    parts.append("\n### Numerical\n")
    issues: List[str] = []
    extreme_speed = 100.0
    vx = metrics.get("object_velocity_x")
    vy = metrics.get("object_velocity_y")
    peak_vx = metrics.get("peak_object_velocity_x")
    peak_vy = metrics.get("peak_object_velocity_y")
    prev_vx = prev.get("object_velocity_x")
    prev_vy = prev.get("object_velocity_y")
    prev_peak_vx = prev.get("peak_object_velocity_x")
    prev_peak_vy = prev.get("peak_object_velocity_y")
    if _is_finite(vx) and abs(float(vx)) > extreme_speed:
        prev_was = _is_finite(prev_vx) and abs(float(prev_vx)) > extreme_speed
        if not prev_was:
            issues.append(f"Extreme vx={float(vx):.1f} m/s (NEW)")
    if _is_finite(vy) and abs(float(vy)) > extreme_speed:
        prev_was = _is_finite(prev_vy) and abs(float(prev_vy)) > extreme_speed
        if not prev_was:
            issues.append(f"Extreme vy={float(vy):.1f} m/s (NEW)")
    if _is_finite(peak_vx) and float(peak_vx) > extreme_speed:
        prev_was = _is_finite(prev_peak_vx) and float(prev_peak_vx) > extreme_speed
        if not prev_was:
            issues.append(f"Extreme peak |vx|={float(peak_vx):.1f} m/s (NEW)")
    if _is_finite(peak_vy) and float(peak_vy) > extreme_speed:
        prev_was = _is_finite(prev_peak_vy) and float(prev_peak_vy) > extreme_speed
        if not prev_was:
            issues.append(f"Extreme peak |vy|={float(peak_vy):.1f} m/s (NEW)")
    obj_x = metrics.get("object_x")
    prev_obj_x = prev.get("object_x")
    if _is_finite(obj_x) and abs(float(obj_x)) > 1000.0:
        prev_was = _is_finite(prev_obj_x) and abs(float(prev_obj_x)) > 1000.0
        if not prev_was:
            issues.append(f"Extreme x={float(obj_x):.1f} m (NEW)")
    if issues:
        parts.append(f"⚠ {len(issues)} new warning(s):")
        for issue in issues:
            parts.append(f"  • {issue}")
    else:
        parts.append("✓ Clean (no new issues)")
    return parts

def _format_delta_report(metrics: Dict[str, Any], prev: Dict[str, Any]) -> List[str]:
    parts: List[str] = []
    parts.extend(_format_events_delta(metrics, prev))
    parts.extend(_format_deltas_section(metrics, prev))
    parts.extend(_format_constraints_delta(metrics, prev))
    parts.extend(_format_health_delta(metrics, prev))
    return parts

def format_task_metrics(metrics: Dict[str, Any]) -> List[str]:
    if not metrics:
        return ["(No metrics available for diagnostic analysis)"]
    sanity_keys = (
        "object_y", "object_velocity_x", "object_velocity_y", "height_gained",
        "max_object_y_reached", "progress", "structure_mass", "lifter_x", "lifter_y",
        "object_x", "target_object_y", "max_lifter_y", "peak_joint_reaction_force",
    )
    sanity_issues = [
        k for k in sanity_keys
        if metrics.get(k) is not None and not _is_finite(metrics[k])
    ]
    parts: List[str] = []
    success = bool(metrics.get("success"))
    failed = bool(metrics.get("failed"))
    status = "SUCCESS ✓" if success else ("FAILED ✗" if failed else "IN PROGRESS")
    parts.append(f"## Outcome: {status}")
    if metrics.get("failure_reason"):
        parts.append(f"Failure: {metrics['failure_reason']}")
    if sanity_issues:
        parts.append(f"⚠ Non-finite values in: {', '.join(sanity_issues)}\n")
    try:
        parts.extend(_format_full_report(metrics))
    except Exception as e:
        parts.append(f"(Formatting error: {type(e).__name__}: {e})")
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
