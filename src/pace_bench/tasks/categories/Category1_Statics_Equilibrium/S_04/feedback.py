from typing import Dict, Any, List

import math

def _is_finite_number(x: Any) -> bool:
    if x is None:
        return False
    try:
        f = float(x)
        return math.isfinite(f)
    except (TypeError, ValueError):
        return False

def _fmt(x: Any, dp: int = 2) -> str:
    if not _is_finite_number(x):
        return str(x)
    try:
        v = float(x)
        return f"{v:+{dp+3}.{dp}f}"
    except (TypeError, ValueError):
        return str(x)

def _fmt_abs(x: Any, dp: int = 2) -> str:
    if not _is_finite_number(x):
        return str(x)
    try:
        return f"{float(x):.{dp}f}"
    except (TypeError, ValueError):
        return str(x)

def _section_temporal_chronology(metrics: Dict[str, Any],
                                 prev_metrics: Dict[str, Any] = None) -> List[str]:
    parts: List[str] = []
    parts.append("## 1. Events")
    events = metrics.get("event_timeline")
    if not events or not isinstance(events, list) or len(events) == 0:
        parts.append("No events recorded.")
        return parts
    sorted_events = sorted(events, key=lambda e: int(e.get("step", 0)) if isinstance(e, dict) else 0)
    if prev_metrics is not None:
        prev_events = prev_metrics.get("event_timeline", [])
        prev_event_ids = set()
        for pe in (prev_events or []):
            if isinstance(pe, dict):
                prev_event_ids.add((int(pe.get("step", -1)), str(pe.get("type", ""))))
        new_events = [e for e in sorted_events
                      if isinstance(e, dict)
                      and (int(e.get("step", -1)), str(e.get("type", ""))) not in prev_event_ids]
    else:
        new_events = sorted_events
    if not new_events:
        parts.append("No new events since previous moment.")
        return parts
    step_count = metrics.get("step_count", 0)
    parts.append(f"{len(new_events)} event(s) (total {len(sorted_events)} across {step_count} steps):")
    for evt in new_events:
        if not isinstance(evt, dict):
            continue
        step = evt.get("step", "?")
        etype = evt.get("type", "unknown")
        if etype == "pivot_destroyed":
            net_t = evt.get("net_torque", "?")
            tq_lim = evt.get("torque_limit", "?")
            line = f"  Step {step}: PIVOT JOINT DESTROYED"
            if _is_finite_number(net_t) and _is_finite_number(tq_lim):
                line += f" — net torque {_fmt(net_t)} N·m exceeds limit {_fmt_abs(tq_lim)} N·m"
            ratio = evt.get("ratio", "?")
            if _is_finite_number(ratio):
                line += f" ({float(ratio) * 100:.1f}% of limit)"
            parts.append(line)
        elif etype == "load_attached":
            lp = evt.get("load_pos")
            if isinstance(lp, (list, tuple)) and len(lp) >= 2:
                parts.append(f"  Step {step}: Load auto-attached at ({_fmt_abs(lp[0])}, {_fmt_abs(lp[1])}) m")
            else:
                parts.append(f"  Step {step}: Load auto-attached")
        elif etype == "load_caught_drop":
            cp = evt.get("catch_pos")
            if isinstance(cp, (list, tuple)) and len(cp) >= 2:
                parts.append(f"  Step {step}: Dropped load caught at ({_fmt_abs(cp[0])}, {_fmt_abs(cp[1])}) m")
            else:
                parts.append(f"  Step {step}: Dropped load caught")
        elif etype == "ground_contact":
            label = evt.get("body_label", f"body_{evt.get('body_index', '?')}")
            y_val = evt.get("y", "?")
            gl = evt.get("ground_limit", "?")
            line = f"  Step {step}: GROUND CONTACT — {label}"
            if _is_finite_number(y_val) and _is_finite_number(gl):
                line += f" at y={_fmt_abs(y_val)} m (limit y≥{_fmt_abs(gl)} m)"
            parts.append(line)
        else:
            parts.append(f"  Step {step}: {etype}")
    return parts

def _section_spatial_diagnostics(metrics: Dict[str, Any],
                                  prev_metrics: Dict[str, Any] = None) -> List[str]:
    parts: List[str] = []
    is_delta = prev_metrics is not None
    parts.append("## 2. Spatial" if not is_delta else "## 2. Spatial Δ")
    tol_deg = metrics.get("max_angle_deviation_deg")
    beam_angle = metrics.get("beam_angle_deg")
    max_angle = metrics.get("max_angle_seen_deg")
    ground_lim = metrics.get("ground_y_limit")
    min_y = metrics.get("min_body_y")
    pivot_y = 5.0
    if is_delta:
        prev_angle = prev_metrics.get("beam_angle_deg")
        if _is_finite_number(beam_angle) and _is_finite_number(prev_angle):
            da = float(beam_angle) - float(prev_angle)
            line = f"angle: {_fmt_abs(prev_angle)}° → {_fmt_abs(beam_angle)}° (Δ{da:+.2f}°)"
            if _is_finite_number(tol_deg):
                angle_margin = tol_deg - abs(float(beam_angle))
                exceeded = " EXCEEDED" if abs(float(beam_angle)) > tol_deg else ""
                line += f"  | margin {angle_margin:+.2f}° of ±{_fmt_abs(tol_deg)}°{exceeded}"
            parts.append(line)
        prev_min_y = prev_metrics.get("min_body_y")
        if _is_finite_number(min_y) and _is_finite_number(prev_min_y) and _is_finite_number(ground_lim):
            dy = float(min_y) - float(prev_min_y)
            g_margin = float(min_y) - float(ground_lim)
            parts.append(f"min y: {_fmt_abs(prev_min_y)} → {_fmt_abs(min_y)} m (Δ{dy:+.2f})  |  margin {g_margin:+.2f} m")
        prev_com_x = prev_metrics.get("structure_com_x")
        prev_com_y = prev_metrics.get("structure_com_y")
        com_x = metrics.get("structure_com_x")
        com_y = metrics.get("structure_com_y")
        if _is_finite_number(com_x) and _is_finite_number(prev_com_x):
            dx = float(com_x) - float(prev_com_x)
            dy = float(com_y) - float(prev_com_y) if _is_finite_number(com_y) and _is_finite_number(prev_com_y) else 0.0
            parts.append(f"CoM: ({_fmt_abs(prev_com_x)}, {_fmt_abs(prev_com_y)}) → ({_fmt_abs(com_x)}, {_fmt_abs(com_y)}) m  (Δ{dx:+.2f}, {dy:+.2f})")
        lp = metrics.get("load_pos")
        prev_lp = prev_metrics.get("load_pos")
        if lp != prev_lp:
            if isinstance(lp, (list, tuple)) and len(lp) >= 2:
                lx, ly = float(lp[0]), float(lp[1])
                parts.append(f"Load: ({_fmt_abs(lx)}, {_fmt_abs(ly)}) m")
            elif lp is None and prev_lp is not None:
                parts.append("Load: lost")
            elif lp is not None and prev_lp is None:
                parts.append("Load: newly available")
        return parts
    if _is_finite_number(beam_angle) and _is_finite_number(tol_deg):
        angle_margin = tol_deg - abs(float(beam_angle))
        pct_used = (abs(float(beam_angle)) / tol_deg * 100.0) if tol_deg > 0 else 100.0
        parts.append(f"Beam angle: {_fmt(beam_angle)}° / ±{_fmt_abs(tol_deg)}°  "
                     f"| margin: {angle_margin:+.2f}° ({100.0 - pct_used:.0f}% remaining)")
    elif _is_finite_number(beam_angle):
        parts.append(f"Beam angle: {_fmt(beam_angle)}°")
    if _is_finite_number(max_angle) and _is_finite_number(tol_deg):
        exceeded = " EXCEEDED" if abs(float(max_angle)) > tol_deg else ""
        parts.append(f"Peak |angle|: {_fmt_abs(max_angle)}° / ±{_fmt_abs(tol_deg)}°{exceeded}")
    elif _is_finite_number(max_angle):
        parts.append(f"Peak |angle|: {_fmt_abs(max_angle)}°")
    if _is_finite_number(min_y) and _is_finite_number(ground_lim):
        g_margin = float(min_y) - float(ground_lim)
        g_status = "GROUNDED" if float(min_y) < float(ground_lim) else "CLEAR"
        parts.append(f"Lowest y: {_fmt_abs(min_y)} m (limit {_fmt_abs(ground_lim)} m, margin {g_margin:+.2f} m, {g_status})")
    elif _is_finite_number(min_y):
        parts.append(f"Lowest y: {_fmt_abs(min_y)} m")
    com_x = metrics.get("structure_com_x")
    com_y = metrics.get("structure_com_y")
    if _is_finite_number(com_x) and _is_finite_number(com_y):
        com_distance = math.hypot(float(com_x) - 0.0, float(com_y) - pivot_y)
        com_angle = math.degrees(math.atan2(float(com_y) - pivot_y, float(com_x) - 0.0))
        parts.append(f"CoM: ({_fmt_abs(com_x)}, {_fmt_abs(com_y)}) m  |  "
                     f"{com_distance:.2f} m from pivot @ {com_angle:+.1f}°")
    lp = metrics.get("load_pos")
    if isinstance(lp, (list, tuple)) and len(lp) >= 2:
        lx, ly = float(lp[0]), float(lp[1])
        parts.append(f"Load: ({_fmt_abs(lx)}, {_fmt_abs(ly)}) m  |  offsets ({lx:+.2f}, {ly - pivot_y:+.2f}) m from pivot")
    else:
        parts.append("Load: not available")
    return parts

def _section_load_distribution(metrics: Dict[str, Any],
                               prev_metrics: Dict[str, Any] = None) -> List[str]:
    parts: List[str] = []
    is_delta = prev_metrics is not None
    parts.append("## 3. Torque Distribution" if not is_delta else "## 3. Torque Δ")
    contribs = metrics.get("torque_contributions")
    if not contribs or not isinstance(contribs, list):
        parts.append("No per-body torque breakdown available.")
        return parts
    net_t = metrics.get("net_torque_about_pivot")
    fragile = metrics.get("fragile_joints", False)
    max_jt = metrics.get("max_joint_torque")
    if is_delta:
        if _is_finite_number(net_t):
            prev_nt = prev_metrics.get("net_torque_about_pivot")
            if _is_finite_number(prev_nt):
                dn = float(net_t) - float(prev_nt)
                parts.append(f"Net: {_fmt(prev_nt)} → {_fmt(net_t)} N·m (Δ{dn:+.1f})")
            else:
                parts.append(f"Net: {_fmt(net_t)} N·m")
        top_n = min(2, len(contribs))
        for i in range(top_n):
            c = contribs[i]
            if not isinstance(c, dict):
                continue
            label = str(c.get("body_label", "?"))[:12]
            tq = _fmt(c.get("torque", 0), 1)
            pct = c.get("torque_pct", 0)
            parts.append(f"  {label}: {tq} N·m ({float(pct):.0f}%)")
        if fragile and _is_finite_number(max_jt):
            torque_limit = abs(float(max_jt))
            critical_items = []
            for c in contribs:
                if not isinstance(c, dict):
                    continue
                abs_tq = abs(float(c.get("torque", 0))) if _is_finite_number(c.get("torque")) else 0.0
                ratio = abs_tq / torque_limit if torque_limit > 0 else 0.0
                if ratio >= 0.5:
                    label = str(c.get("body_label", "?"))[:12]
                    tier = "CRITICAL" if ratio >= 0.8 else "ELEVATED"
                    critical_items.append(f"{label}={tier}({ratio*100:.0f}%)")
            if critical_items:
                parts.append(f"  Tiers: {', '.join(critical_items)}")
        return parts
    if _is_finite_number(net_t):
        parts.append(f"Net: {_fmt(net_t)} N·m  (+CCW / -CW)")
    parts.append("")
    parts.append(f"  {'Rank':<5} {'Body':<14} {'Mass':>8} {'Pos (x,y)':>16} {'Torque':>12} {'%Net':>8}")
    parts.append(f"  {'-'*5} {'-'*14} {'-'*8} {'-'*16} {'-'*12} {'-'*8}")
    for rank, c in enumerate(contribs, 1):
        if not isinstance(c, dict):
            continue
        label = str(c.get("body_label", f"body_{c.get('body_index','?')}"))[:14]
        mass = _fmt_abs(c.get("mass", 0), 2)
        px = _fmt_abs(c.get("pos_x", 0), 2)
        py = _fmt_abs(c.get("pos_y", 0), 2)
        tq = _fmt(c.get("torque", 0), 1)
        pct = c.get("torque_pct", 0)
        pct_str = f"{float(pct):.1f}%"
        parts.append(f"  {rank:<5} {label:<14} {mass:>8} ({px:>7},{py:>7}) {tq:>12} {pct_str:>8}")
    if fragile and _is_finite_number(max_jt):
        torque_limit = abs(float(max_jt))
        parts.append("")
        parts.append(f"Tiers (limit {_fmt_abs(torque_limit)} N·m):")
        for c in contribs:
            if not isinstance(c, dict):
                continue
            label = str(c.get("body_label", f"body_{c.get('body_index','?')}"))[:14]
            abs_tq = abs(float(c.get("torque", 0))) if _is_finite_number(c.get("torque")) else 0.0
            ratio = abs_tq / torque_limit if torque_limit > 0 else 0.0
            if ratio >= 0.8:
                parts.append(f"  {label}: CRITICAL ({ratio*100:.0f}%)")
            elif ratio >= 0.5:
                parts.append(f"  {label}: ELEVATED ({ratio*100:.0f}%)")
    return parts

def _section_energy_flow(metrics: Dict[str, Any]) -> List[str]:
    return ["## 4. Energy", "N/A (statics task)"]

def _section_constraint_profile(metrics: Dict[str, Any],
                                 prev_metrics: Dict[str, Any] = None) -> List[str]:
    parts: List[str] = []
    parts.append("## 5. Constraints")
    dashboard = metrics.get("constraint_dashboard")
    if not dashboard or not isinstance(dashboard, list):
        parts.append("No constraint dashboard available.")
        return parts
    bidirectional_constraints = {"Beam Angle (current)", "Pivot Torque", "Angular Velocity"}
    above_constraints = {"Balance Duration", "Ground Clearance"}
    below_constraints = {"Step Budget"}
    def _calc_utilization(name: str, value: Any, limit: Any) -> float:
        if not _is_finite_number(value) or not _is_finite_number(limit) or float(limit) == 0:
            return -1.0
        v = float(value)
        l = float(limit)
        if name in bidirectional_constraints:
            return abs(v) / abs(l) * 100.0
        elif name in below_constraints:
            return v / l * 100.0
        elif name in above_constraints:
            if v >= l:
                return 0.0
            else:
                return (l - v) / abs(l) * 100.0
        else:
            return abs(v) / abs(l) * 100.0
    pass_count = sum(1 for c in dashboard if isinstance(c, dict) and c.get("status") == "PASS")
    fail_count = sum(1 for c in dashboard if isinstance(c, dict) and c.get("status") == "FAIL")
    warn_count = sum(1 for c in dashboard if isinstance(c, dict) and c.get("status") == "WARN")
    exhausted_count = sum(1 for c in dashboard if isinstance(c, dict) and c.get("status") == "EXHAUSTED")
    show_items = []
    for c in dashboard:
        if not isinstance(c, dict):
            continue
        status = str(c.get("status", ""))
        name = str(c.get("name", ""))
        value = c.get("value")
        limit = c.get("limit")
        util = _calc_utilization(name, value, limit)
        if status in ("FAIL", "WARN", "EXHAUSTED"):
            show_items.append((name, status, value, limit, c.get("margin"), util))
        elif status == "PASS" and util >= 50.0:
            show_items.append((name, status, value, limit, c.get("margin"), util))
    if prev_metrics is not None:
        prev_dash = prev_metrics.get("constraint_dashboard", [])
        prev_statuses = {}
        for pc in (prev_dash or []):
            if isinstance(pc, dict):
                prev_statuses[str(pc.get("name", ""))] = str(pc.get("status", ""))
        changed_names = set()
        for item in show_items:
            name = item[0]
            if prev_statuses.get(name) != item[1]:
                changed_names.add(name)
        show_items = [item for item in show_items
                      if item[1] == "FAIL" or item[0] in changed_names]
    if not show_items:
        parts.append(f"All {pass_count + fail_count + warn_count + exhausted_count} constraints nominal — "
                     f"{pass_count} PASS, {fail_count} FAIL, {warn_count} WARN, {exhausted_count} EXHAUSTED")
        return parts
    parts.append(f"  {'Constraint':<28} {'Status':<10} {'Value':>12} {'Limit':>12} {'Margin':>12} {'%Util':>8}")
    parts.append(f"  {'-'*28} {'-'*10} {'-'*12} {'-'*12} {'-'*12} {'-'*8}")
    for name, status, value, limit, margin, util in show_items:
        if isinstance(value, str):
            val_str = str(value)[:12]
        elif _is_finite_number(value):
            val_str = f"{float(value):.2f}"[:12]
        else:
            val_str = str(value)[:12]
        if _is_finite_number(limit):
            lim_str = f"{float(limit):.2f}"[:12]
        else:
            lim_str = str(limit)[:12]
        if _is_finite_number(margin):
            sn = "+" if float(margin) >= 0 else ""
            mar_str = f"{sn}{float(margin):.2f}"[:12]
        else:
            mar_str = "—"
        util_str = f"{util:.0f}%" if util >= 0 else "—"
        parts.append(f"  {name:<28} {status:<10} {val_str:>12} {lim_str:>12} {mar_str:>12} {util_str:>8}")
    parts.append("")
    parts.append(f"Summary: {pass_count} PASS, {fail_count} FAIL, {warn_count} WARN, {exhausted_count} EXHAUSTED")
    return parts

def _section_numerical_health(metrics: Dict[str, Any],
                               is_first: bool = True) -> List[str]:
    parts: List[str] = []
    parts.append("## 6. Numerical")
    health = metrics.get("numerical_health")
    if health is None or not isinstance(health, list):
        parts.append("Numerical health data unavailable.")
        return parts
    if len(health) == 0:
        parts.append("HEALTHY — no NaN, Inf, or extreme values.")
        return parts
    critical_flags = [f for f in health if isinstance(f, dict) and f.get("severity") == "CRITICAL"]
    warning_flags = [f for f in health if isinstance(f, dict) and f.get("severity") == "WARNING"]
    if not critical_flags and not warning_flags:
        parts.append("HEALTHY — no NaN, Inf, or extreme values.")
        return parts
    if critical_flags:
        parts.append(f"CRITICAL ({len(critical_flags)}):")
        for f in critical_flags:
            parts.append(f"  ◆ {f.get('detail', f.get('tag', '?'))}")
    if warning_flags:
        parts.append(f"WARNINGS ({len(warning_flags)}):")
        for f in warning_flags:
            parts.append(f"  ⚠ {f.get('detail', f.get('tag', '?'))}")
    body_details = metrics.get("body_details")
    if body_details and isinstance(body_details, list):
        extreme_v = []
        for bd in body_details:
            if not isinstance(bd, dict):
                continue
            vx = bd.get("velocity_x", 0)
            vy = bd.get("velocity_y", 0)
            if _is_finite_number(vx) and abs(float(vx)) > 50.0:
                extreme_v.append((bd.get("body_label", "?"), "x", float(vx)))
            if _is_finite_number(vy) and abs(float(vy)) > 50.0:
                extreme_v.append((bd.get("body_label", "?"), "y", float(vy)))
        if extreme_v:
            parts.append("  ⚠ Extreme velocity: " +
                         ", ".join(f"{lbl}.v{ax}={v:.1f}" for lbl, ax, v in extreme_v))
    if critical_flags:
        parts.append("  >> NUMERICAL INSTABILITY — results may be unreliable.")
    return parts

def _section_environment_summary(metrics: Dict[str, Any]) -> List[str]:
    parts: List[str] = []
    parts.append("## Environment")
    parts.append("Gravity: present")
    wind = metrics.get("wind_active", False)
    if wind:
        parts.append("Wind: ACTIVE")
    else:
        parts.append("Wind: INACTIVE")
    fragile = metrics.get("fragile_joints", False)
    max_jt = metrics.get("max_joint_torque")
    if fragile and _is_finite_number(max_jt):
        parts.append(f"Pivot: FRAGILE (limit {_fmt_abs(max_jt)} N·m)")
    elif _is_finite_number(max_jt) and float(max_jt) > 0:
        parts.append(f"Pivot: NON-FRAGILE (limit {_fmt_abs(max_jt)} N·m, inert)")
    else:
        parts.append("Pivot: NON-FRAGILE")
    drop = metrics.get("drop_load_active", False)
    catch_r = metrics.get("catch_radius")
    if drop and _is_finite_number(catch_r):
        parts.append(f"Load: DROPPED (catch radius {_fmt_abs(catch_r)} m)")
    elif not drop and _is_finite_number(catch_r):
        parts.append(f"Load: STATIC (auto-attach < {_fmt_abs(catch_r)} m)")
    else:
        parts.append("Load mode/radius: unavailable")
    load_mass = metrics.get("load_mass")
    if _is_finite_number(load_mass):
        parts.append(f"Load mass: {_fmt_abs(load_mass)} kg")
    tol = metrics.get("max_angle_deviation_deg")
    bal_time = metrics.get("target_balance_time")
    if _is_finite_number(tol):
        parts.append(f"Angle tol: ±{_fmt_abs(tol)}°")
    if _is_finite_number(bal_time):
        parts.append(f"Balance duration required: {_fmt_abs(bal_time)} s")
    ground = metrics.get("ground_y_limit")
    if _is_finite_number(ground):
        parts.append(f"Ground failure: y < {_fmt_abs(ground)} m")
    obstacles = metrics.get("obstacle_world_rects", [])
    if obstacles:
        parts.append(f"Obstacles ({len(obstacles)}):")
        for i, r in enumerate(obstacles):
            if isinstance(r, dict):
                parts.append(f"  #{i}: x=[{_fmt_abs(r.get('xmin'))}, {_fmt_abs(r.get('xmax'))}], "
                             f"y=[{_fmt_abs(r.get('ymin'))}, {_fmt_abs(r.get('ymax'))}]")
    return parts

def format_task_metrics(metrics: Dict[str, Any]) -> List[str]:
    if not isinstance(metrics, dict):
        return ["**Error**: metrics is not a dictionary."]
    if not metrics:
        return ["**No task metrics available.**"]
    step_count = metrics.get("step_count")
    is_first = True
    prev_metrics = None
    parts: List[str] = []
    parts.append("## Outcome")
    success = metrics.get("success", False)
    failed = metrics.get("failed", False)
    failure_reason = metrics.get("failure_reason")
    if success:
        parts.append("Result: SUCCESS")
    elif failed:
        parts.append("Result: FAILED")
        if failure_reason:
            parts.append(f"Reason: {failure_reason}")
    else:
        parts.append("Result: INCOMPLETE")
    parts.append("")
    critical_keys = [
        "beam_angle_deg", "net_torque_about_pivot", "structure_mass",
        "balance_duration", "max_angle_seen_deg",
    ]
    nf_found = []
    for k in critical_keys:
        v = metrics.get(k)
        if v is not None and not _is_finite_number(v):
            nf_found.append((k, v))
    if nf_found:
        parts.append("## CRITICAL: NON-FINITE VALUES")
        for k, v in nf_found:
            parts.append(f"  `{k}` = {v}")
        parts.append("Simulation state invalid — diagnostics may be unreliable.")
        parts.append("")
    if is_first:
        parts.extend(_section_environment_summary(metrics))
        parts.append("")
    elif not is_first and prev_metrics:
        prev_step = prev_metrics.get("step_count", 0)
        delta_step = (step_count or 0) - (prev_step or 0)
        parts.append(f"## Δ from previous moment (+{delta_step} steps)")
        delta_lines = []
        cur_angle = metrics.get("beam_angle_deg")
        prev_angle = prev_metrics.get("beam_angle_deg")
        if _is_finite_number(cur_angle) and _is_finite_number(prev_angle):
            da = float(cur_angle) - float(prev_angle)
            delta_lines.append(f"angle: {_fmt_abs(prev_angle)}° → {_fmt_abs(cur_angle)}° (Δ{da:+.2f}°)")
        cur_torque = metrics.get("net_torque_about_pivot")
        prev_torque = prev_metrics.get("net_torque_about_pivot")
        if _is_finite_number(cur_torque) and _is_finite_number(prev_torque):
            dt = float(cur_torque) - float(prev_torque)
            delta_lines.append(f"net torque: {_fmt(prev_torque)} → {_fmt(cur_torque)} N·m")
        cur_min_y = metrics.get("min_body_y")
        prev_min_y = prev_metrics.get("min_body_y")
        if _is_finite_number(cur_min_y) and _is_finite_number(prev_min_y):
            dy = float(cur_min_y) - float(prev_min_y)
            delta_lines.append(f"min y: {_fmt_abs(prev_min_y)} → {_fmt_abs(cur_min_y)} m (Δ{dy:+.2f})")
        cur_dur = metrics.get("balance_duration")
        prev_dur = prev_metrics.get("balance_duration")
        if _is_finite_number(cur_dur) and _is_finite_number(prev_dur):
            dd = float(cur_dur) - float(prev_dur)
            delta_lines.append(f"balance duration: {_fmt_abs(prev_dur)} → {_fmt_abs(cur_dur)} s (Δ{dd:+.2f})")
        if delta_lines:
            parts.append("  " + "  |  ".join(delta_lines))
        parts.append("")
    parts.extend(_section_temporal_chronology(metrics, prev_metrics))
    parts.append("")
    parts.extend(_section_spatial_diagnostics(metrics, prev_metrics))
    parts.append("")
    parts.extend(_section_load_distribution(metrics, prev_metrics if not is_first else None))
    parts.append("")
    if is_first:
        parts.extend(_section_energy_flow(metrics))
        parts.append("")
    parts.extend(_section_constraint_profile(metrics, prev_metrics))
    parts.append("")
    parts.extend(_section_numerical_health(metrics, is_first))
    return parts

def get_improvement_suggestions(
    metrics: Dict[str, Any],
    score: float,
    success: bool,
    failed: bool,
    failure_reason: str = None,
    error: str = None,

) -> List[str]:
    suggestions = []
    if error:
        error_lower = error.lower()
        if "name 'sandbox' is not defined" in error_lower:
            suggestions.append("- Move all code that uses 'sandbox' inside the build_agent function")
        elif "invalid syntax" in error_lower:
            suggestions.append("- Check for syntax errors in generated code")
        else:
            suggestions.append("- Review error details to identify and fix the issue")
        return suggestions
    return suggestions
