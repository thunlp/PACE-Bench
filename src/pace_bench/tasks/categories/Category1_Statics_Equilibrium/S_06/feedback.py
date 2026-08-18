import math

from typing import Dict, Any, List, Optional, Tuple

def _is_finite_number(x: Any) -> bool:
    if x is None:
        return False
    try:
        f = float(x)
        return math.isfinite(f)
    except (TypeError, ValueError):
        return False

def _ff(val: Any, decimals: int = 3) -> str:
    try:
        v = float(val)
        if not math.isfinite(v):
            return str(val)
        return f"{v:.{decimals}f}"
    except (TypeError, ValueError):
        return str(val)

def _status_icon(ok: bool) -> str:
    return "PASS" if ok else "FAIL"

def _pct(part: float, whole: float) -> str:
    if whole <= 0:
        return "--"
    return f"{100.0 * part / whole:.1f}%"

def _format_event_timeline(metrics: Dict[str, Any]) -> List[str]:
    parts: List[str] = []
    parts.append("## 1. Temporal\n")
    step_count = metrics.get("step_count")
    failure_type = metrics.get("failure_type")
    step_at_failure = metrics.get("step_at_failure")
    time_at_failure = metrics.get("time_at_failure")
    time_step = metrics.get("time_step")
    if _is_finite_number(step_count) and _is_finite_number(time_step):
        t_end = float(step_count) * float(time_step)
        parts.append(f"Ended: step {int(step_count)}  (t = {t_end:.2f}s)")
    elif _is_finite_number(step_count):
        parts.append(f"Ended: step {int(step_count)}")
    else:
        parts.append("Ended: step unknown")
    if _is_finite_number(step_at_failure) and _is_finite_number(time_at_failure):
        ft = failure_type if failure_type else "unspecified"
        parts.append(f"Failed: step {int(step_at_failure)} (t={float(time_at_failure):.2f}s) — {ft}")
    events = metrics.get("failure_event_sequence") or []
    if not isinstance(events, list):
        events = []
    if events:
        non_start_events = [e for e in events if e.get("event") != "simulation_start"]
        if not non_start_events:
            start_ev = events[0]
            bc = start_ev.get("block_count", "?")
            mass = start_ev.get("total_mass", "?")
            if _is_finite_number(mass):
                parts.append(f"Events: simulation_start — {bc} block(s), total mass {float(mass):.2f}")
            else:
                parts.append(f"Events: simulation_start — {bc} block(s)")
        else:
            parts.append("Events:")
            for i, ev in enumerate(events, 1):
                ev_type = ev.get("event", "unknown")
                ev_step = ev.get("step", "?")
                ev_time = ev.get("time", "?")
                t_str = f"t={float(ev_time):.2f}s" if _is_finite_number(ev_time) else f"t={ev_time}"
                line = f"  {i}. step {ev_step} ({t_str}) — {ev_type}"
                if ev_type == "first_movement":
                    mv = ev.get("max_velocity", 0)
                    if _is_finite_number(mv):
                        line += f"  vel={float(mv):.3f}m/s"
                elif ev_type == "com_crossed_edge":
                    margin = ev.get("com_x_margin", 0)
                    if _is_finite_number(margin):
                        line += f"  margin={float(margin):+.3f}m"
                elif ev_type == "first_block_fell":
                    my = ev.get("min_y", 0)
                    if _is_finite_number(my):
                        line += f"  min_y={float(my):.2f}m"
                elif ev_type == "peak_kinetic_energy":
                    ke = ev.get("ke_value", 0)
                    if _is_finite_number(ke):
                        line += f"  KE={float(ke):.2e}J"
                parts.append(line)
    else:
        parts.append("Events: none recorded")
    if metrics.get("ke_spike_detected", False):
        parts.append("KE spike: >=10x step-to-step jump detected")
    return parts

def _format_spatial_diagnostics(metrics: Dict[str, Any]) -> List[str]:
    parts: List[str] = []
    parts.append("## 2. Spatial\n")
    target_overhang = metrics.get("target_overhang")
    table_edge_x = metrics.get("table_edge_x", 0.0)
    max_x = metrics.get("max_x_position")
    edge = float(table_edge_x) if _is_finite_number(table_edge_x) else 0.0
    if _is_finite_number(max_x) and _is_finite_number(target_overhang):
        mx = float(max_x)
        to = float(target_overhang)
        reach = mx - edge
        shortfall = max(0.0, to - mx)
        if shortfall > 0:
            parts.append(f"Overhang: {mx:.3f}m (past edge: {reach:+.3f}m) — shortfall {shortfall:.3f}m ({_pct(shortfall, to)}) vs target {to:.3f}m")
        else:
            parts.append(f"Overhang: {mx:.3f}m (past edge: {reach:+.3f}m) — meets target {to:.3f}m by {-shortfall:.3f}m")
    elif _is_finite_number(max_x):
        parts.append(f"Overhang: {mx:.3f}m (past edge: {mx - edge:+.3f}m) — target unknown")
    else:
        parts.append("Overhang: no data")
    com_x = metrics.get("center_of_mass_x")
    com_y = metrics.get("center_of_mass_y")
    com_margin = metrics.get("com_to_edge_margin")
    if _is_finite_number(com_x):
        cx = float(com_x)
        margin_to_edge = cx - edge
        if margin_to_edge > 0.001:
            parts.append(f"CoM: x={cx:.3f}m ({margin_to_edge:+.3f}m past edge — unstable)")
        elif margin_to_edge > -0.2:
            parts.append(f"CoM: x={cx:.3f}m ({margin_to_edge:+.3f}m to edge — marginal)")
        else:
            parts.append(f"CoM: x={cx:.3f}m ({abs(margin_to_edge):.3f}m behind edge)")
    elif _is_finite_number(com_margin):
        cm = float(com_margin)
        direction = "past" if cm > 0 else "behind"
        parts.append(f"CoM: {cm:+.3f}m ({direction} edge)")
    else:
        parts.append("CoM: no data")
    if _is_finite_number(com_y):
        parts.append(f"CoM y: {float(com_y):.3f}m")
    min_y_v = metrics.get("min_y_position")
    max_y_v = metrics.get("max_y_position")
    ceiling_y = metrics.get("ceiling_y_limit")
    if _is_finite_number(min_y_v):
        parts.append(f"Y range: [{_ff(min_y_v)} .. {_ff(max_y_v) if _is_finite_number(max_y_v) else '?'}]m")
    per_block = metrics.get("per_block_extents") or []
    if isinstance(per_block, list) and per_block and len(per_block) <= 6:
        block_lines = []
        for i, blk in enumerate(per_block):
            xmx = blk.get("x_max")
            mass = blk.get("mass")
            past_str = ""
            if _is_finite_number(xmx) and _is_finite_number(edge):
                past = float(xmx) - edge
                past_str = f"  past={past:+.2f}m" if past > 0.01 else f"  short={abs(past):.2f}m"
            m_str = _ff(mass, 2) if _is_finite_number(mass) else "?"
            xmn = _ff(blk.get("x_min"), 2) if _is_finite_number(blk.get("x_min")) else "?"
            xmx_s = _ff(xmx, 2) if _is_finite_number(xmx) else "?"
            block_lines.append(f"  b{i}: x=[{xmn},{xmx_s}] m={m_str}{past_str}")
        parts.append(f"Blocks ({len(per_block)}):")
        parts.extend(block_lines)
    elif isinstance(per_block, list) and per_block:
        max_past = max(
            (float(b.get("x_max", edge)) - edge) for b in per_block
            if _is_finite_number(b.get("x_max"))
        )
        parts.append(f"Blocks ({len(per_block)}): max past edge = {max_past:+.2f}m")
    y_levels = metrics.get("y_levels") or []
    if isinstance(y_levels, (list, tuple)) and y_levels:
        n_layers = len(y_levels)
        if n_layers == 1:
            parts.append(f"Stack: 1 layer (no vertical stacking)")
        else:
            sorted_levels = sorted(y_levels)
            gaps = [sorted_levels[j] - sorted_levels[j - 1] for j in range(1, len(sorted_levels))]
            avg_gap = sum(gaps) / len(gaps)
            parts.append(f"Stack: {n_layers} layers, avg gap {avg_gap:.3f}m")
    return parts

def _format_load_distribution(metrics: Dict[str, Any]) -> List[str]:
    parts: List[str] = []
    parts.append("## 3. Load & Surface Motion\n")
    structure_mass = metrics.get("structure_mass")
    block_count = metrics.get("block_count")
    if not _is_finite_number(structure_mass):
        parts.append("Structure mass unavailable.")
        return parts
    mass = float(structure_mass)
    bc = int(block_count) if _is_finite_number(block_count) else 0
    parts.append(f"Structure mass={mass:.2f}; blocks={bc}")
    table_velocity = metrics.get("table_velocity")
    if isinstance(table_velocity, (list, tuple)) and len(table_velocity) >= 2:
        if _is_finite_number(table_velocity[0]) and _is_finite_number(table_velocity[1]):
            parts.append(
                f"Observed table velocity=({float(table_velocity[0]):.3f}, "
                f"{float(table_velocity[1]):.3f}) m/s"
            )
    return parts

def _format_energy_flow(metrics: Dict[str, Any]) -> List[str]:
    parts: List[str] = []
    parts.append("## 4. Energy\n")
    ke_cur = metrics.get("total_kinetic_energy")
    ke_peak = metrics.get("peak_kinetic_energy")
    max_vel = metrics.get("max_velocity")
    has_ke = False
    if _is_finite_number(ke_cur):
        parts.append(f"KE final: {float(ke_cur):.3e}J")
        has_ke = True
    if _is_finite_number(ke_peak):
        parts.append(f"KE peak:  {float(ke_peak):.3e}J")
        has_ke = True
    if _is_finite_number(max_vel):
        parts.append(f"Max vel:  {float(max_vel):.3f}m/s")
        has_ke = True
    if not has_ke:
        parts.append("no data")
    return parts

def _format_constraint_profile(metrics: Dict[str, Any]) -> List[str]:
    parts: List[str] = []
    parts.append("## 5. Constraints\n")
    constraints: List[Tuple[str, str, str, str, float]] = []
    max_x = metrics.get("max_x_position")
    target_overhang = metrics.get("target_overhang")
    overhang_ok = metrics.get("overhang_ok", False)
    if _is_finite_number(max_x) and _is_finite_number(target_overhang):
        mx = float(max_x)
        to = float(target_overhang)
        margin = mx - to
        util = max(0.0, 100.0 * (to - mx) / to) if to > 0 else 0.0
        constraints.append((
            "Overhang", _status_icon(overhang_ok),
            f"x_max={mx:.3f}m", f"{margin:+.3f}m vs target {to:.3f}m",
            max(0.0, util)
        ))
    stable_dur = metrics.get("stable_duration")
    target_stab = metrics.get("target_stability_time")
    stability_ok = metrics.get("stability_ok", False)
    if _is_finite_number(stable_dur) and _is_finite_number(target_stab):
        sd = float(stable_dur)
        ts = float(target_stab)
        margin = sd - ts
        util = 100.0 * max(0.0, ts - sd) / ts if ts > 0 else 0.0
        constraints.append((
            "Stability", _status_icon(stability_ok),
            f"{sd:.2f}s", f"{margin:+.2f}s vs {ts:.1f}s",
            util
        ))
    mass = metrics.get("structure_mass")
    max_mass = metrics.get("max_total_mass_limit")
    if _is_finite_number(mass) and _is_finite_number(max_mass):
        m = float(mass)
        mm = float(max_mass)
        ok = m <= mm + 0.01
        margin = mm - m
        util = 100.0 * m / mm if mm > 0 else 0.0
        constraints.append((
            "Mass", _status_icon(ok),
            f"{m:.2f}", f"{margin:+.2f} headroom ({_pct(m, mm)} used)",
            util
        ))
    bc = metrics.get("block_count")
    max_bc = metrics.get("max_block_count_limit")
    if bc is not None and _is_finite_number(max_bc):
        bci = int(bc) if isinstance(bc, (int, float)) else 0
        mbci = int(max_bc)
        ok = bci <= mbci
        util = 100.0 * bci / mbci if mbci > 0 else 0.0
        constraints.append((
            "Block count", _status_icon(ok),
            f"{bci}", f"{mbci - bci} headroom (limit {mbci})",
            util
        ))
    max_y_v = metrics.get("max_y_position")
    ceiling_y = metrics.get("ceiling_y_limit")
    if _is_finite_number(max_y_v) and _is_finite_number(ceiling_y):
        my = float(max_y_v)
        cy = float(ceiling_y)
        ok = my <= cy + 0.01
        clearance = cy - my
        util = 100.0 * (my / cy) if cy > 0 else 0.0
        constraints.append((
            "Ceiling", _status_icon(ok),
            f"max_y={my:.3f}m", f"{clearance:+.3f}m clearance (ceiling {cy:.2f}m)",
            util
        ))
    spawn_zone = metrics.get("spawn_zone")
    design_violations_raw = metrics.get("design_constraint_violations")
    design_violations = design_violations_raw or []
    spawn_fails = [d for d in design_violations if d.get("constraint") == "spawn_zone"]
    if spawn_fails:
        for sf in spawn_fails:
            px = sf.get("position_x")
            smx = sf.get("spawn_max")
            if _is_finite_number(px) and _is_finite_number(smx):
                margin_val = float(smx) - float(px)
                constraints.append((
                    "Spawn zone", "FAIL",
                    f"x={float(px):.3f}m", f"{margin_val:+.3f}m vs max {float(smx):.2f}m",
                    100.0
                ))
    elif (
        design_violations_raw is not None
        and isinstance(spawn_zone, (list, tuple))
        and len(spawn_zone) >= 2
    ):
        sz_str = f"[{_ff(spawn_zone[0], 2)}, {_ff(spawn_zone[1], 2)}]"
        constraints.append((
            "Spawn zone", "PASS", f"zone={sz_str}", "all blocks inside", 0.0
        ))
    dim_fails = [d for d in design_violations if d.get("constraint") in ("block_width", "block_height")]
    if dim_fails:
        for df in dim_fails:
            cname = df.get("constraint", "block_dim")
            val = df.get("value", "?")
            lim = df.get("limit", "?")
            constraints.append((
                cname, "FAIL",
                f"{_ff(val) if _is_finite_number(val) else val}",
                f"limit={_ff(lim) if _is_finite_number(lim) else lim}",
                100.0
            ))
    elif design_violations_raw is not None:
        max_length = metrics.get("max_block_length_limit")
        max_height = metrics.get("max_block_height_limit")
        if _is_finite_number(max_length) and _is_finite_number(max_height):
            dim_value = f"w≤{float(max_length):.2f}m h≤{float(max_height):.2f}m"
        else:
            dim_value = "within evaluator limits"
        constraints.append((
            "Block dims", "PASS", dim_value, "all blocks ok", 0.0
        ))
    failed = [c for c in constraints if "FAIL" in c[1]]
    near_limit = [c for c in constraints if "PASS" in c[1] and c[4] > 50.0]
    show = failed + near_limit
    if len(constraints) <= 3:
        show = constraints
    if not show:
        parts.append("All constraints PASS with wide margins.")
        return parts
    parts.append("| Constraint | Status | Value | Margin |")
    parts.append("|:-----------|:------:|:------|:-------|")
    for name, status, val_str, margin_str, _util in show:
        icon = "PASS" if "PASS" in status else "FAIL"
        flag = " **<--**" if "FAIL" in status else ""
        parts.append(f"| {name} | {icon} | {val_str} | {margin_str}{flag} |")
    has_design_fail = any("FAIL" in c[1] and c[0] in ("Spawn zone", "Block dims", "Block count") for c in failed)
    has_runtime_fail = any("FAIL" in c[1] and c[0] not in ("Spawn zone", "Block dims", "Block count") for c in failed)
    if has_design_fail:
        parts.append("\nDesign-time failure: simulation may not have executed.")
    if has_runtime_fail:
        parts.append("Runtime failure: see Section 1 for event timeline.")
    return parts

def _format_numerical_health(metrics: Dict[str, Any]) -> List[str]:
    parts: List[str] = []
    parts.append("## 6. Numerical Health\n")
    issues: List[str] = []
    numeric_keys = [
        "max_x_position", "target_overhang", "stable_duration", "target_stability_time",
        "min_y_position", "max_y_position", "structure_mass", "block_count",
        "total_kinetic_energy", "max_velocity", "peak_kinetic_energy",
        "center_of_mass_x", "center_of_mass_y", "com_to_edge_margin",
        "step_at_failure", "time_at_failure",
        "step_count", "ceiling_y_limit", "table_edge_x",
    ]
    for key in numeric_keys:
        val = metrics.get(key)
        if val is not None and not _is_finite_number(val):
            issues.append(f"Non-finite: {key}={val}")
    max_vel = metrics.get("max_velocity")
    if _is_finite_number(max_vel):
        mv = float(max_vel)
        if mv > 100.0:
            issues.append(f"Extreme velocity: {mv:.1f} m/s")
    ke_peak = metrics.get("peak_kinetic_energy")
    if _is_finite_number(ke_peak):
        kp = float(ke_peak)
        if kp > 1e6:
            issues.append(f"Extreme KE: {kp:.3e} J")
    per_block = metrics.get("per_block_extents") or []
    if isinstance(per_block, list):
        for i, blk in enumerate(per_block):
            for fkey in ("x_min", "x_max", "y_min", "y_max", "mass"):
                v = blk.get(fkey)
                if v is not None and not _is_finite_number(v):
                    issues.append(f"Block {i} non-finite {fkey}={v}")
    if issues:
        parts.append(f"{len(issues)} issue(s):")
        for iss in issues:
            parts.append(f"  - {iss}")
    else:
        parts.append("OK")
    return parts

def format_task_metrics(metrics: Dict[str, Any]) -> List[str]:
    if not metrics:
        return ["**No metrics available**"]
    parts: List[str] = []
    success = metrics.get("success", False)
    failed = metrics.get("failed", False)
    failure_reason = metrics.get("failure_reason")
    score = metrics.get("score")
    parts.append("# S-06 Diagnostic Report\n")
    if success:
        parts.append("**Outcome**: SUCCESS")
    elif failed:
        reason_str = str(failure_reason) if failure_reason else "unspecified"
        parts.append(f"**Outcome**: FAILED — {reason_str}")
    else:
        parts.append("**Outcome**: INCOMPLETE")
    if _is_finite_number(score):
        parts.append(f"**Score**: {float(score):.1f}/100")
    parts.append("")
    try:
        parts.extend(_format_event_timeline(metrics))
    except Exception as exc:
        parts.append(f"## 1. Temporal\n\n(formatting error: {type(exc).__name__}: {exc})")
    parts.append("")
    try:
        parts.extend(_format_spatial_diagnostics(metrics))
    except Exception as exc:
        parts.append(f"## 2. Spatial\n\n(formatting error: {type(exc).__name__}: {exc})")
    parts.append("")
    try:
        parts.extend(_format_load_distribution(metrics))
    except Exception as exc:
        parts.append(f"## 3. Load & Surface Motion\n\n(formatting error: {type(exc).__name__}: {exc})")
    parts.append("")
    try:
        parts.extend(_format_energy_flow(metrics))
    except Exception as exc:
        parts.append(f"## 4. Energy\n\n(formatting error: {type(exc).__name__}: {exc})")
    parts.append("")
    try:
        parts.extend(_format_constraint_profile(metrics))
    except Exception as exc:
        parts.append(f"## 5. Constraints\n\n(formatting error: {type(exc).__name__}: {exc})")
    parts.append("")
    try:
        parts.extend(_format_numerical_health(metrics))
    except Exception as exc:
        parts.append(f"## 6. Numerical Health\n\n(formatting error: {type(exc).__name__}: {exc})")
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
