from __future__ import annotations

import math

from typing import Any, Dict, List, Optional

def _fin(x: Any) -> bool:
    if x is None:
        return False
    try:
        return math.isfinite(float(x))
    except (TypeError, ValueError):
        return False

def _fv(x: Any, default: float = 0.0) -> float:
    try:
        f = float(x)
        return f if math.isfinite(f) else default
    except (TypeError, ValueError):
        return default

def _stress_tier(pct: float) -> str:
    if pct > 80.0:
        return "CRITICAL"
    if pct > 50.0:
        return "ELEVATED"
    return "NOMINAL"

def _constraint_status(value: float, lo: Optional[float],
                       hi: Optional[float]) -> str:
    if lo is not None and value < lo:
        return "FAIL"
    if hi is not None and value > hi:
        return "FAIL"
    if lo is not None and hi is not None and hi > lo:
        rng = hi - lo
        if rng > 1e-9:
            if (value - lo) / rng < 0.20 or (hi - value) / rng < 0.20:
                return "NEAR-LIMIT"
    if lo is not None and hi is None:
        if value < lo * 1.20:
            return "NEAR-LIMIT"
    if hi is not None and lo is None:
        if value > hi * 0.80:
            return "NEAR-LIMIT"
    return "PASS"

def _is_joint_limit_finite(metrics: Dict[str, Any]) -> bool:
    jfl = _fv(metrics.get("max_joint_force_limit"))
    jtl = _fv(metrics.get("max_joint_torque_limit"))
    return (jfl < 1e19 and jfl > 0) or (jtl < 1e19 and jtl > 0)

def _section_outcome(metrics: Dict[str, Any]) -> List[str]:
    lines: List[str] = []
    lines.append("### Outcome Summary")
    failed = metrics.get("failed", False)
    success = metrics.get("success", False)
    fr = metrics.get("failure_reason", "")
    if success:
        lines.append("**Status**: ✅ SUCCESS")
    elif failed:
        lines.append("**Status**: ❌ FAILED")
        if fr:
            lines.append(f"  Failure: {fr}")
    else:
        lines.append("**Status**: ⏳ IN PROGRESS")
    prog = metrics.get("progress")
    if _fin(prog):
        lines.append(f"  Altitude progress: {_fv(prog):.1f}%")
    ji = metrics.get("joints_initial", 0)
    jr = metrics.get("joints_remaining", ji)
    jb = metrics.get("joints_broken", 0)
    if ji > 0:
        lines.append(f"  Joints: {jr}/{ji} intact ({jb} broken)")
    sc = metrics.get("step_count")
    msr = metrics.get("min_simulation_steps_required")
    if _fin(sc) and _fin(msr) and int(msr) > 0:
        pct = min(100.0, float(sc) / float(msr) * 100.0)
        lines.append(f"  Duration: {int(sc)}/{int(msr)} steps ({pct:.1f}%)")
    lines.append("")
    return lines

def _section_spatial(metrics: Dict[str, Any]) -> List[str]:
    lines: List[str] = []
    lines.append("### 1. Spatial Diagnostics")
    cy = metrics.get("climber_y")
    cx = metrics.get("climber_x")
    target = metrics.get("target_y")
    init_y = float(metrics["initial_y"]) if _fin(metrics.get("initial_y")) else None
    if _fin(cy):
        y = float(cy)
        spawn_text = f" (initial y = {init_y:.1f} m)" if init_y is not None else ""
        lines.append(f"**Altitude**: y = {y:.2f} m{spawn_text}")
        if _fin(target) and init_y is not None:
            ty = float(target)
            diff = y - ty
            pct = max(0.0, min(100.0, (y - init_y) / max(ty - init_y, 0.01) * 100.0))
            status = "REACHED" if y >= ty else f"{pct:.1f}%"
            lines.append(f"  - To target ({ty:.1f} m): {diff:+.2f} m ({status})")
        mh = metrics.get("max_height_reached")
        mn = metrics.get("min_height_seen")
        if _fin(mh) or _fin(mn):
            pk = f"{_fv(mh):.2f}" if _fin(mh) else "—"
            lo = f"{_fv(mn):.2f}" if _fin(mn) else "—"
            lines.append(f"  - Peak: {pk} m  |  Min: {lo} m")
    if _fin(cx) and _fin(metrics.get("wall_contact_x_lo")) and _fin(metrics.get("wall_contact_x_hi")):
        x = float(cx)
        wlo = float(metrics["wall_contact_x_lo"])
        whi = float(metrics["wall_contact_x_hi"])
        m_lo = x - wlo
        m_hi = whi - x
        in_band = wlo <= x <= whi
        icon = "✅" if in_band else "❌"
        lines.append(f"**Wall Contact (x)**: {icon} x = {x:.2f} m [{wlo:.1f}, {whi:.1f}] "
                     f"(margins: {m_lo:+.2f} / {m_hi:+.2f})")
    gap_y = metrics.get("pad_suction_gap_y")
    if _fin(gap_y):
        lines.append(f"**Adhesion observation**: an active pad failed to hold near y={float(gap_y):.2f} m")
    lines.append("")
    return lines

def _section_failures(metrics: Dict[str, Any]) -> List[str]:
    lines: List[str] = []
    jfe: List[dict] = metrics.get("joint_failure_events") or []
    jb = metrics.get("joints_broken", 0)
    if jfe:
        lines.append("### 2. Joint Failures")
        force_limit = _fv(metrics.get("max_joint_force_limit"))
        torque_limit = _fv(metrics.get("max_joint_torque_limit"))
        lines.append(f"**{len(jfe)} joint(s) destroyed** "
                     f"(force limit: {force_limit:.2f} N, "
                     f"torque limit: {torque_limit:.2f} N·m):")
        for i, ev in enumerate(jfe):
            step = int(_fv(ev.get("step", 0)))
            fn = _fv(ev.get("force_N", 0.0))
            fp = _fv(ev.get("force_pct", 0.0))
            tn = _fv(ev.get("torque_Nm", 0.0))
            tp = _fv(ev.get("torque_pct", 0.0))
            tm = _fv(ev.get("time", 0.0))
            exceeded = []
            if fp > 100.0:
                exceeded.append(f"force {fp:.1f}%")
            if tp > 100.0:
                exceeded.append(f"torque {tp:.1f}%")
            reason = "; ".join(exceeded) if exceeded else "unknown"
            lines.append(
                f"  {i + 1}. Step {step} (t={tm:.2f}s): "
                f"force {fn:.2f} N ({fp:.1f}%), "
                f"torque {tn:.2f} N·m ({tp:.1f}%) — {reason}"
            )
        lines.append("")
        return lines
    if isinstance(jb, (int, float)) and jb > 0:
        ji = metrics.get("joints_initial", 0)
        lines.append("### 2. Joint Failures")
        lines.append(f"**{int(jb)}/{ji} joints broken** "
                     f"(no per-event data recorded).")
        lines.append("")
        return lines
    ft = metrics.get("failure_type", "")
    if ft:
        lines.append("### 2. Failure Context")
        fs = metrics.get("failure_step")
        if _fin(fs):
            lines.append(f"**Failure**: type=\"{ft}\" at step {int(fs)}")
        else:
            lines.append(f"**Failure**: type=\"{ft}\"")
        if ft == "fell":
            mh = _fv(metrics.get("min_height_seen"))
            if mh > 0:
                lines.append(f"  Min altitude reached: {mh:.2f} m")
        elif ft == "lost_contact":
            cx = _fv(metrics.get("climber_x"))
            cy = _fv(metrics.get("climber_y"))
            lines.append(f"  Position at loss: x={cx:.2f}, y={cy:.2f} m")
        elif ft in ("mass_above", "mass_below"):
            sm = _fv(metrics.get("structure_mass"))
            msm = _fv(metrics.get("max_structure_mass"))
            lsm = _fv(metrics.get("min_structure_mass"))
            lines.append(f"  Mass at failure: {sm:.2f} kg "
                         f"(budget: [{lsm:.1f}, {msm:.1f}] kg)")
        lines.append("")
    return lines

def _section_load_stress(metrics: Dict[str, Any]) -> List[str]:
    lines: List[str] = []
    jss: List[dict] = metrics.get("joint_stress_summary") or []
    if not jss:
        pf = _fv(metrics.get("peak_joint_force_pct"))
        pt = _fv(metrics.get("peak_joint_torque_pct"))
        jfl = _fv(metrics.get("max_joint_force_limit"))
        jtl = _fv(metrics.get("max_joint_torque_limit"))
        if pf > 0 or pt > 0:
            lines.append("### 3. Load & Stress")
            if pf > 0 and jfl < 1e19:
                lines.append(f"  Peak force: {pf:.1f}% of {jfl:.2f} N ({_stress_tier(pf)})")
            if pt > 0 and jtl < 1e19:
                lines.append(f"  Peak torque: {pt:.1f}% of {jtl:.2f} N·m ({_stress_tier(pt)})")
            lines.append("")
        return lines
    jfl = _fv(metrics.get("max_joint_force_limit"))
    jtl = _fv(metrics.get("max_joint_torque_limit"))
    has_force_limit = jfl < 1e19 and jfl > 0
    has_torque_limit = jtl < 1e19 and jtl > 0
    def _key(j: dict) -> float:
        return max(_fv(j.get("force_pct", 0.0)), _fv(j.get("torque_pct", 0.0)))
    ranked = sorted(jss, key=_key, reverse=True)
    max_stress = _key(ranked[0]) if ranked else 0.0
    lines.append("### 3. Load & Stress")
    if max_stress <= 50.0:
        lines.append(f"**All {len(ranked)} joint(s) NOMINAL** (< 50% of limits).")
        if has_force_limit:
            lines.append(f"  (force limit: {jfl:.2f} N, torque limit: {jtl:.2f} N·m)")
        lines.append("")
        return lines
    significant = [j for j in ranked if _key(j) > 50.0]
    nominal = len(ranked) - len(significant)
    lines.append(f"**{len(ranked)} joint(s) tracked** "
                 f"(force limit: {jfl:.2f} N, torque limit: {jtl:.2f} N·m):")
    for j in significant:
        ji = j.get("joint_index", "?")
        fn = _fv(j.get("force_N", 0.0))
        fp = _fv(j.get("force_pct", 0.0))
        tn = _fv(j.get("torque_Nm", 0.0))
        tp = _fv(j.get("torque_pct", 0.0))
        tier = _stress_tier(max(fp, tp))
        parts = [f"  Joint #{ji}"]
        if has_force_limit:
            margin = 100.0 - fp
            parts.append(f"force {fn:.2f} N / {jfl:.2f} N = {fp:.1f}% ({margin:+.1f}% margin)")
        elif fn > 0:
            parts.append(f"force {fn:.2f} N (unlimited)")
        if has_torque_limit:
            margin = 100.0 - tp
            parts.append(f"torque {tn:.2f} N·m / {jtl:.2f} N·m = {tp:.1f}% ({margin:+.1f}% margin)")
        elif tn > 0:
            parts.append(f"torque {tn:.2f} N·m (unlimited)")
        parts.append(f"[{tier}]")
        lines.append("    ".join(parts))
    if nominal > 0:
        lines.append(f"  ({nominal} other joint(s) NOMINAL)")
    if len(ranked) >= 2:
        worst = max(_key(j) for j in ranked)
        best = min(_key(j) for j in ranked)
        if worst > 20.0 and best < worst * 0.3:
            lines.append(f"  ⚠️  Stress concentration: highest {worst:.1f}% vs "
                         f"lowest {best:.1f}% ({worst / max(best, 0.01):.1f}× ratio)")
    lines.append("")
    return lines

def _section_energy(metrics: Dict[str, Any]) -> List[str]:
    lines: List[str] = []
    ke_cur = metrics.get("physics_total_ke")
    ke_peak = metrics.get("peak_total_ke")
    has_current = _fin(ke_cur)
    peak_ke_val = _fv(ke_peak) if _fin(ke_peak) else 0.0
    has_meaningful_peak = peak_ke_val > 0.001
    if not has_current and not has_meaningful_peak:
        return lines
    lines.append("### 4. Energy & Power")
    if has_current:
        ke = _fv(ke_cur)
        lines.append(f"**Current kinetic energy**: {ke:.3f} J")
    if has_meaningful_peak:
        pk_parts = []
        if peak_ke_val > 0.001:
            pk_parts.append(f"Peak KE: {peak_ke_val:.3f} J")
        if pk_parts:
            lines.append(f"**Peaks**: {'  |  '.join(pk_parts)}")
    lines.append("")
    return lines

def _section_constraints(metrics: Dict[str, Any]) -> List[str]:
    lines: List[str] = []
    lines.append("### 5. Constraints")
    constraints_evaluated: List[tuple] = []
    build_items: List[str] = []
    runtime_items: List[str] = []
    sm = metrics.get("structure_mass")
    max_m = metrics.get("max_structure_mass")
    min_m = metrics.get("min_structure_mass")
    if _fin(sm):
        mass = _fv(sm)
        lo = _fv(min_m) if _fin(min_m) and _fv(min_m) > 0 else None
        hi = _fv(max_m) if _fin(max_m) and _fv(max_m) < 1e19 else None
        lo_v = lo if (lo and lo > 0) else None
        status = _constraint_status(mass, lo_v, hi)
        constraints_evaluated.append(("Mass", status))
        if status != "PASS":
            icon = "⚠️" if status == "NEAR-LIMIT" else "❌"
            parts = [f"  {icon} Mass: {mass:.2f} kg"]
            if lo_v is not None:
                m_lo = mass - lo
                pct_lo = (m_lo / lo * 100.0) if lo > 0.001 else 0.0
                parts.append(f"[min: {lo:.1f} kg, margin: {m_lo:+.2f} kg ({pct_lo:+.1f}%)]")
            if hi is not None:
                m_hi = hi - mass
                pct_hi = (m_hi / hi * 100.0) if hi > 0.001 else 0.0
                parts.append(f"[max: {hi:.1f} kg, margin: {m_hi:+.2f} kg ({pct_hi:+.1f}%)]")
            parts.append(f"[{status}]")
            build_items.append("".join(parts))
    ft = metrics.get("failure_type", "")
    if ft == "build_zone":
        constraints_evaluated.append(("Build Zone", "FAIL"))
        bzx_min = _fv(metrics.get("build_zone_x_min", 0.0))
        bzx_max = _fv(metrics.get("build_zone_x_max", 5.0))
        bzy_min = _fv(metrics.get("build_zone_y_min", 0.0))
        bzy_max = _fv(metrics.get("build_zone_y_max", 25.0))
        build_items.append(f"  ❌ Build zone: x=[{bzx_min:.1f}, {bzx_max:.1f}], "
                           f"y=[{bzy_min:.1f}, {bzy_max:.1f}] [FAIL]")
    wlo = _fv(metrics.get("wall_contact_x_lo")) if _fin(metrics.get("wall_contact_x_lo")) else None
    whi = _fv(metrics.get("wall_contact_x_hi")) if _fin(metrics.get("wall_contact_x_hi")) else None
    ft_val = _fv(metrics.get("fell_height_threshold")) if _fin(metrics.get("fell_height_threshold")) else None
    target = metrics.get("target_y")
    if _fin(metrics.get("climber_x")) and wlo is not None and whi is not None:
        cxv = _fv(metrics.get("climber_x"))
        wc_status = _constraint_status(cxv, wlo, whi)
        constraints_evaluated.append(("Wall Contact", wc_status))
        if wc_status != "PASS":
            icon = "⚠️" if wc_status == "NEAR-LIMIT" else "❌"
            runtime_items.append(
                f"  {icon} Wall contact x: {cxv:.2f} [{wlo:.1f}, {whi:.1f}] "
                f"(margins: {cxv - wlo:+.2f} / {whi - cxv:+.2f}) [{wc_status}]"
            )
    if _fin(metrics.get("climber_y")) and ft_val is not None:
        cyv = _fv(metrics.get("climber_y"))
        fell_status = _constraint_status(cyv, ft_val, None)
        constraints_evaluated.append(("Fell Threshold", fell_status))
        if fell_status != "PASS":
            runtime_items.append(
                f"  ❌ Fell threshold: y={cyv:.2f} m "
                f"(limit: y ≥ {ft_val:.2f} m, margin: {cyv - ft_val:+.2f} m) [{fell_status}]"
            )
    if _fin(target) and _fin(metrics.get("climber_y")):
        ty = float(target)
        cyv = _fv(metrics.get("climber_y"))
        init_y = _fv(metrics.get("initial_y")) if _fin(metrics.get("initial_y")) else cyv
        pct = max(0.0, min(100.0, (cyv - init_y) / max(ty - init_y, 0.01) * 100.0))
        achieved = cyv >= ty
        constraints_evaluated.append(("Target Height", "PASS" if achieved else "IN PROGRESS"))
        if not achieved:
            runtime_items.append(f"  ⏳ Target height: {cyv:.2f} / {ty:.1f} m ({pct:.1f}%)")
    if _is_joint_limit_finite(metrics):
        pf = _fv(metrics.get("peak_joint_force_pct"))
        pt = _fv(metrics.get("peak_joint_torque_pct"))
        jfl = _fv(metrics.get("max_joint_force_limit"))
        jtl = _fv(metrics.get("max_joint_torque_limit"))
        if jfl < 1e19 and jfl > 0:
            js_status = _constraint_status(pf, None, 100.0)
            constraints_evaluated.append(("Joint Force", js_status))
            if js_status != "PASS":
                icon = "⚠️" if js_status == "NEAR-LIMIT" else "❌"
                runtime_items.append(
                    f"  {icon} Joint force: {pf:.1f}% of {jfl:.2f} N "
                    f"(margin: {100.0 - pf:.2f}%) [{js_status}]"
                )
        if jtl < 1e19 and jtl > 0:
            js_status = _constraint_status(pt, None, 100.0)
            constraints_evaluated.append(("Joint Torque", js_status))
            if js_status != "PASS":
                icon = "⚠️" if js_status == "NEAR-LIMIT" else "❌"
                runtime_items.append(
                    f"  {icon} Joint torque: {pt:.1f}% of {jtl:.2f} N·m "
                    f"(margin: {100.0 - pt:.2f}%) [{js_status}]"
                )
    sc = metrics.get("step_count")
    msr = metrics.get("min_simulation_steps_required")
    if _fin(sc) and _fin(msr) and int(msr) > 0:
        sv = int(sc)
        rv = int(msr)
        dur_ok = sv >= rv
        constraints_evaluated.append(("Duration", "PASS" if dur_ok else "NOT MET"))
        if not dur_ok:
            runtime_items.append(
                f"  ⏳ Duration: {sv}/{rv} steps ({sv / rv * 100:.1f}%) [NOT MET]"
            )
    if build_items:
        lines.append("**Build-Time**:")
        lines.extend(build_items)
    if runtime_items:
        lines.append("**Runtime**:")
        lines.extend(runtime_items)
    pass_count = sum(1 for _, s in constraints_evaluated if s == "PASS")
    near_count = sum(1 for _, s in constraints_evaluated if s == "NEAR-LIMIT")
    fail_count = sum(1 for _, s in constraints_evaluated if s in ("FAIL", "NOT MET"))
    in_progress = sum(1 for _, s in constraints_evaluated if s == "IN PROGRESS")
    total = len(constraints_evaluated)
    if total > 0:
        tally_parts = []
        if pass_count > 0:
            tally_parts.append(f"{pass_count}PASS")
        if near_count > 0:
            tally_parts.append(f"{near_count}NEAR-LIMIT")
        if fail_count > 0:
            tally_parts.append(f"{fail_count}FAIL")
        if in_progress > 0:
            tally_parts.append(f"{in_progress}IN PROGRESS")
        lines.append(f"**Tally**: {'  /  '.join(tally_parts)} (of {total})")
    lines.append("")
    return lines

def _section_numerical_health(metrics: Dict[str, Any]) -> List[str]:
    lines: List[str] = []
    flags: List[str] = []
    if metrics.get("nan_flag"):
        flags.append("NaN detected in climber position")
    if metrics.get("inf_flag"):
        flags.append("Inf detected in climber position")
    if metrics.get("extreme_speed_flag"):
        flags.append("Extreme body velocity (>100 m/s or >100 rad/s)")
    for key in ("climber_x", "climber_y", "structure_mass"):
        if key in metrics and metrics[key] is not None and not _fin(metrics[key]):
            flags.append(f"Non-finite or non-numeric metric: {key}={metrics[key]}")
    obs_errors = metrics.get("observation_error_count", 0)
    if isinstance(obs_errors, (int, float)) and obs_errors > 0:
        flags.append(
            f"Observation API errors: {int(obs_errors)}; "
            f"last={metrics.get('last_observation_error') or 'details unavailable'}"
        )
    if flags:
        lines.append("### 6. Numerical Health")
        for f in flags:
            lines.append(f"  ⚠️  {f}")
        lines.append("")
    return lines

def format_task_metrics(metrics: Dict[str, Any]) -> List[str]:
    if not metrics:
        return ["(No metrics available for this run.)"]
    parts: List[str] = []
    parts.extend(_section_outcome(metrics))
    parts.extend(_section_spatial(metrics))
    parts.extend(_section_failures(metrics))
    parts.extend(_section_load_stress(metrics))
    parts.extend(_section_energy(metrics))
    parts.extend(_section_constraints(metrics))
    parts.extend(_section_numerical_health(metrics))
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
        return ["- Code execution failed. Review the reported exception and traceback."]
    return []
