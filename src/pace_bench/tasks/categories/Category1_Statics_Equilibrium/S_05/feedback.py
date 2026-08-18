from typing import Dict, Any, List, Optional

import math

def _is_finite(x: Any) -> bool:
    if x is None:
        return False
    try:
        return math.isfinite(float(x))
    except (TypeError, ValueError):
        return False

def _fm3(val: Any) -> str:
    try:
        return f"{float(val):.3f}"
    except (TypeError, ValueError):
        return str(val)

def _fm2(val: Any) -> str:
    try:
        return f"{float(val):.2f}"
    except (TypeError, ValueError):
        return str(val)

def _fm1(val: Any) -> str:
    try:
        return f"{float(val):.1f}"
    except (TypeError, ValueError):
        return str(val)

def _anch_str(anchor) -> str:
    if anchor is None:
        return "unknown"
    try:
        if isinstance(anchor, (list, tuple)) and len(anchor) >= 2:
            return f"({_fm2(anchor[0])}, {_fm2(anchor[1])})"
    except (TypeError, ValueError):
        pass
    return str(anchor)

def _format_temporal_chronology(metrics: Dict[str, Any]) -> List[str]:
    parts: List[str] = ["### 1. Events & Timeline"]
    events: List[Dict[str, Any]] = []
    jfe = metrics.get("joint_failure_events") or []
    for ev in jfe:
        step = ev.get("step")
        if step is not None:
            events.append({"step": int(step), "type": "joint_breach", "detail": ev})
    cf_step = metrics.get("core_force_step")
    core_force = metrics.get("core_force")
    if cf_step is not None and core_force is not None and float(core_force) > 0.0:
        events.append({"step": int(cf_step), "type": "core_impact", "force": float(core_force)})
    if not events:
        parts.append("No failure or impact events during this run.")
        return parts
    events.sort(key=lambda e: e["step"])
    parts.append("**Failure cascade timeline:**")
    prev_step = 0
    for ev in events:
        step = ev["step"]
        delta = step - prev_step
        if ev["type"] == "joint_breach":
            d = ev["detail"]
            anchor = d.get("anchor")
            force = float(d.get("force", 0.0))
            torque = float(d.get("torque", 0.0))
            jlf = metrics.get("joint_limit_force")
            jlt = metrics.get("joint_limit_torque")
            f_str = ""
            if jlf is not None and _is_finite(jlf) and float(jlf) > 0:
                f_str = f" force={_fm1(force)}N ({force / float(jlf) * 100:.1f}% limit)"
            else:
                f_str = f" force={_fm1(force)}N"
            t_str = ""
            if jlt is not None and _is_finite(jlt) and float(jlt) > 0:
                t_str = f" torque={_fm1(torque)}Nm ({torque / float(jlt) * 100:.1f}% limit)"
            parts.append(
                f"  Step {step} (+{delta}): Joint at {_anch_str(anchor)} breached —{f_str}{t_str}"
            )
        elif ev["type"] == "core_impact":
            parts.append(
                f"  Step {step} (+{delta}): Core impact force {_fm1(ev.get('force', 0.0))}N"
            )
        prev_step = step
    total_breaches = sum(1 for e in events if e["type"] == "joint_breach")
    if total_breaches > 0:
        span = events[-1]["step"] - events[0]["step"]
        parts.append(
            f"Cascade: {total_breaches} breach(es) over {span} steps "
            f"(steps {events[0]['step']}–{events[-1]['step']})."
        )
    return parts

def _format_spatial_diagnostics(metrics: Dict[str, Any]) -> List[str]:
    parts: List[str] = ["### 2. Spatial State"]
    cx = metrics.get("core_x")
    cy = metrics.get("core_y")
    if cx is not None and cy is not None and _is_finite(cx) and _is_finite(cy):
        parts.append(f"Core position: ({_fm2(cx)}, {_fm2(cy)})")
    min_body_y = metrics.get("min_body_y")
    if min_body_y is not None and _is_finite(min_body_y):
        collapse_threshold = metrics.get("collapse_threshold")
        if _is_finite(collapse_threshold):
            ct = float(collapse_threshold)
            margin = float(min_body_y) - ct
            parts.append(
                f"Lowest beam: y={_fm2(min_body_y)}m "
                f"(collapse threshold {ct:.2f}m, margin {'+' if margin >= 0 else ''}{margin:.2f}m)"
            )
        else:
            parts.append(f"Lowest beam: y={_fm2(min_body_y)}m (collapse limit unavailable)")
    meteor_count = metrics.get("meteor_count")
    if meteor_count is not None:
        parts.append(f"Boulder schedule: {meteor_count} total")
    return parts

def _format_load_stress_distribution(metrics: Dict[str, Any]) -> List[str]:
    parts: List[str] = ["### 3. Joint Stress"]
    joint_peak_records = metrics.get("joint_peak_records_v2") or metrics.get("joint_peak_records") or []
    if not joint_peak_records:
        parts.append("No joints tracked.")
        return parts
    jlf = metrics.get("joint_limit_force")
    jlt = metrics.get("joint_limit_torque")
    has_force_limit = jlf is not None and _is_finite(jlf) and float(jlf) > 0
    has_torque_limit = jlt is not None and _is_finite(jlt) and float(jlt) > 0
    scored: List[Dict[str, Any]] = []
    for rec in joint_peak_records:
        pf = float(rec.get("peak_force", 0.0))
        pt = float(rec.get("peak_torque", 0.0))
        fp = (pf / float(jlf) * 100.0) if has_force_limit else 0.0
        tp = (pt / float(jlt) * 100.0) if has_torque_limit else 0.0
        worst = max(fp, tp)
        scored.append({
            "anchor": rec.get("anchor"),
            "force": pf, "torque": pt,
            "force_pct": fp, "torque_pct": tp,
            "worst_pct": worst,
        })
    scored.sort(key=lambda r: r["worst_pct"], reverse=True)
    total = len(scored)
    critical = [s for s in scored if s["worst_pct"] >= 80.0]
    elevated = [s for s in scored if 50.0 <= s["worst_pct"] < 80.0]
    nominal = total - len(critical) - len(elevated)
    parts.append(
        f"Tiers: {len(critical)} critical (≥80%), {len(elevated)} elevated (50–80%), "
        f"{nominal} nominal (<50%) — {total} joints total"
    )
    noteworthy = critical + elevated
    if not noteworthy:
        noteworthy = [s for s in scored if s["worst_pct"] >= 30.0][:3]
    for s in noteworthy:
        tier = (
            "CRITICAL" if s["worst_pct"] >= 80
            else "ELEVATED" if s["worst_pct"] >= 50
            else "nominal"
        )
        icon = "🔴" if tier == "CRITICAL" else "🟡" if tier == "ELEVATED" else "🟢"
        fd = (
            f"force {_fm1(s['force'])}N ({s['force_pct']:.1f}% of limit)"
            if has_force_limit
            else f"force {_fm1(s['force'])}N"
        )
        td = (
            f"torque {_fm1(s['torque'])}Nm ({s['torque_pct']:.1f}% of limit)"
            if has_torque_limit
            else f"torque {_fm1(s['torque'])}Nm"
        )
        parts.append(
            f"  {icon} Joint at {_anch_str(s['anchor'])} — {fd}, {td} — **{tier}** ({s['worst_pct']:.1f}%)"
        )
    if critical:
        parts.append(
            f"  ⚠️ Highest recorded utilization: Joint at {_anch_str(scored[0]['anchor'])} "
            f"({scored[0]['worst_pct']:.1f}%)"
        )
    elif elevated and noteworthy:
        pass
    jbc = metrics.get("joints_broken_count")
    if jbc is not None:
        parts.append(f"Joints broken: {int(jbc)} / {total}")
    return parts

def _format_energy_flow(metrics: Dict[str, Any]) -> List[str]:
    parts: List[str] = ["### 4. Energy & Impact Loads"]
    any_output = False
    tbk = metrics.get("total_boulder_ke")
    if tbk is not None and _is_finite(tbk):
        parts.append(f"Total boulder KE (cumulative): {_fm1(tbk)}J")
        any_output = True
    max_jf = metrics.get("max_joint_force_seen")
    max_jt = metrics.get("max_joint_torque_seen")
    abs_bits = []
    if max_jf is not None and _is_finite(max_jf):
        abs_bits.append(f"peak force {_fm1(max_jf)}N")
    if max_jt is not None and _is_finite(max_jt):
        abs_bits.append(f"peak torque {_fm1(max_jt)}Nm")
    if abs_bits:
        parts.append(f"Structure absorption: {', '.join(abs_bits)}")
        any_output = True
    core_force = metrics.get("core_force")
    if core_force is not None and _is_finite(core_force) and float(core_force) > 0.0:
        parts.append(f"Energy to core: {_fm1(core_force)}N peak force")
        any_output = True
    elif core_force is not None and float(core_force) == 0.0:
        parts.append("Recorded peak force on core: 0.0N (no causal inference from this value alone).")
        any_output = True
    if not any_output:
        parts.append("No energy data available.")
    return parts

def _format_constraint_profile(metrics: Dict[str, Any]) -> List[str]:
    parts: List[str] = ["### 5. Constraints"]
    constraints: List[Dict[str, Any]] = []
    sm = metrics.get("structure_mass")
    mm = metrics.get("max_mass")
    if sm is not None and mm is not None and _is_finite(sm) and _is_finite(mm):
        s, m = float(sm), float(mm)
        margin = m - s
        pct = s / m * 100.0 if m > 0 else 0.0
        constraints.append({
            "name": "Structure Mass",
            "type": "build-time",
            "pct": pct,
            "passed": s <= m,
            "detail": f"{_fm2(s)} / {_fm2(m)}kg ({pct:.1f}% used, {abs(margin):.2f}kg {'available' if margin >= 0 else 'EXCEEDED'})",
        })
    mhl = metrics.get("max_height_limit")
    max_body_y = metrics.get("max_body_y")
    if mhl is not None and _is_finite(mhl) and _is_finite(max_body_y):
        h = float(mhl)
        observed = float(max_body_y)
        constraints.append({
            "name": "Structure Height",
            "type": "build-time",
            "pct": observed / h * 100.0 if h > 0 else float("nan"),
            "passed": observed <= h,
            "detail": f"highest beam y={_fm2(observed)}m / limit {_fm2(h)}m (margin {h - observed:+.2f}m)",
        })
    cf = metrics.get("core_force")
    mcf = metrics.get("max_core_force")
    if cf is not None and mcf is not None and _is_finite(mcf) and float(mcf) > 0:
        if _is_finite(cf):
            c, mc = float(cf), float(mcf)
            margin = mc - c
            pct = c / mc * 100.0 if mc > 0 else 0.0
            constraints.append({
                "name": "Core Impact Force",
                "type": "runtime",
                "pct": pct,
                "passed": c <= mc,
                "detail": f"{_fm1(c)} / {_fm1(mc)}N ({pct:.1f}% of limit, {abs(margin):.1f}N {'below' if c <= mc else 'ABOVE'} limit)",
            })
        else:
            constraints.append({
                "name": "Core Impact Force",
                "type": "runtime",
                "pct": float("nan"),
                "passed": False,
                "detail": "non-finite value",
            })
    mby = metrics.get("min_body_y")
    if mby is not None:
        collapse_threshold = metrics.get("collapse_threshold")
        if _is_finite(mby) and _is_finite(collapse_threshold):
            ct = float(collapse_threshold)
            y = float(mby)
            margin = y - ct
            passed = y >= ct
            constraints.append({
                "name": "Structural Collapse",
                "type": "runtime",
                "pct": float("nan"),
                "passed": passed,
                "detail": (
                    f"lowest y={_fm3(y)}m "
                    f"(threshold {ct:.2f}m, margin {'+' if margin >= 0 else ''}{margin:.3f}m)"
                ),
            })
        else:
            constraints.append({
                "name": "Structural Collapse",
                "type": "runtime",
                "pct": float("nan"),
                "passed": False,
                "detail": "non-finite value",
            })
    jlf = metrics.get("joint_limit_force")
    max_jf = metrics.get("max_joint_force_seen")
    if jlf is not None and _is_finite(jlf):
        jf = float(max_jf) if (max_jf is not None and _is_finite(max_jf)) else 0.0
        jl = float(jlf)
        margin = jl - jf
        pct = jf / jl * 100.0 if jl > 0 else 0.0
        constraints.append({
            "name": "Joint Force",
            "type": "runtime",
            "pct": pct,
            "passed": jf <= jl,
            "detail": f"{_fm1(jf)} / {_fm1(jl)}N ({pct:.1f}% of limit, {abs(margin):.1f}N {'below' if jf <= jl else 'ABOVE'} limit)",
        })
    jlt = metrics.get("joint_limit_torque")
    max_jt = metrics.get("max_joint_torque_seen")
    if jlt is not None and _is_finite(jlt):
        jt = float(max_jt) if (max_jt is not None and _is_finite(max_jt)) else 0.0
        jl = float(jlt)
        margin = jl - jt
        pct = jt / jl * 100.0 if jl > 0 else 0.0
        constraints.append({
            "name": "Joint Torque",
            "type": "runtime",
            "pct": pct,
            "passed": jt <= jl,
            "detail": f"{_fm1(jt)} / {_fm1(jl)}Nm ({pct:.1f}% of limit, {abs(margin):.1f}Nm {'below' if jt <= jl else 'ABOVE'} limit)",
        })
    if not constraints:
        parts.append("No constraint data available.")
        return parts
    constraints.sort(key=lambda c: (
        0 if not c["passed"] else 1,
        0 if (c["passed"] and _is_finite(c.get("pct")) and float(c["pct"]) >= 50.0) else 1,
        0 if c["type"] == "build-time" else 1,
    ))
    total = len(constraints)
    passed_count = sum(1 for c in constraints if c["passed"])
    failed_count = total - passed_count
    near_limit_count = sum(
        1 for c in constraints
        if c["passed"] and _is_finite(c.get("pct")) and float(c["pct"]) >= 70.0
    )
    parts.append(
        f"Summary: {passed_count}/{total} PASS, {failed_count} FAIL "
        f"({near_limit_count} near-limit >70% utilisation)"
    )
    failed_constraints = [c for c in constraints if not c["passed"]]
    near_limit_constraints = [
        c for c in constraints
        if c["passed"] and _is_finite(c.get("pct")) and float(c["pct"]) >= 50.0
    ]
    low_util = [
        c for c in constraints
        if c["passed"] and (c.get("pct") is None or not _is_finite(c.get("pct")) or float(c["pct"]) < 50.0)
    ]
    for c in failed_constraints:
        parts.append(f"  🔴 {c['name']} [{c['type']}]: {c['detail']}")
    for c in near_limit_constraints:
        near_tag = " ⚠️ NEAR LIMIT" if float(c["pct"]) >= 70.0 else ""
        parts.append(f"  🟢 {c['name']} [{c['type']}]: {c['detail']}{near_tag}")
    if low_util:
        names = ", ".join(c["name"] for c in low_util)
        parts.append(f"  🟢 {len(low_util)} other constraint(s) well within limits ({names})")
    failed = metrics.get("failed")
    fr = metrics.get("failure_reason")
    if failed and fr and str(fr).strip():
        parts.append(f"Primary failure: {fr}")
    return parts

def _format_numerical_health(metrics: Dict[str, Any]) -> List[str]:
    parts: List[str] = ["### 6. Numerical Health"]
    issues: List[str] = []
    nic = metrics.get("numerical_instability_count")
    if nic is not None and int(nic) > 0:
        issues.append(f"{int(nic)} instability event(s)")
    observation_errors = metrics.get("joint_observation_errors")
    if isinstance(observation_errors, dict) and observation_errors.get("count"):
        detail = observation_errors.get("last_error")
        issues.append(
            f"{int(observation_errors['count'])} joint telemetry error(s)"
            + (f"; last: {detail}" if detail else "")
        )
    for key, label in [
        ("core_force", "Core force"),
        ("structure_mass", "Structure mass"),
        ("min_body_y", "Lowest beam height"),
    ]:
        v = metrics.get(key)
        if v is not None and not _is_finite(v):
            issues.append(f"Non-finite {label}")
    mbv = metrics.get("max_body_velocity")
    if mbv is not None and _is_finite(mbv) and float(mbv) > 100.0:
        issues.append(f"Excessive velocity: {_fm1(mbv)} m/s")
    elif mbv is not None and _is_finite(mbv):
        pass
    if issues:
        parts.append("⚠️ " + "; ".join(issues))
    elif nic is not None:
        extra = ""
        if mbv is not None and _is_finite(mbv):
            extra = f", max velocity {_fm1(mbv)} m/s"
        parts.append(f"Clean — no instabilities or anomalies{extra}.")
    else:
        parts.append("No health data available.")
    return parts

def format_task_metrics(metrics: Dict[str, Any]) -> List[str]:
    if not metrics:
        return ["**No metrics available** for this evaluation run."]
    parts: List[str] = []
    success = bool(metrics.get("success", False))
    failed = bool(metrics.get("failed", False))
    reason = metrics.get("failure_reason")
    if success:
        parts.append("## Outcome: SUCCESS")
    elif failed:
        parts.append(f"## Outcome: FAILED — {reason or 'evaluator reported failure'}")
    else:
        parts.append("## Outcome: IN PROGRESS")
    parts.append("")
    dims = [
        _format_temporal_chronology,
        _format_spatial_diagnostics,
        _format_load_stress_distribution,
        _format_energy_flow,
        _format_constraint_profile,
        _format_numerical_health,
    ]
    for formatter in dims:
        try:
            dim_parts = formatter(metrics)
            if dim_parts:
                parts.extend(dim_parts)
                parts.append("")
        except Exception as e:
            parts.append(f"### [Error formatting: {type(e).__name__}: {e}]")
            parts.append("")
    return parts

def get_improvement_suggestions(
    metrics: Dict[str, Any],
    score: float,
    success: bool,
    failed: bool,
    failure_reason: Any = None,
    error: Optional[str] = None,

) -> List[str]:
    suggestions: List[str] = []
    if error:
        error_lower = error.lower()
        if "keep-out zone" in error_lower or "1.3m" in error_lower:
            suggestions.append(
            )
        elif "build zone" in error_lower or "outside build" in error_lower:
            suggestions.append(
            )
        elif "height limit" in error_lower:
            suggestions.append(
            )
        elif "beam dimensions" in error_lower or "0.1" in error_lower:
            suggestions.append(
            )
        elif "math is not defined" in error_lower or "name 'math'" in error_lower:
            suggestions.append("- Add 'import math' at the top of your code.")
        else:
            suggestions.append("- Review the error message and fix the build constraint violation.")
    if success:
        return suggestions
    if not metrics:
        return suggestions
    return suggestions
