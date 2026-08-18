from typing import Dict, Any, List

import math

def _is_finite(x: Any) -> bool:
    if x is None:
        return False
    try:
        return math.isfinite(float(x))
    except (TypeError, ValueError):
        return False

def _fmt(x: Any, decimals: int = 2) -> str:
    if x is None:
        return "N/A"
    try:
        return f"{float(x):.{decimals}f}"
    except (TypeError, ValueError):
        return str(x)

def _pct(value: Any, limit: Any) -> str:
    try:
        v, l = float(value), float(limit)
        if l > 0 and math.isfinite(l):
            return f"{100.0 * v / l:.1f}% of limit"
    except (TypeError, ValueError):
        pass
    return ""

def _reason_category(reason: str) -> str:
    if not reason:
        return ""
    r = reason.lower()
    if "width" in r and ">" in r:
        return "width"
    if "beam dimensions" in r:
        return "beam_dimensions"
    if "foundation contact" in r or "limit: " in r:
        return "foundation"
    if "collapsed" in r or "fell too low" in r:
        return "collapse"
    if "tipped" in r or "rel_com_x" in r:
        return "stability"
    if "explosion" in r or "instability" in r:
        return "numerical"
    if "target height not reached" in r or "target: " in r:
        return "target_height"
    return "other"

def _format_env_compact(metrics: Dict[str, Any]) -> str:
    env = metrics.get("env_params", {}) or {}
    items = []
    mjf = env.get("max_joint_force")
    mjt = env.get("max_joint_torque")
    limits = []
    if mjf is not None and _is_finite(mjf):
        limits.append(f"F≤{_fmt(mjf, 0)}N")
    if mjt is not None and _is_finite(mjt):
        limits.append(f"T≤{_fmt(mjt, 0)}N·m")
    if limits:
        items.append(f"joints: {', '.join(limits)}")
    return " | ".join(items) if items else "(no env data)"

def _format_failures_compact(metrics: Dict[str, Any]) -> List[str]:
    parts: List[str] = []
    jfe = metrics.get("joint_failure_events", []) or []
    if not jfe:
        nj = metrics.get("num_joints")
        ijc = metrics.get("initial_joint_count")
        if nj is None and ijc is None:
            parts.append("Joint integrity: data unavailable")
        elif (nj or 0) > 0 or (ijc or 0) > 0:
            parts.append("Joint integrity: no failures recorded")
        else:
            parts.append("No joints present")
        return parts
    eq_start_time = 2.0
    env = metrics.get("env_params", {}) or {}
    if env.get("earthquake_start_time") is not None:
        eq_start_time = float(env["earthquake_start_time"])
    quake_step = int(eq_start_time * 60.0)
    pre_quake = [e for e in jfe if e.get("step", 0) < quake_step]
    during_quake = [e for e in jfe if e.get("step", 0) >= quake_step]
    parts.append(f"Joint failures: {len(jfe)} total "
                 f"(pre-quake: {len(pre_quake)}, during-quake: {len(during_quake)})")
    if pre_quake:
        first_n = min(3, len(pre_quake))
        parts.append(f"  First {first_n} pre-quake:")
        for ev in pre_quake[:first_n]:
            step = ev.get("step", "?")
            force = ev.get("force", 0)
            torque = ev.get("torque", 0)
            fl = ev.get("max_force_limit")
            tl = ev.get("max_torque_limit")
            anchor_y = _fmt(ev.get("anchor_y")) + "m" if ev.get("anchor_y") is not None else ""
            f_str = f"F={_fmt(force, 1)}N"
            if fl and fl > 0:
                f_str += f" ({_pct(force, fl)})"
            t_str = f"T={_fmt(torque, 1)}N·m"
            if tl and tl > 0:
                t_str += f" ({_pct(torque, tl)})"
            parts.append(f"    step={step} anchor_y={anchor_y} | {f_str} {t_str}")
    if len(jfe) > 3:
        last = jfe[-1]
        step = last.get("step", "?")
        anchor_y = _fmt(last.get("anchor_y")) + "m" if last.get("anchor_y") is not None else ""
        force = last.get("force", 0)
        torque = last.get("torque", 0)
        fl = last.get("max_force_limit")
        tl = last.get("max_torque_limit")
        f_str = f"F={_fmt(force, 1)}N"
        if fl and fl > 0:
            f_str += f" ({_pct(force, fl)})"
        t_str = f"T={_fmt(torque, 1)}N·m"
        if tl and tl > 0:
            t_str += f" ({_pct(torque, tl)})"
        parts.append(f"  Last failure: step={step} anchor_y={anchor_y} | {f_str} {t_str}")
    first_step = min(e.get("step", 0) for e in jfe)
    last_step = max(e.get("step", 0) for e in jfe)
    first_anchor = _fmt(jfe[0].get("anchor_y"))
    last_anchor = _fmt(jfe[-1].get("anchor_y"))
    parts.append(
        f"  Cascade: step {first_step} (y≈{first_anchor}m) → "
        f"step {last_step} (y≈{last_anchor}m), "
        f"{last_step - first_step} steps, {len(jfe)} joints"
    )
    if during_quake:
        parts.append(f"  During-quake failures ({len(during_quake)}): not listed "
                     f"(step {during_quake[0].get('step', '?')} → {during_quake[-1].get('step', '?')})")
    return parts

def _format_spatial_compact(metrics: Dict[str, Any]) -> List[str]:
    parts: List[str] = []
    beams = metrics.get("per_beam_positions", []) or []
    target_height = metrics.get("target_height")
    build_half = metrics.get("build_zone_half_width")
    max_width = metrics.get("max_width_limit")
    if not beams:
        return parts
    max_y_beam = max(beams, key=lambda b: b.get("y", 0))
    if target_height is not None and _is_finite(target_height):
        margin = float(target_height) - float(max_y_beam.get("y", 0))
        parts.append(
            f"Height: top beam y={_fmt(max_y_beam.get('y'))}m "
            f"(target {_fmt(target_height)}m, margin {margin:+.2f}m)"
        )
    else:
        parts.append(f"Height: top beam y={_fmt(max_y_beam.get('y'))}m (target unavailable)")
    xs = [b.get("x", 0) for b in beams if _is_finite(b.get("x"))]
    if xs and max_width is not None and _is_finite(max_width):
        spread = max(xs) - min(xs)
        w_margin = float(max_width) - spread
        parts.append(f"Width: spread={_fmt(spread)}m (limit {_fmt(max_width)}m, margin {w_margin:+.2f}m)")
    violations = []
    for b in beams:
        if build_half is not None and _is_finite(build_half) and b.get("y", 0) < 1.01 and abs(b.get("x", 0)) > float(build_half):
            violations.append({"index": b.get("index"), "x": b.get("x"), "dist": abs(b.get("x", 0))})
    if violations and build_half is not None:
        parts.append(f"Foundation breach: {len(violations)} beam(s) outside ±{build_half}m")
        for v in violations[:3]:
            parts.append(f"  Beam #{v['index']}: x={_fmt(v['x'])}m (dist={_fmt(v['dist'])}m)")
    min_y_beam = min(beams, key=lambda b: b.get("y", 0))
    min_y = min_y_beam.get("y", 0)
    if min_y < 1.0:
        survival = metrics.get("survival_threshold")
        if survival is not None and _is_finite(survival):
            parts.append(f"Lowest beam: y={_fmt(min_y)}m (survival threshold {_fmt(survival)}m)")
        else:
            parts.append(f"Lowest beam: y={_fmt(min_y)}m")
    return parts

def _format_load_compact(metrics: Dict[str, Any]) -> List[str]:
    parts: List[str] = []
    jss = metrics.get("per_joint_stress_summary", []) or []
    fl = metrics.get("max_joint_force_limit")
    tl = metrics.get("max_joint_torque_limit")
    peak_jf = metrics.get("peak_joint_force", 0)
    peak_jt = metrics.get("peak_joint_torque", 0)
    jbc = metrics.get("joint_break_count", 0)
    nj = metrics.get("num_joints", 0)
    ijc = metrics.get("initial_joint_count", nj + jbc)
    peak_parts = []
    if peak_jf is not None and _is_finite(peak_jf):
        if fl is not None and _is_finite(fl) and float(fl) > 0:
            pct_f = (float(peak_jf) / float(fl)) * 100
            peak_parts.append(f"max force={_fmt(peak_jf, 0)}N ({pct_f:.0f}% of limit)")
        else:
            peak_parts.append(f"max force={_fmt(peak_jf, 0)}N")
    if peak_jt is not None and _is_finite(peak_jt):
        if tl is not None and _is_finite(tl) and float(tl) > 0:
            pct_t = (float(peak_jt) / float(tl)) * 100
            peak_parts.append(f"max torque={_fmt(peak_jt, 0)}N·m ({pct_t:.0f}% of limit)")
        else:
            peak_parts.append(f"max torque={_fmt(peak_jt, 0)}N·m")
    if peak_parts:
        parts.append(f"Stress peaks: {', '.join(peak_parts)} "
                     f"({nj}/{ijc} joints intact, {jbc} broken)")
    if not jss:
        return parts
    critical = []
    elevated = []
    nominal_count = 0
    for j in jss:
        pf = j.get("peak_force", 0)
        tier_pct = 0.0
        if fl and fl > 0:
            tier_pct = (pf / float(fl)) * 100
        entry = {
            "data": j,
            "pct_force": tier_pct,
            "anchor_y": j.get("anchor_y"),
        }
        if tier_pct >= 80.0:
            critical.append(entry)
        elif tier_pct >= 50.0:
            elevated.append(entry)
        else:
            nominal_count += 1
    if critical:
        show_n = min(5, len(critical))
        parts.append(f"  Top {show_n} most-stressed joints "
                     f"(of {len(critical)} critical, {len(elevated)} elevated, {nominal_count} nominal):")
        for e in critical[:show_n]:
            j = e["data"]
            pf = j.get("peak_force", 0)
            pt = j.get("peak_torque", 0)
            broken = "BROKEN" if j.get("broken", False) else "OK"
            f_str = f"F={_fmt(pf, 0)}N"
            if fl and fl > 0:
                f_str += f"({_pct(pf, fl)})"
            t_str = f"T={_fmt(pt, 0)}N·m"
            if tl and tl > 0:
                t_str += f"({_pct(pt, tl)})"
            parts.append(f"    y={_fmt(e['anchor_y'])}m | {f_str} {t_str} | {broken}")
        if len(critical) > show_n:
            parts.append(f"    ... and {len(critical) - show_n} more critical")
    elif elevated:
        parts.append(f"  {len(elevated)} joints at elevated stress, "
                     f"{nominal_count} nominal — no critical overloads")
    foundation_joints = [j for j in jss if j.get("anchor_y") is not None
                         and float(j.get("anchor_y", 99)) < 2.0]
    upper_joints = [j for j in jss if j.get("anchor_y") is not None
                    and float(j.get("anchor_y", 0)) >= 2.0]
    if foundation_joints and upper_joints:
        fnd_avg = sum(j.get("peak_force", 0) for j in foundation_joints) / max(len(foundation_joints), 1)
        upr_avg = sum(j.get("peak_force", 0) for j in upper_joints) / max(len(upper_joints), 1)
        if upr_avg > 0 and (fnd_avg / upr_avg) > 2.0:
            parts.append(f"  Stress concentration: foundation/upr ratio = {fnd_avg / upr_avg:.1f}:1")
    return parts

def _format_constraints_compact(metrics: Dict[str, Any]) -> List[str]:
    parts: List[str] = []
    acr = metrics.get("all_constraint_results", {}) or {}
    fr = metrics.get("failure_reason")
    if not acr:
        if fr:
            parts.append(f"Primary failure: {fr}")
        return parts
    order = ["beam_dimensions", "foundation_contact", "width",
             "collapse", "tipped_over", "explosion", "target_height"]
    available = [k for k in order if k in acr]
    fail_count = 0
    near_limit = []
    passed_list = []
    for key in available:
        c = acr[key]
        passed = c.get("passed", True)
        label = c.get("label", key)
        val = c.get("value")
        lim = c.get("limit")
        margin = c.get("margin")
        if not passed:
            fail_count += 1
            detail = []
            if isinstance(val, dict) and val:
                if "distance" in val:
                    detail.append(f"distance={_fmt(val['distance'], 2)}m")
                elif "x" in val and "y" in val:
                    detail.append(f"x={_fmt(val['x'], 2)}, y={_fmt(val['y'], 2)}")
                else:
                    inner = ", ".join(f"{k}={_fmt(v, 2)}" for k, v in val.items())
                    detail.append(inner)
            elif val is not None and _is_finite(val):
                detail.append(f"value={_fmt(val, 2)}")
            if lim is not None and not isinstance(lim, dict) and _is_finite(lim):
                detail.append(f"limit={_fmt(lim, 2)}")
            elif isinstance(lim, dict) and lim:
                if "half_width" in lim:
                    detail.append(f"limit=±{_fmt(lim['half_width'], 2)}m")
                elif "max" in lim:
                    detail.append(f"limit≤{_fmt(lim['max'], 2)}")
            if margin is not None and _is_finite(margin):
                detail.append(f"margin={margin:+.2f}")
            parts.append(f"  ❌ FAILED: {label} ({', '.join(detail)})")
        else:
            passed_list.append(label)
            if margin is not None and _is_finite(margin) and isinstance(lim, (int, float)) and lim and float(lim) > 0:
                pct_used = 100.0 - (float(margin) / float(lim)) * 100.0
                if pct_used > 70.0:
                    near_limit.append(f"{label} ({pct_used:.0f}%)")
    if fail_count == 0 and not near_limit:
        parts.append(f"Constraints: all {len(available)} PASSED")
    elif fail_count == 0:
        parts.append(f"Constraints: all {len(available)} PASSED "
                     f"(near-limit: {', '.join(near_limit)})")
    else:
        parts.append(f"Constraints: {fail_count}/{len(available)} FAILED")
        if near_limit:
            parts.append(f"  Near-limit: {', '.join(near_limit)}")
    pfd = metrics.get("peak_foundation_displacement")
    contact_limit = metrics.get("foundation_contact_limit")
    if pfd is not None and _is_finite(pfd) and float(pfd) > 0 and contact_limit is not None and _is_finite(contact_limit):
        pfd_passed = float(pfd) <= float(contact_limit)
        if not pfd_passed:
            parts.append(f"  ❌ FAILED: Foundation lateral drift "
                         f"peak={_fmt(pfd)}m > ±{_fmt(contact_limit)}m")
    if fr and fail_count == 0:
        parts.append(f"Evaluator failure reason: {fr}")
    return parts

def _format_health_compact(metrics: Dict[str, Any]) -> List[str]:
    parts: List[str] = []
    anomalous = []
    for k in ("initial_height", "min_height_during_quake", "rel_com_x",
              "current_height", "peak_joint_force", "peak_joint_torque",
              "peak_foundation_displacement", "structure_mass"):
        v = metrics.get(k)
        if v is not None and not _is_finite(v):
            anomalous.append(f"{k}={v}")
    max_vel = metrics.get("max_body_velocity")
    vel_str = f"max velocity={_fmt(max_vel, 1)} m/s" if max_vel is not None and _is_finite(max_vel) else ""
    if anomalous:
        parts.append(f"⚠ Non-finite metrics: {', '.join(anomalous)} | {vel_str}")
    else:
        parts.append(f"Numerics: clean | {vel_str}")
    observation_errors = metrics.get("joint_observation_error_count")
    if observation_errors is not None and _is_finite(observation_errors) and int(observation_errors) > 0:
        detail = metrics.get("last_joint_observation_error")
        parts.append(
            f"⚠ Joint reaction telemetry failed {int(observation_errors)} time(s)"
            + (f"; last error: {detail}" if detail else "")
        )
    if max_vel is not None and _is_finite(max_vel):
        vel = float(max_vel)
        if vel > 100.0:
            parts.append(f"⚠ Extreme velocity {vel:.0f} m/s — likely solver divergence")
        elif vel > 50.0:
            parts.append(f"⚠ Elevated velocity {vel:.0f} m/s — possible joint-break fling")
    ch = metrics.get("current_height")
    ihl = metrics.get("instability_height_limit")
    if ch is not None and _is_finite(ch) and ihl is not None and _is_finite(ihl) and float(ch) > float(ihl):
        parts.append(f"⚠ Height {_fmt(ch)}m > instability cap {_fmt(ihl)}m")
    return parts

def format_task_metrics(metrics: Dict[str, Any]) -> List[str]:
    parts: List[str] = []
    try:
        if not metrics:
            return ["**No task metrics available.**"]
        if metrics.get("error"):
            parts.append(f"**Evaluation note**: {metrics.get('error')}")
            return parts
        success = metrics.get("success", False)
        failed = metrics.get("failed", False)
        fr = metrics.get("failure_reason")
        eval_step = metrics.get("eval_step", metrics.get("step_count"))
        max_steps = metrics.get("max_steps_setting", "?")
        if success:
            parts.append("**Outcome**: ✅ SUCCESS — all constraints satisfied")
        elif failed:
            cat = _reason_category(str(fr or ""))
            parts.append(f"**Outcome**: ❌ FAILED — [{cat}] {fr}")
        else:
            step_text = eval_step if eval_step is not None else "?"
            parts.append(f"**Outcome**: ⚠ IN PROGRESS — step {step_text}/{max_steps}")
        ih = metrics.get("initial_height")
        mh = metrics.get("min_height_during_quake")
        jbc = metrics.get("joint_break_count")
        nj = metrics.get("num_joints")
        sm = metrics.get("structure_mass")
        nb = metrics.get("num_bodies")
        summary_items = []
        if ih is not None and _is_finite(ih):
            th = metrics.get("target_height")
            if th and _is_finite(th):
                summary_items.append(f"height={float(ih):.2f}/{float(th):.2f}m")
            else:
                summary_items.append(f"height={float(ih):.2f}m")
        if mh is not None and _is_finite(mh):
            st = metrics.get("survival_threshold")
            if st and _is_finite(st):
                summary_items.append(f"min-quake={float(mh):.2f}/{float(st):.2f}m")
            else:
                summary_items.append(f"min-quake={float(mh):.2f}m")
        if nj is not None and jbc is not None:
            summary_items.append(f"joints={nj}/{nj + jbc} intact")
        if sm is not None and _is_finite(sm):
            summary_items.append(f"mass={float(sm):.2f}kg")
        if nb is not None:
            summary_items.append(f"beams={nb}")
        parts.append(
            "**Summary**: " + " | ".join(summary_items)
            if summary_items
            else "**Summary**: no measurements available"
        )
        parts.append("")
        parts.append(f"Environment: {_format_env_compact(metrics)}")
        parts.append("")
        if eval_step == 0:
            try:
                constraint_parts = _format_constraints_compact(metrics)
                if constraint_parts:
                    parts.extend(constraint_parts)
                    parts.append("")
            except Exception as exc:
                parts.append(f"Constraint formatting error: {type(exc).__name__}: {exc}")
            try:
                spatial_parts = _format_spatial_compact(metrics)
                if spatial_parts:
                    parts.extend(spatial_parts)
                    parts.append("")
            except Exception as exc:
                parts.append(f"Spatial formatting error: {type(exc).__name__}: {exc}")
            return parts
        try:
            failure_parts = _format_failures_compact(metrics)
            if failure_parts:
                parts.append("**Failures**:")
                parts.extend(failure_parts)
                parts.append("")
        except Exception as exc:
            parts.append(f"Failure chronology formatting error: {type(exc).__name__}: {exc}")
        try:
            spatial_parts = _format_spatial_compact(metrics)
            if spatial_parts:
                parts.append("**Spatial**:")
                parts.extend(spatial_parts)
                parts.append("")
        except Exception as exc:
            parts.append(f"Spatial formatting error: {type(exc).__name__}: {exc}")
        try:
            load_parts = _format_load_compact(metrics)
            if load_parts:
                parts.append("**Stress**:")
                parts.extend(load_parts)
                parts.append("")
        except Exception as exc:
            parts.append(f"Stress formatting error: {type(exc).__name__}: {exc}")
        try:
            constraint_parts = _format_constraints_compact(metrics)
            if constraint_parts:
                parts.append("**Constraints**:")
                parts.extend(constraint_parts)
                parts.append("")
        except Exception as exc:
            parts.append(f"Constraint formatting error: {type(exc).__name__}: {exc}")
        try:
            health_parts = _format_health_compact(metrics)
            if health_parts:
                parts.append("**Health**:")
                parts.extend(health_parts)
        except Exception as exc:
            parts.append(f"Numerical-health formatting error: {type(exc).__name__}: {exc}")
    except Exception as exc:
        parts = [f"**Task feedback formatting error**: {type(exc).__name__}: {exc}"]
    if not parts:
        try:
            from pace_bench.evaluation.verification.diagnostics import (
                format_generic_execution_metrics,
            )
            parts = format_generic_execution_metrics(metrics)
        except Exception as exc:
            parts = [f"**Generic feedback formatting error**: {type(exc).__name__}: {exc}"]
    return parts

def get_improvement_suggestions(metrics, score, success, failed,
                                failure_reason, error):
    return []
