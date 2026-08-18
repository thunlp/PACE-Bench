from typing import Dict, Any, List

import math

def _is_valid_number(x: Any) -> bool:
    if x is None:
        return False
    try:
        f = float(x)
        return math.isfinite(f)
    except (TypeError, ValueError):
        return False

def _fmt_val(v: Any, decimals: int = 3) -> str:
    try:
        f = float(v)
        if not math.isfinite(f):
            return str(v)
        return f"{f:.{decimals}f}"
    except (TypeError, ValueError):
        return str(v)

def _check_tag(passed: bool) -> str:
    return "PASS" if passed else "FAIL"

def _phase_label(step: int, phase_1: int | None, phase_2: int | None) -> str:
    if phase_1 is None or phase_2 is None:
        return "[phase unavailable]"
    if step < phase_1:
        return "[pre-load]"
    elif step < phase_2:
        return "[L1 active]"
    else:
        return "[L1+L2 active]"

def _build_temporal_chronology(metrics: Dict[str, Any]) -> List[str]:
    parts: List[str] = []
    parts.append("### 1. Temporal Event Chronology\n")
    joint_failure_records = metrics.get("joint_failure_records", []) or []
    first_failure_step = metrics.get("first_failure_step")
    first_warning_step = metrics.get("first_warning_step")
    step_count = metrics.get("step_count")
    final_step = int(step_count) if _is_valid_number(step_count) else 0
    time_step = metrics.get("time_step")
    seconds_per_step = float(time_step) if _is_valid_number(time_step) and float(time_step) > 0 else None
    has_events = (
        (_is_valid_number(first_failure_step) and int(first_failure_step) >= 0)
        or (_is_valid_number(first_warning_step) and int(first_warning_step) >= 0)
        or (joint_failure_records and len(joint_failure_records) > 0)
    )
    if not has_events:
        elapsed = final_step * seconds_per_step if seconds_per_step is not None else None
        parts.append(
            f"  No failures or stress warnings. Simulation stopped at step {final_step}"
            + (f" (~{elapsed:.2f}s).\n" if elapsed is not None else ".\n")
        )
        return parts
    load_attach_time = metrics.get("load_attach_time")
    load_2_attach_time = metrics.get("load_2_attach_time")
    phase_1_step = (
        int(float(load_attach_time) / seconds_per_step)
        if seconds_per_step is not None and _is_valid_number(load_attach_time)
        else None
    )
    phase_2_step = (
        int(float(load_2_attach_time) / seconds_per_step)
        if seconds_per_step is not None and _is_valid_number(load_2_attach_time)
        else None
    )
    if _is_valid_number(first_warning_step) and int(first_warning_step) >= 0:
        fws = int(first_warning_step)
        sim_t = fws * seconds_per_step if seconds_per_step is not None else None
        phase = _phase_label(fws, phase_1_step, phase_2_step)
        time_text = f" (~{sim_t:.2f}s)" if sim_t is not None else ""
        warning_fraction = metrics.get("joint_warning_fraction")
        warning_text = (
            f">{100.0 * float(warning_fraction):.0f}% limit"
            if _is_valid_number(warning_fraction)
            else "threshold crossed"
        )
        parts.append(f"**First stress warning ({warning_text}):** step {fws}{time_text} {phase}")
    else:
        parts.append("**First stress warning:** not triggered")
    if _is_valid_number(first_failure_step) and int(first_failure_step) >= 0:
        ffs = int(first_failure_step)
        sim_t = ffs * seconds_per_step if seconds_per_step is not None else None
        phase = _phase_label(ffs, phase_1_step, phase_2_step)
        time_text = f" (~{sim_t:.2f}s)" if sim_t is not None else ""
        parts.append(f"**First joint failure:** step {ffs}{time_text} {phase}")
    else:
        parts.append("**First joint failure:** none")
    parts.append("")
    if joint_failure_records:
        parts.append("**Failure cascade** (chronological):\n")
        sorted_records = sorted(joint_failure_records, key=lambda r: int(r.get("fail_step", 0)))
        for idx, rec in enumerate(sorted_records, start=1):
            jtype = "Wall anchor" if rec.get("is_wall") else "Internal"
            ax = rec.get("anchor_x", 0.0)
            ay = rec.get("anchor_y", 0.0)
            pf = rec.get("peak_force", 0.0)
            pt = rec.get("peak_torque", 0.0)
            lf = rec.get("limit_force", 0.0)
            lt = rec.get("limit_torque", 0.0)
            fs = rec.get("fail_step", -1)
            sim_t = fs * seconds_per_step if fs >= 0 and seconds_per_step is not None else None
            phase = _phase_label(fs, phase_1_step, phase_2_step)
            exceeded = []
            if lf > 0 and pf > lf:
                pct_f = pf / lf * 100.0
                exceeded.append(f"force {_fmt_val(pf, 1)}N > {_fmt_val(lf, 1)}N ({_fmt_val(pct_f, 1)}%)")
            if lt > 0 and pt > lt:
                pct_t = pt / lt * 100.0
                exceeded.append(f"torque {_fmt_val(pt, 1)}N.m > {_fmt_val(lt, 1)}N.m ({_fmt_val(pct_t, 1)}%)")
            exceed_str = "; ".join(exceeded) if exceeded else "limit data missing"
            parts.append(
                f"  #{idx}: [{jtype}] at ({_fmt_val(ax, 2)}, {_fmt_val(ay, 2)}) m | "
                f"step {fs}{f' (~{sim_t:.2f}s)' if sim_t is not None else ''} {phase} | Exceeded: {exceed_str}"
            )
        parts.append("")
    else:
        parts.append("**Failure cascade:** no joints failed.\n")
    elapsed = final_step * seconds_per_step if seconds_per_step is not None else None
    parts.append(
        f"**Simulation stopped at:** step {final_step}"
        + (f" (~{elapsed:.2f}s)" if elapsed is not None else "")
    )
    return parts

def _build_spatial_diagnostics(metrics: Dict[str, Any]) -> List[str]:
    parts: List[str] = []
    parts.append("### 2. Spatial Diagnostics\n")
    max_reach = metrics.get("max_reach")
    target_reach = metrics.get("target_reach")
    min_tip_y = metrics.get("min_tip_y")
    min_tip_height = metrics.get("min_tip_height")
    structure_mass = metrics.get("structure_mass")
    max_structure_mass = metrics.get("max_structure_mass")
    anchor_count = metrics.get("anchor_count")
    max_anchors_limit = metrics.get("max_anchors_limit")
    if _is_valid_number(max_reach) and _is_valid_number(target_reach):
        mr, tr = float(max_reach), float(target_reach)
        margin = mr - tr
        pct = (mr / tr * 100.0) if tr > 0 else 0.0
        passed = mr >= tr
        parts.append(f"  Reach: {_fmt_val(mr, 2)} m (need >= {_fmt_val(tr, 2)} m, "
                     f"margin {margin:+.2f} m, {_fmt_val(pct, 1)}% of target) [{_check_tag(passed)}]")
    if _is_valid_number(min_tip_y) and _is_valid_number(min_tip_height):
        mty, mth = float(min_tip_y), float(min_tip_height)
        margin = mty - mth
        passed = mty >= mth
        parts.append(f"  Tip height: min y = {mty:.3f} m (limit >= {mth:.2f} m, "
                     f"margin {margin:+.3f} m) [{_check_tag(passed)}]")
    if _is_valid_number(structure_mass) and _is_valid_number(max_structure_mass):
        sm, msm = float(structure_mass), float(max_structure_mass)
        pct = (sm / msm * 100.0) if msm > 0 else 0.0
        passed = sm <= msm
        parts.append(f"  Mass: {_fmt_val(sm, 2)} kg / {_fmt_val(msm, 2)} kg "
                     f"({_fmt_val(pct, 1)}% used) [{_check_tag(passed)}]")
    if _is_valid_number(anchor_count) and _is_valid_number(max_anchors_limit):
        parts.append(f"  Anchors: {int(anchor_count)} / {int(max_anchors_limit)} max")
    forbidden_anchor_y = metrics.get("forbidden_anchor_y")
    wall_anchor_positions = metrics.get("wall_anchor_positions", []) or []
    if forbidden_anchor_y is not None and len(forbidden_anchor_y) == 2 and wall_anchor_positions:
        fy_min, fy_max = float(forbidden_anchor_y[0]), float(forbidden_anchor_y[1])
        in_zone = [y for y in wall_anchor_positions if fy_min <= y <= fy_max]
        if in_zone:
            in_str = ", ".join(f"y={y:.2f}" for y in in_zone)
            parts.append(f"  ⚠ Anchors IN forbidden zone [{fy_min:.1f}, {fy_max:.1f}]: {in_str}")
    parts.append("")
    return parts

def _build_load_distribution(metrics: Dict[str, Any]) -> List[str]:
    parts: List[str] = []
    parts.append("### 3. Load & Stress Distribution\n")
    joint_stress_summary = metrics.get("joint_stress_summary", []) or []
    joint_count = metrics.get("joint_count")
    initial_joint_count = metrics.get("initial_joint_count")
    if _is_valid_number(joint_count) and _is_valid_number(initial_joint_count):
        jc, ijc = int(joint_count), int(initial_joint_count)
        lost = ijc - jc
        if lost > 0:
            parts.append(f"**Joints:** {jc} surviving / {ijc} initial ({lost} broken)")
        else:
            parts.append(f"**Joints:** {jc} / {ijc} (all intact)")
    parts.append("")
    if not joint_stress_summary:
        parts.append("  No per-joint stress data available.\n")
        return parts
    entries = list(joint_stress_summary)
    n_critical = sum(1 for e in entries if e.get("max_stress_pct", 0) >= 100.0)
    n_elevated = sum(1 for e in entries if 80.0 <= e.get("max_stress_pct", 0) < 100.0)
    n_moderate = sum(1 for e in entries if 50.0 <= e.get("max_stress_pct", 0) < 80.0)
    n_broken = sum(1 for e in entries if e.get("failed", False))
    has_interesting = (n_critical + n_elevated + n_moderate + n_broken) > 0
    if not has_interesting:
        parts.append(f"  All {len(entries)} joints: NOMINAL (<50% stress). No stress concerns.\n")
        return parts
    parts.append("**Stress tier summary:**")
    parts.append(f"  BROKEN: {n_broken} joint(s)" if n_broken > 0 else f"  BROKEN: 0")
    parts.append(f"  CRITICAL (>100%): {n_critical} joint(s)")
    parts.append(f"  ELEVATED (80-100%): {n_elevated} joint(s)")
    parts.append(f"  MODERATE (50-80%): {n_moderate} joint(s)")
    parts.append("")
    interesting = [e for e in entries if e.get("max_stress_pct", 0) >= 50.0]
    if interesting:
        parts.append("**Non-NOMINAL joints** (worst first):\n")
        max_show = min(len(interesting), 15)
        for rank, entry in enumerate(interesting[:max_show], start=1):
            is_wall = entry.get("is_wall", False)
            jtype = "WALL" if is_wall else "internal"
            ax = entry.get("anchor_x", 0.0)
            ay = entry.get("anchor_y", 0.0)
            pf = entry.get("peak_force", 0.0)
            pt = entry.get("peak_torque", 0.0)
            f_pct = entry.get("force_pct", 0.0)
            t_pct = entry.get("torque_pct", 0.0)
            max_pct = entry.get("max_stress_pct", 0.0)
            failed = entry.get("failed", False)
            status = "BROKEN" if failed else "OK"
            tier = "CRITICAL" if max_pct >= 100 else ("ELEVATED" if max_pct >= 80 else "MODERATE")
            parts.append(
                f"  {rank:>2d}. [{jtype}] ({_fmt_val(ax, 2)}, {_fmt_val(ay, 2)}) | "
                f"force {_fmt_val(pf, 1)}N ({_fmt_val(f_pct, 1)}%) | "
                f"torque {_fmt_val(pt, 1)}N.m ({_fmt_val(t_pct, 1)}%) | "
                f"stress {_fmt_val(max_pct, 1)}% [{tier}] {status}"
            )
        if len(interesting) > max_show:
            parts.append(f"  ... and {len(interesting) - max_show} more above 50%")
        parts.append("")
    wall_entries = [e for e in entries if e.get("is_wall", False)]
    internal_entries = [e for e in entries if not e.get("is_wall", False)]
    if wall_entries:
        peak_wall = max(e.get("max_stress_pct", 0) for e in wall_entries)
        parts.append(f"  Wall anchor peak stress: {_fmt_val(peak_wall, 1)}% ({len(wall_entries)} anchors)")
    if internal_entries:
        peak_int = max(e.get("max_stress_pct", 0) for e in internal_entries)
        parts.append(f"  Internal joint peak stress: {_fmt_val(peak_int, 1)}% ({len(internal_entries)} joints)")
    return parts

def _build_energy_context(metrics: Dict[str, Any]) -> List[str]:
    parts: List[str] = []
    parts.append("### 4. Energy & Impact Context\n")
    lt = metrics.get("load_type", "static")
    lm = metrics.get("load_mass")
    mass_text = f", payload mass: {_fmt_val(lm, 1)} kg" if _is_valid_number(lm) else ""
    parts.append(f"  Load delivery: {lt}{mass_text}")
    dh = metrics.get("drop_height")
    if lt == "dropped" and _is_valid_number(dh) and float(dh) > 0:
        parts.append(f"  Drop height: {_fmt_val(dh, 1)} m (dropped load — higher impulse than static placement)")
    parts.append("")
    return parts

def _build_constraint_profile(metrics: Dict[str, Any]) -> List[str]:
    parts: List[str] = []
    parts.append("### 5. Constraint Satisfaction Profile\n")
    constraints = []
    sm = metrics.get("structure_mass")
    msm = metrics.get("max_structure_mass")
    if _is_valid_number(sm) and _is_valid_number(msm):
        smf, msmf = float(sm), float(msm)
        pct = (smf / msmf * 100.0) if msmf > 0 else 0.0
        constraints.append({
            "constraint": "Mass Budget",
            "value": f"{_fmt_val(smf, 2)} kg",
            "limit": f"{_fmt_val(msmf, 2)} kg",
            "margin": f"{msmf - smf:+.2f} kg",
            "pct": pct,
            "passed": smf <= msmf,
        })
    ac = metrics.get("anchor_count")
    mal = metrics.get("max_anchors_limit")
    if _is_valid_number(ac) and _is_valid_number(mal):
        acf, malf = int(ac), int(mal)
        constraints.append({
            "constraint": "Anchor Count",
            "value": str(acf),
            "limit": f"<= {malf}",
            "margin": f"{malf - acf:+d}",
            "pct": (acf / malf * 100.0) if malf > 0 else 0.0,
            "passed": acf <= malf,
        })
    forbidden_anchor_y = metrics.get("forbidden_anchor_y")
    wall_anchor_positions = metrics.get("wall_anchor_positions", []) or []
    if forbidden_anchor_y is not None and len(forbidden_anchor_y) == 2 and wall_anchor_positions:
        fy_min, fy_max = float(forbidden_anchor_y[0]), float(forbidden_anchor_y[1])
        in_zone = [y for y in wall_anchor_positions if fy_min <= y <= fy_max]
        constraints.append({
            "constraint": "Forbidden Anchor Zone",
            "value": f"{len(in_zone)}/{len(wall_anchor_positions)} in zone",
            "limit": f"y not in [{fy_min:.1f}, {fy_max:.1f}]",
            "margin": f"{len(wall_anchor_positions) - len(in_zone)} clear",
            "pct": (len(in_zone) / max(1, len(wall_anchor_positions)) * 100.0),
            "passed": len(in_zone) == 0,
        })
    mr = metrics.get("max_reach")
    tr = metrics.get("target_reach")
    cr = metrics.get("current_reach")
    if _is_valid_number(mr) and _is_valid_number(tr):
        mrf, trf = float(mr), float(tr)
        pct = (mrf / trf * 100.0) if trf > 0 else 0.0
        constraints.append({
            "constraint": "Reach Target",
            "value": f"{_fmt_val(mrf, 2)} m",
            "limit": f">= {_fmt_val(trf, 2)} m",
            "margin": f"{mrf - trf:+.3f} m",
            "pct": pct,
            "passed": mrf >= trf,
        })
    elif _is_valid_number(cr) and _is_valid_number(tr):
        crf, trf = float(cr), float(tr)
        pct = (crf / trf * 100.0) if trf > 0 else 0.0
        constraints.append({
            "constraint": "Reach Target",
            "value": f"{_fmt_val(crf, 2)} m",
            "limit": f">= {_fmt_val(trf, 2)} m",
            "margin": f"{crf - trf:+.3f} m",
            "pct": pct,
            "passed": crf >= trf,
        })
    mty = metrics.get("min_tip_y")
    mth = metrics.get("min_tip_height")
    if _is_valid_number(mty) and _is_valid_number(mth):
        mtyf, mthf = float(mty), float(mth)
        margin = mtyf - mthf
        constraints.append({
            "constraint": "Tip Height",
            "value": f"{_fmt_val(mtyf, 3)} m",
            "limit": f">= {_fmt_val(mthf, 2)} m",
            "margin": f"{margin:+.3f} m",
            "pct": None,
            "passed": mtyf >= mthf,
        })
    global_peak_at = metrics.get("global_peak_anchor_torque")
    at_limit = metrics.get("max_anchor_torque_limit")
    if _is_valid_number(global_peak_at) and _is_valid_number(at_limit) and float(at_limit) > 0:
        atf, atlf = float(global_peak_at), float(at_limit)
        pct = atf / atlf * 100.0
        constraints.append({
            "constraint": "Anchor Torque",
            "value": f"{_fmt_val(atf, 1)} N.m",
            "limit": f"<= {_fmt_val(atlf, 1)} N.m",
            "margin": f"{atlf - atf:+.1f} N.m",
            "pct": pct,
            "passed": atf <= atlf,
        })
    global_peak_af = metrics.get("global_peak_anchor_force")
    af_limit = metrics.get("max_anchor_force_limit")
    if _is_valid_number(global_peak_af) and _is_valid_number(af_limit) and float(af_limit) > 0:
        aff, aflf = float(global_peak_af), float(af_limit)
        pct = aff / aflf * 100.0
        constraints.append({
            "constraint": "Anchor Force",
            "value": f"{_fmt_val(aff, 1)} N",
            "limit": f"<= {_fmt_val(aflf, 1)} N",
            "margin": f"{aflf - aff:+.1f} N",
            "pct": pct,
            "passed": aff <= aflf,
        })
    global_peak_it = metrics.get("global_peak_internal_torque")
    it_limit = metrics.get("max_internal_torque_limit")
    if _is_valid_number(global_peak_it) and _is_valid_number(it_limit) and float(it_limit) > 0:
        itf, itlf = float(global_peak_it), float(it_limit)
        pct = itf / itlf * 100.0
        constraints.append({
            "constraint": "Internal Torque",
            "value": f"{_fmt_val(itf, 1)} N.m",
            "limit": f"<= {_fmt_val(itlf, 1)} N.m",
            "margin": f"{itlf - itf:+.1f} N.m",
            "pct": pct,
            "passed": itf <= itlf,
        })
    global_peak_ifield = metrics.get("global_peak_internal_force")
    if_limit = metrics.get("max_internal_force_limit")
    if _is_valid_number(global_peak_ifield) and _is_valid_number(if_limit) and float(if_limit) > 0:
        iff, iflf = float(global_peak_ifield), float(if_limit)
        pct = iff / iflf * 100.0
        constraints.append({
            "constraint": "Internal Force",
            "value": f"{_fmt_val(iff, 1)} N",
            "limit": f"<= {_fmt_val(iflf, 1)} N",
            "margin": f"{iflf - iff:+.1f} N",
            "pct": pct,
            "passed": iff <= iflf,
        })
    anchor_broken = metrics.get("anchor_broken")
    jc = metrics.get("joint_count")
    ijc = metrics.get("initial_joint_count")
    if _is_valid_number(jc) and _is_valid_number(ijc):
        jcf, ijcf = int(jc), int(ijc)
        lost = ijcf - jcf
        constraints.append({
            "constraint": "Structural Integrity",
            "value": "all joints intact" if lost == 0 else f"{lost} broken",
            "limit": "no joint breaks",
            "margin": "0 broken" if lost == 0 else f"{lost} broken",
            "pct": (lost / max(1, ijcf) * 100.0),
            "passed": not anchor_broken,
        })
    load_hold_time = metrics.get("load_hold_time")
    load2_hold_time = metrics.get("load2_hold_time")
    load_duration = metrics.get("load_duration")
    if _is_valid_number(load_hold_time) and _is_valid_number(load_duration):
        lht = float(load_hold_time)
        ld = float(load_duration)
        passed = lht >= ld
        pct = (lht / ld * 100.0) if ld > 0 else 0.0
        constraints.append({
            "constraint": "L1 Load Hold Duration",
            "value": f"{_fmt_val(lht, 2)} s",
            "limit": f">= {ld:.1f} s",
            "margin": f"{lht - ld:+.2f} s",
            "pct": pct,
            "passed": passed,
        })
    if _is_valid_number(load2_hold_time) and _is_valid_number(load_duration):
        l2ht = float(load2_hold_time)
        ld = float(load_duration)
        passed = l2ht >= ld
        pct = (l2ht / ld * 100.0) if ld > 0 else 0.0
        constraints.append({
            "constraint": "L2 Load Hold Duration",
            "value": f"{_fmt_val(l2ht, 2)} s",
            "limit": f">= {ld:.1f} s",
            "margin": f"{l2ht - ld:+.2f} s",
            "pct": pct,
            "passed": passed,
        })
    reach_satisfied_initially = metrics.get("reach_satisfied_initially")
    if reach_satisfied_initially is not None:
        constraints.append({
            "constraint": "Initial Reach (pre-load)",
            "value": "satisfied" if reach_satisfied_initially else "not satisfied",
            "limit": f">= {metrics.get('target_reach', '?')} m before loads",
            "margin": "N/A",
            "pct": None,
            "passed": reach_satisfied_initially,
        })
    build_bounds = metrics.get("build_zone_bounds")
    if isinstance(build_bounds, dict):
        fr_str = str(metrics.get("failure_reason", "") or "")
        build_zone_violated = "outside build zone" in fr_str
        constraints.append({
            "constraint": "Build Zone",
            "value": "within bounds" if not build_zone_violated else "violation",
            "limit": (
                f"beams within [{_fmt_val(build_bounds.get('x_min'), 2)}, "
                f"{_fmt_val(build_bounds.get('x_max'), 2)}] x "
                f"[{_fmt_val(build_bounds.get('y_min'), 2)}, "
                f"{_fmt_val(build_bounds.get('y_max'), 2)}] m"
            ),
            "margin": "N/A",
            "pct": None,
            "passed": not build_zone_violated,
        })
    if not constraints:
        parts.append("No constraint data available.\n")
        return parts
    n_pass = 0
    n_fail = 0
    n_near = 0
    failed_items = []
    near_items = []
    for c in constraints:
        if c["passed"]:
            n_pass += 1
            pct = c.get("pct")
            if pct is not None and pct >= 70.0:
                n_near += 1
                near_items.append(c)
        else:
            n_fail += 1
            failed_items.append(c)
    parts.append(f"**Summary:** {n_pass} passed, {n_fail} failed, {n_near} near-limit (>70%).\n")
    if failed_items:
        parts.append("**Failed constraints:**\n")
        for c in failed_items:
            pct_str = f" ({_fmt_val(c['pct'], 1)}% of limit)" if c.get("pct") is not None else ""
            parts.append(f"  ❌ {c['constraint']}: {c['value']} | limit: {c['limit']} | margin: {c['margin']}{pct_str}")
        parts.append("")
    if near_items:
        parts.append("**Near-limit constraints (>70%):**\n")
        for c in near_items:
            parts.append(f"  ⚠ {c['constraint']}: {c['value']} | limit: {c['limit']} | margin: {c['margin']} ({_fmt_val(c['pct'], 1)}%)")
        parts.append("")
    return parts

_EVALUATOR_NUMERIC_KEYS = (
    "tip_x", "max_reach", "target_reach", "current_reach",
    "min_tip_y", "min_tip_height", "structure_mass", "max_structure_mass",
    "peak_joint_torque", "peak_joint_force",
    "max_anchor_torque_limit", "max_internal_torque_limit",
    "max_anchor_force_limit", "max_internal_force_limit",
    "peak_anchor_force", "peak_anchor_torque",
    "peak_internal_force", "peak_internal_torque",
    "global_peak_anchor_force", "global_peak_anchor_torque",
    "global_peak_internal_force", "global_peak_internal_torque",
    "load_hold_time", "load2_hold_time", "external_force_y",
    "step_count", "first_failure_step", "first_warning_step",
    "reach_tolerance", "joint_count", "initial_joint_count",
    "anchor_count", "max_anchors_limit", "max_anchor_points",
    "load_attach_time", "load_2_attach_time",
    "base_anchor_force", "base_anchor_torque",
    "base_internal_force", "base_internal_torque",
    "drop_height", "load_mass",

)

def _build_numerical_health(metrics: Dict[str, Any]) -> List[str]:
    parts: List[str] = []
    parts.append("### 6. Numerical Health\n")
    issues = []
    non_finite_keys = []
    for key in _EVALUATOR_NUMERIC_KEYS:
        v = metrics.get(key)
        if v is None:
            continue
        try:
            f = float(v)
            if not math.isfinite(f):
                non_finite_keys.append(f"{key}={v}")
        except (TypeError, ValueError):
            pass
    if non_finite_keys:
        issues.append(f"Non-finite values: {', '.join(non_finite_keys)}")
    observation_errors = metrics.get("joint_observation_error_count")
    if _is_valid_number(observation_errors) and int(observation_errors) > 0:
        detail = metrics.get("last_joint_observation_error")
        issues.append(
            f"Joint reaction telemetry failed {int(observation_errors)} time(s)"
            + (f"; last error: {detail}" if detail else "")
        )
    max_reach = metrics.get("max_reach")
    target_reach = metrics.get("target_reach")
    if _is_valid_number(max_reach) and _is_valid_number(target_reach) and float(target_reach) > 0:
        mr, tr = float(max_reach), float(target_reach)
        if mr > tr * 5:
            issues.append(f"Extreme reach: {mr:.2f} m is {mr/tr:.1f}x target — possible solver divergence")
    min_tip_y = metrics.get("min_tip_y")
    if _is_valid_number(min_tip_y) and float(min_tip_y) < -100.0:
        issues.append(f"Extreme tip drop: y = {float(min_tip_y):.2f} m — possible solver explosion")
    sm = metrics.get("structure_mass")
    if _is_valid_number(sm) and float(sm) < 0:
        issues.append(f"Negative structure mass: {float(sm):.2f} kg — invalid")
    global_peak_at = metrics.get("global_peak_anchor_torque")
    at_limit = metrics.get("max_anchor_torque_limit")
    if _is_valid_number(global_peak_at) and _is_valid_number(at_limit) and float(at_limit) > 0:
        if float(global_peak_at) > float(at_limit) * 10:
            issues.append(f"Anchor torque ({_fmt_val(global_peak_at, 1)} N.m) is "
                         f"{float(global_peak_at)/float(at_limit):.1f}x limit — extreme overload")
    global_peak_it = metrics.get("global_peak_internal_torque")
    it_limit = metrics.get("max_internal_torque_limit")
    if _is_valid_number(global_peak_it) and _is_valid_number(it_limit) and float(it_limit) > 0:
        if float(global_peak_it) > float(it_limit) * 10:
            issues.append(f"Internal torque ({_fmt_val(global_peak_it, 1)} N.m) is "
                         f"{float(global_peak_it)/float(it_limit):.1f}x limit — extreme overload")
    if issues:
        for issue in issues:
            parts.append(f"  ⚠ {issue}")
    else:
        parts.append("  All numeric fields finite, no anomalies. ✅")
    return parts

def format_task_metrics(metrics: Dict[str, Any]) -> List[str]:
    if not metrics:
        return ["**No metrics available.**"]
    report: List[str] = []
    success = bool(metrics.get("success", False))
    failed = bool(metrics.get("failed", False))
    reason = metrics.get("failure_reason")
    if success:
        report.append("## Outcome: SUCCESS")
    elif failed:
        report.append(f"## Outcome: FAILED — {reason or 'evaluator reported failure'}")
    else:
        report.append("## Outcome: IN PROGRESS")
    report.extend(_build_constraint_profile(metrics))
    report.extend(_build_spatial_diagnostics(metrics))
    report.extend(_build_temporal_chronology(metrics))
    report.extend(_build_load_distribution(metrics))
    report.extend(_build_energy_context(metrics))
    report.extend(_build_numerical_health(metrics))
    return report


def get_improvement_suggestions(
    metrics: Dict[str, Any],
    score: float,
    success: bool,
    failed: bool,
    failure_reason: str | None = None,
    error: str | None = None,
) -> List[str]:
    if error:
        return ["- Code execution failed. Review the error details above."]
    return []
