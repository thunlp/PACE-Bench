import math

import sys as _sys

from typing import Dict, Any, List

def _safe_float(v: Any, default: float = 0.0) -> float:
    if v is None:
        return default
    try:
        f = float(v)
        return f if math.isfinite(f) else default
    except (TypeError, ValueError):
        return default

def _div(a: float, b: float, default: float = 0.0) -> float:
    if not math.isfinite(a) or not math.isfinite(b) or abs(b) < 1e-12:
        return default
    return a / b

def _pct(a: float, b: float) -> float:
    return _div(a, b, 0.0) * 100.0

_STATE_KEY = "__k06_feedback_moment_state__"

def _fresh_state(step: int) -> dict:
    return {
        'step_count': step,
        'cleaning_pct': None,
        'motor_energy': None,
        'num_events': 0,
        'structure_min_x': None,
        'structure_max_x': None,
        'structure_min_y': None,
        'structure_max_y': None,
        'torque_cap': None,
        'constraint_statuses': {},
        'peak_velocity': None,
        '_prev': None,
    }

def _get_state() -> dict:
    if _STATE_KEY not in _sys.modules:
        holder = type(_sys)('_k06_persist')
        holder.state = _fresh_state(-1)
        _sys.modules[_STATE_KEY] = holder
    return _sys.modules[_STATE_KEY].state

def _snapshot_state(metrics: Dict[str, Any]) -> dict:
    s = {
        'step_count': int(metrics.get('step_count', 0)),
        'cleaning_pct': metrics.get('cleaning_percentage'),
        'motor_energy': metrics.get('motor_energy_joules'),
        'num_events': len(metrics.get('temporal_events') or []),
        'peak_velocity': metrics.get('peak_body_velocity'),
    }
    snap = metrics.get('forensic_snapshot') or {}
    s['structure_min_x'] = snap.get('structure_min_x')
    s['structure_max_x'] = snap.get('structure_max_x')
    s['structure_min_y'] = snap.get('structure_min_y')
    s['structure_max_y'] = snap.get('structure_max_y')
    ta = metrics.get('torque_adequacy') or {}
    s['torque_cap'] = ta.get('torque_cap_nm')
    profile = metrics.get('constraint_profile') or []
    s['constraint_statuses'] = {c.get('constraint', ''): c.get('status', '') for c in profile}
    return s

def _detect_and_update_moment(metrics: Dict[str, Any]) -> int:
    state = _get_state()
    step = int(metrics.get('step_count', 0))
    prev_step = state.get('step_count', -1)
    if prev_step < 0 or step <= prev_step:
        new_state = _snapshot_state(metrics)
        new_state['_prev'] = None
        state.clear()
        state.update(new_state)
        return 1
    prev = state.copy()
    new_state = _snapshot_state(metrics)
    new_state['_prev'] = prev
    state.clear()
    state.update(new_state)
    return 2

def _prev() -> dict:
    return _get_state().get('_prev') or {}

def _section_header(title: str) -> str:
    return f"\n## {title}\n"

def _format_temporal_chronology(metrics: Dict[str, Any], moment_num: int) -> List[str]:
    lines: List[str] = []
    events = metrics.get('temporal_events') or []
    if not events:
        lines.append("No temporal events recorded.")
        return lines
    if moment_num > 1:
        prev_count = _prev().get('num_events', 0)
        if len(events) == prev_count:
            lines.append("No new events since previous moment.")
            return lines
        new_events = events[prev_count:]
        if not new_events:
            lines.append("No new events since previous moment.")
            return lines
        lines.append("New events since previous moment:")
        lines.append("")
        events = new_events
    if moment_num <= 1:
        lines.append("Ordered failure timeline (earliest → latest):")
        lines.append("")
    for i, ev in enumerate(events, 1):
        step = ev.get('step', '?')
        event_name = ev.get('event', 'unknown').replace('_', ' ')
        detail = ev.get('detail', '')
        severity = ev.get('severity', '')
        tag = ''
        if severity == 'critical':
            tag = ' [CRITICAL]'
        elif severity == 'elevated':
            tag = ' [ELEVATED]'
        lines.append(f"  {i}. Step {step} — {event_name}{tag}")
        if detail:
            lines.append(f"     {detail}")
    return lines

def _format_spatial_diagnostics(metrics: Dict[str, Any], moment_num: int) -> List[str]:
    lines: List[str] = []
    snap = metrics.get('forensic_snapshot') or {}
    min_x = snap.get('structure_min_x')
    max_x = snap.get('structure_max_x')
    min_y = snap.get('structure_min_y')
    max_y = snap.get('structure_max_y')
    if moment_num > 1 and _prev():
        px = _prev()
        dx = abs(_safe_float(min_x) - _safe_float(px.get('structure_min_x', min_x)))
        dy = abs(_safe_float(min_y) - _safe_float(px.get('structure_min_y', min_y)))
        if dx < 0.01 and dy < 0.01:
            lines.append("No significant spatial change since previous moment.")
            return lines
    if min_x is not None and max_x is not None and min_y is not None and max_y is not None:
        lines.append(f"Structure: x=[{min_x:.2f}, {max_x:.2f}]m  y=[{min_y:.2f}, {max_y:.2f}]m")
    x_min_m = snap.get('x_min_margin')
    x_max_m = snap.get('x_max_margin')
    y_min_m = snap.get('y_min_margin')
    y_max_m = snap.get('y_max_margin')
    bz_x = snap.get('build_zone_x', [0.0, 12.0])
    bz_y = snap.get('build_zone_y', [2.0, 10.0])
    margin_items = []
    label_map = {0: ('Left (x_min)', bz_x[0], 'x ≥'), 1: ('Right (x_max)', bz_x[1], 'x ≤'),
                 2: ('Bottom (y_min)', bz_y[0], 'y ≥'), 3: ('Top (y_max)', bz_y[1], 'y ≤')}
    vals = [x_min_m, x_max_m, y_min_m, y_max_m]
    for idx, (label, lim, prefix) in label_map.items():
        m = vals[idx]
        if m is not None:
            margin_items.append((label, m, lim, prefix))
    if margin_items:
        tightest = min(margin_items, key=lambda x: x[1])
        label, margin, lim, prefix = tightest
        if margin < 0:
            lines.append(f"Tightest build zone margin: VIOLATED {abs(margin):.3f}m ({label}, limit {prefix}{lim:.1f})")
        elif margin < 0.1:
            lines.append(f"Tightest build zone margin: {margin:.3f}m TIGHT ({label}, limit {prefix}{lim:.1f})")
        else:
            lines.append(f"Build zone: all margins ≥{margin:.3f}m (safe)")
    wiper_bottom_y = snap.get('wiper_bottom_y')
    particle_top_y = snap.get('particle_top_y')
    contact_gap = snap.get('particle_contact_gap')
    if contact_gap is not None:
        if contact_gap < 0:
            lines.append(f"Wiper-particle: INTERPENETRATION {abs(contact_gap):.3f}m (wiper at y={_safe_float(wiper_bottom_y, 0):.3f} below particle tops y={_safe_float(particle_top_y, 0):.3f})")
        else:
            lines.append(f"Wiper-particle: gap {contact_gap:.3f}m (wiper above particles)")
    ja = metrics.get('joint_angle_summary') or []
    if ja:
        entry = ja[0]
        angle_rad = entry.get('angle_rad')
        lower = entry.get('lower_limit_rad')
        upper = entry.get('upper_limit_rad')
        jidx = entry.get('joint_index', '?')
        if angle_rad is not None and lower is not None and upper is not None:
            angle_deg = math.degrees(angle_rad)
            lower_deg = math.degrees(lower)
            upper_deg = math.degrees(upper)
            range_deg = upper_deg - lower_deg
            utilization = abs(angle_rad) / max(abs(lower), abs(upper)) if max(abs(lower), abs(upper)) > 0 else 0
            tightest = min(angle_rad - lower, upper - angle_rad)
            line = f"Joint #{jidx}: {angle_deg:.1f}° [{lower_deg:.1f}°, {upper_deg:.1f}°] · {utilization*100:.1f}% utilized"
            if utilization < 0.3:
                line += " · significantly underutilized"
            lines.append(line)
    particle_positions = metrics.get('particle_positions') or []
    if particle_positions and min_x is not None and max_x is not None:
        uncovered = len([p for p in particle_positions if p[0] < min_x or p[0] > max_x])
        covered = len(particle_positions) - uncovered
        lines.append(f"Particle coverage: {covered}/{len(particle_positions)} within structure x-span [{min_x:.1f}, {max_x:.1f}]")
        if uncovered:
            lines.append(f"  {uncovered} particle(s) outside reach")
    return lines

def _format_load_distribution(metrics: Dict[str, Any], moment_num: int) -> List[str]:
    lines: List[str] = []
    ta = metrics.get('torque_adequacy') or {}
    snap = metrics.get('forensic_snapshot') or {}
    if ta.get('note') and 'no torque cap' in str(ta.get('note', '')):
        lines.append("No torque cap configured.")
        return lines
    torque_cap = ta.get('torque_cap_nm')
    if torque_cap is None:
        return lines
    if moment_num > 1 and _prev():
        prev_tc = _prev().get('torque_cap')
        if prev_tc is not None and abs(_safe_float(torque_cap) - _safe_float(prev_tc)) < 0.001:
            lines.append("Load profile unchanged from previous moment.")
            return lines
    parts = [f"Torque cap: {torque_cap:.2f} N·m"]
    lines.append(" · ".join(parts))
    torque_capped = snap.get('torque_capped', False)
    torque_requested = snap.get('last_torque_requested')
    if torque_capped and torque_cap is not None and torque_requested is not None:
        over_request = torque_requested - torque_cap
        lines.append(f"Overspecification: requested {torque_requested:.1f} N·m, capped at {torque_cap:.1f} N·m")
    return lines

def _format_energy_flow(metrics: Dict[str, Any], moment_num: int) -> List[str]:
    lines: List[str] = []
    motor_energy = metrics.get('motor_energy_joules')
    if motor_energy is None or motor_energy == 0.0:
        lines.append("Motor energy: none recorded.")
        return lines
    if moment_num > 1 and _prev():
        prev_energy = _prev().get('motor_energy')
        if prev_energy is not None:
            delta = motor_energy - prev_energy
            lines.append(f"Motor energy: {motor_energy:.2f} J" + (f" (Δ {delta:+.2f} J from previous moment)" if abs(delta) > 0.001 else " (unchanged)"))
            return lines
    lines.append(f"Motor energy: {motor_energy:.2f} J")
    return lines

def _format_constraint_profile(metrics: Dict[str, Any], moment_num: int) -> List[str]:
    lines: List[str] = []
    profile = metrics.get('constraint_profile') or []
    if not profile:
        profile = _fallback_constraint_profile(metrics)
    if not profile:
        lines.append("No constraint data available.")
        return lines
    if moment_num > 1 and _prev():
        prev_statuses = _prev().get('constraint_statuses', {})
        changed = []
        for c in profile:
            name = c.get('constraint', '?')
            status = c.get('status', '?')
            prev_s = prev_statuses.get(name, status)
            if status != prev_s:
                changed.append(c)
        if not changed:
            lines.append("Constraint statuses unchanged from previous moment.")
            return lines
        lines.append("Changed since previous moment:")
        lines.append("")
        lines.append("| Constraint | Status | Current | Limit |")
        lines.append("|------------|--------|---------|-------|")
        for c in changed:
            name = c.get('constraint', '?')
            status = c.get('status', '?')
            current = c.get('current', '—')
            limit = c.get('limit', '—')
            prev_s = prev_statuses.get(name, '?')
            if 'FAIL' in status:
                status_disp = '**FAIL**'
            elif status in ('CAPPED', 'NEAR-LIMIT'):
                status_disp = f'*{status}*'
            else:
                status_disp = status
            lines.append(f"| {name} | {prev_s}→{status_disp} | {current} | {limit} |")
        return lines
    actionable = []
    all_pass = []
    for c in profile:
        status = c.get('status', '')
        status_clean = status.replace('*', '').replace('**', '')
        utilization = c.get('utilization_pct')
        if status_clean in ('FAIL', 'CAPPED', 'NEAR-LIMIT'):
            actionable.append(c)
        elif utilization is not None and utilization > 50.0:
            actionable.append(c)
        else:
            all_pass.append(c)
    if not actionable:
        lines.append(f"All {len(all_pass)} constraints PASS with safe margins.")
        return lines
    lines.append("| Constraint | Status | Current | Limit | Margin |")
    lines.append("|------------|--------|---------|-------|--------|")
    for c in actionable:
        name = c.get('constraint', '?')
        status = c.get('status', '?')
        current = c.get('current', '—')
        limit = c.get('limit', '—')
        margin = c.get('margin', '—')
        if 'FAIL' in status:
            status_disp = '**FAIL**'
        elif status in ('CAPPED', 'NEAR-LIMIT'):
            status_disp = f'*{status}*'
        else:
            status_disp = status
        lines.append(f"| {name} | {status_disp} | {current} | {limit} | {margin} |")
    if all_pass:
        pass_names = [c.get('constraint', '?') for c in all_pass]
        lines.append(f"  + {len(all_pass)} other constraint(s) PASS: {', '.join(pass_names)}")
    fails = [c for c in profile if 'FAIL' in c.get('status', '')]
    near = [c for c in profile if c.get('status', '').replace('*', '') in ('NEAR-LIMIT', 'CAPPED')]
    if fails or near:
        passes = len(profile) - len(fails) - len(near)
        summary_parts = []
        if passes:
            summary_parts.append(f"{passes} PASS")
        if near:
            summary_parts.append(f"{len(near)} NEAR-LIMIT/CAPPED")
        if fails:
            summary_parts.append(f"{len(fails)} FAIL")
        lines.append(f"  Summary: {', '.join(summary_parts)}" + (f". Failing: {', '.join(c.get('constraint','?') for c in fails)}" if fails else ""))
    return lines

def _fallback_constraint_profile(metrics: Dict[str, Any]) -> List[dict]:
    profile = []
    mass = metrics.get('structure_mass')
    max_mass = metrics.get('max_structure_mass')
    if mass is not None and max_mass is not None:
        m, mm = _safe_float(mass), _safe_float(max_mass)
        if mm > 0:
            margin = mm - m
            profile.append({
                'constraint': 'Mass budget',
                'status': 'PASS' if margin > 0 else 'FAIL',
                'current': f'{m:.3f} kg',
                'limit': f'{mm:.2f} kg',
                'margin': f'{margin:+.3f} kg',
                'utilization_pct': round(m / mm * 100.0, 1) if mm > 0 else 0.0,
                'phase': 'build-time',
            })
    clean_pct = metrics.get('cleaning_percentage')
    max_res = metrics.get('max_residual_percent')
    if clean_pct is not None and max_res is not None:
        cp = _safe_float(clean_pct)
        mr = _safe_float(max_res)
        required = 100.0 - mr
        mrg = cp - required
        profile.append({
            'constraint': 'Cleaning requirement',
            'status': 'PASS' if mrg >= 0 else 'FAIL',
            'current': f'{cp:.1f}%',
            'limit': f'{required:.0f}%',
            'margin': f'{mrg:+.1f}%',
            'phase': 'runtime',
        })
    steps = metrics.get('step_count')
    min_steps = metrics.get('min_simulation_steps_required')
    if steps is not None and min_steps is not None:
        s, ms = int(steps), int(min_steps)
        mrg = s - ms
        profile.append({
            'constraint': 'Motion duration',
            'status': 'PASS' if mrg >= 0 else 'FAIL',
            'current': f'{s} steps',
            'limit': f'{ms} steps',
            'margin': f'{mrg:+d} steps',
            'phase': 'runtime',
        })
    snap = metrics.get('forensic_snapshot') or {}
    if snap.get('torque_capped'):
        profile.append({
            'constraint': 'Motor torque limit',
            'status': 'CAPPED',
            'current': f"req. {snap.get('last_torque_requested', '?')} N·m",
            'limit': f"{snap.get('max_motor_torque_cap', '?')} N·m",
            'margin': 'over-requested',
            'phase': 'runtime',
        })
    return profile

def _format_numerical_health(metrics: Dict[str, Any]) -> List[str]:
    lines: List[str] = []
    snap = metrics.get('forensic_snapshot') or {}
    nan_detected = snap.get('numerical_nan_detected', False)
    peak_vel = metrics.get('peak_body_velocity')
    status_parts = []
    if nan_detected:
        status_parts.append("NaN/Inf DETECTED — potential numerical instability")
    else:
        status_parts.append("clean")
    if peak_vel is not None:
        pv = _safe_float(peak_vel)
        if pv > 1000:
            status_parts.append(f"peak velocity {pv:.1f} m/s CRITICAL")
        elif pv > 100:
            status_parts.append(f"peak velocity {pv:.1f} m/s WARNING")
        else:
            status_parts.append(f"peak velocity {pv:.2f} m/s")
    lines.append("Numerical health: " + " · ".join(status_parts))
    if nan_detected:
        lines.append("  Simulation may be numerically unstable.")
    vel_warnings = metrics.get('body_velocity_warnings') or []
    if vel_warnings:
        lines.append(f"  Velocity warnings: {len(vel_warnings)} recorded")
    mass = metrics.get('structure_mass')
    if mass is not None:
        m = _safe_float(mass)
        if m == 0.0:
            lines.append("  Mass anomaly: structure mass = 0.00 kg — structure may have disintegrated")
    for key in ('cleaning_percentage', 'structure_mass', 'wiper_x', 'wiper_y'):
        if key in metrics and metrics[key] is not None:
            try:
                if not math.isfinite(float(metrics[key])):
                    lines.append(f"  Non-finite metric: {key}={metrics[key]}")
            except (TypeError, ValueError):
                lines.append(f"  Non-numeric metric: {key}={metrics[key]}")
    return lines

def _format_execution_summary(metrics: Dict[str, Any]) -> List[str]:
    lines: List[str] = []
    if metrics.get('error'):
        lines.append(f"**Terminal state**: ERROR — {metrics['error']}")
        return lines
    failed = metrics.get('failed', False)
    success = metrics.get('success', False)
    failure_reason = metrics.get('failure_reason')
    if success:
        outcome = "SUCCESS — all criteria met"
    elif failed:
        outcome = f"FAILED" + (f" — {failure_reason}" if failure_reason else "")
    else:
        outcome = "IN PROGRESS"
    parts = [f"**State**: {outcome}"]
    clean_pct = metrics.get('cleaning_percentage')
    parts.append(f"cleaning: {clean_pct:.1f}%" if clean_pct is not None else "cleaning: —")
    particles_removed = metrics.get('particles_removed')
    initial_count = metrics.get('initial_particle_count')
    if particles_removed is not None and initial_count is not None:
        parts.append(f"removed: {particles_removed}/{initial_count}")
    mass = metrics.get('structure_mass')
    max_mass = metrics.get('max_structure_mass')
    if mass is not None:
        m = _safe_float(mass)
        mm = _safe_float(max_mass) if max_mass is not None else None
        if mm is not None and mm > 0:
            parts.append(f"mass: {m:.3f}/{mm:.2f} kg ({_pct(m, mm):.1f}%)")
        else:
            parts.append(f"mass: {m:.3f} kg")
    steps = metrics.get('step_count')
    min_steps = metrics.get('min_simulation_steps_required')
    if steps is not None:
        if min_steps is not None:
            parts.append(f"steps: {steps}/{min_steps}")
        else:
            parts.append(f"steps: {steps}")
    lines.append(" · ".join(parts))
    rra = metrics.get('removal_rate_analysis') or {}
    if rra.get('saturated'):
        lines.append(f"  Removal saturated at step ~{rra.get('saturation_step', '?')} ({rra.get('pct_removed_at_saturation', '?')}% removed)")
    return lines

def _format_delta(metrics: Dict[str, Any], moment_num: int) -> List[str]:
    lines: List[str] = []
    prev = _prev()
    if not prev:
        return lines
    changes = []
    prev_step = int(prev.get('step_count', 0))
    curr_step = int(metrics.get('step_count', 0))
    if curr_step != prev_step:
        changes.append(f"Step {prev_step}→{curr_step} (Δ {curr_step - prev_step:+d})")
    curr_score = _safe_float(metrics.get('score', 0))
    prev_score = _safe_float(prev.get('score', 0))
    if abs(curr_score - prev_score) > 0.5:
        changes.append(f"Score {prev_score:.1f}→{curr_score:.1f}")
    curr_clean = metrics.get('cleaning_percentage')
    prev_clean = prev.get('cleaning_pct')
    if curr_clean is not None and prev_clean is not None:
        diff = curr_clean - prev_clean
        if abs(diff) > 0.001:
            changes.append(f"Cleaning {prev_clean:.1f}%→{curr_clean:.1f}%")
    profile = metrics.get('constraint_profile') or []
    prev_statuses = prev.get('constraint_statuses', {})
    for c in profile:
        name = c.get('constraint', '')
        status = c.get('status', '')
        prev_s = prev_statuses.get(name, status)
        if status != prev_s:
            clean_status = status.replace('*', '').replace('**', '')
            clean_prev = prev_s.replace('*', '').replace('**', '') if prev_s else '?'
            changes.append(f"'{name}' {clean_prev}→{clean_status}")
    if changes:
        lines.append(f"Delta: {' · '.join(changes)}")
    snap = metrics.get('forensic_snapshot') or {}
    dx = abs(_safe_float(snap.get('structure_min_x')) - _safe_float(prev.get('structure_min_x', snap.get('structure_min_x'))))
    dy = abs(_safe_float(snap.get('structure_min_y')) - _safe_float(prev.get('structure_min_y', snap.get('structure_min_y'))))
    if not changes and dx < 0.01 and dy < 0.01:
        lines.append("No significant changes since previous moment.")
    elif not changes:
        lines.append("No metric changes since previous moment.")
    return lines

def format_task_metrics(metrics: Dict[str, Any]) -> List[str]:
    if not metrics:
        return ["No metrics data available."]
    parts: List[str] = []
    if metrics.get('error'):
        parts.append(f"**Evaluation error**: {metrics['error']}")
        return parts
    moment_num = 1
    parts.append(_section_header("Execution Summary"))
    parts.extend(_format_execution_summary(metrics))
    events = metrics.get('temporal_events') or []
    if events or moment_num <= 1:
        parts.append(_section_header("1. Temporal Events"))
        parts.extend(_format_temporal_chronology(metrics, moment_num))
    parts.append(_section_header("2. Spatial"))
    parts.extend(_format_spatial_diagnostics(metrics, moment_num))
    ta = metrics.get('torque_adequacy') or {}
    snap = metrics.get('forensic_snapshot') or {}
    has_torque_data = ta.get('torque_cap_nm') is not None or snap.get('max_motor_torque_cap') is not None
    if has_torque_data and not ('no torque cap' in str(ta.get('note', '')) and not ta.get('torque_cap_nm')):
        parts.append(_section_header("3. Load"))
        parts.extend(_format_load_distribution(metrics, moment_num))
    motor_energy = metrics.get('motor_energy_joules')
    if moment_num <= 1 or (motor_energy is not None and motor_energy > 0):
        parts.append(_section_header("4. Energy"))
        parts.extend(_format_energy_flow(metrics, moment_num))
    parts.append(_section_header("5. Constraints"))
    parts.extend(_format_constraint_profile(metrics, moment_num))
    if moment_num <= 1:
        parts.append(_section_header("6. Health"))
        parts.extend(_format_numerical_health(metrics))
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
