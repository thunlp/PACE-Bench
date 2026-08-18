import math

_ZONE_CENTERS_FALLBACK = {"A": (2.0, 2.0), "B": (4.95, 3.2), "C": (8.0, 2.0)}

_ZONE_EXTENTS_FALLBACK = {
    "A": (1.5, 2.5, 1.5, 2.5),
    "B": (4.25, 5.65, 2.8, 3.6),
    "C": (7.5, 8.5, 1.5, 2.5),

}

def _max_agent_force(sandbox):
    del sandbox
    return 50.0

def _zone_center(sandbox, name: str):
    del sandbox
    return _ZONE_CENTERS_FALLBACK[name]

def is_inside_zone(sandbox, x, y, zone_name):
    del sandbox
    x_min, x_max, y_min, y_max = _ZONE_EXTENTS_FALLBACK[zone_name]
    return x_min <= x <= x_max and y_min <= y <= y_max

HOLD_RADIUS = 0.75

APPROACH_RADIUS = 2.0

GAIN_APPROACH = 6.0

GAIN_NORMAL = 15.0

HOLD_GAIN = 2.5

HOLD_DAMP = 5.5

APPROACH_DAMP = 1.8

RAMP_X_LO, RAMP_X_HI = 3.5, 6.0

RAMP_Y_TARGET = 3.5

RAMP_X_FRAC = 0.3

RAMP_Y_GAIN = 48.0

_step_when_a_triggered = [None]

def build_agent(sandbox):
    _step_when_a_triggered[0] = None
    return sandbox.get_agent_body()

def agent_action(sandbox, agent_body, step_count):
    bx = float(sandbox.get_barrier_x())
    mf = _max_agent_force(sandbox)
    triggered = sandbox.get_triggered_switches()
    if triggered and "A" in triggered and _step_when_a_triggered[0] is None:
        _step_when_a_triggered[0] = step_count
    next_switch = sandbox.get_next_required_switch()
    cooldown = sandbox.get_cooldown_remaining()
    if next_switch is None:
        vx, vy = sandbox.get_agent_velocity()
        sandbox.apply_agent_force(-HOLD_DAMP * 2.0 * vx, -HOLD_DAMP * 2.0 * vy)
        return
    if cooldown > 0:
        vx, vy = sandbox.get_agent_velocity()
        sandbox.apply_agent_force(-HOLD_DAMP * 2.0 * vx, -HOLD_DAMP * 2.0 * vy)
        return
    if next_switch == "B" and _step_when_a_triggered[0] is not None:
        delay = int(sandbox.get_barrier_delay_steps())
        steps_since_a = step_count - _step_when_a_triggered[0]
        if steps_since_a < delay:
            tx, ty = _zone_center(sandbox, "A")
            x, y = sandbox.get_agent_position()
            vx, vy = sandbox.get_agent_velocity()
            dx, dy = tx - x, ty - y
            sandbox.apply_agent_force(15.0 * dx - HOLD_DAMP * vx, 15.0 * dy - HOLD_DAMP * vy)
            return
    tx, ty = _zone_center(sandbox, next_switch)
    x, y = sandbox.get_agent_position()
    vx, vy = sandbox.get_agent_velocity()
    dx = tx - x
    dy = ty - y
    dist = math.sqrt(dx * dx + dy * dy)
    in_ramp = RAMP_X_LO <= x <= RAMP_X_HI
    use_ramp = in_ramp and (next_switch == "B" or (next_switch == "C" and (x < bx or y > 2.4)))
    on_flat_for_c = (y <= 2.6 or x >= 6.4) and not (3.5 <= x < bx)
    if next_switch == "C" and x < 7.5 and dist > 0.2 and on_flat_for_c:
        fx = mf * 0.98 if dx > 0 else -mf * 0.4
        fy = -HOLD_DAMP * vy
        sandbox.apply_agent_force(fx, fy)
        return
    if dist < 1e-6:
        sandbox.apply_agent_force(-HOLD_DAMP * vx, -HOLD_DAMP * vy)
        return
    inside = is_inside_zone(sandbox, x, y, next_switch)
    if inside or dist < HOLD_RADIUS:
        speed = math.sqrt(vx * vx + vy * vy)
        if next_switch == "C" or speed > 0.1:
            fx = 10.0 * dx - HOLD_DAMP * 8.0 * vx
            fy = 10.0 * dy - HOLD_DAMP * 8.0 * vy
        else:
            fx = HOLD_GAIN * 10.0 * dx - HOLD_DAMP * 2.0 * vx
            fy = HOLD_GAIN * 10.0 * dy - HOLD_DAMP * 2.0 * vy
        mag = math.sqrt(fx * fx + fy * fy)
        if mag > mf:
            fx, fy = fx * mf / mag, fy * mf / mag
        sandbox.apply_agent_force(fx, fy)
        return
    if use_ramp:
        if next_switch == "B" or (next_switch == "C" and x < bx):
            y_target = 3.3
            fx = min(mf * 0.6, 30.0) * (1.0 if dx > 0 else -0.5) - APPROACH_DAMP * 2.0 * vx
            fy = RAMP_Y_GAIN * (y_target - y) - APPROACH_DAMP * 2.0 * vy
        else:
            y_target = 2.0
            dy_local = y_target - y
            if y > 3.0:
                fx = mf * 0.98 if dx > 0 else -mf * 0.3
                y_target_local = 3.9 if x < 6.4 else 2.0
                fy = 60.0 * (y_target_local - y) - APPROACH_DAMP * vy
            else:
                fx = min(mf * 0.95, 47.0) * (1.0 if dx > 0 else 0.5) - APPROACH_DAMP * 2.0 * vx
                fy = RAMP_Y_GAIN * dy_local - APPROACH_DAMP * 2.0 * vy
        mag = math.sqrt(fx * fx + fy * fy)
        if mag > mf:
            fx, fy = fx * mf / mag, fy * mf / mag
        sandbox.apply_agent_force(fx, fy)
        return
    if dist < APPROACH_RADIUS:
        force_mag = min(mf * 0.5, GAIN_APPROACH * 2.0 * dist)
        ux, uy = (dx / dist, dy / dist) if dist > 1e-6 else (0, 0)
        fx = force_mag * ux - APPROACH_DAMP * 3.0 * vx
        fy = force_mag * uy - APPROACH_DAMP * 3.0 * vy
        mag = math.sqrt(fx * fx + fy * fy)
        if mag > mf:
            fx, fy = fx * mf / mag, fy * mf / mag
        sandbox.apply_agent_force(fx, fy)
        return
    gain = GAIN_NORMAL * 0.8 if next_switch == "B" else GAIN_NORMAL
    force_mag = min(mf, gain * dist)
    ux, uy = (dx / dist, dy / dist) if dist > 1e-6 else (0, 0)
    sandbox.apply_agent_force(force_mag * ux - 2.0 * vx, force_mag * uy - 2.0 * vy)

def build_agent_stage_1(sandbox):
    body = sandbox.get_agent_body()
    body.s1_timing = {
        "phase": "approach_a",
        "a": None,
        "b": None,
        "b_crossed": False,
    }
    return body

def agent_action_stage_1(sandbox, agent_body, step_count):
    bx = float(sandbox.get_barrier_x())
    triggered = sandbox.get_triggered_switches()
    timing = agent_body.s1_timing
    if "A" in triggered and timing["a"] is None:
        timing["a"] = step_count
    if "B" in triggered and timing["b"] is None:
        timing["b"] = step_count
    next_switch = sandbox.get_next_required_switch()
    cooldown = sandbox.get_cooldown_remaining()
    x, y = sandbox.get_agent_position()
    vx, vy = sandbox.get_agent_velocity()
    if next_switch is None:
        sandbox.apply_agent_force(-6.0 * vx, -6.0 * vy)
        return
    if cooldown > 0:
        hold_name = "A" if next_switch == "B" else "B"
        if hold_name == "B":
            tx, ty = 4.82, 3.50
            fx = 8.0 * (tx - x) - 12.0 * vx
            fy = 30.0 + 8.0 * (ty - y) - 12.0 * vy
            limit = 34.0
        else:
            tx, ty = _zone_center(sandbox, hold_name)
            fx = 5.5 * (tx - x) - 7.0 * vx
            fy = 5.5 * (ty - y) - 7.0 * vy
            limit = 7.5
        mag = math.hypot(fx, fy)
        if mag > limit:
            fx *= limit / mag
            fy *= limit / mag
        sandbox.apply_agent_force(fx, fy)
        return
    inside = is_inside_zone(sandbox, x, y, next_switch)
    if inside:
        if next_switch == "B":
            tx, ty = 4.82, 3.50
            fx = 8.0 * (tx - x) - 12.0 * vx
            fy = 30.0 + 8.0 * (ty - y) - 12.0 * vy
            limit = 34.0
        else:
            tx, ty = _zone_center(sandbox, next_switch)
            fx = 4.0 * (tx - x) - 8.5 * vx
            fy = 4.0 * (ty - y) - 8.5 * vy
            limit = 7.5
        mag = math.hypot(fx, fy)
        if mag > limit:
            fx *= limit / mag
            fy *= limit / mag
        sandbox.apply_agent_force(fx, fy)
        return
    if next_switch == "A":
        if x < 1.38:
            tx, ty = 1.38, 2.0
        else:
            tx, ty = 2.0, 2.0
    elif next_switch == "B":
        if x > 4.72 and y > 4.12:
            timing["b_crossed"] = True
        if not timing["b_crossed"] and y < 4.28:
            tx, ty = 4.10, 4.48
        elif not timing["b_crossed"]:
            tx, ty = 4.88, 4.45
        else:
            tx, ty = 4.82, 3.50
    else:
        if x < bx + 0.55 and y < 4.35:
            tx, ty = 5.05, 4.55
        elif x < bx + 0.45:
            tx, ty = bx + 0.55, 4.45
        elif x < 7.25:
            tx, ty = 7.35, 2.55
        else:
            tx, ty = 8.0, 2.0
    fx = 32.0 * (tx - x) - 15.0 * vx
    fy = 38.0 * (ty - y) - 15.0 * vy
    if next_switch == "B" and not timing["b_crossed"]:
        fy += 30.0
    mag = math.hypot(fx, fy)
    if mag > 46.0:
        fx *= 46.0 / mag
        fy *= 46.0 / mag
    sandbox.apply_agent_force(fx, fy)

_step_when_a_triggered_s2 = [None]
_s2_gate_crossed = [False]
_s2_c_charged = [False]

def build_agent_stage_2(sandbox):
    _step_when_a_triggered_s2[0] = None
    _s2_gate_crossed[0] = False
    _s2_c_charged[0] = False
    return sandbox.get_agent_body()

def agent_action_stage_2(sandbox, agent_body, step_count):
    bx = float(sandbox.get_barrier_x())
    delay_barrier = int(sandbox.get_barrier_delay_steps())
    mf = _max_agent_force(sandbox)
    SPEED_CAP_S2 = 0.05
    triggered = sandbox.get_triggered_switches()
    if triggered and triggered[0] == "A" and _step_when_a_triggered_s2[0] is None:
        _step_when_a_triggered_s2[0] = step_count
    next_switch = sandbox.get_next_required_switch()
    cooldown = sandbox.get_cooldown_remaining()
    x, y = sandbox.get_agent_position()
    vx, vy = sandbox.get_agent_velocity()
    if next_switch is None:
        vx, vy = sandbox.get_agent_velocity()
        sandbox.apply_agent_force(-HOLD_DAMP * vx, -HOLD_DAMP * vy)
        return
    if cooldown > 0:
        vx, vy = sandbox.get_agent_velocity()
        sandbox.apply_agent_force(-HOLD_DAMP * vx, -HOLD_DAMP * vy)
        return
    if next_switch == "B" and not _s2_gate_crossed[0]:
        if x > 4.72 and y > 4.12:
            _s2_gate_crossed[0] = True
        else:
            tx, ty = ((4.10, 4.48) if y < 4.28 else (4.88, 4.45))
            fx = 32.0 * (tx - x) - 15.0 * vx
            fy = 38.0 * (ty - y) - 15.0 * vy + 30.0
            mag = math.hypot(fx, fy)
            if mag > 46.0:
                fx *= 46.0 / mag
                fy *= 46.0 / mag
            sandbox.apply_agent_force(fx, fy)
            return
    tx, ty = _zone_center(sandbox, next_switch)
    dx, dy = tx - x, ty - y
    dist = math.sqrt(dx * dx + dy * dy)
    speed = math.sqrt(vx * vx + vy * vy)
    in_ramp = RAMP_X_LO <= x <= RAMP_X_HI
    use_ramp = in_ramp and (next_switch == "B" or (next_switch == "C" and (x < bx or y > 2.4)))
    on_flat_for_c = (y <= 2.6 or x >= 6.4) and not (3.5 <= x < bx)
    inside = is_inside_zone(sandbox, x, y, next_switch)
    if next_switch == "B" and _s2_gate_crossed[0]:
        tx, ty = 4.5, 3.15
        fx = 7.0 * (tx - x) - 11.0 * vx
        fy = 30.0 + 7.0 * (ty - y) - 11.0 * vy
        mag = math.hypot(fx, fy)
        if mag > 31.0:
            fx *= 31.0 / mag
            fy *= 31.0 / mag
        sandbox.apply_agent_force(fx, fy)
        return
    if next_switch == "C" and x < 7.45:
        if x < 6.35 and y < 4.20:
            tx, ty = 5.45, 4.55
        elif x < 6.55:
            tx, ty = 6.75, 4.35
        else:
            tx, ty = 7.55, 4.20
        fx = 36.0 * (tx - x) - 13.0 * vx
        fy = 42.0 * (ty - y) - 13.0 * vy + 30.0
        mag = math.hypot(fx, fy)
        if mag > 48.0:
            fx *= 48.0 / mag
            fy *= 48.0 / mag
        sandbox.apply_agent_force(fx, fy)
        return
    if next_switch == "C" and not _s2_c_charged[0]:
        if x > 7.85 and y > 3.05:
            _s2_c_charged[0] = True
        else:
            fx = 34.0 * (8.0 - x) - 14.0 * vx
            fy = 30.0 + 40.0 * (3.20 - y) - 14.0 * vy
            mag = math.hypot(fx, fy)
            if mag > 47.0:
                fx *= 47.0 / mag
                fy *= 47.0 / mag
            sandbox.apply_agent_force(fx, fy)
            return
    if next_switch == "C" and x < 7.5 and dist > 0.2 and on_flat_for_c:
        fx = mf * 0.98 if dx > 0 else -mf * 0.4
        fy = -HOLD_DAMP * vy
        sandbox.apply_agent_force(fx, fy)
        return
    if dist < 1e-6:
        sandbox.apply_agent_force(-HOLD_DAMP * vx, -HOLD_DAMP * vy)
        return
    if inside or dist < 1.0:
        local_cap = 31.0 if inside else mf
        fy_boost = 30.0 if in_ramp else 0.0
        if speed > 0.025:
            fx = -HOLD_DAMP * 60.0 * vx
            fy = -HOLD_DAMP * 60.0 * vy + fy_boost
        else:
            fx = 300.0 * dx - HOLD_DAMP * 20.0 * vx
            fy = 300.0 * dy - HOLD_DAMP * 20.0 * vy + fy_boost
        mag = math.sqrt(fx * fx + fy * fy)
        if mag > local_cap:
            fx, fy = fx * local_cap / mag, fy * local_cap / mag
        sandbox.apply_agent_force(fx, fy)
        return
    if use_ramp:
        if next_switch == "B" or (next_switch == "C" and x < bx):
            y_target = RAMP_Y_TARGET
            fx = min(mf * 0.95, 45.0) * (1.0 if dx > 0 else -0.5) - APPROACH_DAMP * vx
            fy = RAMP_Y_GAIN * (y_target - y) + (40.0 if y < 3.2 else 0.0) - APPROACH_DAMP * vy
        else:
            y_target = 2.0
            dy_local = y_target - y
            if y > 3.0:
                fx = mf * 0.98 if dx > 0 else -mf * 0.3
                y_target_local = 3.9 if x < 6.4 else 2.0
                fy = 60.0 * (y_target_local - y) - APPROACH_DAMP * vy
            else:
                fx = min(mf * 0.95, 47.0) * (1.0 if dx > 0 else 0.5) - APPROACH_DAMP * vx
                fy = RAMP_Y_GAIN * dy_local - APPROACH_DAMP * vx
        mag = math.sqrt(fx * fx + fy * fy)
        if mag > mf:
            fx, fy = fx * mf / mag, fy * mf / mag
        sandbox.apply_agent_force(fx, fy)
        return
    if dist < APPROACH_RADIUS:
        force_mag = min(mf * 0.5, GAIN_APPROACH * 2.0 * dist)
        ux, uy = (dx / dist, dy / dist) if dist > 1e-6 else (0, 0)
        fx = force_mag * ux - APPROACH_DAMP * 1.5 * vx
        fy = (force_mag * uy if y > 2.2 else 15.0) - APPROACH_DAMP * 1.5 * vy
        mag = math.sqrt(fx * fx + fy * fy)
        if mag > mf:
            fx, fy = fx * mf / mag, fy * mf / mag
        sandbox.apply_agent_force(fx, fy)
        return
    gain = GAIN_NORMAL * 2.0 if next_switch == "B" else GAIN_NORMAL * 1.5
    force_mag = min(mf, gain * dist)
    ux, uy = (dx / dist, dy / dist) if dist > 1e-6 else (0, 0)
    sandbox.apply_agent_force(force_mag * ux, force_mag * uy)

def build_agent_stage_3(sandbox):
    body = sandbox.get_agent_body()
    body.s3_timing = {"a": None, "gate_crossed": False, "c_charged": False}
    return body

def agent_action_stage_3(sandbox, agent_body, step_count):
    bx = float(sandbox.get_barrier_x())
    delay_barrier = int(sandbox.get_barrier_delay_steps())
    mf = _max_agent_force(sandbox)
    FORCE_LIMIT = 22.5
    SPEED_CAP = 0.018
    triggered = sandbox.get_triggered_switches()
    timing = agent_body.s3_timing
    if triggered and triggered[0] == "A" and timing["a"] is None:
        timing["a"] = step_count
    next_switch = sandbox.get_next_required_switch()
    cooldown = sandbox.get_cooldown_remaining()
    x, y = sandbox.get_agent_position()
    vx, vy = sandbox.get_agent_velocity()
    speed = math.sqrt(vx * vx + vy * vy)
    if next_switch is None:
        sandbox.apply_agent_force(-18.0 * vx, -18.0 * vy)
        return
    inside = is_inside_zone(sandbox, x, y, next_switch)
    if next_switch == "A":
        near_zone = 1.35 <= x <= 2.65 and 1.35 <= y <= 2.65
    elif next_switch == "B":
        near_zone = 4.05 <= x <= 5.85 and 2.55 <= y <= 3.85
    else:
        near_zone = 7.35 <= x <= 8.65 and 1.35 <= y <= 2.65
    force_cap = (FORCE_LIMIT * 0.96) if near_zone else mf
    in_ramp = RAMP_X_LO <= x <= RAMP_X_HI
    if inside and next_switch == "B" and in_ramp:
        DAMP_GAIN = 80.0
        POS_GAIN = 3.0
    else:
        DAMP_GAIN = 200.0
        POS_GAIN = 5.0
    if cooldown > 0:
        if "A" in triggered and "B" not in triggered:
            hx, hy = _zone_center(sandbox, "A")
        elif "B" in triggered and "C" not in triggered:
            hx, hy = _zone_center(sandbox, "B")
        else:
            hx, hy = x, y
        fx = 60.0 * (hx - x) - 25.0 * vx
        fy = 60.0 * (hy - y) - 25.0 * vy
        mag = math.sqrt(fx * fx + fy * fy)
        if mag > force_cap:
            fx, fy = fx * force_cap / mag, fy * force_cap / mag
        sandbox.apply_agent_force(fx, fy)
        return
    if next_switch == "B" and not timing["gate_crossed"]:
        if x > 4.72 and y > 4.12:
            timing["gate_crossed"] = True
        else:
            tx, ty = ((4.08, 4.52) if y < 4.30 else (4.90, 4.48))
            fx = 30.0 * (tx - x) - 17.0 * vx
            fy = 40.0 * (ty - y) - 17.0 * vy + 30.0
            mag = math.sqrt(fx * fx + fy * fy)
            if mag > 47.0:
                fx, fy = fx * 47.0 / mag, fy * 47.0 / mag
            sandbox.apply_agent_force(fx, fy)
            return
    if next_switch == "B" and timing["gate_crossed"]:
        tx, ty = 4.5, 3.15
        fx = 6.0 * (tx - x) - 12.0 * vx
        fy = 30.0 + 6.0 * (ty - y) - 12.0 * vy
        mag = math.hypot(fx, fy)
        if mag > 25.5:
            fx *= 25.5 / mag
            fy *= 25.5 / mag
        sandbox.apply_agent_force(fx, fy)
        return
    if next_switch == "C" and x < 7.45:
        if x < 6.35 and y < 4.25:
            tx, ty = 5.40, 4.60
        elif x < 6.58:
            tx, ty = 6.78, 4.38
        else:
            tx, ty = 7.58, 4.22
        fx = 34.0 * (tx - x) - 15.0 * vx
        fy = 44.0 * (ty - y) - 15.0 * vy + 30.0
        mag = math.hypot(fx, fy)
        if mag > 48.0:
            fx *= 48.0 / mag
            fy *= 48.0 / mag
        sandbox.apply_agent_force(fx, fy)
        return
    if next_switch == "C" and not timing["c_charged"]:
        if x > 7.85 and y > 3.05:
            timing["c_charged"] = True
        else:
            fx = 32.0 * (8.0 - x) - 16.0 * vx
            fy = 30.0 + 42.0 * (3.20 - y) - 16.0 * vy
            mag = math.hypot(fx, fy)
            if mag > 47.0:
                fx *= 47.0 / mag
                fy *= 47.0 / mag
            sandbox.apply_agent_force(fx, fy)
            return
    tx, ty = _zone_center(sandbox, next_switch)
    dx, dy = tx - x, ty - y
    dist = math.sqrt(dx * dx + dy * dy) if dx or dy else 0.0
    use_ramp = in_ramp and (next_switch == "B" or (next_switch == "C" and (x < bx or y > 2.4)))
    on_flat_for_c = (y <= 2.6 or x >= 6.4) and not (3.5 <= x < bx)
    if inside:
        fx = -DAMP_GAIN * vx + POS_GAIN * dx
        fy = -DAMP_GAIN * vy + POS_GAIN * dy
        if in_ramp and next_switch == "B":
            fx += 14.4
            fy += 14.4
        mag = math.sqrt(fx * fx + fy * fy)
        if mag > force_cap:
            fx, fy = fx * force_cap / mag, fy * force_cap / mag
        sandbox.apply_agent_force(fx, fy)
        return
    if dist < 1.2:
        nd = DAMP_GAIN * 0.8
        np = POS_GAIN * 4.0
        fx = -nd * vx + np * dx
        fy = -nd * vy + np * dy
        if in_ramp and next_switch == "B":
            fx += 14.4
            fy += 14.4
        mag = math.sqrt(fx * fx + fy * fy)
        if mag > force_cap:
            fx, fy = fx * force_cap / mag, fy * force_cap / mag
        sandbox.apply_agent_force(fx, fy)
        return
    if next_switch == "C" and x < 7.5 and dist > 0.2 and on_flat_for_c:
        fx = min(mf * 0.98, force_cap) if dx > 0 else max(-mf * 0.4, -force_cap)
        fy = -HOLD_DAMP * vy
        sandbox.apply_agent_force(fx, fy)
        return
    if dist < 1e-6:
        sandbox.apply_agent_force(-HOLD_DAMP * vx, -HOLD_DAMP * vy)
        return
    if use_ramp:
        if next_switch == "B":
            y_target = RAMP_Y_TARGET
            fx = min(mf * 0.95, 45.0) * (1.0 if dx > 0 else -0.5) - 1.8 * vx
            fy = RAMP_Y_GAIN * 0.8 * (y_target - y) + (40.0 if y < 3.2 else 0.0) - 1.8 * vy
        elif next_switch == "C":
            climb_f = min(mf * 0.95, 43.0)
            fx = climb_f - 2.0 * vx
            fy = climb_f - 2.0 * vy
        else:
            y_target = 2.0
            dy_local = y_target - y
            if y > 3.0:
                fx = mf * 0.98 if dx > 0 else -mf * 0.3
                y_target_local = 3.9 if x < 6.4 else 2.0
                fy = 60.0 * (y_target_local - y) - 1.8 * vy
            else:
                fx = min(mf * 0.95, 47.0) * (1.0 if dx > 0 else 0.5) - 1.8 * vx
                fy = RAMP_Y_GAIN * 0.6 * dy_local - 1.8 * vx
        mag = math.sqrt(fx * fx + fy * fy)
        if mag > force_cap:
            fx, fy = fx * force_cap / mag, fy * force_cap / mag
        sandbox.apply_agent_force(fx, fy)
        return
    if dist < APPROACH_RADIUS:
        force_mag = min(force_cap, max(45.0, GAIN_APPROACH * 10.0 * dist))
        ux, uy = (dx / dist, dy / dist) if dist > 1e-6 else (0, 0)
        fx = force_mag * ux - 1.8 * vx
        fy = (force_mag * uy if y > 2.2 else 25.0) - 1.8 * vy
        mag = math.sqrt(fx * fx + fy * fy)
        if mag > force_cap:
            fx, fy = fx * force_cap / mag, fy * force_cap / mag
        sandbox.apply_agent_force(fx, fy)
        return
    gain = GAIN_NORMAL * 5.0 if next_switch == "B" else GAIN_NORMAL * 3.0
    force_mag = min(force_cap, gain * dist)
    ux, uy = (dx / dist, dy / dist) if dist > 1e-6 else (0, 0)
    sandbox.apply_agent_force(force_mag * ux, force_mag * uy)

_step_when_a_triggered_s4 = [None]
_s4_gate_crossed = [False]
_s4_c_charged = [False]

def build_agent_stage_4(sandbox):
    _step_when_a_triggered_s4[0] = None
    _s4_gate_crossed[0] = False
    _s4_c_charged[0] = False
    return sandbox.get_agent_body()

def agent_action_stage_4(sandbox, agent_body, step_count):
    bx = float(sandbox.get_barrier_x())
    delay_barrier = int(sandbox.get_barrier_delay_steps())
    mf = _max_agent_force(sandbox)
    FORCE_LIMIT_S4 = 22.5
    HOLD_DAMP_S4 = 10.0
    triggered = sandbox.get_triggered_switches()
    if triggered and triggered[0] == "A" and _step_when_a_triggered_s4[0] is None:
        _step_when_a_triggered_s4[0] = step_count
    next_switch = sandbox.get_next_required_switch()
    cooldown = sandbox.get_cooldown_remaining()
    x, y = sandbox.get_agent_position()
    vx, vy = sandbox.get_agent_velocity()
    if next_switch is None:
        vx, vy = sandbox.get_agent_velocity()
        sandbox.apply_agent_force(-HOLD_DAMP_S4 * vx, -HOLD_DAMP_S4 * vy)
        return
    if cooldown > 0:
        vx, vy = sandbox.get_agent_velocity()
        sandbox.apply_agent_force(-HOLD_DAMP_S4 * vx, -HOLD_DAMP_S4 * vy)
        return
    if next_switch == "B" and not _s4_gate_crossed[0]:
        if x > 4.72 and y > 4.12:
            _s4_gate_crossed[0] = True
        else:
            tx, ty = ((4.05, 4.55) if y < 4.32 else (4.92, 4.50))
            fx = 30.0 * (tx - x) - 18.0 * vx
            fy = 42.0 * (ty - y) - 18.0 * vy + 30.0
            mag = math.hypot(fx, fy)
            if mag > 48.0:
                fx *= 48.0 / mag
                fy *= 48.0 / mag
            sandbox.apply_agent_force(fx, fy)
            return
    if next_switch == "B" and _s4_gate_crossed[0]:
        tx, ty = 4.5, 3.15
        fx = 5.0 * (tx - x) - 14.0 * vx
        fy = 30.0 + 5.0 * (ty - y) - 14.0 * vy
        mag = math.hypot(fx, fy)
        if mag > 21.5:
            fx *= 21.5 / mag
            fy *= 21.5 / mag
        sandbox.apply_agent_force(fx, fy)
        return
    if next_switch == "C" and x < 7.45:
        if x < 6.35 and y < 4.28:
            tx, ty = 5.38, 4.65
        elif x < 6.60:
            tx, ty = 6.82, 4.42
        else:
            tx, ty = 7.60, 4.24
        fx = 32.0 * (tx - x) - 17.0 * vx
        fy = 46.0 * (ty - y) - 17.0 * vy + 30.0
        mag = math.hypot(fx, fy)
        if mag > 49.0:
            fx *= 49.0 / mag
            fy *= 49.0 / mag
        sandbox.apply_agent_force(fx, fy)
        return
    if next_switch == "C" and not _s4_c_charged[0]:
        if x > 7.85 and y > 3.05:
            _s4_c_charged[0] = True
        else:
            fx = 30.0 * (8.0 - x) - 18.0 * vx
            fy = 30.0 + 44.0 * (3.20 - y) - 18.0 * vy
            mag = math.hypot(fx, fy)
            if mag > 48.0:
                fx *= 48.0 / mag
                fy *= 48.0 / mag
            sandbox.apply_agent_force(fx, fy)
            return
    if next_switch == "C" and _s4_c_charged[0]:
        inside_c = is_inside_zone(sandbox, x, y, "C")
        fx = 14.0 * (8.0 - x) - 32.0 * vx
        fy = 14.0 * (2.0 - y) - 32.0 * vy
        limit = 21.0 if inside_c else 38.0
        mag = math.hypot(fx, fy)
        if mag > limit:
            fx *= limit / mag
            fy *= limit / mag
        sandbox.apply_agent_force(fx, fy)
        return
    tx, ty = _zone_center(sandbox, next_switch)
    dx, dy = tx - x, ty - y
    dist = math.sqrt(dx * dx + dy * dy)
    speed = math.sqrt(vx * vx + vy * vy)
    in_ramp = RAMP_X_LO <= x <= RAMP_X_HI
    use_ramp = in_ramp and (next_switch == "B" or (next_switch == "C" and (x < bx or y > 2.4)))
    on_flat_for_c = (y <= 2.6 or x >= 6.4) and not (3.5 <= x < bx)
    inside = is_inside_zone(sandbox, x, y, next_switch)
    force_cap = (FORCE_LIMIT_S4 * 0.95) if inside else mf
    if next_switch == "C" and x < 7.5 and dist > 0.2 and on_flat_for_c:
        fx = min(mf * 0.98, force_cap) if dx > 0 else max(-mf * 0.4, -force_cap)
        fy = -HOLD_DAMP_S4 * vy
        sandbox.apply_agent_force(fx, fy)
        return
    if dist < 1e-6:
        sandbox.apply_agent_force(-HOLD_DAMP_S4 * vx, -HOLD_DAMP_S4 * vy)
        return
    if inside or dist < 1.0:
        fy_boost = 100.0 if in_ramp else 50.0
        fx = 5000.0 * dx - HOLD_DAMP_S4 * 100.0 * vx
        fy = 5000.0 * dy - HOLD_DAMP_S4 * 100.0 * vy + fy_boost
        mag = math.sqrt(fx * fx + fy * fy)
        if mag > force_cap:
            fx, fy = fx * force_cap / mag, fy * force_cap / mag
        sandbox.apply_agent_force(fx, fy)
        return
    if use_ramp:
        if next_switch == "B" or (next_switch == "C" and x < bx):
            y_target = RAMP_Y_TARGET
            fx = min(mf * 0.95, force_cap) * (1.0 if dx > 0 else -0.5) - HOLD_DAMP_S4 * vx
            fy = RAMP_Y_GAIN * (0.4 if dist < 1.0 else 0.9) * (y_target - y) + (60.0 if y < 3.2 else 0.0) - HOLD_DAMP_S4 * vy
        else:
            y_target = 2.0
            dy_local = y_target - y
            if y > 3.0:
                fx = mf * 0.98 if dx > 0 else -mf * 0.3
                y_target_local = 3.9 if x < 6.4 else 2.0
                fy = 60.0 * (y_target_local - y) - HOLD_DAMP_S4 * vy
            else:
                fx = min(mf * 0.95, force_cap) if dx > 0 else -mf * 0.3
                fy = RAMP_Y_GAIN * (0.4 if dist < 1.0 else 0.9) * dy_local - HOLD_DAMP_S4 * vx
        mag = math.sqrt(fx * fx + fy * fy)
        if mag > force_cap:
            fx, fy = fx * force_cap / mag, fy * force_cap / mag
        sandbox.apply_agent_force(fx, fy)
        return
    if dist < APPROACH_RADIUS:
        force_mag = min(force_cap, max(200.0, GAIN_APPROACH * 60.0 * dist))
        ux, uy = (dx / dist, dy / dist) if dist > 1e-6 else (0, 0)
        fx = force_mag * ux - HOLD_DAMP_S4 * 10.0 * vx
        fy = (force_mag * uy if y > 2.2 else 150.0) - HOLD_DAMP_S4 * 10.0 * vy
        mag = math.sqrt(fx * fx + fy * fy)
        if mag > force_cap:
            fx, fy = fx * force_cap / mag, fy * force_cap / mag
        sandbox.apply_agent_force(fx, fy)
        return
    gain = GAIN_NORMAL * 20.0
    force_mag = min(force_cap, gain * dist)
    ux, uy = (dx / dist, dy / dist) if dist > 1e-6 else (0, 0)
    sandbox.apply_agent_force(force_mag * ux, force_mag * uy)
