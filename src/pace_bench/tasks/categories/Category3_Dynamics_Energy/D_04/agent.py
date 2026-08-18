import math

def build_agent(sandbox):
    return None

def agent_action(sandbox, agent_body, step_count):
    seat = sandbox.get_swing_seat()
    if seat is None:
        return
    pos = seat.position
    vel = seat.linearVelocity
    vx = vel.x
    target_y = 11.7
    if pos.y < target_y:
        force_mag = 42.0
        if (pos.x < 10.0 and vx > 0) or (pos.x > 10.0 and vx < 0):
            sandbox.apply_force_to_seat(force_mag if vx > 0 else -force_mag, 0)
        elif (pos.x < 10.0 and vx < 0) or (pos.x > 10.0 and vx > 0):
            sandbox.apply_force_to_seat(-force_mag if pos.x < 10.0 else force_mag, 0)
    else:
        sandbox.apply_force_to_seat(-vx * 10.0, 0)

def build_agent_stage_1(sandbox):
    return None

def agent_action_stage_1(sandbox, agent_body, step_count):
    seat = sandbox.get_swing_seat()
    if seat is None: return
    pos = seat.position
    vx, vy = seat.linearVelocity.x, seat.linearVelocity.y
    v_sq = vx**2 + vy**2
    expected_y = pos.y + v_sq / 20.0
    in_target = pos.y >= 11.7 and 9.35 <= pos.x <= 10.65
    dead_zone_left, dead_zone_right = 9.2, 10.4
    in_dead_zone = dead_zone_left <= pos.x <= dead_zone_right
    min_speed_in_zone = 16.0
    if in_target:
        fx = max(-42.0, min(42.0, -vx * 100.0))
        fy = max(-42.0, min(42.0, -vy * 100.0))
        sandbox.apply_force_to_seat(fx, fy)
    elif expected_y > 14.2:
        sandbox.apply_force_to_seat(max(-42.0, min(42.0, -vx * 80.0)), max(-42.0, min(42.0, -vy * 80.0)))
    else:
        if not in_dead_zone:
            if abs(vx) > 0.05:
                fx = 42.0 if vx > 0 else -42.0
            else:
                fx = 42.0 if pos.x < 10.0 else -42.0
            sandbox.apply_force_to_seat(fx, 0)
        else:
            if abs(vx) >= min_speed_in_zone:
                fx = 42.0 if vx > 0 else -42.0
                sandbox.apply_force_to_seat(fx, 0)
            else:
                sandbox.apply_force_to_seat(0, 0)

def build_agent_stage_2(sandbox):
    return None

def agent_action_stage_2(sandbox, agent_body, step_count):
    seat = sandbox.get_swing_seat()
    if seat is None: return
    pos = seat.position
    vx, vy = seat.linearVelocity.x, seat.linearVelocity.y
    speed = math.sqrt(vx**2 + vy**2)
    v_sq = vx**2 + vy**2
    expected_y = pos.y + v_sq / 20.0
    in_target = pos.y >= 11.7 and 9.35 <= pos.x <= 10.65

    if expected_y > 14.0 or in_target:
        fx = max(-42.0, min(42.0, -vx * 100.0))
        fy = max(-42.0, min(42.0, -vy * 100.0))
        sandbox.apply_force_to_seat(fx, fy)
        return

    if speed > 0.1:
        fx = 42.0 * vx / speed
        fy = 42.0 * vy / speed
    else:
        fx = 42.0 if pos.x < 10.0 else -42.0
        fy = 0.0
    sandbox.apply_force_to_seat(fx, fy)

def build_agent_stage_3(sandbox):
    return None

def agent_action_stage_3(sandbox, agent_body, step_count):
    seat = sandbox.get_swing_seat()
    if seat is None: return
    pos = seat.position
    vx, vy = seat.linearVelocity.x, seat.linearVelocity.y
    v_sq = vx**2 + vy**2
    expected_y = pos.y + v_sq / 20.0
    in_target = pos.y >= 11.7 and 9.35 <= pos.x <= 10.65
    if expected_y > 14.1 or in_target:
        fx = max(-42.0, min(0.0, -vx * 100.0))
        fy = max(-42.0, min(42.0, -vy * 100.0))
        sandbox.apply_force_to_seat(fx, fy)
    elif expected_y > 13.95:
        sandbox.apply_force_to_seat(0, 0)
    else:
        if vx < -0.05:
            sandbox.apply_force_to_seat(-42.0, 42.0 if vy > 0 else -42.0)
        elif vx > 0.05:
            sandbox.apply_force_to_seat(0, 42.0 if vy > 0 else -42.0)
        else:
            sandbox.apply_force_to_seat(-42.0, 0)

def build_agent_stage_4(sandbox):
    return None

def agent_action_stage_4(sandbox, agent_body, step_count):
    seat = sandbox.get_swing_seat()
    if seat is None: return
    pos = seat.position
    vx, vy = seat.linearVelocity.x, seat.linearVelocity.y
    v_sq = vx**2 + vy**2
    expected_y = pos.y + v_sq / 20.0
    in_target = pos.y >= 11.7 and 9.35 <= pos.x <= 10.65
    if expected_y > 14.1 or in_target:
        fx = max(0.0, min(42.0, -vx * 100.0))
        fy = max(-42.0, min(42.0, -vy * 100.0))
        sandbox.apply_force_to_seat(fx, fy)
    elif expected_y > 13.95:
        sandbox.apply_force_to_seat(0, 0)
    else:
        if pos.x < 9.8 or pos.x > 10.2:
            if vx > 0.05:
                sandbox.apply_force_to_seat(42.0, 42.0 if vy > 0 else -42.0)
            elif vx < -0.05:
                sandbox.apply_force_to_seat(0, 42.0 if vy > 0 else -42.0)
            else:
                sandbox.apply_force_to_seat(42.0, 0)
