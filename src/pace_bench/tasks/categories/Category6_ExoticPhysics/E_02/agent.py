import math

TX_MIN, TX_MAX = 28.0, 32.0

TY_MIN, TY_MAX = 2.0, 5.0

TARGET_X, TARGET_Y = 30.0, 3.5

G1_X, G1_Y = 13.0, 2.0

G2_X, G2_Y = 23.0, 2.4

DRAIN_LO, DRAIN_HI = 14.5, 17.0

SLIP_LO, SLIP_HI = 17.5, 20.0

WIND_LO, WIND_HI = 20.5, 28.0

WIND_AMP = 20.0

WIND_OMEGA = 0.055

MAX_THRUST = 120.0

LOW_THRUST = 90.0

HEAT_SAFE_FRAC = 0.88

def build_agent(sandbox):
    return None

def _waypoint(x, y):
    if x < G1_X - 0.1:
        return (G1_X, G1_Y)
    if x < G2_X - 0.1:
        return (G2_X, G2_Y)
    return (TARGET_X, TARGET_Y)

def agent_action(sandbox, agent_body, step_count):
    if sandbox.is_overheated():
        return
    pos = sandbox.get_craft_position()
    if pos is None:
        return
    x, y = pos
    heat = sandbox.get_heat()
    overheat_limit = sandbox.get_overheat_limit()
    remaining = overheat_limit - heat
    step_idx = sandbox.get_step_count() if hasattr(sandbox, "get_step_count") else step_count
    if TX_MIN <= x <= TX_MAX and TY_MIN <= y <= TY_MAX:
        sandbox.apply_thrust(0.0, 75.0)
        return
    if x > 31.0 and y < 2.0:
        sandbox.apply_thrust(-150.0, 200.0)
        return
    if heat >= overheat_limit * HEAT_SAFE_FRAC:
        thrust_mag = min(LOW_THRUST, remaining * 0.25)
    else:
        thrust_mag = min(MAX_THRUST, remaining * 0.35)
    if overheat_limit <= 40000.0:
        heat_frac = heat / overheat_limit
        if heat_frac > 0.3:
            thrust_mag = 40.0
        else:
            thrust_mag = 60.0
    if x < 10.0:
        thrust_mag = min(100.0, thrust_mag)
    if 10.0 <= x <= 15.0:
        thrust_mag = min(MAX_THRUST, thrust_mag * 1.2)
    if DRAIN_LO <= x <= DRAIN_HI:
        thrust_mag = min(MAX_THRUST, thrust_mag * 1.4)
    if thrust_mag < 75.0 and overheat_limit > 40000.0:
        sandbox.apply_thrust(0.0, 75.0)
        return
    wx, wy = _waypoint(x, y)
    if x > 24.0:
        wy = 4.0
    if x < G1_X + 0.5 and y < 1.2:
        wy = max(wy, 1.3)
    if y < 1.4:
        wy = max(wy, 1.5)
    if G2_X - 1.0 <= x <= G2_X + 2.0 and y < 1.9:
        wy = max(wy, 2.2)
    if 20.5 <= x <= G2_X + 1.0 and y < 1.85:
        wx, wy = x + 0.3, 2.5
    dx = wx - x
    dy = wy - y
    dist = math.sqrt(dx * dx + dy * dy)
    if dist < 1e-6:
        sandbox.apply_thrust(0.0, 75.0)
        return
    ux = dx / dist
    uy = dy / dist
    fx = thrust_mag * ux
    fy = thrust_mag * uy
    if y < 1.4:
        fy += 100.0
    if x < G1_X + 1.0 and y < 1.1:
        fy += min(30.0, (1.2 - y) * 40.0)
    if 20.5 <= x <= G2_X + 2.0 and y < 1.9:
        fy += 110.0
    if x > 25.0 and y < 3.0:
        fy += 120.0
    if SLIP_LO <= x <= SLIP_HI:
        fx += 35.0
    if WIND_LO <= x <= WIND_HI:
        wind_fy = WIND_AMP * math.sin(WIND_OMEGA * step_idx)
        fy -= wind_fy
    total = math.sqrt(fx * fx + fy * fy)
    cap = thrust_mag * 2.5
    if y < 1.4:
        cap = max(cap, 180.0)
    if 20.5 <= x <= G2_X + 2.0 and y < 1.9:
        cap = max(cap, 250.0)
    if total > cap:
        scale = cap / total
        fx *= scale
        fy *= scale
    sandbox.apply_thrust(fx, fy)

def build_agent_stage_1(sandbox):
    pos = sandbox.get_craft_position()
    x, y = pos if pos is not None else (8.0, 2.0)
    return {
        "prev_x": x,
        "prev_y": y,
        "velocity_x": 0.0,
        "velocity_y": 0.0,
        "bias_x": 0.0,
        "inside_target_steps": 0,
        "finished": False,
    }

def agent_action_stage_1(sandbox, agent_body, step_count):
    if sandbox.is_overheated():
        return
    pos = sandbox.get_craft_position()
    if pos is None:
        return
    x, y = pos
    if agent_body["finished"]:
        sandbox.apply_thrust(0.0, 0.0)
        return

    if TX_MIN <= x <= TX_MAX and TY_MIN <= y <= TY_MAX:
        agent_body["inside_target_steps"] += 1
    else:
        agent_body["inside_target_steps"] = 0
    if agent_body["inside_target_steps"] >= 240:
        agent_body["finished"] = True
        sandbox.apply_thrust(0.0, 0.0)
        return

    raw_vx = (x - agent_body["prev_x"]) * 60.0
    raw_vy = (y - agent_body["prev_y"]) * 60.0
    vx = 0.65 * agent_body["velocity_x"] + 0.35 * raw_vx
    vy = 0.65 * agent_body["velocity_y"] + 0.35 * raw_vy
    agent_body["prev_x"] = x
    agent_body["prev_y"] = y
    agent_body["velocity_x"] = vx
    agent_body["velocity_y"] = vy

    if x < 11.2:
        desired_vx, desired_y = 1.6, 2.0
    elif x < 14.4:
        desired_vx, desired_y = 0.85, 2.0
    elif x < 17.2:
        desired_vx, desired_y = 2.2, 2.15
    elif x < 20.4:
        desired_vx, desired_y = 1.5, 2.2
    elif x < 21.5:
        desired_vx, desired_y = 1.0, 2.35
    elif x < 24.5:
        desired_vx, desired_y = 0.75, 2.4
    elif x < 27.0:
        desired_vx, desired_y = 1.35, 3.2
    else:
        desired_vx = max(-1.5, min(1.5, 0.8 * (30.0 - x)))
        desired_y = 3.5

    velocity_error = desired_vx - vx
    if not (DRAIN_LO <= x <= DRAIN_HI):
        agent_body["bias_x"] += 1.6 * velocity_error
        agent_body["bias_x"] = max(-50.0, min(400.0, agent_body["bias_x"]))

    if DRAIN_LO <= x <= DRAIN_HI:
        fx = agent_body["bias_x"] + 900.0 * velocity_error
        fx = max(-500.0, min(3600.0, fx))
    else:
        fx = agent_body["bias_x"] + 220.0 * velocity_error
        fx = max(-500.0, min(900.0, fx))

    fy = 75.0 + 210.0 * (desired_y - y) - 170.0 * vy
    fy = max(-500.0, min(500.0, fy))
    sandbox.apply_thrust(fx, fy)

def build_agent_stage_2(sandbox):
    pos = sandbox.get_craft_position()
    py = pos[1] if pos else 2.0
    return {"reached_target_once": False, "prev_y": py}

def agent_action_stage_2(sandbox, agent_body, step_count):
    if sandbox.is_overheated(): return
    pos = sandbox.get_craft_position()
    if pos is None: return
    x, y = pos
    step_idx = sandbox.get_step_count() if hasattr(sandbox, "get_step_count") else step_count

    if agent_body.get("reached_target_once"):
        sandbox.apply_thrust(0.0, 0.0)
        return
    if TX_MIN <= x <= TX_MAX and TY_MIN <= y <= TY_MAX:
        agent_body["reached_target_once"] = True
        sandbox.apply_thrust(0.0, 0.0)
        return

    prev_y = agent_body.get("prev_y", y)
    dy_step = y - prev_y
    agent_body["prev_y"] = y
    COUNTER_BASE = 425.0
    VEL_GAIN = 2400.0
    vel_correction = VEL_GAIN * dy_step
    heat = sandbox.get_heat()
    overheat_limit = sandbox.get_overheat_limit()
    remaining = overheat_limit - heat
    if heat >= overheat_limit * 0.85:
        thrust_mag = 50.0
    else:
        thrust_mag = 220.0
    wx, wy = _waypoint(x, y)

    if x < 15.0:
        wy = 1.8
    elif x < 24.5:
        wy = 2.3
    else:
        wy = 3.5

    if y < 1.5:
        wy = max(wy, 2.4)
    if G2_X - 1.0 <= x <= G2_X + 2.0 and y < 2.1:
        wy = max(wy, 2.8)
    if x > 25.0 and y < 2.5:
        wy = max(wy, 3.0)
    dx, dy = wx - x, wy - y
    dist = math.sqrt(dx*dx + dy*dy)
    if dist < 1e-6:
        fy = -COUNTER_BASE - vel_correction
        sandbox.apply_thrust(0.0, fy)
        return
    ux = dx / dist
    uy = dy / dist
    fx = thrust_mag * ux
    fy = thrust_mag * uy - COUNTER_BASE - vel_correction

    cap = max(thrust_mag * 12.0, 2000.0)

    if DRAIN_LO <= x <= DRAIN_HI:
        fx += 2200.0
        cap = max(cap, 3200.0)

    if SLIP_LO <= x <= SLIP_HI:
        fx += 55.0

    if WIND_LO <= x <= WIND_HI:
        wind_fy = 20.0 * math.sin(0.055 * step_idx)
        fy -= wind_fy
        fx += 20.0

    if y < 1.6:
        fy += 450.0
        cap = max(cap, 1800.0)
    if x < G1_X + 1.0 and y < 1.3:
        fy += min(220.0, (1.6 - y) * 450.0)
    if x > 25.0 and y < 2.8:
        fy += 220.0
    total = math.sqrt(fx*fx + fy*fy)
    if total > cap:
        scale = cap / total
        fx *= scale; fy *= scale
    sandbox.apply_thrust(fx, fy)

def build_agent_stage_3(sandbox):
    pos = sandbox.get_craft_position()
    x, y = pos if pos is not None else (8.0, 2.0)
    return {
        "prev_x": x,
        "prev_y": y,
        "velocity_x": 0.0,
        "velocity_y": 0.0,
        "inside_target_steps": 0,
        "finished": False,
    }

def agent_action_stage_3(sandbox, agent_body, step_count):
    if sandbox.is_overheated():
        return
    pos = sandbox.get_craft_position()
    if pos is None:
        return
    x, y = pos

    if TX_MIN <= x <= TX_MAX and TY_MIN <= y <= TY_MAX:
        agent_body["inside_target_steps"] += 1
    else:
        agent_body["inside_target_steps"] = 0
    if agent_body["inside_target_steps"] >= 120:
        agent_body["finished"] = True
    if agent_body["finished"]:
        sandbox.apply_thrust(0.0, 0.0)
        return

    raw_vx = (x - agent_body["prev_x"]) * 60.0
    raw_vy = (y - agent_body["prev_y"]) * 60.0
    vx = 0.7 * agent_body["velocity_x"] + 0.3 * raw_vx
    vy = 0.7 * agent_body["velocity_y"] + 0.3 * raw_vy
    agent_body["prev_x"] = x
    agent_body["prev_y"] = y
    agent_body["velocity_x"] = vx
    agent_body["velocity_y"] = vy

    if DRAIN_LO <= x <= DRAIN_HI:
        fx = 0.0
    elif SLIP_LO <= x <= SLIP_HI:
        desired_vx = 2.4
        fx = 650.0 + 260.0 * (desired_vx - vx)
        fx = max(250.0, min(1500.0, fx))
    elif x >= 27.0:
        desired_vx = max(-0.8, min(0.8, 0.9 * (29.0 - x)))
        fx = -260.0 + 300.0 * (desired_vx - vx)
        fx = max(-900.0, min(300.0, fx))
    elif x >= 20.0:
        fx = min(0.0, 220.0 * (1.65 - vx))
        fx = max(-500.0, fx)
    else:
        fx = 0.0

    if x < 10.5:
        fy = 0.0
    else:
        if x < 14.35:
            desired_y = 1.5
        elif x < 18.8:
            desired_y = None
        elif x < 24.5:
            desired_y = 2.38
        else:
            desired_y = 2.65
        if desired_y is None:
            fy = 0.0
        else:
            fy = 525.0 + 520.0 * (desired_y - y) - 260.0 * vy
            fy = max(-300.0, min(1500.0, fy))

    remaining = sandbox.get_overheat_limit() - sandbox.get_heat()
    requested = math.sqrt(fx * fx + fy * fy)
    available = max(0.0, remaining * 30.0)
    if requested > available and requested > 0.0:
        scale = available / requested
        fx *= scale
        fy *= scale
    sandbox.apply_thrust(fx, fy)

def build_agent_stage_4(sandbox): return {"reached_target_once": False}

def agent_action_stage_4(sandbox, agent_body, step_count):
    if sandbox.is_overheated(): return
    pos = sandbox.get_craft_position()
    if pos is None: return
    x, y = pos
    step_idx = sandbox.get_step_count() if hasattr(sandbox, "get_step_count") else step_count

    if agent_body.get("reached_target_once"):
        sandbox.apply_thrust(0.0, 0.0)
        return
    if TX_MIN <= x <= TX_MAX and TY_MIN <= y <= TY_MAX:
        agent_body["reached_target_once"] = True
        sandbox.apply_thrust(0.0, 0.0)
        return

    thrust_mag = 260.0

    if DRAIN_LO <= x <= DRAIN_HI:
        thrust_mag = 400.0
    wx, wy = _waypoint(x, y)

    if x < 22.0:
        wy = 2.4
    elif x < 25.0:
        wy = 2.6
    else:
        wy = 4.0
    if y < 1.7:
        wy = max(wy, 2.3)
    if 22.0 <= x <= 24.0:
        wy = min(wy, 2.7)
    dx, dy = wx - x, wy - y
    dist = math.sqrt(dx*dx + dy*dy)
    if dist < 1e-6: return
    ux = dx / dist
    uy = dy / dist
    fx = thrust_mag * ux
    fy = thrust_mag * uy

    fy += 75.0

    if y < 1.6: fy += 180.0
    if G2_X - 1.0 <= x <= G2_X + 2.0 and y < 2.2: fy += 140.0
    if x > 25.0 and y < 3.2: fy += 120.0

    if SLIP_LO <= x <= SLIP_HI:
        fx += 85.0

    if WIND_LO <= x <= WIND_HI:
        wind_fy = 105.0 * math.sin(0.48 * step_idx)
        fy -= wind_fy
        fx += 40.0
    sandbox.apply_thrust(fx, fy)
