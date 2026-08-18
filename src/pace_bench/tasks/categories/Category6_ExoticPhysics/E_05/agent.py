import math

TARGET_X_MIN = 28.0

TARGET_X_MAX = 32.0

TARGET_Y_MIN = 6.0

TARGET_Y_MAX = 9.0

TARGET_X_CENTER = 30.0

TARGET_Y_CENTER = 8.85

CORRIDOR_Y = 8.95

KEYHOLE_Y = 6.75

G1_PERIOD, G1_WEAK_LO, G1_WEAK_HI = 52, 8, 20

G2_PERIOD, G2_WEAK_LO, G2_WEAK_HI = 42, 28, 35

KH_PERIOD, KH_WEAK_LO, KH_WEAK_HI = 38, 8, 18

GRAVITY_COMPENSATION = 96.0

MAX_THRUST = 165.0

K_P = 11.0

def _gate1_weak(s): return G1_WEAK_LO <= (s % G1_PERIOD) <= G1_WEAK_HI

def _gate2_weak(s): return G2_WEAK_LO <= (s % G2_PERIOD) <= G2_WEAK_HI

def _keyhole_weak(s): return KH_WEAK_LO <= (s % KH_PERIOD) <= KH_WEAK_HI

def build_agent(sandbox):
    return None

def agent_action(sandbox, agent_body, step_count):
    pos = sandbox.get_body_position()
    if pos is None: return
    x, y = pos
    vel = sandbox.get_body_velocity() or (0.0, 0.0)
    vx, vy = vel[0], vel[1]
    step = sandbox.get_step_count() if hasattr(sandbox, 'get_step_count') else step_count
    grav_comp = GRAVITY_COMPENSATION
    in_target = (TARGET_X_MIN <= x <= TARGET_X_MAX and TARGET_Y_MIN <= y <= TARGET_Y_MAX)
    if in_target:
        fx = -6.0 * vx
        fy = grav_comp - 6.0 * vy
    else:
        fy_ceiling = -60.0 if y > 9.65 else 0.0
        if x < 11.0:
            dy = CORRIDOR_Y - y
            if y < CORRIDOR_Y - 0.15:
                fx = 100.0
                fy = grav_comp + 20.0 * dy + fy_ceiling
            else:
                dx = 11.5 - x
                fy = grav_comp + 5.0 * dy + fy_ceiling
                fx = 110.0 if dx > 0 else max(-80, min(80, -2.5 * vx))
        elif x < 17.0:
            if _gate1_weak(step):
                fx = 78.0
                fy = grav_comp + 28.0 + fy_ceiling
            else:
                fx = max(-50, min(50, -2.5 * vx))
                fy = grav_comp + 4.0 * (CORRIDOR_Y - y) + fy_ceiling
        elif x < 20.5:
            if _gate2_weak(step):
                fx = 78.0
                fy = grav_comp + 28.0 + fy_ceiling
            else:
                fx = max(-50, min(50, -2.5 * vx))
                fy = grav_comp + 4.0 * (CORRIDOR_Y - y) + fy_ceiling
        elif x < 23.5:
            dx, dy = 25.0 - x, CORRIDOR_Y - y
            fx = K_P * dx
            fy = grav_comp + K_P * dy
        elif x < 26.0:
            if _keyhole_weak(step):
                dx, dy = 28.0 - x, KEYHOLE_Y - y
                fx = K_P * dx
                fy = grav_comp + K_P * dy
            else:
                fx = max(-65, min(65, -3.0 * vx))
                fy = grav_comp + 6.0 * (CORRIDOR_Y - y)
        else:
            dx = TARGET_X_CENTER - x
            dy = TARGET_Y_CENTER - y
            fx = K_P * dx
            fy = grav_comp + K_P * dy
        f = math.sqrt(fx * fx + fy * fy)
        if f > MAX_THRUST:
            scale = MAX_THRUST / f
            fx *= scale
            fy *= scale
    sandbox.apply_thrust(fx, fy)

def build_agent_stage_1(sandbox):
    return None

def agent_action_stage_1(sandbox, agent_body, step_count):
    pos = sandbox.get_body_position()
    if pos is None: return
    x, y = pos
    vx, vy = sandbox.get_body_velocity() or (0.0, 0.0)
    if x < 12.0:
        tx, ty = 13.0, 18.0
    elif x < 16.0:
        tx, ty = 17.0, 18.0
    elif x < 20.0:
        tx, ty = 21.0, 7.5
    else:
        tx, ty = 30.0, 7.5
    fx = 40.0 * (tx - x) - 15.0 * vx
    fy = 96.0 + 40.0 * (ty - y) - 15.0 * vy
    f = math.sqrt(fx*fx + fy*fy)
    if f > 165.0:
        fx = fx * 165.0 / f
        fy = fy * 165.0 / f
    sandbox.apply_thrust(fx, fy)

def build_agent_stage_2(sandbox):
    return None

def agent_action_stage_2(sandbox, agent_body, step_count):
    pos = sandbox.get_body_position()
    if pos is None: return
    x, y = pos
    vx, vy = sandbox.get_body_velocity() or (0.0, 0.0)
    physics = sandbox.get_physics_params()
    gravity_compensation = -9.6 * float(physics.get("gravity_y", -10.0))
    max_thrust = float(physics.get("max_thrust", 165.0))
    if x < 13.4:
        target_x, target_y = 14.2, 4.0
        gain_x, gain_y, velocity_gain = 28.0, 32.0, 16.0
    elif x < 15.7 and y < 6.9:
        target_x, target_y = 14.6, 7.5
        gain_x, gain_y, velocity_gain = 32.0, 42.0, 20.0
    elif x < 18.4:
        target_x, target_y = 19.2, 7.4
        gain_x, gain_y, velocity_gain = 28.0, 36.0, 18.0
    elif x < 23.4:
        target_x, target_y = 24.2, 7.0
        gain_x, gain_y, velocity_gain = 28.0, 34.0, 18.0
    else:
        target_x, target_y = 30.0, 7.5
        gain_x, gain_y, velocity_gain = 22.0, 28.0, 18.0
    fx = gain_x * (target_x - x) - velocity_gain * vx
    fy = gravity_compensation + gain_y * (target_y - y) - velocity_gain * vy
    f = math.sqrt(fx * fx + fy * fy)
    if f > max_thrust:
        fx = fx * max_thrust / f
        fy = fy * max_thrust / f
    sandbox.apply_thrust(fx, fy)

def build_agent_stage_3(sandbox):
    return None

def agent_action_stage_3(sandbox, agent_body, step_count):
    pos = sandbox.get_body_position()
    if pos is None: return
    x, y = pos
    vx, vy = sandbox.get_body_velocity() or (0.0, 0.0)
    step = sandbox.get_step_count() if hasattr(sandbox, 'get_step_count') else step_count
    hover_f = 310.0
    m_thrust = 420.0
    corridor_y = 8.95
    if x < 11.5:
        fx = 260.0
        fy = hover_f + 60.0 * (corridor_y - y) - 10.0 * vy
    elif x < 17.0:
        if _gate1_weak(step): fx = 280.0
        else: fx = -20.0 * vx
        fy = hover_f + 60.0 * (corridor_y - y) - 10.0 * vy
    elif x < 20.5:
        if _gate2_weak(step): fx = 280.0
        else: fx = -20.0 * vx
        fy = hover_f + 60.0 * (corridor_y - y) - 10.0 * vy
    elif x < 26.0:
        if _keyhole_weak(step):
            fx = 50.0 * (28.0 - x)
            fy = hover_f + 50.0 * (6.75 - y) - 10.0 * vy
        else:
            fx = -20.0 * vx
            fy = hover_f + 50.0 * (corridor_y - y) - 10.0 * vy
    else:
        fx = 50.0 * (30.0 - x) - 10.0 * vx
        fy = hover_f + 50.0 * (8.0 - y) - 10.0 * vy
    f = math.sqrt(fx*fx + fy*fy)
    if f > m_thrust:
        fx = fx * m_thrust / f
        fy = fy * m_thrust / f
    sandbox.apply_thrust(fx, fy)

def build_agent_stage_4(sandbox):
    return None

def agent_action_stage_4(sandbox, agent_body, step_count):
    pos = sandbox.get_body_position()
    if pos is None: return
    x, y = pos
    vx, vy = sandbox.get_body_velocity() or (0.0, 0.0)
    step = sandbox.get_step_count() if hasattr(sandbox, 'get_step_count') else step_count
    hover_f = 508.8
    m_thrust = 570.0
    corridor_y = 7.5
    push_fx = 155.0

    safe_altitude = y > 6.0
    if x < 18.0:
        fx = push_fx if safe_altitude else 20.0
        fy = hover_f + 60.0 * (corridor_y - y) - 25.0 * vy
    elif x < 22.0:
        if _gate1_weak(step):
            fx = push_fx if safe_altitude else 20.0
        else:
            fx = push_fx * 0.25 if safe_altitude else 10.0
        fy = hover_f + 60.0 * (corridor_y - y) - 25.0 * vy
    else:
        fx = push_fx
        fy = hover_f + 40.0 * (7.5 - y) - 25.0 * vy
    f = math.sqrt(fx*fx + fy*fy)
    if f > m_thrust:
        fx = fx * m_thrust / f
        fy = fy * m_thrust / f
    sandbox.apply_thrust(fx, fy)
