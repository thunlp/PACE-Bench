import math

CHECKPOINT_X_LO = 17.5

CHECKPOINT_X_HI = 19.0

CHECKPOINT_Y_LO = 3.8

CHECKPOINT_Y_HI = 4.5

CHECKPOINT_X_CENTER = (CHECKPOINT_X_LO + CHECKPOINT_X_HI) / 2

CHECKPOINT_Y_CENTER = (CHECKPOINT_Y_LO + CHECKPOINT_Y_HI) / 2

CHECKPOINT_B_X_LO = 23.0

CHECKPOINT_B_X_HI = 24.5

CHECKPOINT_B_Y_LO = 2.5

CHECKPOINT_B_Y_HI = 3.2

CHECKPOINT_B_X_CENTER = (CHECKPOINT_B_X_LO + CHECKPOINT_B_X_HI) / 2

CHECKPOINT_B_Y_CENTER = (CHECKPOINT_B_Y_LO + CHECKPOINT_B_Y_HI) / 2

TARGET_X_MIN = 28.0

TARGET_X_MAX = 32.0

TARGET_X_CENTER = (TARGET_X_MIN + TARGET_X_MAX) / 2

TARGET_Y_CENTER = (2.2 + 2.8) / 2

MOMENTUM_DRAIN_X_LO = 11.0

MOMENTUM_DRAIN_X_HI = 17.0

REVERSE_THRUST_X_LO = 20.0

REVERSE_THRUST_X_HI = 25.0

THRUST_SCALE_X_LO = 19.5

THRUST_SCALE_X_HI = 21.0

THRUST_SCALE_FACTOR = 0.5

OSCILLATING_FX_X_LO = 21.0

OSCILLATING_FX_X_HI = 27.0

OSCILLATING_FX_AMP = 30.0

OSCILLATING_FX_OMEGA = 0.04

WIND_ZONE_X_LO = 14.0

WIND_ZONE_X_HI = 28.0

WIND_FY_BASE = 20.0

WIND_FY_AMP = 35.0

WIND_OMEGA = 0.06

SPEED_PENALTY_X_LO = 22.0

SPEED_PENALTY_X_HI = 26.0

SPEED_PENALTY_THRESHOLD = 4.0

VERT_REVERSE_X_LO = 26.5

VERT_REVERSE_X_HI = 28.5

GRAVITY_COMPENSATION = 260.0

MAX_THRUST = 500.0

K_X = 8.0

K_X_DRAIN = 28.0

K_Y = 100.0

K_Y_CHECKPOINT = 120.0

K_BRAKE = 55.0

X_BRAKE_START = 28.0

SPEED_ZONE_FX_SCALE = 0.4

def build_agent(sandbox):
    return None

def agent_action(sandbox, agent_body, step_count):
    pos = sandbox.get_sled_position()
    if pos is None:
        return
    x, y = pos
    vel = sandbox.get_sled_velocity() or (0.0, 0.0)
    vx, vy = vel[0], vel[1]
    speed = math.sqrt(vx * vx + vy * vy)
    past_a = x >= CHECKPOINT_X_HI
    past_b = sandbox.get_checkpoint_b_reached() if hasattr(sandbox, "get_checkpoint_b_reached") else (x >= CHECKPOINT_B_X_HI)
    in_drain = MOMENTUM_DRAIN_X_LO <= x <= MOMENTUM_DRAIN_X_HI
    in_reverse = REVERSE_THRUST_X_LO <= x <= REVERSE_THRUST_X_HI
    in_thrust_scale = THRUST_SCALE_X_LO <= x <= THRUST_SCALE_X_HI
    in_oscillating_fx = OSCILLATING_FX_X_LO <= x <= OSCILLATING_FX_X_HI
    in_speed_penalty = SPEED_PENALTY_X_LO <= x <= SPEED_PENALTY_X_HI
    in_vert_reverse = VERT_REVERSE_X_LO <= x <= VERT_REVERSE_X_HI
    if not past_a:
        dy = CHECKPOINT_Y_CENTER - y
        fy = GRAVITY_COMPENSATION + K_Y_CHECKPOINT * dy
    elif past_b:
        if in_vert_reverse:
            fy = -(GRAVITY_COMPENSATION + K_Y * (TARGET_Y_CENTER - y))
        else:
            fy = GRAVITY_COMPENSATION + K_Y * (TARGET_Y_CENTER - y)
    else:
        dy = CHECKPOINT_B_Y_CENTER - y
        fy = GRAVITY_COMPENSATION + K_Y * dy * 1.4
    if WIND_ZONE_X_LO <= x <= WIND_ZONE_X_HI:
        fy -= WIND_FY_BASE + WIND_FY_AMP * math.sin(step_count * WIND_OMEGA)
    if x > TARGET_X_MAX:
        if in_reverse:
            fx = K_BRAKE * vx if vx > 0 else K_BRAKE * 2.0
        else:
            fx = -K_BRAKE * vx if vx > 0 else -K_BRAKE * 2.0
        if not in_reverse and fx > 0:
            fx = 0
        if in_reverse and fx < 0:
            fx = 0
    elif x >= X_BRAKE_START:
        if in_reverse:
            fx = K_BRAKE * vx
        else:
            fx = -K_BRAKE * vx
    elif x > REVERSE_THRUST_X_HI:
        fx = K_X * (TARGET_X_CENTER - x)
    elif past_b and x <= REVERSE_THRUST_X_HI:
        fx = -K_X * (TARGET_X_CENTER - x)
    elif past_b:
        fx = K_X * (TARGET_X_CENTER - x)
    elif past_a and x < REVERSE_THRUST_X_LO:
        fx = K_X * (CHECKPOINT_B_X_CENTER - x)
        if in_drain:
            fx *= (K_X_DRAIN / K_X)
    elif past_a:
        dx_b = CHECKPOINT_B_X_CENTER - x
        fx = -K_X * dx_b
        if in_drain:
            fx *= (K_X_DRAIN / K_X)
        if in_speed_penalty and speed > SPEED_PENALTY_THRESHOLD * 0.8:
            fx *= SPEED_ZONE_FX_SCALE
        if CHECKPOINT_B_X_LO <= x <= CHECKPOINT_B_X_HI:
            if y > CHECKPOINT_B_Y_HI:
                fx *= 0.45
            else:
                fx *= 0.7
        elif OSCILLATING_FX_X_LO <= x < CHECKPOINT_B_X_LO:
            fx *= 0.85
    else:
        dx = CHECKPOINT_X_CENTER - x
        fx = K_X * dx
        if in_drain:
            fx *= (K_X_DRAIN / K_X)
        if 15.0 <= x < CHECKPOINT_X_LO and dx > 0:
            fx *= 1.4
    if in_oscillating_fx:
        fx -= OSCILLATING_FX_AMP * math.sin(step_count * OSCILLATING_FX_OMEGA)
    if in_thrust_scale:
        fx *= (1.0 / THRUST_SCALE_FACTOR)
        fy *= (1.0 / THRUST_SCALE_FACTOR)
    f = math.sqrt(fx * fx + fy * fy)
    if f > MAX_THRUST:
        scale = MAX_THRUST / f
        fx *= scale
        fy *= scale
    sandbox.apply_thrust(fx, fy)

def build_agent_stage_1(sandbox):
    return None

def agent_action_stage_1(sandbox, agent_body, step_count):
    pos = sandbox.get_sled_position()
    if pos is None: return
    x, y = pos
    vel = sandbox.get_sled_velocity() or (0.0, 0.0)
    vx, vy = vel[0], vel[1]
    speed = math.sqrt(vx * vx + vy * vy)
    k_x_s1 = 30.0
    k_x_drain_s1 = 80.0
    gravity_comp_s1 = 260.0
    past_a = sandbox.get_checkpoint_a_reached() if hasattr(sandbox, "get_checkpoint_a_reached") else (x >= CHECKPOINT_X_HI)
    past_b = sandbox.get_checkpoint_b_reached() if hasattr(sandbox, "get_checkpoint_b_reached") else (x >= CHECKPOINT_B_X_HI)
    in_drain = MOMENTUM_DRAIN_X_LO <= x <= MOMENTUM_DRAIN_X_HI
    in_reverse = REVERSE_THRUST_X_LO <= x <= REVERSE_THRUST_X_HI
    in_thrust_scale = THRUST_SCALE_X_LO <= x <= THRUST_SCALE_X_HI
    in_oscillating_fx = OSCILLATING_FX_X_LO <= x <= OSCILLATING_FX_X_HI
    in_speed_penalty = SPEED_PENALTY_X_LO <= x <= SPEED_PENALTY_X_HI
    in_vert_reverse = VERT_REVERSE_X_LO <= x <= VERT_REVERSE_X_HI
    if not past_a:
        fy = gravity_comp_s1 + K_Y_CHECKPOINT * (CHECKPOINT_Y_CENTER - y)
    elif past_b:
        if in_vert_reverse:
            fy = -(gravity_comp_s1 + K_Y * (TARGET_Y_CENTER - y))
        else:
            fy = gravity_comp_s1 + K_Y * (TARGET_Y_CENTER - y)
    else:
        fy = gravity_comp_s1 + K_Y * (CHECKPOINT_B_Y_CENTER - y) * 1.4
    if WIND_ZONE_X_LO <= x <= WIND_ZONE_X_HI:
        fy -= WIND_FY_BASE + WIND_FY_AMP * math.sin(step_count * WIND_OMEGA)
    if x > TARGET_X_MAX:
        fx = (K_BRAKE * vx if vx > 0 else K_BRAKE * 2.0) if in_reverse else (-K_BRAKE * vx if vx > 0 else -K_BRAKE * 2.0)
        if not in_reverse and fx > 0: fx = 0
        if in_reverse and fx < 0: fx = 0
    elif x >= X_BRAKE_START:
        fx = K_BRAKE * vx if in_reverse else -K_BRAKE * vx
    elif x > REVERSE_THRUST_X_HI:
        fx = k_x_s1 * (TARGET_X_CENTER - x)
    elif past_b and x <= REVERSE_THRUST_X_HI:
        fx = -k_x_s1 * (TARGET_X_CENTER - x)
    elif past_b:
        fx = k_x_s1 * (TARGET_X_CENTER - x)
    elif past_a and x < REVERSE_THRUST_X_LO:
        fx = k_x_s1 * (CHECKPOINT_B_X_CENTER - x)
        if in_drain: fx *= (k_x_drain_s1 / k_x_s1)
    elif past_a:
        fx = -k_x_s1 * (CHECKPOINT_B_X_CENTER - x)
        if in_drain: fx *= (k_x_drain_s1 / k_x_s1)
        if in_speed_penalty and speed > SPEED_PENALTY_THRESHOLD * 0.8: fx *= SPEED_ZONE_FX_SCALE
    else:
        dx = CHECKPOINT_X_CENTER - x
        fx = k_x_s1 * dx
        if in_drain: fx *= (k_x_drain_s1 / k_x_s1)
        if 15.0 <= x < CHECKPOINT_X_LO and dx > 0: fx *= 1.4
    if in_oscillating_fx:
        fx -= OSCILLATING_FX_AMP * math.sin(step_count * OSCILLATING_FX_OMEGA)
    if in_thrust_scale:
        fx *= (1.0 / THRUST_SCALE_FACTOR)
        fy *= (1.0 / THRUST_SCALE_FACTOR)
    f = math.sqrt(fx * fx + fy * fy)
    if f > MAX_THRUST:
        scale = MAX_THRUST / f
        fx *= scale; fy *= scale
    sandbox.apply_thrust(fx, fy)

def build_agent_stage_2(sandbox):
    return None

def agent_action_stage_2(sandbox, agent_body, step_count):
    pos = sandbox.get_sled_position()
    if pos is None: return
    x, y = pos
    vel = sandbox.get_sled_velocity() or (0.0, 0.0)
    vx, vy = vel[0], vel[1]
    speed = math.sqrt(vx * vx + vy * vy)
    k_x_s2 = 30.0
    k_x_drain_s2 = 80.0
    gravity_comp_s2 = 437.0
    past_a = sandbox.get_checkpoint_a_reached() if hasattr(sandbox, "get_checkpoint_a_reached") else (x >= CHECKPOINT_X_HI)
    past_b = sandbox.get_checkpoint_b_reached() if hasattr(sandbox, "get_checkpoint_b_reached") else (x >= CHECKPOINT_B_X_HI)
    in_drain = MOMENTUM_DRAIN_X_LO <= x <= MOMENTUM_DRAIN_X_HI
    in_reverse = REVERSE_THRUST_X_LO <= x <= REVERSE_THRUST_X_HI
    in_thrust_scale = THRUST_SCALE_X_LO <= x <= THRUST_SCALE_X_HI
    in_oscillating_fx = OSCILLATING_FX_X_LO <= x <= OSCILLATING_FX_X_HI
    in_speed_penalty = SPEED_PENALTY_X_LO <= x <= SPEED_PENALTY_X_HI
    in_vert_reverse = VERT_REVERSE_X_LO <= x <= VERT_REVERSE_X_HI
    if not past_a:
        fy = gravity_comp_s2 + K_Y_CHECKPOINT * (CHECKPOINT_Y_CENTER - y)
    elif past_b:
        if in_vert_reverse:
            fy = -(gravity_comp_s2 + K_Y * (TARGET_Y_CENTER - y))
        else:
            fy = gravity_comp_s2 + K_Y * (TARGET_Y_CENTER - y)
    else:
        fy = gravity_comp_s2 + K_Y * (CHECKPOINT_B_Y_CENTER - y) * 1.4
    if WIND_ZONE_X_LO <= x <= WIND_ZONE_X_HI:
        fy -= WIND_FY_BASE + WIND_FY_AMP * math.sin(step_count * WIND_OMEGA)
    if x > TARGET_X_MAX:
        fx = (K_BRAKE * vx if vx > 0 else K_BRAKE * 2.0) if in_reverse else (-K_BRAKE * vx if vx > 0 else -K_BRAKE * 2.0)
        if not in_reverse and fx > 0: fx = 0
        if in_reverse and fx < 0: fx = 0
    elif x >= X_BRAKE_START:
        fx = K_BRAKE * vx if in_reverse else -K_BRAKE * vx
    elif x > REVERSE_THRUST_X_HI:
        fx = k_x_s2 * (TARGET_X_CENTER - x)
    elif past_b and x <= REVERSE_THRUST_X_HI:
        fx = -k_x_s2 * (TARGET_X_CENTER - x)
    elif past_b:
        fx = k_x_s2 * (TARGET_X_CENTER - x)
    elif past_a and x < REVERSE_THRUST_X_LO:
        fx = k_x_s2 * (CHECKPOINT_B_X_CENTER - x)
        if in_drain: fx *= (k_x_drain_s2 / k_x_s2)
    elif past_a:
        fx = -k_x_s2 * (CHECKPOINT_B_X_CENTER - x)
        if in_drain: fx *= (k_x_drain_s2 / k_x_s2)
        if in_speed_penalty and speed > SPEED_PENALTY_THRESHOLD * 0.8: fx *= SPEED_ZONE_FX_SCALE
    else:
        dx = CHECKPOINT_X_CENTER - x
        fx = k_x_s2 * dx
        if in_drain: fx *= (k_x_drain_s2 / k_x_s2)
        if 15.0 <= x < CHECKPOINT_X_LO and dx > 0: fx *= 1.4
    if in_oscillating_fx:
        fx -= OSCILLATING_FX_AMP * math.sin(step_count * OSCILLATING_FX_OMEGA)
    if in_thrust_scale:
        fx *= (1.0 / THRUST_SCALE_FACTOR)
        fy *= (1.0 / THRUST_SCALE_FACTOR)
    f = math.sqrt(fx * fx + fy * fy)
    if f > MAX_THRUST:
        scale = MAX_THRUST / f
        fx *= scale; fy *= scale
    sandbox.apply_thrust(fx, fy)

def build_agent_stage_3(sandbox):
    return None

def agent_action_stage_3(sandbox, agent_body, step_count):
    pos = sandbox.get_sled_position()
    if pos is None:
        return
    x, y = pos
    vx, vy = sandbox.get_sled_velocity() or (0.0, 0.0)
    past_a = sandbox.get_checkpoint_a_reached()
    past_b = sandbox.get_checkpoint_b_reached()
    if not past_a:
        target_x = CHECKPOINT_X_CENTER
        target_y = CHECKPOINT_Y_CENTER
        kx = 360.0
        ky = 520.0
    elif not past_b:
        target_x = CHECKPOINT_B_X_CENTER
        target_y = CHECKPOINT_B_Y_CENTER
        kx = 460.0
        ky = 460.0
    else:
        target_x = TARGET_X_CENTER
        target_y = TARGET_Y_CENTER
        kx = 520.0
        ky = 500.0
    delivered_fx = kx * (target_x - x) - 95.0 * vx
    delivered_fy = 455.0 + ky * (target_y - y) - 105.0 * vy
    if WIND_ZONE_X_LO <= x <= WIND_ZONE_X_HI:
        delivered_fy -= WIND_FY_BASE + WIND_FY_AMP * math.sin(step_count * WIND_OMEGA)
    fx = -delivered_fx if REVERSE_THRUST_X_LO <= x <= REVERSE_THRUST_X_HI else delivered_fx
    fy = -delivered_fy if VERT_REVERSE_X_LO <= x <= VERT_REVERSE_X_HI else delivered_fy
    if THRUST_SCALE_X_LO <= x <= THRUST_SCALE_X_HI:
        fx /= 0.12
        fy /= 0.12
    if OSCILLATING_FX_X_LO <= x <= OSCILLATING_FX_X_HI:
        fx -= OSCILLATING_FX_AMP * math.sin(step_count * OSCILLATING_FX_OMEGA)
    magnitude = math.sqrt(fx * fx + fy * fy)
    if magnitude > 2800.0:
        scale = 2800.0 / magnitude
        fx *= scale
        fy *= scale
    sandbox.apply_thrust(fx, fy)

def build_agent_stage_4(sandbox):
    return None

def agent_action_stage_4(sandbox, agent_body, step_count):
    pos = sandbox.get_sled_position()
    if pos is None: return
    x, y = pos
    vel = sandbox.get_sled_velocity() or (0.0, 0.0)
    vx, vy = vel[0], vel[1]
    speed = math.sqrt(vx * vx + vy * vy)
    MAX_THRUST_S4 = 3000.0
    GRAVITY_COMP_S4 = 450.0
    K_X_S4 = 400.0
    K_X_DRAIN_S4 = 2500.0
    K_Y_S4 = 250.0
    K_Y_CP_S4 = 300.0
    BRAKE_S4 = 120.0
    THRUST_SCALE_INV = 1.0 / 0.06
    SPEED_LIMIT_S4 = 1.2
    SPEED_ZONE_SCALE_S4 = 0.2
    past_a = sandbox.get_checkpoint_a_reached() if hasattr(sandbox, "get_checkpoint_a_reached") else (x >= CHECKPOINT_X_HI)
    past_b = sandbox.get_checkpoint_b_reached() if hasattr(sandbox, "get_checkpoint_b_reached") else (x >= CHECKPOINT_B_X_HI)
    in_drain = MOMENTUM_DRAIN_X_LO <= x <= MOMENTUM_DRAIN_X_HI
    in_reverse = REVERSE_THRUST_X_LO <= x <= REVERSE_THRUST_X_HI
    in_thrust_scale = THRUST_SCALE_X_LO <= x <= THRUST_SCALE_X_HI
    in_oscillating_fx = OSCILLATING_FX_X_LO <= x <= OSCILLATING_FX_X_HI
    in_speed_penalty = SPEED_PENALTY_X_LO <= x <= SPEED_PENALTY_X_HI
    in_vert_reverse = VERT_REVERSE_X_LO <= x <= VERT_REVERSE_X_HI
    if not past_a:
        fy = GRAVITY_COMP_S4 + K_Y_CP_S4 * (CHECKPOINT_Y_CENTER - y)
    elif past_b:
        if in_vert_reverse:
            fy = -(GRAVITY_COMP_S4 + K_Y_S4 * (TARGET_Y_CENTER - y))
        else:
            fy = GRAVITY_COMP_S4 + K_Y_S4 * (TARGET_Y_CENTER - y)
    else:
        fy = GRAVITY_COMP_S4 + K_Y_S4 * (CHECKPOINT_B_Y_CENTER - y) * 2.5
    if WIND_ZONE_X_LO <= x <= WIND_ZONE_X_HI:
        fy -= WIND_FY_BASE + WIND_FY_AMP * math.sin(step_count * WIND_OMEGA)
    if x > TARGET_X_MAX:
        fx = (BRAKE_S4 * vx if vx > 0 else BRAKE_S4 * 2.0) if in_reverse else (-BRAKE_S4 * vx if vx > 0 else -BRAKE_S4 * 2.0)
        if not in_reverse and fx > 0: fx = 0
        if in_reverse and fx < 0: fx = 0
    elif x >= X_BRAKE_START:
        fx = BRAKE_S4 * vx if in_reverse else -BRAKE_S4 * vx
    elif x > REVERSE_THRUST_X_HI:
        fx = K_X_S4 * (TARGET_X_CENTER - x)
    elif past_b and x <= REVERSE_THRUST_X_HI:
        fx = -K_X_S4 * (TARGET_X_CENTER - x)
    elif past_b:
        fx = K_X_S4 * (TARGET_X_CENTER - x)
    elif past_a and x < REVERSE_THRUST_X_LO:
        fx = K_X_S4 * (CHECKPOINT_B_X_CENTER - x)
        if in_drain: fx *= (K_X_DRAIN_S4 / K_X_S4)
    elif past_a:
        dx_b = CHECKPOINT_B_X_CENTER - x
        fx = -K_X_S4 * dx_b * 3.0
        if in_drain: fx *= (K_X_DRAIN_S4 / K_X_S4)
        if in_speed_penalty and speed > SPEED_LIMIT_S4 * 0.7:
            fx *= SPEED_ZONE_SCALE_S4
    else:
        dx = CHECKPOINT_X_CENTER - x
        fx = K_X_S4 * dx
        if in_drain: fx *= (K_X_DRAIN_S4 / K_X_S4)
        if 15.0 <= x < CHECKPOINT_X_LO and dx > 0: fx *= 1.4
    if in_oscillating_fx:
        fx -= OSCILLATING_FX_AMP * math.sin(step_count * OSCILLATING_FX_OMEGA)
    if in_thrust_scale:
        fx *= THRUST_SCALE_INV
        fy *= THRUST_SCALE_INV
    if in_speed_penalty and speed > SPEED_LIMIT_S4:
        fx *= 0.05
        fy *= 0.05
    f = math.sqrt(fx * fx + fy * fy)
    if f > MAX_THRUST_S4:
        scale = MAX_THRUST_S4 / f
        fx *= scale; fy *= scale
    sandbox.apply_thrust(fx, fy)
