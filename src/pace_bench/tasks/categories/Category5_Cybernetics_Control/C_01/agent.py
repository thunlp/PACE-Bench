import math

CART_FORCE_LIMIT_NEWTONS = 450.0

INITIAL_REF_TRACK_CENTER_X = 10.0

def normalize_angle(theta):
    return math.atan2(math.sin(theta), math.cos(theta))

def build_agent(sandbox): return sandbox.get_cart_body()

def agent_action(sandbox, cart, step_count):
    theta = normalize_angle(sandbox.get_pole_angle())
    omega = sandbox.get_pole_angular_velocity()
    x, v = sandbox.get_cart_position(), sandbox.get_cart_velocity()
    target_x = INITIAL_REF_TRACK_CENTER_X
    force = -4200.0 * theta - 1050.0 * omega - 62.0 * (x - target_x) - 155.0 * v
    lim = CART_FORCE_LIMIT_NEWTONS
    sandbox.apply_cart_force(max(-lim, min(lim, force)))

def build_agent_stage_1(sandbox): return sandbox.get_cart_body()

def agent_action_stage_1(sandbox, cart, step_count):
    g, pole_len, delay_steps, F_lim = 10.0, 2.0, 10, 450.0
    target_x = sandbox.TRACK_CENTER_X
    theta = normalize_angle(sandbox.get_pole_angle())
    omega = sandbox.get_pole_angular_velocity()
    x = sandbox.get_cart_position()
    v = sandbox.get_cart_velocity()
    dt = 1.0 / 60.0

    history = list(getattr(cart, "_velocity_history_s1", ()))
    history.append(float(v))
    history = history[-(delay_steps + 1):]
    cart._velocity_history_s1 = history
    if len(history) == delay_steps + 1:
        for previous_v, next_v in zip(history, history[1:]):
            cart_acceleration = (next_v - previous_v) / dt
            angular_acceleration = (
                0.8 * (1.5 / pole_len) * math.cos(theta) * cart_acceleration
                + (1.5 * g / pole_len) * math.sin(theta)
            )
            omega += angular_acceleration * dt
            theta += omega * dt
    else:
        horizon = delay_steps * dt
        natural_rate = math.sqrt(1.5 * g / pole_len)
        argument = natural_rate * horizon
        ch, sh = math.cosh(argument), math.sinh(argument)
        theta, omega = (
            theta * ch + omega * sh / natural_rate,
            theta * natural_rate * sh + omega * ch,
        )

    pos_err = x - target_x
    force = -380.0 * theta - 15.0 * omega - 0.05 * pos_err - 0.5 * v

    sandbox.apply_cart_force(max(-F_lim, min(F_lim, force)))

def build_agent_stage_2(sandbox): return sandbox.get_cart_body()

def agent_action_stage_2(sandbox, cart, step_count):
    g, pole_len, m_p, m_c, delay_steps, F_lim = 12.0, 2.0, 1.0, 10.0, 10, 0.5
    safe_half = float(sandbox.SAFE_HALF_RANGE)
    target_x = sandbox.TRACK_CENTER_X
    theta = normalize_angle(sandbox.get_pole_angle())
    omega = sandbox.get_pole_angular_velocity()
    x = sandbox.get_cart_position()
    v = sandbox.get_cart_velocity()
    dt = 1.0 / 60.0

    M_tot = m_p + m_c
    denom = max((2.0 * pole_len / 3.0) * M_tot - m_p * pole_len / 2.0, 1e-6)
    omega0_sq = max(g * M_tot / denom, 1e-6)
    omega0 = math.sqrt(omega0_sq)

    T_delay = delay_steps * dt
    max_exp_arg = 20.0
    arg = min(omega0 * T_delay, max_exp_arg)
    ch = math.cosh(arg)
    sh = math.sinh(arg)

    theta_p = normalize_angle(theta * ch + omega / max(omega0, 1e-9) * sh)
    omega_p = theta * omega0 * sh + omega * ch

    if step_count < 30:
        theta_p = max(-0.15, min(0.15, theta_p))

    scale_g = g / 10.0
    scale_m = M_tot / 11.0

    k_theta = 130.0 * scale_g * scale_m
    k_omega = 25.0 * scale_g * scale_m
    force = -k_theta * theta_p - k_omega * omega_p

    if not hasattr(cart, '_angle_integral_s2'):
        cart._angle_integral_s2 = 0.0
    ki_angle = 200.0 * scale_g * scale_m
    cart._angle_integral_s2 += theta_p * 0.003
    cart._angle_integral_s2 = max(-0.08, min(0.08, cart._angle_integral_s2))
    force += -ki_angle * cart._angle_integral_s2

    pos_err = x - target_x
    k_pos = 0.15 * scale_g * scale_m
    k_vel = 0.5 * scale_g * scale_m
    force += -k_pos * pos_err - k_vel * v

    margin = safe_half - abs(pos_err)
    moving_outward = (pos_err > 0 and v > 0.001) or (pos_err < 0 and v < -0.001)
    if margin < safe_half * 0.45 and moving_outward:
        sandbox.apply_cart_force(-F_lim if pos_err > 0 else F_lim)
        return

    sandbox.apply_cart_force(max(-F_lim, min(F_lim, force)))

def build_agent_stage_3(sandbox): return sandbox.get_cart_body()

def agent_action_stage_3(sandbox, cart, step_count):
    g, m_p, m_c, delay_steps, F_lim = 22.0, 5.0, 3.0, 3, 4.0
    theta = normalize_angle(sandbox.get_pole_angle())
    omega = sandbox.get_pole_angular_velocity()
    x = sandbox.get_cart_position()
    v = sandbox.get_cart_velocity()
    target_x = sandbox.TRACK_CENTER_X
    safe_half = float(sandbox.SAFE_HALF_RANGE)
    dt = 1.0 / 60.0
    if step_count <= delay_steps:
        sandbox.apply_cart_force(0.0)
        return
    theta_p = normalize_angle(theta + omega * delay_steps * dt)
    scale_g = g / 10.0
    scale_m = (m_p + m_c) / 11.0
    force = (-250.0 * scale_g * scale_m * theta_p
             - 16.0 * scale_g * scale_m * omega)
    if not hasattr(cart, '_angle_integral_s3'):
        cart._angle_integral_s3 = 0.0
    ki_angle_eff = 400.0 * scale_g * scale_m
    cart._angle_integral_s3 += theta_p * 0.005
    cart._angle_integral_s3 = max(-0.05, min(0.05, cart._angle_integral_s3))
    force += -ki_angle_eff * cart._angle_integral_s3
    pos_err = x - target_x
    force += (-0.5 * scale_g * scale_m * pos_err
              - 1.0 * scale_g * scale_m * v)
    force = max(-F_lim, min(F_lim, force))
    sandbox.apply_cart_force(force)
    margin = safe_half - abs(pos_err)
    moving_outward = (pos_err > 0 and v > 0.01) or (pos_err < 0 and v < -0.01)
    if margin < safe_half * 0.30 and moving_outward:
        sandbox.apply_cart_force(-F_lim if pos_err > 0 else F_lim)

def build_agent_stage_4(sandbox): return sandbox.get_cart_body()

def agent_action_stage_4(sandbox, cart, step_count):
    g, pole_len, m_p, m_c, F_lim, delay_steps = 36.0, 0.5, 12.5, 1.95, 50.0, 3
    theta = normalize_angle(sandbox.get_pole_angle())
    omega = sandbox.get_pole_angular_velocity()
    x = sandbox.get_cart_position()
    v = sandbox.get_cart_velocity()
    target_x = sandbox.TRACK_CENTER_X
    dt = 1.0 / 60.0

    if step_count <= delay_steps:
        sandbox.apply_cart_force(0.0)
        return

    M_tot = m_p + m_c
    denom = max((2.0 * pole_len / 3.0) * M_tot - m_p * pole_len / 2.0, 1e-6)
    omega0_sq = max(g * M_tot / denom, 1e-6)
    omega0 = math.sqrt(omega0_sq)

    T_delay = delay_steps * dt
    max_exp_arg = 6.0
    arg = min(omega0 * T_delay, max_exp_arg)
    ch = math.cosh(arg)
    sh = math.sinh(arg)
    theta_p = theta * ch + omega / omega0 * sh

    scale_g = g / 10.0
    scale_m = M_tot / 11.0
    k_theta = 90.0 * scale_g * scale_m
    k_omega = 3.0 * scale_g * scale_m
    force = -k_theta * theta_p - k_omega * omega

    pos_err = x - target_x
    force += -0.45 * scale_g * scale_m * pos_err - 1.0 * scale_g * scale_m * v

    force = max(-F_lim, min(F_lim, force))
    sandbox.apply_cart_force(force)
