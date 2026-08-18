import math

_DT = 1.0 / 60.0
_LANDER_MASS = 50.0
_MAX_TORQUE = 120.0


def _platform_kinematics(t):
    base = 17.0
    amp = 1.8
    per = 6.0
    if per <= 1e-6:
        per = 6.0
    w = 2.0 * math.pi / per
    plat_x = base + amp * math.sin(w * t)
    plat_vx = amp * w * math.cos(w * t)
    return plat_x, plat_vx

def _corridor_target_altitude(yt, yb, clearance_obstacle=2.5, clearance_ceiling=1.5):
    y_lo = yt + clearance_obstacle
    y_hi = yb - clearance_ceiling
    if y_hi <= y_lo:
        return 0.5 * (yt + yb)
    inner = y_hi - y_lo
    y_pref = y_lo + 0.35 * inner
    return max(y_lo, min(y_pref, y_hi))


def _defer_command_one_step(sandbox, agent, thrust, torque):
    prior_thrust, prior_torque = agent["deferred_command"]
    sandbox.apply_thrust(prior_thrust, prior_torque)
    agent["deferred_command"] = (thrust, torque)


def _build_precision_agent(sandbox, terminal_target_vy, low_altitude_tilt):
    return {
        "body": sandbox.get_lander_body(),
        "phase": "transit",
        "vertical_integral": 0.0,
        "horizontal_integral": 0.0,
        "last_thrust": 500.0,
        "terminal_target_vy": terminal_target_vy,
        "low_altitude_tilt": low_altitude_tilt,
    }


def build_agent(sandbox):
    return _build_precision_agent(sandbox, -0.30, 0.17)


def agent_action(sandbox, agent, step_count):
    _precision_lander_action(sandbox, agent, step_count)


def build_agent_stage_1(sandbox):
    return _build_precision_agent(sandbox, -0.035, 0.185)


def agent_action_stage_1(sandbox, agent, step_count):
    _precision_lander_action(sandbox, agent, step_count)


def _precision_lander_action(sandbox, agent, step_count):
    body = agent["body"]
    dt = 1.0 / 60.0
    x, y = sandbox.get_lander_position()
    vx = float(body.linearVelocity.x)
    vy = float(body.linearVelocity.y)
    angle = sandbox.get_lander_angle()
    omega = sandbox.get_lander_angular_velocity()
    delay_steps = sandbox.get_thrust_delay_steps()
    delay_t = delay_steps * dt
    ground_y = sandbox.get_ground_y_top()
    _half_width, half_height = sandbox.get_lander_size()
    clearance = y - ground_y - half_height

    if agent["phase"] == "transit" and x >= 14.0:
        agent["phase"] = "rendezvous"

    if agent["phase"] == "transit":
        target_x = 15.0
        target_vx = 2.2
        target_y = 12.0
        target_vy = 0.0
        vertical_acceleration = 4.0 * (target_y - y) + 7.0 * (target_vy - vy)
        horizontal_acceleration = 1.8 * (target_x - x) + 3.5 * (target_vx - vx)
        max_tilt = 0.26
    else:
        command_time = (step_count + delay_steps) * dt + 0.30
        platform_omega = 2.0 * math.pi / 6.0
        target_x = 17.0 + 1.8 * math.sin(platform_omega * command_time)
        target_vx = 1.8 * platform_omega * math.cos(platform_omega * command_time)

        if clearance > 6.0:
            target_vy = -3.0
        elif clearance > 3.0:
            target_vy = -2.5
        elif clearance > 1.5:
            target_vy = -1.5
        elif clearance > 0.80:
            target_vy = -0.60
        elif clearance > 0.24:
            target_vy = -0.50
        else:
            target_vy = agent["terminal_target_vy"]

        queued_vertical_acceleration = (
            agent["last_thrust"] * math.cos(angle) / 50.0 - 10.0
        )
        predicted_vy = vy + queued_vertical_acceleration * delay_t
        agent["vertical_integral"] += (target_vy - predicted_vy) * dt
        agent["vertical_integral"] = max(
            -0.25, min(0.25, agent["vertical_integral"])
        )
        if clearance <= 0.24:
            agent["vertical_integral"] = 0.0
        vertical_acceleration = (
            5.0 * (target_vy - predicted_vy)
            + 1.2 * agent["vertical_integral"]
        )

        predicted_x = x + vx * delay_t
        horizontal_error = target_x - predicted_x
        agent["horizontal_integral"] += horizontal_error * dt
        agent["horizontal_integral"] = max(
            -0.8, min(0.8, agent["horizontal_integral"])
        )
        horizontal_acceleration = (
            2.0 * horizontal_error
            + 4.0 * (target_vx - vx)
            + 0.35 * agent["horizontal_integral"]
        )
        if clearance > 2.0:
            max_tilt = 0.24
        else:
            max_tilt = agent["low_altitude_tilt"]

    target_angle = max(
        -max_tilt, min(max_tilt, -math.atan2(horizontal_acceleration, 10.0))
    )
    if clearance < 1.0:
        low_tilt = agent["low_altitude_tilt"]
        target_angle = max(-low_tilt, min(low_tilt, target_angle))
        angle_kp, angle_kd = 100.0, 40.0
    else:
        angle_kp, angle_kd = 100.0, 24.0
    torque = angle_kp * (target_angle - angle) - angle_kd * omega
    torque = max(-120.0, min(120.0, torque))

    thrust = 50.0 * (10.0 + vertical_acceleration)
    thrust /= max(0.85, math.cos(angle))
    thrust = max(0.0, min(600.0, thrust))
    agent["last_thrust"] = thrust
    sandbox.apply_thrust(thrust, torque)

def build_agent_stage_2(sandbox):
    return {
        "body": sandbox.get_lander_body(),
        "vertical_integral": 0.0,
        "deferred_command": (0.0, 0.0),
    }

def agent_action_stage_2(sandbox, agent, step_count):
    body = agent["body"]
    dt = _DT
    ds = sandbox.get_thrust_delay_steps()
    g = 10.0
    pos, vel = body.position, body.linearVelocity
    x, y, vx, vy = pos.x, pos.y, vel.x, vel.y
    angle, omega = body.angle, body.angularVelocity
    max_thrust, max_torque = 600.0, _MAX_TORQUE
    m = _LANDER_MASS
    plat_x, plat_vx = _platform_kinematics((step_count + ds) * dt + 0.15)
    xl, xr, yt, yb = 10.5, 13.5, 6.0, 7.0
    corridor_centre = 0.5 * (yt + yb)
    in_barrier = (x >= xl - 0.9 and x <= xr + 0.8)
    if in_barrier:
        agent["vertical_integral"] += (corridor_centre - y) * dt
        agent["vertical_integral"] = max(-0.5, min(0.5, agent["vertical_integral"]))
    else:
        agent["vertical_integral"] = 0.0
    if in_barrier:
        tx = xr + 3.0
        tvx = 0.8
        ty = corridor_centre
        tvy = 0.0
        ay = 35.0 * (ty - y) + 55.0 * (tvy - vy) + 20.0 * agent["vertical_integral"]
        thrust = m * (g + ay) / max(0.3, math.cos(angle))
        thrust = max(0.0, min(max_thrust, thrust))
        target_angle = 0.0
        torque = 300.0 * (target_angle - angle) - 80.0 * omega
        torque = max(-max_torque, min(max_torque, torque))
        _defer_command_one_step(sandbox, agent, thrust, torque)
        return
    if x < xl - 0.9:
        altitude_error = y - corridor_centre
        ready_for_corridor = abs(altitude_error) <= 0.08 and abs(vy) <= 0.25
        hover = m * g
        if not ready_for_corridor:
            tx = xl - 2.5; tvx = -0.2
            ty = corridor_centre
            tvy = 0.0
        else:
            tx = xl + 2.0; tvx = 3.0
            ty = corridor_centre; tvy = 0.0
        if altitude_error > 0.2:
            upward_acceleration = max(0.1, max_thrust / m - g)
            stopping_distance = max(0.0, -vy) ** 2 / (2.0 * upward_acceleration)
            thrust = max_thrust if stopping_distance + 1.95 >= altitude_error else 0.0
        else:
            thrust = (
                hover + m * 6.0 * (tvy - vy) + m * 3.0 * (ty - y)
            ) / max(0.3, math.cos(angle))
        thrust = max(0.0, min(max_thrust, thrust))
        ax = 1.5 * (tx - x) + 3.0 * (tvx - vx)
        target_angle = max(-0.10, min(0.10, -0.05 * ax))
        torque = 30.0 * (target_angle - angle) - 6.0 * omega
        torque = max(-max_torque, min(max_torque, torque))
        _defer_command_one_step(sandbox, agent, thrust, torque)
        return
    tx = plat_x - 2.0
    tvx = plat_vx
    ty = 1.05
    if y > 3.5:
        tvy = -2.0
        vertical_position_gain = 0.0
        vertical_velocity_gain = 4.0
    else:
        tvy = -2.0
        vertical_position_gain = 1.0
        vertical_velocity_gain = 4.0
    ay = vertical_position_gain * (ty - y) + vertical_velocity_gain * (tvy - vy)
    thrust = m * (g + ay)
    if y < 2.0:
        thrust += 75.0
    thrust = max(0.0, min(max_thrust, thrust))
    ax = 3.0 * (tx - x) + 6.0 * (tvx - vx)
    target_angle = max(-0.15, min(0.15, -0.06 * ax))
    if y < 2.2:
        target_angle = 0.15
    elif y < 4.0:
        target_angle = 0.12
    torque = 80.0 * (target_angle - angle) - 20.0 * omega
    torque = max(-max_torque, min(max_torque, torque))
    _defer_command_one_step(sandbox, agent, thrust, torque)

def build_agent_stage_3(sandbox):
    return {
        "body": sandbox.get_lander_body(),
        "vertical_integral": 0.0,
        "horizontal_integral": 0.0,
        "deferred_command": (0.0, 0.0),
    }

def agent_action_stage_3(sandbox, agent, step_count):
    body = agent["body"]
    dt = _DT
    ds = sandbox.get_thrust_delay_steps()
    g = 11.8 if step_count >= 250 else 10.0
    pos, vel = body.position, body.linearVelocity
    x, y, vx, vy = pos.x, pos.y, vel.x, vel.y
    angle, omega = body.angle, body.angularVelocity
    max_thrust, max_torque = 650.0, _MAX_TORQUE
    m = _LANDER_MASS
    plat_x, plat_vx = _platform_kinematics((step_count + ds) * dt + 0.15)
    pred_t = ds * dt
    yp = y + vy * pred_t + 0.5 * (-g) * pred_t * pred_t
    vyp = vy - g * pred_t
    xp = x + vx * pred_t
    xl, xr, yt, yb = 10.5, 13.5, 6.0, 8.5
    corridor_centre = 0.5 * (yt + yb)
    in_barrier = (x >= xl - 0.9 and x <= xr + 0.8)
    if in_barrier:
        vert_err = corridor_centre - y
        agent["vertical_integral"] += vert_err * dt
        agent["vertical_integral"] = max(-0.5, min(0.5, agent["vertical_integral"]))
    else:
        agent["vertical_integral"] = 0.0
    if in_barrier:
        tx = xr + 2.0
        tvx = 1.0
        ty = corridor_centre
        tvy = 0.0
        ay = 25.0 * (ty - y) + 40.0 * (tvy - vy) + 15.0 * agent["vertical_integral"]
        thrust = m * (g + ay) / max(0.3, math.cos(angle))
        thrust = max(0.0, min(max_thrust, thrust))
        ax = 1.5 * (tx - x) + 3.0 * (tvx - vx)
        target_angle = max(-0.03, min(0.03, -0.02 * ax))
        torque = 60.0 * (target_angle - angle) - 8.0 * omega
        torque = max(-max_torque, min(max_torque, torque))
    elif x < xl - 0.9:
        tx = xl + 1.0
        ty = corridor_centre
        tvy = -1.0
        ay = 4.0 * (ty - y) + 8.0 * (tvy - vy)
        thrust = m * (g + ay) / max(0.4, math.cos(angle))
        thrust = max(0.0, min(max_thrust, thrust))
        ax = 1.5 * (tx - x) + 3.0 * (1.5 - vx)
        target_angle = max(-0.08, min(0.08, -0.04 * ax))
        torque = 45.0 * (target_angle - angle) - 6.0 * omega
        torque = max(-max_torque, min(max_torque, torque))
    else:
        alt = max(0.0, y - 1.0)
        if alt > 4.0:
            tx = plat_x - 5.0
            tvx = plat_vx
        elif alt > 2.5:
            blend = (4.0 - alt) / 1.5
            tx = plat_x - 5.0 * (1.0 - blend)
            tvx = plat_vx
        elif alt > 1.5:
            tx = plat_x - 1.0
            tvx = plat_vx
        else:
            tx = plat_x
            tvx = plat_vx
        if alt > 2.0:
            agent["horizontal_integral"] += (tx - x) * dt
            agent["horizontal_integral"] = max(-5.0, min(5.0, agent["horizontal_integral"]))
        else:
            agent["horizontal_integral"] = 0.0
        ax = 4.0 * (tx - x) + 8.0 * (tvx - vx) + 2.0 * agent["horizontal_integral"]
        if alt > 2.0:
            target_angle = max(-0.08, min(0.08, -0.05 * ax))
        elif alt > 0.5:
            blend_ang = (alt - 0.5) / 1.5
            raw = max(-0.04, min(0.04, -0.03 * ax))
            target_angle = blend_ang * raw
        else:
            target_angle = 0.0
        hover_thrust = m * g
        if alt > 4.0:
            ff_thrust = hover_thrust - 75.0
            tvy = -2.5;  v_kd = 4.0
        elif alt > 3.0:
            ff_thrust = hover_thrust + 10.0
            tvy = -1.8;  v_kd = 6.0
        elif alt > 2.0:
            ff_thrust = hover_thrust + 30.0
            tvy = -1.2;  v_kd = 8.0
        else:
            if alt > 1.5:
                ff_thrust = hover_thrust
                tvy = -0.10;  v_kd = 15.0
            elif alt > 1.0:
                ff_thrust = hover_thrust + 35.0
                tvy = -0.05;  v_kd = 22.0
            elif alt > 0.5:
                ff_thrust = hover_thrust + 55.0
                tvy = -0.02;  v_kd = 30.0
            elif alt > 0.2:
                ff_thrust = max_thrust
                tvy = -0.01;  v_kd = 36.0
            else:
                ff_thrust = max_thrust
                tvy = -0.005;  v_kd = 42.0
        if alt > 2.0:
            pd_ay = v_kd * (tvy - vy)
            thrust = (ff_thrust + m * pd_ay) / max(0.4, math.cos(angle))
            tkp, tkd = 60.0, 8.0
        else:
            pd_ay = v_kd * (tvy - vy)
            thrust = (ff_thrust + m * pd_ay) / max(0.3, math.cos(angle))
            if alt > 0.8:
                tkp, tkd = 150.0, 18.0
            else:
                tkp, tkd = 300.0, 35.0
        thrust = max(0.0, min(max_thrust, thrust))
        torque = tkp * (target_angle - angle) - tkd * omega
        torque = max(-max_torque, min(max_torque, torque))
    _defer_command_one_step(sandbox, agent, thrust, torque)

def build_agent_stage_4(sandbox):
    return {
        "body": sandbox.get_lander_body(),
        "deferred_command": (0.0, 0.0),
    }

def agent_action_stage_4(sandbox, agent, step_count):
    body = agent["body"]
    dt = _DT
    delay_t = 0.22
    g = 11.5 if step_count >= 150 else 10.0
    pos, vel = body.position, body.linearVelocity
    x, y, vx, vy = pos.x, pos.y, vel.x, vel.y
    angle, omega = body.angle, body.angularVelocity
    angle = angle + omega * delay_t
    vx_p = vx
    vy_p = vy - g * delay_t
    x_p = x + vx * delay_t
    y_p = y + vy * delay_t - 0.5 * g * delay_t**2
    ds = sandbox.get_thrust_delay_steps()
    plat_x, plat_vx = _platform_kinematics((step_count + ds) * dt + 0.15)
    xr, yt, yb = 13.5, 6.0, 15.5
    if x < xr + 1.7:
        tx = xr + 3.0
        ty = max(yt + 2.5, min(13.5, yb - 1.5))
        tvx, tvy = 0.4, 0.0
    else:
        tx, ty, tvx, tvy = plat_x, 1.05, plat_vx, -0.2
        if y < 4.0:
            tvy = -0.10
            tvx = plat_vx
    m = _LANDER_MASS
    ay = 15.0 * (ty - y_p) + 20.0 * (tvy - vy_p)
    acc_y = max(3.0, g + ay)
    thrust = m * acc_y / max(0.1, math.cos(angle))
    max_thrust, max_torque = 1200.0, _MAX_TORQUE
    thrust = max(0.0, min(max_thrust, thrust))
    ax = 2.0 * (tx - x_p) + 6.0 * (tvx - vx_p)
    target_angle = max(-0.35, min(0.35, -0.1 * ax))
    if y < 3.0: target_angle = 0.0
    tkp, tkd = 4000.0, 1200.0
    if y < 3.0:
        tkp, tkd = 2000.0, 800.0
    torque = tkp * (target_angle - angle) - tkd * omega
    torque = max(-max_torque, min(max_torque, torque))
    _defer_command_one_step(sandbox, agent, thrust, torque)
