DT = 1.0 / 60.0

_integral_0 = 0.0

_omega_d_prev_0 = 0.0

_angle_est_0 = 0.0

def build_agent(sandbox):
    global _integral_0, _omega_d_prev_0, _angle_est_0
    _integral_0 = 0.0
    _omega_d_prev_0 = 0.0
    _angle_est_0 = 0.0
    return None

def agent_action(sandbox, agent_body, step_count):
    global _integral_0, _omega_d_prev_0, _angle_est_0
    import math
    omega_d = sandbox.get_wheel_angular_velocity()
    target = sandbox.get_target_speed()
    delay_est = 5.0
    kp = 18.0
    ki = 0.52
    base_ff = 1.94
    kff_quadratic = 0.528
    integral_clamp = 5.0
    integral_err_threshold = 1.5
    low_speed_threshold = 0.65
    stiction_boost = 2.05
    deadzone_min_torque = 2.25
    if step_count >= 1 and abs(omega_d - _omega_d_prev_0) < 0.25:
        omega_pred = omega_d + delay_est * (omega_d - _omega_d_prev_0)
        omega_pred = max(0.0, min(5.0, omega_pred))
    else:
        omega_pred = omega_d
    _omega_d_prev_0 = omega_d
    _angle_est_0 += omega_d * DT
    if abs(_angle_est_0) > 100.0:
        _angle_est_0 = math.fmod(_angle_est_0, 2.0 * math.pi)
    err = target - omega_pred
    if abs(err) < integral_err_threshold:
        _integral_0 += err * DT
        _integral_0 = max(-integral_clamp, min(integral_clamp, _integral_0))
    if abs(omega_d) < low_speed_threshold:
        torque = (1.0 if target >= 0 else -1.0) * 100.0
        if abs(omega_d) < 0.35:
            torque += stiction_boost if target >= 0 else -stiction_boost
    else:
        ff = base_ff + kff_quadratic * (omega_pred * omega_pred)
        torque = ff + kp * err + ki * _integral_0
        if abs(err) > 0.08 and abs(torque) > 1e-6 and abs(torque) < deadzone_min_torque:
            torque = (torque / abs(torque)) * deadzone_min_torque
    sandbox.apply_motor_torque(torque)

_integral_1 = 0.0

_omega_d_prev_1 = 0.0

_angle_est_1 = 0.0

def build_agent_stage_1(sandbox):
    global _integral_1, _omega_d_prev_1, _angle_est_1
    _integral_1 = 0.0
    _omega_d_prev_1 = 0.0
    _angle_est_1 = 0.0
    return None

def agent_action_stage_1(sandbox, agent_body, step_count):
    global _integral_1, _omega_d_prev_1, _angle_est_1
    import math
    omega_d = sandbox.get_wheel_angular_velocity()
    target = sandbox.get_target_speed()
    delay_est = 5.0
    kp = 28.0
    ki = 3.2
    base_ff = 2.0
    kff_quadratic = 3.0
    integral_clamp = 18.0
    integral_err_threshold = 2.5
    low_speed_threshold = 0.65
    stiction_boost = 2.8
    deadzone_min_torque = 2.25
    predict_delta_threshold = 0.25
    if step_count >= 1 and abs(omega_d - _omega_d_prev_1) < predict_delta_threshold:
        omega_pred = omega_d + delay_est * (omega_d - _omega_d_prev_1)
        omega_pred = max(0.0, min(6.0, omega_pred))
    else:
        omega_pred = omega_d
    _omega_d_prev_1 = omega_d
    _angle_est_1 += omega_d * DT
    if abs(_angle_est_1) > 100.0:
        _angle_est_1 = math.fmod(_angle_est_1, 2.0 * math.pi)
    err = target - omega_pred
    if abs(err) < integral_err_threshold:
        _integral_1 += err * DT
        _integral_1 = max(-integral_clamp, min(integral_clamp, _integral_1))
    if abs(omega_d) < low_speed_threshold:
        torque = (1.0 if target >= 0 else -1.0) * 100.0
        if abs(omega_d) < 0.35:
            torque += stiction_boost if target >= 0 else -stiction_boost
    else:
        ff = base_ff + kff_quadratic * (omega_pred * omega_pred)
        torque = ff + kp * err + ki * _integral_1
        if abs(err) > 0.08 and abs(torque) > 1e-6 and abs(torque) < deadzone_min_torque:
            torque = (torque / abs(torque)) * deadzone_min_torque
    sandbox.apply_motor_torque(torque)

_integral_2 = 0.0

_omega_d_prev_2 = 0.0

_angle_est_2 = 0.0

def build_agent_stage_2(sandbox):
    global _integral_2, _omega_d_prev_2, _angle_est_2
    _integral_2 = 0.0
    _omega_d_prev_2 = 0.0
    _angle_est_2 = 0.0
    return None

def agent_action_stage_2(sandbox, agent_body, step_count):
    global _integral_2, _omega_d_prev_2, _angle_est_2
    import math
    omega_d = sandbox.get_wheel_angular_velocity()
    target = sandbox.get_target_speed()
    delay_est = 5.0
    kp = 25.0
    ki = 2.0
    base_ff = 2.0
    kff_quadratic = 2.6
    integral_clamp = 8.0
    integral_err_threshold = 2.5
    low_speed_threshold = 0.65
    stiction_boost = 2.8
    deadzone_min_torque = 2.25
    cogging_est = 1.3
    predict_delta_threshold = 0.3
    if step_count >= 1 and abs(omega_d - _omega_d_prev_2) < predict_delta_threshold:
        omega_pred = omega_d + delay_est * (omega_d - _omega_d_prev_2)
        omega_pred = max(0.0, min(6.0, omega_pred))
    else:
        omega_pred = omega_d
    _omega_d_prev_2 = omega_d
    _angle_est_2 += omega_d * DT
    if abs(_angle_est_2) > 100.0:
        _angle_est_2 = math.fmod(_angle_est_2, 2.0 * math.pi)
    err = target - omega_pred
    if abs(err) < integral_err_threshold:
        _integral_2 += err * DT
        _integral_2 = max(-integral_clamp, min(integral_clamp, _integral_2))
    if abs(omega_d) < low_speed_threshold:
        torque = (1.0 if target >= 0 else -1.0) * 100.0
        if abs(omega_d) < 0.35:
            torque += stiction_boost if target >= 0 else -stiction_boost
    else:
        ff = base_ff + kff_quadratic * (omega_pred * omega_pred)
        torque = ff + kp * err + ki * _integral_2
        phase_advance = delay_est * DT * max(0, omega_pred)
        torque += cogging_est * math.sin(_angle_est_2 + phase_advance)
        if abs(err) > 0.08 and abs(torque) > 1e-6 and abs(torque) < deadzone_min_torque:
            torque = (torque / abs(torque)) * deadzone_min_torque
    sandbox.apply_motor_torque(torque)

_integral_3 = 0.0

_omega_d_prev_3 = 0.0

_angle_est_3 = 0.0

_prev_target_3 = 3.0

def build_agent_stage_3(sandbox):
    global _integral_3, _omega_d_prev_3, _angle_est_3, _prev_target_3
    _integral_3 = 0.0
    _omega_d_prev_3 = 0.0
    _angle_est_3 = 0.0
    _prev_target_3 = 3.0
    return None

def agent_action_stage_3(sandbox, agent_body, step_count):
    global _integral_3, _omega_d_prev_3, _angle_est_3, _prev_target_3
    import math
    omega_d = sandbox.get_wheel_angular_velocity()
    target = sandbox.get_target_speed()
    if abs(target - _prev_target_3) > 0.01:
        _integral_3 *= 0.5
        _prev_target_3 = target
    delay_est = 6.5
    cogging_est = 2.2
    kp = 22.0
    ki = 3.8
    base_ff = 2.0
    kff_quadratic = 2.35
    integral_clamp = 14.0
    integral_err_threshold = 2.5
    low_speed_threshold = 0.75
    stiction_boost = 2.8
    deadzone_min_torque = 2.8
    predict_delta_threshold = 0.22
    if step_count >= 1 and abs(omega_d - _omega_d_prev_3) < predict_delta_threshold:
        omega_pred = omega_d + delay_est * (omega_d - _omega_d_prev_3)
        omega_pred = max(0.0, min(6.0, omega_pred))
    else:
        omega_pred = omega_d
    _omega_d_prev_3 = omega_d
    _angle_est_3 += omega_d * DT
    if abs(_angle_est_3) > 100.0:
        _angle_est_3 = math.fmod(_angle_est_3, 2.0 * math.pi)
    err = target - omega_pred
    if abs(err) < integral_err_threshold:
        _integral_3 += err * DT
        _integral_3 = max(-integral_clamp, min(integral_clamp, _integral_3))
    if abs(omega_d) < low_speed_threshold:
        torque = (1.0 if target >= 0 else -1.0) * 100.0
        if abs(omega_d) < 0.35:
            torque += stiction_boost if target >= 0 else -stiction_boost
    else:
        ff = base_ff + kff_quadratic * (omega_pred * omega_pred)
        torque = ff + kp * err + ki * _integral_3
        phase_advance = delay_est * DT * max(0, omega_pred)
        torque += cogging_est * math.sin(_angle_est_3 + phase_advance)
        if abs(err) > 0.08 and abs(torque) > 1e-6 and abs(torque) < deadzone_min_torque:
            torque = (torque / abs(torque)) * deadzone_min_torque
    sandbox.apply_motor_torque(torque)

_integral_4 = 0.0

_omega_d_prev_4 = 0.0

_angle_est_4 = 0.0

def build_agent_stage_4(sandbox):
    global _integral_4, _omega_d_prev_4, _angle_est_4
    _integral_4 = 0.0
    _omega_d_prev_4 = 0.0
    _angle_est_4 = 0.0
    return None

def agent_action_stage_4(sandbox, agent_body, step_count):
    global _integral_4, _omega_d_prev_4, _angle_est_4
    import math
    omega_d = sandbox.get_wheel_angular_velocity()
    target = sandbox.get_target_speed()
    delay_est = 6.5
    cogging_est = 2.5
    kp = 15.5
    ki = 1.9
    base_ff = 2.6
    kff_quadratic = 0.77
    integral_clamp = 16.0
    integral_err_threshold = 5.0
    low_speed_threshold = 0.65
    stiction_boost = 2.5
    deadzone_min_torque = 2.7
    if step_count >= 1 and abs(omega_d - _omega_d_prev_4) < 0.25:
        omega_pred = omega_d + delay_est * (omega_d - _omega_d_prev_4)
        omega_pred = max(0.0, min(6.0, omega_pred))
    else:
        omega_pred = omega_d
    _omega_d_prev_4 = omega_d
    _angle_est_4 += omega_d * DT
    if abs(_angle_est_4) > 100.0:
        _angle_est_4 = math.fmod(_angle_est_4, 2.0 * math.pi)
    err = target - omega_pred
    if abs(err) < integral_err_threshold:
        _integral_4 += err * DT
        _integral_4 = max(-integral_clamp, min(integral_clamp, _integral_4))
    if abs(omega_d) < low_speed_threshold:
        torque = (1.0 if target >= 0 else -1.0) * 100.0
        if abs(omega_d) < 0.35:
            torque += stiction_boost if target >= 0 else -stiction_boost
    else:
        ff = base_ff + kff_quadratic * (omega_pred * omega_pred)
        torque = ff + kp * err + ki * _integral_4
        phase_advance = delay_est * DT * max(0, omega_pred)
        torque += cogging_est * math.sin(_angle_est_4 + phase_advance)
        if abs(err) > 0.08 and abs(torque) > 1e-6 and abs(torque) < deadzone_min_torque:
            torque = (torque / abs(torque)) * deadzone_min_torque
    sandbox.apply_motor_torque(torque)
