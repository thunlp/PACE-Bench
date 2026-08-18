import math

def _build_agent_internal(sandbox, torque=3000.0, arm_config=None, initial_angle=0.0, beam_density=20.0, scoop_density=20.0, tower_density=400.0, bucket_torque=None):
    if bucket_torque is None:
        bucket_torque = torque
    sandbox.add_anchored_base(-2.0, 0.2, 0.4, 0.4, angle=0, density=10.0)
    tower = sandbox.add_anchored_base(-2.0, 0.75, 0.4, 1.5, angle=0, density=tower_density)
    if arm_config is None:
        arm_config = [1.5, 1.5, 1.0]
    prev_body = tower
    prev_anchor = (-2.0, 1.5)
    current_x, current_y = -2.0, 1.5
    arm_joint = None
    for i, length in enumerate(arm_config):
        center_x = current_x + (length / 2.0) * math.cos(initial_angle)
        center_y = current_y + (length / 2.0) * math.sin(initial_angle)
        body = sandbox.add_beam(center_x, center_y, length, 0.2, angle=initial_angle, density=beam_density)
        if i == 0:
            arm_joint = sandbox.add_revolute_joint(
                prev_body,
                body,
                prev_anchor,
                enable_motor=True,
                max_motor_torque=torque,
            )
        else:
            sandbox.add_joint(prev_body, body, (current_x, current_y))
        prev_body = body
        current_x += length * math.cos(initial_angle)
        current_y += length * math.sin(initial_angle)
    scoop_w, scoop_h = 2.0, 1.0
    if arm_joint is None:
        raise ValueError("arm_config must contain at least one segment")
    scoop = sandbox.add_scoop(current_x, current_y, scoop_w, scoop_h, angle=initial_angle, density=scoop_density)
    bucket_joint = sandbox.add_revolute_joint(
        prev_body,
        scoop,
        (current_x, current_y),
        enable_motor=True,
        max_motor_torque=bucket_torque,
    )
    return {
        "body": scoop,
        "arm_joint": arm_joint,
        "bucket_joint": bucket_joint,
    }


def _control_components(agent_components):
    if not isinstance(agent_components, dict):
        return None, None, None
    return (
        agent_components.get("body"),
        agent_components.get("arm_joint"),
        agent_components.get("bucket_joint"),
    )

def build_agent(sandbox):
    return _build_agent_internal(sandbox, torque=3000.0)

def agent_action(sandbox, agent_body, step_count):
    scoop, _aj, _bj = _control_components(agent_body)
    if scoop is None or not _aj or not _bj:
        return
    dt = 1.0 / 60.0
    t = step_count * dt
    phase_duration = 10.0
    phase = (t % phase_duration) / phase_duration
    arm_angle = _aj.angle
    bucket_world_angle = scoop.angle
    if phase < 0.3:
        ta, tb = -0.2, 0.5
    elif phase < 0.5:
        ta, tb = 0.0, 0.0
    elif phase < 0.8:
        ta, tb = 2.4, 0.0
    elif phase < 0.9:
        ta, tb = 2.4, 1.2
    else:
        ta, tb = 0.5, 1.2
    _aj.motorSpeed = 1.0 * (ta - arm_angle)
    _bj.motorSpeed = 3.0 * (tb - bucket_world_angle)
    _aj.motorEnabled = True
    _bj.motorEnabled = True

def build_agent_stage_1(sandbox):
    return _build_agent_internal(
        sandbox,
        torque=6000.0,
        arm_config=[1.5, 1.5, 1.0],
        tower_density=5.0,
        beam_density=2.0,
        scoop_density=3.0,
        bucket_torque=5000.0,
    )

def agent_action_stage_1(sandbox, agent_body, step_count):
    scoop, _aj, _bj = _control_components(agent_body)
    if scoop is None or not _aj or not _bj:
        return
    t = step_count * (1.0 / 60.0)
    phase_duration = 8.0
    phase = (t % phase_duration) / phase_duration
    arm_angle = _aj.angle
    bucket_world_angle = scoop.angle
    if phase < 0.20:
        ta, tb = -0.25, 0.55
    elif phase < 0.30:
        ta, tb = 0.0, -0.10
    elif phase < 0.60:
        ta, tb = 2.45, -0.10
    elif phase < 0.90:
        ta, tb = 2.45, 1.45
    else:
        ta, tb = 0.60, 1.30
    _aj.motorSpeed = 1.2 * (ta - arm_angle)
    _bj.motorSpeed = 2.5 * (tb - bucket_world_angle)
    _aj.motorEnabled = True
    _bj.motorEnabled = True

def build_agent_stage_2(sandbox):
    return _build_agent_internal(sandbox, torque=10000.0, tower_density=60.0,
                                 beam_density=6.0, scoop_density=6.0,
                                 arm_config=[1.5, 1.5, 1.0])

def agent_action_stage_2(sandbox, agent_body, step_count):
    scoop, _aj, _bj = _control_components(agent_body)
    if scoop is None or not _aj or not _bj:
        return
    dt = 1.0 / 60.0
    t = step_count * dt
    phase_duration = 1.6
    phase = (t % phase_duration) / phase_duration
    arm_angle = _aj.angle
    bucket_world_angle = scoop.angle
    if phase < 0.250:
        ta, tb = -0.35, 0.50
    elif phase < 0.400:
        ta, tb = 0.00, -0.15
    elif phase < 0.750:
        ta, tb = 2.40, -0.15
    elif phase < 0.880:
        ta, tb = 2.40, 1.40
    else:
        ta, tb = 0.50, 1.40
    _aj.motorSpeed = 3.0 * (ta - arm_angle)
    _bj.motorSpeed = 4.0 * (tb - bucket_world_angle)
    _aj.motorEnabled = True
    _bj.motorEnabled = True

def build_agent_stage_3(sandbox):
    return _build_agent_internal(
        sandbox,
        torque=21000.0,
        tower_density=60.0,
        beam_density=6.0,
        scoop_density=6.0,
        arm_config=[1.5, 1.5],
    )

def agent_action_stage_3(sandbox, agent_body, step_count):
    scoop, _aj, _bj = _control_components(agent_body)
    if scoop is None or not _aj or not _bj:
        return
    t = step_count * (1.0 / 60.0)
    phase_duration = 2.0
    phase = (t % phase_duration) / phase_duration
    arm_angle = _aj.angle
    bucket_world_angle = scoop.angle
    if phase < 0.250:
        ta, tb = -0.35, 0.50
    elif phase < 0.400:
        ta, tb = 0.00, -0.15
    elif phase < 0.750:
        ta, tb = 2.40, -0.15
    elif phase < 0.880:
        ta, tb = 2.40, 1.40
    else:
        ta, tb = 0.50, 1.25
    def _angle_diff(target, current):
        d = target - current
        d = (d + math.pi) % (2.0 * math.pi) - math.pi
        return d
    arm_err = _angle_diff(ta, arm_angle)
    bkt_err = _angle_diff(tb, bucket_world_angle)
    _aj.motorSpeed = 3.0 * arm_err
    _bj.motorSpeed = 4.0 * bkt_err
    _aj.motorEnabled = True
    _bj.motorEnabled = True

def build_agent_stage_4(sandbox):
    return _build_agent_internal(
        sandbox,
        torque=18000.0,
        arm_config=[1.5, 1.5],
        beam_density=5.0,
        scoop_density=5.0,
        tower_density=50.0,
    )

def agent_action_stage_4(sandbox, agent_body, step_count):
    scoop, _aj, _bj = _control_components(agent_body)
    if scoop is None or not _aj or not _bj:
        return
    t = step_count * (1.0 / 60.0)
    phase_duration = 1.8
    phase = (t % phase_duration) / phase_duration
    arm_angle = _aj.angle
    bucket_world_angle = scoop.angle
    if phase < 0.250:
        ta, tb = -0.35, 0.50
    elif phase < 0.400:
        ta, tb = 0.00, -0.20
    elif phase < 0.750:
        ta, tb = 2.50, -0.20
    elif phase < 0.880:
        ta, tb = 2.50, 1.40
    else:
        ta, tb = 0.50, 1.20
    _aj.motorSpeed = 3.0 * (ta - arm_angle)
    _bj.motorSpeed = 4.0 * (tb - bucket_world_angle)
    _aj.motorEnabled = True
    _bj.motorEnabled = True
