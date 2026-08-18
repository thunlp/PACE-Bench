import math


def _get_joints(agent_components):
    if isinstance(agent_components, dict):
        return agent_components.get("joints", [])
    return []


def build_agent(sandbox):
    start_x = 10.0
    start_y = 2.5
    torso_width = 2.0
    torso_height = 0.5
    torso_density = 2.0
    torso = sandbox.add_beam(
        x=start_x, y=start_y, width=torso_width, height=torso_height, angle=0, density=torso_density
    )
    sandbox.set_material_properties(torso, restitution=0.1, friction=0.5)
    leg_length = 1.0
    leg_width = 0.1
    leg_density = 1.0
    num_legs_per_wheel = 6
    joints = []
    wheel_center_x = start_x - 1.0
    wheel_center_y = start_y
    for i in range(num_legs_per_wheel):
        angle = i * 2 * math.pi / num_legs_per_wheel
        leg_x = wheel_center_x + math.cos(angle) * leg_length / 2
        leg_y = wheel_center_y + math.sin(angle) * leg_length / 2
        leg = sandbox.add_beam(x=leg_x, y=leg_y, width=leg_width, height=leg_length, angle=angle, density=leg_density)
        sandbox.set_material_properties(leg, restitution=0.1, friction=0.8)
        pivot = sandbox.add_joint(torso, leg, (wheel_center_x, wheel_center_y), type='pivot')
        joints.append(pivot)
    wheel_center_x = start_x + 1.0
    wheel_center_y = start_y
    for i in range(num_legs_per_wheel):
        angle = i * 2 * math.pi / num_legs_per_wheel
        leg_x = wheel_center_x + math.cos(angle) * leg_length / 2
        leg_y = wheel_center_y + math.sin(angle) * leg_length / 2
        leg = sandbox.add_beam(x=leg_x, y=leg_y, width=leg_width, height=leg_length, angle=angle, density=leg_density)
        sandbox.set_material_properties(leg, restitution=0.1, friction=0.8)
        pivot = sandbox.add_joint(torso, leg, (wheel_center_x, wheel_center_y), type='pivot')
        joints.append(pivot)
    return {"body": torso, "joints": joints}

def agent_action(sandbox, agent_body, step_count):
    joints = _get_joints(agent_body)
    torso = agent_body.get("body") if isinstance(agent_body, dict) else None
    if torso is not None and torso.position.x > 28.0:
        rotation_speed = 0.0
    else:
        rotation_speed = -25.0
    for j in joints:
        sandbox.set_motor(j, rotation_speed, 2000.0)

def build_agent_stage_1(sandbox):
    start_x = 10.0
    start_y = 2.0
    torso = sandbox.add_beam(
        x=start_x, y=start_y, width=1.35, height=0.12, angle=0, density=0.18
    )
    sandbox.set_material_properties(torso, restitution=0.0, friction=0.08)
    sandbox.set_fixed_rotation(torso, True)
    joints = []
    for wheel_x in (start_x - 0.52, start_x + 0.52):
        wheel = sandbox.add_wheel(wheel_x, 2.0, radius=0.55, density=0.075)
        sandbox.set_material_properties(wheel, restitution=0.0, friction=0.08)
        joints.append(
            sandbox.add_joint(
                torso,
                wheel,
                (wheel_x, 2.0),
                type='pivot',
                lower_limit=-math.pi,
                upper_limit=None,
            )
        )
    return {"body": torso, "joints": joints}

def agent_action_stage_1(sandbox, agent_body, step_count):
    joints = agent_body.get("joints", []) if isinstance(agent_body, dict) else []
    torso = agent_body.get("body") if isinstance(agent_body, dict) else None
    speed = 0.0 if torso is not None and torso.position.x >= 25.4 else -14.0
    for joint in joints:
        sandbox.set_motor(joint, speed, 45.0)

def build_agent_stage_2(sandbox):
    start_x = 10.0
    start_y = 2.8
    leg_length = 0.7
    leg_width = 0.1
    torso = sandbox.add_beam(x=start_x, y=start_y, width=1.8, height=0.4, density=1.6)
    sandbox.set_material_properties(torso, restitution=0.05, friction=1.0)
    joints = []
    for cx in (start_x - 0.9, start_x + 0.9):
        for i in range(8):
            angle = i * 2 * math.pi / 8
            lx = cx + math.cos(angle) * leg_length / 2
            ly = start_y + math.sin(angle) * leg_length / 2
            leg = sandbox.add_beam(x=lx, y=ly, width=leg_width, height=leg_length, angle=angle, density=0.7)
            sandbox.set_material_properties(leg, restitution=0.05, friction=1.0)
            pivot = sandbox.add_joint(
                torso,
                leg,
                (cx, start_y),
                type='pivot',
                lower_limit=-math.pi,
                upper_limit=None,
            )
            joints.append(pivot)
    return {"body": torso, "joints": joints}

def agent_action_stage_2(sandbox, agent_body, step_count):
    limit_lo, limit_hi = -math.pi / 12, math.pi / 12
    margin = 0.04
    speed, torque = 5.0, 2.5
    for j in _get_joints(agent_body):
        a = j.angle
        if a >= limit_hi - margin:
            sandbox.set_motor(j, -speed, torque)
        else:
            sandbox.set_motor(j, speed, torque)

def build_agent_stage_3(sandbox):
    start_x = 10.0
    start_y = 3.5
    torso = sandbox.add_beam(
        x=start_x, y=start_y, width=1.4, height=0.2, density=1.0
    )
    sandbox.set_material_properties(torso, restitution=0.0, friction=0.2)
    sandbox.set_fixed_rotation(torso, True)
    joints = []
    for hip_x in (start_x - 0.4, start_x + 0.4):
        thigh = sandbox.add_beam(
            x=hip_x, y=start_y - 0.5, width=0.06, height=1.0, density=0.3
        )
        shin = sandbox.add_beam(
            x=hip_x, y=start_y - 1.5, width=0.06, height=1.0, density=0.3
        )
        sandbox.set_material_properties(thigh, restitution=0.0, friction=0.2)
        sandbox.set_material_properties(shin, restitution=0.0, friction=0.2)
        hip = sandbox.add_joint(
            torso,
            thigh,
            (hip_x, start_y),
            type='pivot',
            lower_limit=-0.7,
            upper_limit=0.7,
        )
        knee = sandbox.add_joint(
            thigh,
            shin,
            (hip_x, start_y - 1.0),
            type='pivot',
            lower_limit=0.0,
            upper_limit=1.2,
        )
        joints.extend((hip, knee))
    return {"body": torso, "joints": joints}

def agent_action_stage_3(sandbox, agent_body, step_count):
    joints = _get_joints(agent_body)
    cycle_steps = 60
    for leg_index in range(2):
        hip = joints[2 * leg_index]
        knee = joints[2 * leg_index + 1]
        phase = ((step_count + leg_index * cycle_steps // 2) % cycle_steps) / cycle_steps
        front_angle, rear_angle = 0.6, -0.6
        if phase < 0.20:
            hip_target, knee_target = rear_angle, 1.1
        elif phase < 0.45:
            swing_progress = (phase - 0.20) / 0.25
            hip_target = rear_angle + (front_angle - rear_angle) * swing_progress
            knee_target = 1.1
        elif phase < 0.55:
            hip_target, knee_target = front_angle, 0.0
        else:
            stance_progress = (phase - 0.55) / 0.45
            hip_target = front_angle + (rear_angle - front_angle) * stance_progress
            knee_target = 0.0
        hip_speed = max(-5.0, min(5.0, 12.0 * (hip_target - hip.angle)))
        knee_speed = max(-5.0, min(5.0, 12.0 * (knee_target - knee.angle)))
        for joint, speed in ((hip, hip_speed), (knee, knee_speed)):
            sandbox.set_motor(joint, speed, 30.0)

def build_agent_stage_4(sandbox):
    start_x = 10.0
    start_y = 3.5
    torso = sandbox.add_beam(
        x=start_x, y=start_y, width=1.4, height=0.2, density=1.0
    )
    sandbox.set_material_properties(torso, restitution=0.0, friction=0.008)
    sandbox.set_fixed_rotation(torso, True)
    joints = []
    for hip_x in (start_x - 0.4, start_x + 0.4):
        thigh = sandbox.add_beam(
            x=hip_x, y=start_y - 0.5, width=0.06, height=1.0, density=0.3
        )
        shin = sandbox.add_beam(
            x=hip_x, y=start_y - 1.5, width=0.06, height=1.0, density=0.3
        )
        sandbox.set_material_properties(thigh, restitution=0.0, friction=0.008)
        sandbox.set_material_properties(shin, restitution=0.0, friction=0.008)
        hip = sandbox.add_joint(
            torso,
            thigh,
            (hip_x, start_y),
            type='pivot',
            lower_limit=-0.7,
            upper_limit=0.7,
        )
        knee = sandbox.add_joint(
            thigh,
            shin,
            (hip_x, start_y - 1.0),
            type='pivot',
            lower_limit=0.0,
            upper_limit=1.2,
        )
        joints.extend((hip, knee))
    return {"body": torso, "joints": joints}

def agent_action_stage_4(sandbox, agent_body, step_count):
    joints = _get_joints(agent_body)
    cycle_steps = 60
    for leg_index in range(2):
        hip = joints[2 * leg_index]
        knee = joints[2 * leg_index + 1]
        phase = ((step_count + leg_index * cycle_steps // 2) % cycle_steps) / cycle_steps
        front_angle, rear_angle = 0.6, -0.6
        if phase < 0.20:
            hip_target, knee_target = rear_angle, 1.1
        elif phase < 0.45:
            swing_progress = (phase - 0.20) / 0.25
            hip_target = rear_angle + (front_angle - rear_angle) * swing_progress
            knee_target = 1.1
        elif phase < 0.55:
            hip_target, knee_target = front_angle, 0.0
        else:
            stance_progress = (phase - 0.55) / 0.45
            hip_target = front_angle + (rear_angle - front_angle) * stance_progress
            knee_target = 0.0
        hip_speed = max(-12.0, min(12.0, 20.0 * (hip_target - hip.angle)))
        knee_speed = max(-12.0, min(12.0, 20.0 * (knee_target - knee.angle)))
        for joint, speed in ((hip, hip_speed), (knee, knee_speed)):
            sandbox.set_motor(joint, speed, 100.0)
