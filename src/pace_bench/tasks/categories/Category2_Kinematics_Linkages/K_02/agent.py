import math

def build_agent(sandbox):
    start_x = 4.5
    top = sandbox.add_beam(start_x, 21.5, 0.4, 1.0, density=5.0)
    prev_b = top
    for i in range(6, -1, -1):
        y = 1.5 + i * 3.0
        b = sandbox.add_beam(start_x, y, 0.4, 3.0, density=5.0)
        joint_y = y + 1.5
        sandbox.add_joint(prev_b, b, (start_x, joint_y), type='rigid')
        prev_b = b
    return top

def agent_action(sandbox, agent_body, step_count):
    pass

def build_agent_stage_1(sandbox):
    x_pos = 4.9
    base_y = 1.5
    torso = sandbox.add_beam(x_pos, base_y, 0.05, 0.4, density=0.03)
    p1 = sandbox.add_pad(x_pos + 0.1, base_y - 0.12, radius=0.06, density=0.03)
    p2 = sandbox.add_pad(x_pos + 0.1, base_y + 0.12, radius=0.06, density=0.03)
    sandbox.add_joint(torso, p1, (x_pos + 0.1, base_y - 0.12), type='rigid')
    sandbox.add_joint(torso, p2, (x_pos + 0.1, base_y + 0.12), type='rigid')
    arm = sandbox.add_beam(x_pos, base_y + 0.6, 0.04, 0.8, density=0.03)
    p3 = sandbox.add_pad(x_pos + 0.1, base_y + 0.9, radius=0.06, density=0.03)
    p4 = sandbox.add_pad(x_pos + 0.1, base_y + 1.0, radius=0.06, density=0.03)
    sandbox.add_joint(arm, p3, (x_pos + 0.1, base_y + 0.9), type='rigid')
    sandbox.add_joint(arm, p4, (x_pos + 0.1, base_y + 1.0), type='rigid')
    joint = sandbox.add_joint(torso, arm, (x_pos, base_y + 0.2), type='pivot', lower_limit=-0.05, upper_limit=2.0)
    control = {
        'p_torso': [p1, p2],
        'p_arm': [p3, p4],
        'joint': joint,
        'cycle': 100,
        'speed': 1.5,
        'torque': 2.0,
        'overlap': 15
    }
    return {"body": torso, "control": control}

def agent_action_stage_1(sandbox, agent_body, step_count):
    control = agent_body.get("control", {}) if isinstance(agent_body, dict) else {}
    if not control:
        return
    cycle = control['cycle']
    phase = step_count % (2 * cycle)
    overlap = control.get('overlap', 10)
    speed = control.get('speed', 10.0)
    torque = control.get('torque', 1000.0)
    if phase < cycle:
        for pad in control['p_torso']:
            sandbox.set_pad_active(pad, True)
        for pad in control['p_arm']:
            sandbox.set_pad_active(pad, phase <= overlap)
        sandbox.set_motor(control['joint'], speed, torque)
    else:
        for pad in control['p_arm']:
            sandbox.set_pad_active(pad, True)
        for pad in control['p_torso']:
            sandbox.set_pad_active(pad, phase - cycle <= overlap)
        sandbox.set_motor(control['joint'], -speed, torque)

def build_agent_stage_2(sandbox):
    x_pos = 4.85
    base_y = 1.0
    torso = sandbox.add_beam(x_pos, base_y, 0.05, 0.6, density=200.0)
    p1 = sandbox.add_pad(x_pos + 0.1, base_y - 0.2, radius=0.1, density=10.0)
    p2 = sandbox.add_pad(x_pos + 0.1, base_y + 0.2, radius=0.1, density=10.0)
    sandbox.add_joint(torso, p1, (x_pos + 0.1, base_y - 0.2), type='rigid')
    sandbox.add_joint(torso, p2, (x_pos + 0.1, base_y + 0.2), type='rigid')
    arm = sandbox.add_beam(x_pos, 4.8, 0.12, 3.0, density=20.0)
    p3 = sandbox.add_pad(x_pos + 0.1, 8.2, radius=0.12, density=5.0)
    p4 = sandbox.add_pad(x_pos + 0.1, 8.5, radius=0.12, density=5.0)
    sandbox.add_joint(arm, p3, (x_pos + 0.1, 8.2), type='rigid')
    sandbox.add_joint(arm, p4, (x_pos + 0.1, 8.5), type='rigid')
    joint = sandbox.add_joint(torso, arm, (x_pos, 1.3), type='pivot', lower_limit=-0.1, upper_limit=5.0)
    control = {
        'p_torso': [p1, p2],
        'p_arm': [p3, p4],
        'joint': joint,
        'cycle': 240,
        'speed': 10.0,
        'torque': 180000.0,
        'overlap': 70
    }
    return {"body": torso, "control": control}

def agent_action_stage_2(sandbox, agent_body, step_count):
    control = agent_body.get("control", {}) if isinstance(agent_body, dict) else {}
    if not control:
        return
    cycle = control['cycle']
    phase = step_count % (2 * cycle)
    overlap = control.get('overlap', 10)
    speed = control.get('speed', 10.0)
    torque = control.get('torque', 1000.0)
    if phase < cycle:
        for pad in control['p_torso']:
            sandbox.set_pad_active(pad, True)
        for pad in control['p_arm']:
            sandbox.set_pad_active(pad, phase <= overlap)
        sandbox.set_motor(control['joint'], speed, torque)
    else:
        for pad in control['p_arm']:
            sandbox.set_pad_active(pad, True)
        for pad in control['p_torso']:
            sandbox.set_pad_active(pad, phase - cycle <= overlap)
        sandbox.set_motor(control['joint'], -speed, torque)

def build_agent_stage_3(sandbox):
    spine_x = 4.85
    lower = sandbox.add_beam(spine_x, 2.0, 0.25, 1.7, density=48.92)
    upper = sandbox.add_beam(spine_x, 3.5, 0.25, 1.7, density=48.92)
    lower_pad = sandbox.add_pad(5.0, 1.1, radius=0.11, density=0.1)
    upper_pad = sandbox.add_pad(5.0, 4.4, radius=0.11, density=0.1)
    sandbox.add_joint(lower, lower_pad, (5.0, 1.1), type='rigid')
    sandbox.add_joint(upper, upper_pad, (5.0, 4.4), type='rigid')
    centering_joint = sandbox.add_joint(
        lower,
        upper,
        (spine_x, 2.75),
        type='pivot',
        lower_limit=-0.02,
        upper_limit=0.02,
    )
    control = {
        'pads': [lower_pad, upper_pad],
        'joint': centering_joint,
    }
    return {"body": lower, "control": control}

def agent_action_stage_3(sandbox, agent_body, step_count):
    bridge = agent_body.get("control", {}) if isinstance(agent_body, dict) else {}
    for pad in bridge.get('pads', []):
        sandbox.set_pad_active(pad, True)
    joint = bridge.get('joint')
    if joint is not None:
        sandbox.set_motor(joint, 0.0, 60.0)

def build_agent_stage_4(sandbox):
    x_pos = 4.8
    base_y = 1.0
    torso = sandbox.add_beam(x_pos, base_y, 0.2, 0.8, density=100.0)
    p1 = sandbox.add_pad(x_pos + 0.1, base_y - 0.2, radius=0.12, density=10.0)
    p2 = sandbox.add_pad(x_pos + 0.1, base_y + 0.2, radius=0.12, density=10.0)
    sandbox.add_joint(torso, p1, (x_pos + 0.1, base_y - 0.2), type='rigid')
    sandbox.add_joint(torso, p2, (x_pos + 0.1, base_y + 0.2), type='rigid')
    arm = sandbox.add_beam(x_pos, base_y + 1.2, 0.2, 1.8, density=50.0)
    p3 = sandbox.add_pad(x_pos + 0.1, base_y + 1.8, radius=0.12, density=10.0)
    p4 = sandbox.add_pad(x_pos + 0.1, base_y + 2.0, radius=0.12, density=10.0)
    sandbox.add_joint(arm, p3, (x_pos + 0.1, base_y + 1.8), type='rigid')
    sandbox.add_joint(arm, p4, (x_pos + 0.1, base_y + 2.0), type='rigid')
    joint = sandbox.add_joint(torso, arm, (x_pos, base_y + 0.4), type='pivot', lower_limit=-0.1, upper_limit=3.5)
    control = {
        'p_torso': [p1, p2],
        'p_arm': [p3, p4],
        'joint': joint,
        'cycle': 150,
        'speed': 8.0,
        'torque': 150000.0,
        'overlap': 40
    }
    return {"body": torso, "control": control}

def agent_action_stage_4(sandbox, agent_body, step_count):
    control = agent_body.get("control", {}) if isinstance(agent_body, dict) else {}
    if not control:
        return
    cycle = control['cycle']
    phase = step_count % (2 * cycle)
    overlap = control.get('overlap', 10)
    speed = control.get('speed', 10.0)
    torque = control.get('torque', 1000.0)
    if phase < cycle:
        for pad in control['p_torso']:
            sandbox.set_pad_active(pad, True)
        for pad in control['p_arm']:
            sandbox.set_pad_active(pad, phase <= overlap)
        sandbox.set_motor(control['joint'], speed, torque)
    else:
        for pad in control['p_arm']:
            sandbox.set_pad_active(pad, True)
        for pad in control['p_torso']:
            sandbox.set_pad_active(pad, phase - cycle <= overlap)
        sandbox.set_motor(control['joint'], -speed, torque)
