import math

import Box2D

def build_agent(sandbox):
    gantry = sandbox.get_anchor_for_gripper()
    base = sandbox.add_beam(x=5.0, y=9.8, width=0.6, height=0.4, angle=0, density=1.5)
    sandbox.add_joint(gantry, base, (5.0, 10.0), type='rigid')
    slider = sandbox.add_beam(x=5.0, y=7.6, width=0.35, height=2.0, angle=0, density=0.6)
    sandbox.set_fixed_rotation(slider, True)
    slider_joint = sandbox.add_joint(base, slider, (5.0, 9.6), type='slider', axis=(0, -1), lower_translation=0.0, upper_translation=7.0, enable_motor=True, motor_speed=1.2, max_motor_force=10000.0)
    left_finger = sandbox.add_beam(x=4.72, y=5.35, width=0.28, height=0.5, angle=0.1 * math.pi, density=0.5)
    sandbox.set_material_properties(left_finger, restitution=0.05, friction=0.95)
    left_joint = sandbox.add_joint(slider, left_finger, (5.0, 5.6), type='pivot', enable_motor=True, motor_speed=0.0, max_motor_torque=5000.0)
    right_finger = sandbox.add_beam(x=5.28, y=5.35, width=0.28, height=0.5, angle=-0.1 * math.pi, density=0.5)
    sandbox.set_material_properties(right_finger, restitution=0.05, friction=0.95)
    right_joint = sandbox.add_joint(slider, right_finger, (5.0, 5.6), type='pivot', enable_motor=True, motor_speed=0.0, max_motor_torque=5000.0)
    return {
        "body": base,
        "joints": {
            "slider": slider_joint,
            "left_finger": left_joint,
            "right_finger": right_joint,
        },
    }

def agent_action(sandbox, agent_body, step_count):
    joints = agent_body.get("joints", {}) if isinstance(agent_body, dict) else {}
    if not joints:
        return
    t = step_count / 60.0
    if t < 5.0:
        sandbox.set_slider_motor(joints['slider'], 1.2, 10000.0)
        sandbox.set_motor(joints['left_finger'], 0.0, 5000.0)
        sandbox.set_motor(joints['right_finger'], 0.0, 5000.0)
    elif t < 7.0:
        sandbox.set_slider_motor(joints['slider'], 0.0, 10000.0)
        sandbox.set_motor(joints['left_finger'], 2.0, 5000.0)
        sandbox.set_motor(joints['right_finger'], -2.0, 5000.0)
    elif t < 10.0:
        sandbox.set_slider_motor(joints['slider'], 0.0, 10000.0)
        sandbox.set_motor(joints['left_finger'], 4.0, 5000.0)
        sandbox.set_motor(joints['right_finger'], -4.0, 5000.0)
    else:
        sandbox.set_slider_motor(joints['slider'], -1.8, 10000.0)
        sandbox.set_motor(joints['left_finger'], 4.0, 5000.0)
        sandbox.set_motor(joints['right_finger'], -4.0, 5000.0)

def build_agent_stage_1(sandbox):
    gantry = sandbox.get_anchor_for_gripper()
    mount = sandbox.add_beam(x=6.65, y=9.82, width=0.45, height=0.24, angle=0, density=0.8)
    sandbox.add_joint(gantry, mount, (6.65, 10.0), type='rigid')
    outrigger = sandbox.add_beam(x=7.42, y=9.78, width=1.55, height=0.16, angle=0, density=0.7)
    sandbox.add_joint(mount, outrigger, (6.65, 9.82), type='rigid')
    shuttle = sandbox.add_beam(x=8.2, y=7.7, width=0.24, height=1.8, angle=0, density=0.5)
    sandbox.set_fixed_rotation(shuttle, True)
    lift = sandbox.add_joint(outrigger, shuttle, (8.2, 9.65), type='slider', axis=(0, -1), lower_translation=0.0, upper_translation=6.0, enable_motor=True, motor_speed=0.0, max_motor_force=18000.0)
    cap = sandbox.add_beam(x=8.2, y=5.86, width=0.72, height=0.12, angle=0, density=0.7)
    left_wedge = sandbox.add_beam(x=7.965, y=5.36, width=0.10, height=0.92, angle=0.025 * math.pi, density=0.7)
    right_wedge = sandbox.add_beam(x=8.435, y=5.36, width=0.10, height=0.92, angle=-0.025 * math.pi, density=0.7)
    sandbox.set_material_properties(left_wedge, restitution=0.0, friction=1.2)
    sandbox.set_material_properties(right_wedge, restitution=0.0, friction=1.2)
    sandbox.add_joint(shuttle, cap, (8.2, 5.86), type='rigid')
    sandbox.add_joint(cap, left_wedge, (7.965, 5.80), type='rigid')
    sandbox.add_joint(cap, right_wedge, (8.435, 5.80), type='rigid')
    return {"body": mount, "joints": {"lift": lift}}

def agent_action_stage_1(sandbox, agent_body, step_count):
    joints = agent_body.get("joints", {}) if isinstance(agent_body, dict) else {}
    if not joints:
        return
    if step_count < 210:
        velocity = 1.05
    elif step_count < 250:
        velocity = 0.0
    else:
        velocity = -1.25
    sandbox.set_slider_motor(joints['lift'], velocity, 18000.0)

def build_agent_stage_2(sandbox):
    gantry = sandbox.get_anchor_for_gripper()
    mount = sandbox.add_beam(x=6.65, y=9.82, width=0.5, height=0.28, angle=0, density=1.2)
    sandbox.add_joint(gantry, mount, (6.65, 10.0), type='rigid')
    boom = sandbox.add_beam(x=7.42, y=9.78, width=1.55, height=0.2, angle=0, density=1.2)
    sandbox.add_joint(mount, boom, (6.65, 9.82), type='rigid')
    shuttle = sandbox.add_beam(x=8.2, y=7.7, width=0.28, height=1.8, angle=0, density=0.8)
    sandbox.set_fixed_rotation(shuttle, True)
    lift = sandbox.add_joint(boom, shuttle, (8.2, 9.65), type='slider', axis=(0, -1), lower_translation=0.0, upper_translation=6.0, enable_motor=True, motor_speed=0.0, max_motor_force=120000.0)
    yoke = sandbox.add_beam(x=8.2, y=5.88, width=1.4, height=0.14, angle=0, density=1.0)
    sandbox.add_joint(shuttle, yoke, (8.2, 5.82), type='rigid')
    left_pad = sandbox.add_beam(x=7.7, y=5.35, width=0.14, height=0.6, angle=0.10 * math.pi, density=1.0)
    right_pad = sandbox.add_beam(x=8.7, y=5.35, width=0.14, height=0.6, angle=-0.10 * math.pi, density=1.0)
    sandbox.set_material_properties(left_pad, restitution=0.0, friction=2.0)
    sandbox.set_material_properties(right_pad, restitution=0.0, friction=2.0)
    left = sandbox.add_joint(yoke, left_pad, (7.7, 5.60), type='slider', axis=(1, 0), lower_translation=0.0, upper_translation=0.50, enable_motor=True, motor_speed=0.0, max_motor_force=40000.0)
    right = sandbox.add_joint(yoke, right_pad, (8.7, 5.60), type='slider', axis=(-1, 0), lower_translation=0.0, upper_translation=0.50, enable_motor=True, motor_speed=0.0, max_motor_force=40000.0)
    return {"body": mount, "joints": {"lift": lift, "left": left, "right": right}}

def agent_action_stage_2(sandbox, agent_body, step_count):
    joints = agent_body.get("joints", {}) if isinstance(agent_body, dict) else {}
    if not joints:
        return
    if step_count < 195:
        lift_speed = 1.0
        clamp_speed = 0.0
    elif step_count < 420:
        lift_speed = 0.0
        clamp_speed = 0.4
    else:
        lift_speed = -0.5
        left_target = 0.32
        right_target = 0.28
        left_speed = max(-0.8, min(0.8, 8.0 * (left_target - joints['left'].translation)))
        right_speed = max(-0.8, min(0.8, 8.0 * (right_target - joints['right'].translation)))
        sandbox.set_slider_motor(joints['lift'], lift_speed, 120000.0)
        sandbox.set_slider_motor(joints['left'], left_speed, 40000.0)
        sandbox.set_slider_motor(joints['right'], right_speed, 40000.0)
        return
    sandbox.set_slider_motor(joints['lift'], lift_speed, 120000.0)
    sandbox.set_slider_motor(joints['left'], clamp_speed, 40000.0)
    sandbox.set_slider_motor(joints['right'], clamp_speed, 40000.0)

def build_agent_stage_3(sandbox):
    gantry = sandbox.get_anchor_for_gripper()
    mount = sandbox.add_beam(x=6.6, y=9.82, width=0.5, height=0.3, angle=0, density=1.4)
    sandbox.add_joint(gantry, mount, (6.6, 10.0), type='rigid')
    upper_boom = sandbox.add_beam(x=7.4, y=9.78, width=1.6, height=0.2, angle=0, density=1.4)
    lower_brace = sandbox.add_beam(x=7.4, y=9.42, width=1.65, height=0.12, angle=-0.03 * math.pi, density=1.2)
    sandbox.add_joint(mount, upper_boom, (6.6, 9.82), type='rigid')
    sandbox.add_joint(mount, lower_brace, (6.6, 9.48), type='rigid')
    carriage = sandbox.add_beam(x=8.2, y=7.65, width=0.3, height=1.9, angle=0, density=0.9)
    sandbox.set_fixed_rotation(carriage, True)
    lift = sandbox.add_joint(upper_boom, carriage, (8.2, 9.62), type='slider', axis=(0, -1), lower_translation=0.0, upper_translation=6.2, enable_motor=True, motor_speed=0.0, max_motor_force=220000.0)
    yoke = sandbox.add_beam(x=8.2, y=5.72, width=1.5, height=0.16, angle=0, density=1.2)
    sandbox.add_joint(carriage, yoke, (8.2, 5.72), type='rigid')
    left_wedge = sandbox.add_beam(x=7.65, y=5.25, width=0.15, height=0.65, angle=0.13 * math.pi, density=1.1)
    right_wedge = sandbox.add_beam(x=8.75, y=5.25, width=0.15, height=0.65, angle=-0.13 * math.pi, density=1.1)
    sandbox.set_material_properties(left_wedge, restitution=0.0, friction=2.0)
    sandbox.set_material_properties(right_wedge, restitution=0.0, friction=2.0)
    left = sandbox.add_joint(yoke, left_wedge, (7.65, 5.55), type='slider', axis=(1, 0), lower_translation=0.0, upper_translation=0.55, enable_motor=True, motor_speed=0.0, max_motor_force=90000.0)
    right = sandbox.add_joint(yoke, right_wedge, (8.75, 5.55), type='slider', axis=(-1, 0), lower_translation=0.0, upper_translation=0.55, enable_motor=True, motor_speed=0.0, max_motor_force=90000.0)
    return {"body": mount, "joints": {"lift": lift, "left": left, "right": right}}

def agent_action_stage_3(sandbox, agent_body, step_count):
    joints = agent_body.get("joints", {}) if isinstance(agent_body, dict) else {}
    if not joints:
        return
    if step_count < 190:
        lift_speed = 1.0
        clamp_speed = 0.0
    elif step_count < 410:
        lift_speed = 0.0
        clamp_speed = 0.45
    else:
        lift_speed = -0.45
        left_speed = max(-0.8, min(0.8, 8.0 * (0.38 - joints['left'].translation)))
        right_speed = max(-0.8, min(0.8, 8.0 * (0.32 - joints['right'].translation)))
        sandbox.set_slider_motor(joints['lift'], lift_speed, 220000.0)
        sandbox.set_slider_motor(joints['left'], left_speed, 90000.0)
        sandbox.set_slider_motor(joints['right'], right_speed, 90000.0)
        return
    sandbox.set_slider_motor(joints['lift'], lift_speed, 220000.0)
    sandbox.set_slider_motor(joints['left'], clamp_speed, 90000.0)
    sandbox.set_slider_motor(joints['right'], clamp_speed, 90000.0)

def build_agent_stage_4(sandbox):
    gantry = sandbox.get_anchor_for_gripper()
    mount = sandbox.add_beam(x=6.55, y=9.82, width=0.55, height=0.32, angle=0, density=1.6)
    sandbox.add_joint(gantry, mount, (6.55, 10.0), type='rigid')
    top = sandbox.add_beam(x=7.38, y=9.8, width=1.65, height=0.22, angle=0, density=1.5)
    diagonal = sandbox.add_beam(x=7.38, y=9.42, width=1.7, height=0.14, angle=-0.035 * math.pi, density=1.4)
    sandbox.add_joint(mount, top, (6.55, 9.82), type='rigid')
    sandbox.add_joint(mount, diagonal, (6.55, 9.48), type='rigid')
    shuttle = sandbox.add_beam(x=8.2, y=7.6, width=0.32, height=2.0, angle=0, density=1.0)
    sandbox.set_fixed_rotation(shuttle, True)
    lift = sandbox.add_joint(top, shuttle, (8.2, 9.62), type='slider', axis=(0, -1), lower_translation=0.0, upper_translation=6.3, enable_motor=True, motor_speed=0.0, max_motor_force=320000.0)
    yoke = sandbox.add_beam(x=8.2, y=5.58, width=1.4, height=0.18, angle=0, density=1.3)
    sandbox.add_joint(shuttle, yoke, (8.2, 5.58), type='rigid')
    left_arc = sandbox.add_beam(x=7.7, y=5.15, width=0.16, height=0.7, angle=0.12 * math.pi, density=1.2)
    right_arc = sandbox.add_beam(x=8.7, y=5.15, width=0.16, height=0.7, angle=-0.12 * math.pi, density=1.2)
    sandbox.set_material_properties(left_arc, restitution=0.0, friction=2.0)
    sandbox.set_material_properties(right_arc, restitution=0.0, friction=2.0)
    left = sandbox.add_joint(yoke, left_arc, (7.7, 5.48), type='slider', axis=(1, 0), lower_translation=0.0, upper_translation=0.55, enable_motor=True, motor_speed=0.0, max_motor_force=150000.0)
    right = sandbox.add_joint(yoke, right_arc, (8.7, 5.48), type='slider', axis=(-1, 0), lower_translation=0.0, upper_translation=0.55, enable_motor=True, motor_speed=0.0, max_motor_force=150000.0)
    return {"body": mount, "joints": {"lift": lift, "left": left, "right": right}}

def agent_action_stage_4(sandbox, agent_body, step_count):
    joints = agent_body.get("joints", {}) if isinstance(agent_body, dict) else {}
    if not joints:
        return
    if step_count < 195:
        lift_speed = 1.0
        clamp_speed = 0.0
    elif step_count < 430:
        lift_speed = 0.0
        clamp_speed = 0.4
    else:
        lift_speed = -0.4
        left_speed = max(-0.8, min(0.8, 8.0 * (0.34 - joints['left'].translation)))
        right_speed = max(-0.8, min(0.8, 8.0 * (0.30 - joints['right'].translation)))
        sandbox.set_slider_motor(joints['lift'], lift_speed, 320000.0)
        sandbox.set_slider_motor(joints['left'], left_speed, 150000.0)
        sandbox.set_slider_motor(joints['right'], right_speed, 150000.0)
        return
    sandbox.set_slider_motor(joints['lift'], lift_speed, 320000.0)
    sandbox.set_slider_motor(joints['left'], clamp_speed, 150000.0)
    sandbox.set_slider_motor(joints['right'], clamp_speed, 150000.0)
