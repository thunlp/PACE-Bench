import math

CENTER_X = 4.0

BASE_Y = 1.15

def build_agent(sandbox):
    base = sandbox.add_beam(x=CENTER_X, y=BASE_Y, width=4.0, height=0.2, density=10.0)
    sandbox.weld_to_ground(base, (CENTER_X, BASE_Y))
    plat = sandbox.add_beam(x=CENTER_X, y=1.5, width=4.0, height=0.2, density=5.0)
    sandbox.add_joint(base, plat, (CENTER_X, 1.5), type='slider', axis=(0, 1), lower_translation=-10.0, upper_translation=10.0)
    sandbox.set_fixed_rotation(plat, True)
    return {"body": base, "platform": plat}

def agent_action(sandbox, agent_body, step_count):
    plat = agent_body.get("platform") if isinstance(agent_body, dict) else None
    if plat is None:
        return
    target_y = 9.5
    if plat.position.y < target_y:
        sandbox.apply_force(plat, (0, 2000.0))
    else:
        sandbox.apply_force(plat, (0, 40.0))

def agent_action_pd(sandbox, agent_body, target_y, kp=400.0, kd=300.0, hover_force=300.0, max_f=2000.0, step_count=0):
    plat = agent_body.get("platform") if isinstance(agent_body, dict) else None
    if plat is None:
        return
    osc_target = target_y + 0.1 * math.sin(step_count * 0.1)
    error = osc_target - plat.position.y
    vel = plat.linearVelocity.y
    force_y = hover_force + error * kp - vel * kd
    force_y = max(0.0, min(max_f, force_y))
    jitter_x = 10.0 * math.cos(step_count * 0.1)
    sandbox.apply_force(plat, (jitter_x, force_y))

def build_agent_stage_1(sandbox):
    plat = sandbox.add_beam(x=CENTER_X, y=1.5, width=2.4, height=0.2, density=5.0)
    base = sandbox.add_beam(x=CENTER_X, y=BASE_Y, width=2.4, height=0.2, density=10.0)
    sandbox.weld_to_ground(base, (CENTER_X, BASE_Y))
    sandbox.add_joint(base, plat, (CENTER_X, 1.5), type='slider', axis=(0, 1), lower_translation=-1.0, upper_translation=15.0)
    sandbox.set_fixed_rotation(plat, True)
    wall_l = sandbox.add_beam(x=CENTER_X - 1.1, y=2.5, width=0.2, height=2.0, density=1.0)
    sandbox.add_joint(plat, wall_l, (CENTER_X - 1.1, 1.5), type='rigid')
    wall_r = sandbox.add_beam(x=CENTER_X + 1.1, y=2.5, width=0.2, height=2.0, density=1.0)
    sandbox.add_joint(plat, wall_r, (CENTER_X + 1.1, 1.5), type='rigid')
    return {"body": plat, "platform": plat}

def agent_action_stage_1(sandbox, agent_body, step_count):
    agent_action_pd(sandbox, agent_body, 9.5, hover_force=260.0, step_count=step_count)

def build_agent_stage_2(sandbox):
    plat = sandbox.add_beam(x=CENTER_X, y=1.75, width=1.5, height=0.3, density=20.0)
    sandbox.set_material_properties(plat, friction=1.0)
    base = sandbox.add_beam(x=CENTER_X, y=BASE_Y, width=1.5, height=0.2, density=10.0)
    sandbox.weld_to_ground(base, (CENTER_X, BASE_Y))
    sandbox.add_joint(base, plat, (CENTER_X, 1.75), type='slider', axis=(0, 1), lower_translation=-1.0, upper_translation=15.0)
    sandbox.set_fixed_rotation(plat, True)
    return {"body": plat, "platform": plat}

def agent_action_stage_2(sandbox, agent_body, step_count):
    plat = agent_body.get("platform") if isinstance(agent_body, dict) else None
    if plat is None:
        return
    if step_count < 20:
        sandbox.apply_force(plat, (0, 4500.0))
    else:
        agent_action_pd(sandbox, agent_body, 9.5, hover_force=4500.0, kp=400.0, kd=300.0, max_f=8000.0, step_count=step_count)

def build_agent_stage_3(sandbox):
    plat = sandbox.add_beam(x=CENTER_X, y=1.5, width=1.6, height=0.2, density=8.0)
    sandbox.set_material_properties(plat, friction=1.0)
    base = sandbox.add_beam(x=CENTER_X, y=BASE_Y, width=1.6, height=0.2, density=10.0)
    sandbox.weld_to_ground(base, (CENTER_X, BASE_Y))
    sandbox.add_joint(base, plat, (CENTER_X, 1.5), type='slider', axis=(0, 1), lower_translation=-1.0, upper_translation=20.0)
    sandbox.set_fixed_rotation(plat, True)
    wall_l = sandbox.add_beam(x=CENTER_X - 0.35, y=2.5, width=0.1, height=2.0, density=1.0)
    sandbox.set_material_properties(wall_l, friction=1.0)
    sandbox.add_joint(plat, wall_l, (CENTER_X - 0.35, 1.5), type='rigid')
    wall_r = sandbox.add_beam(x=CENTER_X + 0.35, y=2.5, width=0.1, height=2.0, density=1.0)
    sandbox.set_material_properties(wall_r, friction=1.0)
    sandbox.add_joint(plat, wall_r, (CENTER_X + 0.35, 1.5), type='rigid')
    return {"body": plat, "platform": plat}

def agent_action_stage_3(sandbox, agent_body, step_count):
    agent_action_pd(sandbox, agent_body, 12.5, kp=600.0, kd=500.0, hover_force=1000.0, max_f=4000.0, step_count=step_count)

def build_agent_stage_4(sandbox):
    plat = sandbox.add_beam(x=CENTER_X, y=1.5, width=1.3, height=0.2, density=5.0)
    base = sandbox.add_beam(x=CENTER_X, y=BASE_Y, width=1.3, height=0.2, density=10.0)
    sandbox.weld_to_ground(base, (CENTER_X, BASE_Y))
    for dx in [-0.5, -0.2, 0.2, 0.5]:
        sandbox.add_joint(base, plat, (CENTER_X + dx, 1.5), type='slider', axis=(0, 1), lower_translation=-1.0, upper_translation=15.0)
    sandbox.set_fixed_rotation(plat, True)
    wall_l = sandbox.add_beam(x=CENTER_X - 0.55, y=2.5, width=0.1, height=1.5, density=1.0)
    sandbox.add_joint(plat, wall_l, (CENTER_X - 0.55, 1.5), type='rigid')
    wall_r = sandbox.add_beam(x=CENTER_X + 0.55, y=2.5, width=0.1, height=1.5, density=1.0)
    sandbox.add_joint(plat, wall_r, (CENTER_X + 0.55, 1.5), type='rigid')
    return {"body": plat, "platform": plat}

def agent_action_stage_4(sandbox, agent_body, step_count):
    agent_action_pd(sandbox, agent_body, 10.5, kp=400.0, kd=600.0, hover_force=460.0, max_f=1400.0, step_count=step_count)
