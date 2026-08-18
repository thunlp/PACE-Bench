import math

def _base_build_agent(sandbox):
    jumper = sandbox.get_jumper()
    if jumper is None:
        raise ValueError("Jumper not found in environment")
    pad = sandbox.add_beam(
        x=5.0,
        y=2.75,
        width=1.0,
        height=0.2,
        angle=0,
        density=40.0,
    )
    sandbox.set_material_properties(pad, restitution=0.2)
    sandbox.add_joint(pad, None, (5.0, 2.75), type="rigid")
    total_mass = sandbox.get_structure_mass()
    if total_mass > sandbox.MAX_STRUCTURE_MASS:
        raise ValueError(
            f"Structure mass {total_mass:.2f} kg exceeds the "
            f"{sandbox.MAX_STRUCTURE_MASS:.2f} kg limit"
        )
    return jumper

def build_agent(sandbox):
    return _base_build_agent(sandbox)

def agent_action(sandbox, agent_body, step_count):
    if step_count != 0:
        return
    vx, vy = 10.0, 15.9
    sandbox.set_jumper_velocity(vx, vy)

def build_agent_stage_1(sandbox):
    return _base_build_agent(sandbox)

def agent_action_stage_1(sandbox, agent_body, step_count):
    if step_count != 0:
        return
    vx, vy = 130.0, 110.0
    sandbox.set_jumper_velocity(vx, vy)

def build_agent_stage_2(sandbox):
    jumper = sandbox.get_jumper()
    if jumper is None:
        raise ValueError("Jumper not found in environment")
    return jumper

def agent_action_stage_2(sandbox, agent_body, step_count):
    x, y = sandbox.get_body_position()
    if x < 17.92:
        vx = 8.0
        target_y = 5.25
    elif x < 18.15:
        vx = 0.08
        target_y = 15.45
    elif x < 19.92:
        vx = 8.0
        target_y = 15.45
    elif x < 20.15:
        vx = 0.08
        target_y = 4.85
    else:
        vx = 8.0
        target_y = 4.85
    vy = max(-12.0, min(12.0, 10.0 * (target_y - y)))
    sandbox.set_jumper_velocity(vx, vy)

def build_agent_stage_3(sandbox):
    return _base_build_agent(sandbox)

def agent_action_stage_3(sandbox, agent_body, step_count):
    if step_count != 0:
        return
    vx, vy = 115.87, 98.24
    sandbox.set_jumper_velocity(vx, vy)

def build_agent_stage_4(sandbox):
    return _base_build_agent(sandbox)

def agent_action_stage_4(sandbox, agent_body, step_count):
    if step_count != 0:
        return
    vx, vy = 74.47, 63.77
    sandbox.set_jumper_velocity(vx, vy)
