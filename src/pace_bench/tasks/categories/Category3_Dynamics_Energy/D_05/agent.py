import math

def build_agent(sandbox):
    pivot_x, pivot_y = 12.0, 7.5
    arm_len = 5.5
    arm_center_y = pivot_y - arm_len / 2
    arm = sandbox.add_beam(
        x=pivot_x,
        y=arm_center_y,
        width=0.26,
        height=arm_len,
        angle=0.0,
        density=28.0,
    )
    sandbox.add_joint(arm, None, (pivot_x, pivot_y), type="pivot")
    head_dist = 5.5
    head = sandbox.add_beam(
        x=pivot_x,
        y=pivot_y - head_dist,
        width=0.55,
        height=0.55,
        angle=0.0,
        density=85.0,
    )
    sandbox.add_joint(arm, head, (pivot_x, pivot_y - head_dist), type="rigid")
    sandbox.set_material_properties(arm, restitution=0.2)
    sandbox.set_material_properties(head, restitution=0.2)
    mass = sandbox.get_structure_mass()
    if mass > sandbox.MAX_STRUCTURE_MASS:
        raise ValueError(f"Structure mass {mass:.2f} kg exceeds limit {sandbox.MAX_STRUCTURE_MASS} kg")
    return head

def agent_action(sandbox, agent_body, step_count):
    arm = sandbox.bodies[0] if sandbox.bodies else None
    if not arm:
        return
    if step_count == 380:
        arm.angularVelocity = 26.0
    elif step_count == 398:
        arm.angularVelocity = 2.2
    elif step_count == 408:
        arm.angularVelocity = 12.0

def build_agent_stage_1(sandbox):
    pivot_x, pivot_y = 12.0, 7.5
    arm_len = 5.5
    arm_center_y = pivot_y - arm_len / 2
    arm = sandbox.add_beam(
        x=pivot_x,
        y=arm_center_y,
        width=0.26,
        height=arm_len,
        angle=0.0,
        density=28.0,
    )
    sandbox.add_joint(arm, None, (pivot_x, pivot_y), type="pivot")
    head_dist = 5.5
    head = sandbox.add_beam(
        x=pivot_x,
        y=pivot_y - head_dist,
        width=0.55,
        height=0.55,
        angle=0.0,
        density=110.0,
    )
    sandbox.add_joint(arm, head, (pivot_x, pivot_y - head_dist), type="rigid")
    sandbox.set_material_properties(arm, restitution=0.2)
    sandbox.set_material_properties(head, restitution=0.2)
    return head

def agent_action_stage_1(sandbox, agent_body, step_count):
    arm = sandbox.bodies[0] if sandbox.bodies else None
    if not arm:
        return
    if step_count == 380:
        arm.angularVelocity = 26.0
    elif step_count == 398:
        arm.angularVelocity = 2.2
    elif step_count == 408:
        arm.angularVelocity = 24.0

def build_agent_stage_2(sandbox):
    pivot_x, pivot_y = 12.0, 7.5
    arm_len = 5.5
    arm_center_y = pivot_y - arm_len / 2
    arm = sandbox.add_beam(
        x=pivot_x,
        y=arm_center_y,
        width=0.26,
        height=arm_len,
        angle=0.0,
        density=28.0,
    )
    sandbox.add_joint(arm, None, (pivot_x, pivot_y), type="pivot")
    head_dist = 5.5
    head = sandbox.add_beam(
        x=pivot_x,
        y=pivot_y - head_dist,
        width=0.55,
        height=0.55,
        angle=0.0,
        density=85.0,
    )
    sandbox.add_joint(arm, head, (pivot_x, pivot_y - head_dist), type="rigid")
    sandbox.set_material_properties(arm, restitution=0.2)
    sandbox.set_material_properties(head, restitution=0.2)
    return head

def agent_action_stage_2(sandbox, agent_body, step_count):
    arm = sandbox.bodies[0] if sandbox.bodies else None
    if not arm:
        return
    if step_count == 308:
        arm.angularVelocity = 26.0
    elif step_count == 326:
        arm.angularVelocity = 2.2
    elif step_count == 336:
        arm.angularVelocity = 12.0

def build_agent_stage_3(sandbox):
    pivot_x, pivot_y = 12.0, 7.5
    arm_len = 5.5
    arm_center_y = pivot_y - arm_len / 2
    arm = sandbox.add_beam(
        x=pivot_x,
        y=arm_center_y,
        width=0.26,
        height=arm_len,
        angle=0.0,
        density=28.0,
    )
    sandbox.add_joint(arm, None, (pivot_x, pivot_y), type="pivot")
    head_dist = 5.5
    head = sandbox.add_beam(
        x=pivot_x,
        y=pivot_y - head_dist,
        width=0.55,
        height=0.55,
        angle=0.0,
        density=110.0,
    )
    sandbox.add_joint(arm, head, (pivot_x, pivot_y - head_dist), type="rigid")
    sandbox.set_material_properties(arm, restitution=0.2)
    sandbox.set_material_properties(head, restitution=0.2)
    return head

def agent_action_stage_3(sandbox, agent_body, step_count):
    arm = sandbox.bodies[0] if sandbox.bodies else None
    if not arm:
        return
    if step_count == 380:
        arm.angularVelocity = 26.0
    elif step_count == 398:
        arm.angularVelocity = 2.2
    elif step_count == 408:
        arm.angularVelocity = 24.0

def build_agent_stage_4(sandbox):
    pivot_x, pivot_y = 12.0, 7.5
    arm_len = 5.5
    arm_center_y = pivot_y - arm_len / 2
    arm = sandbox.add_beam(
        x=pivot_x,
        y=arm_center_y,
        width=0.26,
        height=arm_len,
        angle=0.0,
        density=28.0,
    )
    sandbox.add_joint(arm, None, (pivot_x, pivot_y), type="pivot")
    head_dist = 5.5
    head = sandbox.add_beam(
        x=pivot_x,
        y=pivot_y - head_dist,
        width=0.55,
        height=0.55,
        angle=0.0,
        density=110.0,
    )
    sandbox.add_joint(arm, head, (pivot_x, pivot_y - head_dist), type="rigid")
    sandbox.set_material_properties(arm, restitution=0.2)
    sandbox.set_material_properties(head, restitution=0.2)
    return head

def agent_action_stage_4(sandbox, agent_body, step_count):
    arm = sandbox.bodies[0] if sandbox.bodies else None
    if not arm:
        return
    if step_count == 334:
        arm.angularVelocity = 26.0
    elif step_count == 352:
        arm.angularVelocity = 2.2
    elif step_count == 362:
        arm.angularVelocity = 24.0
