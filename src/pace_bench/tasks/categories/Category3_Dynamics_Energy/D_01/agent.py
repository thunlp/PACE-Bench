import math

def build_agent(sandbox):
    ground = sandbox.get_ground()
    projectile = sandbox.get_projectile()
    if not ground or not projectile:
        raise ValueError("Ground or projectile not found in environment")
    pivot_x, pivot_y = 12.0, 1.0
    tip_x, tip_y = 9.5, 2.8
    arm_dx = tip_x - pivot_x
    arm_dy = tip_y - pivot_y
    arm_length = math.sqrt(arm_dx * arm_dx + arm_dy * arm_dy)
    arm_angle = math.atan2(arm_dy, arm_dx)
    arm_center_x = (pivot_x + tip_x) / 2
    arm_center_y = (pivot_y + tip_y) / 2
    arm = sandbox.add_beam(
        x=arm_center_x,
        y=arm_center_y,
        width=arm_length,
        height=0.2,
        angle=arm_angle,
        density=2.0,
    )
    sandbox.set_material_properties(arm, restitution=0.35)
    sandbox.add_joint(arm, None, (pivot_x, pivot_y), type="pivot")
    sandbox.add_spring(
        ground,
        arm,
        (12.0, 3.0),
        (11.0, 2.0),
        rest_length=0.28,
        stiffness=2800.0,
        damping_ratio=0.24,
    )
    total_mass = sandbox.get_structure_mass()
    if total_mass > sandbox.MAX_STRUCTURE_MASS:
        raise ValueError(
            f"Launcher mass {total_mass:.2f} kg exceeds the "
            f"{sandbox.MAX_STRUCTURE_MASS:.2f} kg budget"
        )
    return projectile

def agent_action(sandbox, agent_body, step_count):
    pass

def build_agent_stage_1(sandbox):
    ground = sandbox.get_ground()
    projectile = sandbox.get_projectile()
    pivot_x, pivot_y = 12.0, 1.0
    tip_x, tip_y = 9.5, 3.2
    arm = sandbox.add_beam(
        x=(pivot_x+tip_x)/2, y=(pivot_y+tip_y)/2,
        width=math.sqrt((tip_x-pivot_x)**2+(tip_y-pivot_y)**2),
        height=0.2, angle=math.atan2(tip_y-pivot_y, tip_x-pivot_x),
        density=20.0
    )
    sandbox.set_material_properties(arm, restitution=1.0)
    sandbox.add_joint(arm, None, (pivot_x, pivot_y), type="pivot")
    sandbox.add_spring(ground, arm, (15.5, 8.0), (11.0, 2.0), rest_length=0.01, stiffness=3000.0, damping_ratio=0.5)
    return projectile

def agent_action_stage_1(sandbox, agent_body, step_count):
    pass

def build_agent_stage_2(sandbox):
    ground = sandbox.get_ground()
    projectile = sandbox.get_projectile()
    pivot_x, pivot_y = 12.0, 1.0
    tip_x, tip_y = 9.5, 2.8
    arm = sandbox.add_beam(
        x=(pivot_x+tip_x)/2, y=(pivot_y+tip_y)/2,
        width=math.sqrt((tip_x-pivot_x)**2+(tip_y-pivot_y)**2),
        height=0.2, angle=math.atan2(tip_y-pivot_y, tip_x-pivot_x),
        density=2.0
    )
    sandbox.set_material_properties(arm, restitution=0.35)
    sandbox.add_joint(arm, None, (pivot_x, pivot_y), type="pivot")
    sandbox.add_spring(ground, arm, (12.0, 3.0), (11.0, 2.0), rest_length=0.21, stiffness=3000.0, damping_ratio=0.24)
    return projectile

def agent_action_stage_2(sandbox, agent_body, step_count):
    pass

def build_agent_stage_3(sandbox):
    ground = sandbox.get_ground()
    projectile = sandbox.get_projectile()
    pivot_x, pivot_y = 12.0, 1.0
    tip_x, tip_y = 9.5, 2.8
    arm = sandbox.add_beam(
        x=(pivot_x+tip_x)/2, y=(pivot_y+tip_y)/2,
        width=math.sqrt((tip_x-pivot_x)**2+(tip_y-pivot_y)**2),
        height=0.2, angle=math.atan2(tip_y-pivot_y, tip_x-pivot_x),
        density=20.0
    )
    sandbox.set_material_properties(arm, restitution=1.0)
    sandbox.add_joint(arm, None, (pivot_x, pivot_y), type="pivot")
    sandbox.add_spring(ground, arm, (12.0, 8.0), (11.0, 2.0), rest_length=0.001, stiffness=3000.0, damping_ratio=0.5)
    return projectile

def agent_action_stage_3(sandbox, agent_body, step_count):
    pass

def build_agent_stage_4(sandbox):
    ground = sandbox.get_ground()
    projectile = sandbox.get_projectile()
    pivot_x, pivot_y = 12.0, 1.0
    tip_x, tip_y = 9.5, 3.0
    arm = sandbox.add_beam(
        x=(pivot_x+tip_x)/2, y=(pivot_y+tip_y)/2,
        width=math.sqrt((tip_x-pivot_x)**2+(tip_y-pivot_y)**2),
        height=0.2, angle=math.atan2(tip_y-pivot_y, tip_x-pivot_x),
        density=20.0
    )
    sandbox.set_material_properties(arm, restitution=1.0)
    sandbox.add_joint(arm, None, (pivot_x, pivot_y), type="pivot")
    sandbox.add_spring(ground, arm, (12.6, 8.0), (11.0, 2.0), rest_length=0.001, stiffness=3000.0, damping_ratio=0.5)
    return projectile

def agent_action_stage_4(sandbox, agent_body, step_count):
    pass
