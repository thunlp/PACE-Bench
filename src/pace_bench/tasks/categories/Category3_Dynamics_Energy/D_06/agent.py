import math

GROUND_TOP = 0.5

def build_agent(sandbox):
    density = 0.05
    pillar_w = 0.1
    rest = 0.0
    slab_y = 2.65
    p1 = sandbox.add_beam(7.08, 1.75, pillar_w, 2.5, 0, density)
    sandbox.set_material_properties(p1, restitution=rest)
    sandbox.add_joint(p1, None, (7.08, GROUND_TOP), type="rigid")
    p2 = sandbox.add_beam(7.16, 1.75, pillar_w, 2.5, 0, density)
    sandbox.set_material_properties(p2, restitution=rest)
    sandbox.add_joint(p2, None, (7.16, GROUND_TOP), type="rigid")
    slab_left = sandbox.add_beam(7.12, slab_y, 0.2, 0.22, 0, density)
    sandbox.set_material_properties(slab_left, restitution=0.0)
    sandbox.add_joint(p1, slab_left, (7.08, slab_y), type="rigid")
    sandbox.add_joint(p2, slab_left, (7.16, slab_y), type="rigid")
    p5 = sandbox.add_beam(9.75, 1.75, pillar_w, 2.0, 0, density)
    sandbox.set_material_properties(p5, restitution=rest)
    sandbox.add_joint(p5, None, (9.75, GROUND_TOP), type="rigid")
    slab_right_a = sandbox.add_beam(9.75, slab_y, 0.35, 0.25, 0, density)
    sandbox.set_material_properties(slab_right_a, restitution=0.0)
    sandbox.add_joint(p5, slab_right_a, (9.75, slab_y), type="rigid")
    slab_right_b = sandbox.add_beam(10.75, 1.7, 0.45, 0.3, 0, density)
    sandbox.set_material_properties(slab_right_b, restitution=0.0)
    sandbox.add_joint(slab_right_b, None, (10.75, GROUND_TOP), type="rigid")
    n = len(sandbox.bodies)
    if n > sandbox.MAX_BEAM_COUNT:
        raise ValueError(f"Beam count {n} > {sandbox.MAX_BEAM_COUNT}")
    mass = sandbox.get_structure_mass()
    if mass >= sandbox.MAX_STRUCTURE_MASS:
        raise ValueError(f"Mass {mass:.2f} must be < {sandbox.MAX_STRUCTURE_MASS} kg")
    return slab_right_a

def agent_action(sandbox, agent_body, step_count):
    pass

def build_agent_stage_1(sandbox):
    anchor = sandbox.add_beam(7.06, 5.42, 0.1, 0.1, 0, 0.001)
    sandbox.set_material_properties(anchor, restitution=0.0)
    sandbox.add_joint(anchor, None, (7.06, 5.5), type="rigid")
    for y, angle in [
        (0.75, 0.0),
        (0.78, 0.18),
        (0.82, -0.18),
        (1.75, 0.55),
        (1.78, -0.55),
        (2.75, 0.9),
        (3.82, -0.9),
    ]:
        blade = sandbox.add_beam(7.77, y, 1.3, 0.12, angle, 0.025)
        sandbox.set_damping(blade, 300.0, 300.0)
        sandbox.set_material_properties(blade, restitution=0.0)
    return anchor

def build_agent_stage_2(sandbox):
    anchor = None
    for y in [0.75, 1.75, 2.75, 3.85]:
        outer = sandbox.add_beam(9.72, y, 1.35, 0.1, 0, 0.18)
        sandbox.set_damping(outer, 300.0, 300.0)
        sandbox.set_material_properties(outer, restitution=0.0)
        sandbox.add_joint(outer, None, (9.72, y), type="rigid")
        inner = sandbox.add_beam(7.77, y, 1.35, 0.1, 0, 0.18)
        sandbox.set_damping(inner, 300.0, 300.0)
        sandbox.set_material_properties(inner, restitution=0.0)
        sandbox.add_joint(inner, None, (7.77, y), type="rigid")
        if anchor is None:
            anchor = inner
    top = sandbox.add_beam(7.77, 5.0, 1.35, 0.1, 0, 0.18)
    sandbox.set_damping(top, 300.0, 300.0)
    sandbox.set_material_properties(top, restitution=0.0)
    sandbox.add_joint(top, None, (7.77, 5.0), type="rigid")
    return anchor

def build_agent_stage_3(sandbox):
    anchor = sandbox.add_beam(7.1, 5.4, 0.1, 0.1, 0, 0.1)
    sandbox.add_joint(anchor, None, (7.1, 5.5), type="rigid")
    for y in [0.75, 1.75, 2.75, 3.85]:
        outer = sandbox.add_beam(9.98, y, 1.2, 0.1, 0, 8.0)
        sandbox.set_damping(outer, 220.0, 220.0)
        sandbox.set_material_properties(outer, restitution=0.0)
        inner = sandbox.add_beam(7.77, y, 1.2, 0.1, 0, 8.0)
        sandbox.set_damping(inner, 220.0, 220.0)
        sandbox.set_material_properties(inner, restitution=0.0)
    return anchor

def build_agent_stage_4(sandbox):
    anchor = sandbox.add_beam(7.1, 5.4, 0.1, 0.1, 0, 1.0)
    sandbox.add_joint(anchor, None, (7.1, 5.5), type="rigid")
    for y in [0.75, 1.75, 2.75, 3.85]:
        outer = sandbox.add_beam(9.55, y, 1.0, 0.1, 0, 10.0)
        sandbox.set_damping(outer, 220.0, 220.0)
        sandbox.set_material_properties(outer, restitution=0.0)
        inner = sandbox.add_beam(7.77, y, 1.0, 0.1, 0, 10.0)
        sandbox.set_damping(inner, 220.0, 220.0)
        sandbox.set_material_properties(inner, restitution=0.0)
    return anchor

def agent_action_stage_1(sandbox, agent_body, step_count):
    pass

def agent_action_stage_2(sandbox, agent_body, step_count):
    pass

def agent_action_stage_3(sandbox, agent_body, step_count):
    pass

def agent_action_stage_4(sandbox, agent_body, step_count):
    pass
