import math

def build_agent(sandbox):
    beam_w = 0.62
    beam_h = 0.35
    density = 35.0
    centers_x = [2.31 + i * 0.62 for i in range(9)]
    y_center = 0.5 + beam_h / 2
    bodies = []
    for x in centers_x:
        b = sandbox.add_beam(x, y_center, beam_w, beam_h, angle=0, density=density)
        sandbox.set_material_properties(b, restitution=0.05)
        bodies.append(b)
    for i in range(8):
        anchor_x = (centers_x[i] + centers_x[i + 1]) / 2
        sandbox.add_joint(bodies[i], bodies[i + 1], (anchor_x, y_center), type='rigid')
    return bodies[-1]

def agent_action(sandbox, agent_body, step_count):
    if agent_body is None or not agent_body.active:
        return
    front_x = sandbox.get_vehicle_front_x() or agent_body.position.x
    target_x = 26.0
    if front_x >= target_x + 0.1:
        for b in sandbox.bodies:
            if b.active:
                b.linearVelocity = (0, 0)
                b.angularVelocity = 0
        return
    THRUST_F = 520.0
    LIFT_F = 400.0
    in_danger_zone = (12.0 <= front_x <= 22.0)
    for b in sandbox.bodies:
        if not b.active: continue
        fx = THRUST_F
        fy = 0.0
        if in_danger_zone:
            fy = LIFT_F
        vx = b.linearVelocity.x
        if vx > 1.8: fx *= 0.1
        elif vx > 1.0: fx *= 0.4
        sandbox.apply_force(b, fx, fy, step_count=step_count)

def build_agent_stage_1(sandbox):
    bodies = []
    num_beams = 200
    w = 0.15
    h = 0.15
    density = 1.0
    x_center = 5.0
    y0 = 3.0
    for i in range(num_beams):
        x = x_center
        y = y0 + i * 0.0003
        b = sandbox.add_beam(x, y, w, h, angle=0, density=density)
        sandbox.set_material_properties(b, restitution=0.05)
        bodies.append(b)

    for i in range(num_beams - 1):
        ax = (bodies[i].position.x + bodies[i + 1].position.x) / 2
        ay = (bodies[i].position.y + bodies[i + 1].position.y) / 2
        sandbox.add_joint(bodies[i], bodies[i + 1], (ax, ay), type='rigid')
    return bodies[-1]

def agent_action_stage_1(sandbox, agent_body, step_count):
    front_x = sandbox.get_vehicle_front_x()
    if front_x and front_x > 32.0:
        return
    cooldown = 200

    for i, b in enumerate(sandbox.bodies):
        if not b.active:
            continue
        if step_count % cooldown == i % cooldown:
            sandbox.apply_force(b, 500.0, 140.0, step_count=step_count)

def build_agent_stage_2(sandbox):
    bodies = []
    num_beams = 8
    w = 0.5
    h = 0.15
    density = 10.0
    for i in range(num_beams):
        b = sandbox.add_beam(2.5 + i * w, 0.5, w, h, density=density)
        sandbox.set_material_properties(b, restitution=0.05)
        bodies.append(b)
    return bodies[-1]

def agent_action_stage_2(sandbox, agent_body, step_count):
    if agent_body is None or not agent_body.active:
        return
    front_x = sandbox.get_vehicle_front_x()
    if front_x is None or front_x > 32.0:
        return
    for b in sandbox.bodies:
        if not b.active:
            continue
        if front_x < 7.0:
            if b.linearVelocity.x < 2.0:
                sandbox.apply_force(b, 520.0, 0.0, step_count=step_count)
        elif front_x < 25.0:
            sandbox.apply_force(b, 520.0, 520.0, step_count=step_count)
        else:
            sandbox.apply_force(b, 520.0, 0.0, step_count=step_count)

def build_agent_stage_3(sandbox):
    bodies = []
    num_beams = 8
    w = 0.6
    for i in range(num_beams):
        b = sandbox.add_beam(2.5 + i*w, 0.5, w, 0.1, density=0.01)
        sandbox.set_material_properties(b, restitution=0.05)
        bodies.append(b)
    for i in range(num_beams - 1):
        sandbox.add_joint(bodies[i], bodies[i+1], (2.5 + (i+0.5)*w, 0.5))
    return bodies[-1]

def agent_action_stage_3(sandbox, agent_body, step_count):
    front_x = sandbox.get_vehicle_front_x()
    if front_x > 34.0: return
    for b in sandbox.bodies:
        if not b.active: continue
        fx = 300.0 if b.linearVelocity.x < 3.0 else 0.0
        sandbox.apply_force(b, fx, 520.0, step_count=step_count)

def build_agent_stage_4(sandbox):
    b = sandbox.add_beam(2.3, 2.5, 0.3, 0.15, angle=0, density=1.2)
    sandbox.set_material_properties(b, restitution=0.05)
    return b

def agent_action_stage_4(sandbox, agent_body, step_count):
    for b in sandbox.bodies:
        if not b.active:
            continue
        x = b.position.x
        y = b.position.y
        if x >= 25.0:
            b.linearVelocity = (0.0, 0.0)
            b.angularVelocity = 0.0
        elif x < 14.0:
            fy = 400.0 if y < 2.5 else 120.0
            sandbox.apply_force(b, 500.0, fy, step_count=step_count)
        elif x < 19.5:
            sandbox.apply_force(b, 300.0, 520.0, step_count=step_count)
        else:
            fy = 200.0 if y < 2.0 else 0.0
            sandbox.apply_force(b, 520.0, fy, step_count=step_count)
