def build_agent(sandbox):
    x_left = 12.5
    x_middle = 13.0
    x_right = 13.5
    beam_w = sandbox.MAX_BEAM_WIDTH
    max_h = sandbox.MAX_BEAM_HEIGHT
    min_bottom_y = sandbox.MIN_BEAM_BOTTOM_Y
    density = 44.0
    left_layer_h = min(0.7, max_h)
    left_n = 8
    left_first_y = min_bottom_y + left_layer_h / 2
    left_beams = []
    for i in range(left_n):
        cy = left_first_y + i * left_layer_h
        b = sandbox.add_beam(x_left, cy, width=beam_w, height=left_layer_h, angle=0, density=density)
        sandbox.set_material_properties(b, restitution=0.05)
        left_beams.append(b)
    for i in range(1, left_n):
        y_anchor = (left_beams[i].position.y + left_beams[i - 1].position.y) / 2
        sandbox.add_joint(left_beams[i], left_beams[i - 1], (x_left, y_anchor), type='rigid')
    middle_h = min(0.6, max_h)
    middle_y = 6.0
    middle_beam = sandbox.add_beam(x_middle, middle_y, width=beam_w, height=middle_h, angle=0, density=density)
    sandbox.set_material_properties(middle_beam, restitution=0.05)
    right_layer_h = min(1.5, max_h)
    right_n = 2
    right_first_y = min_bottom_y + right_layer_h / 2
    right_beams = []
    for i in range(right_n):
        cy = right_first_y + i * right_layer_h
        b = sandbox.add_beam(x_right, cy, width=beam_w, height=right_layer_h, angle=0, density=density)
        sandbox.set_material_properties(b, restitution=0.05)
        right_beams.append(b)
    sandbox.add_joint(right_beams[1], right_beams[0], (x_right, (right_beams[1].position.y + right_beams[0].position.y) / 2), type='rigid')
    top_left = left_beams[-1]
    anchor_left_mid_x = (x_left + x_middle) / 2
    anchor_left_mid_y = (top_left.position.y + middle_beam.position.y) / 2
    sandbox.add_joint(top_left, middle_beam, (anchor_left_mid_x, anchor_left_mid_y), type='rigid')
    anchor_mid_right_x = (x_middle + x_right) / 2
    anchor_mid_right_y = (middle_beam.position.y + right_beams[1].position.y) / 2
    sandbox.add_joint(middle_beam, right_beams[1], (anchor_mid_right_x, anchor_mid_right_y), type='rigid')
    cross_anchor_y = (left_beams[0].position.y + right_beams[0].position.y) / 2
    sandbox.add_joint(left_beams[0], right_beams[0], (13.0, cross_anchor_y), type='rigid')
    return left_beams[0]

def agent_action(sandbox, agent_body, step_count):
    pass

def build_agent_stage_1(sandbox):
    left = []
    for i in range(7):
        x = 12.49 if i % 2 == 0 else 12.51
        beam = sandbox.add_beam(x, 1.051 + 0.95 * i, width=0.6, height=1.1, angle=0, density=11.5)
        sandbox.set_material_properties(beam, restitution=0.0)
        sandbox.set_damping(beam, linear=40.0, angular=45.0)
        left.append(beam)
    middle = sandbox.add_beam(13.0, 3.8, width=0.6, height=1.4, angle=0, density=11.5)
    low_right = sandbox.add_beam(13.58, 1.25, width=0.6, height=1.5, angle=0, density=11.5)
    high_right = sandbox.add_beam(13.42, 5.75, width=0.6, height=1.5, angle=0, density=11.5)
    for beam in (middle, low_right, high_right):
        sandbox.set_material_properties(beam, restitution=0.0)
        sandbox.set_damping(beam, linear=40.0, angular=45.0)
    for i in range(1, 7):
        sandbox.add_joint(left[i - 1], left[i], (12.5, 0.575 + 0.95 * i), type='rigid')
    sandbox.add_joint(left[3], middle, (12.75, 3.8), type='rigid')
    sandbox.add_joint(left[1], low_right, (13.0, 1.9), type='rigid')
    sandbox.add_joint(left[5], high_right, (13.0, 5.7), type='rigid')
    sandbox.add_joint(left[0], low_right, (13.1, 1.2), type='rigid')
    sandbox.add_joint(left[2], low_right, (13.1, 2.8), type='rigid')
    sandbox.add_joint(left[4], high_right, (13.1, 4.9), type='rigid')
    sandbox.add_joint(left[6], high_right, (13.1, 6.6), type='rigid')
    sandbox.add_joint(middle, low_right, (13.25, 2.5), type='rigid')
    sandbox.add_joint(middle, high_right, (13.25, 4.8), type='rigid')
    return {
        'body': left[0], 'left_1': left[1], 'left_2': left[2], 'left_3': left[3],
        'left_4': left[4], 'left_5': left[5], 'left_6': left[6], 'middle': middle,
        'low_right': low_right, 'high_right': high_right,
    }

def agent_action_stage_1(sandbox, agent_body, step_count):
    targets = {
        'body': (12.49, 1.051), 'left_1': (12.51, 2.001), 'left_2': (12.49, 2.951),
        'left_3': (12.51, 3.901), 'left_4': (12.49, 4.851), 'left_5': (12.51, 5.801),
        'left_6': (12.49, 6.751), 'middle': (13.0, 3.8), 'low_right': (13.58, 1.25),
        'high_right': (13.42, 5.75),
    }
    for name, body in agent_body.items():
        tx, ty = targets[name]
        fx = body.mass * (80.0 * (tx - body.position.x) - 20.0 * body.linearVelocity.x - 9.0)
        fy = body.mass * (45.0 * (ty - body.position.y) - 16.0 * body.linearVelocity.y + 7.0)
        sandbox.apply_force(body, (fx, fy))

def build_agent_stage_2(sandbox):
    left = []
    for i in range(7):
        x = 12.51 if i % 2 == 0 else 12.49
        angle = 0.01 if i % 2 == 0 else -0.01
        beam = sandbox.add_beam(x, 1.08 + 0.94 * i, width=0.6, height=1.1, angle=angle, density=9.0)
        sandbox.set_material_properties(beam, restitution=0.0)
        sandbox.set_damping(beam, linear=50.0, angular=55.0)
        left.append(beam)
    middle = sandbox.add_beam(12.92, 6.65, width=0.6, height=1.2, angle=-0.12, density=9.0)
    low_right = sandbox.add_beam(13.42, 2.1, width=0.6, height=1.4, angle=0.08, density=9.0)
    high_right = sandbox.add_beam(13.58, 5.0, width=0.6, height=1.4, angle=-0.08, density=9.0)
    for beam in (middle, low_right, high_right):
        sandbox.set_material_properties(beam, restitution=0.0)
        sandbox.set_damping(beam, linear=50.0, angular=55.0)
    path = [low_right, left[0], left[2], left[4], left[6], middle, high_right, left[5], left[3], left[1]]
    for first, second in zip(path, path[1:]):
        sandbox.add_joint(first, second, ((first.position.x + second.position.x) / 2, (first.position.y + second.position.y) / 2), type='rigid')
    for i in range(1, 7):
        sandbox.add_joint(left[i - 1], left[i], (12.5, 0.61 + 0.94 * i), type='rigid')
    return {
        'body': low_right, 'left_0': left[0], 'left_2': left[2], 'left_4': left[4],
        'left_6': left[6], 'middle': middle, 'high_right': high_right, 'left_5': left[5],
        'left_3': left[3], 'left_1': left[1],
    }

def agent_action_stage_2(sandbox, agent_body, step_count):
    targets = {
        'body': (13.42, 2.1), 'left_0': (12.51, 1.08), 'left_2': (12.51, 2.96),
        'left_4': (12.51, 4.84), 'left_6': (12.51, 6.72), 'middle': (12.92, 6.65),
        'high_right': (13.58, 5.0), 'left_5': (12.49, 5.78), 'left_3': (12.49, 3.9),
        'left_1': (12.49, 2.02),
    }
    cycle = step_count % 2000
    gain = 150.0 if cycle < 100 or cycle > 1900 else 95.0
    drag = 32.0 if cycle < 100 or cycle > 1900 else 24.0
    shift = -0.5 * min(step_count / 500.0, 1.0)
    for name, body in agent_body.items():
        base_x, ty = targets[name]
        tx = base_x + shift
        fx = body.mass * (gain * (tx - body.position.x) - drag * body.linearVelocity.x - 2.0)
        fy = body.mass * (60.0 * (ty - body.position.y) - 22.0 * body.linearVelocity.y + 10.0)
        sandbox.apply_force(body, (fx, fy))

def build_agent_stage_3(sandbox):
    left = []
    for i in range(7):
        x = 12.5
        angle = 0.0
        beam = sandbox.add_beam(x, 1.1 + i, width=0.6, height=1.2, angle=angle, density=1.2)
        sandbox.set_material_properties(beam, restitution=0.0)
        sandbox.set_damping(beam, linear=160.0, angular=180.0)
        left.append(beam)
    hub = sandbox.add_beam(13.0, 3.75, width=0.6, height=1.5, angle=0, density=1.2)
    low_right = sandbox.add_beam(13.45, 1.35, width=0.6, height=1.5, angle=-0.08, density=1.2)
    high_right = sandbox.add_beam(13.55, 6.25, width=0.6, height=1.5, angle=0.08, density=1.2)
    for beam in (hub, low_right, high_right):
        sandbox.set_material_properties(beam, restitution=0.0)
        sandbox.set_damping(beam, linear=160.0, angular=180.0)
    for beam in left:
        sandbox.add_joint(hub, beam, ((hub.position.x + beam.position.x) / 2, (hub.position.y + beam.position.y) / 2), type='rigid')
    sandbox.add_joint(hub, low_right, (13.25, 2.55), type='rigid')
    sandbox.add_joint(hub, high_right, (13.25, 5.0), type='rigid')
    for i in range(1, 7):
        sandbox.add_joint(left[i - 1], left[i], (12.5, 0.61 + 0.94 * i), type='rigid')
    return {
        'body': hub, 'left_0': left[0], 'left_1': left[1], 'left_2': left[2],
        'left_3': left[3], 'left_4': left[4], 'left_5': left[5], 'left_6': left[6],
        'low_right': low_right, 'high_right': high_right,
    }

def agent_action_stage_3(sandbox, agent_body, step_count):
    targets = {
        'body': (13.0, 3.75), 'left_0': (12.5, 1.1), 'left_1': (12.5, 2.1),
        'left_2': (12.5, 3.1), 'left_3': (12.5, 4.1), 'left_4': (12.5, 5.1),
        'left_5': (12.5, 6.1), 'left_6': (12.5, 7.1), 'low_right': (13.45, 1.35),
        'high_right': (13.55, 6.25),
    }
    pulse = -8.0 if step_count % 2500 < 140 else 0.0
    for name, body in agent_body.items():
        base_x, ty = targets[name]
        tx = base_x
        radial = 1.0 if name == 'body' else 0.72
        fx = body.mass * (radial * 280.0 * (tx - body.position.x) - 52.0 * body.linearVelocity.x - 9.0 + pulse)
        fy = body.mass * (radial * 160.0 * (ty - body.position.y) - 38.0 * body.linearVelocity.y + 8.0)
        sandbox.apply_force(body, (fx, fy))

def build_agent_stage_4(sandbox):
    left = []
    for i in range(7):
        x = 12.5
        angle = 0.0
        beam = sandbox.add_beam(x, 1.1 + i, width=0.6, height=1.2, angle=angle, density=1.8)
        sandbox.set_material_properties(beam, restitution=0.0)
        sandbox.set_damping(beam, linear=240.0, angular=260.0)
        left.append(beam)
    middle = sandbox.add_beam(13.0, 4.45, width=0.6, height=1.5, angle=-0.08, density=1.8)
    low_right = sandbox.add_beam(13.43, 1.35, width=0.6, height=1.5, angle=0.08, density=1.8)
    high_right = sandbox.add_beam(13.57, 6.25, width=0.6, height=1.5, angle=-0.08, density=1.8)
    for beam in (middle, low_right, high_right):
        sandbox.set_material_properties(beam, restitution=0.0)
        sandbox.set_damping(beam, linear=240.0, angular=260.0)
    for i in range(1, 7):
        sandbox.add_joint(left[i - 1], left[i], (12.5, 0.6 + i), type='rigid')
    sandbox.add_joint(low_right, high_right, (13.5, 3.8), type='rigid')
    sandbox.add_joint(left[4], middle, (12.75, 4.775), type='rigid')
    sandbox.add_joint(left[0], low_right, (12.965, 1.225), type='rigid')
    sandbox.add_joint(left[2], low_right, (12.965, 2.225), type='rigid')
    sandbox.add_joint(left[5], high_right, (13.035, 6.175), type='rigid')
    sandbox.add_joint(left[6], high_right, (13.035, 6.675), type='rigid')
    sandbox.add_joint(middle, low_right, (13.215, 2.9), type='rigid')
    sandbox.add_joint(left[3], middle, (12.75, 4.275), type='rigid')
    return {
        'body': left[0], 'left_1': left[1], 'left_2': left[2], 'left_3': left[3],
        'left_4': left[4], 'left_5': left[5], 'left_6': left[6],
        'middle': middle, 'low_right': low_right, 'high_right': high_right,
    }

def agent_action_stage_4(sandbox, agent_body, step_count):
    targets = {
        'body': (12.5, 1.1), 'left_1': (12.5, 2.1), 'left_2': (12.5, 3.1),
        'left_3': (12.5, 4.1), 'left_4': (12.5, 5.1), 'left_5': (12.5, 6.1),
        'left_6': (12.5, 7.1), 'middle': (13.0, 4.45),
        'low_right': (13.43, 1.35), 'high_right': (13.57, 6.25),
    }
    phase = step_count % 1000
    gain = 235.0 if phase < 120 or phase > 880 else 165.0
    drag = 44.0 if phase < 120 or phase > 880 else 34.0
    for name, body in agent_body.items():
        base_x, ty = targets[name]
        tx = base_x
        fx = body.mass * (gain * (tx - body.position.x) - drag * body.linearVelocity.x - 10.0)
        fy = body.mass * (128.0 * (ty - body.position.y) - 32.0 * body.linearVelocity.y + 9.0)
        sandbox.apply_force(body, (fx, fy))
