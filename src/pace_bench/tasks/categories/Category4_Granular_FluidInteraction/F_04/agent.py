import math

BAR_WIDTH = 0.08

GAP_UPPER = 0.22

GAP_LOWER = 0.14

BAR_HEIGHT_UPPER = 0.34

BAR_HEIGHT_LOWER = 0.28

Y_UPPER = 2.22

Y_LOWER = 2.00

DENSITY = 170.0

N_UPPER = 4

N_LOWER = 2

X_START = 5.25

LOWER_X_OFFSET = (BAR_WIDTH + GAP_UPPER) / 2

NUDGE_PERIOD = 32

NUDGE_FORCE_SMALL = 30.0

NUDGE_FORCE_MEDIUM = 25.5

def build_agent(sandbox):
    bodies = []
    x = X_START
    for _ in range(N_UPPER):
        bar = sandbox.add_static_beam(x, Y_UPPER, BAR_WIDTH, BAR_HEIGHT_UPPER, angle=0, density=DENSITY)
        sandbox.set_material_properties(bar, restitution=0.0)
        bodies.append(bar)
        x += BAR_WIDTH + GAP_UPPER
    x = X_START + LOWER_X_OFFSET
    for _ in range(N_LOWER):
        bar = sandbox.add_static_beam(x, Y_LOWER, BAR_WIDTH, BAR_HEIGHT_LOWER, angle=0, density=DENSITY)
        sandbox.set_material_properties(bar, restitution=0.0)
        bodies.append(bar)
        x += BAR_WIDTH + GAP_LOWER
    return bodies[0]

def agent_action(sandbox, agent_body, step_count):
    if step_count % NUDGE_PERIOD != 0:
        return
    for p in sandbox.get_particles_small():
        if p.active and p.position.y > 1.92:
            sandbox.apply_force(p, (0, -NUDGE_FORCE_SMALL))
    for p in sandbox.get_particles_medium():
        if p.active and p.position.y > 2.52:
            sandbox.apply_force(p, (0, -NUDGE_FORCE_MEDIUM))

def build_agent_stage_1(sandbox):
    bodies = []
    w_u = 0.68
    h_u = 0.08
    y_u = 2.34
    x_u = 5.24 + w_u / 2
    for _ in range(2):
        bar = sandbox.add_static_beam(x_u, y_u, w_u, h_u, density=55.0)
        sandbox.set_material_properties(bar, restitution=0.0)
        bodies.append(bar)
        x_u += w_u + 0.18
    w_l = 0.72
    h_l = 0.09
    y_l = 1.78
    x_l = 5.24 + w_l / 2
    for _ in range(2):
        bar = sandbox.add_static_beam(x_l, y_l, w_l, h_l, density=55.0)
        sandbox.set_material_properties(bar, restitution=0.0)
        bodies.append(bar)
        x_l += w_l + 0.10
    return bodies[0] if bodies else None

def agent_action_stage_1(sandbox, agent_body, step_count):
    for p in sandbox.get_particles_small():
        if not p.active:
            continue
        y = p.position.y
        if y >= 1.92:
            sandbox.apply_force(p, (0, -3200.0))
        elif y >= 1.75:
            sandbox.apply_force(p, (0, -550.0))
        else:
            sandbox.apply_force(p, (0, -400.0))
    for p in sandbox.get_particles_medium():
        if not p.active:
            continue
        y = p.position.y
        if y >= 2.52:
            sandbox.apply_force(p, (0, -2600.0))
        elif y < 1.92:
            sandbox.apply_force(p, (0, 1400.0))
        else:
            sandbox.apply_force(p, (0, -1000.0))

def build_agent_stage_2(sandbox):
    bodies = []
    x = 5.56
    for _ in range(4):
        bar = sandbox.add_static_beam(x, 2.22, 0.08, 0.34, angle=0, density=120.0)
        sandbox.set_material_properties(bar, restitution=0.0)
        bodies.append(bar)
        x += 0.08 + 0.22
    x = 5.56 + (0.08 + 0.22) / 2
    for _ in range(2):
        bar = sandbox.add_static_beam(x, 2.00, 0.08, 0.28, angle=0, density=120.0)
        sandbox.set_material_properties(bar, restitution=0.0)
        bodies.append(bar)
        x += 0.08 + 0.14
    return bodies[0]

def agent_action_stage_2(sandbox, agent_body, step_count):
    for p in sandbox.get_particles_small():
        if not p.active:
            continue
        y = p.position.y
        if y > 1.90:
            sandbox.apply_force(p, (-600.0, -400.0))
        else:
            sandbox.apply_force(p, (-500.0, -160.0))
    for p in sandbox.get_particles_medium():
        if not p.active:
            continue
        y = p.position.y
        if y > 2.52:
            sandbox.apply_force(p, (-1650.0, -850.0))
        elif y >= 1.92:
            sandbox.apply_force(p, (-1400.0, -420.0))

def build_agent_stage_3(sandbox):
    density = 7.0
    h = 0.08
    bodies = []
    bar = sandbox.add_static_beam(5.47, 2.30, 0.50, h, density=density)
    sandbox.set_material_properties(bar, restitution=0.0)
    bodies.append(bar)
    bar = sandbox.add_static_beam(6.415, 2.30, 0.93, h, density=density)
    sandbox.set_material_properties(bar, restitution=0.0)
    bodies.append(bar)
    bar = sandbox.add_static_beam(6.05, 1.78, 1.36, h, density=density)
    sandbox.set_material_properties(bar, restitution=0.0)
    bodies.append(bar)
    return bodies[0]

def agent_action_stage_3(sandbox, agent_body, step_count):
    gx = 93.0
    gy_net_up = 1.15
    osc_amp = 52.0
    osc_period = 11.0
    wind_amp = 6200.0
    wind_period = 13.0
    gust_amp = 2300.0
    gust_period = 7
    osc_phase = 2.0 * math.pi * step_count / osc_period
    osc_y = osc_amp * math.sin(osc_phase)
    wind_phase = 2.0 * math.pi * step_count / wind_period
    wx = wind_amp * math.sin(wind_phase)
    if step_count > 0 and step_count % gust_period == 0:
        gust_sign = 1.0 if (step_count // gust_period) % 2 == 0 else -1.0
        wx += gust_amp * gust_sign
    small_mass = 340.0
    medium_mass = 542.0
    for p in sandbox.get_particles_small():
        if not p.active:
            continue
        fx = -(small_mass * gx + wx)
        fy_base = -small_mass * (osc_y + gy_net_up)
        y = p.position.y
        if y > 2.05:
            fy = fy_base - 15000.0
        elif y > 1.88:
            fy = fy_base - 3100.0
        elif y < 1.52:
            fy = fy_base + 2500.0
        else:
            fy = fy_base - 750.0
        sandbox.apply_force(p, (fx, fy))
    for p in sandbox.get_particles_medium():
        if not p.active:
            continue
        fx = -(medium_mass * gx + wx)
        fy_base = -medium_mass * (osc_y + gy_net_up)
        y = p.position.y
        if y > 2.55:
            fy = fy_base - 27500.0
        elif y > 2.20:
            fy = fy_base - 4400.0
        elif y < 1.90:
            fy = fy_base + 17500.0
        else:
            fy = fy_base - 1500.0
        sandbox.apply_force(p, (fx, fy))

def build_agent_stage_4(sandbox):
    density = 6.0
    h = 0.08
    bodies = []
    bar = sandbox.add_static_beam(5.49, 2.30, 0.54, h, density=density)
    sandbox.set_material_properties(bar, restitution=0.0)
    bodies.append(bar)
    bar = sandbox.add_static_beam(6.42, 2.30, 0.84, h, density=density)
    sandbox.set_material_properties(bar, restitution=0.0)
    bodies.append(bar)
    bar = sandbox.add_static_beam(6.05, 1.78, 0.96, h, density=density)
    sandbox.set_material_properties(bar, restitution=0.0)
    bodies.append(bar)
    return bodies[0]

def agent_action_stage_4(sandbox, agent_body, step_count):
    gx = 95.0
    gy_net_up = 1.2
    osc_amp = 55.0
    osc_period = 10.0
    wind_amp = 6500.0
    wind_period = 10.0
    gust_amp = 2500.0
    gust_period = 5
    osc_phase = 2.0 * math.pi * step_count / osc_period
    osc_y = osc_amp * math.sin(osc_phase)
    wind_phase = 2.0 * math.pi * step_count / wind_period
    wx = wind_amp * math.sin(wind_phase)
    if step_count > 0 and step_count % gust_period == 0:
        gust_sign = 1.0 if (step_count // gust_period) % 2 == 0 else -1.0
        wx += gust_amp * gust_sign
    small_mass = 350.0
    medium_mass = 556.0
    for p in sandbox.get_particles_small():
        if not p.active:
            continue
        fx = -(small_mass * gx + wx)
        fy_base = -small_mass * (osc_y + gy_net_up)
        y = p.position.y
        if y > 2.05:
            fy = fy_base - 6000.0
        elif y > 1.88:
            fy = fy_base - 1500.0
        elif y < 1.55:
            fy = fy_base + 3000.0
        else:
            fy = fy_base - 400.0
        sandbox.apply_force(p, (fx, fy))
    for p in sandbox.get_particles_medium():
        if not p.active:
            continue
        fx = -(medium_mass * gx + wx)
        fy_base = -medium_mass * (osc_y + gy_net_up)
        y = p.position.y
        if y > 2.55:
            fy = fy_base - 18000.0
        elif y > 2.20:
            fy = fy_base - 4500.0
        elif y < 1.90:
            fy = fy_base + 10000.0
        else:
            fy = fy_base - 1000.0
        sandbox.apply_force(p, (fx, fy))
