import math

def build_agent(sandbox):
    col_l = sandbox.add_beam(8.0, 2.0, 0.4, 4.0, angle=0, density=10.0)
    col_r = sandbox.add_beam(12.0, 2.0, 0.4, 4.0, angle=0, density=10.0)
    sandbox.add_joint(col_l, None, (8.0, 0.0), type='rigid')
    sandbox.add_joint(col_r, None, (12.0, 0.0), type='rigid')
    roof = sandbox.add_beam(10.0, 4.2, 6.0, 0.4, angle=0, density=10.0)
    sandbox.add_joint(col_l, roof, (8.0, 4.0), type='rigid')
    sandbox.add_joint(col_r, roof, (12.0, 4.0), type='rigid')
    return col_l

def agent_action(sandbox, agent_body, step_count):
    pass

def build_agent_stage_1(sandbox):
    d = 0.06
    p1 = sandbox.add_beam(8.0, 2.0, 0.2, 3.5, angle=0, density=d)
    p2 = sandbox.add_beam(12.0, 2.0, 0.2, 3.5, angle=0, density=d)
    sandbox.add_joint(p1, None, (8.0, 0.0), type='rigid')
    sandbox.add_joint(p2, None, (12.0, 0.0), type='rigid')
    r1 = sandbox.add_beam(8.5, 3.9, 1.0, 0.15, angle=0, density=d)
    r2 = sandbox.add_beam(9.5, 3.9, 1.0, 0.15, angle=0, density=d)
    r3 = sandbox.add_beam(10.5, 3.9, 1.0, 0.15, angle=0, density=d)
    r4 = sandbox.add_beam(11.5, 3.9, 1.0, 0.15, angle=0, density=d)
    sandbox.add_joint(p1, r1, (8.0, 3.5), type='rigid')
    sandbox.add_joint(r1, r2, (9.0, 3.5), type='rigid')
    sandbox.add_joint(r2, r3, (10.0, 3.5), type='rigid')
    sandbox.add_joint(r3, r4, (11.0, 3.5), type='rigid')
    sandbox.add_joint(p2, r4, (12.0, 3.5), type='rigid')
    return p1

def build_agent_stage_2(sandbox):
    d = 0.035
    post_h = 2.0
    post_xs = [6.3, 6.9, 7.5, 8.1, 8.7, 9.3, 9.9, 10.5, 11.1]
    posts = []
    for px in post_xs:
        p = sandbox.add_beam(px, post_h / 2, 0.15, post_h, angle=0, density=d)
        sandbox.add_joint(p, None, (px, 0.0), type='rigid')
        posts.append(p)
    roof_w = 8.8
    roof_cx = 8.7
    roof_h = 0.10
    roof = sandbox.add_beam(roof_cx, post_h, roof_w, roof_h, angle=0, density=d)
    for px, p in zip(post_xs, posts):
        sandbox.add_joint(p, roof, (px, post_h - roof_h / 2), type='rigid')
    return posts[0]

def build_agent_stage_3(sandbox):
    d = 0.018
    apex_y = 5.4
    left_feet = [6.8, 7.1, 7.4, 7.7]
    right_feet = [13.2, 12.9, 12.6, 12.3]
    ribs = []
    for foot_x, apex_x in [(x, 10.2) for x in left_feet] + [(x, 9.8) for x in right_feet]:
        dx = apex_x - foot_x
        length = math.hypot(dx, apex_y)
        rib = sandbox.add_beam(
            (foot_x + apex_x) / 2,
            apex_y / 2,
            length,
            0.12,
            angle=math.atan2(apex_y, dx),
            density=d,
        )
        sandbox.add_joint(rib, None, (foot_x, 0.04), type='rigid')
        sandbox.add_joint(rib, None, (foot_x - 0.04 if foot_x < 10.0 else foot_x + 0.04, 0.02), type='rigid')
        ribs.append(rib)
    for left_rib, right_rib in zip(ribs[:4], ribs[4:]):
        sandbox.add_joint(left_rib, right_rib, (10.0, 5.35), type='rigid')

    shields = []
    for x in [8.92, 9.08, 10.92, 11.08]:
        shield = sandbox.add_beam(x, 2.45, 0.12, 4.9, angle=0, density=d)
        sandbox.add_joint(shield, None, (x - 0.035, 0.02), type='rigid')
        sandbox.add_joint(shield, None, (x + 0.035, 0.02), type='rigid')
        feet = left_feet if x < 10.0 else right_feet
        apex_x = 10.2 if x < 10.0 else 9.8
        members = ribs[:4] if x < 10.0 else ribs[4:]
        for foot_x, rib in zip(feet, members):
            intersection_y = apex_y * (x - foot_x) / (apex_x - foot_x)
            sandbox.add_joint(shield, rib, (x, intersection_y), type='rigid')
        shields.append(shield)
    return ribs[0]

def build_agent_stage_4(sandbox):
    d = 0.04
    wall_h = 5.8
    wall_specs = [(12.4, 0.2), (14.94, 0.2)]
    walls = []
    for x, width in wall_specs:
        wall = sandbox.add_beam(x, wall_h / 2, width, wall_h, angle=0, density=d)
        for dx in [-0.06, 0.0, 0.06]:
            sandbox.add_joint(wall, None, (x + dx, 0.02), type='rigid')
        walls.append(wall)

    shield = sandbox.add_beam(13.7, 4.05, 3.0, 3.2, angle=0, density=0.02)
    for wall in walls:
        x = wall.position.x
        for y in [2.5, 3.1, 3.7, 4.3, 4.9, 5.5]:
            sandbox.add_joint(shield, wall, (x, y), type='rigid')
    return walls[0]

def agent_action_stage_1(sandbox, agent_body, step_count):
    pass

def agent_action_stage_2(sandbox, agent_body, step_count):
    pass

def agent_action_stage_3(sandbox, agent_body, step_count):
    pass

def agent_action_stage_4(sandbox, agent_body, step_count):
    pass
