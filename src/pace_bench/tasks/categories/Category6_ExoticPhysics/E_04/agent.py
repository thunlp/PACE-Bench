import math

def build_agent(sandbox):
    ground_top = sandbox.get_ground_y_top()
    x_min, x_max, _, _ = sandbox.get_build_zone()
    span_left, span_right = sandbox.get_span_bounds()
    deck_cy = 2.0
    deck_h = 0.2
    density = 0.05
    strut_w = 0.18
    strut_density = 0.05
    strut_h = deck_cy - ground_top
    strut_cy = ground_top + strut_h / 2
    left_span = sandbox.add_beam(
        x=span_left, y=deck_cy,
        width=0.5, height=deck_h,
        angle=0, density=density,
    )
    right_span = sandbox.add_beam(
        x=span_right, y=deck_cy,
        width=0.5, height=deck_h,
        angle=0, density=density,
    )
    deck_left = sandbox.add_beam(
        x=8.0, y=deck_cy,
        width=2.0, height=deck_h,
        angle=0, density=density,
    )
    deck_right = sandbox.add_beam(
        x=12.0, y=deck_cy,
        width=2.0, height=deck_h,
        angle=0, density=density,
    )
    strut_xs = [6.0, 7.0, 8.0, 9.0, 11.0, 12.0, 13.0, 14.0]
    strut_bodies = []
    for sx in strut_xs:
        s = sandbox.add_beam(
            x=sx, y=strut_cy,
            width=strut_w, height=strut_h,
            angle=0, density=strut_density,
        )
        strut_bodies.append((sx, s))
        sandbox.add_joint(s, None, (sx, ground_top), type="rigid")
        if sx <= 9:
            sandbox.add_joint(deck_left, s, (sx, deck_cy), type="rigid")
        else:
            sandbox.add_joint(deck_right, s, (sx, deck_cy), type="rigid")
    sx_center = 10.0
    strut_center = sandbox.add_beam(
        x=sx_center, y=strut_cy,
        width=strut_w, height=strut_h,
        angle=0, density=strut_density,
    )
    sandbox.add_joint(strut_center, None, (sx_center, ground_top), type="rigid")
    sandbox.add_joint(deck_left, strut_center, (sx_center, deck_cy), type="rigid")
    sandbox.add_joint(deck_right, strut_center, (sx_center, deck_cy), type="pivot")
    sandbox.add_joint(left_span, deck_left, (7.0, deck_cy), type="rigid")
    sandbox.add_joint(deck_left, deck_right, (10.0, deck_cy), type="rigid")
    sandbox.add_joint(deck_right, right_span, (13.0, deck_cy), type="rigid")
    sandbox.add_joint(left_span, strut_bodies[0][1], (span_left, deck_cy), type="rigid")
    sandbox.add_joint(right_span, strut_bodies[-1][1], (span_right, deck_cy), type="rigid")
    return deck_left

def agent_action(sandbox, agent_body, step_count):
    pass

def build_agent_stage_1(sandbox):
    ground_y = sandbox.get_ground_y_top()
    span_left, span_right = sandbox.get_span_bounds()
    density = 0.05
    deck_y = ground_y + 1.2
    center_x = (span_left + span_right) / 2.0

    piers = []
    for x in (span_left, center_x, span_right):
        pier = sandbox.add_beam(
            x=x,
            y=(ground_y + deck_y) / 2.0,
            width=0.1,
            height=deck_y - ground_y,
            density=density,
        )
        piers.append(pier)
        sandbox.add_joint(pier, None, (x, ground_y), type="pivot")

    left_girder = sandbox.add_beam(
        x=(span_left + center_x) / 2.0,
        y=deck_y,
        width=center_x - span_left,
        height=0.1,
        density=density,
    )
    right_girder = sandbox.add_beam(
        x=(center_x + span_right) / 2.0,
        y=deck_y,
        width=span_right - center_x,
        height=0.1,
        density=density,
    )
    sandbox.add_joint(left_girder, piers[0], (span_left, deck_y), type="pivot")
    sandbox.add_joint(left_girder, piers[1], (center_x, deck_y), type="pivot")
    sandbox.add_joint(right_girder, piers[1], (center_x, deck_y), type="pivot")
    sandbox.add_joint(right_girder, piers[2], (span_right, deck_y), type="pivot")

    for ground_x, top_x, pier in (
        (span_left + 0.2, center_x, piers[1]),
        (center_x + 0.2, span_right, piers[2]),
    ):
        dx = top_x - ground_x
        dy = deck_y - ground_y
        brace = sandbox.add_beam(
            x=(ground_x + top_x) / 2.0,
            y=(ground_y + deck_y) / 2.0,
            width=math.sqrt(dx * dx + dy * dy),
            height=0.1,
            angle=-math.atan2(dy, dx),
            density=density,
        )
        sandbox.add_joint(brace, None, (ground_x, ground_y), type="pivot")
        sandbox.add_joint(brace, pier, (top_x, deck_y), type="pivot")
    return left_girder

def agent_action_stage_1(sandbox, agent_body, step_count): pass

def build_agent_stage_2(sandbox):
    L_X, R_X = 6.0, 14.0
    density = 0.0001
    n = 10
    nodes = []
    for i in range(n + 1):
        x = L_X + i * (R_X - L_X) / n
        y = 4.5 - 0.15 * (x - 10.0)**2
        nodes.append((x, y))
    arch = []
    for i in range(n):
        x1, y1 = nodes[i]; x2, y2 = nodes[i+1]
        d = math.sqrt((x2-x1)**2 + (y2-y1)**2)
        b = sandbox.add_beam(x=(x1+x2)/2, y=(y1+y2)/2, width=d+0.05, height=0.1,
                             angle=-math.atan2(y2-y1, x2-x1), density=density)
        arch.append(b)
        if i == 0: sandbox.add_joint(b, None, nodes[i], type="pivot")
        elif i == n-1:
            sandbox.add_joint(arch[i-1], b, nodes[i], type="pivot")
            sandbox.add_joint(b, None, nodes[i+1], type="pivot")
        else: sandbox.add_joint(arch[i-1], b, nodes[i], type="pivot")
    deck = []
    for i in range(n):
        x = (nodes[i][0] + nodes[i+1][0]) / 2
        db = sandbox.add_beam(x=x, y=4.7, width=(R_X-L_X)/n + 0.05, height=0.1, density=density)
        deck.append(db)
        v = sandbox.add_beam(x=x, y=(nodes[i][1]+4.7)/2, width=0.05, height=4.7-nodes[i][1], density=density)
        sandbox.add_joint(v, arch[i], (x, nodes[i][1]), type="pivot")
        sandbox.add_joint(v, db, (x, 4.7), type="pivot")
        if i > 0: sandbox.add_joint(deck[i-1], db, (nodes[i][0], 4.7), type="pivot")
    sandbox.add_beam(x=6.0, y=4.7, width=0.1, height=0.1, density=density)
    sandbox.add_beam(x=14.0, y=4.7, width=0.1, height=0.1, density=density)
    return deck[0]

def agent_action_stage_2(sandbox, agent_body, step_count): pass

def build_agent_stage_3(sandbox):
    density = 0.00000001
    nodes = []
    for x in (6.0, 8.0, 10.0, 12.0, 14.0):
        node = sandbox.add_beam(
            x=x,
            y=1.56,
            width=0.1,
            height=0.1,
            angle=0,
            density=density,
        )
        nodes.append(node)
        sandbox.add_joint(node, None, (x - 0.025, 1.5), type="pivot")
        sandbox.add_joint(node, None, (x + 0.025, 1.5), type="pivot")
    return nodes[2]

def agent_action_stage_3(sandbox, agent_body, step_count): pass

def build_agent_stage_4(sandbox):
    L_X, R_X = 6.0, 14.0
    density = 0.0001
    deck_y = 3.0
    n = 12
    w = (R_X - L_X) / n
    deck = []
    for i in range(n):
        b = sandbox.add_beam(x=L_X+(i+0.5)*w, y=deck_y, width=w+0.05, height=0.05, density=density)
        deck.append(b)
        if i > 0: sandbox.add_joint(deck[i-1], b, (L_X+i*w, deck_y), type="pivot")
    for px in [6.5, 8.5, 11.5, 13.5]:
        pylon = sandbox.add_beam(x=px, y=5.0, width=0.1, height=4.0, density=density*2)
        sandbox.add_joint(pylon, None, (px, 1.5), type="pivot")
        for sy in [4.0, 6.0]:
            stay = sandbox.add_beam(x=px-1.0, y=(1.5+sy)/2, width=math.sqrt(2.0**2+(sy-1.5)**2),
                                    height=0.02, angle=-math.atan2(sy-1.5, -2.0), density=density)
            sandbox.add_joint(stay, None, (px-2.0, 1.5), type="pivot")
            sandbox.add_joint(stay, pylon, (px, sy), type="pivot")
        for i in range(n):
            dx = (L_X+(i+0.5)*w) - px
            dist = math.sqrt(dx**2 + (7.0-deck_y)**2)
            s = sandbox.add_beam(x=px+dx/2, y=(7.0+deck_y)/2, width=dist, height=0.02,
                                 angle=-math.atan2(deck_y-7.0, dx), density=density)
            sandbox.add_joint(s, pylon, (px, 7.0), type="pivot")
            sandbox.add_joint(s, deck[i], (L_X+(i+0.5)*w, deck_y), type="pivot")
    sandbox.add_beam(x=6.0, y=deck_y, width=0.1, height=0.1, density=density)
    sandbox.add_beam(x=14.0, y=deck_y, width=0.1, height=0.1, density=density)
    return deck[0]

def agent_action_stage_4(sandbox, agent_body, step_count): pass
