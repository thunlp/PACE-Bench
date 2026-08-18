import math

def build_truss(sandbox, start_x, end_x, top_y, bottom_y, num_panels, deck_density=50.0, truss_density=20.0, joint_type='rigid'):
    gap_width = end_x - start_x
    panel_width = gap_width / num_panels
    deck_height = 0.4
    deck_beams = []
    for i in range(num_panels):
        cx = start_x + (i + 0.5) * panel_width
        b = sandbox.add_beam(x=cx, y=top_y, width=panel_width+0.01, height=deck_height, density=deck_density)
        deck_beams.append(b)
    bottom_beams = []
    for i in range(num_panels):
        cx = start_x + (i + 0.5) * panel_width
        b = sandbox.add_beam(x=cx, y=bottom_y, width=panel_width+0.01, height=0.3, density=truss_density)
        bottom_beams.append(b)
    v_beams = []
    for i in range(num_panels + 1):
        x = start_x + i * panel_width
        b = sandbox.add_beam(x=x, y=(top_y+bottom_y)/2, width=0.3, height=abs(top_y-bottom_y), density=truss_density)
        v_beams.append(b)
    diag_beams = []
    for i in range(num_panels):
        x1, y1 = start_x + i * panel_width, bottom_y
        x2, y2 = start_x + (i+1) * panel_width, top_y
        cx, cy = (x1+x2)/2, (y1+y2)/2
        dist = math.sqrt((x2-x1)**2 + (y2-y1)**2)
        angle = math.atan2(y2-y1, x2-x1)
        b = sandbox.add_beam(x=cx, y=cy, width=dist+0.1, height=0.3, angle=angle, density=truss_density)
        diag_beams.append(b)
    for i in range(num_panels + 1):
        nx = start_x + i * panel_width
        node_pos = (nx, top_y)
        master = v_beams[i]
        if i > 0: sandbox.add_joint(master, deck_beams[i-1], node_pos, type='rigid')
        if i < num_panels: sandbox.add_joint(master, deck_beams[i], node_pos, type='rigid')
        if i > 0: sandbox.add_joint(master, diag_beams[i-1], node_pos, type=joint_type)
    for i in range(num_panels + 1):
        nx = start_x + i * panel_width
        node_pos = (nx, bottom_y)
        master = v_beams[i]
        if i > 0: sandbox.add_joint(master, bottom_beams[i-1], node_pos, type=joint_type)
        if i < num_panels: sandbox.add_joint(master, bottom_beams[i], node_pos, type=joint_type)
        if i < num_panels: sandbox.add_joint(master, diag_beams[i], node_pos, type=joint_type)
    return deck_beams, bottom_beams, v_beams


def build_agent(sandbox):
    L_X, R_X = 10.0, 25.0
    deck, bottom, v_beams = build_truss(sandbox, L_X, R_X, 10.0, 8.0, 6, deck_density=35.0, truss_density=15.0, joint_type='rigid')
    sandbox.add_joint(v_beams[0], None, (L_X, 10.0), type='rigid')
    sandbox.add_joint(v_beams[-1], None, (R_X, 10.0), type='rigid')
    ext = sandbox.add_beam(x=R_X+2.5, y=10.0, width=5.0, height=0.4, density=30.0)
    sandbox.add_joint(v_beams[-1], ext, (R_X, 10.0), type='rigid')
    return deck[0]

def agent_action(sandbox, agent_body, step_count):
    pass

def build_agent_stage_1(sandbox):
    L_X, R_X = 10.0, 25.0
    num_panels = 6
    top_y = 9.8
    bottom_y = 7.5
    gap_width = R_X - L_X
    panel_width = gap_width / num_panels
    deck_beams = []
    bottom_beams = []
    v_beams = []
    diag_beams = []
    for i in range(num_panels):
        cx = L_X + (i + 0.5) * panel_width
        b = sandbox.add_beam(x=cx, y=top_y, width=panel_width+0.01, height=0.4, density=25.0)
        deck_beams.append(b)
        b_bottom = sandbox.add_beam(x=cx, y=bottom_y, width=panel_width+0.01, height=0.3, density=10.5)
        bottom_beams.append(b_bottom)
    for i in range(num_panels + 1):
        x = L_X + i * panel_width
        b_v = sandbox.add_beam(x=x, y=(top_y+bottom_y)/2, width=0.3, height=abs(top_y-bottom_y), density=10.5)
        v_beams.append(b_v)
    for i in range(num_panels):
        x1, y1 = L_X + i * panel_width, bottom_y
        x2, y2 = L_X + (i+1) * panel_width, top_y
        cx, cy = (x1+x2)/2, (y1+y2)/2
        dist = math.sqrt((x2-x1)**2 + (y2-y1)**2)
        angle = math.atan2(y2-y1, x2-x1)
        b_diag = sandbox.add_beam(x=cx, y=cy, width=dist+0.1, height=0.3, angle=angle, density=10.5)
        diag_beams.append(b_diag)
    for i in range(num_panels + 1):
        nx = L_X + i * panel_width
        node_pos_top = (nx, top_y)
        node_pos_bottom = (nx, bottom_y)
        master_v = v_beams[i]
        if i > 0: sandbox.add_joint(master_v, deck_beams[i-1], node_pos_top, type='pivot')
        if i < num_panels: sandbox.add_joint(master_v, deck_beams[i], node_pos_top, type='pivot')
        if i > 0: sandbox.add_joint(master_v, diag_beams[i-1], node_pos_top, type='pivot')
        if i > 0: sandbox.add_joint(master_v, bottom_beams[i-1], node_pos_bottom, type='pivot')
        if i < num_panels: sandbox.add_joint(master_v, bottom_beams[i], node_pos_bottom, type='pivot')
        if i < num_panels: sandbox.add_joint(master_v, diag_beams[i], node_pos_bottom, type='pivot')
    sandbox.add_joint(v_beams[0], None, (L_X, top_y), type='pivot')
    sandbox.add_joint(v_beams[0], None, (L_X, bottom_y), type='pivot')
    sandbox.add_joint(v_beams[-1], None, (R_X, top_y), type='pivot')
    sandbox.add_joint(v_beams[-1], None, (R_X, bottom_y), type='pivot')
    ext = sandbox.add_beam(x=R_X+2.5, y=top_y, width=5.0, height=0.4, density=25.0)
    sandbox.add_joint(v_beams[-1], ext, (R_X, top_y), type='pivot')
    dist2 = math.sqrt((5.0)**2 + (top_y-bottom_y)**2)
    angle2 = math.atan2(top_y-bottom_y, 5.0)
    ext_diag = sandbox.add_beam(x=R_X+2.5, y=(top_y+bottom_y)/2, width=dist2, height=0.3, angle=angle2, density=10.5)
    sandbox.add_joint(ext, ext_diag, (R_X+5.0, top_y), type='pivot')
    sandbox.add_joint(v_beams[-1], ext_diag, (R_X, bottom_y), type='pivot')
    return deck_beams[0]

def build_agent_stage_2(sandbox):
    L_X, R_X = 10.0, 25.0
    deck_y = 10.0
    bottom_y = 7.0
    deck, bottom, v_beams = build_truss(
        sandbox, L_X, R_X, deck_y, bottom_y, 18,
        deck_density=25.0, truss_density=10.0, joint_type='rigid',
    )
    for anchor_y in (deck_y, (deck_y + bottom_y) / 2, bottom_y):
        sandbox.add_joint(v_beams[0], None, (L_X, anchor_y), type='rigid')
        sandbox.add_joint(v_beams[-1], None, (R_X, anchor_y), type='rigid')
    ext = sandbox.add_beam(x=R_X + 2.5, y=deck_y, width=5.0, height=0.4,
                           density=20.0)
    sandbox.add_joint(v_beams[-1], ext, (R_X, deck_y), type='rigid')
    return deck[0]

def build_agent_stage_3(sandbox):
    L_X, R_X = 10.0, 30.0
    num_panels = 16
    top_y = 9.7
    bottom_y = 5.2
    panel_width = (R_X - L_X) / num_panels

    deck_beams = []
    lower_beams = []
    posts = []
    braces = []
    for i in range(num_panels):
        x = L_X + (i + 0.5) * panel_width
        deck_beams.append(sandbox.add_beam(
            x=x, y=top_y, width=panel_width + 0.01, height=0.4,
            density=18.0, friction=0.0,
        ))
        lower_beams.append(sandbox.add_beam(
            x=x, y=bottom_y, width=panel_width + 0.01, height=0.3,
            density=5.0,
        ))

    midpoint_y = (top_y + bottom_y) / 2
    truss_height = top_y - bottom_y
    for i in range(num_panels + 1):
        posts.append(sandbox.add_beam(
            x=L_X + i * panel_width, y=midpoint_y,
            width=0.3, height=truss_height, density=5.0,
        ))

    brace_length = math.sqrt(panel_width ** 2 + truss_height ** 2)
    brace_slope = math.atan2(truss_height, panel_width)
    for i in range(num_panels):
        braces.append(sandbox.add_beam(
            x=L_X + (i + 0.5) * panel_width,
            y=midpoint_y,
            width=brace_length + 0.1,
            height=0.3,
            angle=brace_slope if i < num_panels // 2 else -brace_slope,
            density=5.0,
        ))

    for i, post in enumerate(posts):
        x = L_X + i * panel_width
        if i > 0:
            sandbox.add_joint(post, deck_beams[i - 1], (x, top_y), type='pivot')
            sandbox.add_joint(post, lower_beams[i - 1], (x, bottom_y), type='pivot')
            previous_brace_y = top_y if i - 1 < num_panels // 2 else bottom_y
            sandbox.add_joint(
                post, braces[i - 1], (x, previous_brace_y), type='pivot'
            )
        if i < num_panels:
            sandbox.add_joint(post, deck_beams[i], (x, top_y), type='pivot')
            sandbox.add_joint(post, lower_beams[i], (x, bottom_y), type='pivot')
            next_brace_y = bottom_y if i < num_panels // 2 else top_y
            sandbox.add_joint(post, braces[i], (x, next_brace_y), type='pivot')

    for anchor_y in (top_y, midpoint_y, bottom_y):
        sandbox.add_joint(posts[0], None, (L_X, anchor_y), type='pivot')
        sandbox.add_joint(posts[-1], None, (R_X, anchor_y), type='pivot')

    extension = sandbox.add_beam(
        x=R_X + 2.5, y=top_y, width=5.0, height=0.4,
        density=18.0, friction=0.0,
    )
    sandbox.add_joint(posts[-1], extension, (R_X, top_y), type='pivot')
    extension_brace_length = math.sqrt(5.0 ** 2 + truss_height ** 2)
    extension_brace = sandbox.add_beam(
        x=R_X + 2.5,
        y=midpoint_y,
        width=extension_brace_length,
        height=0.3,
        angle=math.atan2(truss_height, 5.0),
        density=5.0,
    )
    sandbox.add_joint(posts[-1], extension_brace, (R_X, bottom_y), type='pivot')
    sandbox.add_joint(
        extension, extension_brace, (R_X + 5.0, top_y), type='pivot'
    )
    runway_end = R_X + 5.0
    road_y = 10.0
    runway = [sandbox.add_beam(
        x=10.0, y=road_y, width=10.4, height=0.1,
        density=30.0, friction=0.0,
    )]
    spans = [(5.0, 15.0)]
    left_edge = 15.0
    while left_edge < runway_end:
        width = min(10.0, runway_end - left_edge)
        spans.append((left_edge, left_edge + width))
        runway.append(sandbox.add_beam(
            x=left_edge + width / 2.0,
            y=road_y,
            width=width + 0.4,
            height=0.1,
            density=30.0,
            friction=0.0,
        ))
        left_edge += width
    sandbox.add_joint(runway[0], None, (10.0, road_y), type='pivot')
    for index, (beam, span) in enumerate(zip(runway, spans)):
        left_x, right_x = span
        if left_x >= R_X:
            left_support = extension
        else:
            support_index = int((left_x - L_X) / panel_width)
            left_support = deck_beams[max(0, min(support_index, len(deck_beams) - 1))]
        if index > 0:
            sandbox.add_joint(runway[index - 1], beam, (left_x, road_y), type='pivot')
            sandbox.add_joint(beam, left_support, (left_x + 0.1, road_y), type='pivot')
        inner_right = min(right_x - 0.1, R_X + 2.5)
        if inner_right >= R_X:
            right_support = extension
        else:
            support_index = int((inner_right - L_X) / panel_width)
            right_support = deck_beams[max(0, min(support_index, len(deck_beams) - 1))]
        sandbox.add_joint(beam, right_support, (inner_right, road_y), type='pivot')
    sandbox.add_joint(runway[-1], None, (runway_end, road_y), type='pivot')
    return deck_beams[0]

def build_agent_stage_4(sandbox):
    L_X, R_X = 10.0, 38.0
    num_panels = 7
    top_y = 10.0
    bottom_y = 5.5
    midpoint_y = (top_y + bottom_y) / 2
    truss_height = top_y - bottom_y
    panel_width = (R_X - L_X) / num_panels

    deck_beams = []
    lower_beams = []
    posts = []
    braces = []
    for i in range(num_panels):
        x = L_X + (i + 0.5) * panel_width
        deck_beams.append(sandbox.add_beam(
            x=x, y=top_y, width=panel_width + 0.01, height=0.4,
            density=15.0,
        ))
        lower_beams.append(sandbox.add_beam(
            x=x, y=bottom_y, width=panel_width + 0.01, height=0.3,
            density=4.0,
        ))

    for i in range(num_panels + 1):
        posts.append(sandbox.add_beam(
            x=L_X + i * panel_width,
            y=midpoint_y,
            width=0.3,
            height=truss_height,
            density=4.0,
        ))

    brace_length = math.sqrt(panel_width ** 2 + truss_height ** 2)
    brace_slope = math.atan2(truss_height, panel_width)
    for i in range(num_panels):
        braces.append(sandbox.add_beam(
            x=L_X + (i + 0.5) * panel_width,
            y=midpoint_y,
            width=brace_length + 0.05,
            height=0.25,
            angle=brace_slope if i % 2 == 0 else -brace_slope,
            density=4.0,
        ))

    for i, post in enumerate(posts):
        x = L_X + i * panel_width
        if i > 0:
            sandbox.add_joint(post, deck_beams[i - 1], (x, top_y), type='rigid')
            sandbox.add_joint(post, lower_beams[i - 1], (x, bottom_y), type='rigid')
            previous_brace_y = top_y if (i - 1) % 2 == 0 else bottom_y
            sandbox.add_joint(
                post, braces[i - 1], (x, previous_brace_y), type='rigid'
            )
        if i < num_panels:
            sandbox.add_joint(post, deck_beams[i], (x, top_y), type='rigid')
            sandbox.add_joint(post, lower_beams[i], (x, bottom_y), type='rigid')
            next_brace_y = bottom_y if i % 2 == 0 else top_y
            sandbox.add_joint(post, braces[i], (x, next_brace_y), type='rigid')

    for anchor_y in (top_y, midpoint_y, bottom_y):
        sandbox.add_joint(posts[0], None, (L_X, anchor_y), type='rigid')
        sandbox.add_joint(posts[-1], None, (R_X, anchor_y), type='rigid')

    extension = sandbox.add_beam(
        x=R_X + 2.5, y=top_y, width=5.0, height=0.4,
        density=15.0,
    )
    sandbox.add_joint(posts[-1], extension, (R_X, top_y), type='rigid')
    extension_brace_length = math.sqrt(5.0 ** 2 + truss_height ** 2)
    extension_brace = sandbox.add_beam(
        x=R_X + 2.5,
        y=midpoint_y,
        width=extension_brace_length + 0.05,
        height=0.25,
        angle=math.atan2(truss_height, 5.0),
        density=4.0,
    )
    sandbox.add_joint(posts[-1], extension_brace, (R_X, bottom_y), type='rigid')
    sandbox.add_joint(
        extension, extension_brace, (R_X + 5.0, top_y), type='rigid'
    )
    return deck_beams[0]

def agent_action_stage_1(sandbox, agent_body, step_count): pass

def agent_action_stage_2(sandbox, agent_body, step_count): pass

def agent_action_stage_3(sandbox, agent_body, step_count):
    pass

def agent_action_stage_4(sandbox, agent_body, step_count):
    sandbox.apply_vehicle_force(8000.0, 20000.0)
