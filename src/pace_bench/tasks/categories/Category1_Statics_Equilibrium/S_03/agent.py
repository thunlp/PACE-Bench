import math

def build_agent(sandbox):
    target_reach = 17.0
    structure_y = 10.0
    WALL_X = 0.0
    num_segments = 8
    seg_len = target_reach / num_segments
    top_chord = []
    bot_chord = []
    angle = 0.05
    anchor_sep = 1.2
    anchor_top_y = structure_y + anchor_sep/2
    anchor_bot_y = structure_y - anchor_sep/2
    for i in range(num_segments):
        x = WALL_X + (i + 0.5) * seg_len
        ty = anchor_top_y + i * seg_len * math.sin(angle)
        by = anchor_bot_y + i * seg_len * math.sin(angle)
        tb = sandbox.add_beam(x=x, y=ty, width=seg_len + 0.15, height=0.55, angle=angle, density=35.0)
        bb = sandbox.add_beam(x=x, y=by, width=seg_len + 0.15, height=0.55, angle=angle, density=35.0)
        top_chord.append(tb)
        bot_chord.append(bb)
        if i > 0:
            sandbox.add_joint(top_chord[i-1], tb, (WALL_X + i * seg_len, ty), type='rigid')
            sandbox.add_joint(bot_chord[i-1], bb, (WALL_X + i * seg_len, by), type='rigid')
    sandbox.add_joint(top_chord[0], None, (WALL_X, anchor_top_y), type='rigid')
    sandbox.add_joint(bot_chord[0], None, (WALL_X, anchor_bot_y), type='rigid')
    for i in range(num_segments):
        x = WALL_X + i * seg_len
        ty = anchor_top_y + i * seg_len * math.sin(angle)
        by = anchor_bot_y + i * seg_len * math.sin(angle)
        next_ty = anchor_top_y + (i+1) * seg_len * math.sin(angle)
        next_by = anchor_bot_y + (i+1) * seg_len * math.sin(angle)
        v = sandbox.add_beam(x=x + seg_len, y=(next_ty + next_by)/2, width=0.25, height=next_ty - next_by, density=25.0)
        sandbox.add_joint(top_chord[i], v, (x + seg_len, next_ty), type='rigid')
        sandbox.add_joint(bot_chord[i], v, (x + seg_len, next_by), type='rigid')
        d = sandbox.add_beam(x=x + seg_len/2, y=(ty + next_by)/2, width=math.sqrt(seg_len**2 + (ty-next_by)**2), height=0.25, angle=-math.atan2(ty-next_by, seg_len), density=25.0)
        sandbox.add_joint(top_chord[i], d, (x, ty), type='rigid')
        sandbox.add_joint(bot_chord[i], d, (x + seg_len, next_by), type='rigid')
    return top_chord[0]

def agent_action(sandbox, agent_body, step_count):
    pass

def build_agent_stage_1(sandbox):
    target_reach = 13.0
    WALL_X = 0.0
    anchor_top_y = 28.0
    anchor_bot_y = -10.0
    mid_y = (anchor_top_y + anchor_bot_y) / 2.0
    full_top_mid = anchor_top_y - mid_y
    full_mid_bot = mid_y - anchor_bot_y
    num_segments = 12
    seg_len = target_reach / num_segments
    top_chord = []
    mid_chord = []
    bot_chord = []
    chord_density = 0.4
    web_density = 0.25
    for i in range(num_segments):
        x = WALL_X + (i + 0.5) * seg_len
        progress = i / max(num_segments - 1, 1)
        taper = 1.0 - progress * 0.55
        top_mid_d = full_top_mid * taper
        mid_bot_d = full_mid_bot * taper
        ty = mid_y + top_mid_d
        my = mid_y
        by = mid_y - mid_bot_d
        tb = sandbox.add_beam(x=x, y=ty, width=seg_len + 0.35, height=0.55, density=chord_density)
        mb = sandbox.add_beam(x=x, y=my, width=seg_len + 0.35, height=0.55, density=chord_density)
        bb = sandbox.add_beam(x=x, y=by, width=seg_len + 0.35, height=0.55, density=chord_density)
        top_chord.append(tb)
        mid_chord.append(mb)
        bot_chord.append(bb)
        if i > 0:
            sandbox.add_joint(top_chord[i - 1], tb, (WALL_X + i * seg_len, ty), type='rigid')
            sandbox.add_joint(mid_chord[i - 1], mb, (WALL_X + i * seg_len, my), type='rigid')
            sandbox.add_joint(bot_chord[i - 1], bb, (WALL_X + i * seg_len, by), type='rigid')
    first_x = WALL_X + 0.5 * seg_len
    sandbox.add_joint(top_chord[0], None, (first_x, anchor_top_y), type='rigid')
    sandbox.add_joint(bot_chord[0], None, (first_x, anchor_bot_y), type='rigid')
    for i in range(num_segments):
        x = WALL_X + i * seg_len
        progress_i = i / max(num_segments - 1, 1)
        progress_next = (i + 1) / max(num_segments - 1, 1)
        taper_i = 1.0 - progress_i * 0.55
        taper_next = 1.0 - progress_next * 0.55
        top_mid_i = full_top_mid * taper_i
        mid_bot_i = full_mid_bot * taper_i
        top_mid_n = full_top_mid * taper_next
        mid_bot_n = full_mid_bot * taper_next
        ty = mid_y + top_mid_i
        my = mid_y
        by = mid_y - mid_bot_i
        next_ty = mid_y + top_mid_n
        next_my = mid_y
        next_by = mid_y - mid_bot_n
        vh_upper = abs(next_ty - next_my)
        if vh_upper > 0.15:
            vu = sandbox.add_beam(x=x + seg_len, y=(next_ty + next_my) / 2.0,
                                  width=0.18, height=vh_upper, density=web_density)
            sandbox.add_joint(top_chord[i], vu, (x + seg_len, next_ty), type='rigid')
            sandbox.add_joint(mid_chord[i], vu, (x + seg_len, next_my), type='rigid')
        vh_lower = abs(next_my - next_by)
        if vh_lower > 0.15:
            vl = sandbox.add_beam(x=x + seg_len, y=(next_my + next_by) / 2.0,
                                  width=0.18, height=vh_lower, density=web_density)
            sandbox.add_joint(mid_chord[i], vl, (x + seg_len, next_my), type='rigid')
            sandbox.add_joint(bot_chord[i], vl, (x + seg_len, next_by), type='rigid')
        d1_len = math.sqrt(seg_len ** 2 + (ty - next_my) ** 2)
        if d1_len > 0.3:
            d1 = sandbox.add_beam(x=x + seg_len / 2.0, y=(ty + next_my) / 2.0,
                                  width=d1_len, height=0.18,
                                  angle=-math.atan2(ty - next_my, seg_len), density=web_density)
            sandbox.add_joint(top_chord[i], d1, (x, ty), type='rigid')
            sandbox.add_joint(mid_chord[i], d1, (x + seg_len, next_my), type='rigid')
        d2_len = math.sqrt(seg_len ** 2 + (next_ty - my) ** 2)
        if d2_len > 0.3:
            d2 = sandbox.add_beam(x=x + seg_len / 2.0, y=(my + next_ty) / 2.0,
                                  width=d2_len, height=0.18,
                                  angle=math.atan2(next_ty - my, seg_len), density=web_density)
            sandbox.add_joint(mid_chord[i], d2, (x, my), type='rigid')
            sandbox.add_joint(top_chord[i], d2, (x + seg_len, next_ty), type='rigid')
        d3_len = math.sqrt(seg_len ** 2 + (my - next_by) ** 2)
        if d3_len > 0.3:
            d3 = sandbox.add_beam(x=x + seg_len / 2.0, y=(my + next_by) / 2.0,
                                  width=d3_len, height=0.18,
                                  angle=-math.atan2(my - next_by, seg_len), density=web_density)
            sandbox.add_joint(mid_chord[i], d3, (x, my), type='rigid')
            sandbox.add_joint(bot_chord[i], d3, (x + seg_len, next_by), type='rigid')
        d4_len = math.sqrt(seg_len ** 2 + (next_my - by) ** 2)
        if d4_len > 0.3:
            d4 = sandbox.add_beam(x=x + seg_len / 2.0, y=(by + next_my) / 2.0,
                                  width=d4_len, height=0.18,
                                  angle=math.atan2(next_my - by, seg_len), density=web_density)
            sandbox.add_joint(bot_chord[i], d4, (x, by), type='rigid')
            sandbox.add_joint(mid_chord[i], d4, (x + seg_len, next_my), type='rigid')
    return top_chord[0]

def agent_action_stage_1(sandbox, agent_body, step_count):
    pass

def build_agent_stage_2(sandbox):
    monolith = sandbox.add_beam(
        x=6.1,
        y=10.0,
        width=12.2,
        height=15.0,
        density=0.02,
    )
    sandbox.add_joint(monolith, None, (0.0, 17.5), type='rigid')
    sandbox.add_joint(monolith, None, (0.0, 2.5), type='rigid')
    return monolith

def agent_action_stage_2(sandbox, agent_body, step_count):
    pass

def build_agent_stage_3(sandbox):
    target_reach = 36.0
    WALL_X = 0.0
    anchor_top_y = -6.1
    anchor_bot_y = -14.5
    mid_y = (anchor_top_y + anchor_bot_y) / 2.0
    depth = anchor_top_y - anchor_bot_y
    rise = 6.0
    num_segments = 10
    seg_len = target_reach / num_segments
    angle = math.atan2(rise, target_reach)
    top_chord = []
    bot_chord = []
    for i in range(num_segments):
        x = WALL_X + (i + 0.5) * seg_len
        centre_y = mid_y + (i + 0.5) * seg_len * math.sin(angle)
        progress = i / num_segments
        taper = 1.0 - progress * 0.45
        cur_depth = depth * taper
        cur_top = centre_y + cur_depth / 2.0
        cur_bot = centre_y - cur_depth / 2.0
        tb = sandbox.add_beam(x=x, y=cur_top, width=seg_len + 0.45, height=0.9, density=48.0)
        bb = sandbox.add_beam(x=x, y=cur_bot, width=seg_len + 0.45, height=0.9, density=48.0)
        top_chord.append(tb)
        bot_chord.append(bb)
        if i > 0:
            sandbox.add_joint(top_chord[i - 1], tb, (WALL_X + i * seg_len, cur_top), type='rigid')
            sandbox.add_joint(bot_chord[i - 1], bb, (WALL_X + i * seg_len, cur_bot), type='rigid')
    sandbox.add_joint(top_chord[0], None, (WALL_X, anchor_top_y), type='rigid')
    sandbox.add_joint(bot_chord[0], None, (WALL_X, anchor_bot_y), type='rigid')
    for i in range(num_segments):
        x = WALL_X + i * seg_len
        progress_i = i / num_segments
        progress_next = (i + 1) / num_segments
        taper_i = 1.0 - progress_i * 0.45
        taper_next = 1.0 - progress_next * 0.45
        cur_depth_i = depth * taper_i
        cur_depth_n = depth * taper_next
        centre_i = mid_y + i * seg_len * math.sin(angle)
        centre_n = mid_y + (i + 1) * seg_len * math.sin(angle)
        cur_top = centre_i + cur_depth_i / 2.0
        cur_bot = centre_i - cur_depth_i / 2.0
        next_top = centre_n + cur_depth_n / 2.0
        next_bot = centre_n - cur_depth_n / 2.0
        v = sandbox.add_beam(x=x + seg_len, y=(next_top + next_bot) / 2.0,
                              width=0.35, height=cur_depth_n, density=26.0)
        sandbox.add_joint(top_chord[i], v, (x + seg_len, next_top), type='rigid')
        sandbox.add_joint(bot_chord[i], v, (x + seg_len, next_bot), type='rigid')
        if i % 2 == 0:
            dlen = math.sqrt(seg_len ** 2 + (cur_top - next_bot) ** 2)
            d = sandbox.add_beam(x=x + seg_len / 2.0, y=(cur_top + next_bot) / 2.0,
                                  width=dlen, height=0.28,
                                  angle=-math.atan2(cur_top - next_bot, seg_len), density=26.0)
            sandbox.add_joint(top_chord[i], d, (x, cur_top), type='rigid')
            sandbox.add_joint(bot_chord[i], d, (x + seg_len, next_bot), type='rigid')
        else:
            dlen = math.sqrt(seg_len ** 2 + (next_top - cur_bot) ** 2)
            d = sandbox.add_beam(x=x + seg_len / 2.0, y=(cur_bot + next_top) / 2.0,
                                  width=dlen, height=0.28,
                                  angle=math.atan2(next_top - cur_bot, seg_len), density=26.0)
            sandbox.add_joint(bot_chord[i], d, (x, cur_bot), type='rigid')
            sandbox.add_joint(top_chord[i], d, (x + seg_len, next_top), type='rigid')
    return top_chord[0]

def agent_action_stage_3(sandbox, agent_body, step_count):
    pass

def build_agent_stage_4(sandbox):
    target_reach = 41.0
    WALL_X = 0.0
    anchor_top_y = -3.05
    anchor_bot_y = -19.5
    root_center_y = (anchor_top_y + anchor_bot_y) / 2.0
    root_depth = anchor_top_y - anchor_bot_y
    tip_center_y = 8.0
    rise = tip_center_y - root_center_y
    truss_angle = math.atan2(rise, target_reach)
    tip_depth = 4.5
    num_segments = 24
    seg_len = target_reach / num_segments
    CHORD_DENSITY = 26.0
    CHORD_HEIGHT = 0.95
    WEB_DENSITY = 7.5
    top_chord = []
    bot_chord = []
    for i in range(num_segments):
        x = WALL_X + (i + 0.5) * seg_len
        progress = i / max(num_segments - 1, 1)
        cur_center_y = root_center_y + progress * target_reach * math.sin(truss_angle)
        cur_depth = root_depth - progress * (root_depth - tip_depth)
        ty = cur_center_y + cur_depth / 2.0
        by = cur_center_y - cur_depth / 2.0
        tb = sandbox.add_beam(x=x, y=ty, width=seg_len + 0.25, height=CHORD_HEIGHT, density=CHORD_DENSITY)
        bb = sandbox.add_beam(x=x, y=by, width=seg_len + 0.25, height=CHORD_HEIGHT, density=CHORD_DENSITY)
        top_chord.append(tb)
        bot_chord.append(bb)
        if i > 0:
            sandbox.add_joint(top_chord[i - 1], tb, (WALL_X + i * seg_len, ty), type='rigid')
            sandbox.add_joint(bot_chord[i - 1], bb, (WALL_X + i * seg_len, by), type='rigid')
    sandbox.add_joint(top_chord[0], None, (WALL_X, anchor_top_y), type='rigid')
    sandbox.add_joint(bot_chord[0], None, (WALL_X, anchor_bot_y), type='rigid')
    for i in range(num_segments):
        x = WALL_X + i * seg_len
        progress_i = i / max(num_segments - 1, 1)
        progress_next = (i + 1) / max(num_segments - 1, 1)
        cur_depth_i = root_depth - progress_i * (root_depth - tip_depth)
        cur_depth_n = root_depth - progress_next * (root_depth - tip_depth)
        centre_i = root_center_y + progress_i * target_reach * math.sin(truss_angle)
        centre_n = root_center_y + progress_next * target_reach * math.sin(truss_angle)
        cur_top = centre_i + cur_depth_i / 2.0
        cur_bot = centre_i - cur_depth_i / 2.0
        next_top = centre_n + cur_depth_n / 2.0
        next_bot = centre_n - cur_depth_n / 2.0
        dlen = math.sqrt(seg_len ** 2 + (cur_top - next_bot) ** 2)
        d_angle = -math.atan2(cur_top - next_bot, seg_len)
        d = sandbox.add_beam(x=x + seg_len / 2.0, y=(cur_top + next_bot) / 2.0,
                              width=dlen, height=0.18, angle=d_angle, density=WEB_DENSITY)
        sandbox.add_joint(top_chord[i], d, (x, cur_top), type='rigid')
        sandbox.add_joint(bot_chord[i], d, (x + seg_len, next_bot), type='rigid')
        dlen2 = math.sqrt(seg_len ** 2 + (next_top - cur_bot) ** 2)
        d_angle2 = math.atan2(next_top - cur_bot, seg_len)
        d2 = sandbox.add_beam(x=x + seg_len / 2.0, y=(cur_bot + next_top) / 2.0,
                               width=dlen2, height=0.18, angle=d_angle2, density=WEB_DENSITY)
        sandbox.add_joint(bot_chord[i], d2, (x, cur_bot), type='rigid')
        sandbox.add_joint(top_chord[i], d2, (x + seg_len, next_top), type='rigid')
    return top_chord[0]

def agent_action_stage_4(sandbox, agent_body, step_count):
    pass
