import math

def build_agent_stage_4(sandbox):
    chord_density = 5.0
    chord_h = 0.10
    web_density = 2.0
    web_w = 0.1
    bottom_y = 1.55
    top_y = 6.25
    FORBIDDEN_LO = 9.7
    FORBIDDEN_HI = 10.3
    xs = [5.75, 9.6, 13.5]
    for xv in xs:
        if FORBIDDEN_LO <= xv <= FORBIDDEN_HI:
            raise ValueError("Grid point {} in forbidden zone".format(xv))
    n = len(xs)
    def seg_w(i):
        if i < n - 1:
            return xs[i + 1] - xs[i]
        return 2.0

    bottom_chord = []
    for i in range(n):
        w = seg_w(i)
        bottom_chord.append(sandbox.add_beam(xs[i], bottom_y, w, chord_h, 0, chord_density))
    for i in range(n - 1):
        ax = (xs[i] + xs[i + 1]) / 2.0
        sandbox.add_joint(bottom_chord[i], bottom_chord[i + 1], (ax, bottom_y), type="rigid")

    top_chord = []
    for i in range(n):
        w = seg_w(i)
        ty = top_y - (xs[i] - xs[0]) * 0.04
        top_chord.append(sandbox.add_beam(xs[i], ty, w, chord_h, 0, chord_density))
    for i in range(n - 1):
        ax = (xs[i] + xs[i + 1]) / 2.0
        ay = (top_chord[i].position.y + top_chord[i + 1].position.y) / 2.0
        sandbox.add_joint(top_chord[i], top_chord[i + 1], (ax, ay), type="rigid")

    for i in range(n):
        by_i = bottom_chord[i].position.y
        ty_i = top_chord[i].position.y
        vy = (by_i + ty_i) / 2.0
        vh = min(4.0, ty_i - by_i - chord_h - 0.02)
        if vh > 0.06:
            vert = sandbox.add_beam(xs[i], vy, web_w, vh, 0, web_density)
            sandbox.add_joint(bottom_chord[i], vert, (xs[i], by_i + chord_h / 2.0), type="rigid")
            sandbox.add_joint(top_chord[i], vert, (xs[i], ty_i - chord_h / 2.0), type="rigid")

    for i in range(n - 1):
        cx = (xs[i] + xs[i + 1]) / 2.0
        if FORBIDDEN_LO <= cx <= FORBIDDEN_HI:
            continue
        dx = xs[i + 1] - xs[i]
        by_i = bottom_chord[i].position.y
        ty_ip1 = top_chord[i + 1].position.y
        dy_up = ty_ip1 - by_i
        d_up = min(4.0, math.sqrt(dx * dx + dy_up * dy_up) * 0.96)
        ang_up = math.atan2(dy_up, dx)
        cy_up = (by_i + ty_ip1) / 2.0
        diag_up = sandbox.add_beam(cx, cy_up, web_w, d_up, ang_up, web_density)
        sandbox.add_joint(bottom_chord[i], diag_up, (xs[i] + 0.05, by_i + 0.02), type="rigid")
        sandbox.add_joint(top_chord[i + 1], diag_up, (xs[i + 1] - 0.05, ty_ip1 - 0.02), type="rigid")
        ty_i = top_chord[i].position.y
        by_ip1 = bottom_chord[i + 1].position.y
        dy_dn = ty_i - by_ip1
        d_dn = min(4.0, math.sqrt(dx * dx + dy_dn * dy_dn) * 0.96)
        ang_dn = math.atan2(dy_dn, -dx)
        cy_dn = (ty_i + by_ip1) / 2.0
        diag_dn = sandbox.add_beam(cx, cy_dn, web_w, d_dn, math.pi - abs(ang_dn), web_density)
        sandbox.add_joint(top_chord[i], diag_dn, (xs[i] + 0.05, ty_i - 0.02), type="rigid")
        sandbox.add_joint(bottom_chord[i + 1], diag_dn, (xs[i + 1] - 0.05, by_ip1 + 0.02), type="rigid")

    last_b = bottom_chord[-1]
    last_t = top_chord[-1]
    tip_x = xs[-1] + 0.20
    tip_by = last_b.position.y
    tip_ty = last_t.position.y
    tip_mid_y = (tip_by + tip_ty) / 2.0
    tip_h = min(4.0, abs(tip_ty - tip_by) - chord_h - 0.02)
    if tip_h > 0.06:
        tip_close = sandbox.add_beam(tip_x, tip_mid_y, web_w, tip_h, 0, web_density)
        sandbox.add_joint(last_b, tip_close, (tip_x, tip_by + chord_h / 2.0), type="rigid")
        sandbox.add_joint(last_t, tip_close, (tip_x, tip_ty - chord_h / 2.0), type="rigid")

    sandbox.add_joint(bottom_chord[0], None, (5.75, 1.0), type="rigid")
    total_mass = sandbox.get_structure_mass()
    mass_limit = sandbox.get_structure_mass_limit()
    if total_mass > mass_limit:
        raise ValueError(f"Mass {total_mass} exceeds {mass_limit}")
    return bottom_chord[0]

def agent_action_stage_4(sandbox, agent_body, step_count):
    pass

def build_agent_stage_3(sandbox):
    chord_density = 2.5
    chord_h = 0.55
    web_density = 1.1
    web_w = 0.1
    diag_w = 0.1
    bottom_y = 1.55
    top_y = 6.1
    FORBIDDEN_LO = 9.7
    FORBIDDEN_HI = 10.3
    xs = [5.75, 6.5, 8.0, 10.5, 13.5]
    for xv in xs:
        if FORBIDDEN_LO <= xv <= FORBIDDEN_HI:
            raise ValueError(f"Grid point {xv} in forbidden zone")
    n = len(xs)
    def seg_w(i):
        if i < n - 1:
            return xs[i + 1] - xs[i]
        return 0.50

    bottom_chord = []
    for i in range(n):
        w = seg_w(i)
        bottom_chord.append(sandbox.add_beam(xs[i], bottom_y, w, chord_h, 0, chord_density))
    for i in range(n - 1):
        ax = (xs[i] + xs[i + 1]) / 2.0
        sandbox.add_joint(bottom_chord[i], bottom_chord[i + 1], (ax, bottom_y), type="rigid")

    top_chord = []
    for i in range(n):
        w = seg_w(i)
        ty = top_y - (xs[i] - xs[0]) * 0.03
        top_chord.append(sandbox.add_beam(xs[i], ty, w, chord_h, 0, chord_density))
    for i in range(n - 1):
        ax = (xs[i] + xs[i + 1]) / 2.0
        ay = (top_chord[i].position.y + top_chord[i + 1].position.y) / 2.0
        sandbox.add_joint(top_chord[i], top_chord[i + 1], (ax, ay), type="rigid")

    for i in range(n):
        by_i = bottom_chord[i].position.y
        ty_i = top_chord[i].position.y
        vy = (by_i + ty_i) / 2.0
        vh = ty_i - by_i - chord_h - 0.03
        if vh > 0.06:
            vert = sandbox.add_beam(xs[i], vy, web_w, vh, 0, web_density)
            sandbox.add_joint(bottom_chord[i], vert, (xs[i], by_i + chord_h / 2.0), type="rigid")
            sandbox.add_joint(top_chord[i], vert, (xs[i], ty_i - chord_h / 2.0), type="rigid")

    for i in range(n - 1):
        cx = (xs[i] + xs[i + 1]) / 2.0
        if FORBIDDEN_LO <= cx <= FORBIDDEN_HI:
            continue
        dx = xs[i + 1] - xs[i]
        by_i = bottom_chord[i].position.y
        ty_ip1 = top_chord[i + 1].position.y
        dy_up = ty_ip1 - by_i
        d_up = min(4.0, math.sqrt(dx * dx + dy_up * dy_up) * 0.96)
        ang_up = math.atan2(dy_up, dx)
        cy_up = (by_i + ty_ip1) / 2.0
        diag_up = sandbox.add_beam(cx, cy_up, diag_w, d_up, ang_up, web_density)
        sandbox.add_joint(bottom_chord[i], diag_up, (xs[i] + 0.06, by_i + 0.02), type="rigid")
        sandbox.add_joint(top_chord[i + 1], diag_up, (xs[i + 1] - 0.06, ty_ip1 - 0.02), type="rigid")
        ty_i = top_chord[i].position.y
        by_ip1 = bottom_chord[i + 1].position.y
        dy_dn = ty_i - by_ip1
        d_dn = min(4.0, math.sqrt(dx * dx + dy_dn * dy_dn) * 0.96)
        ang_dn = math.atan2(dy_dn, -dx)
        cy_dn = (ty_i + by_ip1) / 2.0
        diag_dn = sandbox.add_beam(cx, cy_dn, diag_w, d_dn, math.pi - abs(ang_dn), web_density)
        sandbox.add_joint(top_chord[i], diag_dn, (xs[i] + 0.06, ty_i - 0.02), type="rigid")
        sandbox.add_joint(bottom_chord[i + 1], diag_dn, (xs[i + 1] - 0.06, by_ip1 + 0.02), type="rigid")

    last_b = bottom_chord[-1]
    last_t = top_chord[-1]
    tip_x = xs[-1] + 0.25
    tip_by = last_b.position.y
    tip_ty = last_t.position.y
    tip_mid_y = (tip_by + tip_ty) / 2.0
    tip_h = abs(tip_ty - tip_by) - chord_h - 0.03
    if tip_h > 0.06:
        tip_close = sandbox.add_beam(tip_x, tip_mid_y, web_w, tip_h, 0, web_density)
        sandbox.add_joint(last_b, tip_close, (tip_x, tip_by + chord_h / 2.0), type="rigid")
        sandbox.add_joint(last_t, tip_close, (tip_x, tip_ty - chord_h / 2.0), type="rigid")

    ground_y = 1.0
    sandbox.add_joint(bottom_chord[0], None, (5.75, ground_y), type="rigid")
    total_mass = sandbox.get_structure_mass()
    mass_limit = sandbox.get_structure_mass_limit()
    if total_mass > mass_limit:
        raise ValueError(f"Mass {total_mass} exceeds {mass_limit}")
    return bottom_chord[0]

def agent_action_stage_3(sandbox, agent_body, step_count):
    pass

def build_agent_stage_2(sandbox):
    component_area = 3.0 * 4.0 * 4.0
    density = 0.95 * sandbox.get_structure_mass_limit() / component_area

    root_slab = sandbox.add_beam(6.5, 3.0, 4.0, 4.0, 0, density)
    middle_slab = sandbox.add_beam(9.5, 3.0, 4.0, 4.0, 0, density)
    tip_slab = sandbox.add_beam(13.0, 5.0, 4.0, 4.0, 0, density)

    sandbox.add_joint(root_slab, middle_slab, (8.0, 3.0), type="rigid")
    sandbox.add_joint(middle_slab, tip_slab, (11.25, 4.0), type="rigid")
    sandbox.add_joint(root_slab, None, (6.5, 1.0), type="rigid")

    if sandbox.get_structure_mass() > sandbox.get_structure_mass_limit():
        raise ValueError("Structure exceeds the available mass budget")
    return root_slab

def agent_action_stage_2(sandbox, agent_body, step_count):
    pass

def build_agent_stage_1(sandbox):
    component_area = 1.2 * 3.2 + 2.0 * 1.0 + 4.0 * 4.0 + 4.0 * 4.0
    density = 0.95 * sandbox.get_structure_mass_limit() / component_area
    mast = sandbox.add_beam(5.75, 3.2, 1.2, 3.2, 0, density)
    root_plate = sandbox.add_beam(6.5, 5.0, 2.0, 1.0, 0, density)
    middle_plate = sandbox.add_beam(9.5, 5.0, 4.0, 4.0, 0, density)
    tip_plate = sandbox.add_beam(13.0, 5.0, 4.0, 4.0, 0, density)

    sandbox.add_joint(mast, root_plate, (5.75, 4.7), type="rigid")
    sandbox.add_joint(root_plate, middle_plate, (7.5, 5.0), type="rigid")
    sandbox.add_joint(middle_plate, tip_plate, (11.25, 5.0), type="rigid")
    sandbox.add_joint(mast, None, (5.75, 1.0), type="rigid")

    if sandbox.get_structure_mass() > sandbox.get_structure_mass_limit():
        raise ValueError("Structure exceeds the available mass budget")
    return mast

def agent_action_stage_1(sandbox, agent_body, step_count):
    pass

def build_agent(sandbox):
    x_min, x_max, _, _ = sandbox.get_build_zone()
    support_lo, support_hi = 5.0, 6.5
    bw, bh = 0.35, 0.25
    density = 1.2
    bottom_y = 2.0
    top_y = 5.8
    xs = [5.75, 6.5, 8.0, 9.0, 10.35, 12.0, 14.0, 15.0]
    xs = [min(x_max - bw / 2, max(x_min + bw / 2, x)) for x in xs]
    def seg(i):
        return (xs[i + 1] - xs[i]) if i < len(xs) - 1 else 0.8
    bottom_beams = [sandbox.add_beam(xs[i], bottom_y, seg(i), bh, 0, density) for i in range(len(xs))]
    for i in range(len(bottom_beams) - 1):
        ax = (xs[i] + xs[i + 1]) / 2
        sandbox.add_joint(bottom_beams[i], bottom_beams[i + 1], (ax, bottom_y), type="rigid")
    top_beams = [sandbox.add_beam(xs[i], top_y, seg(i), bh, 0, density) for i in range(len(xs))]
    for i in range(len(top_beams) - 1):
        ax = (xs[i] + xs[i + 1]) / 2
        sandbox.add_joint(top_beams[i], top_beams[i + 1], (ax, top_y), type="rigid")
    vy = (bottom_y + top_y) / 2
    vh = top_y - bottom_y - bh - 0.03
    for i in range(len(xs)):
        vert = sandbox.add_beam(xs[i], vy, 0.25, vh, 0, density)
        sandbox.add_joint(bottom_beams[i], vert, (xs[i], bottom_y + bh / 2), type="rigid")
        sandbox.add_joint(top_beams[i], vert, (xs[i], top_y - bh / 2), type="rigid")
    for i in range(len(xs) - 1):
        dx = xs[i + 1] - xs[i]
        d = math.sqrt(dx**2 + (top_y - bottom_y)**2) * 0.92
        ang = math.atan2(top_y - bottom_y, dx)
        diag1 = sandbox.add_beam((xs[i] + xs[i + 1]) / 2, vy, 0.2, d, ang, density)
        sandbox.add_joint(bottom_beams[i], diag1, (xs[i] + 0.06, bottom_y + 0.02), type="rigid")
        sandbox.add_joint(top_beams[i + 1], diag1, (xs[i + 1] - 0.06, top_y - 0.02), type="rigid")
        diag2 = sandbox.add_beam((xs[i] + xs[i + 1]) / 2, vy, 0.2, d, -ang, density)
        sandbox.add_joint(bottom_beams[i + 1], diag2, (xs[i + 1] - 0.06, bottom_y + 0.02), type="rigid")
        sandbox.add_joint(top_beams[i], diag2, (xs[i] + 0.06, top_y - 0.02), type="rigid")
    ground_y = 1.0
    sandbox.add_joint(bottom_beams[0], None, (5.75, ground_y), type="rigid")
    return bottom_beams[0]

def agent_action(sandbox, agent_body, step_count):
    pass
