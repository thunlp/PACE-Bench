import math

def build_advanced_tower(sandbox, levels=25, base_w=4.0, beam_h=1.5, base_density=200.0, top_density=5.0,
                         taper_rate=0.8, density_taper=0.95, tmd_params=None):
    foundation_y = 1.0
    foundation = sandbox.get_foundation()
    base = sandbox.add_beam(x=0, y=foundation_y + beam_h/2, width=base_w, height=beam_h, density=base_density)
    if foundation:
        num_joints = max(4, int(base_w * 0.8))
        for i in range(num_joints):
            ax = -(base_w/2 - 0.1) + (i * (base_w-0.2)/(num_joints-1))
            sandbox.add_joint(foundation, base, (ax, foundation_y), type='rigid')
    prev = base
    prev_y = foundation_y + beam_h/2
    beams = [base]
    for i in range(1, levels):
        curr_y = prev_y + beam_h
        progress = i / (levels - 1)
        curr_w = max(0.4, base_w * (1.0 - progress**taper_rate))
        d = max(top_density, base_density * (1.0 - progress**density_taper))
        b = sandbox.add_beam(x=0, y=curr_y, width=curr_w, height=beam_h, density=d)
        anchor_y = curr_y - beam_h/2
        j_count = 3 if curr_w > 1.5 else 2
        for j in range(j_count):
            jx = -(curr_w/2 - 0.05) + (j * (curr_w-0.1)/(j_count-1))
            sandbox.add_joint(prev, b, (jx, anchor_y), type='rigid')
        prev = b
        prev_y = curr_y
        beams.append(b)
    if tmd_params:
        top = beams[-1]
        mass_w, mass_h = tmd_params.get('size', (1.0, 1.0))
        mass_d = tmd_params.get('density', 100.0)
        stiff = tmd_params.get('stiffness', 4.0)
        damp = tmd_params.get('damping', 0.9)
        tmd_body = sandbox.add_beam(x=0, y=prev_y + 1.5, width=mass_w, height=mass_h, density=mass_d)
        sandbox.add_spring(top, tmd_body, (0, prev_y + beam_h/2), (0, 0), stiffness=stiff, damping=damp)
    return base

def build_agent(sandbox):
    return build_advanced_tower(sandbox, levels=22, base_w=4.0)

def agent_action(sandbox, agent_body, step_count):
    pass

def build_agent_stage_1(sandbox):
    return build_advanced_tower(sandbox, levels=3, base_w=5.0, beam_h=10.0,
                               base_density=0.5, top_density=0.06,
                               taper_rate=0.25, density_taper=0.08)

def build_agent_stage_2(sandbox):
    return build_advanced_tower(
        sandbox, levels=3, base_w=8.5, beam_h=10.0,
        base_density=0.6, top_density=0.1,
        taper_rate=0.3, density_taper=0.08,
    )

def build_agent_stage_3(sandbox):
    return build_advanced_tower(
        sandbox, levels=15, base_w=5.0, beam_h=2.2,
        base_density=13.0, top_density=0.5,
        taper_rate=0.5, density_taper=0.45,
    )

def build_agent_stage_4(sandbox):
    foundation = sandbox.get_foundation()
    mast_x = (-1.55, 1.55)
    mast_width = 0.65
    tier_height = 10.0
    tiers = []

    for x in mast_x:
        column = []
        for level in range(3):
            center_y = 6.0 + level * tier_height
            beam = sandbox.add_beam(
                x=x,
                y=center_y,
                width=mast_width,
                height=tier_height,
                density=0.55 + 0.15 * (2 - level),
            )
            column.append(beam)
            if level > 0:
                lower = column[level - 1]
                sandbox.add_joint(
                    lower,
                    beam,
                    (x, 1.0 + level * tier_height),
                    type="rigid",
                )
        tiers.append(column)

    left, right = tiers
    for level in range(3):
        lower_y = 1.0 + level * tier_height
        upper_y = lower_y + tier_height
        sandbox.add_spring(
            left[level],
            right[level],
            (mast_x[0], upper_y),
            (mast_x[1], lower_y),
            stiffness=2.4,
            damping=0.95,
        )
        sandbox.add_spring(
            right[level],
            left[level],
            (mast_x[1], upper_y),
            (mast_x[0], lower_y),
            stiffness=2.4,
            damping=0.95,
        )
        for column_x, column in zip(mast_x, tiers):
            sandbox.add_spring(
                foundation,
                column[level],
                (-1.9, 1.0),
                (column_x, upper_y),
                stiffness=5.0,
                damping=1.0,
            )
            sandbox.add_spring(
                foundation,
                column[level],
                (1.9, 1.0),
                (column_x, upper_y),
                stiffness=5.0,
                damping=1.0,
            )

    return left[0]

def agent_action_stage_1(sandbox, agent_body, step_count): pass

def agent_action_stage_2(sandbox, agent_body, step_count): pass

def agent_action_stage_3(sandbox, agent_body, step_count): pass

def agent_action_stage_4(sandbox, agent_body, step_count): pass
