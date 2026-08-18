"""Reference designs for F-05.

Every reference is passive.  Construction uses only the documented sandbox
primitives, and ``agent_action`` never mutates the hull, cargo, terrain, or
physics configuration.
"""

BOAT_LEFT_X = 13.5
BOAT_RIGHT_X = 16.5
RAIL_HEIGHT = 0.9
RAIL_WIDTH = 0.2


def _hull_deck_top(sandbox):
    position = sandbox.get_boat_position()
    if position is None:
        hull_y = 2.5
    else:
        hull_y = float(position[1])
    return hull_y + 0.2, hull_y


def _check_mass(sandbox):
    total_mass = sandbox.get_structure_mass()
    if total_mass > sandbox.MAX_STRUCTURE_MASS:
        raise ValueError(
            f"Structure mass {total_mass:.2f} kg exceeds limit "
            f"{sandbox.MAX_STRUCTURE_MASS:.2f} kg"
        )


def build_agent(sandbox):
    """Baseline: low ballast plus deck-edge rails and local cargo stops."""

    deck_top, hull_y = _hull_deck_top(sandbox)
    build_y_min = 2.0
    max_mass = float(sandbox.MAX_STRUCTURE_MASS)
    reference_mass = 59.61
    density_scale = (
        1.0
        if max_mass >= reference_mass - 0.05
        else min(1.0, max(0.35, (max_mass - 0.25) / reference_mass))
    )

    def density(value):
        return float(value) * density_scale

    anchor_y = build_y_min + 0.01
    ballast_y = max(hull_y - 0.26, build_y_min + 0.086)
    ballast_anchor_y = max(deck_top - 0.26, anchor_y)
    rail_anchor_y = max(deck_top - 0.05, anchor_y)
    bodies = []

    for x in (14.25, 15.75):
        beam = sandbox.add_beam(
            x, ballast_y, 0.5, 0.17, density=density(254.0)
        )
        sandbox.set_material_properties(beam, restitution=0.05)
        sandbox.add_joint(beam, None, (x, ballast_anchor_y), type="rigid")
        bodies.append(beam)

    for x in (BOAT_LEFT_X, BOAT_RIGHT_X):
        rail = sandbox.add_beam(
            x,
            deck_top + RAIL_HEIGHT / 2.0,
            RAIL_WIDTH,
            RAIL_HEIGHT,
            density=density(30.0),
        )
        sandbox.set_material_properties(rail, restitution=0.07)
        sandbox.add_joint(rail, None, (x, rail_anchor_y), type="rigid")
        bodies.append(rail)

    for x in (14.5, 15.5):
        lip = sandbox.add_beam(
            x, deck_top + 0.06, 0.18, 0.06, density=density(35.0)
        )
        sandbox.set_material_properties(lip, restitution=0.07)
        sandbox.add_joint(lip, None, (x, deck_top), type="rigid")
        bodies.append(lip)

    for x in (14.5, 15.5):
        stop = sandbox.add_beam(
            x, deck_top + 0.18, 0.26, 0.2, density=density(42.0)
        )
        sandbox.set_material_properties(stop, restitution=0.07)
        sandbox.add_joint(stop, None, (x, deck_top), type="rigid")
        bodies.append(stop)

    _check_mass(sandbox)
    return bodies[0]


def agent_action(sandbox, agent_body, step_count):
    pass


def _build_low_ballast_cage(sandbox, ballast_density):
    """Closed hold with low, far-out ballast for roll inertia."""

    bodies = []
    specifications = []
    for x in (12.5, 17.5):
        for y in (2.06, 2.18, 2.30):
            specifications.append((x, y, 1.0, 0.1, ballast_density))
    for x in (13.8, 14.6, 15.4, 16.2):
        specifications.append((x, 2.85, 0.9, 0.12, 5.0))
    specifications.extend(
        (
            (13.55, 3.28, 0.2, 0.98, 6.0),
            (16.45, 3.28, 0.2, 0.98, 6.0),
        )
    )
    for x in (13.8, 14.6, 15.4, 16.2):
        specifications.append((x, 3.71, 0.9, 0.12, 3.0))

    for x, y, width, height, density in specifications:
        beam = sandbox.add_beam(x, y, width, height, density=density)
        sandbox.set_material_properties(beam, restitution=0.01)
        sandbox.add_joint(beam, None, (x, y), type="rigid")
        bodies.append(beam)

    _check_mass(sandbox)
    return bodies[0]


def build_agent_stage_1(sandbox):
    return _build_low_ballast_cage(sandbox, ballast_density=78.0)


def agent_action_stage_1(sandbox, agent_body, step_count):
    pass


def build_agent_stage_2(sandbox):
    return _build_low_ballast_cage(sandbox, ballast_density=84.0)


def agent_action_stage_2(sandbox, agent_body, step_count):
    pass


def _stage_3_anchor(sandbox):
    deck_top, hull_y = _hull_deck_top(sandbox)
    build_y_min = 2.58
    anchor_y = max(build_y_min + 0.008, deck_top - 0.035)
    return min(anchor_y, deck_top - 0.012), deck_top, hull_y


def build_agent_stage_3(sandbox):
    bodies = []

    def cage_beam(body):
        sandbox.set_material_properties(body, restitution=0.04)
        return body

    anchor_y, deck_top, _hull_y = _stage_3_anchor(sandbox)
    build_y_min = 2.58
    pile_top = deck_top + 0.55 + 0.15 + 0.08
    grill_y = max(pile_top, build_y_min + 0.22)

    for center_x, anchors_x in (
        (12.55, (12.08, 12.28, 12.48, 12.68, 12.88, 13.02)),
        (17.45, (16.98, 17.18, 17.38, 17.58, 17.78, 17.92)),
    ):
        wing = sandbox.add_beam(
            center_x, deck_top + 0.05, 1.0, 0.1, density=34.0
        )
        bodies.append(wing)
        for anchor_x in anchors_x:
            sandbox.add_joint(
                wing, None, (anchor_x, anchor_y), type="rigid"
            )

    low_beam_y = max(deck_top + 0.11, build_y_min + 0.06)
    for x in (13.85, 14.45, 15.0, 15.55, 16.15):
        beam = sandbox.add_beam(x, low_beam_y, 0.4, 0.1, density=72.0)
        bodies.append(beam)
        for offset_x in (-0.11, 0.0, 0.11):
            sandbox.add_joint(
                beam, None, (x + offset_x, anchor_y), type="rigid"
            )

    for x in (12.38, 17.62):
        arm = sandbox.add_beam(x, deck_top + 0.1, 0.42, 0.1, density=44.0)
        bodies.append(arm)
        for offset_x in (-0.11, 0.0, 0.11):
            sandbox.add_joint(
                arm, None, (x + offset_x, anchor_y), type="rigid"
            )

    rail_height = 0.46
    for x in (13.46, 16.54):
        rail = cage_beam(
            sandbox.add_beam(
                x,
                deck_top + rail_height / 2.0,
                0.11,
                rail_height,
                density=8.5,
            )
        )
        bodies.append(rail)
        for offset_y in (0.0, 0.14, 0.28, 0.42):
            sandbox.add_joint(
                rail, None, (x, anchor_y + offset_y), type="rigid"
            )

    ceiling_y = max(
        deck_top + rail_height + 0.12,
        build_y_min + 0.52,
        grill_y + 0.65,
    )
    for center_x in (13.55, 14.48, 15.52, 16.45):
        segment = cage_beam(
            sandbox.add_beam(center_x, ceiling_y, 1.0, 0.09, density=5.4)
        )
        bodies.append(segment)
        for anchor_x in (center_x - 0.32, center_x, center_x + 0.32):
            sandbox.add_joint(
                segment,
                None,
                (min(max(anchor_x, 12.05), 17.95), ceiling_y),
                type="rigid",
            )

    gate_y = (deck_top + 0.12 + ceiling_y) / 2.0
    for x in (13.505, 13.58, 16.42, 16.495):
        gate = cage_beam(
            sandbox.add_beam(x, gate_y, 0.1, 0.98, density=10.0)
        )
        bodies.append(gate)
        for offset_y in (0.12, 0.42, 0.72):
            sandbox.add_joint(
                gate, None, (x, anchor_y + offset_y), type="rigid"
            )

    for x in (13.52, 13.80, 14.08, 14.36, 14.64, 14.92,
              15.20, 15.48, 15.76, 16.04, 16.32, 16.60):
        slat = cage_beam(
            sandbox.add_beam(x, grill_y, 0.1, 0.26, density=30.0)
        )
        bodies.append(slat)
        sandbox.add_joint(slat, None, (x, anchor_y + 0.04), type="rigid")

    for x in (13.7, 14.35, 15.0, 15.65, 16.3):
        bar = cage_beam(
            sandbox.add_beam(x, grill_y + 0.12, 0.1, 0.28, density=26.0)
        )
        bodies.append(bar)
        sandbox.add_joint(bar, None, (x, anchor_y + 0.08), type="rigid")
        sandbox.add_joint(bar, None, (x, anchor_y + 0.2), type="rigid")

    _check_mass(sandbox)
    return bodies[0]


def agent_action_stage_3(sandbox, agent_body, step_count):
    pass


def build_agent_stage_4(sandbox):
    """Tight, overlapping light cage with the remaining mass at the beam."""

    bodies = []
    specifications = [
        (12.5, 2.64, 1.0, 0.1, 60.0),
        (17.5, 2.64, 1.0, 0.1, 60.0),
    ]
    for x in (13.8, 14.6, 15.4, 16.2):
        specifications.append((x, 2.65, 0.9, 0.12, 4.0))
    specifications.extend(
        (
            (13.55, 3.0, 0.2, 0.82, 5.0),
            (16.45, 3.0, 0.2, 0.82, 5.0),
        )
    )
    for x in (13.8, 14.6, 15.4, 16.2):
        specifications.append((x, 3.38, 0.9, 0.12, 2.5))

    for x, y, width, height, density in specifications:
        beam = sandbox.add_beam(x, y, width, height, density=density)
        sandbox.set_material_properties(beam, restitution=0.01)
        sandbox.add_joint(beam, None, (x, y), type="rigid")
        bodies.append(beam)

    _check_mass(sandbox)
    return bodies[0]


def agent_action_stage_4(sandbox, agent_body, step_count):
    pass
