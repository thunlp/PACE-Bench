import math

def build_agent(sandbox):
    main_beam = sandbox.add_beam(x=0.0, y=5.0, width=7.0, height=0.2, density=10.0)
    sandbox.add_joint(main_beam, None, (0.0, 5.0), type="rigid")
    platform = sandbox.add_beam(x=3.0, y=5.5, width=1.0, height=0.2, density=10.0)
    sandbox.add_joint(main_beam, platform, (3.0, 5.5), type="rigid")
    cw = sandbox.add_beam(x=-3.0, y=5.5, width=1.0, height=1.0, density=202.0)
    sandbox.add_joint(main_beam, cw, (-3.0, 5.5), type="rigid")
    return main_beam

def agent_action(sandbox, agent_body, step_count):
    pass

def build_agent_stage_1(sandbox):
    load_deck = sandbox.add_beam(
        x=3.0,
        y=5.5,
        width=6.0,
        height=1.0,
        density=1.0,
        label="load_deck",
    )
    sandbox.add_joint(load_deck, None, (0.0, 5.0), type="pivot")

    supported_mass = 200.0 + (6.0 * 1.0 * 1.0)
    keel_mass = supported_mass * 0.5
    keel = sandbox.add_beam(
        x=0.0,
        y=4.0,
        width=0.5,
        height=2.0,
        density=keel_mass,
        label="vertical_ballast",
    )
    sandbox.add_joint(load_deck, keel, (0.0, 5.0), type="rigid")
    return load_deck

def build_agent_stage_2(sandbox):
    load_deck = sandbox.add_beam(
        x=3.0,
        y=5.5,
        width=6.0,
        height=0.2,
        density=50.0,
        label="load_deck",
    )
    sandbox.add_joint(load_deck, None, (0.0, 5.0), type="pivot")

    supported_mass = 200.0 + (6.0 * 0.2 * 50.0)
    torque_ballast = sandbox.add_beam(
        x=-4.0,
        y=6.0,
        width=1.0,
        height=1.0,
        density=supported_mass * 0.5,
        label="torque_ballast",
    )
    sandbox.add_joint(load_deck, torque_ballast, (0.0, 5.0), type="rigid")

    stability_keel = sandbox.add_beam(
        x=-3.0,
        y=2.0,
        width=4.0,
        height=1.0,
        density=supported_mass / 4.0,
        label="stability_keel",
    )
    sandbox.add_joint(load_deck, stability_keel, (0.0, 5.0), type="rigid")
    return load_deck

def build_agent_stage_3(sandbox):
    arm = sandbox.add_beam(x=0.0, y=5.0, width=8.0, height=0.35, density=1000.0)
    sandbox.add_joint(arm, None, (0.0, 5.0), type="pivot")
    platform = sandbox.add_beam(x=3.0, y=5.5, width=0.4, height=0.2, density=1.0)
    sandbox.add_joint(arm, platform, (3.0, 5.5), type="rigid")
    m_cw = 132.0
    cw = sandbox.add_beam(x=-5.5, y=4.0, width=0.5, height=0.5, density=m_cw/0.25)
    sandbox.add_joint(arm, cw, (-5.5, 5.0), type="rigid")
    return arm

def build_agent_stage_4(sandbox):
    arm = sandbox.add_beam(x=0.0, y=5.0, width=7.0, height=0.4, density=10000.0)
    sandbox.add_joint(arm, None, (0.0, 5.0), type="pivot")
    platform = sandbox.add_beam(x=3.0, y=5.5, width=0.5, height=0.2, density=1.0)
    sandbox.add_joint(arm, platform, (3.0, 5.5), type="rigid")
    m_cw = 119.0
    cw = sandbox.add_beam(x=-3.0, y=0.0, width=0.5, height=0.5, density=m_cw / 0.25)
    sandbox.add_joint(arm, cw, (-3.0, 0.0), type="rigid")
    return arm

def agent_action_stage_1(sandbox, agent_body, step_count): pass

def agent_action_stage_2(sandbox, agent_body, step_count): pass

def agent_action_stage_3(sandbox, agent_body, step_count): pass

def agent_action_stage_4(sandbox, agent_body, step_count): pass
