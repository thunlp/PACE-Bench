def build_agent(sandbox):
    sandbox.add_block(-0.5, 0.11, 1.0, 0.2)
    sandbox.add_block(-0.4, 0.31, 1.0, 0.2)
    return None

def agent_action(sandbox, agent_body, step_count):
    pass

def build_agent_stage_1(sandbox):
    sandbox.add_block(-0.48, 0.11, 1.0, 0.2, density=60.0)
    sandbox.add_block(-0.20, 0.31, 1.0, 0.2, density=35.0)
    sandbox.add_block(0.10, 0.51, 1.0, 0.2, density=25.0)
    return None

def agent_action_stage_1(sandbox, agent_body, step_count):
    pass

def build_agent_stage_2(sandbox):
    sandbox.add_block(-0.70, 0.11, 1.0, 0.2, density=100.0)
    sandbox.add_block(-0.45, 0.31, 1.0, 0.2, density=22.0)
    sandbox.add_block(-0.05, 0.51, 1.0, 0.2, density=9.0)
    sandbox.add_block(0.115, 0.71, 1.0, 0.2, density=4.0)
    return None

def agent_action_stage_2(sandbox, agent_body, step_count):
    pass

def build_agent_stage_3(sandbox):
    sandbox.add_block(-0.2665, 0.11, 1.0, 0.2, density=40.49)
    sandbox.add_block(-0.0508, 0.31, 1.0, 0.2, density=22.25)
    sandbox.add_block(0.1650, 0.51, 1.0, 0.2, density=12.25)
    sandbox.add_block(0.4050, 0.71, 1.0, 0.2, density=7.5)
    sandbox.add_block(0.8850, 0.91, 1.0, 0.2, density=7.5)
    return None

def agent_action_stage_3(sandbox, agent_body, step_count):
    pass

def build_agent_stage_4(sandbox):
    sandbox.add_block(-0.40, 0.11, 1.0, 0.2, density=52.0)
    sandbox.add_block(-0.18, 0.31, 1.0, 0.2, density=19.0)
    sandbox.add_block(0.05, 0.51, 1.0, 0.2, density=12.0)
    return None

def agent_action_stage_4(sandbox, agent_body, step_count):
    pass
