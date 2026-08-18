import math

_booster = None

def build_agent(sandbox):
    global _booster
    chassis_y = 2.1
    chassis_x = 6.9
    chassis = sandbox.add_beam(x=chassis_x, y=chassis_y, width=1.0, height=0.2, density=1.0)
    sandbox.set_fixed_rotation(chassis, True)
    _booster = sandbox.add_beam(x=chassis_x, y=chassis_y, width=0.4, height=0.4, density=200.0)
    sandbox.add_joint(chassis, _booster, (chassis_x, chassis_y), type='rigid')
    for x_off in [-0.4, 0.4]:
        w = sandbox.add_wheel(x=chassis_x + x_off, y=1.7, radius=0.2, density=1.0)
        sandbox.add_joint(chassis, w, (chassis_x + x_off, 1.7), type='pivot')
    plate = sandbox.add_beam(x=chassis_x + 0.6, y=1.9, width=0.4, height=0.8, density=1.0)
    sandbox.add_joint(chassis, plate, (chassis_x + 0.5, 2.1), type='rigid')
    return chassis

def agent_action(sandbox, agent_body, step_count):
    global _booster
    if _booster is None:
        return
    obj_pos = sandbox.get_object_position() if hasattr(sandbox, 'get_object_position') else None
    vx = 0.5
    if obj_pos is not None:
        object_x = obj_pos[0]
        if agent_body.position.x > object_x + 1.5:
            vx = 0.0
    _booster.linearVelocity = (vx, 0.0)

_stage_1_booster = None

def build_agent_stage_1(sandbox):
    global _stage_1_booster
    sled_y = 1.55
    sled = sandbox.add_beam(x=2.0, y=sled_y, width=2.0, height=0.1, density=0.6)
    sandbox.set_fixed_rotation(sled, True)
    _stage_1_booster = sandbox.add_beam(x=1.5, y=sled_y, width=0.3, height=0.3, density=160.0)
    sandbox.add_joint(sled, _stage_1_booster, (1.5, sled_y), type='rigid')
    push_plate = sandbox.add_beam(x=3.3, y=2.20, width=0.15, height=0.9, density=0.6)
    sandbox.add_joint(sled, push_plate, (3.3, sled_y), type='rigid')
    return sled

def agent_action_stage_1(sandbox, agent_body, step_count):
    global _stage_1_booster
    if _stage_1_booster:
        if step_count < 120:
            _stage_1_booster.linearVelocity = (2.0, 0.0)
        else:
            _stage_1_booster.linearVelocity = (4.0, 0.0)

_stage_2_booster = None

def build_agent_stage_2(sandbox):
    global _stage_2_booster
    chassis_x = 5.0
    chassis_y = 2.0
    plate = sandbox.add_beam(x=chassis_x + 1.0, y=2.0, width=0.4, height=2.0, density=10.0)
    sandbox.set_fixed_rotation(plate, True)
    _stage_2_booster = sandbox.add_beam(x=chassis_x, y=2.0, width=0.5, height=0.5, density=100.0)
    sandbox.add_joint(plate, _stage_2_booster, (chassis_x, 2.0), type='rigid')
    w1 = sandbox.add_wheel(x=chassis_x, y=1.5, radius=0.6, density=5.0)
    sandbox.add_joint(plate, w1, (chassis_x, 1.5), type='pivot')
    return plate

def agent_action_stage_2(sandbox, agent_body, step_count):
    global _stage_2_booster
    if _stage_2_booster: _stage_2_booster.linearVelocity = (4.0, 0.0)

_stage_3_booster = None

def build_agent_stage_3(sandbox):
    global _stage_3_booster
    chassis_x = 5.0
    chassis_y = 2.0
    chassis = sandbox.add_beam(x=chassis_x, y=chassis_y, width=1.5, height=0.2, density=1.0)
    sandbox.set_fixed_rotation(chassis, True)
    _stage_3_booster = sandbox.add_beam(x=chassis_x, y=chassis_y, width=0.4, height=0.4, density=50.0)
    sandbox.add_joint(chassis, _stage_3_booster, (chassis_x, chassis_y), type='rigid')
    top_wedge = sandbox.add_beam(x=chassis_x + 1.2, y=chassis_y + 0.5, width=1.0, height=0.1, angle=0.8, density=1.0)
    sandbox.add_joint(chassis, top_wedge, (chassis_x + 0.75, chassis_y), type='rigid')
    bot_wedge = sandbox.add_beam(x=chassis_x + 1.2, y=chassis_y - 0.5, width=1.0, height=0.1, angle=-0.8, density=1.0)
    sandbox.add_joint(chassis, bot_wedge, (chassis_x + 0.75, chassis_y), type='rigid')
    return chassis

def agent_action_stage_3(sandbox, agent_body, step_count):
    global _stage_3_booster
    if _stage_3_booster:
        if step_count < 600:
            _stage_3_booster.linearVelocity = (4.0, 0.0)
        else:
            _stage_3_booster.linearVelocity = (0.0, 0.0)

_stage_4_booster = None

def build_agent_stage_4(sandbox):
    global _stage_4_booster
    sled = sandbox.add_beam(x=4.5, y=1.55, width=5.5, height=0.15, density=0.5)
    sandbox.set_fixed_rotation(sled, True)
    _stage_4_booster = sandbox.add_beam(x=3.5, y=1.55, width=0.3, height=0.3, density=150.0)
    sandbox.add_joint(sled, _stage_4_booster, (3.5, 1.55), type='rigid')
    pusher = sandbox.add_beam(x=7.0, y=1.65, width=0.3, height=0.7, density=0.8)
    sandbox.add_joint(sled, pusher, (7.0, 1.55), type='rigid')
    return sled

def agent_action_stage_4(sandbox, agent_body, step_count):
    global _stage_4_booster
    if _stage_4_booster: _stage_4_booster.linearVelocity = (6.0, 0.0)
