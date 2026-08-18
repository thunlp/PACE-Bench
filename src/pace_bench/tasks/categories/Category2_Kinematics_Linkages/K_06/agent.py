GLASS_Y = 2.0

CENTER_X = 6.0

GROUND_Y = 2.06

BAR_Y = 2.08

BAR_H = 0.24

SEG_W = 2.0

DENSITY = 0.12

BAR_FRICTION = 0.6

def build_agent(sandbox):
    base = sandbox.add_beam(x=CENTER_X, y=GROUND_Y, width=0.5, height=0.12, angle=0, density=1.0)
    if hasattr(sandbox, 'weld_to_glass'):
        sandbox.weld_to_glass(base, (CENTER_X, GLASS_Y))
    seg_centers = [2.0, 4.0, 6.0, 8.0, 10.0]
    bars = []
    for cx in seg_centers:
        b = sandbox.add_beam(x=cx, y=BAR_Y, width=SEG_W, height=BAR_H, angle=0, density=DENSITY)
        sandbox.set_material_properties(b, restitution=0.0, friction=BAR_FRICTION)
        bars.append(b)
    for i in range(len(bars) - 1):
        jx = (seg_centers[i] + seg_centers[i+1]) / 2.0
        sandbox.add_joint(bars[i], bars[i+1], (jx, BAR_Y), type='rigid')
    pivot = sandbox.add_joint(
        base, bars[2], (CENTER_X, BAR_Y), type='pivot',
        lower_limit=-1.3, upper_limit=1.3
    )
    return {"body": base, "motors": [pivot]}

def agent_action(sandbox, agent_body, step_count):
    motors = agent_body.get("motors", []) if isinstance(agent_body, dict) else []
    if not motors:
        return
    if hasattr(sandbox, 'set_awake'):
        for body in sandbox.bodies:
            sandbox.set_awake(body, True)
    period = 300
    half = (step_count // period) % 2
    motor_speed = 18.0 if half == 0 else -18.0
    sandbox.set_motor(motors[0], motor_speed, max_torque=4500.0)

def build_agent_stage_1(sandbox):
    PIVOT_Y = 2.09
    DENSITY = 0.005
    BAR_FRICTION = 0.95
    BAR_H = 0.22
    BAR_W = 1.2
    pivot_xs = [float(x) for x in range(2, 11)]
    motors = []
    first_base = None
    for px in pivot_xs:
        base = sandbox.add_beam(x=px, y=2.09, width=0.10, height=0.06, angle=0, density=0.30)
        if hasattr(sandbox, 'weld_to_glass'):
            sandbox.weld_to_glass(base, (px, 2.0))
        if first_base is None:
            first_base = base
        bar = sandbox.add_beam(x=px, y=PIVOT_Y, width=BAR_W, height=BAR_H, angle=0, density=DENSITY)
        sandbox.set_material_properties(bar, restitution=0.0, friction=BAR_FRICTION)
        pivot = sandbox.add_joint(base, bar, (px, PIVOT_Y), type='pivot', lower_limit=-0.8, upper_limit=0.8)
        motors.append(pivot)
    return {"body": first_base, "motors": motors}

def agent_action_stage_1(sandbox, agent_body, step_count):
    motors = agent_body.get("motors", []) if isinstance(agent_body, dict) else []
    if not motors:
        return
    if hasattr(sandbox, 'set_awake'):
        for body in sandbox.bodies:
            sandbox.set_awake(body, True)
    period = 60
    half = (step_count // period) % 2
    motor_speed = 16.0 if half == 0 else -16.0
    for motor in motors:
        sandbox.set_motor(motor, motor_speed=motor_speed, max_torque=100)

def build_agent_stage_2(sandbox):
    GLASS_Y = 2.0
    CENTER_X = 6.0
    GROUND_Y = 2.06
    BAR_Y = 2.08
    BAR_H = 0.10
    SEG_W = 2.0
    DENSITY = 0.006
    BAR_FRICTION = 0.92
    seg_centers = [2.0, 4.0, 6.0, 8.0, 10.0]
    base = sandbox.add_beam(x=CENTER_X, y=GROUND_Y, width=0.20, height=0.06, angle=0, density=0.30)
    if hasattr(sandbox, 'weld_to_glass'):
        sandbox.weld_to_glass(base, (CENTER_X, GLASS_Y))
    bars = []
    for cx in seg_centers:
        b = sandbox.add_beam(x=cx, y=BAR_Y, width=SEG_W, height=BAR_H, angle=0, density=DENSITY)
        sandbox.set_material_properties(b, restitution=0.0, friction=BAR_FRICTION)
        bars.append(b)
    for i in range(len(bars) - 1):
        jx = (seg_centers[i] + seg_centers[i+1]) / 2.0
        sandbox.add_joint(bars[i], bars[i+1], (jx, BAR_Y), type='rigid')
    pivot = sandbox.add_joint(base, bars[2], (CENTER_X, BAR_Y), type='pivot', lower_limit=-1.3, upper_limit=1.3)
    return {"body": base, "motors": [pivot]}

def agent_action_stage_2(sandbox, agent_body, step_count):
    motors = agent_body.get("motors", []) if isinstance(agent_body, dict) else []
    if not motors:
        return
    if hasattr(sandbox, 'set_awake'):
        for body in sandbox.bodies:
            sandbox.set_awake(body, True)
    period = 80
    half = (step_count // period) % 2
    motor_speed = 4.0 if half == 0 else -4.0
    sandbox.set_motor(motors[0], motor_speed, max_torque=1e8)

def build_agent_stage_3(sandbox):
    pivot_y = 2.05
    pivot_xs = [2.0, 4.0, 6.0, 8.0, 10.0]
    motors = []
    agent_body = None
    for pivot_x in pivot_xs:
        base = sandbox.add_beam(
            x=pivot_x, y=2.09, width=0.08, height=0.08, angle=0, density=0.40
        )
        if hasattr(sandbox, 'weld_to_glass'):
            sandbox.weld_to_glass(base, (pivot_x, 2.0))
        if agent_body is None:
            agent_body = base
        bar = sandbox.add_beam(
            x=pivot_x, y=pivot_y, width=2.0, height=0.08, angle=0, density=0.004
        )
        sandbox.set_material_properties(bar, restitution=0.0, friction=0.95)
        motors.append(
            sandbox.add_joint(
                base,
                bar,
                (pivot_x, pivot_y),
                type='pivot',
                lower_limit=-0.85,
                upper_limit=0.85,
            )
        )
    return {"body": agent_body, "motors": motors}

def agent_action_stage_3(sandbox, agent_body, step_count):
    motors = agent_body.get("motors", []) if isinstance(agent_body, dict) else []
    if not motors:
        return
    if hasattr(sandbox, 'set_awake'):
        for body in sandbox.bodies:
            sandbox.set_awake(body, True)
    period = 50
    half = (step_count // period) % 2
    motor_speed = 14.0 if half == 0 else -14.0
    for motor in motors:
        sandbox.set_motor(motor, motor_speed=motor_speed, max_torque=28.0)

def build_agent_stage_4(sandbox):
    PIVOT_Y = 2.05
    DENSITY = 0.004
    BAR_FRICTION = 0.95
    BAR_H = 0.08
    BAR_W = 2.0
    pivot_xs = [2.0, 4.0, 6.0, 8.0, 10.0]
    motors = []
    first_base = None
    for px in pivot_xs:
        base = sandbox.add_beam(x=px, y=2.09, width=0.08, height=0.08, angle=0, density=0.40)
        if hasattr(sandbox, 'weld_to_glass'):
            sandbox.weld_to_glass(base, (px, 2.0))
        if first_base is None:
            first_base = base
        bar = sandbox.add_beam(x=px, y=PIVOT_Y, width=BAR_W, height=BAR_H, angle=0, density=DENSITY)
        sandbox.set_material_properties(bar, restitution=0.0, friction=BAR_FRICTION)
        pivot = sandbox.add_joint(base, bar, (px, PIVOT_Y), type='pivot', lower_limit=-0.85, upper_limit=0.85)
        motors.append(pivot)
    return {"body": first_base, "motors": motors}

def agent_action_stage_4(sandbox, agent_body, step_count):
    motors = agent_body.get("motors", []) if isinstance(agent_body, dict) else []
    if not motors:
        return
    if hasattr(sandbox, 'set_awake'):
        for body in sandbox.bodies:
            sandbox.set_awake(body, True)
    period = 50
    half = (step_count // period) % 2
    motor_speed = 14.0 if half == 0 else -14.0
    for m in motors:
        sandbox.set_motor(m, motor_speed=motor_speed, max_torque=1000.0)
