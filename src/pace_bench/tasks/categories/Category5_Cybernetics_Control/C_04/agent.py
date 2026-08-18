import math

ACTIVATION_X_MAX = 10.0

ACTIVATION_X_MIN = 5.0

AGENT_MASS = 5.0

BACKWARD_FX_THRESHOLD = -34.0

BACKWARD_SPEED_MAX = 100.0

EXIT_X_MIN = 15.0

TIME_STEP = 1.0 / 60.0

def _control_cfg(_sandbox):
    return {
        "act_lo": ACTIVATION_X_MIN,
        "act_hi": ACTIVATION_X_MAX,
        "bfx": BACKWARD_FX_THRESHOLD,
        "bspd": BACKWARD_SPEED_MAX,
    }

class Memory:
    def __init__(self):
        self.data = {}
    def clear(self):
        self.data = {}

MEM = Memory()

def _weight_comp(sandbox):
    gy = float(sandbox.world.gravity[1])
    return float(AGENT_MASS) * abs(gy)

def _exit_x_min(_sandbox):
    return float(EXIT_X_MIN)

def build_agent(sandbox):
    MEM.clear()
    return sandbox.get_agent_body()

def agent_action(sandbox, agent_body, step_count):
    if "initial_state" not in MEM.data:
        MEM.data["initial_state"] = {
            "phase": "APPROACH",
            "t": 0,
            "lx": None,
            "ly": None,
            "vx": 0.0,
            "vy": 0.0,
        }
    state = MEM.data["initial_state"]
    cfg = _control_cfg(sandbox)
    pd = sandbox.get_agent_position()
    dt = TIME_STEP
    if state["lx"] is not None:
        state["vx"] = (pd[0] - state["lx"]) / dt
        state["vy"] = (pd[1] - state["ly"]) / dt
    state["lx"], state["ly"] = pd[0], pd[1]
    vxd, vyd = state["vx"], state["vy"]
    x, y = pd[0], pd[1]
    ty, w_comp = 1.5, _weight_comp(sandbox)
    if state["phase"] == "APPROACH":
        fx = 15.0 if x < 5.0 else 10.0 * (7.0 - x) - 5.0 * vxd
        fy = 50.0 * (ty - y) - 20.0 * vyd + w_comp
        if cfg["act_lo"] <= x <= cfg["act_hi"] and abs(vxd) < 0.5:
            state["phase"] = "UNLOCK"
            state["t"] = 0
    elif state["phase"] == "UNLOCK":
        vx_p, vy_p = sandbox.get_agent_velocity()
        spd = math.hypot(vx_p, vy_p)
        fx_cmd = cfg["bfx"] - 1.0
        fy_cmd = 50.0 * (ty - y) - 20.0 * vyd + w_comp
        if spd >= cfg["bspd"] * 0.99:
            horiz = -1.5 * vx_p
            if horiz > 0:
                horiz = 0
            fx_cmd += horiz
            fy_cmd -= 1.5 * vy_p
        fx, fy = fx_cmd, fy_cmd
        state["t"] += 1
        if state["t"] > 60:
            state["phase"] = "ESCAPE"
    elif state["phase"] == "ESCAPE":
        fx, fy = 15.0, 50.0 * (ty - y) - 20.0 * vyd + w_comp
        if x > _exit_x_min(sandbox) + 2.5:
            state["phase"] = "HOLD"
    elif state["phase"] == "HOLD":
        fx, fy = 0.0, w_comp
    sandbox.apply_agent_force(fx, fy)

def build_agent_stage_1(sandbox):
    MEM.clear()
    return sandbox.get_agent_body()

def agent_action_stage_1(sandbox, agent_body, step_count):
    cfg = _control_cfg(sandbox)
    p, v = sandbox.get_agent_position(), sandbox.get_agent_velocity()
    w_comp = _weight_comp(sandbox)
    ty = 1.5
    mag_comp = 60.0 if p[1] < 1.6 else 0.0
    if "phase1" not in MEM.data:
        MEM.data["phase1"] = "APPROACH"
        MEM.data["t1"] = step_count
        MEM.data["vx_lag"] = 0.0
        MEM.data["vy_lag"] = 0.0
    phase = MEM.data["phase1"]
    x, y = p[0], p[1]
    if MEM.data["vx_lag"] != 0.0:
        vxd = MEM.data["vx_lag"]
        vyd = MEM.data["vy_lag"]
    else:
        vxd, vyd = v[0], v[1]
    if phase == "APPROACH":
        fx = 15.0 if x < 5.0 else 10.0 * (7.0 - x) - 5.0 * vxd
        fy = 50.0 * (ty - y) - 20.0 * vyd + w_comp + mag_comp
        if cfg["act_lo"] <= x <= cfg["act_hi"] and abs(vxd) < 0.5:
            MEM.data["phase1"] = "UNLOCK"
            MEM.data["t1"] = step_count
    elif phase == "UNLOCK":
        steps_in_unlock = step_count - MEM.data["t1"]
        vx_p, vy_p = sandbox.get_agent_velocity()
        spd = math.hypot(vx_p, vy_p)
        fx_cmd = cfg["bfx"] - 1.0
        fy_cmd = 50.0 * (ty - y) - 20.0 * vyd + w_comp + mag_comp
        if spd >= cfg["bspd"] * 0.99:
            horiz = -1.5 * vx_p
            if horiz > 0:
                horiz = 0
            fx_cmd += horiz
            fy_cmd -= 1.5 * vy_p
        fx, fy = fx_cmd, fy_cmd
        MEM.data["vx_lag"] = vx_p
        MEM.data["vy_lag"] = vy_p
        if steps_in_unlock > 60:
            MEM.data["phase1"] = "ESCAPE"
    elif phase == "ESCAPE":
        fx = 15.0
        fy = 50.0 * (ty - y) - 20.0 * v[1] + w_comp + mag_comp
        if x > _exit_x_min(sandbox) + 2.5:
            MEM.data["phase1"] = "HOLD"
    else:
        fx, fy = 0.0, w_comp + mag_comp
    sandbox.apply_agent_force(fx, fy)

def build_agent_stage_2(sandbox):
    MEM.clear()
    return sandbox.get_agent_body()

def agent_action_stage_2(sandbox, agent_body, step_count):
    cfg = _control_cfg(sandbox)
    p, v = sandbox.get_agent_position(), sandbox.get_agent_velocity()
    w_comp = _weight_comp(sandbox)
    ty = 1.5
    if "phase2" not in MEM.data:
        MEM.data["phase2"] = "APPROACH"
        MEM.data["t2"] = step_count
    phase = MEM.data["phase2"]
    x, y = p[0], p[1]
    if phase == "APPROACH":
        fx = 15.0 if x < 5.0 else 10.0 * (7.0 - x) - 5.0 * v[0]
        ty_use = 2.3 if x > 8.0 else ty
        fy = 50.0 * (ty_use - y) - 20.0 * v[1] + w_comp
        if cfg["act_lo"] <= x <= cfg["act_hi"] and abs(v[0]) < 0.5:
            MEM.data["phase2"] = "UNLOCK"
            MEM.data["t2"] = step_count
    elif phase == "UNLOCK":
        steps_in_unlock = step_count - MEM.data["t2"]
        vx_p, vy_p = sandbox.get_agent_velocity()
        spd = math.hypot(vx_p, vy_p)
        fx_cmd = cfg["bfx"] - 1.0
        ty_use = 2.3
        fy_cmd = 50.0 * (ty_use - y) - 20.0 * v[1] + w_comp
        if spd >= cfg["bspd"] * 0.99:
            horiz = -1.5 * vx_p
            if horiz > 0:
                horiz = 0
            fx_cmd += horiz
            fy_cmd -= 1.5 * vy_p
        fx, fy = fx_cmd, fy_cmd
        if steps_in_unlock > 60:
            MEM.data["phase2"] = "ESCAPE"
    elif phase == "ESCAPE":
        ty_use = 2.3 if x < 14.0 else ty
        fx, fy = 15.0, 50.0 * (ty_use - y) - 20.0 * v[1] + w_comp
        if x > 17.5:
            MEM.data["phase2"] = "HOLD"
    else:
        fx, fy = 0.0, w_comp
    sandbox.apply_agent_force(fx, fy)

def build_agent_stage_3(sandbox):
    MEM.clear()
    return sandbox.get_agent_body()

def agent_action_stage_3(sandbox, agent_body, step_count):
    cfg = _control_cfg(sandbox)
    p, v = sandbox.get_agent_position(), sandbox.get_agent_velocity()
    w_comp = _weight_comp(sandbox)
    ty = 2.0
    if "phase3" not in MEM.data:
        MEM.data["phase3"] = "APPROACH"
        MEM.data["t3"] = step_count
    phase = MEM.data["phase3"]
    x, y = p[0], p[1]
    if phase == "APPROACH":
        fx = 30.0 if x < 5.0 else 80.0
        fy = 60.0 * (ty - y) - 25.0 * v[1] + w_comp
        if cfg["act_lo"] <= x <= cfg["act_hi"] and abs(v[0]) < 0.5:
            MEM.data["phase3"] = "UNLOCK"
            MEM.data["t3"] = step_count
    elif phase == "UNLOCK":
        steps_in_unlock = step_count - MEM.data["t3"]
        vx_p, vy_p = sandbox.get_agent_velocity()
        spd = math.hypot(vx_p, vy_p)
        fx_cmd = cfg["bfx"] - 2.0
        fy_cmd = 60.0 * (ty - y) - 25.0 * v[1] + w_comp
        if spd >= cfg["bspd"] * 0.98:
            horiz = -2.0 * vx_p
            if horiz > 0:
                horiz = 0
            fx_cmd += horiz
            fy_cmd -= 2.0 * vy_p
        fx, fy = fx_cmd, fy_cmd
        if steps_in_unlock > 65:
            MEM.data["phase3"] = "ESCAPE"
    elif phase == "ESCAPE":
        ty_esc = 1.4
        fx, fy = 80.0, 60.0 * (ty_esc - y) - 25.0 * v[1] + w_comp
        if x > 17.5:
            MEM.data["phase3"] = "HOLD"
    else:
        fx, fy = 0.0, w_comp
    sandbox.apply_agent_force(fx, fy)

def build_agent_stage_4(sandbox):
    MEM.clear()
    return sandbox.get_agent_body()

def agent_action_stage_4(sandbox, agent_body, step_count):
    cfg = _control_cfg(sandbox)
    p, v = sandbox.get_agent_position(), sandbox.get_agent_velocity()
    w_comp = _weight_comp(sandbox)
    ty = 1.5
    mag_comp = 80.0 if p[1] < 1.5 else 0.0
    if "phase4" not in MEM.data:
        MEM.data["phase4"] = "APPROACH"
        MEM.data["t4"] = step_count
    phase = MEM.data["phase4"]
    x, y = p[0], p[1]
    if phase == "APPROACH":
        fx = -5.0 if x < 3.0 else -10.0 * (7.0 - x) - 5.0 * v[0]
        fy = 60.0 * (ty - y) - 25.0 * v[1] + w_comp + mag_comp
        if cfg["act_lo"] <= x <= cfg["act_hi"] and abs(v[0]) < 0.4:
            MEM.data["phase4"] = "UNLOCK"
            MEM.data["t4"] = step_count
    elif phase == "UNLOCK":
        steps_in_unlock = step_count - MEM.data["t4"]
        vx_p, vy_p = sandbox.get_agent_velocity()
        spd = math.hypot(vx_p, vy_p)
        fx_cmd = cfg["bfx"] - 1.0
        fy_cmd = 60.0 * (ty - y) - 25.0 * v[1] + w_comp + mag_comp
        if spd >= cfg["bspd"] * 0.99:
            horiz = -1.5 * vx_p
            if horiz > 0:
                horiz = 0
            fx_cmd += horiz
            fy_cmd -= 1.5 * vy_p
        fx, fy = fx_cmd, fy_cmd
        if steps_in_unlock > 70:
            MEM.data["phase4"] = "ESCAPE"
    elif phase == "ESCAPE":
        fx, fy = -15.0, 60.0 * (ty - y) - 25.0 * v[1] + w_comp + mag_comp
        if x > 17.5:
            MEM.data["phase4"] = "HOLD"
    else:
        fx, fy = 0.0, w_comp + mag_comp
    sandbox.apply_agent_force(fx, fy)
