import importlib.util

import json

import os

import re

_env_path = os.path.join(os.path.dirname(__file__), "environment.py")

_spec = importlib.util.spec_from_file_location("c01_environment", _env_path)

_c01_environment = importlib.util.module_from_spec(_spec)

_spec.loader.exec_module(_c01_environment)

_env = _c01_environment

CART_FORCE_LIMIT_NEWTONS = _env.CART_FORCE_LIMIT_NEWTONS

_fn = int(CART_FORCE_LIMIT_NEWTONS) if float(CART_FORCE_LIMIT_NEWTONS).is_integer() else CART_FORCE_LIMIT_NEWTONS

_BAL_DEG = int(_env.BALANCE_ANGLE_DEG) if float(_env.BALANCE_ANGLE_DEG).is_integer() else _env.BALANCE_ANGLE_DEG

_FAIL_DEG = int(_env.FAILURE_ANGLE_DEG) if float(_env.FAILURE_ANGLE_DEG).is_integer() else _env.FAILURE_ANGLE_DEG

_HOLD_STEPS = int(_env.BALANCE_HOLD_STEPS_REQUIRED)

_CART_M = _env.CART_MASS

_POLE_M = _env.POLE_MASS

_POLE_LEN = _env.POLE_LENGTH

_POLE_W = _env.POLE_WIDTH

_TRACK_CX = _env.TRACK_CENTER_X

_SAFE_HALF = _env.SAFE_HALF_RANGE

_RAIL_Y = _env.CART_RAIL_CENTER_Y

_MAX_STEPS = _env.MAX_STEPS

_FPS = _env.FPS

_VEL_IT = _env.WORLD_VELOCITY_ITERATIONS

_POS_IT = _env.WORLD_POSITION_ITERATIONS

from pace_bench.tasks.primitives_api import API_INTRO

with open(os.path.join(os.path.dirname(__file__), '..', '..', 'primitives_api.json'), 'r') as f:
    _api_data = json.load(f)

if 'C_01' in _api_data and 'API_INTRO' in _api_data['C_01']:
    del _api_data['C_01']['API_INTRO']

_clamp_bracket = re.compile(r"\[-\d+(?:\.\d+)?,\s*\d+(?:\.\d+)?\]")

for _k, _v in list(_api_data.get('C_01', {}).items()):
    if isinstance(_v, str) and _k == "APPLY_CART_FORCE":
        _patched = _clamp_bracket.sub(f'[-{_fn}, {_fn}]', _v, count=1)
        _api_data['C_01'][_k] = _patched

TASK_PROMPT = {
    "task_description": f"""
Design a controller to maintain a pole balanced on a moving cart.

- **Cart**: The cart has dimensions 1.0m (width) x 0.5m (height) and an undisclosed mass. It moves along a horizontal track at y={_RAIL_Y}m (center x={_TRACK_CX:g}m, safe range ±{_SAFE_HALF:g}m inclusive). Infer inertial properties from observed response rather than simulator internals.
- **Pole**: The pole has undisclosed mass, width {_POLE_W}m, starts upright (angle = 0° or 0rad), and has visible length {_POLE_LEN:.1f}m. Mutated length and initial angle are directly visible geometry/state and are stated when changed; inertial properties remain latent.
- **Gravity**: Gravitational acceleration is applied per the simulator’s +y-up world frame; the effective vector (magnitude and sign along y) may differ between runs—do not assume a specific value without checking behavior (see any **Environmental Anomalies** appendix on mutated runs).
- **Contact model**: Cart, pole, and track interact through the simulator’s default rigid-body contact model unless a run overrides it.
- **Default contact**: Engine-default friction and restitution apply to cart, pole, and track fixtures (no additional task-specific surface coefficients in the baseline configuration).
- **Physics integrator**: Each `environment.step` runs one Box2D solve with **{_VEL_IT}** velocity iterations and **{_POS_IT}** position iterations per step (fixed for this task).
- **Actuator Limit**: The cart force is limited to ±{_fn}N.
- **Sensor reporting (angle)**: The reported angle may have an undisclosed fixed delay from true state; infer latency from response.
- **Sensor reporting (angular velocity)**: The reported angular velocity may have an undisclosed fixed delay from true state; infer latency from response.
- **Goal**: Maintain the pole in the upright position (|angle| <= {_BAL_DEG}°) for at least **{_HOLD_STEPS} consecutive simulation steps** (one count per completed `environment.step` / physics integration; the harness may call the evaluator once at step index 0 before the first integration—**that call does not advance** the lock-in counter). **Scoring**: After that lock-in, the episode does not end early solely because |angle| exceeds {_BAL_DEG}°, until the pole passes horizontal (|angle| > {_FAIL_DEG}°), you leave the track, or you hit the step limit. **Final success** still requires |angle| <= {_BAL_DEG}° at the last step. Before balance lock-in, the episode does **not** end only because |angle| briefly exceeds {_BAL_DEG}°; only track exit, the step limit, or (after lock-in) |angle| > {_FAIL_DEG}° ends the run early as described above.
- **Grading vs sensors**: All angle conditions above ({_BAL_DEG}°, {_FAIL_DEG}°, lock-in count, and terminal check) use the simulator’s **true** pole state. Values from `get_pole_angle` / `get_pole_angular_velocity` follow the **Sensor reporting** delays above and may therefore differ from the state used for scoring—design accordingly.
- **Episode length**: At most {_MAX_STEPS} simulation steps (must hold balance until the end).
- **Time Step**: Exactly **1/{_FPS}** seconds per simulation step (same as the environment integration step).

Design a control strategy:
1. **Balance**: Achieve {_HOLD_STEPS} consecutive simulation steps with |angle| <= {_BAL_DEG}° (see **Goal** for how the lock-in counter relates to `environment.step`), keep the cart within track limits, and end the episode with |angle| <= {_BAL_DEG}° (see **Goal** for mid-episode rules after lock-in).
2. Observe state through the documented APIs; remember scoring uses **true** pole angles as stated under **Grading vs sensors**.
""",
    "success_criteria": f"""

1. **Stability**: Pole is held within the upright region (|angle| <= {_BAL_DEG}°) for at least {_HOLD_STEPS} consecutive simulation steps (same lock-in counting rule as in **Goal**), where **|angle| is the simulator’s true pole angle** (which may differ from `get_pole_angle` and `get_pole_angular_velocity` when **Sensor reporting (angle)** or **Sensor reporting (angular velocity)** delays are non-zero). After lock-in, early termination is only for track exit, time limit, or |angle| > {_FAIL_DEG}°; **final** success requires |angle| <= {_BAL_DEG}° when the episode ends.
2. **Track Limits**: Cart remains within the safe zone (|x - {_TRACK_CX:g}| ≤ {_SAFE_HALF:g}m).
3. **Episode length**: At most {_MAX_STEPS} simulation steps.

- **Actuator**: Cart force must not exceed ±{_fn}N.
- **Pole**: Width {_POLE_W}m.
- **APIs**: Use only the primitives documented below.
""",
    'primitives_api': API_INTRO + '\n' + '\n\n'.join(_api_data['C_01'].values()),

}
