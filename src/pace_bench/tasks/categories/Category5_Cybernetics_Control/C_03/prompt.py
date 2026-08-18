import os

import json

import importlib.util

_prompt_dir = os.path.dirname(os.path.abspath(__file__))

_spec_c03_env = importlib.util.spec_from_file_location(
    "c03_environment_prompt", os.path.join(_prompt_dir, "environment.py")

)

_c03_env = importlib.util.module_from_spec(_spec_c03_env)

_spec_c03_env.loader.exec_module(_c03_env)

_HREF = _c03_env.HEADING_REFERENCE_MIN_TARGET_SPEED

_ACT_X0 = _c03_env.ACTIVATION_ZONE_X_MIN

_ACT_X1 = _c03_env.ACTIVATION_ZONE_X_MAX

_ACT_STEPS = _c03_env.ACTIVATION_REQUIRED_STEPS

_RDIST = _c03_env.RENDEZVOUS_DISTANCE_DEFAULT

_RZX0 = _c03_env.RENDEZVOUS_ZONE_X_MIN

_RZX1 = _c03_env.RENDEZVOUS_ZONE_X_MAX

_IMPULSE = _c03_env.IMPULSE_BUDGET

_REL_SPD = _c03_env.RENDEZVOUS_REL_SPEED_DEFAULT

_TRACK = _c03_env.TRACK_DISTANCE_DEFAULT

_HEAD_TOL_DEG = _c03_env.RENDEZVOUS_HEADING_TOLERANCE_DEG_DEFAULT

_MAX_ANG_RATE = _c03_env.MAX_ANGULAR_RATE

_COOLDOWN_THRESH = _c03_env.COOLDOWN_THRESHOLD

_COOLDOWN_MAX_THRUST = _c03_env.COOLDOWN_MAX_THRUST

_COOLDOWN_STEPS = _c03_env.COOLDOWN_STEPS

_GY_TOP = _c03_env.DEFAULT_GROUND_Y_TOP

_CORRIDOR_TOL = _c03_env.CORRIDOR_VIOLATION_TOLERANCE

_OBSTACLE_PENETRATION = _c03_env.OBSTACLE_PENETRATION_LIMIT

_EPISODE_STEPS = 10000

_TGT_Y_MIN = _GY_TOP + 0.5

_TGT_Y_MAX = _GY_TOP + 2.0

from pace_bench.tasks.primitives_api import API_INTRO

with open(
    os.path.join(os.path.dirname(__file__), "..", "..", "primitives_api.json"),
    "r",
    encoding="utf-8",
) as f:
    _api_data = json.load(f)

if 'C_03' in _api_data and 'API_INTRO' in _api_data['C_03']:
    del _api_data['C_03']['API_INTRO']

TASK_PROMPT = {
    "task_description": f"""
Design a controller for a seeker craft to achieve multiple heading-aligned rendezvous with a dynamic target.

- **Seeker**: Single thrust vector along current heading (max {_c03_env.MAX_THRUST_MAGNITUDE:g} N). Spawns at (11.0, 1.35) m (x, y) and exhibits drag and rotational resistance. **Heading rotation rate** is limited to {_MAX_ANG_RATE:.2f} rad per simulation step (heading tracks commanded thrust direction at this max rate). **Thrust cooldown**: Exceeding {_COOLDOWN_THRESH:.0f} N thrust triggers a cooldown; during the next {_COOLDOWN_STEPS} steps, maximum thrust is reduced to {_COOLDOWN_MAX_THRUST:.0f} N.
- **Ground**: Top surface at y = {_GY_TOP:.1f} m; ground body has half-height 0.5 m.
- **Target**: Starts at (12.0, 2.0) m and remains inside x ∈ [6, 26] m, y ∈ [{_TGT_Y_MIN:.1f}, {_TGT_Y_MAX:.1f}] m. Its motion can change direction, evade a nearby seeker, and jump; infer the motion law from reported samples.
- **Ice patches**: Low-friction surfaces centered at (9.0, 1.25) m with half-size (1.0, 0.12) m and centered at (16.5, 1.25) m with half-size (1.0, 0.12) m are present in the corridor.
- **Static boxes**: at (7.5, 1.5, 0.3, 0.5); at (14.0, 1.5, 0.3, 0.5); at (20.5, 1.5, 0.3, 0.5).
- **Moving obstacles**: Kinematic boxes oscillating horizontally in the corridor at (10.5, 1.5) and (17.0, 1.5); query `sandbox.get_terrain_obstacles()` for real-time positions.
- **Environmental forcing**: Localized additional forcing may act; infer its presence from seeker motion.
- **Moving corridor**: Allowed seeker x shifts over time (x_lo, x_hi); violation if seeker center leaves corridor by >{_CORRIDOR_TOL:.2f} m. Corridor may pinch inward under certain time conditions.
- **Sensing**: `get_target_position()` is sampled, delayed, and can become stale depending on seeker state and location. Infer target velocity and sensor availability from position history.
- **Activation Gate**: Rendezvous only counts after the seeker "activates" by staying at least {_ACT_STEPS} consecutive steps with seeker x in [{_ACT_X0:.1f}, {_ACT_X1:.1f}] m.
- **Rendezvous slots**: Phase 1 [3700, 3800], [4200, 4300], [4700, 4800]; phase 2 [6200, 6300], [6700, 6800], [7200, 7300] simulation steps. Use `sandbox.get_rendezvous_slots()` for the authoritative intervals for the current run.
- **Fuel**: Total thrust impulse is limited to {_IMPULSE:.0f} N·s.
- **Episode**: {_EPISODE_STEPS} simulation steps.

Design a multi-phase control strategy:
1. **Activation**: Position and hold the seeker in the activation zone until activated.
2. **Slotted Rendezvous**: Complete the first rendezvous in a phase-1 slot and the second in a phase-2 slot; rendezvous requires: distance to target ≤ {_RDIST:.1f} m, relative speed < {_REL_SPD:g} m/s, AND seeker heading within {_HEAD_TOL_DEG:.1f}° of the reference direction (target velocity direction when target speed ≥ {_HREF:g} m/s, else seeker-to-target direction).
3. **Tracking**: Maintain distance <= {_TRACK:g} m from target after the second rendezvous until the end.
""",
    "success_criteria": f"""

1. **Rendezvous Completion**: Achieve rendezvous in both phase-1 and phase-2 designated time slots (verify via `sandbox.get_rendezvous_slots()`).
2. **Capture envelope**: Activation already achieved (≥{_ACT_STEPS} consecutive steps with seeker x ∈ [{_ACT_X0:.1f}, {_ACT_X1:.1f}] m); seeker x ∈ [{_RZX0:.1f}, {_RZX1:.1f}] m; distance to **true** target ≤ {_RDIST:.1f} m; relative speed < {_REL_SPD:g} m/s; heading within {_HEAD_TOL_DEG:.1f}° of target velocity direction if target speed ≥ {_HREF:g} m/s, else seeker-to-target direction.
3. **Tracking**: Maintain distance <= {_TRACK:g} m from target after the second rendezvous until the end.
4. **Safety**: No collisions with corridor obstacles; stay within the moving corridor.
5. **Efficiency**: Total thrust impulse must not exceed **{_IMPULSE:.0f} N·s**.

- **Fuel budget**: {_IMPULSE:.0f} N·s total thrust impulse; reaching or exceeding fails the run.
- **Corridor**: Seeker center must stay within shifting bounds (violation margin >{_CORRIDOR_TOL:.2f} m fails).
- **Obstacle collision**: Seeker must not penetrate any corridor box (static or kinematic) by ≥{_OBSTACLE_PENETRATION:.2f} m.
- **Thrust cooldown**: Exceeding {_COOLDOWN_THRESH:.0f} N thrust triggers cooldown; max thrust reduced to {_COOLDOWN_MAX_THRUST:.0f} N for {_COOLDOWN_STEPS} steps.
- **APIs**: Use only the primitives documented below.
""",
    'primitives_api': API_INTRO + '\n' + '\n\n'.join(_api_data['C_03'].values()),

}
