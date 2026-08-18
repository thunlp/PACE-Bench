import os

import json

from pace_bench.tasks.primitives_api import API_INTRO

with open(os.path.join(os.path.dirname(__file__), '..', '..', 'primitives_api.json'), 'r') as f:
    _api_data = json.load(f)

if 'C_06' in _api_data and 'API_INTRO' in _api_data['C_06']:
    del _api_data['C_06']['API_INTRO']

from pace_bench.core.simulator import TIME_STEP

from pace_bench.tasks.categories.Category5_Cybernetics_Control.C_06.environment import (
    DEFAULT_WHEEL_MASS_KG,
    DEFAULT_WHEEL_RADIUS_M,
    MAX_STEPS,
    MEAN_SPEED_ERROR_THRESHOLD,
    REGULATION_START_STEP,
    STALL_SPEED_THRESHOLD,
    STALL_STEPS_THRESHOLD,
    STEP_LOAD_AT_STEP,
    TARGET_SPEED_RAD_S,
    TORQUE_DEADZONE,
    TORQUE_LIMIT_AT_ZERO,
    TORQUE_LIMIT_OMEGA_CAP_RAD_S,
    TORQUE_LIMIT_SLOPE,

)

TASK_PROMPT = {
    "task_description": f"""
Design a controller (a "governor") to maintain a wheel's rotation at the commanded target speed despite varying external loads.

- **Wheel**: The single circular body has undisclosed mass and visible radius {DEFAULT_WHEEL_RADIUS_M:g} m, and rotates about a fixed vertical axis through its center (revolute joint to the environment). The wheel body is subject to speed-dependent resistance. Infer inertia and additional resisting dynamics from data rather than assuming a simple first-order plant.
- **Motor**: Each step you request motor torque; delivered torque magnitude is **capped** each step. In the source environment, the torque limit at rest is {TORQUE_LIMIT_AT_ZERO:g} N·m, increasing by {TORQUE_LIMIT_SLOPE:g} N·m per rad/s until {TORQUE_LIMIT_OMEGA_CAP_RAD_S:g} rad/s, and the actuator **deadzone** is {TORQUE_DEADZONE:g} N·m. Changed cap/deadzone constraints are stated explicitly; other actuator dynamics remain latent.
- **Target Speed**: The commanded angular velocity **can change during the run**; call the API **each step** for the current setpoint. Only the **initial** segment speed is stated here: {TARGET_SPEED_RAD_S} rad/s—later setpoints must be read from the API.
- **Angular velocity readout**: Use the documented API each step as your feedback signal for control. Note that measurements are **delayed** relative to the true instantaneous state (the exact delay is not disclosed).
- **Time discretization**: Each simulation step is {TIME_STEP:.6f} s ({int(round(1.0 / TIME_STEP))} Hz physics).
- **Opposing dynamics**: Resisting torques and disturbances are **not** fully specified here; they may vary with speed (including stiction effects at very low speeds), time, and internal state. Use closed-loop control and feedback to maintain tracking. Additional sustained load may appear at an undisclosed step.

Design a control loop that:
1. Reads the sensed angular velocity and the time-varying target speed each step.
2. Applies motor torque to regulate speed and reject disturbances. A startup phase of {REGULATION_START_STEP} steps precedes the **regulation phase** in which mean speed error is scored (stall rules below still apply from step 0).
3. Avoids prolonged stall from the **first step onward** (not only during the regulation phase) and keeps mean tracking error within the stated threshold over the regulation phase.

- **Simulation length**: {MAX_STEPS} steps. Success requires a **full** run through all steps. Mean speed error is scored only on steps with step index \u2265 {REGULATION_START_STEP} (after startup); runs that end before that step index cannot succeed.
""",
    "success_criteria": f"""

1. **Speed Regulation**: Mean absolute deviation of the wheel's **true** instantaneous angular velocity from the commanded target during the regulation phase (after startup) must stay <= {MEAN_SPEED_ERROR_THRESHOLD} rad/s.
2. **No Stall**: From the start of the episode through the end, sustained **true** instantaneous angular speed below {STALL_SPEED_THRESHOLD} rad/s for {STALL_STEPS_THRESHOLD} or more consecutive steps counts as failure.

**Scoring vs. sensing**: Regulation and stall are judged on the **physical** wheel angular velocity each step. The documented angular-velocity API is your control feedback and may not match that physical value at the same simulation step. Optimize so the **actual** rotational state meets the criteria above.

- **Speed Regulation Threshold**: MEAN_SPEED_ERROR_THRESHOLD = {MEAN_SPEED_ERROR_THRESHOLD} rad/s.
- **Stall Speed Threshold**: STALL_SPEED_THRESHOLD = {STALL_SPEED_THRESHOLD} rad/s.
- **Stall Steps Threshold**: STALL_STEPS_THRESHOLD = {STALL_STEPS_THRESHOLD} steps.
- **Torque Deadzone**: TORQUE_DEADZONE = {TORQUE_DEADZONE:g} N·m; a non-standard run may differ.
- **APIs**: Use only the primitives documented below.
- **Sensing**: Use only the documented angular-velocity call for rotational speed feedback (do not derive speed from other object or joint state).
""",
    'primitives_api': API_INTRO + '\n' + '\n\n'.join(_api_data['C_06'].values()),

}
