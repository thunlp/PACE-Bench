import os

import json

from pace_bench.tasks.primitives_api import API_INTRO

from pace_bench.tasks.categories.Category5_Cybernetics_Control.C_04.environment import (
    ACTIVATION_X_MAX,
    ACTIVATION_X_MIN,
    AGENT_MASS,
    BACKWARD_FX_THRESHOLD,
    BACKWARD_SPEED_MAX,
    EXIT_X_MIN,
    EXIT_Y_MAX,
    EXIT_Y_MIN,
    FPS,
    HOLD_STEPS,
    MAX_STEPS,
    STRUCTURAL_IMPULSE_SCALE_K,
    WHISKER_RANGE,

)

_structural_impulse_ns = STRUCTURAL_IMPULSE_SCALE_K * AGENT_MASS

if abs(_structural_impulse_ns - round(_structural_impulse_ns)) < 1e-6:
    _structural_impulse_ns_str = f"{int(round(_structural_impulse_ns))}"

else:
    _structural_impulse_ns_str = f"{_structural_impulse_ns:.1f}"

_unlock_qual_fx_example = int(round(BACKWARD_FX_THRESHOLD - 1.0))

with open(os.path.join(os.path.dirname(__file__), '..', '..', 'primitives_api.json'), 'r') as f:
    _api_data = json.load(f)

if 'C_04' in _api_data and 'API_INTRO' in _api_data['C_04']:
    del _api_data['C_04']['API_INTRO']

TASK_PROMPT = {
    "task_description": f"""
Design a controller for a whisker-equipped robot to unlock and escape a narrow maze.

- **Agent**: Undisclosed mass, disk radius 0.2 m, initial position (2.0, 1.5) m.
- **Agent dynamics**: Linear damping (`linearDamping`) applies to translation; the numeric value is **not stated in this document**—infer from coast-down or runtime inspection. **fixedRotation** is enabled (translation only).
- **Contact dynamics**: Wall–agent friction and restitution are set on **Box2D** fixtures; numeric coefficients are **not stated in this document**—infer from motion and impacts (see **Possible Environment Variations** when present on mutated runs).
- **Simulation timing**: `agent_action` runs once per fixed **1/{FPS} s** simulation step.
- **Time-varying horizontal forcing (baseline source)**: In addition to other channels, the simulator may apply oscillatory or otherwise time-dependent horizontal forces (e.g. wind-like terms). **Exact amplitudes, frequencies, and phase rules are not stated in this document**—infer from motion and feedback (see **Possible Environment Variations** when present on mutated runs).
- **Environmental horizontal forcing**: Besides your commanded forces, the world can apply additional horizontal forces (constant back-current, height-dependent shear, one-way assist, and time-varying terms when active). Their non-constraint numeric parameters are not stated; infer their net effect from motion and feedback.
- **Height-dependent horizontal shear (if active)**: Uses vertical position relative to an internal reference height; neither that reference nor the shear gradient magnitude is numerically stated here—infer from motion and feedback.
- **Whiskers**: Three sensors (forward +x, up +y, down -y), each range {WHISKER_RANGE} m.
- **State interfaces**: `get_agent_position()` returns the **reported** pose used for exit and unlock evaluation; `get_agent_velocity()` returns instantaneous physical velocity.
- **Passage**: Maze bounds x in [0, 20] m, y in [0, 3] m.
- **Maze outer shell (indices 0–3; lower-left x, y, width, height in m)**: floor (0.0, 0.0, 20.0, 0.5); ceiling (0.0, 2.5, 20.0, 0.5); left wall (0.0, 0.0, 0.5, 3.0); right wall (20.0, 0.0, 0.5, 3.0).
- **Maze walls (indices 4-6; lower-left x, y, width, height in m)**: (5.0, 0.0, 0.2, 1.0); (9.0, 1.8, 0.2, 1.2); (14.0, 1.8, 0.2, 1.2).
- **Whisker blind band along x**: Its numeric spatial extent is not stated; infer altered sensor behavior through interaction. Suppression, when present, follows **physical** body x because whisker raycasts use true pose.
- **Control lag**: Its numeric duration is not stated; compare requested commands with the force currently evaluated by the unlock condition.
- **One-way rightward assist**: An additional rightward force can become active beyond an undisclosed reported-x threshold; its magnitude and threshold are not stated.
- **Structural impulse limit**: Failure occurs when collision normal impulse exceeds {_structural_impulse_ns_str} N·s.
- **Goal**: Reach the exit zone at the end of the passage.
- **Unlock condition**: While locked, a repelling force field acts in an undisclosed reported-x band with an undisclosed magnitude. To unlock: **reported** position x in [{ACTIVATION_X_MIN:.1f}, {ACTIVATION_X_MAX:.1f}] m with **commanded** horizontal Fx (after control lag) **strictly less than {BACKWARD_FX_THRESHOLD:.1f} N** (e.g. {_unlock_qual_fx_example} N qualifies; {BACKWARD_FX_THRESHOLD:.1f} N does not), and **true** linear speed from **physical** velocity **< {BACKWARD_SPEED_MAX:.1f} m/s**, for at least **{HOLD_STEPS}** consecutive steps.
- **Exit zone**: x >= {EXIT_X_MIN:.1f} m, y in [{EXIT_Y_MIN:.1f}, {EXIT_Y_MAX:.1f}] m; after **unlock**, hold there for at least **{HOLD_STEPS}** consecutive steps using **reported** position (before unlock, time in the exit zone does **not** count toward this hold).
- **Time limit**: At most {MAX_STEPS:,} simulation steps.

Design a control loop that:
1. Uses whisker readings to navigate the winding passage.
2. Performs the unlock behavior in the activation zone.
3. Reaches the exit zone **after unlocking** and holds for at least **{HOLD_STEPS}** consecutive steps (hold counts only after unlock).
""",
    "success_criteria": f"""

1. **Unlock & Reach**: Unlock and reach x >= {EXIT_X_MIN:.1f} m, y in [{EXIT_Y_MIN:.1f}, {EXIT_Y_MAX:.1f}] m.
2. **Hold**: After unlock, at least **{HOLD_STEPS}** consecutive steps in the exit zone using **reported** position; before unlock, exit-zone occupancy does **not** count toward this hold.
3. **Survival**: Stay below the structural impulse limit: **{_structural_impulse_ns_str} N·s** at baseline.

- **Unlock Speed**: True linear speed < {BACKWARD_SPEED_MAX:.1f} m/s.
- **APIs**: Use only the primitives documented below.
""",
    'primitives_api': API_INTRO + '\n' + '\n\n'.join(_api_data['C_04'].values()),

}
