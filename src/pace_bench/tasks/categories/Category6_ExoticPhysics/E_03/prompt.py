import os

import json

from pace_bench.tasks.primitives_api import API_INTRO

with open(os.path.join(os.path.dirname(__file__), '..', '..', 'primitives_api.json'), 'r') as f:
    _api_data = json.load(f)

if 'E_03' in _api_data and 'API_INTRO' in _api_data['E_03']:
    del _api_data['E_03']['API_INTRO']

TASK_PROMPT = {
    "task_description": """
Design a controller for a sled to navigate across a level environment and pass through checkpoints.

- **Sled**: A 1.0 m × 0.5 m sliding body whose center starts at (x=8.0 m, y=2.0 m).
- **Checkpoints** (must be passed in order):
  - **First checkpoint (Alpha)**: Sled center must enter the zone x in [17.5, 19.0] m, y in [3.8, 4.5] m.
  - **Second checkpoint (Beta)**: Sled center must enter the zone x in [23.0, 24.5] m, y in [2.5, 3.2] m.
- **Environment**: The ground is level. Local, undisclosed physical anomalies can alter the sled's motion or the effect of a commanded thrust. Their behavior must be inferred from measured run feedback.
- **Final target**: After both checkpoints, the sled center must enter the zone x in [28, 32] m, y in [2.2, 2.8] m.
- **Control timing**: `agent_action` is called once per fixed 1/60 s simulation step. `apply_thrust` sets the force command for that step; no separate actuator force cap is imposed by the public API.
- **Time limit**: The task must be completed within 10,000 simulation steps; the run terminates after this limit and failure is declared if the objective is not met by then.

Design a control loop that:
1. Directs the sled toward the next checkpoint.
2. Manages speed and orientation to successfully pass through checkpoints.
3. Uses observations and run feedback to adapt to the local dynamics.
""",
    "success_criteria": """

1. **Checkpoint Completion**: Sled passes through both required checkpoints in order (Alpha, then Beta).
2. **Target Reach**: Sled center enters the final target zone (x in [28, 32], y in [2.2, 2.8]).
3. **Efficiency**: Reaches the final target within 10,000 simulation steps.

- **APIs**: Use only the primitives documented below.
""",
    'primitives_api': API_INTRO + '\n' + '\n\n'.join(_api_data['E_03'].values()),

}
