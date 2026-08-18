import os

import json

from pace_bench.tasks.primitives_api import API_INTRO

with open(os.path.join(os.path.dirname(__file__), '..', '..', 'primitives_api.json'), 'r') as f:
    _api_data = json.load(f)

if 'D_05' in _api_data and 'API_INTRO' in _api_data['D_05']:
    del _api_data['D_05']['API_INTRO']

TASK_PROMPT = {
    "task_description": """
Design a hammer mechanism to break a hard shell located behind a protective barrier.

- **The Shell**: A breakable target at x=16.0m, y=2.6m. It requires a large instantaneous force to break.
- **The Slot Barrier**: A wall at x=15.0m with a narrow vertical gap (y from 1.85 m to 3.35 m). Your hammer head must pass through this gap.
- **The Oscillating Bar**: A horizontal bar inside the slot gap that moves up and down. You must time your swing to pass when the bar is away.
- **Obstacles**: A swinging pendulum pivoted at x=7.0m, y=4.0m with a rod length of 3.5m may block your swing.
- **Build Zone**: x=[2.0, 12.0] m, y=[2.0, 8.0] m. You must anchor your mechanism to the ground.

Design a hammer that:
1. Is anchored to the ground and consists of a handle and a heavy head.
2. Achieves a trajectory that passes through the narrow slot gap without hitting the walls or the oscillating bar.
3. Delivers a strike with enough kinetic energy and force (≥ 5000 N) to break the shell.
""",
    "success_criteria": """

1. **Breakage**: The shell at x=16.0m must be broken by the impact.
2. **Precision**: The hammer must not collide with the slot walls, the oscillating bar, or the pendulum.

- **Mass Budget**: Total structure mass < 70.0 kg.
- **APIs**: Use only the primitives documented below.
""",
    'primitives_api': API_INTRO + '\n' + '\n\n'.join(_api_data['D_05'].values()),

}
