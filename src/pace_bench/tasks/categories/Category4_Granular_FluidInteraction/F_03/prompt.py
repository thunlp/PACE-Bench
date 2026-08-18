import os

import json

from pace_bench.tasks.primitives_api import API_INTRO

with open(os.path.join(os.path.dirname(__file__), '..', '..', 'primitives_api.json'), 'r') as f:
    _api_data = json.load(f)

if 'F_03' in _api_data and 'API_INTRO' in _api_data['F_03']:
    del _api_data['F_03']['API_INTRO']

TASK_PROMPT = {
    "task_description": """
Design an excavator arm and scoop to move granular material over an obstacle.

- **Material**: 200 sand particles in a pit with x=[0.0, 5.0] m and y=[0.0, 2.5] m.
- **Obstacle**: A central wall at x=-1.0m. Use `has_central_wall()` to check environment state.
- **Target Hopper**: Located at x=-5.0m, y=3.0m. Particles count as deposited if their center lies in the hopper zone x=[-6.0, -4.0] m, y=[0.5, 5.0] m.
- **Build Zone**: Mechanism must be built in x=[-4.0, 2.0], y=[0.0, 5.0]. Base is anchored at x=-2.0 m, y=0.0 m (evaluator accepts any body within 0.5 m of this position).
- **Time Limit**: Complete the task within 40 seconds.

Design a mechanism that:
1. Scoops up granular material from the pit (x > 0).
2. Lifts and moves the material over the central wall (x = -1.0).
3. Deposits the material into the target hopper at x=-5.0.

- **Mass Budget**: Total structure mass <= 800 kg.
- **Beam Dimensions**: Each beam width and height must be between 0.1 m and 1.5 m.
- **Per-scoop capacity**: Maximum 999 particles can be retained and transported by the scoop per trip.
- **Kinematic Requirement**: The mechanism must have at least 2 degrees of freedom (Arm + Bucket), i.e. at least 2 revolute joints.
- **APIs**: Use only the primitives documented below.
""",
    "success_criteria": """
1. **Material Transfer**: At least 15 sand particles are deposited in the hopper zone (x=[-6.0, -4.0] m, y=[0.5, 5.0] m; center at x=-5.0, y=3.0).

2. **Integrity**: Mechanism remains intact throughout the operation.
""",
    'primitives_api': API_INTRO + '\n' + '\n\n'.join(_api_data['F_03'].values()),

}
