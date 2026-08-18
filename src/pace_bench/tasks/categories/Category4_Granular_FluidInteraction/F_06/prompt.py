import os

import json

from pace_bench.tasks.primitives_api import API_INTRO

with open(os.path.join(os.path.dirname(__file__), '..', '..', 'primitives_api.json'), 'r') as f:
    _api_data = json.load(f)

if 'F_06' in _api_data and 'API_INTRO' in _api_data['F_06']:
    del _api_data['F_06']['API_INTRO']

TASK_PROMPT = {
    "task_description": """
Design a transport system to route fluid particles to a narrow target zone while avoiding environmental hazards.

- **Fluid**: A batch of 60 small fluid particles (radius 0.10 m each) released from a source container (x in [2, 6] m, y in [0, 1.5] m).
- **Target Zone**: x in [18, 22] m, y in [0, 1.5] m.
- **Time limit**: Simulation runs for 40 seconds (2400 steps at 60 fps).
- **Hazards (pits, loss zones)**: Particles entering these regions are lost. Pit 1: x=[13.5, 15.5] m, y=[0, 2] m. Pit 2: x=[16, 17.5] m, y=[0, 1.6] m. Pit 3: x=[11, 12.5] m, y=[0, 1.6] m.
- **Headwind**: Above y = 3.0 m a horizontal opposing force acts on particles.
- **Gravity well**: In the region x in [10, 14] m, y in [1.5, 3.5] m a downward force acts on particles.
- **Build Zone**: x=[6, 18] m, y=[0, 6] m. Component centers may lie up to 0.1 m outside these bounds.
- **Beam dimensions**: Each structure component (beam) must have width and height between 0.1 m and 1.2 m.
- **Control**: You can apply forces directly to particles using `apply_force_to_particle()`.

Design a system (structure and control) that:
1. Directs at least 90% of released fluid particles into the target zone.
2. Navigates particles safely over any pits or obstacles.
3. Operates within a per-step force budget of 12000 N and total structure mass limit.
""",
    "success_criteria": """

1. **Delivery Efficiency**: At least 90% of released particles reach the target zone.
2. **Resource Management**: Per-step force usage must not exceed 12000 N per step.

- **Mass Budget**: Total structure mass <= 380 kg.
- **Force Budget**: 12000 N per step.
- **Anchoring**: The structure must be anchored to the ground using at least one joint.
- **APIs**: Use only the primitives documented below.
""",
    'primitives_api': API_INTRO + '\n' + '\n\n'.join(_api_data['F_06'].values()),

}
