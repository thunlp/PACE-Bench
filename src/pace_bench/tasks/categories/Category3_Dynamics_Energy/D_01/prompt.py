import os

import json

from pace_bench.tasks.primitives_api import API_INTRO

with open(os.path.join(os.path.dirname(__file__), '..', '..', 'primitives_api.json'), 'r') as f:
    _api_data = json.load(f)

if 'D_01' in _api_data and 'API_INTRO' in _api_data['D_01']:
    del _api_data['D_01']['API_INTRO']

TASK_PROMPT = {
    "task_description": """
You need to design a launcher that propels a projectile to hit a distant target.

- **Ground**: Flat surface at y=0 to y=1 m. Non-visual material properties are intentionally hidden.
- **Build Zone**: x=[5, 15] m, y=[1.5, 8] m. All beam centers must lie inside this zone. (Evaluated: beams whose centers are outside this bounding box will cause immediate design failure.)
- **Projectile**: A ball of radius 0.25 m starts at rest at position (10, 3) m. Its non-visual inertial and damping properties are intentionally hidden.
- **Target Zone**: x from 40 m to 45 m, and y from 2 m to 5 m. Success requires the projectile center to be inside this rectangle.

Design a launcher that:
1. Uses levers, whip-like motion, and/or spring energy storage to propel the projectile.
2. Launches the projectile so that it reaches and hits the target zone.
3. Stays within the build zone and material budget.
""",
    "success_criteria": """

1. **Hit**: Projectile center must lie inside the red target zone (x in [40, 45] m, y in [2, 5] m).
2. **No early failure**: Projectile must not be destroyed or leave the simulation bounds (x in [-10, 60] m, y ≥ -5 m).

- **Mass Budget**: Total structure mass must not exceed 500.0 kg (enforced strictly — exceeding this causes immediate design failure).
- **Build Zone**: All beam centers must lie within x=[5, 15] m and y=[1.5, 8] m (enforced strictly — any component center outside this box causes immediate design failure).
- **Beam dimensions**: Each beam width and height must be in [0.1, 5.0] m (enforced by the environment).
- **Spring stiffness**: Spring stiffness must be in [10, 3000] N/m (enforced by the environment).
- **APIs**: Use only the primitives documented below.
""",
    'primitives_api': API_INTRO + '\n' + '\n\n'.join(_api_data['D_01'].values()),

}
