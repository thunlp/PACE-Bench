import os

import json

from pace_bench.tasks.primitives_api import API_INTRO

with open(os.path.join(os.path.dirname(__file__), '..', '..', 'primitives_api.json'), 'r') as f:
    _api_data = json.load(f)

if 'K_06' in _api_data and 'API_INTRO' in _api_data['K_06']:
    del _api_data['K_06']['API_INTRO']

task_data = _api_data['K_06']

if 'API_INTRO' in task_data:
    del task_data['API_INTRO']

TASK_PROMPT = {
    'task_description': """
Design a wiper mechanism that can clean all particles from a glass surface using motor rotation.

- **Glass Surface**: Flat surface at y=2.0m, length 12m (x from 0 to 12).
- **Particles**: 45 small particles are randomly distributed on the glass.
- **Particle Removal**: A particle is considered "removed" when pushed off the glass surface boundaries. A particle counts as **remaining** on the glass when 0.5 ≤ x ≤ 11.5 m and |y − 2.0| < 0.5 m; outside this region it is **removed**.
- **Build Zone**: x=[0, 12], y=[2, 10]. All structure components must be placed within this zone.
- **Hidden dynamics**: Glass and particle material properties, particle mass, and gravity are intentionally hidden and must be inferred from observed motion.
- **Wiper–Glass**: The wiper does NOT collide with the glass surface itself; it only collides with the particles.

- **Cleaning**: At least 80% of particles must be removed (residual <= 20%).
- **Motion**: The wiper must maintain active movement for at least 8.0 seconds.
- **Mass Budget**: Total structure mass must be less than 15 kg.
- **Beam Dimensions**: 0.05 <= width, height <= 2.0 meters.
- **Pivot joint angle limits**: ±π radians (full rotation) for revolute/pivot joints.
- **Motor torque**: No environment cap (solver may request up to API limits).

1. **Anchor Base**: Use `weld_to_glass(body, anchor_point)` to fix your wiper's base relative to the glass surface.
2. **Sweep**: Design a sweeping mechanism to cover the entire width of the glass.
3. **Control**: Use `set_motor` on pivot joints in `agent_action` to drive the sweeping motion.
""",
    'success_criteria': """

1. **Cleaning**: At least 80% of particles removed (residual <= 20%).
2. **Locomotion**: Sustained sweeping motion for >= 8.0 seconds.
3. **Stability**: Structure remains within build zone and mass limits.

- **Mass Budget**: < 15 kg.
- **APIs**: Use only the primitives documented below.
""",
    'primitives_api': API_INTRO + '\n\n' + '\n\n'.join(task_data.values()),

}
