import os

import json

from pace_bench.tasks.primitives_api import API_INTRO

with open(os.path.join(os.path.dirname(__file__), '..', '..', 'primitives_api.json'), 'r') as f:
    _api_data = json.load(f)

if 'D_03' in _api_data and 'API_INTRO' in _api_data['D_03']:
    del _api_data['D_03']['API_INTRO']

TASK_PROMPT = {
    "task_description": """
Design an attachment for a cart to help it pass through multiple rotating gates at precise moments.

- **The Cart**: A cabin starts at x=4.0m, y=2.5m with an initial horizontal speed of 10.0m/s.
- **Rotating Gates**: Four gates with rotating rods are located at x=10.0, 11.5, 11.75, and 12.5. Each gate is only 'open' (passable) during a narrow angular window around vertical (half-widths in rad: gate 1 and 4 ≈ 0.56, gate 2 ≈ 1.50, gate 3 ≈ 0.60). Gate parameters: Gate 1 — rod length 0.9m, angular velocity 1.8 rad/s, initial angle 0.38 rad; Gate 2 — rod length 0.75m, angular velocity 0.58 rad/s, initial angle 0.63 rad; Gate 3 — rod length 0.7m, angular velocity 1.2 rad/s, initial angle 0.0 rad; Gate 4 — rod length 0.65m, angular velocity 0.9 rad/s, initial angle 0.4 rad.
- **Mud zone**: x=[5.5, 7.5] m; this region may apply latent velocity-dependent resistance whose coefficient is not disclosed.
- **First impulse zone**: x=[8.0, 9.0] m; entering this region may apply a latent one-time impulse.
- **Second impulse zone**: x=[10.5, 11.0] m; entering this region may apply a latent one-time impulse.
- **Decel zone**: x=[9.5, 11.0] m; this region may apply latent velocity-dependent resistance whose coefficient is not disclosed.
- **Brake zone**: x=[12.0, 15.0] m; this region may apply latent velocity-dependent resistance for final speed tuning.
- **Speed trap**: When the cart center first crosses x=9 m, its speed must be at least 2.8 m/s or the run fails.
- **Checkpoint at x=11**: When the cart center first crosses x=11 m, its speed must be in the band [1.1, 2.7] m/s or the run fails.
- **Build Zone**: x=[4.8, 9.0] m, y=[2.0, 3.2] m. All beams must be attached to the cart cabin.
- **Success Criteria**: The cart must reach the target zone (x >= 11.75m) with a final speed between 0.45m/s and 2.6m/s, without colliding with any rotating gate encountered before success. The fourth gate at x=12.5m lies beyond the success line and is not required once success has been recorded.

Design an attachment that:
1. Uses a specific number of beams (between 4 and 5) to adjust the cart's mass and momentum.
2. Carefully tunes the cart's arrival time at each gate to match the 'open' phase.
3. Successfully passes the rotating gates and reaches the target distance.
""",
    "success_criteria": """

1. **Gate Clearance**: Do not collide with any rotating gate encountered before reaching the target; the gate at x=12.5m is beyond the success line.
2. **Speed trap**: When first crossing x=9 m, cart speed must be >= 2.8 m/s.
3. **Checkpoint at x=11**: When first crossing x=11 m, cart speed must be in [1.1, 2.7] m/s.
4. **Target**: Reach x >= 11.75m with final speed in [0.45, 2.6] m/s.

- **Beam Count**: Exactly 4 or 5 beams must be used.
- **Beam dimensions**: Each beam width and height must be in [0.08, 2.0] m (min 0.08 m, max 2.0 m per dimension).
- **Mass Budget**: Total structure mass < 14.0 kg.
- **APIs**: Use only the primitives documented below.
""",
    'primitives_api': API_INTRO + '\n' + '\n\n'.join(_api_data['D_03'].values()),

}
