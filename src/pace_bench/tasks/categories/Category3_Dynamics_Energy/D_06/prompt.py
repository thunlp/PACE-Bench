import os

import json

from pace_bench.tasks.primitives_api import API_INTRO

with open(os.path.join(os.path.dirname(__file__), '..', '..', 'primitives_api.json'), 'r') as f:
    _api_data = json.load(f)

if 'D_06' in _api_data and 'API_INTRO' in _api_data['D_06']:
    del _api_data['D_06']['API_INTRO']

TASK_PROMPT = {
    "task_description": """
Design a structure to catch and stabilize seven heavy balls launched sequentially toward a target zone.

- **The Projectiles**: Seven balls spawn on the right at x=22.0 m with nominal radius 0.35 m. Initial center heights (m) are ball1 y=4.0, ball2 y=3.5, ball3 y=4.5, ball4 y=3.8, ball5 y=4.2, ball6 y=3.9, ball7 y=4.1. Ball 1 moves left immediately; balls 2–7 start at rest and receive horizontal velocity at the listed times (simulation time in seconds after reset): ball 2 at t=0.4 s, ball 3 at t=1.0 s, ball 4 at t=1.3 s, ball 5 at t=1.8 s, ball 6 at t=2.2 s, ball 7 at t=2.7 s. Nominal horizontal speeds at release (m/s) are ball1=-24, ball2=-26, ball3=-24, ball4=-28, ball5=-25, ball6=-26, ball7=-25. Mutated environments may change these times and speeds.
- **Projectile inertia**: Ball density is a latent material property and is not disclosed numerically; infer inertia from observed trajectories and impacts.
- **Projectile damping and contact**: Balls experience linear and angular damping, restitution, and sliding friction against surfaces; numerical values are fixed for a given environment and may differ under mutations—infer behavior from motion and feedback rather than assuming textbook defaults.
- **Ground surface**: The top of the horizontal ground strip is at **y = 0.5** m (the pit-failure check uses **y < 0.72** m). Ground–ball and ground–beam friction/restitution are environment parameters not duplicated here.
- **Environmental loads**: Oscillating lateral forces act on balls, scaled by each ball’s mass. Amplitude, period, and internal coupling factors are not disclosed numerically—treat lateral loading as environment-specific.
- **The Deflector**: A kinematic bar is present near the central corridor and moves vertically over time. Contact response is environment-tuned; infer interaction behavior from motion and feedback.
- **Left boundary**: A static wall blocks motion to the left of approximately x=6.95 m (just left of the build zone minimum when the build zone starts at x=7.0 m), so paths cannot bypass the build region on the left.
- **Forbidden Zones**: Beam centers must not be placed in five specific vertical zones (x in [8.5, 9.5], [7.35, 7.75], [7.78, 8.55], [10.0, 10.5], [7.18, 7.34]).
- **Sweeper Bands**: Four horizontal bands (y in [2.95, 3.55], [4.15, 4.75], [1.0, 1.5], [2.0, 2.5]) are forbidden for any beam centers.
- **Build Zone**: x=[7.0, 11.0] m, y=[0.5, 5.5] m. **Ground anchoring (required)**: At least one beam must be rigidly connected to the static environment with `add_joint(beam, None, anchor, 'rigid')` (unanchored designs are invalid at evaluation).
- **Catcher beams**: New beams use world defaults for damping unless changed with `set_damping`. Structure–terrain contact behavior follows environment settings.
- **Simulation budget**: The environment exposes `sandbox.MAX_STEPS` (default **10000** physics steps). If the run reaches that step cap without all success conditions, the episode ends in failure. Eval harnesses may pass a lower `max_steps`; treat the effective cap as the minimum of the runner limit and `MAX_STEPS`.

Design a catching structure that:
1. Absorbs the kinetic energy of all seven balls.
2. Keeps all balls within the target zone (x=[7, 11], y=[0.5, 5.5]) with a final speed < 0.35 m/s.
3. **Sequential rule (evaluation)**: For balls ordered as ball1 … ball7, when a ball’s center first crosses x < 7.4 m (approach to the catch region), every lower-index ball must already be “caught” (speed < 0.35 m/s inside the target box). Otherwise the run fails for order violation.
4. **Pit failure**: If any ball that is not yet caught has y < 0.72 m and speed > 1.0 m/s, the task fails immediately.

Beam center placement (build zone, forbidden x-bands, sweeper y-bands), beam count, mass budget, and **at least one rigid ground anchor joint** are checked once at the start of the simulation from the initial design.
""",
    "success_criteria": """

1. **Catch All**: All seven balls must be caught and stabilized within the target area.
2. **Order**: Same as the sequential rule in the task description (x < 7.4 m approach line; prior balls must already be caught).
3. **Integrity**: The structure must not break under the impact forces (see structural limits below).
4. **No pit failure**: No uncaught ball may have y < 0.72 m with speed > 1.0 m/s.

- **Beam Limit**: Maximum 9 beams.
- **Beam half-extents**: Each beam’s width and height passed to `add_beam` are clamped to [0.1, 3.0] m before creation.
- **Mass Budget**: Total structure mass must be strictly less than 10.0 kg.
- **Joint force limit**: In any single simulation step, joints fail if the reaction force magnitude reaches or exceeds 880 N (peak failure). Additionally, if the reaction force magnitude is strictly greater than 760 N for two consecutive simulation steps, the joint fails (fatigue). The structure must remain intact.
- **APIs**: Use only the primitives documented below.
""",
    'primitives_api': API_INTRO + '\n' + '\n\n'.join(_api_data['D_06'].values()),

}
