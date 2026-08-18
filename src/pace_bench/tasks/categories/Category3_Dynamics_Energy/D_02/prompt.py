import os

import json

from pace_bench.tasks.primitives_api import API_INTRO

with open(os.path.join(os.path.dirname(__file__), '..', '..', 'primitives_api.json'), 'r') as f:
    _api_data = json.load(f)

if 'D_02' in _api_data and 'API_INTRO' in _api_data['D_02']:
    del _api_data['D_02']['API_INTRO']

TASK_PROMPT = {
    "task_description": """
Design a jumping mechanism to clear a pit and land on a distant platform by passing through multiple narrow slots.

- **Platforms**: A left platform (start) and a right platform (target) separated by a wide pit.
- **Barriers**: Three vertical barriers with narrow horizontal slots (gaps) are positioned in the pit at approximately x ≈ 17 m, x ≈ 19 m, and x ≈ 21 m. Each barrier has a lower red bar and an upper red bar; the trajectory must pass through the vertical gap between them. Each slot's rule applies when the jumper center's x is within 0.5 m of the slot center: **Slot 1** x in [16.5, 17.5] m; **Slot 2** x in [20.5, 21.5] m; **Slot 3** x in [18.5, 19.5] m. When in that x-range, the jumper must stay at least 0.05 m clear of the gap floor and ceiling with its edges (i.e. the jumper's bottom edge above floor+0.05 m and top edge below ceiling−0.05 m). With jumper height 0.6 m, the jumper center y must therefore lie in (floor+0.35, ceiling−0.35) for that slot. Slot vertical gaps (floor to ceiling, y in m): **Slot 1** (x ≈ 17 m): y in [13.2, 14.7]; **Slot 2** (x ≈ 21 m): y in [11.3, 13.3]; **Slot 3** (x ≈ 19 m): y in [12.4, 14.2].
- **Jumper**: The jumper body has width 0.8 m and height 0.6 m; it starts at position (5.0, 5.0) m (center). Its center must stay within each slot's gap when crossing (see slot x-ranges and 0.05 m clearance above).
- **Pit Failure**: Jumper center must remain at y ≥ 0 m; below y = 0 m is considered in the pit and results in failure.
- **Build Zone**: x in [1.5, 6.5] m, y in [2.5, 5.5] m. All beam centers must be within this zone.
- **Beam Dimensions**: Each beam width and height is clamped by the environment to [0.1, 4.0] m.
- **Goal**: Reach the right platform (x >= 26.0m) by jumping from the left platform.

Design a controller that:
1. Determines the optimal launch velocity (magnitude and direction) to jump over the pit.
2. Ensures the trajectory passes through the narrow gaps in all intermediate barriers.
3. Successfully lands on the right platform without falling into the pit or hitting the red sections of the barriers.
""",
    "success_criteria": """

1. **Target Reach**: Body reaches the right platform (x >= 26.0 m, y >= 1.0 m).
2. **Gap Clearance**: Trajectory successfully passes through all barrier slots without collision.

- **Mass Budget**: Total structure mass ≤ 180 kg.
- **Pit Safety**: Jumper center y must be ≥ 0 m (below 0 m = failure).
- **APIs**: Use only the primitives documented below.
""",
    'primitives_api': API_INTRO + '\n' + '\n\n'.join(_api_data['D_02'].values()),

}
