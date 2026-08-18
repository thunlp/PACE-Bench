import os

import json

from pace_bench.tasks.primitives_api import API_INTRO

with open(
    os.path.join(os.path.dirname(__file__), '..', '..', 'primitives_api.json'),
    'r',
    encoding='utf-8',
) as f:
    _api_data = json.load(f)

if 'E_02' in _api_data and 'API_INTRO' in _api_data['E_02']:
    del _api_data['E_02']['API_INTRO']

TASK_PROMPT = {
    "task_description": """
Design a controller for a craft navigating through a complex "thick air" environment.

- **Craft**: A vehicle subject to intense drag, momentum drain, and thermal constraints. The craft starts at position (x=8.0 m, y=2.0 m).
- **Terrain**: The path contains narrow gate structures with these visible opening regions:
  - **Gate 1**: x in [12.0, 14.0] m, y in [1.0, 2.8] m.
  - **Gate 2**: x in [22.0, 24.0] m, y in [1.8, 3.0] m.
- **Anomalies**:
  - **High-Drag Atmosphere**: Intense air resistance affects terminal velocity.
  - **Linear Damping**: The craft's linear damping affects its terminal velocity.
  - **Angular Damping**: The craft's angular damping affects how quickly rotational motion subsides.
  - **Localized Momentum Transfer**: Some regions may alter the craft's motion response.
  - **Localized Body Forces**: Some regions may apply additional forces.
  - **Time-Varying Disturbances**: Local atmospheric forces may vary during the run.
- **Goal**: Reach a target coordinate (x in [28.0, 32.0], y in [2.0, 5.0]) while managing internal heat.
- **Heat**: Applying thrust increases craft heat. The overheat limit is 72000 N·s; exceeding it causes mission failure.
- **Time budget**: You have at most 10000 simulation steps to reach the target.

Design a control loop that:
1. Navigates the craft toward the target position through the visible terrain.
2. Responds to local physical anomalies as they are encountered.
3. Monitors heat levels and manages thrust to avoid reaching the overheat limit.
""",
    "success_criteria": """

1. **Target Reach**: Craft center enters the target zone.
2. **Thermal Safety**: Craft heat stays below the overheat limit (72000 N·s).

- **APIs**: Use only the primitives documented below.
""",
    'primitives_api': API_INTRO + '\n' + '\n\n'.join(_api_data['E_02'].values()),

}
