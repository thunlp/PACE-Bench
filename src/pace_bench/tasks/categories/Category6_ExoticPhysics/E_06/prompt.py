import os

import json

from pace_bench.tasks.primitives_api import API_INTRO

with open(os.path.join(os.path.dirname(__file__), '..', '..', 'primitives_api.json'), 'r') as f:
    _api_data = json.load(f)

if 'E_06' in _api_data and 'API_INTRO' in _api_data['E_06']:
    del _api_data['E_06']['API_INTRO']

TASK_PROMPT = {
    "task_description": """
Design a robust cantilever structure that remains intact under distance-scaled stochastic and moment-dominant impulsive loads.

- **Support**: Ground anchors are allowed ONLY in the left support zone (x in [5.0, 6.5]m). Exactly one ground anchor is required, and its y-coordinate must be the ground surface y=1.0 m.
- **Cantilever**: The structure must extend from the support zone to the right.
- **Loads**: The structure is subject to continuous noise and periodic "storms". Excitation intensity increases with distance from the support.
- **Momentum**: Periodic coherent pulses create significant overturning moments, especially at the structure's tip.
- **Build Zone**: Every beam center must be placed in x=[5.0, 15.0] m, y=[1.5, 8.0] m. Beam centers in the **forbidden zone** x in [9.7, 10.3] m are rejected. Maximum 48 beams and 75 joints. Beam width and height must each be in [0.1, 4.0] m; density must be positive.
- **Structural limits**: Joints fail above 280800 N reaction force or 414000 N·m reaction torque; cumulative damage fails at 100 pts. A beam is destroyed if angular velocity exceeds 2.2 rad/s for 10 consecutive steps.
- **Goal**: Maintain structural integrity (avoid joint failure or beam destruction) for the duration of the test while spanning the required horizontal distance.

Design a cantilever that:
1. Anchors to the ground at exactly one point within x=[5.0, 6.5].
2. Spans from the support to at least x=13.0m and reaches a height of y=5.0m.
3. Withstands distance-scaled vibration and impulsive moments without any part of the structure breaking or being destroyed.
""",
    "success_criteria": """

1. **Integrity**: The structure remains entirely intact; no joints break and no beams are destroyed.
2. **Span**: Structure reaches at least x <= 7.0m (near support) and x >= 13.0m (tip), and a height of y >= 5.0m.
3. **Mass**: Initial structure mass does not exceed the live mass limit (120 kg).

- **Anchor Limit**: Exactly 1 ground anchor, with x in [5.0, 6.5] m and y=1.0 m.
- **Mass Budget**: Total structure mass <= 120 kg.
- **Beams**: At most 48 beams; each beam width and height in [0.1, 4.0] m.
- **Joints**: At most 75 joints; joint failure at force > 280800 N or torque > 414000 N·m; damage failure at 100 pts. A beam is destroyed if angular velocity exceeds 2.2 rad/s for 10 consecutive steps.
- **Forbidden zone**: No beam centers in x in [9.7, 10.3] m.
- **APIs**: Use only the primitives documented below.
""",
    'primitives_api': API_INTRO + '\n' + '\n\n'.join(_api_data['E_06'].values()),

}
