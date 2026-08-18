import os

import json

from pace_bench.tasks.primitives_api import API_INTRO

with open(os.path.join(os.path.dirname(__file__), '..', '..', 'primitives_api.json'), 'r') as f:
    _api_data = json.load(f)

if 'S_03' in _api_data and 'API_INTRO' in _api_data['S_03']:
    del _api_data['S_03']['API_INTRO']

TASK_PROMPT = {
    'task_description': """
Construct a horizontal cantilever structure anchored ONLY to the left wall at x=0.
The structure must extend horizontally to reach the target distance and support multiple heavy vertical payloads applied at different times (e.g., at t=5s and t=15s).

- **Wall**: A vertical static surface at x=0. You may use a maximum of 2 anchor points on this wall.
- **Forbidden Anchor Zones**: Wall anchors may be restricted to certain vertical segments (y range). In the source environment there are no restrictions.
- **Build Zone**: Structure must be built within x = [0, 50] m and y = [-20, 30] m. Beams outside this region violate design constraints.
- **Beam Limits**: Each beam's width and height are clamped to [0.1 m, 15.0 m] per dimension.
- **Payloads**: Payloads will be applied to your structure to test its load-bearing capacity and stability. Each payload has mass **500 kg** (applied at t=5s and t=15s). The first payload attaches to the rightmost (tip) body; the second attaches to the body at median horizontal position (mid-span). Payload application is either **static** (placed on the structure at the given times) or **dropped** from a specified height; in the source environment payloads are placed statically (no drop).
- **Minimum Tip Height**: The structure must not sag below y = -15.0 m (no part of the structure may go below this level); otherwise the task fails.
- **Reach Deflection Tolerance**: Under load, the tip may deflect; reach is still satisfied if tip x remains within 1.0 m of the target.
- **Internal Joint Limits**: Beam-to-beam joints fail if force exceeds **100,000,000 N** or torque exceeds **100,000,000 N·m**.
- **Wall Anchor Limits**: Wall anchors fail if force exceeds **100,000,000 N** or torque exceeds **100,000,000 N·m** (exceeding causes anchor failure). When segment-specific anchor strength applies, the vertical segment (y range) and force/torque multipliers are stated explicitly.
- **Atmosphere**: The environment exhibits physical properties that will test the structural integrity of your design.
- **Obstacles**: The build zone may contain static obstructions that your structure must navigate (originally none in the source environment).

Design a robust cantilever structure that can:
1. Extend horizontally from the wall to reach the target distance.
- **Goal**: Reach x >= 12.0m.
2. Support all applied payloads for 10 seconds each without collapsing or excessive sagging.
3. Manage internal stresses so that the wall anchors and structural joints remain intact.
- **Mass Limit**: < 15,000 kg.
""",
    'success_criteria': """

1. **Reach**: Structure reaches the required horizontal distance (Tip reaches x >= 12.0m).
2. **Load Bearing**: Successfully supports all payloads for the 10s test duration.
3. **Anchor Integrity**: All wall anchors and joints remain intact.
4. **Stability**: No part of the structure may go below the allowed vertical threshold (y >= -15.0 m).

- **Mass Budget**: < 15,000 kg.
- **Payload Mass**: 500 kg per applied load.
- **Payload application**: Static (placed on structure at the given times) in the source environment.
- **Payload Hold Duration**: Each payload must remain attached and the structure must not collapse for the full required hold duration (10.0 s per payload).
- **Anchor Limit**: Maximum 2 anchor points on the wall.
- **Forbidden Anchor Zones**: None in the source environment.
- **Regional anchor strength**: None in the source environment; when present, the vertical segment and force/torque multipliers are stated.
- **Build Zone**: x = [0, 50] m, y = [-20, 30] m.
- **Beam Limits**: Width and height in [0.1, 15.0] m per dimension.
- **Internal Joint Limits**: Max force 100,000,000 N; max torque 100,000,000 N·m (exceeding causes failure).
- **Wall Anchor Limits**: Max force 100,000,000 N; max torque 100,000,000 N·m (exceeding causes failure).
- **Reach Tolerance**: Under load, tip x may be up to 1.0 m short of target and still satisfy reach.
- **APIs**: Use only the primitives documented below.
""",
    'primitives_api': API_INTRO + '\n' + '\n\n'.join(_api_data['S_03'].values()),

}
