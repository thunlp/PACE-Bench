import json
from pathlib import Path

from pace_bench.tasks.primitives_api import API_INTRO

_API_PATH = Path(__file__).resolve().parents[2] / 'primitives_api.json'
with _API_PATH.open(encoding='utf-8') as f:
    _api_data = json.load(f)

if 'S_01' in _api_data and 'API_INTRO' in _api_data['S_01']:
    del _api_data['S_01']['API_INTRO']

TASK_PROMPT = {
    'task_description': """
Design a static bridge to connect two cliffs. A vehicle will spawn on the left cliff and attempt to cross to the right.

- **Cliffs**: Two static platforms separated by a wide gap.
- **Left Cliff**: Ends at x=10.0m, y=10.0m.
- **Right Cliff**: Starts at x=25.0m, y=10.0m.
- **Vehicle**: A vehicle (mass: 2000.0 kg) will spawn on the left cliff at x=5.0 m, y=10.5 m with an initial rightward velocity of 5.0 m/s. Vehicle footprint: wheelbase 3.0 m, chassis 2.0 m × 0.5 m, wheel radius 0.4 m.
- **Fail Zone**: A water surface exists at y=0m. The task fails if the center of the vehicle chassis or the center of any structural component reaches y ≤ 0.5 m at an evaluation sample.
- **Target**: The vehicle must fully cross the gap and reach at least x=30.0m on the right side.

Design a stable bridge structure that can:
1. Span the gap and connect the two cliffs.
2. Support the dynamic load of the heavy vehicle as it crosses.
3. Provide a continuous and smooth deck surface for the vehicle's wheels.
4. Maintain structural integrity under load. Joints have strength limits; excessive force or torque will cause them to break.

- **Mass Budget**: Total structure mass must be at most 2000 kg.
- **Build Zone**: Structure must be built within x=[10, 30], y=[5, 15] (the upper x-bound is the target position so the deck can reach the goal).
- **Beam Dimensions**: 0.1 <= width, height <= 10.0 meters.
- **Joint Strength**: Maximum linear force for structural joints is 80.0; maximum torque is 300.0.
- **Anchor Strength**: Maximum linear force for structural cliff anchors is 100.0; maximum torque is 500.0.
- **Atmospheric Wind**: In some stages, constant lateral and/or vertical wind forces may act on all bodies — the uniform suffix warns of potential changes.
- **Flip Condition**: The vehicle chassis must not tip beyond ±90° from its upright orientation at any evaluation sample.
""",
    'success_criteria': """

1. **Passage**: Vehicle reaches x >= 30.0m.
2. **Integrity**: No structural breaks (all joints must remain intact during the crossing).
3. **Smoothness**: The vehicle's sampled vertical acceleration (change in vertical velocity divided by elapsed time between evaluator samples) must not exceed 19.6 m/s² (2.0g).
4. **Stability**: The vehicle's angular velocity must not exceed 2.0 rad/s for 5 consecutive evaluator samples after simulation step 200, and net airborne rotation must not exceed 180 degrees. The vehicle is considered **airborne** when its center is more than 0.5 m above the cliff top (y > 10.5 m); the 180° limit applies to rotation accumulated only while in that state.
5. **No Flipping**: At every evaluation sample, the vehicle must not tip beyond ±90° from upright.

- **Mass Budget**: <= 2000 kg.
- **Joint Strength**: Maximum linear force for structural joints is 80.0; maximum torque is 300.0.
- **Anchor Strength**: Maximum linear force for structural cliff anchors is 100.0; maximum torque is 500.0.
- **APIs**: Use only the primitives documented below.
""",
    'primitives_api': API_INTRO + '\n' + '\n\n'.join(_api_data['S_01'].values()),

}
