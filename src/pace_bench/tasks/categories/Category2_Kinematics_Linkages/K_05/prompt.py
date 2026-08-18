import os

import json

from pace_bench.tasks.primitives_api import API_INTRO

with open(os.path.join(os.path.dirname(__file__), '..', '..', 'primitives_api.json'), 'r') as f:
    _api_data = json.load(f)

if 'K_05' in _api_data and 'API_INTRO' in _api_data['K_05']:
    del _api_data['K_05']['API_INTRO']

task_data = _api_data['K_05']

if 'API_INTRO' in task_data:
    del task_data['API_INTRO']

TASK_PROMPT = {
    'task_description': """
Design a scissor lift mechanism that can lift objects vertically using motor rotation or linear forces.

- **Ground**: A flat horizontal surface at y=1.0m (friction coefficient varies).
- **Target Object**: A block (0.6 m × 0.4 m, width × height) resting at x=4.0m, y=1.8m. Its mass, friction, and center-of-mass placement are intentionally hidden.
- **Target Height**: Lift the object so its center reaches at least y=9.0m.
- **Build Zone**: x=[0, 8], y=[1, 12]. All structure components must be placed within this zone.
- **Ceiling**: None (no vertical obstacle).

- **Vertical Lift**: Object center reaches y >= 9.0m.
- **Sustain**: Object cumulatively maintained at or above target height for at least 3.0 seconds (vertical velocity must remain >= -0.4 m/s; sliding down does not count as held).
- **Mass Budget**: Total structure mass must be less than 60 kg.
- **Beam Dimensions**: 0.05 <= width, height <= 4.0 meters.
- **Joint Angle Limits**: Pivot joint angle limits (when used) are clamped to [-π, π] radians.
- **Slider Translation**: Prismatic (slider) joints have default translation range ±10 m along the axis if not specified.
- **Motor limits**: Default maximum torque for pivot (revolute) motors is 100 N·m and default maximum force for slider (prismatic) motors is 5000 N if not specified when calling `set_motor` or `set_slider_motor`.
- **Joint reaction limit**: Structural joints do not break under reaction force in the base environment.
- **Lifting threshold**: For failure detection, the object is considered "lifted" only when its center rises at least 0.5 m above its initial height (y=1.8 m).

1. **Design**: Create a scissor lift or telescoping mechanism.
2. **Control**: Use `set_motor` on pivot joints or `set_slider_motor` on prismatic joints to drive the lift.
""",
    'success_criteria': """

1. **Vertical Movement**: Object reaches y >= 9.0m.
2. **Sustain**: Cumulatively maintained at or above target height for >= 3.0 seconds (vertical velocity >= -0.4 m/s; sliding down disqualifies).
3. **Integrity**: Structure remains intact (no joint breaks).

- **Mass Budget**: < 60 kg.
- **Object Surface Friction**: Friction coefficient varies (affects how securely the payload is held during sustain).
- **APIs**: Use only the primitives documented below.
""",
    'primitives_api': API_INTRO + '\n\n' + '\n\n'.join(task_data.values()),

}
