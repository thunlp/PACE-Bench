import os

import json

from pace_bench.tasks.primitives_api import API_INTRO

with open(os.path.join(os.path.dirname(__file__), '..', '..', 'primitives_api.json'), 'r') as f:
    _api_data = json.load(f)

if 'K_04' in _api_data and 'API_INTRO' in _api_data['K_04']:
    del _api_data['K_04']['API_INTRO']

task_data = _api_data['K_04']

if 'API_INTRO' in task_data:
    del task_data['API_INTRO']

TASK_PROMPT = {
    'task_description': """
Design a ground-based pusher vehicle that can move a heavy object across a high-friction surface.

- **Ground**: A horizontal surface at y=1.0m. Its non-visual material properties are intentionally hidden and must be inferred from motion.
- **Gravity**: The gravitational acceleration vector (its magnitude and direction affect body weight and drive effectiveness).
- **Heavy Object**: A rectangular block 1.0 m × 0.8 m (width × height) at x=8.0m. Its mass, material properties, damping, and center-of-mass placement are intentionally hidden.
- **Build Zone**: x=[0, 15], y=[1.5, 8]. All structure components must be placed within this zone.
- **Target**: Push the object to at least x=18.0m (10 meters forward from starting x).

- **Distance**: The object center reaches x >= 18.0m.
- **Motion Duration**: The evaluation window is 12.0 seconds (720 simulation steps at 60 FPS). The agent must push the object to the target distance within this window.
- **Mass Budget**: Total structure mass must be less than 40 kg.
- **Ground Friction**: The ground surface friction coefficient affects traction.
- **Object Friction**: The object's surface friction coefficient affects push dynamics and contact forces.
- **Beam Dimensions**: 0.05 <= width, height <= 3.0 meters.
- **Wheel Radius**: 0.05 <= radius <= 0.8 meters (for add_wheel).
- **Motor torque** *(Constraint)*: Maximum motor torque for pivot joints is 100 N·m (set_motor max_torque); exceeding this value will cause API rejection.
- **Pivot Joint Angle Limits**: Radians in [-π, π] when using limits on pivot joints.
- **Payload Support**: Object must remain on the platform; object center y below 0.5 m is failure.
- **Pusher Stability**: The pusher chassis must not tip over. Chassis tilt angle exceeding ±π/6 radians (~±30°) causes immediate failure.
- **Effective Push**: The pusher must maintain meaningful contact with the object. If the pusher chassis is moving forward at more than 0.5 m/s while the object velocity falls below 0.05 m/s for more than 200 simulation steps (~3.3 seconds), the evaluator treats this as failure to push effectively.

1. **Design**: Create a wheeled or sliding pusher vehicle.
2. **Control**: Use `set_motor` on pivot joints (wheels) or apply forces/torques to drive the vehicle.
""",
    'success_criteria': """

1. **Movement**: Object reaches x >= 18.0m.
2. **Locomotion**: Maintains active motion for >= 12.0 seconds.
3. **Stability**: Structure remains intact; chassis tilt must stay within ±π/6 radians (~±30°) and object center y must stay above 0.5 m.

- **Mass Budget**: < 40 kg.
- **APIs**: Use only the primitives documented below.
""",
    'primitives_api': API_INTRO + '\n\n' + '\n\n'.join(task_data.values()),

}
