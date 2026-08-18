import os

import json

from pace_bench.tasks.primitives_api import API_INTRO

with open(os.path.join(os.path.dirname(__file__), '..', '..', 'primitives_api.json'), 'r') as f:
    _api_data = json.load(f)

if 'K_01' in _api_data and 'API_INTRO' in _api_data['K_01']:
    del _api_data['K_01']['API_INTRO']

task_data = _api_data['K_01']

if 'API_INTRO' in task_data:
    del task_data['API_INTRO']

TASK_PROMPT = {
    'task_description': """
Design a 2D side-view walker that moves forward using motor-driven joints.

- **Ground**: A flat horizontal surface at y=1.0m.
- **Ground friction**: Ground traction is a latent contact parameter; infer it from observed motion rather than expecting a numeric coefficient in the prompt.
- **Gravitational acceleration**: The source environment uses an Earth-like downward field. Anomalous environments may differ; infer its effect from observed motion.
- **Build Zone**: x=[0, 50], y=[2, 10]. All structure components must be placed within this zone. During motion, the torso must remain within x=[0, 50] and y ≥ 1.2 and y ≤ 10.
- **Starting Position**: Walker components should be centered around x=10m, y=2.0m (within the build zone).
- **Target**: Move the walker's torso to at least x=25.0m (15 meters forward from starting x).

- **Stability**: The torso (main body) must always stay at or above y=1.2m (y >= 1.2m). If the torso falls below y=1.2m, the task fails. The evaluator treats the first body you create as the torso; create the torso first so that stability is measured correctly.
- **Motion**: The walker must maintain forward motion for at least 15.0 seconds.
- **Mass Budget**: Total structure mass must not exceed the allowed budget (≤ limit). Query `sandbox.get_structure_mass_limit()` to obtain the exact numerical limit for the current environment (default: 100 kg).
- **Build Zone**: All components must be placed within x=[0, 50], y=[2, 10]; during motion the torso must stay within x=[0, 50] and y ≥ 1.2 and y ≤ 10.
- **Beam Dimensions**: 0.05 <= width, height <= 5.0 meters.
- **Wheel Radius** (if used): 0.05 <= radius <= 0.8 meters.
- **Ground Friction**: See the source-environment description above. Mutated values remain hidden and must be inferred from interaction.
- **Body Friction Cap**: `sandbox.MAX_BODY_FRICTION` limits the maximum friction coefficient usable via `set_material_properties`. The default friction cap is 1.0, some mutated stages lower this cap, silently clamping your friction values to the cap. Never set friction above the current cap value.
- **Gravity**: The gravity vector is latent and must be inferred from motion.
- **Linear Damping**: The linear velocity damping coefficient of dynamic bodies, which may differ in mutated environments.
- **Angular Damping**: The angular velocity damping coefficient of dynamic bodies, which may differ in mutated environments.
- **Motor Torque**: The default maximum torque for a motor joint via `set_motor` is 100 N·m. If you do not explicitly specify `max_torque`, this default applies.
- **Pivot Joint Angle Range**: By default, pivot (revolute) joints allow rotation in the range -π to π radians (full circle); mutated environments may impose a narrower default range. If you do not explicitly specify `lower_limit` and `upper_limit` in `add_joint`, the environment's default limits apply (full ±π radians in the initial environment).

1. **Design**: Create a walker structure (e.g., bipedal, quadrupedal, or using rotating linkages).
2. **Control**: Use `set_motor` on pivot joints in `agent_action` to drive the walker forward.
""",
    'success_criteria': """

1. **Movement**: Reaches x >= 25.0m.
2. **Stability**: Torso y >= 1.2m at all times.
3. **Locomotion**: Maintains active motion for >= 15.0 seconds.

- **Mass Budget**: ≤ sandbox.get_structure_mass_limit() (query for the exact numerical limit; default: 100 kg).
- **APIs**: Use only the primitives documented below.
""",
    'primitives_api': API_INTRO + '\n\n' + '\n\n'.join(task_data.values()),

}
