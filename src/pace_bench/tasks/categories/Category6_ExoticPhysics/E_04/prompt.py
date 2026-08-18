import os

import json

from pace_bench.tasks.primitives_api import API_INTRO

with open(
    os.path.join(os.path.dirname(__file__), '..', '..', 'primitives_api.json'),
    'r',
    encoding='utf-8',
) as f:
    _api_data = json.load(f)

if 'E_04' in _api_data and 'API_INTRO' in _api_data['E_04']:
    del _api_data['E_04']['API_INTRO']

TASK_PROMPT = {
    "task_description": """
Design a complex structure that remains intact under sinusoidally varying mass and environmental vibration.

- **Wind Pressure**: The environment may apply lateral wind pressure.
- **Mass Variation**: Every beam's mass varies over time according to sinusoidal frequency components; the phase of mass oscillation varies with beam position along the structure.
- **Base Excitation**: The ground support oscillates vertically and horizontally in an elliptical pattern.
- **Fatigue**: Joint strength may decay over time according to latent environment-specific dynamics.
- **Joint Limits (nominal)**: Joints fail if reaction force exceeds 6 N or reaction torque exceeds 10 N·m (before fatigue decay).
- **Beam Size**: Each beam's width and height are independently bounded: width and height each between 0.1 m and 4.0 m.
- **Build Zone**: x in [5.0, 15.0] m, y in [1.5, 8.0] m.
- **Simulation Duration**: The run lasts for 12000 simulation steps. Success is evaluated at the end of the run; the structure must remain intact for the entire simulation.
- **Goal**: Maintain structural integrity until the end of the simulation.

Design a structure that:
1. Spans from x=6.0m to x=14.0m.
2. Uses at least 5 beams and 6 joints.
3. Must include at least one pivot (revolute) joint.
4. Withstands the varying inertial loads and base vibration without breaking any joints.
""",
    "success_criteria": """

1. **Integrity**: All joints remain intact throughout the simulation.
2. **Simulation Length**: Structure must remain intact for the full simulation duration.
3. **Span**: Structure spans from at least x <= 6.0m to x >= 14.0m.
4. **Complexity**: Meets the minimum beam (5) and joint (6) counts.
5. **Variety**: At least one joint must be a pivot (`type='pivot'`).

- **Mass Budget**: Total instantaneous structure mass must never exceed 400 kg.
- **Beam Count**: At least 5 beams are required.
- **Joint Count**: At least 6 joints are required.
- **Pivot Requirement**: At least one joint must be a pivot (`type='pivot'`).
- **Span Left**: At least one beam center must have x ≤ 6.0 m.
- **Span Right**: At least one beam center must have x ≥ 14.0 m.
- **Build Zone**: All beams must be placed within x ∈ [5.0, 15.0] m, y ∈ [1.5, 8.0] m.
- **Beam Size**: Each beam's width and height are each independently bounded between 0.1 m and 4.0 m.
- **APIs**: Use only the primitives documented below.
""",
    'primitives_api': API_INTRO + '\n' + '\n\n'.join(_api_data['E_04'].values()),

}
