import os

import json

from pace_bench.tasks.primitives_api import API_INTRO

with open(os.path.join(os.path.dirname(__file__), '..', '..', 'primitives_api.json'), 'r') as f:
    _api_data = json.load(f)

if 'S_02' in _api_data and 'API_INTRO' in _api_data['S_02']:
    del _api_data['S_02']['API_INTRO']

TASK_PROMPT = {
    'task_description': """
    Build a tall tower on a narrow foundation that must survive an earthquake and strong wind.
    You may consider structural damping to counteract vibrations.

    - **Foundation**: The allowed ground-contact zone is ±4.5 m from the foundation center (x=[-4.5, 4.5] when the foundation is at rest). All ground contact must lie within this zone; the foundation may move during seismic activity, and contact is evaluated relative to its current position. (The physical foundation platform may be smaller than this zone.)
    - **Gravity**: The vertical gravitational acceleration is the standard Earth value.
    - **Seismic Activity**: The earthquake starts at t = 2.0 s. From that time, the foundation undergoes horizontal oscillations. The earthquake amplitude may evolve in intensity over the course of the seismic event. Your structure must survive these vibrations.
    - **Wind**: Lateral wind forces are applied to structural components above a certain altitude threshold, applied horizontally in the +x direction. Wind force scales with altitude according to the formula F = wind_force × (1 + wind_shear_factor × (y − wind_height_threshold)); the wind shear factor determines whether force increases with height. The wind oscillation frequency may vary across environments.
    - **Beam Dimensions**: 0.1 <= width, height <= 10.0 meters. Values outside this range are clamped to the nearest boundary.
    - **Width Limit**: Total structure width cannot exceed 24.0m.
    - **Joint Strength**: Maximum linear force for a joint is unlimited; maximum torque is unlimited.
    - **Safety Limit**: If the structure's topmost point exceeds y=150.0m, it is considered physically unstable and the task fails.

    Design a stable skyscraper structure that can:
    1. Reach a peak height greater than 30 m during the stable period before the earthquake (initial height criterion). The stable period is from the start of the simulation until t = 2.0 s; the initial height is the maximum top height recorded from step 10 until the earthquake starts (to allow for initial construction and settling).
    2. Maintain its integrity and height during and after the earthquake.
    3. Resist lateral wind forces at high altitudes.
    4. Maintain global stability (the structure's center of mass must remain within a safe lateral range).
    """,
    'success_criteria': """

    1. **Height**: Peak topmost point during the pre-seismic stable period must exceed 30 m (stable period: from step 10 until the earthquake starts at t = 2.0 s).
    2. **Survival**: Structure remains standing (top point remains ≥ 5 m) after the earthquake and wind duration.
    3. **Stability**: Global center of mass stays within x=[-300, 300] relative to foundation.
    4. **Physical Limits**: Topmost point must never exceed 150.0 m.

    - **Foundation Contact**: All ground contact must be within ±4.5 m of the foundation center (evaluated relative to foundation position).
    - **Height Requirement**: Must exceed 30 m (peak during stable period, step 10 until t = 2.0 s).
    - **Survival Threshold**: Topmost point must remain at or above y = 5.0 m at all times during and after the earthquake; falling below this height constitutes structural collapse.
    - **Stability Zone**: The global center of mass must remain within ±300 m of the foundation's lateral position.
    - **Width Limit**: Total structure width cannot exceed 24.0m.
    - **Joint strength**: Limits apply as stated in Task Environment.
    - **Safety Limit**: The topmost point must never exceed y = 150.0 m (exceeding this height indicates physics solver divergence and constitutes failure).
    - **APIs**: Use only the primitives documented below.
    """,
    'primitives_api': API_INTRO + '\n' + '\n\n'.join(_api_data['S_02'].values()),

}
