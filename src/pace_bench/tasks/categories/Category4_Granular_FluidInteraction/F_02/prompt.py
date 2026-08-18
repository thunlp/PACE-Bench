import os

import json

from pace_bench.tasks.primitives_api import API_INTRO

with open(os.path.join(os.path.dirname(__file__), '..', '..', 'primitives_api.json'), 'r') as f:
    _api_data = json.load(f)

if 'F_02' in _api_data and 'API_INTRO' in _api_data['F_02']:
    del _api_data['F_02']['API_INTRO']

TASK_PROMPT = {
    "task_description": """
Design an amphibian vehicle to cross a body of water and reach the target bank.

- **Water**: A 14m wide water gap between x=10m and x=24m, with vertical extent y=[0.0, 2.0]m (surface at y=2.0m, bottom at y=0.0m).
- **Target**: Reach the right bank at x >= 26.0m.
- **Build Zone**: Vehicle must be built on the left bank in x=[2.0, 8.0], y=[0.0, 4.0].
- **Obstacles**: Three pillars (radius 0.46 m each) are located in the water at (x=14.0m, y=0.88m), (x=17.0m, y=0.90m), and (x=20.0m, y=0.92m).
- **Deep Channel**: Between x=16.5m and x=19.5m buoyancy is reduced (scale factor 0.35); vehicles in this zone experience much less upward buoyant force and may sink if not designed for it.
- **Beam Size**: Each beam dimension (width or height) must be between 0.15 m and 2.0 m.
- **Propulsion Limit**: Maximum force per component per thrust application is 520 N.
- **Speed Cap**: Maximum linear speed is 4.0 m/s (simulation stability).
- **Environmental Factors**: Fluid resistance, current strength/direction, and localized forces are latent physical parameters that must be inferred from observed motion.
- **Electromagnetic Deadzone**: The source environment has no EMP region. A non-standard environment may contain an undisclosed region that disables thrust; infer it from actuator response.
- **Corrosive Altitude Ceiling**: The source environment has no toxic atmospheric layer. A non-standard environment may contain an undisclosed altitude above which strong downward loading occurs.
- **Abyssal Whirlpool**: The source environment has no localized vortex. A non-standard environment may contain an undisclosed region with strong downward suction.
- **Sink Threshold**: Vehicle is considered sunk and fails if its lowest point falls below y = -0.5 m.
- **Propulsion**: Use `apply_force()` for paddling. **Cooldown**: Each component has a 3-step cooldown between thrusts.

Design a vehicle that:
1. Remains buoyant while crossing the water.
2. Uses effective propulsion (e.g., multiple paddles) to move forward against currents and other environmental resistance.
3. Can navigate over or through obstacles in the path.
4. Reaches the target bank.
""",
    "success_criteria": """

1. **Goal Reach**: Vehicle front reaches x >= 26.0m.
2. **Survival**: Vehicle does not sink (lowest point y < -0.5m).

- **Mass Budget**: Total structure mass <= 600 kg.
- **Joint Strength**: Structural connections do not break under load (no force limit).
- **APIs**: Use only the primitives documented below.
""",
    'primitives_api': API_INTRO + '\n' + '\n\n'.join(_api_data['F_02'].values()),

}
