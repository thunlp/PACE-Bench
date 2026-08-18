import os

import json

from pace_bench.tasks.primitives_api import API_INTRO

with open(os.path.join(os.path.dirname(__file__), '..', '..', 'primitives_api.json'), 'r') as f:
    _api_data = json.load(f)

if 'S_05' in _api_data and 'API_INTRO' in _api_data['S_05']:
    del _api_data['S_05']['API_INTRO']

TASK_PROMPT = {
    'task_description': """
Protect a fragile Core (a sensitive circular object at x=10.0, y=1.0) from heavy falling boulders that come from multiple directions.
Boulders will spawn above the core, mostly targeting the center, but some will fall from the left and right sides. The core is extremely delicate and will fail if subjected to significant impact forces.

- **Core**: A circular object centered at (10.0, 1.0) with radius 0.5 m. It fails if any single impact force exceeds 150 N (its structural tolerance).
- **Ground**: A static surface at y=0.0. Your shelter must be supported by the ground outside the designated keep-out zone.
- **Lateral boundaries**: The scene has no lateral containment walls; the build zone is open at the sides.
- **Boulders**: Boulders fall from a high altitude (y=15m). They target the build zone broadly. Their restitution and density are latent material properties; infer their effects from observed impacts rather than expecting numeric coefficients in the prompt. In the nominal mission, 12 boulders spawn from above (one every 30 simulation steps), and 4 additional boulders spawn from the left and right sides (every 90 steps). Success is evaluated only after the simulation has run at least 1000 steps and the full bombardment sequence has completed (i.e. at least meteor_count × spawn_interval steps, whichever is larger). You must design a structure to deflect or absorb these impacts.
- **Build Zone**: Structure must be built within x=[5, 15] m and y=[0, 8] m (beam centers and joint anchors must lie inside this region).
- **Gravitational Constant**: Downward acceleration may differ from nominal.
- **Wind**: No lateral wind in the nominal mission.
- **Beam Dimensions**: 0.1 <= width, height <= 10.0 m.
- **Joint Limits**: Joints and anchors have a maximum linear force capacity of 1e12 N and a maximum torque capacity of 1e12 Nm in the nominal mission; these limits may be restricted in mission variants.

Design a shelter structure that:
1. Effectively shields the core from impacts originating from above.
2. Deflects or absorbs the kinetic energy of heavy falling boulders so that the force reaching the core remains within its tolerance.
3. Maintains structural stability under its own weight and during repeated impacts.
4. Complies with the mission's mass budget and height restrictions.

- **Build Zone**: All beam centers and joint anchors must lie within x=[5, 15] m, y=[0, 8] m.
- **Keep-Out Zone**: You cannot build any structural components within 1.3m of the core center (10.0, 1.0).
- **Height Limit**: No beam center may be above y=7.5m.
- **Collapse Threshold**: Beam centers must remain above y=0.3 m. The shelter is considered collapsed if any beam center falls below this threshold at any point during the bombardment.
- **Structural Integrity**: The shelter must remain standing throughout the bombardment. Structural collapse is defined as any beam center falling below y=0.3 m. Joints may break if force or torque exceeds their limits.
- **Mass Budget**: Total structure mass must not exceed 300 kg.
- **Bombardment Parameters**: In the nominal mission, 12 boulders spawn (one every 30 simulation steps) and 4 additional boulders spawn from the sides (every 90 steps). Restitution and density remain undisclosed latent properties. These parameters define the loading environment the structure must withstand.
""",
    'success_criteria': """

1. **Protection**: The core survives the entire bombardment; peak impact force on the core must not exceed 150 N.
2. **Stability**: The shelter does not collapse (no beam center below y=0.3 m) under its own weight or the weight of the debris.

- **Build Zone**: Beam centers and joint anchors within x=[5, 15] m, y=[0, 8] m.
- **Lateral boundaries**: The scene has no lateral containment walls.
- **Keep-Out Zone**: Beam center distance to (10.0, 1.0) must be >= 1.3m.
- **Mass Budget**: <= 300 kg.
- **Height Limit**: No beam center may be above y=7.5m.
- **Core Force**: Peak force on core <= 150 N.
- **Gravitational Constant**: Downward acceleration may differ from nominal.
- **APIs**: Use only the primitives documented below.
""",
    'primitives_api': API_INTRO + '\n' + '\n\n'.join(_api_data['S_05'].values()),

}
