import os

import json

from pace_bench.tasks.primitives_api import API_INTRO

with open(os.path.join(os.path.dirname(__file__), '..', '..', 'primitives_api.json'), 'r') as f:
    _api_data = json.load(f)

if 'F_01' in _api_data and 'API_INTRO' in _api_data['F_01']:
    del _api_data['F_01']['API_INTRO']

TASK_PROMPT = {
    "task_description": """
Design a free-standing dam to block water particles in an extreme environment.

- **Water Particles**: 300 circular particles in a reservoir (default radius 0.12 m each unless the task configuration overrides it).
- **Reservoir fill height**: 7.0 m.
- **Reservoir-side forcing**: Time-varying fluid impulses apply only to water particles whose center is on the reservoir side (x < 12.0 m).
- **Dynamic disturbances**: The run includes forward/backward slosh, vertical fluid kicks, periodic debris impacts, and horizontal shaking of dam beams. Exact timing and magnitudes are intentionally not disclosed and must be inferred from interaction feedback.
- **Debris launch**: Periodic debris impacts occur during the run; their exact timing and launch velocity are intentionally undisclosed and must be inferred from observed behavior.
- **Build zone**: Three disjoint narrow vertical strips: x=[12.4, 12.6], [12.9, 13.1], and [13.4, 13.6], with y in [0, 7.5] m. **Each beam’s center (x, y) must lie in one of these strips** (x within that strip’s interval, y within [0, 7.5]); design validation checks centers, consistent with the span and vertical-band rules below.
- **Constraint**: Mandatory underflow gap; no beams allowed below y=0.5m.
- **Constraint**: ZERO floor anchors; the dam must be free-standing.
- **Constraint**: Maximum beam width is 0.6 m; maximum beam height is 1.5 m. Each beam width and height is also clamped to at least 0.2 m and at most 4.0 m by the simulator (values below 0.2 m are raised; values above the max width/height caps are lowered).
- **Constraint**: Beam-to-beam welds break when reaction force **reaches or exceeds** 50000 N for 3 consecutive simulation steps (the consecutive-step window is also mutable across stages; the simulator treats **≥** threshold as contributing to failure).
- **Constraint**: Middle build strip [12.9, 13.1] must contain at least 1 but at most 1 beam (forces bridge topology).
- **Constraint**: Right strip [13.4, 13.6] may contain at most 2 beams.
- **Constraint**: The dam must span the gate: at least one beam center in the left strip x=[12.4, 12.6] and at least one beam center in the right strip x=[13.4, 13.6].
- **Constraint**: All beams must form one connected structure: every beam must be reachable from every other via beam-to-beam joints (no isolated beams or separate sub-structures).
- **Constraint**: At least 3 beam centers must lie in each vertical band: y=[0.5, 2.5], [2.5, 5], and [5, 7.5].
- **Moving downstream wall**: The downstream wall (0.5 m wide) oscillates laterally over time with amplitude and phase parameters, where the wall center follows a sinusoidal trajectory each step. The instantaneous leak line is the wall's left edge.
- **Leakage definition**: The fixed downstream flood boundary is x=14.0 m. Particles with x greater than this boundary count as full leak; particles in the 0.5 m band immediately upstream, x in (13.5, 14.0], count as half-leak. The oscillating wall affects the physics but does not move the scoring boundary.

Design a structure that:
1. Blocks water particles such that the leakage rate does not exceed 0.10%.
2. Maintains structural integrity under dynamic disturbances (fluid surges/slosh, debris impact, wall oscillation, and horizontal shaking): after design validation, no weld joints may be removed (the initial joint count from your design must be preserved through the run).
3. Places every beam center inside the allowed build strips and respects the underflow requirement.
""",
    "success_criteria": """

1. **Leakage Rate**: Total leakage <= 0.10%.
2. **Integrity**: Structure does not collapse; no joints may break after the design is validated (same as: all initial welds remain intact for the full simulation).

- **Mass Budget**: Total structure mass <= 380 kg.
- **Beam Limit**: Between 10 and 18 beams (minimum 10, maximum 18).
- **Joint Limit**: Maximum 15 beam-to-beam joints.
- **Beam material default**: `add_beam` uses default areal density 500 kg/m² unless you pass a different `density` (total mass follows density × width × height).
- **Simulation horizon**: Scoring and integrity checks apply at the end of a run with a maximum of 10000 simulation steps (unless the harness overrides this).
- **APIs**: Use only the primitives documented below.
""",
    'primitives_api': API_INTRO + '\n' + '\n\n'.join(_api_data['F_01'].values()),

}
