import os

import json

from pace_bench.tasks.primitives_api import API_INTRO

with open(os.path.join(os.path.dirname(__file__), '..', '..', 'primitives_api.json'), 'r') as f:
    _api_data = json.load(f)

if 'F_04' in _api_data and 'API_INTRO' in _api_data['F_04']:
    del _api_data['F_04']['API_INTRO']

TASK_PROMPT = {
    "task_description": """
Design a static filtering structure to separate granular particles (three discrete size classes) into three target zones.

- **Particles**: Nominal radii ~0.06 m (small), ~0.10 m (medium), ~0.14 m (large), with per-spawn uniform radius jitter in **±0.006 m** before clamping; realized radii are clamped to small ∈ [0.04, 0.08] m, medium ∈ [0.07, 0.12] m, large ∈ [0.11, 0.16] m before release.
- **Particle counts (default)**: 15 small, 15 medium, and 15 large in the first wave; the same counts again at step 1800; and 15 of each size again at step 3600 — **135 particles total** (45 small, 45 medium, 45 large) after all waves have spawned. **Classification purity** uses the number of particle bodies **currently in the simulation** as the denominator (each batch adds to that count when it spawns), matching feedback zone totals.
- **Feed schedule**: Additional batches of particles are released at fixed simulation steps (second batch at step 1800, third at step 3600 by default). **Simulation budget**: allow **at least 5000** steps so every batch can spawn and the system can settle after the third wave (which starts at step 3600). **Step cap**: The sandbox exposes `MAX_STEPS` (default 10000); runners typically stop at that many steps unless configured otherwise—plan so the task is solvable within both the minimum above and this cap.
- **World floor (default)**: The ground spans horizontally about x=[0, 16] m (see `floor_length` in the environment; may be overridden via configuration). Vertical placement of the floor slab is fixed by the simulation.
- **Feed Zone**: Particles are introduced in x=[5.2, 6.9] m, y=[3.0, 5.0] m (bounds may be overridden via environment configuration).
- **Obstacles (default layout; baffles or sweepers may be disabled in some environment configs)**: Static vertical baffles and moving horizontal sweepers in the feed zone disturb particle flow. **Baffle geometry**: lower edge at y=2.4 m; four thin walls near x=5.45, 5.90, 6.35, 6.75 m (each ~0.08 m wide in x), extending upward to about y=5.2 m. **Sweepers**: two horizontal bars (each ~1.0 m wide × ~0.1 m tall in world units): lower bar centered near y=4.0 m shuttling in x≈[5.25, 6.85] m at nominal speed ~0.09 m/s; upper bar near y=4.5 m in x≈[5.30, 6.80] m at ~0.05 m/s in the opposite direction. Effective sweeper speeds may differ from these nominal values under some environment configurations (infer from observed motion).
- **Build Zone**: The entire footprint of every structural beam (all corners after rotation) must lie inside x=[5.20, 6.90] m, y=[1.72, 2.45] m. Each beam must have width and height in [0.08, 1.0] m.
- **Target Zones**:
  1. Small particles should be directed to the bottom (y < 1.92m).
  2. Medium particles should be directed to the middle (1.92m <= y < 2.52m).
  3. Large particles should be retained in the upper zone (y >= 2.52m).
- **Diagnostics naming**: Feedback may refer to the **sieve band** as the middle height range **1.92 m ≤ y < 2.52 m** (between the small-zone ceiling and the large-zone floor). Counts labeled “above sieve” / “below sieve” use those same boundaries.

Design a multi-layered filter that:
1. Effectively separates particles by size into their respective target zones.
2. Achieves **classification purity ≥ 35%** (fraction of particle bodies **currently in the simulation** that lie in the correct target zone for their size class), consistent with the success criteria.
3. Minimizes cross-zone contamination (see feedback metrics for counts below the feed when **y < 3.0 m** (Feed Zone lower y in the source environment)).
4. Maintains structural stability under particle load and sweeper impact using static beams (use `add_joint` only as documented if you connect multiple parts).
5. Preserves the initial structural pose throughout the run: any beam translation > 0.001 m or rotation > 0.001 rad from its post-build state is treated as integrity loss.
- **Design limits**: Total structure mass <= 75 kg; Maximum 6 beams.
""",
    "success_criteria": """

1. **Classification Purity**: Overall purity (correctly categorized particles / total particles **currently in the simulation**) >= 35%.
2. **Cross-zone quality**: Feedback reports contamination among particles below the feed when **y < 3.0 m** (Feed Zone lower y in the source environment). The scored gate is purity in criterion 1, not a separate contamination threshold.
3. **Integrity**: Structure remains stable and intact (joint count must not decrease, and each beam must remain within 0.001 m position drift and 0.001 rad angular drift from its initial post-build pose).

- **Zone Boundary Thresholds**: The horizontal boundary between the small zone and the medium (sieve) band is at **y = 1.92 m**; the boundary between the medium (sieve) band and the large zone is at **y = 2.52 m**. A particle that ends a run on the wrong side of either boundary (relative to its size class) directly reduces classification purity.
- **Mass Budget**: Total structure mass <= 75 kg.
- **Beam Limit**: Maximum 6 beams.
- **APIs**: Use only the primitives documented below.
""",
    'primitives_api': API_INTRO + '\n' + '\n\n'.join(_api_data['F_04'].values()),

}
