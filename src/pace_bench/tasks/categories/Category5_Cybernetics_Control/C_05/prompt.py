import os

import json

from pace_bench.core.simulator import TIME_STEP

from pace_bench.tasks.primitives_api import API_INTRO

with open(os.path.join(os.path.dirname(__file__), '..', '..', 'primitives_api.json'), 'r') as f:
    _api_data = json.load(f)

if 'C_05' in _api_data and 'API_INTRO' in _api_data['C_05']:
    del _api_data['C_05']['API_INTRO']

C05_MAX_EPISODE_STEPS = 35000

TASK_PROMPT = {
    "task_description": f"""
Design a controller for an agent to trigger a "Logic Lock" by activating switches in a strict temporal and spatial sequence.

- **Terrain layout**: World x roughly 0–12 m. Flat ground at y = 2 m for x in [0, 4] and [7, 12]; ramp up from (4, 2) toward platform; platform segment with top near y = 3.5 m for x in [5, 6]; ramp down toward y = 2 by x ≈ 7. Zone B sits on the elevated approach; zones A and C on the lower ground.
- **Solid terrain profile**: Flat segments, the platform, and ramp bodies use a vertical half-thickness of 0.25 m about the nominal top contact line (internal collision geometry).
- **Authoritative collision layout (matches simulator bodies)**: Low ground: axis-aligned segments with top surface y = 2 m for x ∈ [0, 4] m (body center (2.0, 1.75) m, half-extents 2.0×0.25 m) and x ∈ [7, 12] m (center (9.5, 1.75) m, half-extents 2.5×0.25 m). Ramp-up static polygon centered at (4.75, 2.75) m (low friction). Platform segment centered at (5.5, 3.25) m, half-size 0.5×0.25 m (top y = 3.5 m). Ramp-down static polygon centered at (6.5, 2.75) m (low friction). The vertical barrier is a separate static box at x = 4.5 m (same centerline x as the **Barrier** bullet below).
- **Switches**: Three switches (A, B, C). Zone A: center (2.0, 2.0) m, half-width 0.5 m, half-height 0.5 m. Zone B: center (4.95, 3.2) m, half-width 0.7 m, half-height 0.4 m. Zone C: center (8.0, 2.0) m, half-width 0.5 m, half-height 0.5 m.
- **Sequence**: Switches must be triggered in the order A -> B -> C. **Wrong order is fatal**: if the **next** required switch is still A, entering B or C fails the run; if the next required is B (after A has triggered), entering C before B has triggered fails the run. (Re-entering an **already triggered** zone, e.g. standing in A again after A fired, does not by itself fail.) **Time limit**: Not completing A→B→C before the episode step budget (below) ends counts as failure.
- **Activation duration**: The agent must stay inside a zone for 25 consecutive steps (with speed and force constraints below) to trigger it.
- **Speed cap inside zones**: Maximum velocity allowed inside a trigger zone for progress to count is 0.5 m/s; exceeding this resets that zone's progress.
- **Cooldown between triggers**: After triggering a zone, the agent must wait 55 steps before the next zone will accept progress.
- **Barrier**: A narrow vertical gate (half-width ≈ 0.08 m) at x = 4.5 m, spanning y from 0 to 4 m, blocks passage until it opens according to **Barrier delay after A** below.
- **Barrier delay after A**: The gate opens 70 steps after zone A is triggered, not immediately.
- **Temporal window A to B**: Zone B only counts stay-steps if the agent was in zone A within the last 160 steps. While the agent center remains inside zone A, the simulator refreshes this recency reference every step, so time spent waiting in A—including after A has already triggered—does not by itself consume the A→B window.
- **Temporal window B to C**: Zone C only counts stay-steps if the agent was in zone B within the last 400 steps.
- **C altitude requirement**: Zone C only counts stay-steps if the agent's maximum y over the retained y-history window (length up to 150 simulation steps; shorter early in the episode) is at least 2.9 m (approach from elevated path).
- **Force limit inside zone**: Applying **controller** force with magnitude above 60 N (Newtons) while inside a zone resets that zone's progress. Diagonal inputs near the per-axis cap can exceed this limit.
- **Repulsion**: Repulsive forces may act near B and C. Their range, radial strength, and any tangential component are latent; infer the field from observed motion. The agent must navigate these fields (B until A triggered, C until B triggered).
- **Agent**: Spawn at (0.5, 1.95) m; visible radius 0.2 m; mass and damping are latent. Infer inertial and resistance effects from observations.
- **Agent max applied force**: The controller can apply at most 50.0 N (Newtons) per axis per step (same convention as **apply_agent_force** in the API below).
- **Collision and unstated dynamics**: Contacts use zero restitution (no bounce). Friction coefficients for terrain, agent, and barrier are not enumerated here; infer them from observed acceleration behavior. Any other simulator-side influences on motion not enumerated here may require inference from observations.
- **Ambient wind / lateral forcing**: Time-varying lateral body forces may or may not be present; amplitude and period are not stated here—treat unexpected drift as a cue to infer such forcing from observations.

- **Maximum steps per episode**: {C05_MAX_EPISODE_STEPS} simulation steps. The task must be completed within this horizon.
- **Simulation timestep**: Each physics step advances time by {TIME_STEP} s (fixed dt; "steps" in dwell/windows are simulation steps). The environment uses this same default dt when stepped without an explicit argument.

Design a control loop that:
1. Navigates to switch A to begin the sequence.
2. After A triggers, reach B while the **Temporal window A to B** rule still holds, and cross only after the barrier opens per **Barrier delay after A**.
3. Trigger C while respecting **Cooldown between triggers**, **Temporal window B to C**, **C altitude requirement**, **Speed cap inside zones**, and **Force limit inside zone**.
4. Stay within speed cap (0.5 m/s) and force limit 60 N inside zones so dwell time counts toward triggers.
""",
    "success_criteria": f"""

1. **Sequence Completion**: Switches A, B, and C triggered in the correct order and within their respective temporal/spatial windows.
2. **Efficiency**: Full A→B→C sequence completed within {C05_MAX_EPISODE_STEPS} simulation steps (including clearing the timed barrier once it opens).

- **Wrong-order rule**: If the **next** required switch is still A, entering B or C fails the run; if the next required is B, entering C before B fails. Re-entering an already triggered zone alone does not fail.
- **Activation duration**: 25 consecutive steps per zone (with speed <= 0.5 m/s and force <= 60 N inside zone).
- **Cooldown**: 55 steps between triggers.
- **Barrier geometry**: Vertical gate at x = 4.5 m (centerline), half-width ≈ 0.08 m, spanning y from 0 to 4 m; blocks passage until opened per **Barrier delay** below.
- **Barrier delay**: 70 steps after A before gate opens.
- **Temporal windows**: A to B within 160 steps; B to C within 400 steps.
- **C altitude history**: Rolling window of up to 150 simulation steps (TRIGGER_STAY_STEPS = 25, C_HIGH_HISTORY = 150).
- **Repulsion**: Field range and radial/tangential strengths are latent; infer them from observed motion. Fields may act near B until A triggers and near C until B triggers.
- **Agent max applied force**: At most 50.0 N per axis per simulation step (**apply_agent_force**), matching the task API.
- **APIs**: Use only the primitives documented below.
""",
    'primitives_api': API_INTRO + '\n' + '\n\n'.join(_api_data['C_05'].values()),

}
