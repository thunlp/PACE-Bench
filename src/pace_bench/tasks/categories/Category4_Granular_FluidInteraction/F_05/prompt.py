import os

import json

from pace_bench.tasks.primitives_api import API_INTRO

with open(os.path.join(os.path.dirname(__file__), '..', '..', 'primitives_api.json'), 'r') as f:
    _api_data = json.load(f)

if 'F_05' in _api_data and 'API_INTRO' in _api_data['F_05']:
    del _api_data['F_05']['API_INTRO']

# The central manifest is shared by every task and retains legacy explanatory
# text.  F-05 keeps the callable signatures while removing invisible simulator
# coefficients from the solver-facing copy.
if 'F_05' in _api_data:
    _add_beam = _api_data['F_05'].get('ADD_BEAM', '')
    _friction_start = _add_beam.find('- **Friction**:')
    _friction_end = _add_beam.find('- **Function**:', _friction_start)
    if _friction_start >= 0 and _friction_end > _friction_start:
        _add_beam = (
            _add_beam[:_friction_start]
            + '- **Contact behavior**: Beam contact parameters are environment-defined; infer their effect from measured outcomes.\n'
            + _add_beam[_friction_end:]
        )
    _api_data['F_05']['ADD_BEAM'] = _add_beam
    _cargo_count_api = _api_data['F_05'].get('GET_CARGO_IN_WATER_COUNT', '')
    _api_data['F_05']['GET_CARGO_IN_WATER_COUNT'] = _cargo_count_api.replace(
        'default 120', 'default 180'
    )
    _boat_api = _api_data['F_05'].get('GET_BOAT_BODY', '')
    _api_data['F_05']['GET_BOAT_BODY'] = _boat_api + (
        '\n- **Read-only diagnostic**: Do not call Box2D state-mutating methods on the returned hull. '
        'Candidate code may only change bodies it created through the documented construction primitives.'
    )

TASK_PROMPT = {
    "task_description": """
Design a stabilization and containment structure for a boat in rough water.

- **Water / interaction band**: Buoyancy and wave forcing apply when the hull is in x ∈ [5.0, 25.0] m and its center satisfies y ≤ 3.0 m (i.e. within 1.0 m above the nominal free surface y = 2.0 m).
- **Water volume (sensor)**: The nominal water region is a Box2D **sensor** (not a solid frictional boundary). Baseline geometry is a horizontal rectangle about **20 m** wide, centered at **x = 15 m**, spanning approximately **y ∈ [0, 3] m** (3 m tall), consistent with the hull interaction band above. It does not provide Coulomb traction between “water” and bodies in the usual sense—support and forcing follow the buoyancy and band rules below, not water–body friction parameters stated here.
- **Cargo vs hull in water**: Cargo particles receive an extra upward fluid-style force only while their center lies **strictly below** the nominal free surface y = 2.0 m and x is in the same water band; the hull uses the y ≤ 3.0 m band above. Coupling differs between hull and cargo in the simulator—treat both bands as part of the coupled fluid interaction.
- **Buoyancy and fluid coupling**: In the water band, the hull receives upward support that scales with total mass (hull + structure + cargo) and with vertical offset relative to a reference height tied to the nominal free surface (**y = 2.0 m**); the simulator clamps that support to be nonnegative. Submerged cargo (**center strictly below y = 2.0 m** in the water band) receives additional upward support each step using the same underlying weight-intensity scaling. Exact coupling coefficients and reference offsets are environment-defined and not enumerated here; infer net buoyancy from motion and feedback. Wave and impulsive forces are separate. This coupling is **not** the same as the evaluator's cargo **loss plane** (see Success Criteria).
- **Evaluator loss plane vs free surface**: Retention is scored against a separate horizontal **loss plane** (y given under Success Criteria), which may be **above or below** the nominal free surface y = 2.0 m in variants. A particle can fail retention without ever dropping below y = 2.0 m if the loss plane is raised.
- **Loss-plane grace window**: The evaluator ignores loss-plane crossings during the first **180** physics steps. The first numbered cargo-retention rule in the scoring section uses that baseline value; staged prompts update it when a scored variant changes it.
- **Boat**: Hull center at x≈15 m, y≈2.5 m. The hull is a 3.0 m × 0.4 m rectangle (full width × height) and is dynamically simulated (it can translate and roll). The hull's **upper deck / top contact band** for placement math lies **0.2 m** above the hull center (half of the **0.4 m** hull height).
- **Hull & beam contacts**: The hull and beams you build share environment-defined contact behavior. Infer effective traction at cargo, floor, and rock contacts from measured outcomes.
- **Cargo**: 10 circular particles, radius 0.15 m. Density, restitution, friction, and damping are environment-defined contact properties.
- **Cargo placement**: Spawn layouts are pseudo-random but reproducible under the standard harness. Disk centers begin across approximately **[-1.35, 1.35] m** from the hull center, with vertical offsets in **[0, 0.55] m** above the upper-deck reference before the disk radius is added.
- **Submerged obstacles**: four rocks: (13.50, 1.00, r=0.24); (14.50, 1.10, r=0.22); (15.50, 1.05, r=0.23); (16.50, 1.08, r=0.22). Each rock uses environment-defined contact parameters (magnitudes omitted in the task text); variants may change positions/radii and therefore the hazard field.
- **Seabed / floor**: A **0.3 m** thick horizontal slab spans x ∈ [0, 30] m with its **upper surface** near **y = 0 m** (baseline body center near **y = −0.15 m**). The floor participates in frictional contact with hull, cargo, and beams; effective friction at a contact combines both fixtures' coefficients in Box2D—infer traction from interaction and feedback rather than from a single global coefficient printed here.
- **Build zone**: Beam centers must lie in x=[12.0, 18.0], y=[2.0, 4.5]. Every weld anchor for `add_joint` (hull attachment or beam–beam) must lie in the same box (enforced at build time and in design checks). **Beam footprint**: each beam is an oriented rectangle; **all four corner vertices in world space** (after rotation) must lie inside this same x/y box—the design checker tests these corners (not a separate world-axis-aligned hull of the slanted rectangle).
- **Damping (baseline)**: Hull, beams, and cargo use linear damping and angular damping by default (simulator units; variants may override per body class).
- **Passive hull roll restoring**: In addition to waves, wind, current, and impulsive loads, the simulator applies a hull **torque that opposes roll angle** (a restoring couple toward upright). Stiffness is environment-defined and not stated numerically here; staged variants may change it—infer behavior jointly from motion, feedback, and the roll limit in Success Criteria.
- **Simulator integration**: The standard harness calls `sandbox.step` with **Δt = 1/60 s** (60 FPS `common.simulator.TIME_STEP`, unless a caller overrides the step argument). Each step runs Box2D **`World.Step(Δt, 10, 10)`** (10 velocity and 10 position iterations). Weld failure checks sample reactions with **`inv_dt = 1/Δt`**, so the force cap and the coupled torque check scale consistently with the timestep.

Design a structure that:
1. Prevents cargo from crossing the evaluator loss plane despite severe vessel motion.
2. Lowers the center of mass or provides stabilization to prevent the boat from capsizing.
3. Withstands periodic and impulsive loads from the simulated sea state.
""",
    "success_criteria": """

1. **Cargo Retention**: A particle fails if its center **ever** falls below y = 1.90 m after the first 180 physics steps (brief spawn/settling is ignored).
2. **Stability**: Peak absolute hull roll angle after that same settling window must stay at or below 18.0 degrees.
3. **Tertiary**: Structure remains intact (all welds survive the episode).

- **Mass Budget**: Total structure mass <= 60 kg.
- **Beam dimensions**: Each beam width and height is clamped by the environment to [0.1, 1.0] m (see Add Beam in the API below).
- **Joint structural limits**: When no per-weld load cap is configured, welds do not break under reaction loads. When a force cap **F_max** (newtons) is configured, a weld breaks if the simulated per-weld reaction **force** magnitude exceeds **F_max** or the simulated per-weld reaction **torque** magnitude exceeds **0.4 × F_max**. Both samples use the Box2D timestep-scaled reaction API with **`inv_dt = 1/Δt`** (same **Δt** as `World.Step`), so the torque threshold is **not** an independent SI torque budget — it is a coupled scalar limit tied to **F_max** and the integration timestep. Numeric **F_max** and the paired **0.4 × F_max** torque threshold appear in Success Criteria when configured; use episode feedback when limits are not printed.
- **APIs**: Use only the primitives documented below.
- **Environment integrity**: Treat the boat, cargo, rocks, floor, water sensor, and physics configuration as read-only. Do not apply forces/torques directly to environment bodies or assign their state. `agent_action` may only use documented primitives and may be empty for a passive design.
""",
    'primitives_api': API_INTRO + '\n' + '\n\n'.join(_api_data['F_05'].values()),

}
