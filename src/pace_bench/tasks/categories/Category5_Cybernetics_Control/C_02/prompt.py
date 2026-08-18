import json

import math

import os

from pace_bench.tasks.primitives_api import API_INTRO

with open(
    os.path.join(os.path.dirname(__file__), '..', '..', 'primitives_api.json'),
    'r',
    encoding='utf-8',
) as f:
    _api_data = json.load(f)

if 'C_02' in _api_data and 'API_INTRO' in _api_data['C_02']:
    del _api_data['C_02']['API_INTRO']

try:
    from .environment import (
        BARRIER_X_LEFT,
        BARRIER_X_RIGHT,
        BARRIER_Y_BOTTOM,
        BARRIER_Y_TOP,
        DEFAULT_TIME_STEP_LABEL,
        GROUND_LENGTH,
        GROUND_SLAB_HEIGHT,
        GROUND_Y_TOP,
        LAND_TOLERANCE,
        LANDER_HALF_HEIGHT,
        LANDER_HALF_WIDTH,
        MAX_EPISODE_STEPS,
        MAX_LANDING_ANGLE,
        MAX_SAFE_VERTICAL_SPEED,
        MAX_THRUST,
        MAX_TORQUE,
        MIN_FUEL_REMAINING_AT_LANDING,
        PLATFORM_AMPLITUDE,
        PLATFORM_CENTER_BASE,
        PLATFORM_HALF_WIDTH,
        PLATFORM_PERIOD,
        SPAWN_X,
        SPAWN_Y,
        THRUST_DELAY_STEPS,
        TOTAL_FUEL_IMPULSE,
    )
except ImportError:
    from pace_bench.tasks.categories.Category5_Cybernetics_Control.C_02.environment import (
        BARRIER_X_LEFT,
        BARRIER_X_RIGHT,
        BARRIER_Y_BOTTOM,
        BARRIER_Y_TOP,
        DEFAULT_TIME_STEP_LABEL,
        GROUND_LENGTH,
        GROUND_SLAB_HEIGHT,
        GROUND_Y_TOP,
        LAND_TOLERANCE,
        LANDER_HALF_HEIGHT,
        LANDER_HALF_WIDTH,
        MAX_EPISODE_STEPS,
        MAX_LANDING_ANGLE,
        MAX_SAFE_VERTICAL_SPEED,
        MAX_THRUST,
        MAX_TORQUE,
        MIN_FUEL_REMAINING_AT_LANDING,
        PLATFORM_AMPLITUDE,
        PLATFORM_CENTER_BASE,
        PLATFORM_HALF_WIDTH,
        PLATFORM_PERIOD,
        SPAWN_X,
        SPAWN_Y,
        THRUST_DELAY_STEPS,
        TOTAL_FUEL_IMPULSE,
    )

_LANDING_ANGLE_DEG = math.degrees(MAX_LANDING_ANGLE)

_PLATFORM_TOTAL_WIDTH = 2.0 * PLATFORM_HALF_WIDTH

_LANDER_FULL_W = 2.0 * LANDER_HALF_WIDTH

_LANDER_FULL_H = 2.0 * LANDER_HALF_HEIGHT

_TASK_DESCRIPTION = f"""
Design a controller to safely land a craft on a moving platform while navigating around a no-fly zone.

- **Lander**: Rectangular hull {_LANDER_FULL_W:.1f} m × {_LANDER_FULL_H:.1f} m (half-width {LANDER_HALF_WIDTH} m, half-height {LANDER_HALF_HEIGHT} m). Starting position (spawn x={SPAWN_X} m, y={SPAWN_Y} m). The craft's mass and unmodeled environmental dynamics are not disclosed; infer their effects from the documented observations and interaction API.
- **Simulation**: Fixed time step {DEFAULT_TIME_STEP_LABEL} s per step. Each evaluation run is limited to **{MAX_EPISODE_STEPS} simulation steps**; the episode ends if this horizon is reached before a successful landing (treat as mission failure).
- **No-Fly Zone**: A vertical corridor at x in [{BARRIER_X_LEFT}, {BARRIER_X_RIGHT}] m. The lower bound is y={BARRIER_Y_TOP:.1f} m (ground-based obstacle top); the upper bound is y={BARRIER_Y_BOTTOM:.1f} m (ceiling) within that x band. If any hull corner lies in this x-range with y **strictly below** the lower bound or **strictly above** the upper bound, the mission fails—plan a path that keeps all corners within the allowed vertical band (including on the stated bounds) while in that x-range.
- **Ground**: The landing surface (ground and platform) is at y={GROUND_Y_TOP:.1f} m; the static ground fixture extends downward from that plane by {GROUND_SLAB_HEIGHT:.1f} m (slab thickness). The terrain extends horizontally over roughly {GROUND_LENGTH:.0f} m. Touchdown is detected when the craft's lowest point is within {LAND_TOLERANCE:.2f} m of the ground surface.
- **Landing Zone**: A moving platform on the ground. Its center oscillates around x={PLATFORM_CENTER_BASE} m with an amplitude of {PLATFORM_AMPLITUDE} m and a period of {PLATFORM_PERIOD} s. The valid landing area is {_PLATFORM_TOTAL_WIDTH:.1f} m total (center ± {PLATFORM_HALF_WIDTH:.1f} m) and its position depends on the time of landing.
- **Thrust**: Main engine thrust is applied along the craft's **body-up** axis (world +y when upright; max {MAX_THRUST:g} N); steering thrusters provide torque (max {MAX_TORQUE:g} N·m). **Commands are not latched:** you must send thrust and torque again on every simulation step you want them applied; after each physics step the pending command queue advances and the values you last issued are cleared until you set new ones. **Fuel / impulse is depleted only by main-engine thrust magnitude** (per simulation step); steering torque does not consume the impulse budget. If remaining impulse in a step is less than the thrust×Δt that would be applied, thrust is scaled down so impulse cost never exceeds what remains.
- **Control Latency**: Issued thrust/steering commands may be delayed by a fixed, undisclosed number of simulation steps before taking physical effect; infer latency from observed response.
- **Impulse Budget**: Total fuel impulse is {int(TOTAL_FUEL_IMPULSE) if abs(TOTAL_FUEL_IMPULSE - round(TOTAL_FUEL_IMPULSE)) < 1e-9 else TOTAL_FUEL_IMPULSE:g} N·s. You must land with at least the minimum remaining impulse stated in the success criteria below. **If remaining impulse reaches zero before touchdown, the mission fails** (no further main-engine thrust is available).

Design a control loop that:
1. Transits the permitted corridor without any hull corner entering a forbidden region.
2. Tracks the moving landing zone and times the descent accordingly.
3. Successfully soft-lands on the platform within specified safety and fuel-efficiency limits.
"""

_SUCCESS_CRITERIA = f"""

1. **Soft Landing**: At touchdown, vertical speed magnitude must satisfy |vy| <= {MAX_SAFE_VERTICAL_SPEED:.2f} m/s. Measurement uses world-frame vertical velocity (+y upward; evaluator uses |vy| at first ground contact).
2. **Upright Orientation**: Land with the craft within the stated angular limit (|angle| <= {_LANDING_ANGLE_DEG:.2f} degrees).
3. **Accuracy**: At touchdown, the craft's entire ground-contact width must lie within the valid landing platform: **{_PLATFORM_TOTAL_WIDTH:.1f} m total (center ± {PLATFORM_HALF_WIDTH:.1f} m)** at the instant of landing (zone position at that time; not only the center x).
4. **Efficiency**: Land with at least {int(MIN_FUEL_REMAINING_AT_LANDING) if abs(MIN_FUEL_REMAINING_AT_LANDING - round(MIN_FUEL_REMAINING_AT_LANDING)) < 1e-9 else MIN_FUEL_REMAINING_AT_LANDING:g} N·s of impulse budget remaining.

- **No-Fly Zone**: x in [{BARRIER_X_LEFT}, {BARRIER_X_RIGHT}] m.
- **Landing Orientation**: |angle| <= {_LANDING_ANGLE_DEG:.2f} degrees.
- **APIs**: Use only the primitives documented below.
"""

TASK_PROMPT = {
    "task_description": _TASK_DESCRIPTION,
    "success_criteria": _SUCCESS_CRITERIA,
    'primitives_api': API_INTRO + '\n' + '\n\n'.join(_api_data['C_02'].values()),

}
