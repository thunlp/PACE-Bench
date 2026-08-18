TASK_PROMPT = {
    'task_description': """
You need to design a vehicle that can climb slopes in DaVinci Sandbox.

- Start position: x=5.0m
- Target position: x=30.0m (must pass all obstacles)
- Terrain: Contains rough terrain and obstacles
  - **Ground**: Starts from x=0, width 50m, **ground top is at y=1.0m** (ground bottom is at y=0m, ground height is 1.0m)
  - Obstacle 1: Position x=15m, height 2m, angle 0.2 radians
  - Obstacle 2: Position x=25m, height 3m, angle -0.3 radians

Design a mechanical structure (vehicle) that can:
1. Move stably on the terrain
2. Pass all obstacles
3. Reach the target position (x=30.0m)

- Chassis height cannot exceed 1.0m
- Wheel radius must be in range [0.3, 2.0]m
- Maximum 2 wheels allowed (vehicle must have at most 2 wheels)
- Wheels must contact the ground (cannot be suspended)
- Connection distance cannot exceed 5.0m
- Motor speed should be in range [-50, 50] rad/s
- Maximum torque cannot exceed 2000 N·m
""",
    'success_criteria': """

- **Primary Objective**: Agent's chassis must reach position x=30.0m
- **Secondary Constraint**: Agent cannot fall off the map (y < -10)
- **Design Constraint**: Agent cannot move backward too much (x < start_x - 5)
- **Stability Constraint**: Agent must move stably on the terrain
  - Angular velocity must remain below 2.0 rad/s (excessive rotation indicates unstable/hacking behavior)
  - Altitude must remain below 8.0m (excessive height indicates flying/hacking behavior)
  - **Cannot rotate more than 180 degrees while airborne** (agent cannot flip/spin excessively in the air)

- Successful completion: 100 points (must meet all constraints including stability)
- Partial completion: Based on distance traveled, up to 80 points
- Failure: 0 points (fell off map, moved backward too much, or violated stability constraints)
""",
    'primitives_api': """

```python
chassis = sandbox.add_beam(x, y, width, height, angle=0, density=1.0)
```
- `x, y`: Chassis center position (meters)
- `width, height`: Chassis dimensions (meters), height cannot exceed 1.0m
- `angle`: Rotation angle (radians, default 0)
- `density`: Density (kg/m³, range 0-10)
- Returns: Chassis body object

```python
wheel = sandbox.add_wheel(x, y, radius, density=1.0, friction=0.8)
```
- `x, y`: Wheel center position (meters)
- `radius`: Wheel radius (meters, range [0.3, 2.0])
- `density`: Density (kg/m³, range 0-10)
- `friction`: Friction coefficient (range [0, 5])
- Returns: Wheel body object
- **Important**: Ground top is at **y=1.0m**. For a wheel to contact the ground, its center y position should be **y = 1.0 + radius** (so wheel bottom at y=1.0m touches ground top)
- **Constraint**: Maximum 2 wheels allowed in the design

```python
joint = sandbox.connect(body_a, body_b, anchor_x, anchor_y, motor_speed=0.0, max_torque=0.0)
```
- `body_a, body_b`: Two body objects to connect
- `anchor_x, anchor_y`: Connection point position (meters)
- `motor_speed`: Motor speed (rad/s, range [-50, 50])
- `max_torque`: Maximum torque (N·m, range [0, 2000])
- Returns: Joint object
- **Note**: Connection point distance from body cannot exceed 5.0m

```python
is_valid, errors = sandbox.validate_design(chassis)
```
- `chassis`: Chassis body object
- Returns: `(is_valid, errors)` tuple
- Checks if design meets physical constraints
"""

}
