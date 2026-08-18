API_INTRO = """

**STRICT RULE**: You are restricted to using ONLY the `primitives_api` explicitly documented here. You are STRICTLY PROHIBITED from hallucinating APIs or utilizing any undocumented functions. You may access any body's state (e.g., `body.position`, `body.linearVelocity`, `body.angle`) for control logic. You are permitted to modify the physical properties of **dynamically created tools** (e.g., setting `tool.linearVelocity`, `tool.angularVelocity`, or using `tool.ApplyForceToCenter`). However, directly modifying physics properties of environmental bodies (like ground friction) is STRICTLY PROHIBITED. Use forces and torques for primary control.
"""

ADD_BEAM = """

```python
beam = sandbox.add_beam(x, y, width, height, angle=0, density=1.0)
```
- `x, y`: The coordinates (meters) for the center of the beam.
- `width, height`: The full width and height (meters) of the rectangular beam.
- `angle`: The initial rotation of the beam in radians (clockwise is positive).
- `density`: The material density in kg/m³. Mass is automatically calculated based on area.
- **Function**: Creates a dynamic rectangular body that responds to gravity and collisions.
- **Returns**: A Box2D body object.
"""

ADD_BLOCK = """

```python
block = sandbox.add_block(x, y, width, height)
```
- `x, y`: The coordinates (meters) for the center of the block.
- `width, height`: The dimensions (meters) of the rectangular block.
- **Function**: Adds a simple un-jointed block to the simulation. Often used in stacking tasks.
- **Returns**: A Box2D body object.
"""

ADD_WHEEL = """

```python
wheel = sandbox.add_wheel(x, y, radius, density=0.6)
```
- `x, y`: The coordinates (meters) for the center of the wheel.
- `radius`: The radius (meters) of the circular body.
- `density`: The material density in kg/m³.
- **Function**: Creates a dynamic circular body. Ideal for rolling mechanisms or leg endpoints.
- **Returns**: A Box2D body object.
"""

ADD_PAD = """

```python
pad = sandbox.add_pad(x, y, radius, density=0.8)
```
- `x, y`: The coordinates (meters) for the center of the suction pad.
- `radius`: The radius (meters) of the pad.
- `density`: The material density in kg/m³.
- **Function**: Creates a circular body that can provide adhesion forces when used with `set_pad_active`.
- **Returns**: A Box2D body object.
"""

ADD_STATIC_BEAM = """

```python
beam = sandbox.add_static_beam(x, y, width, height, angle=0, density=1.0)
```
- `x, y`: Center coordinates (meters).
- `width, height`: Dimensions (meters).
- `angle`: Rotation in radians.
- `density`: Density in kg/m³.
- **Function**: Adds a rectangular body that is IMMOVABLE (fixed in space). Use this for permanent structural supports or fixed obstacles that don't need to be anchored via joints.
- **Returns**: A Box2D static body object.
"""

ADD_ANCHORED_BASE = """

```python
base = sandbox.add_anchored_base(x, y, width, height, angle=0, density=1.0)
```
- `x, y`: Center coordinates (meters).
- `width, height`: Dimensions (meters).
- `angle`: Rotation in radians.
- `density`: Density in kg/m³.
- **Function**: A convenience method that creates a dynamic beam and immediately welds it to the static ground at the specified (x, y) location. Useful for creating fixed bases for arms or towers.
- **Returns**: A Box2D body object.
"""

ADD_SCOOP = """

```python
scoop = sandbox.add_scoop(x, y, width, height, angle=0, density=1.0)
```
- `x, y`: The coordinates (meters) of the hinge corner.
- `width`: The length of the horizontal "floor" of the L-shape.
- `height`: The height of the vertical "back wall" of the L-shape.
- `angle`: Rotation in radians.
- `density`: Density in kg/m³.
- **Function**: Creates an L-shaped compound body designed to hold and transport granular or fluid particles. The hinge corner is located at (x, y).
- **Returns**: A Box2D body object.
"""

ADD_JOINT_RIGID = """

```python
joint = sandbox.add_joint(body_a, body_b, anchor_point, type='rigid')
```
- `body_a`: The first body to connect.
- `body_b`: The second body to connect. If `None`, the joint anchors `body_a` to the static environment.
- `anchor_point`: The (x, y) coordinates in the world where the connection occurs.
- `type`: Must be `'rigid'`.
- **Function**: Creates a Weld joint that prevents all relative motion between the two bodies.
- **Returns**: A Box2D joint object.
"""

ADD_JOINT_PIVOT = """

```python
joint = sandbox.add_joint(body_a, body_b, anchor_point, type='pivot', **kwargs)
```
- `body_a, body_b`: The two bodies to connect.
- `anchor_point`: The (x, y) coordinates of the rotation axis.
- `type`: Must be `'pivot'`.
- `**kwargs`:
  - `lower_limit, upper_limit`: (Optional) Rotation limits in radians.
  - `enable_motor`: (Optional) Set to `True` to enable the motor.
  - `motor_speed`: (Optional) Target angular velocity (rad/s).
  - `max_motor_torque`: (Optional) Maximum torque (N·m).
- **Function**: Creates a Revolute joint allowing rotation about the anchor point.
- **Returns**: A Box2D joint object.
"""

ADD_JOINT_SLIDER = """

```python
joint = sandbox.add_joint(body_a, body_b, anchor_point, type='slider', **kwargs)
```
- `body_a, body_b`: The two bodies to connect.
- `anchor_point`: The (x, y) coordinates of the slider axis origin.
- `type`: Must be `'slider'`.
- `**kwargs`:
  - `axis`: (Optional) A tuple (dx, dy) defining the direction of motion (e.g., `(0, 1)` for vertical).
  - `lower_translation, upper_translation`: (Optional) Limits of motion in meters.
  - `enable_motor`: (Optional) Set to `True` to enable linear motor.
  - `motor_speed`: (Optional) Target linear velocity (m/s).
  - `max_motor_force`: (Optional) Maximum force (N).
- **Function**: Creates a Prismatic joint allowing translation along a single axis.
- **Returns**: A Box2D joint object.
"""

ADD_REVOLUTE_JOINT = """

```python
joint = sandbox.add_revolute_joint(body_a, body_b, anchor_point, enable_motor=False, motor_speed=0.0, max_motor_torque=100.0)
```
- `body_a, body_b`: Bodies to connect.
- `anchor_point`: (x, y) coordinates of the pivot.
- `enable_motor`: Whether the motor is active.
- `motor_speed`: Initial target speed (rad/s).
- `max_motor_torque`: Maximum torque limit (N·m).
- **Function**: A specialized rotating joint with explicit motor parameters.
- **Returns**: A Box2D joint object.
"""

ADD_SPRING_STATICS = """

```python
spring = sandbox.add_spring(body_a, body_b, anchor_a, anchor_b, stiffness, damping)
```
- `body_a, body_b`: The two bodies to connect.
- `anchor_a, anchor_b`: Attachment points (x, y) on `body_a` and `body_b`.
- `stiffness`: Spring stiffness constant.
- `damping`: Damping coefficient.
- **Function**: Creates a spring-damper connection (Distance Joint). Often used for vibration control in statics.
"""

ADD_SPRING_DYNAMICS = """

```python
spring = sandbox.add_spring(body_a, body_b, anchor_a, anchor_b, rest_length=None, stiffness=500.0, damping_ratio=0.5)
```
- `body_a, body_b`: The two bodies to connect.
- `anchor_a, anchor_b`: Local anchor points on each body.
- `rest_length`: The natural length of the spring (meters). If `None`, defaults to the distance at creation.
- `stiffness`: Spring stiffness in N/m.
- `damping_ratio`: Damping ratio (0.0 = no damping, 1.0 = critical damping).
- **Function**: Creates a dynamic spring-damper for energy storage, suspension, or launching.
"""

SET_MOTOR = """

```python
sandbox.set_motor(joint, motor_speed, max_torque)
```
- `joint`: A pivot (revolute) joint object.
- `motor_speed`: The desired angular velocity in radians per second.
- `max_torque`: The maximum torque (N·m) the motor can exert to reach that speed.
- **Function**: Dynamically updates the motor parameters of a pivot joint during simulation.
"""

SET_SLIDER_MOTOR = """

```python
sandbox.set_slider_motor(joint, speed, max_force)
```
- `joint`: A slider (prismatic) joint object.
- `speed`: The desired linear velocity in meters per second.
- `max_force`: The maximum force (N) the motor can exert.
- **Function**: Dynamically updates the motor parameters of a slider joint during simulation.
"""

SET_MATERIAL_PROPERTIES = """

```python
sandbox.set_material_properties(body, restitution=0.2, friction=None)
```
- `body`: The target body object.
- `restitution`: The bounciness coefficient (0.0 = no bounce, 1.0 = perfect elastic).
- `friction`: The friction coefficient (typically 0.0 to 1.0). If `None`, uses environment defaults.
- **Function**: Updates the physical surface properties of all fixtures on a body.
"""

SET_PAD_ACTIVE = """

```python
sandbox.set_pad_active(pad, active)
```
- `pad`: A pad body object created by `add_pad`.
- `active`: Boolean. `True` enables suction/adhesion; `False` disables it.
- **Function**: Controls the adhesive state of a suction pad.
"""

SET_FIXED_ROTATION = """

```python
sandbox.set_fixed_rotation(body, fixed)
```
- `body`: The target body object.
- `fixed`: Boolean. `True` prevents the body from rotating; `False` allows normal rotation.
- **Function**: Locks or unlocks the angular degree of freedom of a body.
"""

SET_AWAKE = """

```python
sandbox.set_awake(body, awake)
```
- `body`: The target body object.
- `awake`: Boolean. `True` forces the body to wake up and participate in physics.
- **Function**: Prevents the physics engine from putting a body into "sleep" mode (deactivating it when stationary).
"""

GET_STRUCTURE_MASS = """

```python
total_mass = sandbox.get_structure_mass()
```
- **Returns**: The total mass (kg) of all components created by the agent.
- **Function**: Used to monitor compliance with the task's mass budget.
"""

GET_STRUCTURE_BOUNDS = """

```python
bounds = sandbox.get_structure_bounds()
```
- **Returns**: A dictionary:
  - `'top'`: Max Y coordinate.
  - `'width'`: Total horizontal span.
  - `'center_x'`: X-coordinate of the geometric center.
  - `'min_x', 'max_x'`: Horizontal extremes.
"""

GET_STRUCTURE_REACH = """

```python
max_x = sandbox.get_structure_reach()
```
- **Returns**: The maximum horizontal coordinate (meters) reached by any structural component.
"""

GET_STRUCTURE_MASS_LIMIT = """

```python
limit = sandbox.get_structure_mass_limit()
```
- **Returns**: The numerical value (kg) of the maximum allowed mass for this task.
"""

GET_MIN_BEAMS = """

```python
min_n = sandbox.get_min_beams()
```
- **Returns**: The minimum number of beam components required for a valid submission.
"""

GET_MIN_JOINTS = """

```python
min_j = sandbox.get_min_joints()
```
- **Returns**: The minimum number of joints required for a valid submission.
"""

GET_BUILD_ZONE = """

```python
min_x, max_x, min_y, max_y = sandbox.get_build_zone()
```
- **Returns**: Four floats defining the bounding box of the construction area.
"""

GET_SPAN_BOUNDS = """

```python
left, right = sandbox.get_span_bounds()
```
- **Returns**: The X-coordinates of the left and right edges of the gap or void.
"""

GET_ANCHOR_FOR_GRIPPER = """

```python
anchor = sandbox.get_anchor_for_gripper()
```
- **Returns**: A static Box2D body representing the fixed gantry or support.
- **Usage**: Weld your gripper base to this body to prevent the mechanism from falling.
"""

GET_GROUND = """

```python
ground = sandbox.get_ground()
```
- **Returns**: The static ground body object. Useful for attaching springs or anchors.
"""

GET_GROUND_Y_TOP = """

```python
y = sandbox.get_ground_y_top()
```
- **Returns**: The Y-coordinate of the top surface of the ground.
"""

GET_ARENA_BOUNDS = """

```python
min_x, max_x, min_y, max_y = sandbox.get_arena_bounds()
```
- **Returns**: The limits of the simulation world. Staying within these avoids out-of-bounds failures.
"""

GET_TERRAIN_JOINT_COUNT = """

```python
count = sandbox.get_terrain_joint_count()
```
- **Returns**: The current number of joints that connect structural components to the terrain.
"""

GET_OBJECT_POSITION = """

```python
pos = sandbox.get_object_position()
```
- **Returns**: A tuple `(x, y)` containing the current center coordinates of the target object.
"""

GET_PROJECTILE = """

```python
projectile = sandbox.get_projectile()
```
- **Returns**: The Box2D body object representing the projectile to be launched.
"""

GET_JUMPER = """

```python
jumper = sandbox.get_jumper()
```
- **Returns**: The Box2D body object representing the agent that must jump.
"""

GET_VEHICLE_CABIN = """

```python
cabin = sandbox.get_vehicle_cabin()
```
- **Returns**: The Box2D body object representing the main compartment of the vehicle.
"""

GET_SWING_SEAT = """

```python
seat = sandbox.get_swing_seat()
```
- **Returns**: The Box2D body object representing the seat of the swing.
"""

GET_VEHICLE_FRONT_X = """

```python
x = sandbox.get_vehicle_front_x()
```
- **Returns**: The current X-coordinate of the vehicle's front bumper.
"""

GET_FLUID_PARTICLES = """

```python
particles = sandbox.get_fluid_particles()
```
- **Returns**: A list of Box2D body objects, each representing a single fluid particle.
"""

GET_PARTICLES_SMALL = """

```python
particles = sandbox.get_particles_small()
```
- **Returns**: A list of small granular particle bodies.
"""

GET_PARTICLES_MEDIUM = """

```python
particles = sandbox.get_particles_medium()
```
- **Returns**: A list of medium-sized granular particle bodies.
"""

APPLY_FORCE = """

```python
sandbox.apply_force(body, fx, fy, step_count=None)
```
- `body`: The Box2D body object to apply force to.
- `fx, fy`: The force components in Newtons.
- `step_count`: (Optional) The current simulation step count. Used for calculating cooldowns in tasks where propulsion is limited.
- **Function**: Applies a world-space force to the center of the body.
"""

APPLY_THRUST = """

```python
sandbox.apply_thrust(f1, f2)
```
- `f1, f2`: Control inputs.
- **Usage**:
  - In Lunar Lander: `f1` is main engine thrust (vertical), `f2` is steering torque.
  - In Exotic Racer: `f1, f2` are horizontal and vertical thrust components.
- **Function**: Task-specific abstraction for agent propulsion.
"""

APPLY_FORCE_TO_PARTICLE = """

```python
sandbox.apply_force_to_particle(particle, fx, fy)
```
- `particle`: A single particle body object.
- `fx, fy`: Force components (Newtons).
- **Function**: Directly influences the motion of an individual particle.
"""

APPLY_IMPULSE_TO_SEAT = """

```python
sandbox.apply_impulse_to_seat(ix, iy)
```
- `ix, iy`: Impulse components in N·s (Newton-seconds).
- **Function**: Applies an instantaneous change in momentum to the swing seat.
"""

APPLY_FORCE_TO_SEAT = """

```python
sandbox.apply_force_to_seat(fx, fy)
```
- `fx, fy`: Force components (Newtons).
- **Function**: Applies a continuous force to the swing seat center.
"""

SET_JUMPER_VELOCITY = """

```python
sandbox.set_jumper_velocity(vx, vy)
```
- `vx, vy`: Target linear velocity components (m/s).
- **Function**: Directly sets the velocity of the jumper body.
"""

WELD_TO_GLASS = """

```python
sandbox.weld_to_glass(body, anchor_point)
```
- `body`: The structural component to be fixed.
- `anchor_point`: The (x, y) coordinates on the glass surface.
- **Function**: A specialized weld joint that connects a body to the non-colliding glass surface.
"""

REMOVE_INITIAL_TEMPLATE = """

```python
if hasattr(sandbox, 'remove_initial_template'):
    sandbox.remove_initial_template()
```
- **Function**: Clears the visual reference/placeholder provided by the environment. Call this at the very beginning of `build_agent`.
"""

GET_CART_BODY = """

```python
cart = sandbox.get_cart_body()
```
- **Returns**: The Box2D body object of the cart.
"""

GET_POLE_ANGLE = """

```python
angle = sandbox.get_pole_angle()
```
- **Returns**: The current tilt angle of the pole in radians.
"""

GET_POLE_ANGULAR_VELOCITY = """

```python
omega = sandbox.get_pole_angular_velocity()
```
- **Returns**: The current rotation speed of the pole in rad/s.
"""

GET_CART_POSITION = """

```python
x = sandbox.get_cart_position()
```
- **Returns**: The X-coordinate of the cart's center.
"""

GET_CART_VELOCITY = """

```python
vx = sandbox.get_cart_velocity()
```
- **Returns**: The current horizontal speed of the cart.
"""

APPLY_CART_FORCE = """

```python
sandbox.apply_cart_force(f)
```
- `f`: Horizontal force in Newtons.
- **Function**: Applies a force to the cart to control its position and the pole's balance.
"""

GET_LANDER_BODY = """

```python
lander = sandbox.get_lander_body()
```
- **Returns**: The Box2D body object of the lander craft.
"""

GET_LANDER_POSITION = """

```python
pos = sandbox.get_lander_position()
```
- **Returns**: `(x, y)` position of the lander.
"""

GET_LANDER_ANGLE = """

```python
angle = sandbox.get_lander_angle()
```
- **Returns**: Current rotation of the lander in radians.
"""

GET_LANDER_ANGULAR_VELOCITY = """

```python
omega = sandbox.get_lander_angular_velocity()
```
- **Returns**: Rotation speed of the lander in rad/s.
"""

GET_LANDER_SIZE = """

```python
half_width, half_height = sandbox.get_lander_size()
```
- **Returns**: Half-dimensions of the lander's collision box.
"""

GET_SEEKER_BODY = """

```python
seeker = sandbox.get_seeker_body()
```
- **Returns**: The Box2D body of the seeker agent.
"""

GET_SEEKER_POSITION = """

```python
pos = sandbox.get_seeker_position()
```
- **Returns**: `(x, y)` position of the seeker.
"""

GET_SEEKER_VELOCITY = """

```python
vel = sandbox.get_seeker_velocity()
```
- **Returns**: `(vx, vy)` velocity vector.
"""

GET_SEEKER_HEADING = """

```python
angle = sandbox.get_seeker_heading()
```
- **Returns**: The current orientation/heading of the seeker in radians.
"""

APPLY_SEEKER_FORCE = """

```python
sandbox.apply_seeker_force(fx, fy)
```
- `fx, fy`: Force vector components (Newtons).
- **Function**: Primary control input for maneuvering the seeker.
"""

GET_TARGET_POSITION = """

```python
pos = sandbox.get_target_position()
```
- **Returns**: `(x, y)` coordinates of the moving target.
"""

GET_LOCAL_WIND = """

```python
wx, wy = sandbox.get_local_wind()
```
- **Returns**: Current wind force vector (Newtons) acting at the agent's location.
"""

GET_REMAINING_IMPULSE_BUDGET = """

```python
budget = sandbox.get_remaining_impulse_budget()
```
- **Returns**: The remaining amount of fuel/impulse allowed before the agent loses power.
"""

GET_CORRIDOR_BOUNDS = """

```python
x_min, x_max = sandbox.get_corridor_bounds()
```
- **Returns**: The horizontal boundaries of the traversable tunnel.
"""

GET_TERRAIN_OBSTACLES = """

```python
obstacles = sandbox.get_terrain_obstacles()
```
- **Returns**: A list of current obstacles (positions and/or body objects).
"""

GET_AGENT_BODY = """

```python
agent = sandbox.get_agent_body()
```
- **Returns**: The main Box2D body object of the agent.
"""

GET_AGENT_POSITION = """

```python
pos = sandbox.get_agent_position()
```
- **Returns**: `(x, y)` coordinate tuple.
"""

GET_AGENT_VELOCITY = """

```python
vel = sandbox.get_agent_velocity()
```
- **Returns**: `(vx, vy)` velocity vector.
"""

APPLY_AGENT_FORCE = """

```python
sandbox.apply_agent_force(fx, fy)
```
- `fx, fy`: Force components (Newtons).
- **Function**: Standard movement control for the bot.
"""

GET_WHISKER_READINGS = """

```python
front, left, right = sandbox.get_whisker_readings()
```
- **Returns**: Proximity readings from three sensors. Lower values mean obstacles are closer.
"""

GET_TRIGGERED_SWITCHES = """

```python
switches = sandbox.get_triggered_switches()
```
- **Returns**: A list of identifiers for switches that have already been activated.
"""

GET_NEXT_REQUIRED_SWITCH = """

```python
pos = sandbox.get_next_required_switch()
```
- **Returns**: The `(x, y)` position of the next switch that needs to be triggered in the sequence.
"""

GET_WHEEL_BODY = """

```python
wheel = sandbox.get_wheel_body()
```
- **Returns**: The Box2D body object of the rotating wheel.
"""

GET_WHEEL_ANGULAR_VELOCITY = """

```python
omega = sandbox.get_wheel_angular_velocity()
```
- **Returns**: Current angular speed in rad/s.
"""

GET_TARGET_SPEED = """

```python
target = sandbox.get_target_speed()
```
- **Returns**: The current setpoint for the wheel's angular velocity.
"""

APPLY_MOTOR_TORQUE = """

```python
sandbox.apply_motor_torque(torque)
```
- `torque`: Control input in N·m.
- **Function**: Applies a rotational force to reach the target speed.
"""

GET_BODY_POSITION = """

```python
pos = sandbox.get_body_position()
```
- **Returns**: `(x, y)` position of the main body in exotic environments.
"""

GET_BODY_VELOCITY = """

```python
vel = sandbox.get_body_velocity()
```
- **Returns**: `(vx, vy)` velocity vector.
"""

GET_CRAFT_POSITION = """

```python
pos = sandbox.get_craft_position()
```
- **Returns**: `(x, y)` position of the spacecraft.
"""

IS_OVERHEATED = """

```python
status = sandbox.is_overheated()
```
- **Returns**: `True` if thermal damage is occurring.
"""

GET_HEAT = """

```python
h = sandbox.get_heat()
```
- **Returns**: Current heat accumulation value.
"""

GET_OVERHEAT_LIMIT = """

```python
limit = sandbox.get_overheat_limit()
```
- **Returns**: Threshold value where overheating starts.
"""

GET_SLED_POSITION = """

```python
pos = sandbox.get_sled_position()
```
- **Returns**: `(x, y)` position of the sled.
"""

GET_SLED_VELOCITY = """

```python
vel = sandbox.get_sled_velocity()
```
- **Returns**: `(vx, vy)` velocity vector.
"""

GET_CHECKPOINT_B_REACHED = """

```python
status = sandbox.get_checkpoint_b_reached()
```
- **Returns**: `True` if the intermediate goal has been satisfied.
"""

GET_SIM_TIME = """

```python
t = sandbox.get_sim_time()
```
- **Returns**: Current elapsed simulation time in seconds.
"""

GET_WIND_FORCE_AT_TIME = """

```python
fx = sandbox.get_wind_force_at_time(t)
```
- `t`: The time in seconds.
- **Returns**: Predicted horizontal wind force at that time.
"""

GET_STEP_COUNT = """

```python
n = sandbox.get_step_count()
```
- **Returns**: Total number of physics steps executed so far.
"""

JOINTS_LIST = """

- `sandbox.joints`: List of all created joint objects. Use to iterate over mechanisms.
"""

BODIES_LIST = """

- `sandbox.bodies`: List of all created Box2D body objects.
"""

ACCESS_TERRAIN_BODIES = """

```python
terrain_body = sandbox._terrain_bodies.get("key")
```
- **Returns**: The Box2D body for a fixed environment object (e.g., "cliff", "wall", "foundation").
- **Usage**: You may access `sandbox._terrain_bodies` for state information, but you are STRICTLY PROHIBITED from directly assigning values to its contents (e.g., `body.position = ...`) or directly overriding physics state variables (e.g., `body.linearVelocity = ...`, `body.angularVelocity = ...`). Use forces and torques for control.
"""

HAS_CENTRAL_WALL = """

```python
exists = sandbox.has_central_wall()
```
- **Returns**: `True` if the environment contains a central wall barrier.
"""
