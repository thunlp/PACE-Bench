
# Example: Basic Task Solution

## Task Description

You need to design a vehicle that can climb slopes in the Sandbox.

### Task Environment
- Start position: x=5.0m
- Target position: x=30.0m (must pass all obstacles)
- Terrain: Contains rough terrain and obstacles
  - **Ground**: Starts from x=0, width 50m, **ground top is at y=1.0m** (ground bottom is at y=0m, ground height is 1.0m)
  - Obstacle 1: Position x=15m, height 2m, angle 0.2 radians
  - Obstacle 2: Position x=25m, height 3m, angle -0.3 radians

### Task Objective
Design a mechanical structure (vehicle) that can:
1. Move stably on the terrain
2. Pass all obstacles
3. Reach the target position (x=30.0m)

### Success Criteria
- **Primary Objective**: Agent's chassis must reach position x=30.0m
- **Secondary Constraint**: Agent cannot fall off the map (y < -10)
- **Design Constraint**: Agent cannot move backward too much (x < start_x - 5)
- **Stability Constraint**: Agent must move stably on the terrain
  - Angular velocity must remain below 2.0 rad/s
  - Altitude must remain below 8.0m
  - **Cannot rotate more than 180 degrees while airborne** (agent cannot flip/spin excessively in the air)

---

## Step 1: Physical Analysis

1. **Understand the Physics**: This task involves designing a vehicle that can traverse terrain with obstacles. Key physics principles:
   - **Kinematics**: Vehicle must move forward (x-direction) from start (5.0m) to target (30.0m)
   - **Dynamics**: Wheels provide traction and motors provide torque to overcome friction and climb slopes
   - **Geometry**: Wheel radius determines obstacle clearance (obstacles are 2m and 3m high)
   - **Stability**: Vehicle must maintain stable orientation (angular velocity < 2.0 rad/s, no excessive rotation while airborne)

2. **Design Strategy**:
   - Use 2 wheels (maximum allowed) for stability and traction
   - Wheel radius should be large enough (≥1.5m) to clear obstacles (2m and 3m height)
   - Motor speed and torque must be sufficient to overcome friction and climb slopes
   - Chassis should be low and wide for stability

3. **Parameter Reasoning**:
   - **Wheel radius**: 1.5m provides good clearance for 2m obstacle, adequate for 3m obstacle
   - **Motor speed**: -6.0 rad/s provides sufficient forward force
   - **Motor torque**: 1800.0 N·m provides power to overcome friction and climb slopes
   - **Chassis**: Width 5.0m, height 0.4m (within 1.0m limit), positioned above wheels

## Step 2: Code

```python
def build_agent(sandbox):
    GROUND_TOP = 1.0
    WHEEL_RADIUS = 1.5
    wheel_y = GROUND_TOP + WHEEL_RADIUS
    chassis = sandbox.add_beam(x=5.0, y=wheel_y + 0.2, width=5.0, height=0.4, density=3.0)
    wheel1 = sandbox.add_wheel(x=3.2, y=wheel_y, radius=WHEEL_RADIUS, friction=4.0, density=1.0)
    wheel2 = sandbox.add_wheel(x=6.8, y=wheel_y, radius=WHEEL_RADIUS, friction=4.0, density=1.0)
    sandbox.connect(chassis, wheel1, anchor_x=3.2, anchor_y=wheel_y, motor_speed=-6.0, max_torque=1800.0)
    sandbox.connect(chassis, wheel2, anchor_x=6.8, anchor_y=wheel_y, motor_speed=-6.0, max_torque=1800.0)
    return chassis

def agent_action(sandbox, agent_body, step_count):
    pass
```

**Result**: This design successfully reaches the target position (x=30.0m) with score 100/100.

---

# Example 2: Control-Aware Task Solution

## Task Description

You need to design a speed-controlled slider system in the Sandbox.

### Task Environment
- **Track**: A horizontal track from x=0m to x=30m at height y=3.0m
  - Track width: 0.3m
  - Track is frictionless (slider moves smoothly)
- **Slider**: A movable slider on the track
  - Slider starts at x=0m
  - Slider must reach x=30m (target position)
  - Slider can move horizontally along the track
- **Speed Limit Zones** (CRITICAL - Must be enforced):
  - **Zone 1** (x: 0m to 10m): Maximum speed **1.5 m/s** (strict enforcement)
  - **Zone 2** (x: 10m to 20m): Maximum speed **3.0 m/s** (strict enforcement)
  - **Zone 3** (x: 20m to 30m): Maximum speed **2.0 m/s** (strict enforcement)
  - **Violation**: If slider speed exceeds the zone limit, the task fails immediately

### Task Objective
Design a control system that can:
1. Move the slider along the track from start (x=0m) to target (x=30m)
2. **Dynamically adjust slider speed** based on current position to comply with speed limits
3. Reach the target position without violating any speed limits

### Success Criteria
- **Primary Objective**: Slider must reach position x=30.0m
- **Speed Compliance**: Slider must never exceed speed limits in any zone
  - Zone 1 (0-10m): Speed ≤ 1.5 m/s
  - Zone 2 (10-20m): Speed ≤ 3.0 m/s
  - Zone 3 (20-30m): Speed ≤ 2.0 m/s
- **Constraint**: Slider cannot fall off track (y < 2.5m or y > 3.5m)
- **Constraint**: Slider cannot move backward (x < previous_max_x - 0.5m)

---

## Step 1: Physical Analysis

1. **Understand the Physics**: This task involves designing a control system for a slider. Key physics principles:
   - **Kinematics**: Slider must move forward (x-direction) from start (0m) to target (30m)
   - **Control**: Velocity must be dynamically adjusted based on position to comply with zone speed limits
   - **Constraints**: Speed limits are strictly enforced - exceeding limit causes immediate failure
   - **Feedback Control**: Must implement position-based feedback control in `agent_action()` function

2. **Design Strategy**:
   - Create slider at start position (x=0m)
   - Implement dynamic control in `agent_action()` that:
     - Gets current slider position
     - Determines which speed zone the slider is in
     - Sets velocity to comply with zone speed limit (with safety margin)
   - Use 95% of speed limit as target speed to ensure safety margin

3. **Parameter Reasoning**:
   - **Zone boundaries**: Match evaluator boundaries exactly (0-10m, 10-20m, 20-30m)
   - **Target speeds**: Use 95% of limit (Zone 1: 1.425 m/s, Zone 2: 2.85 m/s, Zone 3: 1.9 m/s)
   - **Safety margin**: 5% margin prevents numerical errors from causing violations
   - **Control frequency**: Called every simulation step, allowing real-time adjustment

## Step 2: Code

```python
def build_agent(sandbox):
    # Create slider at start position
    slider = sandbox.add_slider(x=0.0, y=sandbox.TRACK_Y, width=0.5, height=0.3, density=1.0)

    return {
        'slider': slider
    }

def agent_action(sandbox, agent_components, step_count):
    # Get slider
    slider = agent_components.get('slider')
    if not slider:
        return

    # Get slider state
    position_x, velocity_x = sandbox.get_slider_state(slider)

    # Determine target speed based on current zone
    if position_x < 0.0:
        # Before start - move forward slowly
        target_speed = 1.0
    elif 0.0 <= position_x < 10.0:
        # Zone 1: Speed limit 1.5 m/s - use low speed
        target_speed = 1.5 * 0.95  # 95% of limit for safety margin
    elif 10.0 <= position_x < 20.0:
        # Zone 2: Speed limit 3.0 m/s - can use higher speed
        target_speed = 3.0 * 0.95  # 95% of limit for safety margin
    elif 20.0 <= position_x < 30.0:
        # Zone 3: Speed limit 2.0 m/s - reduce speed
        target_speed = 2.0 * 0.95  # 95% of limit for safety margin
    else:
        # After target - stop
        target_speed = 0.0

    # Apply control - use direct velocity setting
    sandbox.set_slider_velocity(slider, target_speed)
```

**Result**: This design successfully reaches the target position (x=30.02m) with score 100/100, no speed violations.

---
