# Example: Revision Process

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

## Previous Attempt (with issues)

```python
def build_agent(sandbox):
    GROUND_TOP = 1.0
    WHEEL_RADIUS = 1.0  # Too small - reduces obstacle clearance
    wheel_y = GROUND_TOP + WHEEL_RADIUS
    chassis = sandbox.add_beam(x=5.0, y=wheel_y + 0.2, width=5.0, height=0.4, density=3.0)
    wheel1 = sandbox.add_wheel(x=3.2, y=wheel_y, radius=WHEEL_RADIUS, friction=4.0, density=1.0)
    wheel2 = sandbox.add_wheel(x=6.8, y=wheel_y, radius=WHEEL_RADIUS, friction=4.0, density=1.0)
    sandbox.connect(chassis, wheel1, anchor_x=3.2, anchor_y=wheel_y, motor_speed=-3.0, max_torque=1000.0)
    sandbox.connect(chassis, wheel2, anchor_x=6.8, anchor_y=wheel_y, motor_speed=-3.0, max_torque=1000.0)
    return chassis

def agent_action(sandbox, agent_body, step_count):
    pass
```

## Evaluation Feedback

## Iteration 1 Evaluation Results

❌ **Task failed**: Vehicle rotated 180.3° while airborne (exceeds 180° limit)

**Score**: 0.0/100

## Task Execution Results

**Distance traveled**: 14.84m
**Current position**: x=19.84m, y=2.76m
**Target position**: x=30.00m
**Progress**: 59.4%
**Maximum distance reached**: 15.75m
**Simulation steps**: 700

**Physical State Information**:
- Agent position: (19.838, 2.759)
- Agent velocity: 4.058 m/s
- Agent velocity components: vx=0.647 m/s, vy=-4.006 m/s
- Agent angular velocity: 1.209 rad/s
- Agent angle: 3.718 rad (213.0°)

**Additional Metrics**:
- is_airborne: True
- airborne_rotation_accumulated: 3.147 rad (180.3°) - net rotation (true flip)

## Step 1: Physical Diagnosis (Required)

### 1. Interpret Metrics
- Distance traveled: 14.84m (target: 25.0m)
- Progress: 59.4%
- Final position: x=19.84m (target: 30.0m)
- Final velocity: 4.058 m/s
- Angular velocity: 1.209 rad/s
- Airborne rotation accumulated: 3.147 rad (180.3°) - this is the **net rotation** (absolute difference between clockwise and counterclockwise rotations), indicating the vehicle has truly flipped

### 2. Identify the Physical Problem
The vehicle failed due to excessive rotation while airborne. The metrics show:
- **Stability violation**: Vehicle rotated 180.3° while airborne (net rotation), exceeding the 180° limit
- **Root cause**: The combination of small wheel radius (1.0m) and low motor power (speed -3.0 rad/s, torque 1000.0 N·m) causes the vehicle to launch into the air when hitting obstacles
- **Physical mechanism**: When the vehicle hits an obstacle with insufficient power and small wheels, it gets launched upward. While airborne, the low motor torque cannot maintain stable orientation, causing the vehicle to flip (net rotation exceeds 180°, meaning the vehicle has truly flipped - rear wheels have passed the front wheels)
- **Note on rotation tracking**: The system tracks net rotation (the absolute difference between clockwise and counterclockwise rotations). If the vehicle rotates CCW then CW back, they cancel out. Only when the net rotation exceeds 180° does it indicate a true flip.
- **Evidence**: Vehicle reached x=19.84m (59% progress) but failed due to instability (flipping), not lack of progress

### 3. Propose Fix
To address these physical issues:
1. **Increase wheel radius** from 1.0m to 1.5m: Larger wheels provide better geometric clearance and reduce the likelihood of launching into the air when hitting obstacles
2. **Increase motor speed** from -3.0 rad/s to -6.0 rad/s: Higher angular velocity generates more forward force, allowing smoother obstacle traversal without launching
3. **Increase motor torque** from 1000.0 N·m to 1800.0 N·m: Higher torque provides more power to maintain stable motion and prevent excessive rotation, especially when airborne

**Why this works**: Larger wheels reduce the impact angle when hitting obstacles, preventing launch. Higher motor parameters provide sufficient force to maintain forward motion and stability, preventing excessive rotation even if the vehicle briefly becomes airborne.

## Step 2: Fixed Code

```python
def build_agent(sandbox):
    GROUND_TOP = 1.0
    WHEEL_RADIUS = 1.5  # Fixed: increased from 1.0m
    wheel_y = GROUND_TOP + WHEEL_RADIUS
    chassis = sandbox.add_beam(x=5.0, y=wheel_y + 0.2, width=5.0, height=0.4, density=3.0)
    wheel1 = sandbox.add_wheel(x=3.2, y=wheel_y, radius=WHEEL_RADIUS, friction=4.0, density=1.0)
    wheel2 = sandbox.add_wheel(x=6.8, y=wheel_y, radius=WHEEL_RADIUS, friction=4.0, density=1.0)
    sandbox.connect(chassis, wheel1, anchor_x=3.2, anchor_y=wheel_y, motor_speed=-6.0, max_torque=1800.0)  # Fixed: increased speed and torque
    sandbox.connect(chassis, wheel2, anchor_x=6.8, anchor_y=wheel_y, motor_speed=-6.0, max_torque=1800.0)  # Fixed: increased speed and torque
    return chassis

def agent_action(sandbox, agent_body, step_count):
    pass
```

**Result**: Fixed design successfully reaches the target position (x=30.0m) with score 100/100.

---

# Example 2: Control-Aware Task Revision

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

## Previous Attempt (with issues)

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

    # WRONG: Using fixed speed (2.5 m/s) for all zones
    # This violates Zone 1 limit (1.5 m/s) and Zone 3 limit (2.0 m/s)
    fixed_speed = 2.5  # Too fast for Zone 1 and Zone 3

    # Apply control
    sandbox.set_slider_velocity(slider, fixed_speed)
```

## Evaluation Feedback

## Iteration 1 Evaluation Results

❌ **Task failed**: Speed limit violated in Zone 1: speed 2.50 m/s exceeds limit 1.50 m/s

**Score**: 0.0/100

## Task Execution Results

**Distance traveled**: 4.17m
**Current position**: x=4.17m
**Target position**: x=30.00m
**Progress**: 13.9%
**Maximum distance reached**: 4.17m

**Speed Zone Information**:
- Current zone: Zone 1
- Speed limit: 1.50 m/s
- Current speed: 2.50 m/s
- ⚠️ **SPEED LIMIT VIOLATED**
- Total speed violations: 2

## Step 1: Physical Diagnosis (Required)

### 1. Interpret Metrics
- Distance traveled: 4.17m (target: 30.0m)
- Progress: 13.9%
- Final position: x=4.17m (target: 30.0m)
- Final velocity: 2.50 m/s
- Speed violation: True (2 violations detected)
- Current zone: Zone 1 (speed limit 1.50 m/s)

### 2. Identify the Physical Problem
The control system failed due to speed limit violation. The metrics show:
- **Speed limit violation**: Slider speed (2.50 m/s) exceeds Zone 1 limit (1.50 m/s)
- **Root cause**: Using fixed speed (2.5 m/s) for all zones instead of dynamic control
  - Fixed speed violates Zone 1 limit (1.5 m/s) - slider speed is 67% over limit
  - Fixed speed would also violate Zone 3 limit (2.0 m/s) if slider reached that zone
  - **Critical issue**: No position-based feedback control - speed is not adjusted based on slider position
- **Physical mechanism**: The task requires dynamic control that adjusts speed as slider moves through different zones. Fixed speed cannot comply with varying speed limits across zones.

### 3. Propose Fix
To address these physical issues:
1. **Implement dynamic control**: Replace fixed speed with position-based speed control in `agent_action()`
2. **Get current position**: Use `sandbox.get_slider_state(slider)` to get current x position
3. **Determine zone**: Check which speed zone the slider is in based on position
4. **Set zone-appropriate speed**: Set velocity to comply with zone limit (use 95% of limit for safety margin)
   - Zone 1 (0-10m): Set speed to 1.425 m/s (95% of 1.5 m/s)
   - Zone 2 (10-20m): Set speed to 2.85 m/s (95% of 3.0 m/s)
   - Zone 3 (20-30m): Set speed to 1.9 m/s (95% of 2.0 m/s)

**Why this works**: Dynamic control allows the slider to adjust speed in real-time based on position. The 95% safety margin prevents numerical errors from causing violations. Position-based feedback ensures compliance with zone-specific speed limits.

## Step 2: Fixed Code

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
    # Fixed: Implement dynamic control based on position
    if position_x < 0.0:
        # Before start - move forward slowly
        target_speed = 1.0
    elif 0.0 <= position_x < 10.0:
        # Zone 1: Speed limit 1.5 m/s - use low speed
        target_speed = 1.5 * 0.95  # Fixed: 95% of limit for safety margin
    elif 10.0 <= position_x < 20.0:
        # Zone 2: Speed limit 3.0 m/s - can use higher speed
        target_speed = 3.0 * 0.95  # Fixed: 95% of limit for safety margin
    elif 20.0 <= position_x < 30.0:
        # Zone 3: Speed limit 2.0 m/s - reduce speed
        target_speed = 2.0 * 0.95  # Fixed: 95% of limit for safety margin
    else:
        # After target - stop
        target_speed = 0.0

    # Apply control - use direct velocity setting
    sandbox.set_slider_velocity(slider, target_speed)
```

**Result**: Fixed design successfully reaches the target position (x=30.02m) with score 100/100, no speed violations.

---
