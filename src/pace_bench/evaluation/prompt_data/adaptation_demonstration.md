# Example: Environment Change Adaptation

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

## Previous Successful Code (worked in the Original Environment)

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

## Test Result in Original Environment

✅ **Task completed successfully!**

**Score**: 100.0/100

**Distance traveled**: 37.43m
**Current position**: x=42.43m, y=2.71m
**Target position**: x=30.00m
**Progress**: 100.0%

---

# Example : Environment Change

## ⚠️ CRITICAL: The Physical Environment Has Changed

The physics environment has been modified. Your previously successful design NO LONGER WORKS.
You must **infer what changed** from the feedback and adapt.

## Feedback from Running Previous Code in the NEW Environment

## Iteration 1 Evaluation Results

⚠️ **Task not completed**

**Score**: 24.8/100

## Task Execution Results

**Distance traveled**: 7.75m
**Current position**: x=12.75m, y=3.64m
**Target position**: x=30.00m
**Progress**: 31.0%
**Maximum distance reached**: 7.75m
**Simulation steps**: 588

**Physical State Information**:
- Agent position: (12.751, 3.639)
- Agent velocity: 0.000 m/s
- Agent velocity components: vx=0.000 m/s, vy=-0.000 m/s
- Agent angular velocity: -0.000 rad/s
- Agent angle: 0.565 rad (32.4°)

**Additional Metrics**:
- is_airborne: True
- airborne_rotation_accumulated: 0.339 rad

## Step 1: Environment Change Diagnosis (Required)

### 1. Compare Expected vs Actual
The code worked successfully in the original environment (reached target, score 100/100).
However, in the new environment, the same code failed or performed poorly.

### 2. Hypothesize
Based on the feedback, the physical environment has changed. Key observations:
- Vehicle only traveled 7.75m (target: 25.0m), far less than expected
- Vehicle appears stuck or unable to maintain forward motion
- **Likely cause**: Ground friction has decreased significantly
- Low friction causes wheels to slip, especially with high motor torque and speed
- High motor speed (-6.0 rad/s) with low ground friction causes excessive wheel spin, preventing forward progress

### 3. Plan Adaptation
To adapt to low friction environment:
1. **Increase wheel friction** from 4.0 to 5.0: Higher friction provides better traction on slippery surfaces
2. **Reduce motor speed** from -6.0 to -3.2 rad/s: Lower speed prevents wheel slip in low friction conditions
3. **Increase wheel radius** from 1.5m to 1.8m: Larger wheels provide better contact area and stability
4. **Widen wheelbase** (1.5 to 8.5m): Wider wheelbase increases stability on slippery surfaces
5. **Lower chassis** (height 0.2m, y=wheel_y+0.0): Lower center of mass improves stability
6. **Adjust density** (chassis 1.5, wheels 1.0): Balanced weight distribution for stability

**Why this works**: In low friction environments, high torque causes wheel slip. Lower speed with higher wheel friction provides better traction. Wider wheelbase and lower center of mass prevent sliding and improve stability.

## Step 2: Adapted Code

```python
def build_agent(sandbox):
    GROUND_TOP = 1.0
    WHEEL_RADIUS = 1.8  # Larger wheels for better obstacle clearance
    wheel_y = GROUND_TOP + WHEEL_RADIUS
    # Very wide wheelbase (1.5 to 8.5) and low chassis for maximum stability
    # Balanced density for stability
    chassis = sandbox.add_beam(x=5.0, y=wheel_y + 0.0, width=7.5, height=0.2, density=1.5)
    wheel1 = sandbox.add_wheel(x=1.5, y=wheel_y, radius=WHEEL_RADIUS, friction=5.0, density=1.0)
    wheel2 = sandbox.add_wheel(x=8.5, y=wheel_y, radius=WHEEL_RADIUS, friction=5.0, density=1.0)
    # Safe speed to prevent rotation, maximum torque to overcome obstacles
    sandbox.connect(chassis, wheel1, anchor_x=1.5, anchor_y=wheel_y, motor_speed=-3.2, max_torque=2000.0)
    sandbox.connect(chassis, wheel2, anchor_x=8.5, anchor_y=wheel_y, motor_speed=-3.2, max_torque=2000.0)
    return chassis

def agent_action(sandbox, agent_body, step_count):
    pass
```

## Test Result in New Environment (with Adapted Code)

✅ **Task completed successfully!**

**Score**: 100.0/100

**Distance traveled**: 29.49m
**Current position**: x=34.49m, y=2.93m
**Target position**: x=30.00m
**Progress**: 100.0%

---
