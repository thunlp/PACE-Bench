import Box2D

from Box2D.b2 import (world, polygonShape, circleShape, staticBody, dynamicBody, revoluteJoint)

import math

class DaVinciSandbox:
    def __init__(self, *, terrain_config=None, physics_config=None):
        terrain_config = terrain_config or {}
        physics_config = physics_config or {}
        self._terrain_config = dict(terrain_config)
        self._physics_config = dict(physics_config)
        gravity = tuple(physics_config.get("gravity", (0, -10)))
        self._default_linear_damping = float(physics_config.get("linear_damping", 0.0))
        self._default_angular_damping = float(physics_config.get("angular_damping", 0.0))
        self._world = world(gravity=gravity, doSleep=True)
        self._bodies = []
        self._joints = []
        self._terrain_bodies = {'obstacles': []}
        self._tracked_body = None
        self._last_tracked_angle = None
        self._airborne_rotation_clockwise = 0.0
        self._airborne_rotation_counterclockwise = 0.0
        self._airborne_rotation_exceeded = False
        self._AIRBORNE_THRESHOLD = 0.5
        self._MAX_AIRBORNE_ROTATION = 3.14159265359
        self.world = self._world
        self.bodies = self._bodies
        self.joints = self._joints
        self._create_terrain(terrain_config)
    def _set_body_friction(self, body, friction: float):
        try:
            for fixture in getattr(body, "fixtures", []) or []:
                fixture.friction = float(friction)
        except Exception:
            pass
    def _create_ground_segment(self, *, x_center: float, half_width: float, friction: float):
        body = self._world.CreateStaticBody(
            position=(x_center, 0.5),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(half_width, 0.5)),
                friction=float(friction),
            ),
        )
        return body
    def _create_terrain(self, terrain_config: dict):
        ground_friction = float(terrain_config.get("ground_friction", 0.8))
        obstacle_friction = float(terrain_config.get("obstacle_friction", ground_friction))
        gap = terrain_config.get("gap", None)
        if gap and isinstance(gap, dict) and "x_start" in gap and "x_end" in gap:
            x_start = float(gap["x_start"])
            x_end = float(gap["x_end"])
            left_half_width = max(0.01, x_start / 2.0)
            left_center = left_half_width
            ground_left = self._create_ground_segment(
                x_center=left_center, half_width=left_half_width, friction=ground_friction
            )
            right_start = x_end
            right_end = 100.0
            right_half_width = max(0.01, (right_end - right_start) / 2.0)
            right_center = right_start + right_half_width
            ground_right = self._create_ground_segment(
                x_center=right_center, half_width=right_half_width, friction=ground_friction
            )
            self.ground = ground_left
            self._terrain_bodies["ground_left"] = ground_left
            self._terrain_bodies["ground_right"] = ground_right
        else:
            self.ground = self._world.CreateStaticBody(
                position=(0, 0.5),
                fixtures=Box2D.b2FixtureDef(
                    shape=polygonShape(box=(50, 0.5)),
                    friction=float(ground_friction),
                ),
            )
        self._terrain_bodies["ground"] = self.ground
        obstacle_1 = terrain_config.get("obstacle_1", {"x": 15, "height": 2.0, "angle": 0.2})
        obstacle_2 = terrain_config.get("obstacle_2", {"x": 25, "height": 3.0, "angle": -0.3})
        obstacles = [obstacle_1, obstacle_2]
        created = []
        for obs in obstacles:
            x = float(obs.get("x", 0.0))
            height = float(obs.get("height", 2.0))
            angle = float(obs.get("angle", 0.0))
            if obs is obstacle_1:
                half_width = 2.0
            elif obs is obstacle_2:
                half_width = 3.0
            else:
                half_width = max(0.5, height)
            half_height = max(0.1, height / 2.0)
            y_center = self.GROUND_TOP + half_height
            body = self._world.CreateStaticBody(
                position=(x, y_center),
                angle=angle,
                fixtures=Box2D.b2FixtureDef(
                    shape=polygonShape(box=(half_width, half_height)),
                    friction=float(obstacle_friction),
                ),
            )
            created.append(body)
        self._terrain_bodies["obstacles"] = created
    GROUND_TOP = 1.0
    MIN_WHEEL_RADIUS = 0.3
    MAX_WHEEL_RADIUS = 2.0
    MAX_CHASSIS_HEIGHT = 1.0
    MAX_CONNECTION_DISTANCE = 5.0
    MAX_WHEELS = 2
    MAX_MOTOR_SPEED = 50.0
    MAX_MOTOR_TORQUE = 2000.0
    def add_beam(self, x, y, width, height, angle=0, density=1.0):
        body = self._world.CreateDynamicBody(
            position=(x, y),
            angle=angle,
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(width/2, height/2)),
                density=density,
                friction=0.5,
            )
        )
        body.linearDamping = self._default_linear_damping
        body.angularDamping = self._default_angular_damping
        self._bodies.append(body)
        return body
    def add_wheel(self, x, y, radius, density=1.0, friction=0.8):
        body = self._world.CreateDynamicBody(
            position=(x, y),
            fixtures=Box2D.b2FixtureDef(
                shape=circleShape(radius=radius),
                density=density,
                friction=friction,
            )
        )
        body.linearDamping = self._default_linear_damping
        body.angularDamping = self._default_angular_damping
        self._bodies.append(body)
        return body
    def connect(self, body_a, body_b, anchor_x, anchor_y, motor_speed=0.0, max_torque=0.0):
        anchor_world = (anchor_x, anchor_y)
        joint = self._world.CreateRevoluteJoint(
            bodyA=body_a,
            bodyB=body_b,
            anchor=anchor_world,
            enableMotor=(max_torque > 0),
            maxMotorTorque=max_torque,
            motorSpeed=motor_speed,
            collideConnected=False
        )
        self._joints.append(joint)
        return joint
    def step(self, time_step):
        self._world.Step(time_step, 10, 10)
        if self._tracked_body is not None and not self._airborne_rotation_exceeded:
            current_y = self._tracked_body.position.y
            current_angle = self._tracked_body.angle
            is_airborne = current_y > (self.GROUND_TOP + self._AIRBORNE_THRESHOLD)
            if is_airborne:
                if self._last_tracked_angle is not None:
                    angle_diff = current_angle - self._last_tracked_angle
                    angle_diff_normalized = ((angle_diff + math.pi) % (2 * math.pi)) - math.pi
                    if abs(angle_diff) < math.pi:
                        angle_diff_unwrapped = angle_diff
                    else:
                        angle_diff_unwrapped = angle_diff_normalized
                    if angle_diff_unwrapped > 1e-6:
                        self._airborne_rotation_counterclockwise += angle_diff_unwrapped
                    elif angle_diff_unwrapped < -1e-6:
                        self._airborne_rotation_clockwise += abs(angle_diff_unwrapped)
                    net_rotation = abs(self._airborne_rotation_counterclockwise - self._airborne_rotation_clockwise)
                    if net_rotation > self._MAX_AIRBORNE_ROTATION:
                        self._airborne_rotation_exceeded = True
            else:
                self._airborne_rotation_clockwise = 0.0
                self._airborne_rotation_counterclockwise = 0.0
            self._last_tracked_angle = current_angle
    def set_tracked_body(self, body):
        self._tracked_body = body
        self._last_tracked_angle = body.angle if body else None
        self._airborne_rotation_clockwise = 0.0
        self._airborne_rotation_counterclockwise = 0.0
        self._airborne_rotation_exceeded = False
    def get_airborne_rotation_status(self):
        net_rotation = abs(self._airborne_rotation_counterclockwise - self._airborne_rotation_clockwise)
        return {
            'accumulated': net_rotation,
            'exceeded': self._airborne_rotation_exceeded
        }
    def validate_design(self, chassis_body):
        errors = []
        wheels = [b for b in self._bodies if b != chassis_body and b.type == dynamicBody]
        if len(wheels) == 0:
            errors.append("Design must have at least one wheel")
        connected_wheels = set()
        for joint in self._joints:
            if joint.bodyA == chassis_body:
                connected_wheels.add(joint.bodyB)
            elif joint.bodyB == chassis_body:
                connected_wheels.add(joint.bodyA)
        unconnected_wheels = [w for w in wheels if w not in connected_wheels]
        if unconnected_wheels:
            errors.append(f"{len(unconnected_wheels)} wheels not connected to chassis")
        if len(self._joints) == 0:
            errors.append("Design must have at least one connection (joint)")
        is_valid = len(errors) == 0
        return is_valid, errors
    def get_agent_position(self, agent_body):
        return (agent_body.position.x, agent_body.position.y)
    def get_terrain_bounds(self):
        obstacle_1 = self._terrain_config.get("obstacle_1", {"x": 15, "height": 2, "angle": 0.2})
        obstacle_2 = self._terrain_config.get("obstacle_2", {"x": 25, "height": 3, "angle": -0.3})
        bounds = {
            "start": 0,
            "end": 50,
            "obstacles": [
                {"x": float(obstacle_1.get("x", 15)), "height": float(obstacle_1.get("height", 2)), "angle": float(obstacle_1.get("angle", 0.2))},
                {"x": float(obstacle_2.get("x", 25)), "height": float(obstacle_2.get("height", 3)), "angle": float(obstacle_2.get("angle", -0.3))},
            ],
        }
        gap = self._terrain_config.get("gap", None)
        if isinstance(gap, dict) and "x_start" in gap and "x_end" in gap:
            bounds["gap"] = {
                "x_start": float(gap["x_start"]),
                "x_end": float(gap["x_end"]),
                "depth": float(gap.get("depth", -10)),
            }
        return bounds
