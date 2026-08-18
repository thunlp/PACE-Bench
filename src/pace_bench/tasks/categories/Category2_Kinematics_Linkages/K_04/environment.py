import Box2D

from Box2D.b2 import (world, polygonShape, circleShape, staticBody, dynamicBody, revoluteJoint, weldJoint)

import math

class Sandbox:
    def __init__(self, *, terrain_config=None, physics_config=None):
        terrain_config = terrain_config or {}
        physics_config = physics_config or {}
        self._terrain_config = dict(terrain_config)
        self._physics_config = dict(physics_config)
        gravity = tuple(physics_config.get("gravity", (0, -10)))
        self._default_linear_damping = float(physics_config.get("linear_damping", 0.0))
        self._default_angular_damping = float(physics_config.get("angular_damping", 0.0))
        self._world = world(gravity=gravity, doSleep=physics_config.get("do_sleep", False))
        self._bodies = []
        self._joints = []
        self._terrain_bodies = {}
        self._pusher_bodies = {}
        self._pusher_joints = []
        self._object_to_push = None
        self._pusher_initial_velocity_applied = False
        self.world = self._world
        self.bodies = self._bodies
        self.joints = self._joints
        self._create_terrain(terrain_config)
        bz = terrain_config.get("build_zone", {})
        bz_x = bz.get("x", [0.0, 15.0])
        bz_y = bz.get("y", [1.5, 8.0])
        self.BUILD_ZONE_X_MIN = float(bz_x[0]) if isinstance(bz_x, (list, tuple)) and len(bz_x) >= 1 else 0.0
        self.BUILD_ZONE_X_MAX = float(bz_x[1]) if isinstance(bz_x, (list, tuple)) and len(bz_x) >= 2 else 15.0
        self.BUILD_ZONE_Y_MIN = float(bz_y[0]) if isinstance(bz_y, (list, tuple)) and len(bz_y) >= 1 else 1.5
        self.BUILD_ZONE_Y_MAX = float(bz_y[1]) if isinstance(bz_y, (list, tuple)) and len(bz_y) >= 2 else 8.0
        self.MAX_STRUCTURE_MASS = float(terrain_config.get("max_structure_mass", 40.0))
        self._create_object(terrain_config)
        self._create_initial_pusher_template(terrain_config)
    def _create_terrain(self, terrain_config: dict):
        ground_friction = float(terrain_config.get("ground_friction", 1.2))
        ground_length = 50.0
        ground_height = 1.0
        ground = self._world.CreateStaticBody(
            position=(ground_length / 2, ground_height / 2),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(ground_length / 2, ground_height / 2)),
                friction=ground_friction,
            ),
        )
        self._terrain_bodies["ground"] = ground
        self._ground_y = ground_height
        self._ground_friction = ground_friction
    def _create_object(self, terrain_config: dict):
        object_config = terrain_config.get("object", {})
        object_mass = float(object_config.get("mass", 50.0))
        object_friction = float(object_config.get("friction", 0.8))
        object_x = 8.0
        width = 1.0
        height = 0.8
        object_y = self._ground_y + height / 2
        density = object_mass / (width * height)
        obj = self._world.CreateDynamicBody(
            position=(object_x, object_y),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(width/2, height/2)),
                density=density,
                friction=object_friction,
            )
        )
        obj.linearDamping = float(object_config.get("linear_damping", self._default_linear_damping))
        obj.angularDamping = float(object_config.get("angular_damping", self._default_angular_damping))
        com_offset = object_config.get("center_of_mass_offset")
        if com_offset is not None and len(com_offset) >= 2:
            md = obj.massData
            new_md = Box2D.b2MassData()
            new_md.mass = md.mass
            new_md.center = (float(com_offset[0]), float(com_offset[1]))
            new_md.I = md.I
            obj.massData = new_md
        self._object_to_push = obj
        self._terrain_bodies["object"] = obj
    def _create_initial_pusher_template(self, terrain_config: dict):
        spawn_x = 3.0
        spawn_y = 2.5
        body_width = 0.4
        body_height = 0.4
        body = self._world.CreateDynamicBody(
            position=(spawn_x, spawn_y),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(body_width/2, body_height/2)),
                density=1.0,
                friction=0.5,
            )
        )
        body.linearDamping = self._default_linear_damping
        body.angularDamping = self._default_angular_damping
        self._pusher_bodies["body_template"] = body
    def remove_initial_template(self):
        if "body_template" in self._pusher_bodies:
            body = self._pusher_bodies.pop("body_template")
            if body and self._world:
                self._world.DestroyBody(body)
    MIN_BEAM_SIZE = 0.05
    MAX_BEAM_SIZE = 3.0
    MIN_WHEEL_RADIUS = 0.05
    MAX_WHEEL_RADIUS = 0.8
    MIN_JOINT_LIMIT = -math.pi
    MAX_JOINT_LIMIT = math.pi
    BUILD_ZONE_X_MIN = 0.0
    BUILD_ZONE_X_MAX = 15.0
    BUILD_ZONE_Y_MIN = 1.5
    BUILD_ZONE_Y_MAX = 8.0
    MAX_STRUCTURE_MASS = 40.0
    def add_beam(self, x, y, width, height, angle=0, density=1.0):
        width = max(self.MIN_BEAM_SIZE, min(width, self.MAX_BEAM_SIZE))
        height = max(self.MIN_BEAM_SIZE, min(height, self.MAX_BEAM_SIZE))
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
    def add_wheel(self, x, y, radius=0.2, density=0.6):
        radius = max(self.MIN_WHEEL_RADIUS, min(radius, self.MAX_WHEEL_RADIUS))
        body = self._world.CreateDynamicBody(
            position=(x, y),
            fixtures=Box2D.b2FixtureDef(
                shape=circleShape(radius=radius),
                density=density,
                friction=0.8,
            )
        )
        body.linearDamping = self._default_linear_damping
        body.angularDamping = self._default_angular_damping
        self._bodies.append(body)
        return body
    def add_joint(self, body_a, body_b, anchor_point, type='pivot', lower_limit=None, upper_limit=None):
        if body_a is None:
            raise ValueError("add_joint: body_a cannot be None. You must provide a valid body object (e.g., from add_beam).")
        if body_b is None:
            raise ValueError("add_joint: body_b cannot be None. You must provide a valid body object.")
        anchor_x, anchor_y = anchor_point[0], anchor_point[1]
        if type == 'rigid':
            joint = self._world.CreateWeldJoint(
                bodyA=body_a,
                bodyB=body_b,
                anchor=(anchor_x, anchor_y),
                collideConnected=False
            )
        elif type == 'pivot':
            joint_kwargs = {
                'bodyA': body_a,
                'bodyB': body_b,
                'anchor': (anchor_x, anchor_y),
                'collideConnected': False
            }
            if lower_limit is not None and upper_limit is not None:
                joint_kwargs['lowerAngle'] = max(self.MIN_JOINT_LIMIT, min(lower_limit, self.MAX_JOINT_LIMIT))
                joint_kwargs['upperAngle'] = min(self.MAX_JOINT_LIMIT, max(upper_limit, self.MIN_JOINT_LIMIT))
                joint_kwargs['enableLimit'] = True
            joint = self._world.CreateRevoluteJoint(**joint_kwargs)
        else:
            raise ValueError(f"Unknown joint type: {type}")
        self._joints.append(joint)
        return joint
    def set_motor(self, joint, motor_speed, max_torque=100.0):
        if not isinstance(joint, Box2D.b2RevoluteJoint):
            raise ValueError("set_motor: joint must be a pivot/revolute joint")
        if float(max_torque) > 100.0:
            raise ValueError(f"set_motor: max_torque {max_torque} N·m exceeds the permitted maximum of 100 N·m")
        joint.enableMotor = True
        joint.motorSpeed = float(motor_speed)
        joint.maxMotorTorque = float(max_torque)
    def get_structure_mass(self):
        total_mass = 0.0
        for body in self._bodies:
            total_mass += body.mass
        return total_mass
    def set_material_properties(self, body, restitution=0.2, friction=None):
        for fixture in body.fixtures:
            fixture.restitution = float(restitution)
            if friction is not None:
                fixture.friction = float(friction)
    def set_fixed_rotation(self, body, fixed=True):
        if body:
            body.fixedRotation = bool(fixed)
    def step(self, time_step):
        if not self._pusher_initial_velocity_applied and self._bodies:
            vx = self._terrain_config.get("pusher_initial_velocity_x")
            if vx is not None:
                v = (float(vx), 0.0)
                for b in self._bodies:
                    b.linearVelocity = v
                self._pusher_initial_velocity_applied = True
        self._world.Step(time_step, 10, 10)
    def get_terrain_bounds(self):
        return {
            "ground": {"y": self._ground_y, "friction": self._ground_friction},
            "build_zone": {
                "x": [self.BUILD_ZONE_X_MIN, self.BUILD_ZONE_X_MAX],
                "y": [self.BUILD_ZONE_Y_MIN, self.BUILD_ZONE_Y_MAX]
            },
            "target_distance": float(self._terrain_config.get("target_distance", 10.0)),
        }
    def get_pusher_position(self):
        if not self._bodies:
            return None
        if self._bodies:
            body = self._bodies[0]
            return (body.position.x, body.position.y)
        return None
    def get_object_position(self):
        if self._object_to_push:
            return (self._object_to_push.position.x, self._object_to_push.position.y)
        return None
