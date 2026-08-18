import Box2D

from Box2D.b2 import (world, polygonShape, circleShape, staticBody, dynamicBody, revoluteJoint, weldJoint)

import math

class Sandbox:
    AGENT_COLLISION_GROUP = -1

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
        self._terrain_bodies = {}
        self._walker_bodies = {}
        self._walker_joints = []
        self.world = self._world
        self.bodies = self._bodies
        self.joints = self._joints
        self._create_terrain(terrain_config)
        self.BUILD_ZONE_X_MIN = 0.0
        self.BUILD_ZONE_X_MAX = 50.0
        self.BUILD_ZONE_Y_MIN = 2.0
        self.BUILD_ZONE_Y_MAX = 10.0
        self.MAX_STRUCTURE_MASS = float(terrain_config.get("max_structure_mass", 100.0))
        self.MAX_BODY_FRICTION = float(physics_config.get("max_body_friction", 1.0))
        self._create_initial_walker_template(terrain_config)
    def _create_terrain(self, terrain_config: dict):
        ground_friction = float(terrain_config.get("ground_friction", 0.8))
        ground_length = 100.0
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
    def _create_initial_walker_template(self, terrain_config: dict):
        spawn_x = 10.0
        spawn_y = 2.0
        torso_width = 0.5
        torso_height = 0.3
        torso = self._world.CreateDynamicBody(
            position=(spawn_x, spawn_y),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(torso_width/2, torso_height/2)),
                density=1.0,
                friction=0.5,
            )
        )
        torso.linearDamping = self._default_linear_damping
        torso.angularDamping = self._default_angular_damping
        self._walker_bodies["torso_template"] = torso
    MIN_BEAM_SIZE = 0.05
    MAX_BEAM_SIZE = 5.0
    MIN_WHEEL_RADIUS = 0.05
    MAX_WHEEL_RADIUS = 0.8
    MIN_JOINT_LIMIT = -math.pi
    MAX_JOINT_LIMIT = math.pi
    BUILD_ZONE_X_MIN = 0.0
    BUILD_ZONE_X_MAX = 50.0
    BUILD_ZONE_Y_MIN = 2.0
    BUILD_ZONE_Y_MAX = 10.0
    MAX_STRUCTURE_MASS = 100.0
    MAX_BODY_FRICTION = 1.0

    def _configure_agent_body(self, body):
        """Keep overlapping articulated parts from colliding with each other."""
        for fixture in body.fixtures:
            filter_data = fixture.filterData
            filter_data.groupIndex = self.AGENT_COLLISION_GROUP
            fixture.filterData = filter_data
        return body

    def add_beam(self, x, y, width, height, angle=0, density=1.0):
        if not (self.BUILD_ZONE_X_MIN <= x <= self.BUILD_ZONE_X_MAX and self.BUILD_ZONE_Y_MIN <= y <= self.BUILD_ZONE_Y_MAX):
            raise ValueError(
            )
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
        self._configure_agent_body(body)
        self._bodies.append(body)
        return body
    def add_wheel(self, x, y, radius=0.2, density=0.6):
        if not (self.BUILD_ZONE_X_MIN <= x <= self.BUILD_ZONE_X_MAX and self.BUILD_ZONE_Y_MIN <= y <= self.BUILD_ZONE_Y_MAX):
            raise ValueError(
            )
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
        self._configure_agent_body(body)
        self._bodies.append(body)
        return body
    def add_joint(self, body_a, body_b, anchor_point, type='pivot', lower_limit=None, upper_limit=None):
        if body_a is None:
            raise ValueError("add_joint: body_a cannot be None. You must provide a valid body object (e.g., from add_beam).")
        if body_b is None:
            raise ValueError("add_joint: body_b cannot be None. You must provide a valid body object.")
        anchor_x, anchor_y = anchor_point[0], anchor_point[1]
        if type == 'pivot' and lower_limit is None and upper_limit is None:
            def_lo = self._physics_config.get("default_joint_lower_limit")
            def_hi = self._physics_config.get("default_joint_upper_limit")
            if def_lo is not None and def_hi is not None:
                lower_limit = float(def_lo)
                upper_limit = float(def_hi)
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
        joint.motorEnabled = True
        joint.motorSpeed = float(motor_speed)
        joint.maxMotorTorque = float(max_torque)
    def get_structure_mass(self):
        total_mass = 0.0
        for body in self._bodies:
            total_mass += body.mass
        return total_mass
    def get_structure_mass_limit(self):
        return self.MAX_STRUCTURE_MASS
    def set_material_properties(self, body, restitution=0.2, friction=None):
        max_friction = self._physics_config.get("max_body_friction")
        for fixture in body.fixtures:
            fixture.restitution = float(restitution)
            if friction is not None:
                f = float(friction)
                if max_friction is not None:
                    f = min(f, float(max_friction))
                fixture.friction = f
    def set_fixed_rotation(self, body, fixed=True):
        if body:
            body.fixedRotation = bool(fixed)
    def get_environment_physics(self) -> dict:
        ground_friction = float(self._terrain_config.get("ground_friction", 0.8))
        gravity_x, gravity_y = self._physics_config.get("gravity", (0, -10))
        linear_damping = self._default_linear_damping
        angular_damping = self._default_angular_damping
        max_body_friction = float(self._physics_config.get("max_body_friction", 1.0))
        def_lo = self._physics_config.get("default_joint_lower_limit")
        def_hi = self._physics_config.get("default_joint_upper_limit")
        default_joint_lower = float(def_lo) if def_lo is not None else None
        default_joint_upper = float(def_hi) if def_hi is not None else None
        return {
            "ground_friction": ground_friction,
            "gravity_x": gravity_x,
            "gravity_y": gravity_y,
            "linear_damping": linear_damping,
            "angular_damping": angular_damping,
            "max_body_friction": max_body_friction,
            "default_joint_lower_limit": default_joint_lower,
            "default_joint_upper_limit": default_joint_upper,
        }
    def get_walker_body_positions(self) -> dict:
        if not self._bodies:
            return {"min_body_y": None, "min_clearance_above_ground": None}
        min_y = min(body.position.y for body in self._bodies)
        clearance = min_y - 1.0
        return {
            "min_body_y": min_y,
            "min_clearance_above_ground": clearance,
        }
    def get_walker_joints(self) -> list:
        result = []
        for joint in self._joints:
            if not isinstance(joint, Box2D.b2RevoluteJoint):
                continue
            limit_active = getattr(joint, 'limitEnabled', False)
            lo = joint.lowerLimit if limit_active else None
            hi = joint.upperLimit if limit_active else None
            result.append({
                "joint": joint,
                "anchor_x": joint.anchorA.x if hasattr(joint, 'anchorA') else 0.0,
                "anchor_y": joint.anchorA.y if hasattr(joint, 'anchorA') else 0.0,
                "lower_limit": lo,
                "upper_limit": hi,
                "current_angle": joint.angle if hasattr(joint, 'angle') else 0.0,
                "motor_speed": joint.motorSpeed if hasattr(joint, 'motorSpeed') else 0.0,
                "max_torque": joint.maxMotorTorque if hasattr(joint, 'maxMotorTorque') else 0.0,
            })
        return result
    def step(self, time_step):
        self._world.Step(time_step, 10, 10)
    def get_terrain_bounds(self):
        initial_x = float(self._terrain_config.get("initial_x", 10.0))
        target_distance = float(self._terrain_config.get("target_distance", 15.0))
        return {
            "ground": {"y": self._ground_y},
            "build_zone": {
                "x": [self.BUILD_ZONE_X_MIN, self.BUILD_ZONE_X_MAX],
                "y": [self.BUILD_ZONE_Y_MIN, self.BUILD_ZONE_Y_MAX]
            },
            "target_distance": target_distance,
            "initial_x": initial_x,
            "target_x": initial_x + target_distance,
        }
    def get_walker_position(self):
        if not self._bodies:
            return None
        body = self._bodies[0]
        return (body.position.x, body.position.y)
    def get_body_details(self) -> list:
        result = []
        for i, body in enumerate(self._bodies):
            info = {
                'index': i,
                'x': float(body.position.x),
                'y': float(body.position.y),
                'vx': float(body.linearVelocity.x),
                'vy': float(body.linearVelocity.y),
                'angular_velocity': float(body.angularVelocity),
                'mass': float(body.mass),
                'angle': float(body.angle),
            }
            for fixture in body.fixtures:
                shape = fixture.shape
                try:
                    radius = shape.radius
                    info['type'] = 'wheel'
                    info['radius'] = float(radius)
                except (AttributeError, TypeError):
                    info['type'] = 'beam'
                    try:
                        vertices = shape.vertices
                        if vertices:
                            xs = [v.x for v in vertices]
                            ys = [v.y for v in vertices]
                            info['width'] = float(max(xs) - min(xs))
                            info['height'] = float(max(ys) - min(ys))
                    except (AttributeError, TypeError):
                        pass
            result.append(info)
        return result
    def get_wheel_ground_clearances(self) -> list:
        result = []
        ground_y = self._ground_y
        for i, body in enumerate(self._bodies):
            for fixture in body.fixtures:
                shape = fixture.shape
                try:
                    radius = shape.radius
                    wheel_bottom = float(body.position.y) - float(radius)
                    clearance = wheel_bottom - ground_y
                    result.append({
                        'body_index': i,
                        'wheel_center_y': float(body.position.y),
                        'wheel_radius': float(radius),
                        'wheel_bottom_y': wheel_bottom,
                        'ground_clearance': clearance,
                        'touching_ground': clearance <= 0.001,
                    })
                except (AttributeError, TypeError):
                    pass
        return result
    def get_joint_details(self, inv_dt=None) -> list:
        if inv_dt is None:
            inv_dt = 60.0
        result = []
        for i, joint in enumerate(self._joints):
            info = {'index': i}
            if isinstance(joint, Box2D.b2RevoluteJoint):
                info['type'] = 'revolute'
                info['anchor_x'] = float(joint.anchorA.x) if hasattr(joint, 'anchorA') else 0.0
                info['anchor_y'] = float(joint.anchorA.y) if hasattr(joint, 'anchorA') else 0.0
                info['current_angle'] = float(joint.angle) if hasattr(joint, 'angle') else 0.0
                info['motor_speed'] = float(joint.motorSpeed) if hasattr(joint, 'motorSpeed') else 0.0
                info['max_torque'] = float(joint.maxMotorTorque) if hasattr(joint, 'maxMotorTorque') else 0.0
                info['motor_enabled'] = bool(joint.motorEnabled) if hasattr(joint, 'motorEnabled') else False
                if joint.limitEnabled:
                    info['lower_limit'] = float(joint.lowerLimit)
                    info['upper_limit'] = float(joint.upperLimit)
                else:
                    info['lower_limit'] = None
                    info['upper_limit'] = None
                try:
                    info['motor_torque'] = float(joint.GetMotorTorque(float(inv_dt)))
                except Exception:
                    info['motor_torque'] = None
            else:
                info['type'] = 'weld'
            result.append(info)
        return result
