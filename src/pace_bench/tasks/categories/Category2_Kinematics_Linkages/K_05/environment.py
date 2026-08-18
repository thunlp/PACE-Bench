OBJECT_START_Y = 1.8

OBJECT_START_X = 4.0

LIFTING_THRESHOLD_M = 0.5

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
        self._world = world(gravity=gravity, doSleep=True)
        self._bodies = []
        self._joints = []
        self._terrain_bodies = {}
        self._lifter_bodies = {}
        self._lifter_joints = []
        self._object_to_lift = None
        self.world = self._world
        self.bodies = self._bodies
        self.joints = self._joints
        self._create_terrain(terrain_config)
        self.BUILD_ZONE_X_MIN = 0.0
        self.BUILD_ZONE_X_MAX = 8.0
        self.BUILD_ZONE_Y_MIN = 1.0
        self.BUILD_ZONE_Y_MAX = 12.0
        self.MAX_STRUCTURE_MASS = float(terrain_config.get("max_structure_mass", 60.0))
        self.target_object_y = float(terrain_config.get("target_object_y", 9.0))
        self.min_sustain_s = float(terrain_config.get("min_sustain_s", 3.0))
        self._create_object(terrain_config)
        self._create_initial_lifter_template(terrain_config)
    def _create_terrain(self, terrain_config: dict):
        ground_friction = float(terrain_config.get("ground_friction", 0.8))
        ground_length = 20.0
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
        ceiling_config = terrain_config.get("ceiling_gap", None)
        if ceiling_config:
            c_y = ceiling_config.get("y", 6.0)
            c_x_min = ceiling_config.get("x_min", 3.0)
            c_x_max = ceiling_config.get("x_max", 5.0)
            thickness = 0.5
            left_width = c_x_min
            if left_width > 0:
                self._world.CreateStaticBody(
                    position=(left_width / 2, c_y + thickness / 2),
                    fixtures=Box2D.b2FixtureDef(
                        shape=polygonShape(box=(left_width / 2, thickness / 2)),
                        friction=ground_friction,
                    ),
                )
            right_width = ground_length - c_x_max
            if right_width > 0:
                self._world.CreateStaticBody(
                    position=(c_x_max + right_width / 2, c_y + thickness / 2),
                    fixtures=Box2D.b2FixtureDef(
                        shape=polygonShape(box=(right_width / 2, thickness / 2)),
                        friction=ground_friction,
                    ),
                )
    def _create_object(self, terrain_config: dict):
        object_config = terrain_config.get("object", {})
        object_mass = float(object_config.get("mass", 20.0))
        object_friction = float(object_config.get("friction", 0.6))
        com_offset = object_config.get("com_offset")
        if com_offset is not None:
            com_offset = (float(com_offset[0]), float(com_offset[1]))
        object_x = 4.0
        object_y = 1.8
        width, height = 0.6, 0.4
        hw, hh = width / 2.0, height / 2.0
        density = object_mass / (width * height)
        obj = self._world.CreateDynamicBody(
            position=(object_x, object_y),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(hw, hh)),
                density=density,
                friction=object_friction,
            )
        )
        obj.linearDamping = self._default_linear_damping
        obj.angularDamping = self._default_angular_damping
        if com_offset is not None and (com_offset[0] != 0.0 or com_offset[1] != 0.0):
            try:
                mass_data = obj.GetMassData()
                ox, oy = com_offset[0], com_offset[1]
                mass_data.center = (ox, oy)
                mass_data.I = mass_data.I + object_mass * (ox * ox + oy * oy)
                if mass_data.I <= 0:
                    mass_data.I = 0.01
                obj.SetMassData(mass_data)
            except Exception:
                pass
        self._object_to_lift = obj
        self._terrain_bodies["object"] = obj
    def _create_initial_lifter_template(self, terrain_config: dict):
        spawn_x = 4.0
        spawn_y = 2.0
        body_width = 0.3
        body_height = 0.3
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
        self._lifter_bodies["body_template"] = body
    def remove_initial_template(self):
        if "body_template" in self._lifter_bodies:
            body = self._lifter_bodies.pop("body_template")
            if body and self._world:
                self._world.DestroyBody(body)
    LIFTING_THRESHOLD_M = 0.5
    MIN_BEAM_SIZE = 0.05
    MAX_BEAM_SIZE = 4.0
    MIN_JOINT_LIMIT = -math.pi
    MAX_JOINT_LIMIT = math.pi
    BUILD_ZONE_X_MIN = 0.0
    BUILD_ZONE_X_MAX = 8.0
    BUILD_ZONE_Y_MIN = 1.0
    BUILD_ZONE_Y_MAX = 12.0
    MAX_STRUCTURE_MASS = 60.0
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
    def add_joint(self, body_a, body_b, anchor_point, type='pivot', lower_limit=None, upper_limit=None,
                  axis=None, lower_translation=None, upper_translation=None):
        if body_a is None:
            raise ValueError("add_joint: body_a cannot be None. You must provide a valid body object (e.g., from add_beam).")
        if body_b is None:
            body_b = self._terrain_bodies.get("ground")
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
        elif type == 'slider':
            ax = axis if axis is not None else (0, 1)
            lo = float(lower_translation) if lower_translation is not None else -10.0
            hi = float(upper_translation) if upper_translation is not None else 10.0
            joint = self._world.CreatePrismaticJoint(
                bodyA=body_a,
                bodyB=body_b,
                anchor=(anchor_x, anchor_y),
                axis=ax,
                lowerTranslation=lo,
                upperTranslation=hi,
                enableLimit=True,
                collideConnected=False
            )
        else:
            raise ValueError(f"Unknown joint type: {type}")
        self._joints.append(joint)
        return joint
    def set_motor(self, joint, motor_speed, max_torque=100.0):
        if not isinstance(joint, Box2D.b2RevoluteJoint):
            raise ValueError("set_motor: joint must be a pivot/revolute joint")
        joint.enableMotor = True
        joint.motorSpeed = float(motor_speed)
        joint.maxMotorTorque = float(max_torque)
    def set_slider_motor(self, joint, motor_speed, max_force=5000.0):
        if type(joint).__name__ != 'b2PrismaticJoint':
            raise ValueError("set_slider_motor: joint must be a slider/prismatic joint")
        joint.enableMotor = True
        joint.motorSpeed = float(motor_speed)
        joint.maxMotorForce = float(max_force)
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
    def apply_force(self, body, force_vector):
        if body and force_vector:
            body.ApplyForceToCenter(tuple(force_vector), True)
    def step(self, time_step):
        wind_force = self._physics_config.get("wind_force", (0.0, 0.0))
        if wind_force != (0.0, 0.0):
            for body in self._bodies:
                if body.type == Box2D.b2_dynamicBody:
                    body.ApplyForceToCenter(wind_force, True)
            if self._object_to_lift:
                self._object_to_lift.ApplyForceToCenter(wind_force, True)
        self._world.Step(time_step, 10, 10)
        max_joint_force_cfg = self._physics_config.get("max_joint_force", float('inf'))
        if not hasattr(self, '_peak_joint_reaction_force'):
            self._peak_joint_reaction_force = 0.0
        if not hasattr(self, '_joint_peak_forces'):
            self._joint_peak_forces = {}
        if not hasattr(self, '_joint_failure_events'):
            self._joint_failure_events = []
        if max_joint_force_cfg < float('inf'):
            joints_to_destroy = []
            inv_dt = 1.0 / time_step if time_step > 0 else 0.0
            peak_force_this_step = 0.0
            for joint in self._joints:
                force = joint.GetReactionForce(inv_dt)
                force_mag = (force.x**2 + force.y**2)**0.5
                if force_mag > peak_force_this_step:
                    peak_force_this_step = force_mag
                jid = id(joint)
                prev_peak = self._joint_peak_forces.get(jid, 0.0)
                if force_mag > prev_peak:
                    self._joint_peak_forces[jid] = force_mag
                if force_mag > max_joint_force_cfg:
                    joints_to_destroy.append((joint, force_mag))
            if peak_force_this_step > self._peak_joint_reaction_force:
                self._peak_joint_reaction_force = peak_force_this_step
            for joint, force_mag in joints_to_destroy:
                if joint in self._joints:
                    self._joints.remove(joint)
                    a_pos = (joint.bodyA.position.x, joint.bodyA.position.y) if joint.bodyA else (None, None)
                    b_pos = (joint.bodyB.position.x, joint.bodyB.position.y) if joint.bodyB else (None, None)
                    jtype = type(joint).__name__
                    self._joint_failure_events.append({
                        'joint_id': id(joint),
                        'joint_type': jtype,
                        'force': float(force_mag),
                        'limit': float(max_joint_force_cfg),
                        'body_a_pos': a_pos,
                        'body_b_pos': b_pos,
                    })
                    try:
                        self._world.DestroyJoint(joint)
                    except Exception:
                        pass
        else:
            inv_dt = 1.0 / time_step if time_step > 0 else 0.0
            peak_force_this_step = 0.0
            for joint in self._joints:
                force = joint.GetReactionForce(inv_dt)
                force_mag = (force.x**2 + force.y**2)**0.5
                if force_mag > peak_force_this_step:
                    peak_force_this_step = force_mag
                jid = id(joint)
                prev_peak = self._joint_peak_forces.get(jid, 0.0)
                if force_mag > prev_peak:
                    self._joint_peak_forces[jid] = force_mag
            if peak_force_this_step > self._peak_joint_reaction_force:
                self._peak_joint_reaction_force = peak_force_this_step
    def get_terrain_bounds(self):
        return {
            "ground": {"y": self._ground_y},
            "build_zone": {
                "x": [self.BUILD_ZONE_X_MIN, self.BUILD_ZONE_X_MAX],
                "y": [self.BUILD_ZONE_Y_MIN, self.BUILD_ZONE_Y_MAX]
            }
        }
    def get_lifter_position(self):
        if not self._bodies:
            return None
        if self._bodies:
            body = self._bodies[0]
            return (body.position.x, body.position.y)
        return None
    def get_object_position(self):
        if self._object_to_lift:
            return (self._object_to_lift.position.x, self._object_to_lift.position.y)
        return None
    def set_object_position(self, x, y):
        if self._object_to_lift:
            self._object_to_lift.position = (float(x), float(y))
            self._object_to_lift.linearVelocity = (0, 0)
            self._object_to_lift.angularVelocity = 0
    def enforce_object_at_ground(self):
        if self._terrain_config.get('skip_enforce_object_at_ground'):
            return
        if self._object_to_lift:
            self._object_to_lift.position = (OBJECT_START_X, OBJECT_START_Y)
            self._object_to_lift.linearVelocity = (0, 0)
            self._object_to_lift.angularVelocity = 0
    def set_object_damping(self, linear_damping=0.0, angular_damping=0.0):
        if self._object_to_lift:
            self._object_to_lift.linearDamping = float(linear_damping)
            self._object_to_lift.angularDamping = float(angular_damping)
    def weld_to_ground(self, body, anchor_point):
        ground = self._terrain_bodies.get("ground")
        if ground is None or body is None:
            return
        ax, ay = float(anchor_point[0]), float(anchor_point[1])
        joint = self._world.CreateWeldJoint(
            bodyA=ground,
            bodyB=body,
            anchor=(ax, ay),
            collideConnected=False
        )
        self._joints.append(joint)
    def get_target_height(self):
        return self.target_object_y
    def get_target_x(self):
        return OBJECT_START_X
    def get_peak_joint_reaction_force(self):
        return getattr(self, '_peak_joint_reaction_force', 0.0)
    def get_max_joint_force_limit(self):
        raw = self._physics_config.get("max_joint_force", float('inf'))
        return float(raw)
    def get_wind_force(self):
        wf = self._physics_config.get("wind_force", (0.0, 0.0))
        return float(wf[0]), float(wf[1])
    def get_ceiling_gap(self):
        return self._terrain_config.get("ceiling_gap")
    def get_object_config(self):
        obj_cfg = self._terrain_config.get("object", {})
        return {
            "mass": float(obj_cfg.get("mass", 20.0)),
            "friction": float(obj_cfg.get("friction", 0.6)),
            "com_offset": obj_cfg.get("com_offset"),
        }
    def get_object_linear_velocity(self):
        if self._object_to_lift:
            v = self._object_to_lift.linearVelocity
            return float(v.x), float(v.y)
        return None, None
    def get_lifter_platform_y(self):
        if not self._bodies:
            return None
        return max(b.position.y for b in self._bodies)
    def get_joint_peak_forces(self):
        return getattr(self, '_joint_peak_forces', {}).copy()
    def get_joint_failure_events(self):
        return list(getattr(self, '_joint_failure_events', []))
    def get_max_body_width(self):
        if not self._bodies:
            return None
        max_w = 0.0
        for body in self._bodies:
            for fixture in body.fixtures:
                try:
                    shape = fixture.shape
                    if hasattr(shape, 'vertices'):
                        vs = shape.vertices
                        xs = [v[0] for v in vs]
                        w = max(xs) - min(xs)
                        if w > max_w:
                            max_w = w
                except Exception:
                    pass
        return max_w if max_w > 0 else None
    def get_platform_object_offset(self):
        if not self._object_to_lift:
            return None, None, None
        ox = float(self._object_to_lift.position.x)
        oy = float(self._object_to_lift.position.y)
        lifter_top_y = None
        lifter_body_x = None
        for body in self._bodies:
            if lifter_top_y is None or body.position.y > lifter_top_y:
                lifter_top_y = float(body.position.y)
                lifter_body_x = float(body.position.x)
        if lifter_top_y is None:
            return None, None, None
        horizontal_offset = ox - lifter_body_x if lifter_body_x is not None else None
        vertical_offset = oy - lifter_top_y
        return horizontal_offset, vertical_offset, lifter_top_y
