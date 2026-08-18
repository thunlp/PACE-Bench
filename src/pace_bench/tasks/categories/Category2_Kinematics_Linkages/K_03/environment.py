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
        self._gripper_bodies = {}
        self._gripper_joints = []
        self._slider_joint = None
        self._slider_joint_anchor_y = None
        self._slider_body_initial_y = None
        self._objects_to_grasp = []
        self._last_time_step = None
        self._observation_error_count = 0
        self._last_observation_error = None
        self.world = self._world
        self.bodies = self._bodies
        self.joints = self._joints
        self._create_terrain(terrain_config)
        self.BUILD_ZONE_X_MIN = 0.0
        self.BUILD_ZONE_X_MAX = 10.0
        self.BUILD_ZONE_Y_MIN = 5.0
        self.BUILD_ZONE_Y_MAX = 15.0
        self.MAX_STRUCTURE_MASS = float(terrain_config.get("max_structure_mass", 30.0))
        self.TARGET_OBJECT_Y = float(terrain_config.get("target_object_y", 3.5))
        self.MIN_OBJECT_HEIGHT = float(terrain_config.get("min_object_height", 2.0))
        self.MIN_SIMULATION_TIME = float(terrain_config.get("min_simulation_time", 1.34))
        self._create_objects(terrain_config)
        self._create_initial_gripper_template(terrain_config)
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
        gantry_y = 10.0
        gantry_length = 4.0
        gantry = self._world.CreateStaticBody(
            position=(5.0, gantry_y),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(gantry_length / 2, 0.15)),
                friction=0.6,
            ),
        )
        self._terrain_bodies["gantry"] = gantry
        self._gantry_y = gantry_y
        object_config = terrain_config.get("objects", {})
        obj_x = float(object_config.get("x", 5.0))
        obj_y = float(object_config.get("y", 2.0))
        obj_h = 0.4
        platform_top = obj_y - obj_h / 2
        platform_h = 0.4
        platform = self._world.CreateStaticBody(
            position=(obj_x, platform_top - platform_h / 2),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(0.6, platform_h / 2)),
                friction=0.25,
            ),
        )
        self._terrain_bodies["platform"] = platform
    def _create_objects(self, terrain_config: dict):
        object_config = terrain_config.get("objects", {})
        object_shape = object_config.get("shape", "box")
        object_friction = float(object_config.get("friction", 0.6))
        object_mass = float(object_config.get("mass", 1.0))
        object_x = float(object_config.get("x", 5.0))
        object_y = float(object_config.get("y", 2.0))
        if object_shape == "box":
            width = 0.4
            height = 0.4
            density = object_mass / (width * height)
            obj = self._world.CreateDynamicBody(
                position=(object_x, object_y),
                fixtures=Box2D.b2FixtureDef(
                    shape=polygonShape(box=(width/2, height/2)),
                    density=density,
                    friction=object_friction,
                )
            )
        elif object_shape == "circle":
            radius = 0.25
            density = object_mass / (math.pi * radius * radius)
            obj = self._world.CreateDynamicBody(
                position=(object_x, object_y),
                fixtures=Box2D.b2FixtureDef(
                    shape=circleShape(radius=radius),
                    density=density,
                    friction=object_friction,
                )
            )
        else:
            vertices = [(-0.2, -0.2), (0.2, -0.2), (0.0, 0.2)]
            area = 0.5 * 0.4 * 0.4
            density = object_mass / area
            obj = self._world.CreateDynamicBody(
                position=(object_x, object_y),
                fixtures=Box2D.b2FixtureDef(
                    shape=polygonShape(vertices=vertices),
                    density=density,
                    friction=object_friction,
                )
            )
        obj.linearDamping = self._default_linear_damping
        obj.angularDamping = self._default_angular_damping
        self._objects_to_grasp.append(obj)
        self._terrain_bodies["object"] = obj
    def _create_initial_gripper_template(self, terrain_config: dict):
        spawn_x = 5.0
        spawn_y = 8.0
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
        self._gripper_bodies["body_template"] = body
    def remove_initial_template(self):
        if "body_template" in self._gripper_bodies:
            body = self._gripper_bodies.pop("body_template")
            if body and self._world:
                self._world.DestroyBody(body)
    MIN_BEAM_SIZE = 0.05
    MAX_BEAM_SIZE = 2.0
    MIN_JOINT_LIMIT = -math.pi
    MAX_JOINT_LIMIT = math.pi
    BUILD_ZONE_X_MIN = 0.0
    BUILD_ZONE_X_MAX = 10.0
    BUILD_ZONE_Y_MIN = 5.0
    BUILD_ZONE_Y_MAX = 15.0
    MAX_STRUCTURE_MASS = 30.0
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
    def add_joint(self, body_a, body_b, anchor_point, type='pivot', lower_limit=None, upper_limit=None, enable_motor=False, motor_speed=0.0, max_motor_torque=0.0,
                  axis=None, lower_translation=None, upper_translation=None, max_motor_force=0.0):
        if body_a is None or body_b is None:
            raise ValueError("add_joint: body_a and body_b cannot be None.")
        anchor_x, anchor_y = anchor_point[0], anchor_point[1]
        if type == 'rigid':
            joint = self._world.CreateWeldJoint(
                bodyA=body_a, bodyB=body_b, anchor=(anchor_x, anchor_y), collideConnected=False
            )
        elif type == 'slider':
            ax = axis if axis is not None else (0, -1)
            lo = float(lower_translation) if lower_translation is not None else 0.0
            hi = float(upper_translation) if upper_translation is not None else 8.0
            joint = self._world.CreatePrismaticJoint(
                bodyA=body_a,
                bodyB=body_b,
                anchor=(anchor_x, anchor_y),
                axis=ax,
                lowerTranslation=lo,
                upperTranslation=hi,
                enableLimit=True,
                enableMotor=bool(enable_motor),
                motorSpeed=float(motor_speed),
                maxMotorForce=float(max_motor_force) if max_motor_force else 5000.0,
            )
            self._slider_joint = joint
            self._slider_joint_anchor_y = float(anchor_y)
            self._slider_body_initial_y = float(body_b.position.y)
        elif type == 'pivot':
            joint_kwargs = {
                'bodyA': body_a, 'bodyB': body_b, 'anchor': (anchor_x, anchor_y),
                'collideConnected': False,
                'enableMotor': bool(enable_motor),
                'motorSpeed': float(motor_speed),
                'maxMotorTorque': float(max_motor_torque),
            }
            if lower_limit is not None and upper_limit is not None:
                joint_kwargs['lowerAngle'] = max(self.MIN_JOINT_LIMIT, min(lower_limit, self.MAX_JOINT_LIMIT))
                joint_kwargs['upperAngle'] = min(self.MAX_JOINT_LIMIT, max(upper_limit, self.MIN_JOINT_LIMIT))
                joint_kwargs['enableLimit'] = True
            joint = self._world.CreateRevoluteJoint(**joint_kwargs)
        else:
            raise ValueError(f"Unknown joint type: {type}")
        self._joints.append(joint)
        if not hasattr(self, '_joint_metadata'):
            self._joint_metadata = {}
        self._joint_metadata[id(joint)] = {
            'type': type,
            'max_motor_torque': float(max_motor_torque) if type == 'pivot' else None,
            'max_motor_force': float(max_motor_force) if max_motor_force else (5000.0 if type == 'slider' else None),
        }
        return joint
    def set_motor(self, joint, motor_speed, max_torque=100.0):
        if isinstance(joint, Box2D.b2RevoluteJoint):
            joint.enableMotor = True
            joint.motorSpeed = float(motor_speed)
            joint.maxMotorTorque = float(max_torque)
            return
        raise ValueError("set_motor: joint must be a pivot/revolute joint")
    def set_slider_motor(self, joint, speed, max_force=5000.0):
        if type(joint).__name__ != 'b2PrismaticJoint':
            raise ValueError("set_slider_motor: joint must be a prismatic/slider joint")
        joint.enableMotor = True
        joint.motorSpeed = float(speed)
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
    def step(self, time_step):
        self._last_time_step = float(time_step)
        self._world.Step(time_step, 10, 10)
    def _record_observation_error(self, source, exc):
        self._observation_error_count += 1
        self._last_observation_error = f"{source}: {type(exc).__name__}: {exc}"
    def get_terrain_bounds(self):
        return {
            "ground": {"y": self._ground_y},
            "build_zone": {
                "x": [self.BUILD_ZONE_X_MIN, self.BUILD_ZONE_X_MAX],
                "y": [self.BUILD_ZONE_Y_MIN, self.BUILD_ZONE_Y_MAX]
            }
        }
    def get_gripper_position(self):
        if not self._bodies:
            return None
        if self._bodies:
            body = self._bodies[0]
            return (body.position.x, body.position.y)
        return None
    def get_object_position(self):
        if "object" in self._terrain_bodies:
            obj = self._terrain_bodies["object"]
            return (obj.position.x, obj.position.y)
        return None
    def get_slider_position(self):
        if self._slider_joint is None:
            return None
        joint = self._slider_joint
        try:
            joint_trans = joint.translation
        except Exception as exc:
            self._record_observation_error("get_slider_position.translation", exc)
            joint_trans = None
        max_force = None
        if hasattr(self, '_joint_metadata') and id(joint) in self._joint_metadata:
            max_force = self._joint_metadata[id(joint)].get('max_motor_force')
        return {
            'slider_body_y': joint.bodyB.position.y,
            'slider_translation': joint_trans,
            'slider_anchor_y': self._slider_joint_anchor_y,
            'slider_lower_limit': joint.lowerLimit if hasattr(joint, 'lowerLimit') else None,
            'slider_upper_limit': joint.upperLimit if hasattr(joint, 'upperLimit') else None,
            'slider_motor_speed': joint.motorSpeed if hasattr(joint, 'motorSpeed') else 0.0,
            'slider_max_motor_force': max_force,
        }
    def get_anchor_for_gripper(self):
        return self._terrain_bodies.get("gantry")
    def get_object_contact_count(self):
        obj = self._terrain_bodies.get("object")
        if not obj or not self._bodies:
            return 0, 0
        gripper_set = set(self._bodies)
        num_points = 0
        bodies_touching = set()
        try:
            edges = []
            contact_list = getattr(obj, 'contacts', None) or getattr(obj, 'contactList', None)
            if contact_list is not None:
                if hasattr(contact_list, '__iter__') and not hasattr(contact_list, 'next'):
                    edges = list(contact_list)
                else:
                    ce = contact_list
                    while ce:
                        edges.append(ce)
                        ce = getattr(ce, 'next', None)
            for contact_edge in edges:
                contact = getattr(contact_edge, 'contact', contact_edge)
                if getattr(contact, 'touching', False):
                    other = getattr(contact_edge, 'other', None)
                    if other is None and hasattr(contact, 'bodyA'):
                        other = contact.bodyB if contact.bodyA == obj else contact.bodyA
                    if other is not None and other in gripper_set:
                        bodies_touching.add(other)
                        num_points += getattr(getattr(contact, 'manifold', None), 'pointCount', 1)
            if not edges and hasattr(self._world, 'contactList'):
                for contact in self._world.contactList:
                    if not getattr(contact, 'touching', contact.IsTouching() if hasattr(contact, 'IsTouching') else False):
                        continue
                    a, b = contact.bodyA, contact.bodyB
                    if a == obj and b in gripper_set:
                        bodies_touching.add(b)
                        num_points += 1
                    elif b == obj and a in gripper_set:
                        bodies_touching.add(a)
                        num_points += 1
        except Exception as exc:
            self._record_observation_error("get_object_contact_count", exc)
        return num_points, len(bodies_touching)
    def get_object_velocity(self):
        obj = self._terrain_bodies.get("object")
        if not obj:
            return None
        vx = float(obj.linearVelocity.x)
        vy = float(obj.linearVelocity.y)
        return {
            'vx': vx,
            'vy': vy,
            'speed': float(math.hypot(vx, vy)),
            'angular_vel': float(obj.angularVelocity),
        }
    def get_object_info(self):
        obj = self._terrain_bodies.get("object")
        if not obj or not obj.fixtures:
            return None
        f = obj.fixtures[0]
        info = {
            'friction': float(f.friction),
            'mass': float(obj.mass),
        }
        shape = f.shape
        if isinstance(shape, Box2D.b2CircleShape):
            info['shape'] = 'circle'
            info['radius'] = float(shape.radius)
        elif hasattr(shape, 'vertices'):
            verts = shape.vertices
            xs = [v[0] for v in verts]
            ys = [v[1] for v in verts]
            info['shape'] = 'box'
            info['width'] = float(max(xs) - min(xs))
            info['height'] = float(max(ys) - min(ys))
        else:
            info['shape'] = 'unknown'
        return info
    def get_platform_top_y(self):
        platform = self._terrain_bodies.get("platform")
        if platform and platform.fixtures:
            shape = platform.fixtures[0].shape
            if hasattr(shape, 'vertices'):
                max_wy = max(platform.GetWorldPoint((v[0], v[1])).y for v in shape.vertices)
                return float(max_wy)
        return None
    def get_finger_joint_states(self):
        states = []
        inv_dt = 1.0 / self._last_time_step if self._last_time_step else 60.0
        for joint in self._joints:
            if type(joint).__name__ == 'b2RevoluteJoint':
                try:
                    angle_rad = float(joint.angle)
                    motor_torque = None
                    try:
                        motor_torque = float(joint.GetMotorTorque(inv_dt))
                    except Exception as exc:
                        self._record_observation_error("get_finger_joint_states.motor_torque", exc)
                    max_torque = None
                    if hasattr(self, '_joint_metadata') and id(joint) in self._joint_metadata:
                        max_torque = self._joint_metadata[id(joint)].get('max_motor_torque')
                    st = {
                        'angle_rad': angle_rad,
                        'angle_deg': float(math.degrees(angle_rad)),
                        'motor_speed': float(joint.motorSpeed),
                        'joint_speed_actual': float(joint.speed),
                        'motor_torque': motor_torque,
                        'max_motor_torque': max_torque,
                        'motor_enabled': bool(joint.motorEnabled),
                    }
                    try:
                        if joint.limitEnabled:
                            st['lower_limit'] = float(joint.lowerLimit)
                            st['upper_limit'] = float(joint.upperLimit)
                            st['has_limits'] = True
                        else:
                            st['has_limits'] = False
                    except Exception as exc:
                        self._record_observation_error("get_finger_joint_states.limits", exc)
                        st['has_limits'] = False
                    states.append(st)
                except Exception as exc:
                    self._record_observation_error("get_finger_joint_states", exc)
        return states
    def get_slider_motor_force(self):
        if self._slider_joint:
            try:
                inv_dt = 1.0 / self._last_time_step if self._last_time_step else 60.0
                return float(self._slider_joint.GetMotorForce(inv_dt))
            except Exception as exc:
                self._record_observation_error("get_slider_motor_force", exc)
        return None
    def get_slider_computed_translation(self):
        if self._slider_joint is None:
            return None
        joint = self._slider_joint
        try:
            if self._slider_body_initial_y is not None:
                return float(self._slider_body_initial_y - joint.bodyB.position.y)
        except Exception as exc:
            self._record_observation_error("get_slider_computed_translation", exc)
        return None
    def get_object_contact_details(self):
        obj = self._terrain_bodies.get("object")
        if not obj:
            return {"contact_points": 0, "bodies_touching": 0,
                    "max_normal_impulse": 0.0, "total_normal_impulse": 0.0}
        gripper_set = set(self._bodies)
        num_points = 0
        bodies_touching = set()
        max_normal = 0.0
        total_normal = 0.0
        try:
            edges = []
            contact_list = getattr(obj, "contacts", None) or getattr(obj, "contactList", None)
            if contact_list is not None:
                if hasattr(contact_list, "__iter__") and not hasattr(contact_list, "next"):
                    edges = list(contact_list)
                else:
                    ce = contact_list
                    while ce:
                        edges.append(ce)
                        ce = getattr(ce, "next", None)
            for contact_edge in edges:
                contact = getattr(contact_edge, "contact", contact_edge)
                if not getattr(contact, "touching", False):
                    continue
                other = getattr(contact_edge, "other", None)
                if other is None and hasattr(contact, "bodyA"):
                    other = contact.bodyB if contact.bodyA == obj else contact.bodyA
                if other is not None and other in gripper_set:
                    bodies_touching.add(other)
                    manifold = getattr(contact, "manifold", None)
                    if manifold:
                        pts = getattr(manifold, "points", [])
                        for pt in pts:
                            ni = float(getattr(pt, "normalImpulse", 0.0))
                            total_normal += ni
                            if ni > max_normal:
                                max_normal = ni
                            num_points += 1
                    else:
                        num_points += 1
            if not edges and hasattr(self._world, "contactList"):
                for contact in self._world.contactList:
                    touching = getattr(contact, "touching", None)
                    if touching is None:
                        touching = contact.IsTouching() if hasattr(contact, "IsTouching") else False
                    if not touching:
                        continue
                    a, b = contact.bodyA, contact.bodyB
                    target_other = None
                    if a == obj and b in gripper_set:
                        target_other = b
                    elif b == obj and a in gripper_set:
                        target_other = a
                    if target_other:
                        bodies_touching.add(target_other)
                        manifold = getattr(contact, "manifold", None)
                        if manifold:
                            pts = getattr(manifold, "points", [])
                            for pt in pts:
                                ni = float(getattr(pt, "normalImpulse", 0.0))
                                total_normal += ni
                                if ni > max_normal:
                                    max_normal = ni
                                num_points += 1
                        else:
                            num_points += 1
        except Exception as exc:
            self._record_observation_error("get_object_contact_details", exc)
        return {
            "contact_points": num_points,
            "bodies_touching": len(bodies_touching),
            "max_normal_impulse": max_normal,
            "total_normal_impulse": total_normal,
        }
    def get_gravity(self):
        return (float(self._world.gravity.x), float(self._world.gravity.y))
