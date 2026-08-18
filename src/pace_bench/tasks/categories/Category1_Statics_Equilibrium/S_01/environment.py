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
        self._anchor_max_force = float(physics_config.get("anchor_max_force", 100.0))
        self._anchor_max_torque = float(physics_config.get("anchor_max_torque", 500.0))
        self._joint_max_force = float(physics_config.get("joint_max_force", 80.0))
        self._joint_max_torque = float(physics_config.get("joint_max_torque", 300.0))
        self._wind_force = tuple(physics_config.get("wind_force", (0.0, 0.0)))
        self._world = world(gravity=gravity, doSleep=True)
        self._bodies = []
        self._body_creation_positions = []
        self._joints = []
        self._terrain_bodies = {}
        self._tracked_body = None
        self._last_tracked_angle = None
        self._airborne_rotation_clockwise = 0.0
        self._airborne_rotation_counterclockwise = 0.0
        self._airborne_rotation_exceeded = False
        self._AIRBORNE_THRESHOLD = 0.5
        self._MAX_AIRBORNE_ROTATION = math.pi
        self._joint_peak_forces = {}
        self._joint_peak_torques = {}
        self._joint_peak_force_details = {}
        self._joint_peak_torque_details = {}
        self._joint_ids = {}
        self._next_joint_id = 0
        self._joint_failure_events = []
        self._joint_stress_records = []
        self._sim_step_count = 0
        self.world = self._world
        self.bodies = self._bodies
        self.joints = self._joints
        self._create_terrain(terrain_config)
        gap_width = float(terrain_config.get("gap_width", 15.0))
        right_cliff_start = 10.0 + gap_width
        self.BUILD_ZONE_X_MIN = 10.0
        self.BUILD_ZONE_X_MAX = right_cliff_start + 5.0
        self.BUILD_ZONE_Y_MIN = 5.0
        self.BUILD_ZONE_Y_MAX = 15.0
        self.MAX_STRUCTURE_MASS = float(terrain_config.get("max_structure_mass", 2000.0))
        self._create_vehicle(terrain_config)
    def _create_terrain(self, terrain_config: dict):
        left_cliff_x = 10.0
        left_cliff_y = 10.0
        left_cliff = self._world.CreateStaticBody(
            position=(left_cliff_x / 2, left_cliff_y / 2),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(left_cliff_x / 2, left_cliff_y / 2)),
                friction=0.8,
            ),
        )
        self._terrain_bodies["left_cliff"] = left_cliff
        gap_width = float(terrain_config.get("gap_width", 15.0))
        right_cliff_start = left_cliff_x + gap_width
        right_cliff_end = 100.0
        right_cliff_width = right_cliff_end - right_cliff_start
        right_cliff_y = 10.0
        right_cliff = self._world.CreateStaticBody(
            position=(right_cliff_start + right_cliff_width / 2, right_cliff_y / 2),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(right_cliff_width / 2, right_cliff_y / 2)),
                friction=0.8,
            ),
        )
        self._terrain_bodies["right_cliff"] = right_cliff
        self._gap_width = gap_width
        self._right_cliff_start = right_cliff_start
        water_half_width = right_cliff_end + 50.0
        water = self._world.CreateStaticBody(
            position=(water_half_width, 0),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(water_half_width, 0.1)),
                friction=0.0,
                isSensor=True,
            ),
        )
        self._terrain_bodies["water"] = water
    def _create_vehicle(self, terrain_config: dict):
        vehicle_mass = 2000.0
        wheelbase = 3.0
        spawn_x = 5.0
        spawn_y = 10.0 + 0.5
        speed = 5.0
        chassis_width = 2.0
        chassis_height = 0.5
        chassis_density = vehicle_mass / (chassis_width * chassis_height)
        chassis = self._world.CreateDynamicBody(
            position=(spawn_x, spawn_y),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(chassis_width/2, chassis_height/2)),
                density=chassis_density,
                friction=0.8,
            )
        )
        chassis.linearVelocity = (speed, 0)
        chassis.linearDamping = self._default_linear_damping
        chassis.angularDamping = self._default_angular_damping
        wheel_radius = 0.4
        wheel_density = 100.0
        wheel1 = self._world.CreateDynamicBody(
            position=(spawn_x - wheelbase/2, spawn_y - chassis_height/2 - wheel_radius),
            fixtures=Box2D.b2FixtureDef(
                shape=circleShape(radius=wheel_radius),
                density=wheel_density,
                friction=0.8,
            )
        )
        wheel1.linearVelocity = (speed, 0)
        wheel1.linearDamping = self._default_linear_damping
        wheel1.angularDamping = self._default_angular_damping
        wheel2 = self._world.CreateDynamicBody(
            position=(spawn_x + wheelbase/2, spawn_y - chassis_height/2 - wheel_radius),
            fixtures=Box2D.b2FixtureDef(
                shape=circleShape(radius=wheel_radius),
                density=wheel_density,
                friction=0.8,
            )
        )
        wheel2.linearVelocity = (speed, 0)
        wheel2.linearDamping = self._default_linear_damping
        wheel2.angularDamping = self._default_angular_damping
        wheel1_center_x = spawn_x - wheelbase/2
        wheel1_center_y = spawn_y - chassis_height/2 - wheel_radius
        wheel2_center_x = spawn_x + wheelbase/2
        wheel2_center_y = spawn_y - chassis_height/2 - wheel_radius
        joint1 = self._world.CreateRevoluteJoint(
            bodyA=chassis,
            bodyB=wheel1,
            anchor=(wheel1_center_x, wheel1_center_y),
            collideConnected=False
        )
        joint2 = self._world.CreateRevoluteJoint(
            bodyA=chassis,
            bodyB=wheel2,
            anchor=(wheel2_center_x, wheel2_center_y),
            collideConnected=False
        )
        self._terrain_bodies["vehicle_chassis"] = chassis
        self._terrain_bodies["vehicle_wheel1"] = wheel1
        self._terrain_bodies["vehicle_wheel2"] = wheel2
        self._vehicle_joints = [joint1, joint2]
    MIN_BEAM_SIZE = 0.1
    MAX_BEAM_SIZE = 10.0
    MAX_VEHICLE_FORCE_X = 8000.0
    MAX_VEHICLE_FORCE_Y = 20000.0
    BUILD_ZONE_X_MIN = 10.0
    BUILD_ZONE_Y_MIN = 5.0
    BUILD_ZONE_Y_MAX = 15.0
    def add_beam(self, x, y, width, height, angle=0, density=1.0, friction=0.5):
        width = max(self.MIN_BEAM_SIZE, min(width, self.MAX_BEAM_SIZE))
        height = max(self.MIN_BEAM_SIZE, min(height, self.MAX_BEAM_SIZE))
        friction = max(0.0, float(friction))
        body = self._world.CreateDynamicBody(
            position=(x, y),
            angle=angle,
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(width/2, height/2)),
                density=density,
                friction=friction,
            )
        )
        body.linearDamping = self._default_linear_damping
        body.angularDamping = self._default_angular_damping
        self._bodies.append(body)
        self._body_creation_positions.append((float(x), float(y)))
        return body
    def add_joint(self, body_a, body_b, anchor_point, type='rigid'):
        if body_a is None:
            raise ValueError("add_joint: body_a cannot be None.")
        anchor_x, anchor_y = anchor_point[0], anchor_point[1]
        if body_b is None:
            left_cliff = self._terrain_bodies.get("left_cliff")
            right_cliff = self._terrain_bodies.get("right_cliff")
            mid_x = 10.0 + self._gap_width / 2
            if anchor_x <= mid_x:
                body_b = left_cliff
            else:
                body_b = right_cliff
        if type == 'rigid':
            joint = self._world.CreateWeldJoint(bodyA=body_a, bodyB=body_b, anchor=(anchor_x, anchor_y), collideConnected=False)
        elif type == 'pivot':
            joint = self._world.CreateRevoluteJoint(bodyA=body_a, bodyB=body_b, anchor=(anchor_x, anchor_y), collideConnected=False)
        else:
            raise ValueError(f"Unknown joint type: {type}")
        self._joints.append(joint)
        self._joint_ids[joint] = self._next_joint_id
        self._next_joint_id += 1
        return joint
    def get_structure_mass(self):
        return sum(body.mass for body in self._bodies)
    def apply_vehicle_force(self, fx, fy):
        fx = float(fx)
        fy = float(fy)
        if not math.isfinite(fx) or not math.isfinite(fy):
            raise ValueError("Vehicle force components must be finite.")
        fx = max(-self.MAX_VEHICLE_FORCE_X, min(fx, self.MAX_VEHICLE_FORCE_X))
        fy = max(-self.MAX_VEHICLE_FORCE_Y, min(fy, self.MAX_VEHICLE_FORCE_Y))
        chassis = self._terrain_bodies.get("vehicle_chassis")
        if chassis is None:
            raise RuntimeError("Vehicle chassis is unavailable.")
        chassis.ApplyForceToCenter((fx, fy), True)
    def set_material_properties(self, body, restitution=0.2):
        for fixture in body.fixtures:
            fixture.restitution = float(restitution)
    def step(self, time_step):
        if any(self._wind_force):
            for body in self._bodies:
                body.ApplyForceToCenter(self._wind_force, True)
            for body_name in ["vehicle_chassis", "vehicle_wheel1", "vehicle_wheel2"]:
                if body_name in self._terrain_bodies:
                    self._terrain_bodies[body_name].ApplyForceToCenter(self._wind_force, True)
        if self._tracked_body is not None and not self._airborne_rotation_exceeded:
            current_angle = self._tracked_body.angle
            current_y = self._tracked_body.position.y
            cliff_top = self.get_terrain_bounds()["left_cliff"]["y"]
            is_airborne = current_y > (cliff_top + self._AIRBORNE_THRESHOLD)
            if is_airborne:
                if self._last_tracked_angle is not None:
                    angle_diff = current_angle - self._last_tracked_angle
                    angle_diff_normalized = ((angle_diff + math.pi) % (2 * math.pi)) - math.pi
                    angle_diff_unwrapped = angle_diff if abs(angle_diff) < math.pi else angle_diff_normalized
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
        self._world.Step(time_step, 10, 10)
        self._sim_step_count += 1
        joints_to_remove = []
        new_failure_events = []
        self._joint_stress_records = []
        for joint_idx, joint in enumerate(self._joints):
            force = joint.GetReactionForce(time_step)
            force_magnitude = math.sqrt(force.x**2 + force.y**2)
            torque_magnitude = abs(joint.GetReactionTorque(time_step))
            body_a, body_b = joint.bodyA, joint.bodyB
            body_a_pos = (float(body_a.position.x), float(body_a.position.y))
            body_b_pos = (float(body_b.position.x), float(body_b.position.y))
            anchor_a_pos = (float(joint.anchorA.x), float(joint.anchorA.y))
            anchor_b_pos = (float(joint.anchorB.x), float(joint.anchorB.y))
            peak_force = self._joint_peak_forces.get(joint, -math.inf)
            if force_magnitude > peak_force:
                self._joint_peak_forces[joint] = force_magnitude
                self._joint_peak_force_details[joint] = {
                    "step": self._sim_step_count,
                    "body_a_pos": body_a_pos,
                    "body_b_pos": body_b_pos,
                    "anchor_a_pos": anchor_a_pos,
                    "anchor_b_pos": anchor_b_pos,
                }
            peak_torque = self._joint_peak_torques.get(joint, -math.inf)
            if torque_magnitude > peak_torque:
                self._joint_peak_torques[joint] = torque_magnitude
                self._joint_peak_torque_details[joint] = {
                    "step": self._sim_step_count,
                    "body_a_pos": body_a_pos,
                    "body_b_pos": body_b_pos,
                    "anchor_a_pos": anchor_a_pos,
                    "anchor_b_pos": anchor_b_pos,
                }
            is_anchor = (
                body_a.type == staticBody
                or body_b.type == staticBody
                or body_a in self._terrain_bodies.values()
                or body_b in self._terrain_bodies.values()
            )
            if is_anchor:
                max_force, max_torque = self._anchor_max_force, self._anchor_max_torque
            else:
                max_force, max_torque = self._joint_max_force, self._joint_max_torque
            force_details = self._joint_peak_force_details[joint]
            torque_details = self._joint_peak_torque_details[joint]
            self._joint_stress_records.append({
                "joint_idx": joint_idx,
                "joint_id": self._joint_ids[joint],
                "body_a_pos": body_a_pos,
                "body_b_pos": body_b_pos,
                "anchor_a_pos": anchor_a_pos,
                "anchor_b_pos": anchor_b_pos,
                "peak_force": self._joint_peak_forces[joint],
                "peak_torque": self._joint_peak_torques[joint],
                "peak_force_step": force_details["step"],
                "peak_force_anchor_a_pos": force_details["anchor_a_pos"],
                "peak_force_anchor_b_pos": force_details["anchor_b_pos"],
                "peak_torque_step": torque_details["step"],
                "peak_torque_anchor_a_pos": torque_details["anchor_a_pos"],
                "peak_torque_anchor_b_pos": torque_details["anchor_b_pos"],
                "force_limit": max_force,
                "torque_limit": max_torque,
                "is_anchor": is_anchor,
            })
            if (
                self._joint_peak_forces[joint] > max_force
                or self._joint_peak_torques[joint] > max_torque
            ):
                joints_to_remove.append(joint)
                new_failure_events.append({
                    "step": self._sim_step_count,
                    "joint_id": self._joint_ids[joint],
                    "body_a_pos": body_a_pos,
                    "body_b_pos": body_b_pos,
                    "anchor_a_pos": anchor_a_pos,
                    "anchor_b_pos": anchor_b_pos,
                    "peak_force": self._joint_peak_forces[joint],
                    "peak_torque": self._joint_peak_torques[joint],
                    "peak_force_step": force_details["step"],
                    "peak_force_anchor_a_pos": force_details["anchor_a_pos"],
                    "peak_force_anchor_b_pos": force_details["anchor_b_pos"],
                    "peak_torque_step": torque_details["step"],
                    "peak_torque_anchor_a_pos": torque_details["anchor_a_pos"],
                    "peak_torque_anchor_b_pos": torque_details["anchor_b_pos"],
                    "force_limit": max_force,
                    "torque_limit": max_torque,
                    "is_anchor": is_anchor,
                })
        self._joint_failure_events.extend(new_failure_events)
        for joint in joints_to_remove:
            self._world.DestroyJoint(joint)
            self._joints.remove(joint)
            self._joint_peak_forces.pop(joint, None)
            self._joint_peak_torques.pop(joint, None)
            self._joint_peak_force_details.pop(joint, None)
            self._joint_peak_torque_details.pop(joint, None)
            self._joint_ids.pop(joint, None)
    def set_tracked_body(self, body):
        self._tracked_body = body
        self._last_tracked_angle = body.angle if body else None
        self._airborne_rotation_clockwise = 0.0
        self._airborne_rotation_counterclockwise = 0.0
        self._airborne_rotation_exceeded = False
    def get_airborne_rotation_status(self):
        net_rotation = abs(self._airborne_rotation_counterclockwise - self._airborne_rotation_clockwise)
        return {'accumulated': net_rotation, 'exceeded': self._airborne_rotation_exceeded}
    def get_terrain_bounds(self):
        gap_width = getattr(self, '_gap_width', 15.0)
        right_cliff_start = getattr(self, '_right_cliff_start', 25.0)
        return {
            "left_cliff": {"x_end": 10.0, "y": 10.0},
            "right_cliff": {"x_start": right_cliff_start, "y": 10.0},
            "gap": {"x_start": 10.0, "x_end": right_cliff_start, "width": gap_width},
            "build_zone": {
                "x": [self.BUILD_ZONE_X_MIN, self.BUILD_ZONE_X_MAX],
                "y": [self.BUILD_ZONE_Y_MIN, self.BUILD_ZONE_Y_MAX]
            },
            "water_level": 0.0,
            "fail_zone_y": 0.5
        }
    def get_vehicle_position(self):
        if "vehicle_chassis" in self._terrain_bodies:
            chassis = self._terrain_bodies["vehicle_chassis"]
            return (chassis.position.x, chassis.position.y)
        return None
    def get_vehicle_velocity(self):
        if "vehicle_chassis" in self._terrain_bodies:
            chassis = self._terrain_bodies["vehicle_chassis"]
            return (chassis.linearVelocity.x, chassis.linearVelocity.y)
        return None
    def get_joint_max_force(self):
        return self._joint_max_force
    def get_joint_max_torque(self):
        return self._joint_max_torque
    def get_anchor_max_force(self):
        return self._anchor_max_force
    def get_anchor_max_torque(self):
        return self._anchor_max_torque
    def get_joint_failure_events(self):
        return list(self._joint_failure_events)
    def get_joint_stress_summary(self):
        return list(self._joint_stress_records)
    def get_current_step(self):
        return self._sim_step_count
    def get_gravity(self):
        g = self._world.gravity
        return (float(g.x), float(g.y))
    def get_wind_force(self):
        return (float(self._wind_force[0]), float(self._wind_force[1]))
    def get_body_positions_and_angles(self):
        return [
            (float(body.position.x), float(body.position.y), float(body.angle))
            for body in self._bodies
        ]
    def get_body_creation_positions(self):
        return list(self._body_creation_positions)
    def get_anchor_positions(self):
        anchors = []
        mid_x = 10.0 + getattr(self, '_gap_width', 15.0) / 2.0
        for joint in self._joints:
            a_static = joint.bodyA.type == staticBody
            b_static = joint.bodyB.type == staticBody
            if not (a_static or b_static):
                continue
            if a_static:
                awx = float(joint.anchorA.x)
                awy = float(joint.anchorA.y)
                dbx = float(joint.bodyB.position.x)
                dby = float(joint.bodyB.position.y)
                is_left = awx <= mid_x
            else:
                awx = float(joint.anchorB.x)
                awy = float(joint.anchorB.y)
                dbx = float(joint.bodyA.position.x)
                dby = float(joint.bodyA.position.y)
                is_left = awx <= mid_x
            anchors.append((awx, awy, dbx, dby, is_left))
        return anchors
    def get_terrain_extents(self):
        bounds = self.get_terrain_bounds()
        right_cliff = bounds.get("right_cliff", {})
        right_start = float(right_cliff.get("x_start", 25.0))
        right_end = 100.0
        return {
            "left_cliff_x_end": 10.0,
            "right_cliff_x_start": right_start,
            "right_cliff_x_end": right_end,
            "gap_width": float(bounds.get("gap", {}).get("width", 15.0)),
            "cliff_top_y": float(bounds.get("left_cliff", {}).get("y", 10.0)),
        }
    def get_vehicle_chassis(self):
        return self._terrain_bodies.get("vehicle_chassis")
