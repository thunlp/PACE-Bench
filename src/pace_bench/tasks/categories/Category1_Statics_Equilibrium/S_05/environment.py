import Box2D

from Box2D.b2 import (world, polygonShape, circleShape, staticBody, dynamicBody)

import math

import random

def _is_finite(x):
    try:
        return math.isfinite(float(x))
    except (TypeError, ValueError):
        return False

class DaVinciSandbox:
    def __init__(self, terrain_config=None, physics_config=None):
        terrain_config = terrain_config or {}
        physics_config = physics_config or {}
        self._terrain_config = dict(terrain_config)
        self._physics_config = dict(physics_config)
        gravity = tuple(physics_config.get("gravity", (0, -10)))
        do_sleep = True if abs(gravity[1]) < 20 else False
        self._world = world(gravity=gravity, doSleep=do_sleep)
        self._bodies = []
        self._joints = []
        self._terrain_bodies = {}
        self._meteors = []
        self._max_force_on_core = 0.0
        self._joint_anchor_positions = []
        self._joint_peak_forces = []
        self._joint_peak_torques = []
        self._first_joint_breach_step = None
        self._first_joint_breach_anchor = None
        self._joints_broken_count = 0
        self._numerical_instability_count = 0
        self._joint_breach_events = []
        self._total_boulder_ke = 0.0
        self._max_body_velocity_seen = 0.0
        self._core_force_step = None
        self._seed = int(terrain_config.get("seed", 42))
        self._rng = random.Random(self._seed)
        self.world = self._world
        self.bodies = self._bodies
        self.joints = self._joints
        self.CORE_X = float(terrain_config.get("core_x", 10.0))
        self.CORE_Y = float(terrain_config.get("core_y", 1.0))
        self.CORE_RADIUS = 0.5
        self.CORE_MAX_FORCE = float(terrain_config.get("max_core_force", 150.0))
        self.BUILD_ZONE_X_MIN = 5.0
        self.BUILD_ZONE_X_MAX = 15.0
        self.BUILD_ZONE_Y_MIN = 0.0
        self.BUILD_ZONE_Y_MAX = 8.0
        self.MAX_STRUCTURE_HEIGHT = 7.5
        self.MAX_STRUCTURE_MASS = float(terrain_config.get("max_structure_mass", 300.0))
        self._meteor_count = int(terrain_config.get("meteor_count", 12))
        self._meteor_spawn_interval = int(terrain_config.get("meteor_spawn_interval", 30))
        self._wind_force = float(terrain_config.get("wind_force", 0.0))
        self._meteor_restitution = float(terrain_config.get("meteor_restitution", 0.2))
        self._meteor_density = float(terrain_config.get("meteor_density", 5.0))
        self._floor_friction = float(terrain_config.get("floor_friction", 0.5))
        self._floor_restitution = float(terrain_config.get("floor_restitution", 0.0))
        self._structure_restitution = float(terrain_config.get("structure_restitution", 0.0))
        self._structure_friction = float(terrain_config.get("structure_friction", 0.5))
        self._meteor_vx_range = terrain_config.get("meteor_vx_range", [-2.0, 2.0])
        self._max_joint_force = float(terrain_config.get("max_joint_force", 1e12))
        self._max_joint_torque = float(terrain_config.get("max_joint_torque", 1e12))
        self._has_walls = terrain_config.get("has_walls", False)
        self._step_count = 0
        self._max_reaction_force_seen = 0.0
        self._max_reaction_torque_seen = 0.0
        self._joint_telemetry_v2 = {}
        self._joint_observation_error_count = 0
        self._last_joint_observation_error = None
        self._create_terrain(terrain_config)
        self._create_core(terrain_config)
    def _create_terrain(self, terrain_config: dict):
        floor_length = 40.0
        floor_height = 0.5
        floor = self._world.CreateStaticBody(
            position=(floor_length / 2, -floor_height / 2),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(floor_length / 2, floor_height / 2)),
                friction=self._floor_friction,
                restitution=self._floor_restitution,
                userData="floor"
            ),
        )
        self._terrain_bodies["floor"] = floor
        if self._has_walls:
            wall_width = 0.5
            wall_height = 20.0
            left_wall = self._world.CreateStaticBody(
                position=(-wall_width / 2, wall_height / 2),
                fixtures=Box2D.b2FixtureDef(
                    shape=polygonShape(box=(wall_width / 2, wall_height / 2)),
                    friction=0.0,
                    restitution=1.0,
                    userData="wall"
                ),
            )
            right_wall = self._world.CreateStaticBody(
                position=(floor_length + wall_width / 2, wall_height / 2),
                fixtures=Box2D.b2FixtureDef(
                    shape=polygonShape(box=(wall_width / 2, wall_height / 2)),
                    friction=0.0,
                    restitution=1.0,
                    userData="wall"
                ),
            )
            self._terrain_bodies["left_wall"] = left_wall
            self._terrain_bodies["right_wall"] = right_wall
    def _create_core(self, terrain_config: dict):
        core = self._world.CreateDynamicBody(
            position=(self.CORE_X, self.CORE_Y),
            fixtures=Box2D.b2FixtureDef(
                shape=circleShape(radius=self.CORE_RADIUS),
                density=1.0,
                friction=0.5,
                userData="core"
            ),
            allowSleep=False,
        )
        self._terrain_bodies["core"] = core
    def _spawn_side_meteor(self):
        side = self._rng.choice([-1, 1])
        x = self.CORE_X + side * 15.0
        y = self._rng.uniform(1.0, 5.0)
        radius = self._rng.uniform(0.2, 0.4)
        vx = -side * self._rng.uniform(10.0, 20.0)
        vy = self._rng.uniform(0, 5.0)
        meteor = self._world.CreateDynamicBody(
            position=(x, y),
            fixtures=Box2D.b2FixtureDef(
                shape=circleShape(radius=radius),
                density=self._meteor_density,
                friction=0.0,
                restitution=self._meteor_restitution,
                userData="meteor"
            ),
            bullet=True
        )
        meteor.linearVelocity = (vx, vy)
        self._meteors.append(meteor)
        ke = 0.5 * meteor.mass * (vx * vx + vy * vy)
        self._total_boulder_ke += ke
    def _spawn_meteor(self):
        x = self._rng.uniform(self.BUILD_ZONE_X_MIN - 4, self.BUILD_ZONE_X_MAX + 4)
        y = 15.0
        radius = self._rng.uniform(0.2, 0.5)
        vx = self._rng.uniform(self._meteor_vx_range[0], self._meteor_vx_range[1])
        vy = self._rng.uniform(-15.0, -10.0)
        meteor = self._world.CreateDynamicBody(
            position=(x, y),
            fixtures=Box2D.b2FixtureDef(
                shape=circleShape(radius=radius),
                density=self._meteor_density,
                friction=0.5,
                restitution=self._meteor_restitution,
                userData="meteor"
            ),
            bullet=True
        )
        meteor.linearVelocity = (vx, vy)
        self._meteors.append(meteor)
        ke = 0.5 * meteor.mass * (vx * vx + vy * vy)
        self._total_boulder_ke += ke
    def add_beam(self, x, y, width, height, angle=0, density=1.0):
        if x < self.BUILD_ZONE_X_MIN or x > self.BUILD_ZONE_X_MAX:
            raise ValueError(f"Beam center x={x} is outside build zone x=[{self.BUILD_ZONE_X_MIN}, {self.BUILD_ZONE_X_MAX}]")
        if y < self.BUILD_ZONE_Y_MIN or y > self.BUILD_ZONE_Y_MAX:
            raise ValueError(f"Beam center y={y} is outside build zone y=[{self.BUILD_ZONE_Y_MIN}, {self.BUILD_ZONE_Y_MAX}]")
        if y > self.MAX_STRUCTURE_HEIGHT:
            raise ValueError(f"Beam center y={y} exceeds height limit {self.MAX_STRUCTURE_HEIGHT}m")
        dist_to_core = math.sqrt((x - self.CORE_X)**2 + (y - self.CORE_Y)**2)
        if dist_to_core < 1.3 - 1e-6:
            raise ValueError(f"Beam center distance to core {dist_to_core:.2f}m is within 1.3m keep-out zone")
        if width < 0.1 or width > 10.0 or height < 0.1 or height > 10.0:
            raise ValueError(f"Beam dimensions must be in [0.1, 10.0] m; got width={width}, height={height}")
        body = self._world.CreateDynamicBody(position=(x, y), angle=angle)
        fixture = body.CreatePolygonFixture(box=(width/2, height/2), density=density, friction=self._structure_friction)
        fixture.restitution = self._structure_restitution
        fixture.userData = "beam"
        self._bodies.append(body)
        return body
    def add_joint(self, body_a, body_b, anchor, type='rigid'):
        if anchor[0] < self.BUILD_ZONE_X_MIN or anchor[0] > self.BUILD_ZONE_X_MAX:
            raise ValueError(f"Joint anchor x={anchor[0]} is outside build zone x=[{self.BUILD_ZONE_X_MIN}, {self.BUILD_ZONE_X_MAX}]")
        if anchor[1] < self.BUILD_ZONE_Y_MIN or anchor[1] > self.BUILD_ZONE_Y_MAX:
            raise ValueError(f"Joint anchor y={anchor[1]} is outside build zone y=[{self.BUILD_ZONE_Y_MIN}, {self.BUILD_ZONE_Y_MAX}]")
        if anchor[1] > self.MAX_STRUCTURE_HEIGHT:
            raise ValueError(f"Joint anchor y={anchor[1]} exceeds height limit {self.MAX_STRUCTURE_HEIGHT}m")
        if body_b is None: body_b = self._terrain_bodies["floor"]
        joint_def = Box2D.b2WeldJointDef()
        joint_def.Initialize(body_a, body_b, anchor)
        j = self._world.CreateJoint(joint_def)
        self._joints.append(j)
        self._joint_anchor_positions.append(tuple(anchor))
        self._joint_peak_forces.append(0.0)
        self._joint_peak_torques.append(0.0)
        self._joint_telemetry_v2[id(j)] = {
            'anchor': tuple(anchor),
            'peak_force': 0.0,
            'peak_torque': 0.0,
            'broken': False,
            'breach_step': None,
        }
        return j
    def get_structure_mass(self):
        return sum(b.mass for b in self._bodies)
    def set_material_properties(self, body, restitution=0.2):
        for fixture in body.fixtures:
            fixture.restitution = float(restitution)
    def step(self, time_step):
        if self._step_count < self._meteor_count * self._meteor_spawn_interval:
            if self._step_count % self._meteor_spawn_interval == 0:
                self._spawn_meteor()
                if self._step_count % (self._meteor_spawn_interval * 3) == 0:
                    self._spawn_side_meteor()
        self._step_count += 1
        if self._wind_force != 0:
            core = self._terrain_bodies.get("core")
            for body in self._world.bodies:
                if body.type == Box2D.b2_dynamicBody and body != core:
                    body.ApplyForceToCenter((self._wind_force * body.mass, 0), True)
        sub_steps = 6
        sub_dt = time_step / sub_steps
        broken_joints = []
        for _ in range(sub_steps):
            self._world.Step(sub_dt, 30, 30)
            for body in self._world.bodies:
                if body.type == Box2D.b2_dynamicBody:
                    if not (_is_finite(body.position.x) and _is_finite(body.position.y) and
                            _is_finite(body.linearVelocity.x) and _is_finite(body.linearVelocity.y)):
                        self._numerical_instability_count += 1
                    else:
                        v = body.linearVelocity.length
                        if v > self._max_body_velocity_seen:
                            self._max_body_velocity_seen = v
            inv_sub_dt = 1.0 / sub_dt if sub_dt > 0 else 0.0
            for idx, j in enumerate(list(self._joints)):
                if j in broken_joints:
                    continue
                try:
                    force = j.GetReactionForce(inv_sub_dt).length
                    torque = abs(j.GetReactionTorque(inv_sub_dt))
                    self._max_reaction_force_seen = max(self._max_reaction_force_seen, force)
                    self._max_reaction_torque_seen = max(self._max_reaction_torque_seen, torque)
                    telemetry = self._joint_telemetry_v2.get(id(j))
                    if telemetry is not None:
                        telemetry['peak_force'] = max(telemetry['peak_force'], force)
                        telemetry['peak_torque'] = max(telemetry['peak_torque'], torque)
                    if idx < len(self._joint_peak_forces):
                        self._joint_peak_forces[idx] = max(self._joint_peak_forces[idx], force)
                    if idx < len(self._joint_peak_torques):
                        self._joint_peak_torques[idx] = max(self._joint_peak_torques[idx], torque)
                    if self._first_joint_breach_step is None:
                        if force > self._max_joint_force or torque > self._max_joint_torque:
                            self._first_joint_breach_step = self._step_count
                            if idx < len(self._joint_anchor_positions):
                                self._first_joint_breach_anchor = self._joint_anchor_positions[idx]
                    if force > self._max_joint_force or torque > self._max_joint_torque:
                        anchor = self._joint_anchor_positions[idx] if idx < len(self._joint_anchor_positions) else (0.0, 0.0)
                        self._joint_breach_events.append({
                            'step': self._step_count,
                            'anchor': tuple(anchor),
                            'force': float(force),
                            'torque': float(torque),
                        })
                        broken_joints.append(j)
                        if telemetry is not None:
                            telemetry['broken'] = True
                            telemetry['breach_step'] = self._step_count
                except Exception as exc:
                    self._joint_observation_error_count += 1
                    self._last_joint_observation_error = f"{type(exc).__name__}: {exc}"
                    continue
        for j in broken_joints:
            self._world.DestroyJoint(j)
            self._joints.remove(j)
            self._joints_broken_count += 1
        core = self._terrain_bodies["core"]
        for contact_edge in core.contacts:
            if contact_edge.contact.touching:
                manifold = contact_edge.contact.manifold
                if manifold.pointCount > 0:
                    f1, f2 = contact_edge.contact.fixtureA, contact_edge.contact.fixtureB
                    if f1.userData in ["floor", "wall"] or f2.userData in ["floor", "wall"]:
                        continue
                    other_fixture = f2 if f1.userData == "core" else f1
                    if other_fixture.userData not in ["meteor", "beam"]:
                        continue
                    impulse = manifold.points[0].normalImpulse
                    force = impulse / time_step
                    if force > self._max_force_on_core:
                        self._max_force_on_core = force
                        self._core_force_step = self._step_count
    def get_core_max_force(self):
        return self._max_force_on_core
    def reset_max_core_force(self):
        self._max_force_on_core = 0.0
    def get_first_joint_breach_step(self):
        return self._first_joint_breach_step
    def get_first_joint_breach_anchor(self):
        return self._first_joint_breach_anchor
    def get_joints_broken_count(self):
        return self._joints_broken_count
    def get_joint_peak_records(self):
        records = []
        for idx, j in enumerate(self._joints):
            if idx < len(self._joint_anchor_positions):
                records.append({
                    'peak_force': self._joint_peak_forces[idx] if idx < len(self._joint_peak_forces) else 0.0,
                    'peak_torque': self._joint_peak_torques[idx] if idx < len(self._joint_peak_torques) else 0.0,
                    'anchor': self._joint_anchor_positions[idx],
                })
        return records
    def get_joint_peak_records_v2(self):
        return [dict(record) for record in self._joint_telemetry_v2.values()]
    def get_joint_observation_errors(self):
        return {
            'count': self._joint_observation_error_count,
            'last_error': self._last_joint_observation_error,
        }
    def get_joint_limits(self):
        return {
            'max_joint_force': self._max_joint_force,
            'max_joint_torque': self._max_joint_torque,
        }
    def get_numerical_instability_count(self):
        return self._numerical_instability_count
    def get_joint_breach_events(self):
        return list(self._joint_breach_events)
    def get_total_boulder_ke(self):
        return self._total_boulder_ke
    def get_max_body_velocity(self):
        return self._max_body_velocity_seen
    def get_core_force_step(self):
        return self._core_force_step
    def get_terrain_bounds(self):
        out = {
            "core": {"x": self.CORE_X, "y": self.CORE_Y, "radius": self.CORE_RADIUS},
            "build_zone": {"x": [self.BUILD_ZONE_X_MIN, self.BUILD_ZONE_X_MAX], "y": [self.BUILD_ZONE_Y_MIN, self.BUILD_ZONE_Y_MAX]},
            "core_max_force": self.CORE_MAX_FORCE,
            "max_structure_mass": self.MAX_STRUCTURE_MASS,
            "max_structure_height": self.MAX_STRUCTURE_HEIGHT,
            "meteor_count": self._meteor_count,
            "meteor_spawn_interval": self._meteor_spawn_interval,
        }
        out["max_joint_force"] = self._max_joint_force
        out["max_joint_torque"] = self._max_joint_torque
        return out
