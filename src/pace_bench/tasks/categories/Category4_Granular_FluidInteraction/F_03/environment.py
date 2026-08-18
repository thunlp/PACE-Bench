import Box2D

from Box2D.b2 import (world, polygonShape, circleShape, staticBody, dynamicBody, weldJoint)

import math

import random

class Sandbox:
    def __init__(self, *, terrain_config=None, physics_config=None):
        terrain_config = terrain_config or {}
        physics_config = physics_config or {}
        self._terrain_config = dict(terrain_config)
        self._physics_config = dict(physics_config)
        self._observation_errors = []
        gravity = tuple(physics_config.get("gravity", (0, -10)))
        self._default_linear_damping = float(physics_config.get("linear_damping", 0.02))
        self._default_angular_damping = float(physics_config.get("angular_damping", 0.02))
        self._joint_max_force = float(physics_config.get("joint_max_force", float("inf")))
        self._joint_max_torque = float(physics_config.get("joint_max_torque", float("inf")))
        self._world = world(gravity=gravity, doSleep=True)
        self._bodies = []
        self._joints = []
        self._revolute_joints = []
        self._terrain_bodies = {}
        self._particles = []
        self._scoop_bodies = []
        self._prev_scoop_state = {}
        self.world = self._world
        self.bodies = self._bodies
        self.joints = self._joints
        self.PIT_X_MIN = 0.0
        self.PIT_X_MAX = 5.0
        self.PIT_Y_MIN = 0.0
        self.PIT_Y_MAX = 2.5
        self.HOPPER_X_MIN = -6.0
        self.HOPPER_X_MAX = -4.0
        self.HOPPER_Y_MIN = 0.5
        self.HOPPER_Y_MAX = 5.0
        self.HOPPER_CENTER_X = -5.0
        self.HOPPER_CENTER_Y = 3.0
        self.BASE_X = -2.0
        self.BASE_Y = 0.0
        self.BUILD_ZONE_X_MIN = -4.0
        self.BUILD_ZONE_X_MAX = float(terrain_config.get("build_zone_x_max", 2.0))
        self.BUILD_ZONE_Y_MIN = 0.0
        self.BUILD_ZONE_Y_MAX = float(terrain_config.get("build_zone_y_max", 5.0))
        self.MAX_STRUCTURE_MASS = float(terrain_config.get("max_structure_mass", 800.0))
        self.MIN_PARTICLES_IN_HOPPER = int(terrain_config.get("min_particles_in_hopper", 15))
        self.MAX_TIME_SECONDS = float(terrain_config.get("max_time_seconds", 40.0))
        self.PIT_DRIFT_FORCE = float(terrain_config.get("pit_drift_force", 0.0))
        self.HOPPER_VALID_X_MIN = float(terrain_config.get("hopper_valid_x_min", self.HOPPER_X_MIN))
        self.HOPPER_VALID_X_MAX = float(terrain_config.get("hopper_valid_x_max", self.HOPPER_X_MAX))
        self.HOPPER_VALID_Y_MIN = float(terrain_config.get("hopper_valid_y_min", self.HOPPER_Y_MIN))
        self.HOPPER_VALID_Y_MAX = float(terrain_config.get("hopper_valid_y_max", self.HOPPER_Y_MAX))
        self.SCOOP_CAPACITY = int(terrain_config.get("scoop_capacity", 999))
        self.agent_arm_joint = None
        self.agent_bucket_joint = None
        self._step_counter = 0
        self._scoop_traj_x_min = float('inf')
        self._scoop_traj_x_max = float('-inf')
        self._scoop_traj_y_min = float('inf')
        self._scoop_traj_y_max = float('-inf')
        self._peak_joint_force = 0.0
        self._peak_joint_torque = 0.0
        self._peak_body_speed = 0.0
        self._peak_angular_velocity = 0.0
        self._joint_break_events = []
        self._carry_log = []
        self._max_carry_log_entries = 120
        self._max_particle_speed = 0.0
        self._prev_particle_vel = {}
        self._create_terrain(terrain_config)
        self._create_particles(terrain_config)
    def has_central_wall(self):
        return self._terrain_bodies.get("central_wall") is not None
    def _create_terrain(self, terrain_config: dict):
        floor_length = 20.0
        floor_height = 0.3
        floor_center_x = 0.0
        floor = self._world.CreateStaticBody(
            position=(floor_center_x, -floor_height / 2),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(floor_length / 2, floor_height / 2)),
                friction=0.5,
            ),
        )
        self._terrain_bodies["floor"] = floor
        if terrain_config.get("central_wall", True):
            wall_x = -1.0
            wall_bottom = 0.5
            wall_top = 1.5
            wall_height = wall_top - wall_bottom
            wall_half_w = 0.12
            central_wall = self._world.CreateStaticBody(
                position=(wall_x, (wall_bottom + wall_top) / 2),
                fixtures=Box2D.b2FixtureDef(
                    shape=polygonShape(box=(wall_half_w, wall_height / 2)),
                    friction=0.6,
                    restitution=0.05,
                ),
            )
            self._terrain_bodies["central_wall"] = central_wall
        hopper_w = 2.0
        hopper_h = 2.0
        hopper_body = self._world.CreateStaticBody(
            position=(self.HOPPER_CENTER_X, self.HOPPER_CENTER_Y),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(hopper_w / 2, hopper_h / 2)),
                friction=0.4,
                isSensor=True,
            ),
        )
        self._terrain_bodies["hopper"] = hopper_body
    def _create_particles(self, terrain_config: dict):
        pit_config = terrain_config.get("particles", {})
        num_particles = int(pit_config.get("count", 200))
        particle_radius = float(pit_config.get("radius", 0.06))
        density = float(pit_config.get("density", 1500.0))
        friction = float(pit_config.get("friction", 0.7))
        seed = int(pit_config.get("seed", 42))
        rng = random.Random(seed)
        for _ in range(num_particles):
            x = rng.uniform(self.PIT_X_MIN + particle_radius, self.PIT_X_MAX - particle_radius)
            y = rng.uniform(self.PIT_Y_MIN + particle_radius, self.PIT_Y_MAX - particle_radius)
            p = self._world.CreateDynamicBody(
                position=(x, y),
                fixtures=Box2D.b2FixtureDef(
                    shape=circleShape(radius=particle_radius),
                    density=density,
                    friction=friction,
                    restitution=0.05,
                ),
            )
            p.linearDamping = self._default_linear_damping
            p.angularDamping = self._default_angular_damping
            self._particles.append(p)
        self._initial_particle_count = len(self._particles)
    MIN_BEAM_SIZE = 0.1
    MAX_BEAM_SIZE = 1.5
    def add_beam(self, x, y, width, height, angle=0, density=300.0):
        width = max(self.MIN_BEAM_SIZE, min(width, self.MAX_BEAM_SIZE))
        height = max(self.MIN_BEAM_SIZE, min(height, self.MAX_BEAM_SIZE))
        body = self._world.CreateDynamicBody(
            position=(x, y),
            angle=angle,
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(width / 2, height / 2)),
                density=density,
                friction=0.5,
            ),
        )
        body.linearDamping = self._default_linear_damping
        body.angularDamping = self._default_angular_damping
        self._bodies.append(body)
        return body
    def add_anchored_base(self, x, y, width, height, angle=0, density=400.0):
        beam = self.add_beam(x, y, width, height, angle=angle, density=density)
        self.add_joint(beam, None, (x, 0.0))
        return beam
    def add_bucket(self, x, y, width, height, angle=0, density=280.0):
        beam = self.add_beam(x, y, width, height, angle=angle, density=density)
        self.set_material_properties(beam, restitution=0.05)
        return beam
    def register_scoop_body(self, body):
        if body is not None and body not in self._scoop_bodies:
            self._scoop_bodies.append(body)
    def add_scoop(self, x, y, width, height, angle=0, density=280.0):
        w, h = width, height
        body = self._world.CreateDynamicBody(
            position=(x, y),
            angle=angle,
        )
        back_verts = [(-w/2, 0), (-w/2, h/2), (0, h/2), (0, 0)]
        body.CreateFixture(
            shape=Box2D.b2PolygonShape(vertices=back_verts),
            density=density,
            friction=0.6,
            restitution=0.05,
        )
        floor_verts = [(-w/2, -h/2), (-w/2, 0), (0, 0), (0, -h/2)]
        body.CreateFixture(
            shape=Box2D.b2PolygonShape(vertices=floor_verts),
            density=density,
            friction=0.7,
            restitution=0.05,
        )
        body.linearDamping = self._default_linear_damping
        body.angularDamping = self._default_angular_damping
        self._bodies.append(body)
        self._scoop_bodies.append(body)
        return body
    def add_joint(self, body_a, body_b, anchor_point, type='rigid'):
        if body_a is None:
            raise ValueError("add_joint: body_a cannot be None.")
        anchor_x, anchor_y = anchor_point[0], anchor_point[1]
        if body_b is None:
            body_b = self._terrain_bodies.get("floor")
            if body_b is None:
                raise ValueError("add_joint: floor not found.")
        joint = self._world.CreateWeldJoint(
            bodyA=body_a,
            bodyB=body_b,
            anchor=(anchor_x, anchor_y),
            collideConnected=False
        )
        self._joints.append(joint)
        return joint
    def add_revolute_joint(self, body_a, body_b, anchor_point, enable_motor=False, motor_speed=0.0, max_motor_torque=100.0):
        if body_a is None or body_b is None:
            raise ValueError("add_revolute_joint: body_a and body_b cannot be None.")
        anchor_x, anchor_y = anchor_point[0], anchor_point[1]
        anchor_world = Box2D.b2Vec2(anchor_x, anchor_y)
        jd = Box2D.b2RevoluteJointDef()
        jd.Initialize(body_a, body_b, anchor_world)
        jd.collideConnected = False
        jd.enableMotor = bool(enable_motor)
        jd.motorSpeed = float(motor_speed)
        jd.maxMotorTorque = float(max_motor_torque)
        joint = self._world.CreateJoint(jd)
        self._joints.append(joint)
        self._revolute_joints.append(joint)
        return joint
    def get_structure_mass(self):
        return sum(b.mass for b in self._bodies)
    def set_material_properties(self, body, restitution=0.1):
        for fixture in body.fixtures:
            fixture.restitution = float(restitution)
    def step(self, time_step):
        self._step_counter += 1
        drift = getattr(self, "PIT_DRIFT_FORCE", 0.0)
        if drift > 0:
            for p in self._particles:
                if p is None or not p.active:
                    continue
                px, py = p.position.x, p.position.y
                if self.PIT_X_MIN <= px <= self.PIT_X_MAX and self.PIT_Y_MIN <= py <= self.PIT_Y_MAX:
                    p.ApplyForce((drift, 0), p.position, wake=True)
        DUMP_ANGLE_THRESHOLD = 0.6
        CARRY_MARGIN = 2.0
        scoop_cap = getattr(self, "SCOOP_CAPACITY", 999)
        for body in self._scoop_bodies:
            if not body.active:
                continue
            bx, by = body.position.x, body.position.y
            ba = body.angle
            prev = self._prev_scoop_state.get(id(body))
            if prev is None:
                self._prev_scoop_state[id(body)] = (bx, by)
                prev = (bx, by)
            w, h = 0.6, 0.35
            try:
                for fixt in body.fixtures:
                    if hasattr(fixt.shape, 'vertices') and fixt.shape.vertices:
                        vs = list(fixt.shape.vertices)
                        if vs:
                            xs = [v[0] for v in vs]
                            ys = [v[1] for v in vs]
                            w = max(w, max(xs) - min(xs) + 0.2)
                            h = max(h, max(ys) - min(ys) + 0.2)
            except (AttributeError, TypeError, ValueError) as exc:
                self._observation_errors.append(
                    f"scoop geometry observation unavailable at step {self._step_counter}: {exc}"
                )
            dx = w / 2 + CARRY_MARGIN
            dy = h / 2 + CARRY_MARGIN
            over_hopper = (self.HOPPER_X_MIN <= bx <= self.HOPPER_X_MAX and by >= self.HOPPER_Y_MIN)
            dumping = ba > DUMP_ANGLE_THRESHOLD and over_hopper
            carried = 0
            for p in self._particles:
                if p is None or not p.active:
                    continue
                if carried >= scoop_cap:
                    break
                px, py = p.position.x, p.position.y
                in_aabb = abs(px - bx) <= dx and abs(py - by) <= dy
                if in_aabb and not dumping:
                    p.linearVelocity = body.linearVelocity
                    p.angularVelocity = body.angularVelocity
                    if prev is not None:
                        p.position = (px + (bx - prev[0]), py + (by - prev[1]))
                    carried += 1
            over_pit = (self.PIT_X_MIN <= bx <= self.PIT_X_MAX and
                        self.PIT_Y_MIN <= by <= self.PIT_Y_MAX)
            if carried > 0 or dumping or self._step_counter % 30 == 0:
                entry = {
                    "step": self._step_counter,
                    "carried": carried,
                    "over_pit": over_pit,
                    "over_hopper": over_hopper,
                    "dumping": dumping,
                    "scoop_x": round(bx, 3),
                    "scoop_y": round(by, 3),
                    "scoop_angle": round(ba, 3),
                }
                if len(self._carry_log) >= self._max_carry_log_entries:
                    self._carry_log.pop(0)
                self._carry_log.append(entry)
        self._world.Step(time_step, 10, 10)
        for body in self._scoop_bodies:
            if body is None or not body.active:
                continue
            bx, by = body.position.x, body.position.y
            if bx < self._scoop_traj_x_min:
                self._scoop_traj_x_min = bx
            if bx > self._scoop_traj_x_max:
                self._scoop_traj_x_max = bx
            if by < self._scoop_traj_y_min:
                self._scoop_traj_y_min = by
            if by > self._scoop_traj_y_max:
                self._scoop_traj_y_max = by
            lv = body.linearVelocity
            speed = (lv.x * lv.x + lv.y * lv.y) ** 0.5
            if math.isfinite(speed) and speed > self._peak_body_speed:
                self._peak_body_speed = speed
            av = abs(getattr(body, 'angularVelocity', 0.0))
            if math.isfinite(av) and av > self._peak_angular_velocity:
                self._peak_angular_velocity = av
        for body in self._scoop_bodies:
            if body.active:
                self._prev_scoop_state[id(body)] = (body.position.x, body.position.y)
        if self._joint_max_force < float("inf") or self._joint_max_torque < float("inf"):
            inv_dt = 1.0 / time_step if time_step > 0 else 0.0
            to_destroy = []
            for j in list(self._joints):
                try:
                    force = j.GetReactionForce(inv_dt).length
                    torque = abs(j.GetReactionTorque(inv_dt))
                    if math.isfinite(force) and force > self._peak_joint_force:
                        self._peak_joint_force = force
                    if math.isfinite(torque) and torque > self._peak_joint_torque:
                        self._peak_joint_torque = torque
                    if force > self._joint_max_force or torque > self._joint_max_torque:
                        to_destroy.append(j)
                        self._joint_break_events.append({
                            "step": self._step_counter,
                            "force_N": round(force, 2),
                            "torque_Nm": round(torque, 2),
                            "limit_force_N": round(self._joint_max_force, 2),
                            "limit_torque_Nm": round(self._joint_max_torque, 2),
                        })
                except (AttributeError, TypeError, ValueError) as exc:
                    self._observation_errors.append(
                        f"joint reaction observation unavailable at step {self._step_counter}: {exc}"
                    )
            for j in to_destroy:
                if j in self._joints:
                    self._world.DestroyJoint(j)
                    self._joints.remove(j)
                    if j in self._revolute_joints:
                        self._revolute_joints.remove(j)
        margin = 1.0
        x_lo = self.HOPPER_X_MIN - margin
        x_hi = self.HOPPER_X_MAX + margin
        y_lo = self.HOPPER_Y_MIN
        y_hi = self.HOPPER_Y_MAX
        for p in self._particles:
            if p is None or not p.active:
                continue
            x, y = p.position.x, p.position.y
            if x_lo <= x <= x_hi and y_lo <= y <= y_hi:
                cx = max(self.HOPPER_X_MIN, min(self.HOPPER_X_MAX, x))
                cy = max(self.HOPPER_Y_MIN, min(self.HOPPER_Y_MAX, y))
                p.position = (cx, cy)
                p.linearVelocity = (0.0, 0.0)
                p.angularVelocity = 0.0
        for p in self._particles:
            if p is None or not p.active:
                continue
            lv = p.linearVelocity
            ps = (lv.x * lv.x + lv.y * lv.y) ** 0.5
            if math.isfinite(ps) and ps > self._max_particle_speed:
                self._max_particle_speed = ps
    def get_terrain_bounds(self):
        return {
            "pit": {"x_min": self.PIT_X_MIN, "x_max": self.PIT_X_MAX,
                    "y_min": self.PIT_Y_MIN, "y_max": self.PIT_Y_MAX},
            "hopper": {"x_min": self.HOPPER_X_MIN, "x_max": self.HOPPER_X_MAX,
                       "y_min": self.HOPPER_Y_MIN, "y_max": self.HOPPER_Y_MAX},
            "build_zone": {"x": [self.BUILD_ZONE_X_MIN, self.BUILD_ZONE_X_MAX],
                           "y": [self.BUILD_ZONE_Y_MIN, self.BUILD_ZONE_Y_MAX]},
        }
    def get_initial_particle_count(self):
        return self._initial_particle_count
    def get_particles_in_hopper_count(self):
        x_min = getattr(self, "HOPPER_VALID_X_MIN", self.HOPPER_X_MIN)
        x_max = getattr(self, "HOPPER_VALID_X_MAX", self.HOPPER_X_MAX)
        y_min = getattr(self, "HOPPER_VALID_Y_MIN", self.HOPPER_Y_MIN)
        y_max = getattr(self, "HOPPER_VALID_Y_MAX", self.HOPPER_Y_MAX)
        count = 0
        for p in self._particles:
            if p is None or not p.active:
                continue
            x, y = p.position.x, p.position.y
            if x_min <= x <= x_max and y_min <= y <= y_max:
                count += 1
        return count
    def get_particles_in_truck_count(self):
        return self.get_particles_in_hopper_count()
    def get_particles_in_pit_count(self):
        cnt = 0
        for p in self._particles:
            if p is None or not p.active:
                continue
            x, y = p.position.x, p.position.y
            if self.PIT_X_MIN <= x <= self.PIT_X_MAX and self.PIT_Y_MIN <= y <= self.PIT_Y_MAX:
                cnt += 1
        return cnt
    def get_particles_escaped_count(self):
        in_pit = set()
        in_hop = set()
        for p in self._particles:
            if p is None or not p.active:
                continue
            pid = id(p)
            x, y = p.position.x, p.position.y
            if self.PIT_X_MIN <= x <= self.PIT_X_MAX and self.PIT_Y_MIN <= y <= self.PIT_Y_MAX:
                in_pit.add(pid)
            if (self.HOPPER_VALID_X_MIN <= x <= self.HOPPER_VALID_X_MAX and
                    self.HOPPER_VALID_Y_MIN <= y <= self.HOPPER_VALID_Y_MAX):
                in_hop.add(pid)
        total_active = sum(1 for p in self._particles if p is not None and p.active)
        return max(0, total_active - len(in_pit | in_hop))
    def get_step_counter(self):
        return self._step_counter
    def get_scoop_trajectory_envelope(self):
        if (self._scoop_traj_x_min > self._scoop_traj_x_max or
                not math.isfinite(self._scoop_traj_x_min)):
            return None
        return (
            self._scoop_traj_x_min,
            self._scoop_traj_x_max,
            self._scoop_traj_y_min,
            self._scoop_traj_y_max,
        )
    def get_peak_joint_force(self):
        return self._peak_joint_force
    def get_peak_joint_torque(self):
        return self._peak_joint_torque
    def get_joint_break_events(self):
        return list(self._joint_break_events)
    def get_peak_body_speed(self):
        return self._peak_body_speed
    def get_peak_angular_velocity(self):
        return self._peak_angular_velocity
    def get_max_particle_speed(self):
        return self._max_particle_speed
    def get_carry_log(self):
        return list(self._carry_log)
    def get_hopper_valid_bounds(self):
        return {
            "x_min": getattr(self, "HOPPER_VALID_X_MIN", self.HOPPER_X_MIN),
            "x_max": getattr(self, "HOPPER_VALID_X_MAX", self.HOPPER_X_MAX),
            "y_min": getattr(self, "HOPPER_VALID_Y_MIN", self.HOPPER_Y_MIN),
            "y_max": getattr(self, "HOPPER_VALID_Y_MAX", self.HOPPER_Y_MAX),
        }
    def get_all_forensic_state(self):
        traj = self.get_scoop_trajectory_envelope()
        return {
            "step_counter": self._step_counter,
            "particles_in_pit": self.get_particles_in_pit_count(),
            "particles_escaped": self.get_particles_escaped_count(),
            "scoop_traj_x_min": traj[0] if traj else None,
            "scoop_traj_x_max": traj[1] if traj else None,
            "scoop_traj_y_min": traj[2] if traj else None,
            "scoop_traj_y_max": traj[3] if traj else None,
            "peak_joint_force": self._peak_joint_force,
            "peak_joint_torque": self._peak_joint_torque,
            "joint_break_events": list(self._joint_break_events),
            "peak_body_speed": self._peak_body_speed,
            "peak_angular_velocity": self._peak_angular_velocity,
            "max_particle_speed": self._max_particle_speed,
            "carry_log": list(self._carry_log),
            "observation_errors": list(self._observation_errors),
        }
