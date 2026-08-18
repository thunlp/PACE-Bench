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
        self._default_linear_damping = float(physics_config.get("linear_damping", 0.1))
        self._default_angular_damping = float(physics_config.get("angular_damping", 0.05))
        self._world = world(gravity=gravity, doSleep=True)
        self._bodies = []
        self._joints = []
        self._terrain_joints = []
        self._terrain_bodies = {}
        self._water_particles = []
        self._debris_bodies = []
        self.world = self._world
        self.bodies = self._bodies
        self.joints = self._joints
        self._create_terrain(terrain_config)
        self.DAM_X_LEFT = 12.0
        self.DAM_X_RIGHT = 14.0
        self.DOWNSTREAM_X_START = 14.0
        self.RESERVOIR_X_MAX = 12.0
        self.BUILD_ZONE_LEFT_X_MIN = float(terrain_config.get("build_zone_left_x_min", 12.4))
        self.BUILD_ZONE_LEFT_X_MAX = float(terrain_config.get("build_zone_left_x_max", 12.6))
        self.BUILD_ZONE_MIDDLE_X_MIN = float(terrain_config.get("build_zone_middle_x_min", 12.9))
        self.BUILD_ZONE_MIDDLE_X_MAX = float(terrain_config.get("build_zone_middle_x_max", 13.1))
        self.BUILD_ZONE_RIGHT_X_MIN = float(terrain_config.get("build_zone_right_x_min", 13.4))
        self.BUILD_ZONE_RIGHT_X_MAX = float(terrain_config.get("build_zone_right_x_max", 13.6))
        self.BUILD_ZONE_Y_MIN = 0.0
        self.BUILD_ZONE_Y_MAX = float(terrain_config.get("build_zone_y_max", 7.5))
        self.BUILD_ZONE_X_MIN = self.BUILD_ZONE_LEFT_X_MIN
        self.BUILD_ZONE_X_MAX = self.BUILD_ZONE_RIGHT_X_MAX
        self.MAX_STRUCTURE_MASS = float(terrain_config.get("max_structure_mass", 380.0))
        self.MAX_BEAM_COUNT = int(terrain_config.get("max_beam_count", 18))
        self.MIN_BEAM_COUNT = int(terrain_config.get("min_beam_count", 10))
        self.MAX_BEAMS_MIDDLE_STRIP = int(terrain_config.get("max_beams_middle_strip", 1))
        self.MAX_BEAMS_RIGHT_STRIP = int(terrain_config.get("max_beams_right_strip", 2))
        self.RESERVOIR_FILL_HEIGHT = float(terrain_config.get("fluid_height", 7.0))
        self.MAX_TERRAIN_ANCHORS = 0
        self.MIN_BEAM_BOTTOM_Y = float(terrain_config.get("min_beam_bottom_y", 0.5))
        self.MAX_BEAM_WIDTH = float(terrain_config.get("max_beam_width", 0.6))
        self.MAX_BEAM_HEIGHT = float(terrain_config.get("max_beam_height", 1.5))
        self.MAX_JOINT_COUNT = int(terrain_config.get("max_joint_count", 15))
        self.JOINT_BREAK_FORCE = float(terrain_config.get("joint_break_force", 50000.0))
        self.MIN_BEAMS_PER_BAND = int(terrain_config.get("min_beams_per_band", 3))
        self.MAX_LEAKAGE_RATE = float(terrain_config.get("max_leakage_rate", 0.001))
        self._create_water_particles(terrain_config)
        self._initial_particle_count = len(self._water_particles)
        self._step_count = 0
        self._surge_steps_applied = 0
        self._joint_force_history = {}
        self._joint_force_history_len = int(terrain_config.get("joint_break_consecutive_steps", 3))
        self._joint_force_history_len = max(1, min(self._joint_force_history_len, 10))
        default_impulses = [0.7, 0.85, 1.0, 1.15, 1.3, 1.4, 1.5, 1.6, 1.7]
        surge_impulses = terrain_config.get("surge_impulses")
        if surge_impulses is not None:
            self._surge_impulses = list(surge_impulses)
        else:
            self._surge_impulses = default_impulses
        self._surge_steps = [1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000][:len(self._surge_impulses)]
        if len(self._surge_steps) < len(self._surge_impulses):
            self._surge_impulses = self._surge_impulses[:len(self._surge_steps)]
        self._backward_slosh_steps = [1500, 3000, 4500, 6000, 7500, 9000, 10000]
        self._backward_slosh_impulse_x = float(terrain_config.get("backward_slosh_impulse_x", -0.7))
        self._upward_surge_steps = [2500, 5500, 8500]
        self._upward_surge_impulse_y = float(terrain_config.get("upward_surge_impulse_y", 1.0))
        self.MAX_STEPS = int(terrain_config.get("max_steps", 10000))
        self._joint_break_events = []
        self._joint_peak_forces = {}
        self._joint_force_limit = float(terrain_config.get("joint_break_force", 50000.0))
        self._joint_break_consecutive_steps = int(terrain_config.get("joint_break_consecutive_steps", 3))
    def _create_terrain(self, terrain_config: dict):
        floor_length = 40.0
        floor_height = 0.3
        floor = self._world.CreateStaticBody(
            position=(floor_length / 2, -floor_height / 2),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(floor_length / 2, floor_height / 2)),
                friction=0.6,
            ),
        )
        self._terrain_bodies["floor"] = floor
        wall_height = 10.0
        wall_width = 0.5
        left_wall = self._world.CreateStaticBody(
            position=(wall_width / 2, wall_height / 2),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(wall_width / 2, wall_height / 2)),
                friction=0.5,
            ),
        )
        self._terrain_bodies["left_wall"] = left_wall
        self._downstream_wall_y = wall_height / 2
        self._downstream_wall_half_w = 0.25
        downstream_wall_x0 = 13.85
        downstream_wall = self._world.CreateDynamicBody(
            position=(downstream_wall_x0, self._downstream_wall_y),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(self._downstream_wall_half_w, wall_height / 2)),
                density=0.0,
                friction=0.5,
            ),
        )
        downstream_wall.type = Box2D.b2.kinematicBody
        self._terrain_bodies["downstream_wall"] = downstream_wall
        self._debris_spawn_steps = [2000, 5000, 8000]
        self._debris_spawned = []
        self._earthquake_steps = [2500, 5000, 7500, 10000]
        self._earthquake_impulse_x = float(terrain_config.get("earthquake_impulse_x", 0.35))
        self._downstream_wall_amplitude = float(terrain_config.get("downstream_wall_amplitude", 0.4))
        self._downstream_wall_phase_divisor = float(terrain_config.get("downstream_wall_phase_divisor", 100.0))
        self._downstream_wall_phase_divisor = max(1.0, self._downstream_wall_phase_divisor)
        self._structure_friction = terrain_config.get("structure_friction")
        if self._structure_friction is not None:
            self._structure_friction = float(self._structure_friction)
        self._debris_linear_velocity_x = float(terrain_config.get("debris_linear_velocity_x", 2.2))
        self._debris_linear_velocity_y = float(terrain_config.get("debris_linear_velocity_y", 0.0))
    def _create_water_particles(self, terrain_config: dict):
        fluid_config = terrain_config.get("fluid", {})
        num_particles = int(fluid_config.get("count", 300))
        particle_radius = float(fluid_config.get("particle_radius", 0.12))
        fluid_density = float(fluid_config.get("density", 1000.0))
        seed = int(fluid_config.get("seed", 42))
        initial_flow_speed = float(fluid_config.get("initial_flow_speed", 0.65))
        fp_override = terrain_config.get("fluid_particle_friction")
        if fp_override is not None:
            particle_friction = float(fp_override)
        else:
            particle_friction = float(fluid_config.get("particle_friction", 0.1))
        pr_override = terrain_config.get("fluid_particle_restitution")
        if pr_override is not None:
            particle_restitution = float(pr_override)
        else:
            particle_restitution = float(fluid_config.get("particle_restitution", 0.05))
        rng = random.Random(seed)
        reservoir_x_min = 1.0
        reservoir_x_max = 11.0
        reservoir_y_min = particle_radius + 0.1
        reservoir_y_max = self.RESERVOIR_FILL_HEIGHT
        self.RESERVOIR_X_MIN = reservoir_x_min
        for _ in range(num_particles):
            x = rng.uniform(reservoir_x_min, reservoir_x_max)
            y = rng.uniform(reservoir_y_min, reservoir_y_max)
            mass = fluid_density * (math.pi * particle_radius ** 2)
            density = mass / (math.pi * particle_radius ** 2)
            particle = self._world.CreateDynamicBody(
                position=(x, y),
                fixtures=Box2D.b2FixtureDef(
                    shape=circleShape(radius=particle_radius),
                    density=density,
                    friction=particle_friction,
                    restitution=particle_restitution,
                ),
            )
            particle.linearDamping = self._default_linear_damping
            particle.angularDamping = self._default_angular_damping
            particle.linearVelocity = (initial_flow_speed, 0.0)
            self._water_particles.append(particle)
        self._initial_particle_count = len(self._water_particles)
    MIN_BEAM_SIZE = 0.2
    MAX_BEAM_SIZE = 4.0
    MAX_BEAM_WIDTH = 0.6
    MAX_BEAM_HEIGHT = 1.5
    def add_beam(self, x, y, width, height, angle=0, density=500.0):
        if len(self._bodies) >= self.MAX_BEAM_COUNT:
            raise ValueError(f"Beam count would exceed maximum {self.MAX_BEAM_COUNT}")
        max_w = getattr(self, 'MAX_BEAM_WIDTH', 0.6)
        max_h = getattr(self, 'MAX_BEAM_HEIGHT', 1.5)
        width = max(self.MIN_BEAM_SIZE, min(width, self.MAX_BEAM_SIZE, max_w))
        height = max(self.MIN_BEAM_SIZE, min(height, self.MAX_BEAM_SIZE, max_h))
        sf = self._structure_friction if self._structure_friction is not None else 0.5
        body = self._world.CreateDynamicBody(
            position=(x, y),
            angle=angle,
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(width / 2, height / 2)),
                density=density,
                friction=float(sf),
            ),
        )
        body.linearDamping = self._default_linear_damping
        body.angularDamping = self._default_angular_damping
        self._bodies.append(body)
        return body
    def add_joint(self, body_a, body_b, anchor_point, type='rigid'):
        if body_a is None:
            raise ValueError("add_joint: body_a cannot be None.")
        anchor_x, anchor_y = anchor_point[0], anchor_point[1]
        floor_body = self._terrain_bodies.get("floor")
        if body_b is None:
            body_b = floor_body
        if body_b is None:
            raise ValueError("add_joint: terrain body not found for anchor.")
        if body_b == floor_body:
            if len(self._terrain_joints) >= self.MAX_TERRAIN_ANCHORS:
                raise ValueError(f"Terrain anchor count would exceed maximum {self.MAX_TERRAIN_ANCHORS}")
        if body_b != floor_body:
            beam_joints = len(self._joints) - len(self._terrain_joints)
            max_joints = getattr(self, 'MAX_JOINT_COUNT', 15)
            if beam_joints >= max_joints:
                raise ValueError(f"Beam-to-beam joint count would exceed maximum {max_joints}")
        if type != 'rigid':
            type = 'rigid'
        joint = self._world.CreateWeldJoint(
            bodyA=body_a,
            bodyB=body_b,
            anchor=(anchor_x, anchor_y),
            collideConnected=False
        )
        self._joints.append(joint)
        if body_b == floor_body:
            self._terrain_joints.append(joint)
        return joint
    def get_terrain_joint_count(self):
        return len(self._terrain_joints)
    def get_structure_mass(self):
        total_mass = 0.0
        for body in self._bodies:
            total_mass += body.mass
        return total_mass
    def set_material_properties(self, body, restitution=0.1):
        for fixture in body.fixtures:
            fixture.restitution = float(restitution)
    def set_damping(self, body, linear=None, angular=None):
        if linear is not None:
            body.linearDamping = float(linear)
        if angular is not None:
            body.angularDamping = float(angular)
    def apply_force(self, body, force_vector):
        body.ApplyForce(force_vector, body.worldCenter, True)
    def _in_build_zone(self, x, y):
        in_left = self.BUILD_ZONE_LEFT_X_MIN <= x <= self.BUILD_ZONE_LEFT_X_MAX
        in_middle = getattr(self, 'BUILD_ZONE_MIDDLE_X_MIN', 12.9) <= x <= getattr(self, 'BUILD_ZONE_MIDDLE_X_MAX', 13.1)
        in_right = self.BUILD_ZONE_RIGHT_X_MIN <= x <= self.BUILD_ZONE_RIGHT_X_MAX
        in_y = self.BUILD_ZONE_Y_MIN <= y <= self.BUILD_ZONE_Y_MAX
        return (in_left or in_middle or in_right) and in_y
    def step(self, time_step):
        self._step_count += 1
        wall = self._terrain_bodies.get("downstream_wall")
        if wall is not None:
            amp = getattr(self, "_downstream_wall_amplitude", 0.4)
            ph = getattr(self, "_downstream_wall_phase_divisor", 100.0)
            new_x = 13.85 + amp * math.sin(self._step_count / ph)
            wall.position = (new_x, self._downstream_wall_y)
            wall.angle = 0.0
        for t in self._debris_spawn_steps:
            if self._step_count == t and t not in self._debris_spawned:
                debris = self._world.CreateDynamicBody(
                    position=(11.2, 3.8),
                    fixtures=Box2D.b2FixtureDef(
                        shape=polygonShape(box=(0.25, 0.25)),
                        density=200.0,
                        friction=0.4,
                        restitution=0.1,
                    ),
                )
                dvx = getattr(self, "_debris_linear_velocity_x", 2.2)
                dvy = getattr(self, "_debris_linear_velocity_y", 0.0)
                debris.linearVelocity = (dvx, dvy)
                debris.linearDamping = 0.05
                self._debris_bodies.append(debris)
                self._debris_spawned.append(t)
                break
        for t in self._earthquake_steps:
            if self._step_count == t:
                sign = 1 if (self._step_count // 2500) % 2 == 0 else -1
                for body in self._bodies:
                    if body is not None and body.active:
                        vx, vy = body.linearVelocity
                        body.linearVelocity = (vx + sign * self._earthquake_impulse_x, vy)
                break
        self._world.Step(time_step, 10, 10)
        if time_step > 0:
            inv_dt = 1.0 / time_step
            to_remove = []
            floor_body = self._terrain_bodies.get("floor")
            for joint in list(self._joints):
                if joint.bodyB == floor_body:
                    continue
                force = joint.GetReactionForce(inv_dt)
                mag = math.sqrt(force.x ** 2 + force.y ** 2)
                hist = self._joint_force_history.setdefault(joint, [])
                hist.append(mag)
                if len(hist) > self._joint_force_history_len:
                    hist.pop(0)
                threshold = getattr(self, 'JOINT_BREAK_FORCE', 50000.0)
                jid = id(joint)
                prev = self._joint_peak_forces.get(jid)
                if prev is None:
                    try:
                        anchor = joint.anchorB
                        ax, ay = float(anchor.x), float(anchor.y)
                        apos = (float(joint.bodyA.position.x), float(joint.bodyA.position.y))
                        bpos = (float(joint.bodyB.position.x), float(joint.bodyB.position.y))
                        self._joint_peak_forces[jid] = {
                            'anchor': (ax, ay),
                            'body_a_pos': apos,
                            'body_b_pos': bpos,
                            'peak_force': mag,
                            'first_seen_step': self._step_count,
                        }
                    except (AttributeError, TypeError, ValueError) as exc:
                        self._observation_errors.append(
                            f"joint peak metadata unavailable at step {self._step_count}: {exc}"
                        )
                        self._joint_peak_forces[jid] = {'peak_force': mag}
                elif mag > prev.get('peak_force', 0.0):
                    prev['peak_force'] = mag
                if len(hist) >= self._joint_force_history_len and all(h >= threshold for h in hist):
                    to_remove.append(joint)
                    try:
                        anchor = joint.anchorB
                        ax, ay = float(anchor.x), float(anchor.y)
                        amass = float(joint.bodyA.mass) if hasattr(joint.bodyA, 'mass') else 0.0
                        bmass = float(joint.bodyB.mass) if hasattr(joint.bodyB, 'mass') else 0.0
                        apos = (float(joint.bodyA.position.x), float(joint.bodyA.position.y))
                        bpos = (float(joint.bodyB.position.x), float(joint.bodyB.position.y))
                        self._joint_break_events.append({
                            'step': self._step_count,
                            'anchor': (ax, ay),
                            'body_a_pos': apos,
                            'body_b_pos': bpos,
                            'body_a_mass': amass,
                            'body_b_mass': bmass,
                            'force': mag,
                            'threshold': threshold,
                        })
                    except (AttributeError, TypeError, ValueError) as exc:
                        self._observation_errors.append(
                            f"joint break metadata unavailable at step {self._step_count}: {exc}"
                        )
            for joint in to_remove:
                self._joint_force_history.pop(joint, None)
                self._world.DestroyJoint(joint)
                if joint in self._joints:
                    self._joints.remove(joint)
        if self._surge_steps_applied < len(self._surge_steps) and self._step_count >= self._surge_steps[self._surge_steps_applied]:
            impulse = self._surge_impulses[self._surge_steps_applied]
            self._surge_steps_applied += 1
            for p in self._water_particles:
                if p is not None and p.active and p.position.x < self.RESERVOIR_X_MAX:
                    vx, vy = p.linearVelocity
                    p.linearVelocity = (vx + impulse, vy)
        for t in self._backward_slosh_steps:
            if self._step_count == t:
                for p in self._water_particles:
                    if p is not None and p.active and p.position.x < self.RESERVOIR_X_MAX:
                        vx, vy = p.linearVelocity
                        p.linearVelocity = (vx + self._backward_slosh_impulse_x, vy)
                break
        for t in self._upward_surge_steps:
            if self._step_count == t:
                for p in self._water_particles:
                    if p is not None and p.active and p.position.x < self.RESERVOIR_X_MAX:
                        vx, vy = p.linearVelocity
                        p.linearVelocity = (vx, vy + self._upward_surge_impulse_y)
                break
    def get_terrain_bounds(self):
        res_x_min = getattr(self, "RESERVOIR_X_MIN", 1.0)
        return {
            "reservoir": {"x_min": res_x_min, "x_max": self.RESERVOIR_X_MAX, "fill_height": self.RESERVOIR_FILL_HEIGHT},
            "dam_zone": {"x_min": self.DAM_X_LEFT, "x_max": self.DAM_X_RIGHT},
            "downstream_x_start": self.DOWNSTREAM_X_START,
            "build_zone_left": {"x": [self.BUILD_ZONE_LEFT_X_MIN, self.BUILD_ZONE_LEFT_X_MAX], "y": [self.BUILD_ZONE_Y_MIN, self.BUILD_ZONE_Y_MAX]},
            "build_zone_middle": {"x": [self.BUILD_ZONE_MIDDLE_X_MIN, self.BUILD_ZONE_MIDDLE_X_MAX], "y": [self.BUILD_ZONE_Y_MIN, self.BUILD_ZONE_Y_MAX]},
            "build_zone_right": {"x": [self.BUILD_ZONE_RIGHT_X_MIN, self.BUILD_ZONE_RIGHT_X_MAX], "y": [self.BUILD_ZONE_Y_MIN, self.BUILD_ZONE_Y_MAX]},
        }
    def get_joint_break_events(self):
        return list(self._joint_break_events)
    def get_joint_peak_forces(self):
        result = []
        threshold = getattr(self, 'JOINT_BREAK_FORCE', 50000.0)
        for jid, info in self._joint_peak_forces.items():
            entry = dict(info)
            entry['threshold'] = threshold
            pct = (info.get('peak_force', 0.0) / threshold * 100.0) if threshold > 0 else 0.0
            entry['pct_of_threshold'] = pct
            entry['tier'] = 'critical' if pct >= 80.0 else ('elevated' if pct >= 50.0 else 'nominal')
            result.append(entry)
        result.sort(key=lambda x: x.get('peak_force', 0.0), reverse=True)
        return result
    def get_joint_force_limit(self):
        return float(getattr(self, 'JOINT_BREAK_FORCE', 50000.0))
    def get_joint_break_consecutive_steps(self):
        return int(getattr(self, '_joint_force_history_len', 3))
    def get_max_steps(self):
        return int(getattr(self, 'MAX_STEPS', 10000))
    def get_surge_impulses(self):
        return list(getattr(self, '_surge_impulses', []))
    def get_surge_steps(self):
        return list(getattr(self, '_surge_steps', []))
    def get_backward_slosh_steps(self):
        return list(getattr(self, '_backward_slosh_steps', []))
    def get_backward_slosh_impulse_x(self):
        return float(getattr(self, '_backward_slosh_impulse_x', -0.7))
    def get_upward_surge_steps(self):
        return list(getattr(self, '_upward_surge_steps', []))
    def get_upward_surge_impulse_y(self):
        return float(getattr(self, '_upward_surge_impulse_y', 1.0))
    def get_earthquake_steps(self):
        return list(getattr(self, '_earthquake_steps', []))
    def get_earthquake_impulse_x(self):
        return float(getattr(self, '_earthquake_impulse_x', 0.35))
    def get_debris_spawn_steps(self):
        return list(getattr(self, '_debris_spawn_steps', []))
    def get_debris_velocity(self):
        return (float(getattr(self, '_debris_linear_velocity_x', 2.2)),
                float(getattr(self, '_debris_linear_velocity_y', 0.0)))
    def get_disturbance_timeline(self):
        timeline = []
        for s in sorted(set(self._surge_steps + self._backward_slosh_steps +
                            self._upward_surge_steps + self._earthquake_steps +
                            self._debris_spawn_steps)):
            events = []
            if s in self._surge_steps:
                idx = self._surge_steps.index(s)
                imp = self._surge_impulses[idx] if idx < len(self._surge_impulses) else 0.0
                events.append(('surge', imp))
            if s in self._backward_slosh_steps:
                events.append(('backward_slosh', self._backward_slosh_impulse_x))
            if s in self._upward_surge_steps:
                events.append(('upward_surge', self._upward_surge_impulse_y))
            if s in self._earthquake_steps:
                events.append(('earthquake', self._earthquake_impulse_x))
            if s in self._debris_spawn_steps:
                dvx, dvy = self.get_debris_velocity()
                events.append(('debris_spawn', dvx, dvy))
            timeline.append({'step': s, 'events': events})
        return timeline
    def get_leak_height_distribution(self):
        # Leakage is measured at the fixed downstream flood boundary.  Using the
        # oscillating wall's instantaneous upstream face would relabel contained
        # reservoir particles as leaked whenever the wall moved left.
        leak_x = self.DOWNSTREAM_X_START
        min_bottom = getattr(self, 'MIN_BEAM_BOTTOM_Y', 0.5)
        fill_height = getattr(self, 'RESERVOIR_FILL_HEIGHT', 7.0)
        bins = [
            ('underflow', 0.0, min_bottom),
            ('low', min_bottom, 2.5),
            ('mid', 2.5, 5.0),
            ('high', 5.0, fill_height),
            ('overtopping', fill_height, 999.0),
        ]
        counts = [0.0 for _ in bins]
        seepage_start = leak_x - 0.5
        for p in self._water_particles:
            if p is not None and p.active:
                x = p.position.x
                y = p.position.y
                weight = 1.0 if x > leak_x else (0.5 if x > seepage_start else 0.0)
                if weight > 0:
                    for i, (_, lo, hi) in enumerate(bins):
                        if lo <= y < hi:
                            counts[i] += weight
                            break
        result = {}
        for (label, lo, hi), cnt in zip(bins, counts):
            result[label] = {
                'count': cnt,
                'y_range': (lo, hi),
            }
        return result
    def get_beam_coverage_envelope(self):
        strips = {
            'left': (self.BUILD_ZONE_LEFT_X_MIN, self.BUILD_ZONE_LEFT_X_MAX),
            'middle': (getattr(self, 'BUILD_ZONE_MIDDLE_X_MIN', 12.9),
                       getattr(self, 'BUILD_ZONE_MIDDLE_X_MAX', 13.1)),
            'right': (self.BUILD_ZONE_RIGHT_X_MIN, self.BUILD_ZONE_RIGHT_X_MAX),
        }
        result = {}
        for strip_name, (x_min, x_max) in strips.items():
            covered = []
            for body in self._bodies:
                bx, by = body.position.x, body.position.y
                if not (x_min <= bx <= x_max):
                    continue
                hy = None
                try:
                    if body.fixtures:
                        shape = body.fixtures[0].shape
                        if hasattr(shape, 'box'):
                            _, hy = shape.box
                except (IndexError, TypeError, AttributeError):
                    pass
                if hy is None:
                    continue
                bot = by - hy
                top = by + hy
                covered.append((bot, top))
            covered.sort()
            merged = []
            for bot, top in covered:
                if merged and bot <= merged[-1][1] + 1e-4:
                    merged[-1] = (merged[-1][0], max(merged[-1][1], top))
                else:
                    merged.append((bot, top))
            fill_height = getattr(self, 'RESERVOIR_FILL_HEIGHT', 7.0)
            total_span = 0.0
            for bot, top in merged:
                span_bot = max(bot, getattr(self, 'MIN_BEAM_BOTTOM_Y', 0.5))
                span_top = min(top, fill_height)
                if span_top > span_bot:
                    total_span += span_top - span_bot
            result[strip_name] = {
                'intervals': merged,
                'beam_count': len(covered),
                'coverage_span': total_span,
                'fill_height': fill_height,
                'max_coverage_possible': max(0.0, fill_height - getattr(self, 'MIN_BEAM_BOTTOM_Y', 0.5)),
            }
        return result
    def get_numerical_health_warnings(self):
        warnings = []
        for body in self._bodies:
            if body is None or not body.active:
                continue
            vx, vy = body.linearVelocity.x, body.linearVelocity.y
            vmag = math.sqrt(vx ** 2 + vy ** 2)
            if not (math.isfinite(vx) and math.isfinite(vy)):
                warnings.append({'type': 'non_finite_velocity', 'body_pos': (body.position.x, body.position.y), 'velocity': (vx, vy)})
            elif vmag > 100.0:
                warnings.append({'type': 'extreme_beam_velocity', 'body_pos': (body.position.x, body.position.y), 'speed': vmag})
            px, py = body.position.x, body.position.y
            if not (math.isfinite(px) and math.isfinite(py)):
                warnings.append({'type': 'non_finite_position', 'body_pos': (px, py)})
        for p in self._water_particles:
            if p is None or not p.active:
                continue
            vx, vy = p.linearVelocity.x, p.linearVelocity.y
            vmag = math.sqrt(vx ** 2 + vy ** 2)
            if vmag > 100.0:
                warnings.append({'type': 'extreme_particle_velocity', 'body_pos': (p.position.x, p.position.y), 'speed': vmag})
                break
            if not (math.isfinite(vx) and math.isfinite(vy)):
                warnings.append({'type': 'non_finite_particle_velocity', 'body_pos': (p.position.x, p.position.y), 'velocity': (vx, vy)})
                break
        for debris in self._debris_bodies:
            if debris is None or not debris.active:
                continue
            vx, vy = debris.linearVelocity.x, debris.linearVelocity.y
            vmag = math.sqrt(vx ** 2 + vy ** 2)
            if not (math.isfinite(vx) and math.isfinite(vy)):
                warnings.append({'type': 'non_finite_debris_velocity', 'body_pos': (debris.position.x, debris.position.y)})
            elif vmag > 100.0:
                warnings.append({'type': 'extreme_debris_velocity', 'body_pos': (debris.position.x, debris.position.y), 'speed': vmag})
        return warnings
    def get_observation_errors(self):
        return list(self._observation_errors)
    def get_initial_particle_count(self):
        return self._initial_particle_count
    def get_particle_count(self):
        return len([p for p in self._water_particles if p is not None and p.active])
    def get_leaked_particle_count(self):
        count = 0.0
        leak_x = self.DOWNSTREAM_X_START
        seepage_start = leak_x - 0.5
        for p in self._water_particles:
            if p is not None and p.active:
                x = p.position.x
                if x > leak_x:
                    count += 1.0
                elif x > seepage_start:
                    count += 0.5
        return count
