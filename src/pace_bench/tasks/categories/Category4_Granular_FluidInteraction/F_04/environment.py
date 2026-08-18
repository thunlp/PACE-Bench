import Box2D

from Box2D.b2 import (world, polygonShape, circleShape, staticBody, dynamicBody, weldJoint)

import math

import random

class Sandbox:
    MAX_STEPS = 10000
    def __init__(self, *, terrain_config=None, physics_config=None):
        terrain_config = terrain_config or {}
        physics_config = physics_config or {}
        self._terrain_config = dict(terrain_config)
        self._physics_config = dict(physics_config)
        gravity = tuple(physics_config.get("gravity", (0, -10)))
        self._default_linear_damping = float(physics_config.get("linear_damping", 0.02))
        self._default_angular_damping = float(physics_config.get("angular_damping", 0.02))
        self._gravity_base = gravity
        self._gravity_osc_amplitude = float(physics_config.get("gravity_oscillation_amplitude", 0.0))
        self._gravity_osc_period = float(physics_config.get("gravity_oscillation_period", 1.0))
        self._world = world(gravity=gravity, doSleep=True)
        self._bodies = []
        self._joints = []
        self._terrain_bodies = {}
        self._particles_small = []
        self._particles_medium = []
        self._particles_large = []
        self._step_count = 0
        self.WIND_AMPLITUDE = float(terrain_config.get("wind_amplitude", 20.0))
        self.WIND_PERIOD_STEPS = int(terrain_config.get("wind_period_steps", 450))
        self.GUST_AMPLITUDE = float(terrain_config.get("gust_amplitude", 28.0))
        self.GUST_PERIOD_STEPS = int(terrain_config.get("gust_period_steps", 200))
        self.world = self._world
        self.bodies = self._bodies
        self.joints = self._joints
        self.FEED_X_MIN = float(terrain_config.get("feed_x_min", 5.2))
        self.FEED_X_MAX = float(terrain_config.get("feed_x_max", 6.9))
        self.FEED_Y_MIN = float(terrain_config.get("feed_y_min", 3.0))
        self.FEED_Y_MAX = float(terrain_config.get("feed_y_max", 5.0))
        self.SMALL_ZONE_Y_MAX = 1.92
        self.MEDIUM_ZONE_Y_MIN = 1.92
        self.MEDIUM_ZONE_Y_MAX = 2.52
        self.LARGE_ZONE_Y_MIN = 2.52
        self.BUILD_ZONE_X_MIN = float(terrain_config.get("build_x_min", 5.20))
        self.BUILD_ZONE_X_MAX = float(terrain_config.get("build_x_max", 6.90))
        self.BUILD_ZONE_Y_MIN = 1.72
        self.BUILD_ZONE_Y_MAX = 2.45
        self.MAX_STRUCTURE_MASS = float(terrain_config.get("max_structure_mass", 75.0))
        self.MAX_BEAMS = int(terrain_config.get("max_beams", 6))
        self.MIN_PURITY = float(terrain_config.get("min_purity", 0.35))
        self._pending_second_wave = []
        self._pending_third_wave = []
        self._wave_events = []
        self.SECOND_WAVE_STEP = int(terrain_config.get("second_wave_step", 1800))
        self.THIRD_WAVE_STEP = int(terrain_config.get("third_wave_step", 3600))
        self._beam_fixture_friction = float(terrain_config.get("beam_friction", 0.4))
        self._create_terrain(terrain_config)
        self._create_particles(terrain_config)
        self._create_baffles(terrain_config)
        self._create_sweepers(terrain_config)
    def _create_terrain(self, terrain_config: dict):
        floor_length = float(terrain_config.get("floor_length", 16.0))
        self.FLOOR_LENGTH = floor_length
        floor_height = 0.3
        floor = self._world.CreateStaticBody(
            position=(floor_length / 2, -floor_height / 2),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(floor_length / 2, floor_height / 2)),
                friction=0.4,
            ),
        )
        self._terrain_bodies["floor"] = floor
    def _create_baffles(self, terrain_config: dict):
        baffle_config = terrain_config.get("baffles", {})
        enabled = baffle_config.get("enabled", True)
        if not enabled:
            return
        x_positions = baffle_config.get("x_positions", [5.45, 5.9, 6.35, 6.75])
        y_bottom = baffle_config.get("y_bottom", 2.4)
        y_top = baffle_config.get("y_top", 5.2)
        half_w = baffle_config.get("half_width", 0.04)
        half_h = (y_top - y_bottom) / 2
        cy = (y_top + y_bottom) / 2
        for i, x in enumerate(x_positions):
            baffle = self._world.CreateStaticBody(
                position=(x, cy),
                fixtures=Box2D.b2FixtureDef(
                    shape=polygonShape(box=(half_w, half_h)),
                    friction=0.35,
                    restitution=0.25,
                ),
            )
            self._terrain_bodies[f"baffle_{i}"] = baffle
    def _create_sweepers(self, terrain_config: dict):
        sweep_config = terrain_config.get("sweeper", {})
        if not sweep_config.get("enabled", True):
            return
        kinematic_body_type = Box2D.b2.kinematicBody
        y1 = float(sweep_config.get("y1", 4.0))
        x_min1 = float(sweep_config.get("x_min1", 5.25))
        x_max1 = float(sweep_config.get("x_max1", 6.85))
        half_w = float(sweep_config.get("half_width", 0.5))
        half_h = float(sweep_config.get("half_height", 0.05))
        speed_scale = float(sweep_config.get("speed_scale", 1.0))
        v1 = float(sweep_config.get("v_sweep1", 0.09)) * speed_scale
        body1 = self._world.CreateDynamicBody(
            position=((x_min1 + x_max1) / 2, y1),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(half_w, half_h)),
                friction=0.45,
                restitution=0.35,
            ),
        )
        body1.type = kinematic_body_type
        body1.userData = {"sweeper": True, "x_min": x_min1, "x_max": x_max1, "v_sweep": v1}
        self._terrain_bodies["sweeper1"] = body1
        if sweep_config.get("sweeper2_enabled", True):
            y2 = float(sweep_config.get("y2", 4.5))
            x_min2 = float(sweep_config.get("x_min2", 5.3))
            x_max2 = float(sweep_config.get("x_max2", 6.8))
            v2 = -float(sweep_config.get("v_sweep2", 0.05)) * speed_scale
            body2 = self._world.CreateDynamicBody(
                position=((x_min2 + x_max2) / 2, y2),
                fixtures=Box2D.b2FixtureDef(
                    shape=polygonShape(box=(half_w, half_h)),
                    friction=0.45,
                    restitution=0.35,
                ),
            )
            body2.type = kinematic_body_type
            body2.userData = {"sweeper": True, "x_min": x_min2, "x_max": x_max2, "v_sweep": v2}
            self._terrain_bodies["sweeper2"] = body2
    def _create_particles(self, terrain_config: dict):
        mix_config = terrain_config.get("mix", {})
        r_s = float(mix_config.get("radius_small", 0.06))
        r_m = float(mix_config.get("radius_medium", 0.10))
        r_l = float(mix_config.get("radius_large", 0.14))
        radius_jitter = float(mix_config.get("radius_jitter", 0.006))
        density = float(mix_config.get("density", 800.0))
        friction = float(mix_config.get("friction", 0.32))
        particle_restitution = float(mix_config.get("restitution", 0.10))
        n_first = int(mix_config.get("count_first_wave", 15))
        n_small = int(mix_config.get("count_small", n_first))
        n_medium = int(mix_config.get("count_medium", n_first))
        n_large = int(mix_config.get("count_large", n_first))
        n_third = int(mix_config.get("count_third_wave", 15))
        n_third_small = int(mix_config.get("count_third_small", n_third))
        n_third_medium = int(mix_config.get("count_third_medium", n_third))
        n_third_large = int(mix_config.get("count_third_large", n_third))
        rng = random.Random(42)
        for _ in range(n_small):
            radius_small = r_s + rng.uniform(-radius_jitter, radius_jitter)
            radius_small = max(0.04, min(0.08, radius_small))
            x = rng.uniform(self.FEED_X_MIN + radius_small, self.FEED_X_MAX - radius_small)
            y = rng.uniform(self.FEED_Y_MIN + radius_small, self.FEED_Y_MAX - radius_small)
            mass = density * (math.pi * radius_small ** 2)
            body = self._world.CreateDynamicBody(
                position=(x, y),
                fixtures=Box2D.b2FixtureDef(
                    shape=circleShape(radius=radius_small),
                    density=mass / (math.pi * radius_small ** 2),
                    friction=friction,
                    restitution=particle_restitution,
                ),
            )
            body.linearDamping = self._default_linear_damping
            body.angularDamping = self._default_angular_damping
            self._particles_small.append(body)
        for _ in range(n_medium):
            radius_medium = r_m + rng.uniform(-radius_jitter, radius_jitter)
            radius_medium = max(0.07, min(0.12, radius_medium))
            x = rng.uniform(self.FEED_X_MIN + radius_medium, self.FEED_X_MAX - radius_medium)
            y = rng.uniform(self.FEED_Y_MIN + radius_medium, self.FEED_Y_MAX - radius_medium)
            mass = density * (math.pi * radius_medium ** 2)
            body = self._world.CreateDynamicBody(
                position=(x, y),
                fixtures=Box2D.b2FixtureDef(
                    shape=circleShape(radius=radius_medium),
                    density=mass / (math.pi * radius_medium ** 2),
                    friction=friction,
                    restitution=particle_restitution,
                ),
            )
            body.linearDamping = self._default_linear_damping
            body.angularDamping = self._default_angular_damping
            self._particles_medium.append(body)
        for _ in range(n_large):
            radius_large = r_l + rng.uniform(-radius_jitter, radius_jitter)
            radius_large = max(0.11, min(0.16, radius_large))
            x = rng.uniform(self.FEED_X_MIN + radius_large, self.FEED_X_MAX - radius_large)
            y = rng.uniform(self.FEED_Y_MIN + radius_large, self.FEED_Y_MAX - radius_large)
            mass = density * (math.pi * radius_large ** 2)
            body = self._world.CreateDynamicBody(
                position=(x, y),
                fixtures=Box2D.b2FixtureDef(
                    shape=circleShape(radius=radius_large),
                    density=mass / (math.pi * radius_large ** 2),
                    friction=friction,
                    restitution=particle_restitution,
                ),
            )
            body.linearDamping = self._default_linear_damping
            body.angularDamping = self._default_angular_damping
            self._particles_large.append(body)
        rng = random.Random(43)
        for _ in range(n_small):
            rs = r_s + rng.uniform(-radius_jitter, radius_jitter)
            rs = max(0.04, min(0.08, rs))
            x = rng.uniform(self.FEED_X_MIN + rs, self.FEED_X_MAX - rs)
            y = rng.uniform(self.FEED_Y_MIN + rs, self.FEED_Y_MAX - rs)
            self._pending_second_wave.append(("small", x, y, rs, density, friction, particle_restitution))
        for _ in range(n_medium):
            rm = r_m + rng.uniform(-radius_jitter, radius_jitter)
            rm = max(0.07, min(0.12, rm))
            x = rng.uniform(self.FEED_X_MIN + rm, self.FEED_X_MAX - rm)
            y = rng.uniform(self.FEED_Y_MIN + rm, self.FEED_Y_MAX - rm)
            self._pending_second_wave.append(("medium", x, y, rm, density, friction, particle_restitution))
        for _ in range(n_large):
            rl = r_l + rng.uniform(-radius_jitter, radius_jitter)
            rl = max(0.11, min(0.16, rl))
            x = rng.uniform(self.FEED_X_MIN + rl, self.FEED_X_MAX - rl)
            y = rng.uniform(self.FEED_Y_MIN + rl, self.FEED_Y_MAX - rl)
            self._pending_second_wave.append(("large", x, y, rl, density, friction, particle_restitution))
        if n_third_small + n_third_medium + n_third_large > 0:
            rng = random.Random(44)
            for _ in range(n_third_small):
                rs = r_s + rng.uniform(-radius_jitter, radius_jitter)
                rs = max(0.04, min(0.08, rs))
                x = rng.uniform(self.FEED_X_MIN + rs, self.FEED_X_MAX - rs)
                y = rng.uniform(self.FEED_Y_MIN + rs, self.FEED_Y_MAX - rs)
                self._pending_third_wave.append(("small", x, y, rs, density, friction, particle_restitution))
            for _ in range(n_third_medium):
                rm = r_m + rng.uniform(-radius_jitter, radius_jitter)
                rm = max(0.07, min(0.12, rm))
                x = rng.uniform(self.FEED_X_MIN + rm, self.FEED_X_MAX - rm)
                y = rng.uniform(self.FEED_Y_MIN + rm, self.FEED_Y_MAX - rm)
                self._pending_third_wave.append(("medium", x, y, rm, density, friction, particle_restitution))
            for _ in range(n_third_large):
                rl = r_l + rng.uniform(-radius_jitter, radius_jitter)
                rl = max(0.11, min(0.16, rl))
                x = rng.uniform(self.FEED_X_MIN + rl, self.FEED_X_MAX - rl)
                y = rng.uniform(self.FEED_Y_MIN + rl, self.FEED_Y_MAX - rl)
                self._pending_third_wave.append(("large", x, y, rl, density, friction, particle_restitution))
        self._initial_small_count = n_small * 2 + n_third_small
        self._initial_medium_count = n_medium * 2 + n_third_medium
        self._initial_large_count = n_large * 2 + n_third_large
    MIN_BEAM_SIZE = 0.08
    MAX_BEAM_SIZE = 1.0
    def add_beam(self, x, y, width, height, angle=0, density=200.0):
        width = max(self.MIN_BEAM_SIZE, min(width, self.MAX_BEAM_SIZE))
        height = max(self.MIN_BEAM_SIZE, min(height, self.MAX_BEAM_SIZE))
        body = self._world.CreateDynamicBody(
            position=(x, y),
            angle=angle,
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(width / 2, height / 2)),
                density=density,
                friction=self._beam_fixture_friction,
                restitution=0.1,
            ),
        )
        body.linearDamping = self._default_linear_damping
        body.angularDamping = self._default_angular_damping
        self._bodies.append(body)
        return body
    def add_static_beam(self, x, y, width, height, angle=0, density=200.0):
        width = max(self.MIN_BEAM_SIZE, min(width, self.MAX_BEAM_SIZE))
        height = max(self.MIN_BEAM_SIZE, min(height, self.MAX_BEAM_SIZE))
        body = self._world.CreateStaticBody(
            position=(x, y),
            angle=angle,
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(width / 2, height / 2)),
                friction=self._beam_fixture_friction,
                restitution=0.1,
            ),
        )
        body.design_mass = width * height * density
        self._bodies.append(body)
        return body
    def add_joint(self, body_a, body_b, anchor_point, type='rigid'):
        if body_a is None:
            raise ValueError("add_joint: body_a cannot be None.")
        anchor_x, anchor_y = anchor_point[0], anchor_point[1]
        if body_b is None:
            body_b = self._terrain_bodies.get("floor")
            if body_b is None:
                raise ValueError("add_joint: floor not found.")
        if type != 'rigid':
            type = 'rigid'
        joint = self._world.CreateWeldJoint(
            bodyA=body_a,
            bodyB=body_b,
            anchor=(anchor_x, anchor_y),
            collideConnected=False
        )
        self._joints.append(joint)
        return joint
    def get_structure_mass(self):
        return sum(b.mass for b in self._bodies) + sum(getattr(b, 'design_mass', 0) for b in self._bodies)
    def set_material_properties(self, body, restitution=0.1):
        for fixture in body.fixtures:
            fixture.restitution = float(restitution)
    def apply_force(self, body, force):
        if body and hasattr(body, 'ApplyForceToCenter'):
            body.ApplyForceToCenter(force, wake=True)
    def step(self, time_step):
        phase = 2 * math.pi * self._step_count / max(1, self.WIND_PERIOD_STEPS)
        fx_wind = self.WIND_AMPLITUDE * math.sin(phase)
        if self._step_count % self.GUST_PERIOD_STEPS == 0 and self._step_count > 0:
            fx_wind += self.GUST_AMPLITUDE * (1.0 if (self._step_count // self.GUST_PERIOD_STEPS) % 2 == 0 else -1.0)
        for p in self._particles_small + self._particles_medium + self._particles_large:
            if p.active:
                p.ApplyForceToCenter((fx_wind, 0), wake=True)
        if self._gravity_osc_amplitude > 0:
            grav_osc_phase = 2 * math.pi * self._step_count / max(1.0, self._gravity_osc_period)
            fy_grav_osc = self._gravity_osc_amplitude * math.sin(grav_osc_phase)
            for p in self._particles_small + self._particles_medium + self._particles_large:
                if p.active:
                    p.ApplyForceToCenter((0, p.mass * fy_grav_osc), wake=True)
        for key in ("sweeper1", "sweeper2"):
            sweeper = self._terrain_bodies.get(key)
            if sweeper is not None and hasattr(sweeper, "userData") and sweeper.userData:
                ud = sweeper.userData
                x_min, x_max = ud.get("x_min", 5.0), ud.get("x_max", 9.0)
                v = ud.get("v_sweep", 0.08)
                x = sweeper.position.x
                if x >= x_max:
                    ud["v_sweep"] = -abs(v) if v > 0 else -abs(v)
                elif x <= x_min:
                    ud["v_sweep"] = abs(v)
                sweeper.linearVelocity = (ud["v_sweep"], 0)
        if self._step_count == self.SECOND_WAVE_STEP and self._pending_second_wave:
            wave_size = len(self._pending_second_wave)
            for item in self._pending_second_wave:
                kind, x, y, rad, density, friction, rest = item[0], item[1], item[2], item[3], item[4], item[5], item[6]
                mass = density * (math.pi * rad ** 2)
                body = self._world.CreateDynamicBody(
                    position=(x, y),
                    fixtures=Box2D.b2FixtureDef(
                        shape=circleShape(radius=rad),
                        density=mass / (math.pi * rad ** 2),
                        friction=friction,
                        restitution=rest,
                    ),
                )
                body.linearDamping = self._default_linear_damping
                body.angularDamping = self._default_angular_damping
                if kind == "small":
                    self._particles_small.append(body)
                elif kind == "medium":
                    self._particles_medium.append(body)
                else:
                    self._particles_large.append(body)
            self._pending_second_wave.clear()
            self._wave_events.append({
                "wave": 2,
                "step": self._step_count,
                "spawned_count": wave_size,
                "active_count": self.get_spawned_particle_count(),
            })
        if self._step_count == self.THIRD_WAVE_STEP and self._pending_third_wave:
            wave_size = len(self._pending_third_wave)
            for item in self._pending_third_wave:
                kind, x, y, rad, density, friction, rest = item[0], item[1], item[2], item[3], item[4], item[5], item[6]
                mass = density * (math.pi * rad ** 2)
                body = self._world.CreateDynamicBody(
                    position=(x, y),
                    fixtures=Box2D.b2FixtureDef(
                        shape=circleShape(radius=rad),
                        density=mass / (math.pi * rad ** 2),
                        friction=friction,
                        restitution=rest,
                    ),
                )
                body.linearDamping = self._default_linear_damping
                body.angularDamping = self._default_angular_damping
                if kind == "small":
                    self._particles_small.append(body)
                elif kind == "medium":
                    self._particles_medium.append(body)
                else:
                    self._particles_large.append(body)
            self._pending_third_wave.clear()
            self._wave_events.append({
                "wave": 3,
                "step": self._step_count,
                "spawned_count": wave_size,
                "active_count": self.get_spawned_particle_count(),
            })
        self._step_count += 1
        self._world.Step(time_step, 10, 10)
    def get_particles_small(self):
        return list(self._particles_small)
    def get_particles_medium(self):
        return list(self._particles_medium)
    def get_terrain_bounds(self):
        return {
            "feed": {"x_min": self.FEED_X_MIN, "x_max": self.FEED_X_MAX,
                     "y_min": self.FEED_Y_MIN, "y_max": self.FEED_Y_MAX},
            "small_zone_y_max": self.SMALL_ZONE_Y_MAX,
            "medium_zone": {"y_min": self.MEDIUM_ZONE_Y_MIN, "y_max": self.MEDIUM_ZONE_Y_MAX},
            "large_zone_y_min": self.LARGE_ZONE_Y_MIN,
            "build_zone": {"x": [self.BUILD_ZONE_X_MIN, self.BUILD_ZONE_X_MAX],
                          "y": [self.BUILD_ZONE_Y_MIN, self.BUILD_ZONE_Y_MAX]},
        }
    def get_initial_small_count(self):
        return self._initial_small_count
    def get_initial_medium_count(self):
        return self._initial_medium_count
    def get_initial_large_count(self):
        return self._initial_large_count
    def get_initial_particle_count(self):
        return self._initial_small_count + self._initial_medium_count + self._initial_large_count
    def get_spawned_particle_count(self):
        parts = self._particles_small + self._particles_medium + self._particles_large
        return sum(1 for p in parts if p is not None and p.active)
    def get_wave_events(self):
        return [dict(event) for event in self._wave_events]
    def get_small_in_small_zone_count(self):
        return sum(1 for p in self._particles_small if p.active and p.position.y < self.SMALL_ZONE_Y_MAX)
    def get_medium_in_medium_zone_count(self):
        return sum(1 for p in self._particles_medium if p.active
                   and self.MEDIUM_ZONE_Y_MIN <= p.position.y < self.MEDIUM_ZONE_Y_MAX)
    def get_large_in_large_zone_count(self):
        return sum(1 for p in self._particles_large if p.active and p.position.y >= self.LARGE_ZONE_Y_MIN)
    def get_large_in_small_zone_count(self):
        return sum(1 for p in self._particles_large if p.active and p.position.y < self.SMALL_ZONE_Y_MAX)
    def get_small_in_large_zone_count(self):
        return sum(1 for p in self._particles_small if p.active and p.position.y >= self.LARGE_ZONE_Y_MIN)
    def get_medium_in_small_zone_count(self):
        return sum(1 for p in self._particles_medium if p.active and p.position.y < self.MEDIUM_ZONE_Y_MIN)
    def get_medium_in_large_zone_count(self):
        return sum(1 for p in self._particles_medium if p.active and p.position.y >= self.MEDIUM_ZONE_Y_MAX)
    def get_small_above_sieve_count(self):
        return sum(1 for p in self._particles_small if p.active and p.position.y >= self.SMALL_ZONE_Y_MAX)
    def get_small_in_sieve_band_count(self):
        return sum(1 for p in self._particles_small if p.active
                   and self.SMALL_ZONE_Y_MAX <= p.position.y < self.LARGE_ZONE_Y_MIN)
    def get_large_below_sieve_count(self):
        return sum(1 for p in self._particles_large if p.active and p.position.y < self.LARGE_ZONE_Y_MIN)
    def get_large_in_sieve_band_count(self):
        return sum(1 for p in self._particles_large if p.active
                   and self.SMALL_ZONE_Y_MAX <= p.position.y < self.LARGE_ZONE_Y_MIN)
    def get_classification_purity(self):
        total = self.get_spawned_particle_count()
        if total == 0:
            return 1.0
        correct = (self.get_small_in_small_zone_count() +
                   self.get_medium_in_medium_zone_count() +
                   self.get_large_in_large_zone_count())
        return correct / total
    def has_contamination(self):
        y_lim = self.FEED_Y_MIN
        def below_feed(p):
            return p.active and p.position.y < y_lim
        if any(below_feed(p) and p.position.y < self.SMALL_ZONE_Y_MAX for p in self._particles_large):
            return True
        if any(below_feed(p) and p.position.y >= self.LARGE_ZONE_Y_MIN for p in self._particles_small):
            return True
        if any(
            below_feed(p)
            and (p.position.y < self.MEDIUM_ZONE_Y_MIN or p.position.y >= self.MEDIUM_ZONE_Y_MAX)
            for p in self._particles_medium
        ):
            return True
        return False
    def get_particle_y_stats(self):
        import statistics as _st
        result = {}
        for label, lst in [("small", self._particles_small),
                           ("medium", self._particles_medium),
                           ("large", self._particles_large)]:
            ys = [p.position.y for p in lst if p is not None and p.active]
            n = len(ys)
            if n == 0:
                result[label] = {"min": None, "max": None, "median": None, "mean": None, "count": 0}
            else:
                ys_sorted = sorted(ys)
                result[label] = {
                    "min": float(ys_sorted[0]),
                    "max": float(ys_sorted[-1]),
                    "median": float(_st.median(ys_sorted)),
                    "mean": float(sum(ys_sorted) / n),
                    "count": n,
                }
        return result
    def get_particle_velocity_stats(self):
        import statistics as _st
        result = {}
        for label, lst in [("small", self._particles_small),
                           ("medium", self._particles_medium),
                           ("large", self._particles_large)]:
            speeds = [math.hypot(p.linearVelocity.x, p.linearVelocity.y)
                      for p in lst if p is not None and p.active]
            n = len(speeds)
            if n == 0:
                result[label] = {"min": None, "max": None, "median": None, "mean": None, "count": 0}
            else:
                sp = sorted(speeds)
                result[label] = {
                    "min": float(sp[0]),
                    "max": float(sp[-1]),
                    "median": float(_st.median(sp)),
                    "mean": float(sum(sp) / n),
                    "count": n,
                }
        return result
    def get_particle_mass_stats(self):
        import statistics as _st
        result = {}
        for label, lst in [("small", self._particles_small),
                           ("medium", self._particles_medium),
                           ("large", self._particles_large)]:
            ms = [p.mass for p in lst if p is not None and p.active]
            n = len(ms)
            if n == 0:
                result[label] = {"min": None, "max": None, "mean": None, "median": None, "count": 0}
            else:
                sm = sorted(ms)
                result[label] = {
                    "min": float(sm[0]),
                    "max": float(sm[-1]),
                    "mean": float(sum(sm) / n),
                    "median": float(_st.median(sm)),
                    "count": n,
                }
        return result
    def get_total_kinetic_energy(self):
        total = 0.0
        for lst in [self._particles_small, self._particles_medium, self._particles_large]:
            for p in lst:
                if p is None or not p.active:
                    continue
                vx = p.linearVelocity.x
                vy = p.linearVelocity.y
                total += 0.5 * p.mass * (vx * vx + vy * vy)
        return float(total)
    def get_physics_summary(self):
        base = self._physics_config
        return {
            "gravity_base": tuple(base.get("gravity", (0, -10))),
            "gravity_osc_amplitude": float(base.get("gravity_oscillation_amplitude", 0.0)),
            "gravity_osc_period": float(base.get("gravity_oscillation_period", 1.0)),
            "linear_damping": float(base.get("linear_damping", 0.02)),
            "angular_damping": float(base.get("angular_damping", 0.02)),
            "wind_amplitude": self.WIND_AMPLITUDE,
            "wind_period_steps": self.WIND_PERIOD_STEPS,
            "gust_amplitude": self.GUST_AMPLITUDE,
            "gust_period_steps": self.GUST_PERIOD_STEPS,
            "beam_friction": self._beam_fixture_friction,
            "step_count": self._step_count,
        }
    def get_beam_build_zone_margins(self):
        eps = 1e-4
        x_min, x_max = self.BUILD_ZONE_X_MIN - eps, self.BUILD_ZONE_X_MAX + eps
        y_min, y_max = self.BUILD_ZONE_Y_MIN - eps, self.BUILD_ZONE_Y_MAX + eps
        records = []
        for i, body in enumerate(self._bodies):
            cx, cy = float(body.position.x), float(body.position.y)
            worst_x_margin = None
            worst_y_margin = None
            worst_vx, worst_vy = None, None
            saw_vertex = False
            for fixture in body.fixtures:
                shape = fixture.shape
                if not hasattr(shape, "vertices"):
                    continue
                for v in shape.vertices:
                    saw_vertex = True
                    wx, wy = body.GetWorldPoint(v)
                    mx = min(wx - x_min, x_max - wx)
                    my = min(wy - y_min, y_max - wy)
                    if worst_x_margin is None or mx < worst_x_margin:
                        worst_x_margin = mx
                        worst_vx = float(wx)
                    if worst_y_margin is None or my < worst_y_margin:
                        worst_y_margin = my
                        worst_vy = float(wy)
            if not saw_vertex:
                mx = min(cx - x_min, x_max - cx)
                my = min(cy - y_min, y_max - cy)
                worst_x_margin, worst_y_margin = mx, my
                worst_vx, worst_vy = cx, cy
            records.append({
                "beam_index": i,
                "centroid": (round(cx, 3), round(cy, 3)),
                "vertex_x_margin": round(float(worst_x_margin or 0.0), 4),
                "vertex_y_margin": round(float(worst_y_margin or 0.0), 4),
                "vertex_witness": (round(float(worst_vx or cx), 3), round(float(worst_vy or cy), 3)),
            })
        return records
    def get_numerical_health(self):
        extreme_v = []
        nan_detected = False
        inf_detected = False
        for label, lst in [("small", self._particles_small),
                           ("medium", self._particles_medium),
                           ("large", self._particles_large)]:
            for p in lst:
                if p is None or not p.active:
                    continue
                vx, vy = p.linearVelocity.x, p.linearVelocity.y
                spd = math.hypot(vx, vy)
                if spd > 100.0:
                    extreme_v.append({"class": label, "speed": round(spd, 1),
                                       "vx": round(vx, 1), "vy": round(vy, 1)})
                state_values = (vx, vy, p.position.x, p.position.y)
                if any(math.isnan(value) for value in state_values):
                    nan_detected = True
                if any(math.isinf(value) for value in state_values):
                    inf_detected = True
        return {
            "extreme_velocity_events": extreme_v,
            "extreme_velocity_count": len(extreme_v),
            "nan_detected": nan_detected,
            "inf_detected": inf_detected,
        }
    def get_env_force_estimates(self):
        gx, gy = self._gravity_base
        gy_osc = 0.0
        if self._gravity_osc_amplitude > 0:
            phase = 2.0 * math.pi * self._step_count / max(1.0, self._gravity_osc_period)
            gy_osc = self._gravity_osc_amplitude * math.sin(phase)
        wind_phase = 2.0 * math.pi * self._step_count / max(1, self.WIND_PERIOD_STEPS)
        fx_wind = self.WIND_AMPLITUDE * math.sin(wind_phase)
        if self._step_count % self.GUST_PERIOD_STEPS == 0 and self._step_count > 0:
            gust_sign = 1.0 if (self._step_count // self.GUST_PERIOD_STEPS) % 2 == 0 else -1.0
            fx_wind += self.GUST_AMPLITUDE * gust_sign
        result = {}
        for label, lst in [("small", self._particles_small),
                           ("medium", self._particles_medium),
                           ("large", self._particles_large)]:
            ms = [p.mass for p in lst if p is not None and p.active]
            n = len(ms)
            if n == 0:
                result[label] = {"avg_mass": 0.0, "grav_x": 0.0, "grav_y": 0.0,
                                  "osc_y": 0.0, "wind_x": 0.0, "fx_total": 0.0,
                                  "fy_gravity": 0.0, "count": 0}
                continue
            avg_m = sum(ms) / n
            result[label] = {
                "avg_mass": round(avg_m, 3),
                "grav_x": round(avg_m * gx, 2),
                "grav_y": round(avg_m * gy, 2),
                "osc_y": round(avg_m * gy_osc, 2),
                "wind_x": round(fx_wind, 2),
                "fx_total": round(avg_m * gx + fx_wind, 2),
                "fy_gravity": round(avg_m * (gy + gy_osc), 2),
                "count": n,
            }
        return result
