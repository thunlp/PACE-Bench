import Box2D

from Box2D.b2 import world, polygonShape, circleShape

import math

import random

class Sandbox:
    def __init__(self, *, terrain_config=None, physics_config=None):
        terrain_config = terrain_config or {}
        physics_config = physics_config or {}
        self._terrain_config = dict(terrain_config)
        self._physics_config = dict(physics_config)
        gravity = tuple(physics_config.get("gravity", (0, -10)))
        self._default_linear_damping = float(physics_config.get("linear_damping", 0.05))
        self._default_angular_damping = float(physics_config.get("angular_damping", 0.05))
        self._world = world(gravity=gravity, doSleep=True)
        self._bodies = []
        self._joints = []
        self._terrain_bodies = {}
        self._fluid_particles = []
        self._step_counter = 0
        self.world = self._world
        self.bodies = self._bodies
        self.joints = self._joints
        self.SOURCE_X_MIN = 2.0
        self.SOURCE_X_MAX = 6.0
        self.SOURCE_Y_MIN = 0.0
        self.SOURCE_Y_MAX = 1.5
        self.TARGET_X_MIN = float(terrain_config.get("target_x_min", 18.0))
        self.TARGET_X_MAX = float(terrain_config.get("target_x_max", 22.0))
        self.TARGET_Y_MIN = float(terrain_config.get("target_y_min", 0.0))
        self.TARGET_Y_MAX = float(terrain_config.get("target_y_max", 1.5))
        self.BUILD_ZONE_X_MIN = 6.0
        self.BUILD_ZONE_X_MAX = 18.0
        self.BUILD_ZONE_Y_MIN = 0.0
        self.BUILD_ZONE_Y_MAX = 6.0
        self.PIT_X_MIN = 13.5
        self.PIT_X_MAX = 15.5
        self.PIT_Y_MIN = 0.0
        self.PIT_Y_MAX = float(terrain_config.get("pit1_y_max", 2.0))
        self.PIT2_X_MIN = 16.0
        self.PIT2_X_MAX = 17.5
        self.PIT2_Y_MIN = 0.0
        self.PIT2_Y_MAX = float(terrain_config.get("pit2_y_max", 1.6))
        self.PIT3_X_MIN = float(terrain_config.get("pit3_x_min", 11.0))
        self.PIT3_X_MAX = float(terrain_config.get("pit3_x_max", 12.5))
        self.PIT3_Y_MIN = 0.0
        self.PIT3_Y_MAX = float(terrain_config.get("pit3_y_max", 1.6))
        self.HEADWIND_Y_THRESHOLD = 3.0
        self.HEADWIND_FX_BASE = float(terrain_config.get("headwind_fx_base", -120.0))
        self.GRAVWELL_X_MIN = 10.0
        self.GRAVWELL_X_MAX = 14.0
        self.GRAVWELL_Y_MIN = 1.5
        self.GRAVWELL_Y_MAX = 3.5
        self.GRAVWELL_FY = float(terrain_config.get("gravwell_fy", -120.0))
        self.MAX_TIME_SECONDS = float(physics_config.get("max_time_seconds", 40.0))
        self.FORCE_BUDGET_PER_STEP = float(physics_config.get("force_budget", 12000.0))
        self._force_budget_used = 0.0
        self._peak_budget_used = 0.0
        self._last_step_budget_used = 0.0
        self._hazard_loss_counts = {
            "pit1": 0, "pit2": 0, "pit3": 0,
            "headwind": 0, "gravwell": 0,
            "out_of_bounds": 0, "floor": 0,
        }
        self._hazard_loss_positions = {
            "pit1": [], "pit2": [], "pit3": [],
            "headwind": [], "gravwell": [],
        }
        self._budget_used_history = []
        self._transport_timeline = {
            "first_source_exit_step": None,
            "first_build_zone_entry_step": None,
            "first_target_entry_step": None,
            "peak_particles_in_target": 0,
            "peak_particles_in_target_step": None,
        }
        self._first_hazard_loss_steps = {
            "pit1": None,
            "pit2": None,
            "pit3": None,
            "out_of_bounds": None,
            "floor": None,
        }
        self._first_hazard_exposure_steps = {
            "headwind": None,
            "gravwell": None,
        }
        if "max_steps" in physics_config:
            self.MAX_STEPS = int(physics_config["max_steps"])
        else:
            self.MAX_STEPS = int(self.MAX_TIME_SECONDS * 60)
        self.MAX_STRUCTURE_MASS = float(terrain_config.get("max_structure_mass", 380.0))
        self.MIN_DELIVERY_RATIO = float(terrain_config.get("min_delivery_ratio", 0.90))
        self._create_terrain(terrain_config)
        self._create_fluid_particles(terrain_config)
    def _create_terrain(self, terrain_config: dict):
        floor_length = 26.0
        floor_height = 0.3
        floor = self._world.CreateStaticBody(
            position=(floor_length / 2, -floor_height / 2),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(floor_length / 2, floor_height / 2)),
                friction=0.5,
            ),
        )
        self._terrain_bodies["floor"] = floor
        source_width = 4.0
        source_height = 1.5
        source_center_x = 4.0
        source_center_y = 0.75
        source_body = self._world.CreateStaticBody(
            position=(source_center_x, source_center_y),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(source_width / 2, source_height / 2)),
                friction=0.3,
                isSensor=True,
            ),
        )
        self._terrain_bodies["source"] = source_body
        target_center_x = (self.TARGET_X_MIN + self.TARGET_X_MAX) / 2
        target_center_y = (self.TARGET_Y_MIN + self.TARGET_Y_MAX) / 2
        target_hw = (self.TARGET_X_MAX - self.TARGET_X_MIN) / 2
        target_hh = (self.TARGET_Y_MAX - self.TARGET_Y_MIN) / 2
        target_body = self._world.CreateStaticBody(
            position=(target_center_x, target_center_y),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(target_hw, target_hh)),
                friction=0.3,
                isSensor=True,
            ),
        )
        self._terrain_bodies["target"] = target_body
    def _create_fluid_particles(self, terrain_config: dict):
        fluid_config = terrain_config.get("fluid", {})
        num_particles = int(fluid_config.get("count", 60))
        particle_radius = float(fluid_config.get("radius", 0.10))
        density = float(fluid_config.get("density", 800.0))
        viscosity = float(fluid_config.get("viscosity", 0.25))
        seed = int(fluid_config.get("seed", 42))
        rng = random.Random(seed)
        for _ in range(num_particles):
            x = rng.uniform(
                self.SOURCE_X_MIN + particle_radius,
                self.SOURCE_X_MAX - particle_radius,
            )
            y = rng.uniform(
                self.SOURCE_Y_MIN + particle_radius,
                self.SOURCE_Y_MAX - particle_radius,
            )
            mass = density * (math.pi * particle_radius ** 2)
            body = self._world.CreateDynamicBody(
                position=(x, y),
                fixtures=Box2D.b2FixtureDef(
                    shape=circleShape(radius=particle_radius),
                    density=mass / (math.pi * particle_radius ** 2),
                    friction=0.2,
                    restitution=0.05,
                ),
            )
            body.linearDamping = viscosity
            body.angularDamping = viscosity
            self._fluid_particles.append(body)
        self._initial_particle_count = len(self._fluid_particles)
    MIN_BEAM_SIZE = 0.1
    MAX_BEAM_SIZE = 1.2
    def add_beam(self, x, y, width, height, angle=0, density=250.0):
        width = max(self.MIN_BEAM_SIZE, min(width, self.MAX_BEAM_SIZE))
        height = max(self.MIN_BEAM_SIZE, min(height, self.MAX_BEAM_SIZE))
        body = self._world.CreateDynamicBody(
            position=(x, y),
            angle=angle,
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(width / 2, height / 2)),
                density=density,
                friction=0.4,
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
        return sum(b.mass for b in self._bodies)
    def set_material_properties(self, body, restitution=0.1):
        for fixture in body.fixtures:
            fixture.restitution = float(restitution)
    _PARTICLE_RADIUS = 0.10
    def _record_hazard_loss(self, name, x=None, y=None):
        self._hazard_loss_counts[name] += 1
        if name in self._hazard_loss_positions and x is not None and y is not None:
            self._hazard_loss_positions[name].append((x, y))
        if name in self._first_hazard_loss_steps:
            if self._first_hazard_loss_steps[name] is None:
                self._first_hazard_loss_steps[name] = self._step_counter

    def _record_transport_diagnostics(self):
        active_positions = [
            (p.position.x, p.position.y)
            for p in self._fluid_particles
            if p is not None and p.active
        ]
        timeline = self._transport_timeline
        if timeline["first_source_exit_step"] is None and any(
            not (
                self.SOURCE_X_MIN <= x <= self.SOURCE_X_MAX
                and self.SOURCE_Y_MIN <= y <= self.SOURCE_Y_MAX
            )
            for x, y in active_positions
        ):
            timeline["first_source_exit_step"] = self._step_counter
        if timeline["first_build_zone_entry_step"] is None and any(
            self.BUILD_ZONE_X_MIN <= x <= self.BUILD_ZONE_X_MAX
            and self.BUILD_ZONE_Y_MIN <= y <= self.BUILD_ZONE_Y_MAX
            for x, y in active_positions
        ):
            timeline["first_build_zone_entry_step"] = self._step_counter
        in_target = sum(
            1
            for x, y in active_positions
            if self.TARGET_X_MIN <= x <= self.TARGET_X_MAX
            and self.TARGET_Y_MIN <= y <= self.TARGET_Y_MAX
        )
        if timeline["first_target_entry_step"] is None and in_target > 0:
            timeline["first_target_entry_step"] = self._step_counter
        if in_target > timeline["peak_particles_in_target"]:
            timeline["peak_particles_in_target"] = in_target
            timeline["peak_particles_in_target_step"] = self._step_counter

    def step(self, time_step):
        self._step_counter += 1
        self._world.Step(time_step, 20, 20)
        for p in self._fluid_particles:
            if p is not None and p.active:
                x, y = p.position.x, p.position.y
                if y < self._PARTICLE_RADIUS:
                    p.transform = ((x, self._PARTICLE_RADIUS), p.angle)
                    vx = p.linearVelocity.x
                    p.linearVelocity = (vx, 0.0)
                if x < -5.0 or x > 30.0 or y < -10.0:
                    self._record_hazard_loss("out_of_bounds", x, y)
                    p.active = False
        headwind_fx = self.HEADWIND_FX_BASE + 60.0 * math.sin(self._step_counter / 50.0)
        for p in self._fluid_particles:
            if p is None or not p.active:
                continue
            x, y = p.position.x, p.position.y
            if (self.PIT3_X_MIN <= x <= self.PIT3_X_MAX and
                    self.PIT3_Y_MIN <= y <= self.PIT3_Y_MAX):
                self._record_hazard_loss("pit3", x, y)
                p.active = False
                continue
            if (self.PIT_X_MIN <= x <= self.PIT_X_MAX and
                    self.PIT_Y_MIN <= y <= self.PIT_Y_MAX):
                self._record_hazard_loss("pit1", x, y)
                p.active = False
                continue
            if (self.PIT2_X_MIN <= x <= self.PIT2_X_MAX and
                    self.PIT2_Y_MIN <= y <= self.PIT2_Y_MAX):
                self._record_hazard_loss("pit2", x, y)
                p.active = False
                continue
            if y > self.HEADWIND_Y_THRESHOLD:
                p.ApplyForceToCenter((headwind_fx, 0), wake=True)
                self._hazard_loss_counts["headwind"] += 1
                if self._first_hazard_exposure_steps["headwind"] is None:
                    self._first_hazard_exposure_steps["headwind"] = self._step_counter
            if (self.GRAVWELL_X_MIN <= x <= self.GRAVWELL_X_MAX and
                    self.GRAVWELL_Y_MIN <= y <= self.GRAVWELL_Y_MAX):
                p.ApplyForceToCenter((0, self.GRAVWELL_FY), wake=True)
                self._hazard_loss_counts["gravwell"] += 1
                if self._first_hazard_exposure_steps["gravwell"] is None:
                    self._first_hazard_exposure_steps["gravwell"] = self._step_counter
        self._record_transport_diagnostics()
        self._peak_budget_used = max(self._peak_budget_used, self._force_budget_used)
        self._last_step_budget_used = self._force_budget_used
        if self._step_counter % 100 == 0:
            self._budget_used_history.append((self._step_counter, self._force_budget_used))
        self._force_budget_used = 0.0
    def get_fluid_particles(self):
        return [p for p in self._fluid_particles if p is not None and p.active]
    def apply_force_to_particle(self, particle, fx, fy):
        if particle not in self._fluid_particles or not particle.active:
            return
        mag = math.sqrt(fx * fx + fy * fy)
        if mag <= 0:
            return
        if self._force_budget_used + mag <= self.FORCE_BUDGET_PER_STEP:
            particle.ApplyForceToCenter((fx, fy), wake=True)
            self._force_budget_used += mag
    def get_terrain_bounds(self):
        return {
            "source": {"x_min": self.SOURCE_X_MIN, "x_max": self.SOURCE_X_MAX,
                      "y_min": self.SOURCE_Y_MIN, "y_max": self.SOURCE_Y_MAX},
            "target": {"x_min": self.TARGET_X_MIN, "x_max": self.TARGET_X_MAX,
                       "y_min": self.TARGET_Y_MIN, "y_max": self.TARGET_Y_MAX},
            "build_zone": {"x": [self.BUILD_ZONE_X_MIN, self.BUILD_ZONE_X_MAX],
                           "y": [self.BUILD_ZONE_Y_MIN, self.BUILD_ZONE_Y_MAX]},
            "pit": {"x_min": self.PIT_X_MIN, "x_max": self.PIT_X_MAX,
                    "y_min": self.PIT_Y_MIN, "y_max": self.PIT_Y_MAX},
            "pit2": {"x_min": self.PIT2_X_MIN, "x_max": self.PIT2_X_MAX,
                     "y_min": self.PIT2_Y_MIN, "y_max": self.PIT2_Y_MAX},
            "pit3": {"x_min": self.PIT3_X_MIN, "x_max": self.PIT3_X_MAX,
                     "y_min": self.PIT3_Y_MIN, "y_max": self.PIT3_Y_MAX},
        }
    def get_initial_particle_count(self):
        return self._initial_particle_count
    def get_particles_in_target_count(self):
        count = 0
        for p in self._fluid_particles:
            if p is None or not p.active:
                continue
            x, y = p.position.x, p.position.y
            if (self.TARGET_X_MIN <= x <= self.TARGET_X_MAX and
                    self.TARGET_Y_MIN <= y <= self.TARGET_Y_MAX):
                count += 1
        return count
    def get_delivery_ratio(self):
        total = self.get_initial_particle_count()
        if total == 0:
            return 1.0
        return self.get_particles_in_target_count() / total
    def get_particles_in_source_count(self):
        count = 0
        for p in self._fluid_particles:
            if p is None or not p.active:
                continue
            x, y = p.position.x, p.position.y
            if (self.SOURCE_X_MIN <= x <= self.SOURCE_X_MAX and
                    self.SOURCE_Y_MIN <= y <= self.SOURCE_Y_MAX):
                count += 1
        return count
    def get_particles_in_build_zone_count(self):
        count = 0
        for p in self._fluid_particles:
            if p is None or not p.active:
                continue
            x, y = p.position.x, p.position.y
            if (self.BUILD_ZONE_X_MIN <= x <= self.BUILD_ZONE_X_MAX and
                    self.BUILD_ZONE_Y_MIN <= y <= self.BUILD_ZONE_Y_MAX):
                count += 1
        return count
    def get_particle_stats(self):
        xs, ys, n = [], [], 0
        for p in self._fluid_particles:
            if p is None or not p.active:
                continue
            xs.append(p.position.x)
            ys.append(p.position.y)
            n += 1
        mean_x = sum(xs) / n if n else 0.0
        mean_y = sum(ys) / n if n else 0.0
        max_x = max(xs) if xs else 0.0
        min_x = min(xs) if xs else 0.0
        return {
            "mean_x": mean_x, "mean_y": mean_y,
            "max_x": max_x, "min_x": min_x,
            "active_count": n,
        }
    def get_hazard_losses(self):
        return dict(self._hazard_loss_counts)
    def get_hazard_loss_positions(self):
        return {k: list(v) for k, v in self._hazard_loss_positions.items()}
    def get_hazard_zone_bounds(self):
        return {
            "pit1": {
                "x_min": self.PIT_X_MIN, "x_max": self.PIT_X_MAX,
                "y_min": self.PIT_Y_MIN, "y_max": self.PIT_Y_MAX,
            },
            "pit2": {
                "x_min": self.PIT2_X_MIN, "x_max": self.PIT2_X_MAX,
                "y_min": self.PIT2_Y_MIN, "y_max": self.PIT2_Y_MAX,
            },
            "pit3": {
                "x_min": self.PIT3_X_MIN, "x_max": self.PIT3_X_MAX,
                "y_min": self.PIT3_Y_MIN, "y_max": self.PIT3_Y_MAX,
            },
            "headwind": {"y_threshold": self.HEADWIND_Y_THRESHOLD},
            "gravwell": {
                "x_min": self.GRAVWELL_X_MIN, "x_max": self.GRAVWELL_X_MAX,
                "y_min": self.GRAVWELL_Y_MIN, "y_max": self.GRAVWELL_Y_MAX,
            },
        }
    def get_transport_timeline(self):
        result = dict(self._transport_timeline)
        result["first_hazard_loss_steps"] = dict(self._first_hazard_loss_steps)
        result["first_hazard_exposure_steps"] = dict(self._first_hazard_exposure_steps)
        return result
    def get_closest_particle_to_target(self):
        best_dist = float('inf')
        best_pos = (0.0, 0.0)
        tx_min, tx_max = self.TARGET_X_MIN, self.TARGET_X_MAX
        ty_min, ty_max = self.TARGET_Y_MIN, self.TARGET_Y_MAX
        for p in self._fluid_particles:
            if p is None or not p.active:
                continue
            x, y = p.position.x, p.position.y
            dx = max(0.0, tx_min - x, x - tx_max)
            dy = max(0.0, ty_min - y, y - ty_max)
            dist = math.sqrt(dx * dx + dy * dy)
            if dist < best_dist:
                best_dist = dist
                best_pos = (x, y)
        return {"distance": best_dist if math.isfinite(best_dist) else -1.0,
                "position": best_pos}
    def get_force_budget_utilization(self):
        budget = self.FORCE_BUDGET_PER_STEP
        return {
            "last_step_used": self._last_step_budget_used,
            "peak_used": self._peak_budget_used,
            "budget": budget,
            "peak_utilization_pct": (self._peak_budget_used / budget * 100.0) if budget > 0 else 0.0,
            "last_utilization_pct": (self._last_step_budget_used / budget * 100.0) if budget > 0 else 0.0,
        }
    def get_zone_velocity_stats(self):
        zones = {
            "source": {"x_min": self.SOURCE_X_MIN, "x_max": self.SOURCE_X_MAX,
                       "y_min": self.SOURCE_Y_MIN, "y_max": self.SOURCE_Y_MAX},
            "build_pre_pit3": {"x_min": self.BUILD_ZONE_X_MIN, "x_max": self.PIT3_X_MIN,
                               "y_min": self.BUILD_ZONE_Y_MIN, "y_max": self.BUILD_ZONE_Y_MAX},
            "pit3_zone": {"x_min": self.PIT3_X_MIN, "x_max": self.PIT3_X_MAX,
                          "y_min": 0.0, "y_max": 10.0},
            "build_mid": {"x_min": self.PIT3_X_MAX, "x_max": self.PIT_X_MIN,
                          "y_min": self.BUILD_ZONE_Y_MIN, "y_max": self.BUILD_ZONE_Y_MAX},
            "pit1_zone": {"x_min": self.PIT_X_MIN, "x_max": self.PIT_X_MAX,
                          "y_min": 0.0, "y_max": 10.0},
            "pit2_zone": {"x_min": self.PIT2_X_MIN, "x_max": self.PIT2_X_MAX,
                          "y_min": 0.0, "y_max": 10.0},
            "build_post_pit2": {"x_min": self.PIT2_X_MAX, "x_max": self.TARGET_X_MIN,
                                "y_min": self.BUILD_ZONE_Y_MIN, "y_max": self.BUILD_ZONE_Y_MAX},
            "target": {"x_min": self.TARGET_X_MIN, "x_max": self.TARGET_X_MAX,
                       "y_min": self.TARGET_Y_MIN, "y_max": self.TARGET_Y_MAX},
            "headwind": {"x_min": 0.0, "x_max": 30.0,
                         "y_min": self.HEADWIND_Y_THRESHOLD, "y_max": 20.0},
            "gravwell": {"x_min": self.GRAVWELL_X_MIN, "x_max": self.GRAVWELL_X_MAX,
                         "y_min": self.GRAVWELL_Y_MIN, "y_max": self.GRAVWELL_Y_MAX},
        }
        result = {}
        for zone_name, bounds in zones.items():
            vxs, vys, n = [], [], 0
            for p in self._fluid_particles:
                if p is None or not p.active:
                    continue
                x, y = p.position.x, p.position.y
                if (bounds["x_min"] <= x <= bounds["x_max"] and
                        bounds["y_min"] <= y <= bounds["y_max"]):
                    vxs.append(p.linearVelocity.x)
                    vys.append(p.linearVelocity.y)
                    n += 1
            mean_vx = sum(vxs) / n if n else 0.0
            mean_vy = sum(vys) / n if n else 0.0
            result[zone_name] = {"mean_vx": mean_vx, "mean_vy": mean_vy, "count": n}
        return result
