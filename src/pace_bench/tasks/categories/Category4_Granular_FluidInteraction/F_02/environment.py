import Box2D

from Box2D.b2 import (world, polygonShape, circleShape, staticBody, dynamicBody, weldJoint)

import math

class Sandbox:
    def __init__(self, *, terrain_config=None, physics_config=None):
        terrain_config = terrain_config or {}
        physics_config = physics_config or {}
        self._terrain_config = dict(terrain_config)
        self._physics_config = dict(physics_config)
        self._observation_errors = []
        gravity = tuple(physics_config.get("gravity", (0, -10)))
        self._default_linear_damping = float(physics_config.get("linear_damping", 0.0))
        self._default_angular_damping = float(physics_config.get("angular_damping", 0.0))
        self._world = world(gravity=gravity, doSleep=True)
        self._bodies = []
        self._joints = []
        self._terrain_bodies = {}
        self.world = self._world
        self.bodies = self._bodies
        self.joints = self._joints
        self._create_terrain(terrain_config)
        self.WATER_X_LEFT = 10.0
        self.WATER_X_RIGHT = 24.0
        self.WATER_SURFACE_Y = 2.0
        self.WATER_BOTTOM_Y = 0.0
        self.TARGET_X = 26.0
        self.BUILD_ZONE_X_MIN = 2.0
        self.BUILD_ZONE_X_MAX = 8.0
        self.BUILD_ZONE_Y_MIN = 0.0
        self.BUILD_ZONE_Y_MAX = 4.0
        self.MAX_STRUCTURE_MASS = float(terrain_config.get("max_structure_mass", 600.0))
        self.LEFT_BANK_X_MAX = 10.0
        self.RIGHT_BANK_X_MIN = 24.0
        liquid_density = float(terrain_config.get("liquid_density", 1000.0))
        self._buoyancy_factor = min(1.5, (liquid_density / 1000.0) * 0.8)
        self._step_count = 0
        self._current_per_kg = float(terrain_config.get("current_per_kg", 5.5))
        self._water_drag_coef = float(terrain_config.get("water_drag_coef", 115.0))
        self._wind_amplitude = float(terrain_config.get("wind_amplitude", 200.0))
        self._wind_period_steps = int(terrain_config.get("wind_period_steps", 90))
        self._wind_x_left = 12.0
        self._wind_x_right = 22.0
        self._deep_channel_x_left = float(terrain_config.get("deep_channel_x_left", 16.5))
        self._deep_channel_x_right = float(terrain_config.get("deep_channel_x_right", 19.5))
        self._deep_channel_buoyancy_scale = float(terrain_config.get("deep_channel_buoyancy_scale", 0.35))
    def _create_terrain(self, terrain_config: dict):
        floor_height = 0.3
        lb_width = 10.0
        lb_floor = self._world.CreateStaticBody(
            position=(lb_width / 2, -floor_height / 2),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(lb_width / 2, floor_height / 2)),
                friction=0.6,
            ),
        )
        self._terrain_bodies["floor"] = lb_floor
        rb_width = 30.0
        self._world.CreateStaticBody(
            position=(24.0 + rb_width / 2, -floor_height / 2),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(rb_width / 2, floor_height / 2)),
                friction=0.6,
            ),
        )
        self._world.CreateStaticBody(
            position=(17.0, -10.0),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(7.0, 0.5)),
                friction=0.0,
            ),
        )
        water_width = 14.0
        water_center_x = 17.0
        water_height = 2.0
        water_body = self._world.CreateStaticBody(
            position=(water_center_x, water_height / 2),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(water_width / 2, water_height / 2)),
                friction=0.0,
                isSensor=True,
            ),
        )
        self._terrain_bodies["water"] = water_body
        pillar_radius = float(terrain_config.get("pillar_radius", 0.46))
        pillar_positions = [(14.0, 0.88), (17.0, 0.90), (20.0, 0.92)]
        for i, (px, py) in enumerate(pillar_positions):
            pillar = self._world.CreateStaticBody(
                position=(px, py),
                fixtures=Box2D.b2FixtureDef(
                    shape=circleShape(radius=pillar_radius),
                    friction=0.4,
                ),
            )
            self._terrain_bodies[f"pillar_{i}"] = pillar
        self._pillar_positions = pillar_positions
        self._pillar_radius = pillar_radius
        self._headwind_burst_x_left = 15.0
        self._headwind_burst_x_right = 19.0
        self._headwind_burst_per_kg = float(terrain_config.get("headwind_burst_per_kg", 0.8))
        self._thrust_cooldown_steps = int(terrain_config.get("thrust_cooldown_steps", 3))
        self._max_joint_force = float(terrain_config.get("max_joint_force", float('inf')))
        self._last_thrust_step = {}
        self._current_step = 0
        self._emp_zone = terrain_config.get("emp_zone", None)
        self._corrosive_y = float(terrain_config.get("corrosive_y", float('inf')))
        self._whirlpool = terrain_config.get("whirlpool", None)
        self._zone_crossing_events = []
        self._joint_failure_events = []
        self._speed_cap_count = 0
        self._sink_samples = []
        self._prev_front_x = None
        self._prev_lowest_y = None
        self._max_vertical_accel_seen = 0.0
        self._prev_front_vy = None
        self._joint_force_samples = []
    MIN_BEAM_SIZE = 0.15
    MAX_BEAM_SIZE = 2.0
    BUILD_ZONE_X_MIN = 2.0
    BUILD_ZONE_X_MAX = 8.0
    BUILD_ZONE_Y_MIN = 0.0
    BUILD_ZONE_Y_MAX = 4.0
    MAX_STRUCTURE_MASS = 600.0
    def add_beam(self, x, y, width, height, angle=0, density=200.0):
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
    def add_joint(self, body_a, body_b, anchor_point, type='rigid'):
        if body_a is None:
            raise ValueError("add_joint: body_a cannot be None.")
        anchor_x, anchor_y = anchor_point[0], anchor_point[1]
        if body_b is None:
            body_b = self._terrain_bodies.get("floor")
            if body_b is None:
                raise ValueError("add_joint: floor not found for anchor.")
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
        total_mass = 0.0
        for body in self._bodies:
            total_mass += body.mass
        return total_mass
    def set_material_properties(self, body, restitution=0.1):
        for fixture in body.fixtures:
            fixture.restitution = float(restitution)
    _MAX_THRUST_PER_BODY = 520.0
    def apply_force(self, body, force_x, force_y, step_count=None):
        if body is not None and body.active:
            if getattr(self, '_emp_zone', None) is not None:
                if self._emp_zone[0] <= body.position.x <= self._emp_zone[1]:
                    return
            step = step_count if step_count is not None else getattr(self, '_current_step', 0)
            if self._thrust_cooldown_steps > 0:
                bid = id(body)
                last = self._last_thrust_step.get(bid, -999)
                if step - last < self._thrust_cooldown_steps:
                    return
                self._last_thrust_step[bid] = step
            fx, fy = float(force_x), float(force_y)
            mag = math.sqrt(fx * fx + fy * fy)
            if mag > self._MAX_THRUST_PER_BODY:
                scale = self._MAX_THRUST_PER_BODY / mag
                fx, fy = fx * scale, fy * scale
            body.ApplyForceToCenter((fx, fy), wake=True)
    _MAX_LINEAR_SPEED = 4.0
    def step(self, time_step):
        self._step_count += 1
        for body in self._bodies:
            if not body.active:
                continue
            vx, vy = body.linearVelocity.x, body.linearVelocity.y
            speed = math.sqrt(vx * vx + vy * vy)
            if speed > self._MAX_LINEAR_SPEED:
                self._speed_cap_count += 1
                scale = self._MAX_LINEAR_SPEED / speed
                body.linearVelocity = (vx * scale, vy * scale)
        for body in self._bodies:
            if not body.active:
                continue
            x, y = body.position.x, body.position.y
            vx, vy = body.linearVelocity.x, body.linearVelocity.y
            in_water = (self.WATER_X_LEFT <= x <= self.WATER_X_RIGHT and
                        self.WATER_BOTTOM_Y <= y <= self.WATER_SURFACE_Y)
            if in_water:
                submerged = self.WATER_SURFACE_Y - y
                g = abs(self._world.gravity[1]) if len(self._world.gravity) > 1 else 10.0
                bf = self._buoyancy_factor
                if self._deep_channel_x_left <= x <= self._deep_channel_x_right:
                    bf *= self._deep_channel_buoyancy_scale
                buoyancy_up = bf * submerged * body.mass * g
                body.ApplyForceToCenter((0, buoyancy_up), wake=True)
                f_current = -self._current_per_kg * body.mass
                body.ApplyForceToCenter((f_current, 0), wake=True)
                speed = math.sqrt(vx * vx + vy * vy)
                if speed > 0.01:
                    drag_mag = self._water_drag_coef * speed * speed
                    drag_x = -drag_mag * (vx / speed)
                    drag_y = -drag_mag * (vy / speed)
                    body.ApplyForceToCenter((drag_x, drag_y), wake=True)
                if self._wind_x_left <= x <= self._wind_x_right:
                    phase = 2.0 * math.pi * self._step_count / self._wind_period_steps
                    f_wind_y = self._wind_amplitude * math.sin(phase)
                    body.ApplyForceToCenter((0, f_wind_y), wake=True)
                if self._headwind_burst_x_left <= x <= self._headwind_burst_x_right:
                    f_headwind = -self._headwind_burst_per_kg * body.mass
                    body.ApplyForceToCenter((f_headwind, 0), wake=True)
            if y > getattr(self, '_corrosive_y', float('inf')):
                body.ApplyForceToCenter((0, -2000.0 * body.mass), wake=True)
            if getattr(self, '_whirlpool', None) is not None:
                wx = float(self._whirlpool.get("x", 17.0))
                ww = float(self._whirlpool.get("width", 2.0))
                wf = float(self._whirlpool.get("force", 100.0))
                if wx - ww/2.0 <= x <= wx + ww/2.0:
                    body.ApplyForceToCenter((0, -wf * body.mass), wake=True)
        if self._max_joint_force < float('inf'):
            for j in list(self._joints):
                if not j.active:
                    continue
                force = j.GetReactionForce(1.0/time_step).length
                if force > self._max_joint_force:
                    try:
                        ba = j.bodyA
                        bb = j.bodyB
                        ax, ay = j.anchorA.x, j.anchorA.y
                        ba_idx = self._bodies.index(ba) if ba in self._bodies else -1
                        bb_idx = self._bodies.index(bb) if bb in self._bodies else -1
                        self._joint_failure_events.append({
                            "step": self._step_count,
                            "body_a_idx": ba_idx,
                            "body_b_idx": bb_idx,
                            "anchor": (float(ax), float(ay)),
                            "reaction_force": float(force),
                            "force_limit": float(self._max_joint_force),
                        })
                    except (AttributeError, TypeError, ValueError) as exc:
                        self._observation_errors.append(
                            f"joint failure metadata unavailable at step {self._step_count}: {exc}"
                        )
                        self._joint_failure_events.append({
                            "step": self._step_count,
                            "reaction_force": float(force),
                            "force_limit": float(self._max_joint_force),
                        })
                    self._world.DestroyJoint(j)
                    self._joints.remove(j)
        self._world.Step(time_step, 10, 10)
        for body in self._bodies:
            if not body.active:
                continue
            vx, vy = body.linearVelocity.x, body.linearVelocity.y
            speed = math.sqrt(vx * vx + vy * vy)
            if speed > self._MAX_LINEAR_SPEED:
                self._speed_cap_count += 1
                scale = self._MAX_LINEAR_SPEED / speed
                body.linearVelocity = (vx * scale, vy * scale)
        fx = self.get_vehicle_front_x()
        ly = self.get_vehicle_lowest_y()
        if self._prev_front_x is not None and fx is not None and ly is not None:
            st = self._step_count
            _cross = lambda old, new, b: old < b <= new
            zone_boundaries = [
                ("water_entry", 10.0),
                ("deep_channel_entry", self._deep_channel_x_left),
                ("deep_channel_exit", self._deep_channel_x_right),
                ("wind_zone_entry", self._wind_x_left),
                ("headwind_entry", self._headwind_burst_x_left),
                ("headwind_exit", self._headwind_burst_x_right),
                ("target_x", self.TARGET_X),
            ]
            for zname, zbound in zone_boundaries:
                if _cross(self._prev_front_x, fx, zbound):
                    self._zone_crossing_events.append({
                        "step": st, "zone": zname,
                        "front_x": round(float(fx), 3),
                        "lowest_y": round(float(ly), 3),
                    })
            if self._emp_zone is not None:
                for zname, zbound in [("emp_entry", self._emp_zone[0]), ("emp_exit", self._emp_zone[1])]:
                    if _cross(self._prev_front_x, fx, zbound):
                        self._zone_crossing_events.append({
                            "step": st, "zone": zname,
                            "front_x": round(float(fx), 3),
                            "lowest_y": round(float(ly), 3),
                        })
            if self._whirlpool is not None:
                wx = float(self._whirlpool.get("x", 17.0))
                ww = float(self._whirlpool.get("width", 2.0))
                for zname, zbound in [("whirlpool_entry", wx - ww/2.0), ("whirlpool_exit", wx + ww/2.0)]:
                    if _cross(self._prev_front_x, fx, zbound):
                        self._zone_crossing_events.append({
                            "step": st, "zone": zname,
                            "front_x": round(float(fx), 3),
                            "lowest_y": round(float(ly), 3),
                        })
            if self._prev_lowest_y is not None:
                if self._prev_lowest_y >= -0.5 and ly < -0.5:
                    self._zone_crossing_events.append({
                        "step": st, "zone": "sink_threshold_crossed",
                        "front_x": round(float(fx), 3),
                        "lowest_y": round(float(ly), 3),
                    })
                cy = getattr(self, '_corrosive_y', float('inf'))
                if cy < float('inf'):
                    if self._prev_lowest_y <= cy and ly > cy:
                        self._zone_crossing_events.append({
                            "step": st, "zone": "corrosive_ceiling_entry",
                            "front_x": round(float(fx), 3),
                            "lowest_y": round(float(ly), 3),
                        })
        self._prev_front_x = fx
        self._prev_lowest_y = ly
        if self._step_count % 100 == 0:
            if ly is not None:
                self._sink_samples.append({
                    "step": self._step_count,
                    "lowest_y": round(float(ly), 3),
                    "front_x": round(float(fx), 3) if fx is not None else None,
                })
        if self._step_count % 50 == 0 and self._joints:
            for ji, j in enumerate(self._joints):
                if not j.active:
                    continue
                try:
                    force = j.GetReactionForce(1.0/time_step).length
                    ba = j.bodyA
                    bb = j.bodyB
                    ax, ay = j.anchorA.x, j.anchorA.y
                    ba_idx = self._bodies.index(ba) if ba in self._bodies else -1
                    bb_idx = self._bodies.index(bb) if bb in self._bodies else -1
                    self._joint_force_samples.append({
                        "step": self._step_count,
                        "joint_idx": ji,
                        "body_a_idx": ba_idx,
                        "body_b_idx": bb_idx,
                        "anchor": (float(ax), float(ay)),
                        "reaction_force": float(force),
                        "force_limit": float(self._max_joint_force),
                    })
                except (AttributeError, TypeError, ValueError) as exc:
                    self._observation_errors.append(
                        f"joint force sample unavailable at step {self._step_count}: {exc}"
                    )
        if self._prev_front_vy is not None and fx is not None:
            vely = self._prev_front_vy
            front_body = None
            max_x = -1e12
            for b in self._bodies:
                if b.active and b.position.x > max_x:
                    max_x = b.position.x
                    front_body = b
            if front_body is not None:
                cur_vy = front_body.linearVelocity.y
                vertical_accel = abs(cur_vy - vely) / time_step if time_step > 0 else 0.0
                if vertical_accel > self._max_vertical_accel_seen:
                    self._max_vertical_accel_seen = float(vertical_accel)
                self._prev_front_vy = cur_vy
        elif fx is not None:
            front_body = None
            max_x = -1e12
            for b in self._bodies:
                if b.active and b.position.x > max_x:
                    max_x = b.position.x
                    front_body = b
            if front_body is not None:
                self._prev_front_vy = front_body.linearVelocity.y
    def get_terrain_bounds(self):
        return {
            "left_bank": {"x_max": self.LEFT_BANK_X_MAX},
            "water": {"x_left": self.WATER_X_LEFT, "x_right": self.WATER_X_RIGHT,
                      "surface_y": self.WATER_SURFACE_Y, "bottom_y": self.WATER_BOTTOM_Y},
            "right_bank": {"x_min": self.RIGHT_BANK_X_MIN},
            "target_x": self.TARGET_X,
            "build_zone": {
                "x": [self.BUILD_ZONE_X_MIN, self.BUILD_ZONE_X_MAX],
                "y": [self.BUILD_ZONE_Y_MIN, self.BUILD_ZONE_Y_MAX],
            },
        }
    def get_vehicle_front_x(self):
        active = [body for body in self._bodies if body.active]
        if not active:
            return None
        return max(self._body_axis_extent(body, axis=0, upper=True) for body in active)
    def get_vehicle_lowest_y(self):
        active = [body for body in self._bodies if body.active]
        if not active:
            return None
        return min(self._body_axis_extent(body, axis=1, upper=False) for body in active)
    @staticmethod
    def _body_axis_extent(body, *, axis, upper):
        coordinates = []
        for fixture in body.fixtures:
            vertices = getattr(fixture.shape, "vertices", None)
            if vertices:
                coordinates.extend(float(body.GetWorldPoint(vertex)[axis]) for vertex in vertices)
        if coordinates:
            return max(coordinates) if upper else min(coordinates)
        return float(body.position[axis])
    def get_vehicle_velocity(self):
        if not self._bodies:
            return None
        front = max(self._bodies, key=lambda b: b.position.x if b.active else -1e9)
        if not front.active:
            return None
        return (front.linearVelocity.x, front.linearVelocity.y)
    def get_zone_crossing_events(self):
        return list(self._zone_crossing_events)
    def get_joint_failure_events(self):
        return list(self._joint_failure_events)
    def get_sink_trajectory(self):
        return list(self._sink_samples)
    def get_speed_cap_count(self):
        return int(self._speed_cap_count)
    def get_joint_force_samples(self):
        return list(self._joint_force_samples)
    def get_max_vertical_accel(self):
        return float(self._max_vertical_accel_seen)
    def get_observation_errors(self):
        return list(self._observation_errors)
    def get_body_observations(self):
        observations = []
        for index, body in enumerate(self._bodies):
            if not body.active:
                continue
            x = float(body.position.x)
            y = float(body.position.y)
            vx = float(body.linearVelocity.x)
            vy = float(body.linearVelocity.y)
            observations.append({
                "body_idx": index,
                "x": round(x, 3),
                "y": round(y, 3),
                "vx": round(vx, 3),
                "vy": round(vy, 3),
                "speed": round(math.hypot(vx, vy), 3),
                "in_water": self.WATER_X_LEFT <= x <= self.WATER_X_RIGHT
                and self.WATER_BOTTOM_Y <= y <= self.WATER_SURFACE_Y,
            })
        return observations
    def get_env_parameters(self):
        wp = self._whirlpool
        emp = self._emp_zone
        return {
            "buoyancy_factor": float(self._buoyancy_factor),
            "current_per_kg": float(self._current_per_kg),
            "water_drag_coef": float(self._water_drag_coef),
            "wind_amplitude": float(self._wind_amplitude),
            "wind_zone": [float(self._wind_x_left), float(self._wind_x_right)],
            "deep_channel_zone": [float(self._deep_channel_x_left), float(self._deep_channel_x_right)],
            "deep_channel_buoyancy_scale": float(self._deep_channel_buoyancy_scale),
            "headwind_zone": [float(self._headwind_burst_x_left), float(self._headwind_burst_x_right)],
            "headwind_per_kg": float(self._headwind_burst_per_kg),
            "emp_zone": [float(emp[0]), float(emp[1])] if emp is not None else None,
            "corrosive_y": float(getattr(self, '_corrosive_y', float('inf'))),
            "whirlpool": {
                "x": float(wp.get("x", 17.0)),
                "width": float(wp.get("width", 2.0)),
                "force": float(wp.get("force", 100.0)),
            } if wp is not None else None,
            "max_joint_force": float(self._max_joint_force),
            "thrust_cooldown_steps": int(self._thrust_cooldown_steps),
            "max_linear_speed": float(self._MAX_LINEAR_SPEED),
            "water_surface_y": float(self.WATER_SURFACE_Y),
            "sink_y_threshold": -0.5,
            "target_x": float(self.TARGET_X),
            "build_zone_x": [float(self.BUILD_ZONE_X_MIN), float(self.BUILD_ZONE_X_MAX)],
            "build_zone_y": [float(self.BUILD_ZONE_Y_MIN), float(self.BUILD_ZONE_Y_MAX)],
            "max_structure_mass": float(self.MAX_STRUCTURE_MASS),
        }
    def compute_force_decomposition(self):
        decomp = []
        g = abs(self._world.gravity[1]) if len(self._world.gravity) > 1 else 10.0
        for i, body in enumerate(self._bodies):
            if not body.active:
                continue
            x = float(body.position.x)
            y = float(body.position.y)
            vx = float(body.linearVelocity.x)
            vy = float(body.linearVelocity.y)
            spd = math.sqrt(vx * vx + vy * vy)
            mass = float(body.mass)
            forces = {
                "gravity": (0.0, -mass * g),
                "buoyancy": (0.0, 0.0),
                "current": (0.0, 0.0),
                "drag": (0.0, 0.0),
                "wind": (0.0, 0.0),
                "headwind": (0.0, 0.0),
                "corrosive": (0.0, 0.0),
                "whirlpool": (0.0, 0.0),
            }
            in_water = (self.WATER_X_LEFT <= x <= self.WATER_X_RIGHT and
                        self.WATER_BOTTOM_Y <= y <= self.WATER_SURFACE_Y)
            if in_water:
                submerged = self.WATER_SURFACE_Y - y
                bf = self._buoyancy_factor
                in_dc = self._deep_channel_x_left <= x <= self._deep_channel_x_right
                if in_dc:
                    bf *= self._deep_channel_buoyancy_scale
                forces["buoyancy"] = (0.0, bf * submerged * mass * g)
                forces["current"] = (-self._current_per_kg * mass, 0.0)
                if spd > 0.01:
                    drag_mag = self._water_drag_coef * spd * spd
                    forces["drag"] = (-drag_mag * vx / spd, -drag_mag * vy / spd)
                if self._wind_x_left <= x <= self._wind_x_right:
                    phase = 2.0 * math.pi * self._step_count / self._wind_period_steps
                    forces["wind"] = (0.0, self._wind_amplitude * math.sin(phase))
                if self._headwind_burst_x_left <= x <= self._headwind_burst_x_right:
                    forces["headwind"] = (-self._headwind_burst_per_kg * mass, 0.0)
            cy = getattr(self, '_corrosive_y', float('inf'))
            if y > cy:
                forces["corrosive"] = (0.0, -2000.0 * mass)
            wp = getattr(self, '_whirlpool', None)
            if wp is not None:
                wx = float(wp.get("x", 17.0))
                ww = float(wp.get("width", 2.0))
                wf = float(wp.get("force", 100.0))
                if wx - ww / 2.0 <= x <= wx + ww / 2.0:
                    forces["whirlpool"] = (0.0, -wf * mass)
            net_x = sum(f[0] for f in forces.values())
            net_y = sum(f[1] for f in forces.values())
            in_emp = False
            if self._emp_zone is not None:
                in_emp = self._emp_zone[0] <= x <= self._emp_zone[1]
            decomp.append({
                "body_idx": i,
                "x": round(x, 3),
                "y": round(y, 3),
                "mass": round(mass, 3),
                "speed": round(spd, 3),
                "in_water": in_water,
                "in_deep_channel": (self._deep_channel_x_left <= x <= self._deep_channel_x_right),
                "in_emp": in_emp,
                "forces": {k: (round(v[0], 2), round(v[1], 2)) for k, v in forces.items()},
                "net_force": (round(net_x, 2), round(net_y, 2)),
            })
        return decomp
