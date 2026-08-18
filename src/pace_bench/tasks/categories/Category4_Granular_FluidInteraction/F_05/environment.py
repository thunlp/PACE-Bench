import Box2D

from Box2D.b2 import (world, polygonShape, circleShape, staticBody, dynamicBody, weldJoint)

import math

import random

from typing import Optional, Dict

WELD_TORQUE_FORCE_RATIO = 0.4

class Sandbox:
    def __init__(self, *, terrain_config=None, physics_config=None):
        terrain_config = terrain_config or {}
        physics_config = physics_config or {}
        self._terrain_config = dict(terrain_config)
        self._physics_config = dict(physics_config)
        gravity = tuple(physics_config.get("gravity", (0, -10)))
        self._default_linear_damping = float(physics_config.get("linear_damping", 0.1))
        self._default_angular_damping = float(physics_config.get("angular_damping", 0.05))
        self._world = world(gravity=gravity, doSleep=True)
        self._bodies = []
        self._joints = []
        self._terrain_bodies = {}
        self._cargo = []
        self.world = self._world
        self.bodies = self._bodies
        self.joints = self._joints
        self.WATER_X_MIN = 5.0
        self.WATER_X_MAX = 25.0
        self.WATER_SURFACE_Y = 2.0
        self.CARGO_WATER_Y = float(terrain_config.get("cargo_water_y", 1.90))
        self.BOAT_MAX_ANGLE_RAD = math.radians(float(terrain_config.get("max_capsize_angle_deg", 18.0)))
        self.BUILD_ZONE_X_MIN = float(terrain_config.get("build_zone_x_min", 12.0))
        self.BUILD_ZONE_X_MAX = float(terrain_config.get("build_zone_x_max", 18.0))
        self.BUILD_ZONE_Y_MIN = float(terrain_config.get("build_zone_y_min", 2.0))
        self.BUILD_ZONE_Y_MAX = float(terrain_config.get("build_zone_y_max", 4.5))
        self.MAX_STRUCTURE_MASS = float(terrain_config.get("max_structure_mass", 60.0))
        wave_amplitude = float(terrain_config.get("wave_amplitude", 10.0))
        wave_freq = float(terrain_config.get("wave_frequency", 0.5))
        self._wave_amplitude = wave_amplitude
        self._wave_omega = 2.0 * math.pi * wave_freq
        self._wave2_amplitude = float(terrain_config.get("wave2_amplitude", 5.0))
        self._wave2_omega = 2.0 * math.pi * float(terrain_config.get("wave2_frequency", 0.27))
        self._gust_amplitude = float(terrain_config.get("gust_amplitude", 4.0))
        self._gust_interval = int(terrain_config.get("gust_interval_steps", 80))
        self._wind_amplitude = float(terrain_config.get("wind_amplitude", 5.0))
        self._wind_omega = 2.0 * math.pi * float(terrain_config.get("wind_frequency", 0.15))
        self._sim_time = 0.0
        self._restoring_coeff = float(terrain_config.get("restoring_coeff", 1600.0))
        self._current_strength = float(terrain_config.get("current_strength", 0.35))
        self._rogue_amplitude = float(terrain_config.get("rogue_amplitude", 14.0))
        self._rogue_interval = int(terrain_config.get("rogue_interval_steps", 380))
        self._rogue_double_step = int(terrain_config.get("rogue_double_step", 5))
        self._lateral_impulse_amplitude = float(terrain_config.get("lateral_impulse_amplitude", 68.0))
        self._lateral_impulse_interval = int(terrain_config.get("lateral_impulse_interval_steps", 200))
        self._hull_roll_impulse_amplitude = float(terrain_config.get("hull_roll_impulse_amplitude", 0.0))
        self._hull_roll_impulse_interval = max(1, int(terrain_config.get("hull_roll_impulse_interval_steps", 90)))
        self._create_terrain(terrain_config)
        self.DECK_FRICTION = float(terrain_config.get("deck_friction", 0.5))
        self.JOINT_MAX_FORCE = float(terrain_config.get("joint_max_force", float('inf')))
        self._create_boat(terrain_config)
        self._create_cargo(terrain_config)
        self._peak_abs_boat_angle_rad = 0.0
        self._cargo_ever_below_loss_plane = False
        self._cargo_ever_below_indices = set()
        self._cargo_loss_first_step: Optional[int] = None
        self._physics_steps_done = 0
        self._cargo_loss_grace_steps = int(terrain_config.get("cargo_loss_grace_steps", 180))
        self._capsize_first_step: Optional[int] = None
        self._first_joint_break_step: Optional[int] = None
        self._joint_peak_force: float = 0.0
        self._joint_peak_torque: float = 0.0
        self._peak_angular_velocity_rad_s: float = 0.0
        self._lowest_beam_y_from_floor_margin: Optional[float] = None
        self._capsize_margin_at_grace_end: Optional[float] = None
        self._cargo_retention_milestones: Dict[int, int] = {}
        self._cargo_lowest_y: Optional[float] = None
        self._cargo_lowest_y_step: Optional[int] = None
    def _create_terrain(self, terrain_config: dict):
        floor_length = 30.0
        floor_height = 0.3
        floor = self._world.CreateStaticBody(
            position=(floor_length / 2, -floor_height / 2),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(floor_length / 2, floor_height / 2)),
                friction=0.4,
            ),
        )
        self._terrain_bodies["floor"] = floor
        water_width = 20.0
        water_center_x = 15.0
        water_height = 3.0
        water = self._world.CreateStaticBody(
            position=(water_center_x, water_height / 2),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(water_width / 2, water_height / 2)),
                friction=0.0,
                isSensor=True,
            ),
        )
        self._terrain_bodies["water"] = water
        rock_config = terrain_config.get("rocks", [])
        if not rock_config:
            rock_config = [
                {"x": 13.5, "y": 1.0, "r": 0.24}, {"x": 14.5, "y": 1.1, "r": 0.22},
                {"x": 15.5, "y": 1.05, "r": 0.23}, {"x": 16.5, "y": 1.08, "r": 0.22}
            ]
        self._rocks = []
        for r in rock_config:
            rx = float(r.get("x", 15.0))
            ry = float(r.get("y", 1.0))
            rr = float(r.get("radius", r.get("r", 0.2)))
            rock = self._world.CreateStaticBody(
                position=(rx, ry),
                fixtures=Box2D.b2FixtureDef(
                    shape=circleShape(radius=rr),
                    friction=0.6,
                    restitution=0.2,
                ),
            )
            self._rocks.append(rock)
    def _create_boat(self, terrain_config: dict):
        boat_width = 3.0
        boat_height = 0.4
        boat_x = 15.0
        boat_y = 2.5 + float(terrain_config.get("boat_y_offset", 0.0))
        hull = self._world.CreateDynamicBody(
            position=(boat_x, boat_y),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(boat_width / 2, boat_height / 2)),
                density=80.0,
                friction=self.DECK_FRICTION,
            ),
        )
        hull.linearDamping = self._default_linear_damping
        hull.angularDamping = self._default_angular_damping
        self._terrain_bodies["boat"] = hull
    def _create_cargo(self, terrain_config: dict):
        cargo_config = terrain_config.get("cargo", {})
        n_cargo = int(cargo_config.get("count", 10))
        radius = float(cargo_config.get("radius", 0.15))
        density = float(cargo_config.get("density", 260.0))
        friction = float(cargo_config.get("friction", 0.28))
        cargo_rest = cargo_config.get("restitution", terrain_config.get("cargo_restitution", None))
        restitution = float(cargo_rest if cargo_rest is not None else 0.12)
        seed = int(cargo_config.get("seed", terrain_config.get("target_rng_seed", 42)))
        rng = random.Random(seed)
        boat = self._terrain_bodies["boat"]
        bx, by = boat.position.x, boat.position.y
        boat_half_w = 1.5
        boat_top_y = by + 0.2
        for i in range(n_cargo):
            ox = rng.uniform(-boat_half_w + radius, boat_half_w - radius)
            oy = rng.uniform(0.0, 0.55)
            x = bx + ox
            y = boat_top_y + oy + radius
            body = self._world.CreateDynamicBody(
                position=(x, y),
                fixtures=Box2D.b2FixtureDef(
                    shape=circleShape(radius=radius),
                    density=density,
                    friction=friction,
                    restitution=restitution,
                ),
            )
            ld = cargo_config.get("linear_damping", terrain_config.get("cargo_linear_damping", None))
            body.linearDamping = float(ld) if ld is not None else self._default_linear_damping
            ad = cargo_config.get("angular_damping", terrain_config.get("cargo_angular_damping", None))
            body.angularDamping = float(ad) if ad is not None else self._default_angular_damping
            self._cargo.append(body)
        self._initial_cargo_count = len(self._cargo)
    MIN_BEAM_SIZE = 0.1
    MAX_BEAM_SIZE = 1.0
    @staticmethod
    def _beam_rect_corners_world(center_x, center_y, half_w, half_h, angle_rad):
        ca, sa = math.cos(angle_rad), math.sin(angle_rad)
        corners = []
        for lx, ly in ((half_w, half_h), (-half_w, half_h), (-half_w, -half_h), (half_w, -half_h)):
            wx = center_x + lx * ca - ly * sa
            wy = center_y + lx * sa + ly * ca
            corners.append((wx, wy))
        return corners
    @staticmethod
    def _weld_reaction_force_torque(joint, inv_dt: float) -> tuple[float, float]:
        force = 0.0
        torque = 0.0
        try:
            rf = joint.GetReactionForce(inv_dt)
            if hasattr(rf, "length"):
                force = float(rf.length)
            elif hasattr(rf, "x") and hasattr(rf, "y"):
                force = math.hypot(float(rf.x), float(rf.y))
            else:
                force = math.hypot(float(rf[0]), float(rf[1]))
        except (AttributeError, TypeError, ValueError, IndexError):
            force = 0.0
        try:
            torque = abs(float(joint.GetReactionTorque(inv_dt)))
        except (AttributeError, TypeError, ValueError):
            torque = 0.0
        return force, torque
    def _beam_footprint_outside_build_zone(self, center_x, center_y, width, height, angle_rad):
        hw, hh = width / 2.0, height / 2.0
        for wx, wy in self._beam_rect_corners_world(center_x, center_y, hw, hh, angle_rad):
            if not (
                self.BUILD_ZONE_X_MIN <= wx <= self.BUILD_ZONE_X_MAX
                and self.BUILD_ZONE_Y_MIN <= wy <= self.BUILD_ZONE_Y_MAX
            ):
                return True, wx, wy
        return False, None, None
    def add_beam(self, x, y, width, height, angle=0, density=150.0):
        if not (
            self.BUILD_ZONE_X_MIN <= x <= self.BUILD_ZONE_X_MAX
            and self.BUILD_ZONE_Y_MIN <= y <= self.BUILD_ZONE_Y_MAX
        ):
            raise ValueError(
                f"Beam center ({x:.2f}, {y:.2f}) outside build zone "
                f"x=[{self.BUILD_ZONE_X_MIN:.2f}, {self.BUILD_ZONE_X_MAX:.2f}], "
                f"y=[{self.BUILD_ZONE_Y_MIN:.2f}, {self.BUILD_ZONE_Y_MAX:.2f}]"
            )
        width = max(self.MIN_BEAM_SIZE, min(width, self.MAX_BEAM_SIZE))
        height = max(self.MIN_BEAM_SIZE, min(height, self.MAX_BEAM_SIZE))
        bad, vx, vy = self._beam_footprint_outside_build_zone(x, y, width, height, angle)
        if bad:
            raise ValueError(
                f"Beam footprint corner ({vx:.2f}, {vy:.2f}) outside build zone "
                f"x=[{self.BUILD_ZONE_X_MIN:.2f}, {self.BUILD_ZONE_X_MAX:.2f}], "
                f"y=[{self.BUILD_ZONE_Y_MIN:.2f}, {self.BUILD_ZONE_Y_MAX:.2f}]"
            )
        body = self._world.CreateDynamicBody(
            position=(x, y),
            angle=angle,
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(width / 2, height / 2)),
                density=density,
                friction=self.DECK_FRICTION,
            ),
        )
        body.linearDamping = self._default_linear_damping
        body.angularDamping = self._default_angular_damping
        self._bodies.append(body)
        hw = width / 2.0
        hh = height / 2.0
        corners_y = []
        for lx, ly in ((hw, hh), (-hw, hh), (-hw, -hh), (hw, -hh)):
            ca, sa = math.cos(angle), math.sin(angle)
            corners_y.append(y + lx * sa + ly * ca)
        lowest_corner_y = min(corners_y)
        margin = lowest_corner_y - self.BUILD_ZONE_Y_MIN
        if self._lowest_beam_y_from_floor_margin is None:
            self._lowest_beam_y_from_floor_margin = margin
        else:
            self._lowest_beam_y_from_floor_margin = min(
                self._lowest_beam_y_from_floor_margin, margin
            )
        return body
    def add_joint(self, body_a, body_b, anchor_point, type='rigid'):
        if body_a is None:
            raise ValueError("add_joint: body_a cannot be None.")
        anchor_x, anchor_y = anchor_point[0], anchor_point[1]
        if not (
            self.BUILD_ZONE_X_MIN <= anchor_x <= self.BUILD_ZONE_X_MAX
            and self.BUILD_ZONE_Y_MIN <= anchor_y <= self.BUILD_ZONE_Y_MAX
        ):
            raise ValueError(
                f"Joint anchor ({anchor_x:.2f}, {anchor_y:.2f}) outside build zone "
                f"x=[{self.BUILD_ZONE_X_MIN:.2f}, {self.BUILD_ZONE_X_MAX:.2f}], "
                f"y=[{self.BUILD_ZONE_Y_MIN:.2f}, {self.BUILD_ZONE_Y_MAX:.2f}]"
            )
        if body_b is None:
            boat = self._terrain_bodies.get("boat")
            if not boat:
                raise ValueError("add_joint: boat not found for hull attachment.")
            body_b = boat
        if type != "rigid":
            raise ValueError("add_joint: only type='rigid' (weld) is supported for F-05.")
        joint = self._world.CreateWeldJoint(
            bodyA=body_a,
            bodyB=body_b,
            anchor=(anchor_x, anchor_y),
            collideConnected=False
        )
        setattr(joint, "_f05_declared_anchor_world", (float(anchor_x), float(anchor_y)))
        self._joints.append(joint)
        return joint
    def get_structure_mass(self):
        return sum(b.mass for b in self._bodies)
    def set_material_properties(self, body, restitution=0.1):
        for fixture in body.fixtures:
            fixture.restitution = float(restitution)
    def step(self, time_step):
        boat = self._terrain_bodies.get("boat")
        if boat and boat.active:
            x, y = boat.position.x, boat.position.y
            if self.WATER_X_MIN <= x <= self.WATER_X_MAX and y <= self.WATER_SURFACE_Y + 1.0:
                g = abs(self._world.gravity[1]) if len(self._world.gravity) > 1 else 10.0
                effective_mass = boat.mass + sum(b.mass for b in self._bodies) + sum(c.mass for c in self._cargo if c.active)
                ref_y = self.WATER_SURFACE_Y + 0.5
                buoyancy = 1.5 * effective_mass * g * (ref_y - y)
                buoyancy = max(0.0, buoyancy)
                boat.ApplyForceToCenter((0, buoyancy), wake=True)
                wave_fy = self._wave_amplitude * math.sin(self._wave_omega * self._sim_time)
                wave_fy += self._wave2_amplitude * math.sin(self._wave2_omega * self._sim_time + 0.7)
                step_int = int(self._sim_time / time_step + 0.5)
                if step_int > 0 and step_int % self._gust_interval == 0:
                    wave_fy += self._gust_amplitude * (1.0 if (step_int // self._gust_interval) % 2 == 0 else -1.0)
                boat.ApplyForceToCenter((0, wave_fy), wake=True)
                ri = self._rogue_interval
                rd = self._rogue_double_step
                if step_int > 0 and step_int % ri == 0:
                    boat.ApplyForceToCenter((0, self._rogue_amplitude), wake=True)
                if step_int > rd and (step_int - rd) % ri == 0:
                    boat.ApplyForceToCenter((0, self._rogue_amplitude * 0.6), wake=True)
                if step_int > 0 and step_int % self._lateral_impulse_interval == 0:
                    sign = 1.0 if (step_int // self._lateral_impulse_interval) % 2 == 0 else -1.0
                    boat.ApplyForceToCenter((sign * self._lateral_impulse_amplitude, 0), wake=True)
                wind_fx = self._wind_amplitude * math.sin(self._wind_omega * self._sim_time)
                boat.ApplyForceToCenter((wind_fx, 0), wake=True)
                current_fx = self._current_strength * (x - 15.0)
                boat.ApplyForceToCenter((current_fx, 0), wake=True)
                boat.ApplyTorque(-self._restoring_coeff * boat.angle, wake=True)
                if self._hull_roll_impulse_amplitude > 0.0 and step_int > 0:
                    if step_int % self._hull_roll_impulse_interval == 0:
                        sign = 1.0 if (step_int // self._hull_roll_impulse_interval) % 2 == 0 else -1.0
                        boat.ApplyAngularImpulse(sign * self._hull_roll_impulse_amplitude, wake=True)
        for c in self._cargo:
            if c.active:
                cx, cy = c.position.x, c.position.y
                if self.WATER_X_MIN <= cx <= self.WATER_X_MAX and cy < self.WATER_SURFACE_Y:
                    g = abs(self._world.gravity[1]) if len(self._world.gravity) > 1 else 10.0
                    buoyancy = 0.5 * c.mass * g
                    c.ApplyForceToCenter((0, buoyancy), wake=True)
        self._sim_time += time_step
        self._world.Step(time_step, 10, 10)
        if self.JOINT_MAX_FORCE < float('inf'):
            broken_joints = []
            inv_dt = 1.0 / time_step
            for j in list(self._joints):
                force, torque = self._weld_reaction_force_torque(j, inv_dt)
                if force > self._joint_peak_force:
                    self._joint_peak_force = float(force)
                if torque > self._joint_peak_torque:
                    self._joint_peak_torque = float(torque)
                if force > self.JOINT_MAX_FORCE or torque > self.JOINT_MAX_FORCE * WELD_TORQUE_FORCE_RATIO:
                    broken_joints.append(j)
            for j in broken_joints:
                if j in self._joints:
                    if self._first_joint_break_step is None:
                        self._first_joint_break_step = self._physics_steps_done + 1
                    self._world.DestroyJoint(j)
                    self._joints.remove(j)
        self._physics_steps_done += 1
        if self._physics_steps_done > self._cargo_loss_grace_steps:
            boat_after = self._terrain_bodies.get("boat")
            if boat_after and boat_after.active:
                abs_angle = abs(float(boat_after.angle))
                abs_ang_vel = abs(float(boat_after.angularVelocity))
                self._peak_abs_boat_angle_rad = max(
                    self._peak_abs_boat_angle_rad, abs_angle
                )
                if self._capsize_first_step is None and abs_angle > self.BOAT_MAX_ANGLE_RAD:
                    self._capsize_first_step = self._physics_steps_done
                if abs_ang_vel > self._peak_angular_velocity_rad_s:
                    self._peak_angular_velocity_rad_s = abs_ang_vel
            for cargo_index, c in enumerate(self._cargo):
                if c.active:
                    cy = float(c.position.y)
                    if cy < self.CARGO_WATER_Y:
                        self._cargo_ever_below_loss_plane = True
                        self._cargo_ever_below_indices.add(cargo_index)
                        if self._cargo_loss_first_step is None:
                            self._cargo_loss_first_step = self._physics_steps_done
                    if self._cargo_lowest_y is None or cy < self._cargo_lowest_y:
                        self._cargo_lowest_y = cy
                        self._cargo_lowest_y_step = self._physics_steps_done
            milestone_steps = (2000, 4000, 6000, 8000, 10000)
            if self._physics_steps_done in milestone_steps:
                retained = max(
                    0,
                    self._initial_cargo_count
                    - len(self._cargo_ever_below_indices),
                )
                self._cargo_retention_milestones[self._physics_steps_done] = retained
            if self._physics_steps_done == self._cargo_loss_grace_steps + 1:
                if boat_after and boat_after.active:
                    self._capsize_margin_at_grace_end = (
                        self.BOAT_MAX_ANGLE_RAD - abs(float(boat_after.angle))
                    )
    def get_terrain_bounds(self):
        return {
            "water": {"x_min": self.WATER_X_MIN, "x_max": self.WATER_X_MAX, "surface_y": self.WATER_SURFACE_Y},
            "cargo_water_y": self.CARGO_WATER_Y,
            "boat_max_angle_rad": self.BOAT_MAX_ANGLE_RAD,
            "build_zone": {"x": [self.BUILD_ZONE_X_MIN, self.BUILD_ZONE_X_MAX],
                           "y": [self.BUILD_ZONE_Y_MIN, self.BUILD_ZONE_Y_MAX]},
        }
    def get_boat_body(self):
        return self._terrain_bodies.get("boat")
    def get_boat_position(self):
        boat = self.get_boat_body()
        if boat is None or not boat.active:
            return None
        return (boat.position.x, boat.position.y)
    def get_boat_angle(self):
        boat = self.get_boat_body()
        if boat is None or not boat.active:
            return None
        return boat.angle
    def get_initial_cargo_count(self):
        return self._initial_cargo_count
    def get_cargo_below_loss_plane_count(self):
        if self._physics_steps_done <= self._cargo_loss_grace_steps:
            return 0
        return sum(1 for c in self._cargo if c.active and c.position.y < self.CARGO_WATER_Y)
    def get_cargo_in_water_count(self):
        return self.get_cargo_below_loss_plane_count()
    def get_peak_abs_boat_angle_rad(self):
        return float(self._peak_abs_boat_angle_rad)
    def get_cargo_ever_below_loss_plane(self) -> bool:
        return bool(self._cargo_ever_below_loss_plane)
    def get_cargo_ever_below_loss_plane_count(self) -> int:
        return len(self._cargo_ever_below_indices)
    def get_cargo_loss_first_step(self) -> Optional[int]:
        return self._cargo_loss_first_step
    def get_capsize_first_step(self) -> Optional[int]:
        return self._capsize_first_step
    def get_joint_peak_force(self) -> float:
        return float(self._joint_peak_force)
    def get_joint_peak_torque(self) -> float:
        return float(self._joint_peak_torque)
    def get_joint_max_torque(self) -> float:
        return float(self.JOINT_MAX_FORCE * WELD_TORQUE_FORCE_RATIO)
    def get_first_joint_break_step(self) -> Optional[int]:
        return self._first_joint_break_step
    def get_peak_angular_velocity_rad_s(self) -> float:
        return float(self._peak_angular_velocity_rad_s)
    def get_lowest_beam_y_floor_margin(self) -> Optional[float]:
        return (
            float(self._lowest_beam_y_from_floor_margin)
            if self._lowest_beam_y_from_floor_margin is not None
            else None
        )
    def get_cargo_retention_milestones(self) -> Dict[int, int]:
        return dict(self._cargo_retention_milestones)
    def get_capsize_margin_at_grace_end(self) -> Optional[float]:
        return (
            float(self._capsize_margin_at_grace_end)
            if self._capsize_margin_at_grace_end is not None
            else None
        )
    def get_joint_max_force(self) -> float:
        return float(self.JOINT_MAX_FORCE)
    def get_grace_steps(self) -> int:
        return int(self._cargo_loss_grace_steps)
    def get_cargo_lowest_y(self) -> Optional[float]:
        return float(self._cargo_lowest_y) if self._cargo_lowest_y is not None else None
    def get_cargo_lowest_y_step(self) -> Optional[int]:
        return self._cargo_lowest_y_step
    def get_cargo_state(self):
        result = []
        for i, c in enumerate(self._cargo):
            if c.active:
                result.append({
                    "index": i,
                    "x": float(c.position.x),
                    "y": float(c.position.y),
                    "vx": float(c.linearVelocity.x),
                    "vy": float(c.linearVelocity.y),
                })
        return result
