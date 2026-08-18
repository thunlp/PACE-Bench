import math

import random

import Box2D

from Box2D.b2 import (
    world,
    polygonShape,
    circleShape,
    staticBody,
    dynamicBody,
    revoluteJoint,
    weldJoint,
    distanceJointDef,

)

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
        self.world = self._world
        self.bodies = self._bodies
        self.joints = self._joints
        self._create_terrain(terrain_config)
        self._create_swing(terrain_config)
        self._define_target(terrain_config)
        self.BUILD_ZONE_X_MIN = float(terrain_config.get("build_zone_x_min", 6.0))
        self.BUILD_ZONE_X_MAX = float(terrain_config.get("build_zone_x_max", 14.0))
        self.BUILD_ZONE_Y_MIN = float(terrain_config.get("build_zone_y_min", 4.0))
        self.BUILD_ZONE_Y_MAX = float(terrain_config.get("build_zone_y_max", 10.0))
        self.MAX_STRUCTURE_MASS = float(terrain_config.get("max_structure_mass", 100.0))
        self.MAX_PUMP_FORCE = float(terrain_config.get("max_pump_force", 42.0))
        self.MAX_STEPS = 15000
        self._wind_strength = float(terrain_config.get("wind_strength", 12.0))
        self._wind_period = float(terrain_config.get("wind_period", 2.8))
        self._gust_prob = float(terrain_config.get("gust_prob", 0.04))
        self._gust_force = float(terrain_config.get("gust_force", 20.0))
        self._sim_time = 0.0
        self._wind_enabled = terrain_config.get("wind_enabled", True)
        self._force_calls = 0
        self._force_applied_count = 0
        self._force_suppressed_count = 0
        self._force_suppressed_deadzone = 0
        self._force_suppressed_fault = 0
        self._force_clamped_count = 0
        self._total_fx_requested = 0.0
        self._total_fy_requested = 0.0
        self._total_fx_delivered = 0.0
        self._total_fy_delivered = 0.0
        self._max_speed_seen = 0.0
        self._extreme_velocity_detected = False
        self._phase_alignment_samples = 0
        self._phase_aligned_count = 0
        self._phase_antialigned_count = 0
        self._wind_force_history = []
        self._last_wind_fx = 0.0
        self._rng = random.Random(0)
        self._impulse_clamped_count = 0
    def _create_terrain(self, terrain_config: dict):
        ground_len = 30.0
        ground_h = 0.5
        ground = self._world.CreateStaticBody(
            position=(ground_len / 2, ground_h / 2),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(ground_len / 2, ground_h / 2)),
                friction=0.6,
            ),
        )
        self._terrain_bodies["ground"] = ground
        self._ground_y = ground_h
        pivot_x = float(terrain_config.get("pivot_x", 10.0))
        pivot_y = float(terrain_config.get("pivot_y", 10.0))
        pivot = self._world.CreateStaticBody(position=(pivot_x, pivot_y))
        pivot.CreateFixture(shape=circleShape(radius=0.2), density=0, friction=0)
        self._terrain_bodies["pivot"] = pivot
        self._pivot_x = pivot_x
        self._pivot_y = pivot_y
    def _create_swing(self, terrain_config: dict):
        pivot_x = self._pivot_x
        pivot_y = self._pivot_y
        rope_length = float(terrain_config.get("rope_length", 4.0))
        self._rope_length = rope_length
        seat_x = pivot_x
        seat_y = pivot_y - rope_length
        seat_radius = 0.3
        seat_density = float(terrain_config.get("seat_density", 35.0))
        seat_linear_damping = float(terrain_config.get("seat_linear_damping", 0.1))
        seat_angular_damping = float(terrain_config.get("seat_angular_damping", 0.1))
        seat = self._world.CreateDynamicBody(
            position=(seat_x, seat_y),
            fixtures=Box2D.b2FixtureDef(
                shape=circleShape(radius=seat_radius),
                density=seat_density,
                friction=0.3,
                restitution=0.1,
            ),
        )
        seat.linearDamping = seat_linear_damping
        seat.angularDamping = seat_angular_damping
        self._terrain_bodies["swing_seat"] = seat
        pivot_body = self._terrain_bodies["pivot"]
        defn = distanceJointDef()
        defn.bodyA = pivot_body
        defn.bodyB = seat
        defn.localAnchorA = pivot_body.GetLocalPoint((pivot_x, pivot_y))
        defn.localAnchorB = seat.GetLocalPoint((seat_x, seat_y))
        defn.length = rope_length
        defn.collideConnected = False
        self._world.CreateJoint(defn)
    def _define_target(self, terrain_config: dict):
        self._target_y_min = float(terrain_config.get("target_y_min", 11.7))
        self._target_x_min = float(terrain_config.get("target_x_min", 9.35))
        self._target_x_max = float(terrain_config.get("target_x_max", 10.65))
    MIN_BEAM_SIZE = 0.1
    MAX_BEAM_SIZE = 3.0
    def add_beam(self, x, y, width, height, angle=0, density=1.0):
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
    def add_joint(self, body_a, body_b, anchor_point, type="rigid"):
        if body_a is None:
            raise ValueError("add_joint: body_a cannot be None.")
        anchor_x, anchor_y = anchor_point[0], anchor_point[1]
        if body_b is None:
            body_b = self._terrain_bodies.get("ground")
            if body_b is None:
                raise ValueError("add_joint: Cannot anchor to ground.")
        if type == "rigid":
            joint = self._world.CreateWeldJoint(
                bodyA=body_a, bodyB=body_b,
                anchor=(anchor_x, anchor_y), collideConnected=False,
            )
        elif type == "pivot":
            joint = self._world.CreateRevoluteJoint(
                bodyA=body_a, bodyB=body_b,
                anchor=(anchor_x, anchor_y), collideConnected=False,
            )
        else:
            raise ValueError(f"Unknown joint type: {type}")
        self._joints.append(joint)
        return joint
    def get_structure_mass(self):
        return sum(b.mass for b in self._bodies)
    def set_material_properties(self, body, restitution=0.2):
        for fixture in body.fixtures:
            fixture.restitution = float(restitution)
    def step(self, time_step):
        seat = self._terrain_bodies.get("swing_seat")
        if seat is not None:
            vx, vy = seat.linearVelocity.x, seat.linearVelocity.y
            speed = math.sqrt(vx * vx + vy * vy)
            if speed > self._max_speed_seen:
                self._max_speed_seen = speed
            if speed > 100.0:
                self._extreme_velocity_detected = True
        if self._wind_enabled and seat is not None:
            if self._wind_period == 0:
                wind_fx = self._wind_strength
            else:
                wind_fx = self._wind_strength * math.sin(2.0 * math.pi * self._sim_time / self._wind_period)
            if self._rng.random() < self._gust_prob:
                wind_fx += self._gust_force * (1 if self._rng.random() > 0.5 else -1)
            self._last_wind_fx = wind_fx
            if len(self._wind_force_history) < 500:
                self._wind_force_history.append((self._sim_time, wind_fx))
            try:
                seat.ApplyForceToCenter((wind_fx, 0.0), True)
            except TypeError:
                seat.ApplyForceToCenter((wind_fx, 0.0), wake=True)
        qd = self._terrain_config.get("quadratic_damping", 0.0)
        if qd > 0.0 and seat is not None:
            vx, vy = seat.linearVelocity.x, seat.linearVelocity.y
            speed_sq = vx**2 + vy**2
            speed = math.sqrt(speed_sq)
            if speed > 0:
                drag_mag = qd * speed_sq
                drag_fx = -drag_mag * (vx / speed)
                drag_fy = -drag_mag * (vy / speed)
                try:
                    seat.ApplyForceToCenter((drag_fx, drag_fy), True)
                except TypeError:
                    seat.ApplyForceToCenter((drag_fx, drag_fy), wake=True)
        self._sim_time += time_step
        self._world.Step(time_step, 10, 10)
    def get_terrain_bounds(self):
        return {
            "ground_y": self._ground_y,
            "pivot_x": self._pivot_x,
            "pivot_y": self._pivot_y,
            "rope_length": getattr(self, "_rope_length", 4.0),
            "target_y_min": self._target_y_min,
            "target_x_min": self._target_x_min,
            "target_x_max": self._target_x_max,
            "max_pump_force": getattr(self, "MAX_PUMP_FORCE", 42.0),
            "build_zone": {
                "x": [self.BUILD_ZONE_X_MIN, self.BUILD_ZONE_X_MAX],
                "y": [self.BUILD_ZONE_Y_MIN, self.BUILD_ZONE_Y_MAX],
            },
        }
    def get_swing_seat_position(self):
        s = self._terrain_bodies.get("swing_seat")
        return (s.position.x, s.position.y) if s else None
    def get_swing_seat_velocity(self):
        s = self._terrain_bodies.get("swing_seat")
        return (s.linearVelocity.x, s.linearVelocity.y) if s else None
    def get_wind_force_at_time(self, sim_time):
        if not self._wind_enabled:
            return 0.0
        if self._wind_period == 0:
            return self._wind_strength
        return self._wind_strength * math.sin(2.0 * math.pi * sim_time / self._wind_period)
    def get_swing_seat(self):
        return self._terrain_bodies.get("swing_seat")
    def apply_force_to_seat(self, fx, fy):
        seat = self._terrain_bodies.get("swing_seat")
        if seat is not None:
            fx_req = float(fx)
            fy_req = float(fy)
            self._total_fx_requested += abs(fx_req)
            self._total_fy_requested += abs(fy_req)
            self._force_calls += 1
            max_f = getattr(self, "MAX_PUMP_FORCE", 42.0)
            fx_clamped = max(-max_f, min(max_f, float(fx_req)))
            fy_clamped = max(-max_f, min(max_f, float(fy_req)))
            if abs(fx_clamped) < abs(fx_req) - 1e-9 or abs(fy_clamped) < abs(fy_req) - 1e-9:
                self._force_clamped_count += 1
            fx_before_fault = fx_clamped
            fault = self._terrain_config.get("actuator_fault")
            if fault == "right_only" and fx_clamped < 0: fx_clamped = 0.0
            if fault == "left_only" and fx_clamped > 0: fx_clamped = 0.0
            if abs(fx_before_fault - fx_clamped) > 1e-9:
                self._force_suppressed_fault += 1
            fx_before_dz = fx_clamped
            fy_before_dz = fy_clamped
            dead_zone = self._terrain_config.get("dead_zone")
            if dead_zone:
                in_dz = dead_zone[0] <= seat.position.x <= dead_zone[1]
                if in_dz:
                    min_speed = self._terrain_config.get("dead_zone_min_speed")
                    if min_speed is not None:
                        vx = seat.linearVelocity.x
                        if abs(vx) < float(min_speed):
                            fx_clamped = 0.0
                            fy_clamped = 0.0
                    else:
                        fx_clamped = 0.0
                        fy_clamped = 0.0
            if abs(fx_before_dz - fx_clamped) > 1e-9 or abs(fy_before_dz - fy_clamped) > 1e-9:
                self._force_suppressed_deadzone += 1
            if abs(fx_clamped) > 1e-9 or abs(fy_clamped) > 1e-9:
                self._force_applied_count += 1
            else:
                self._force_suppressed_count += 1
            self._total_fx_delivered += abs(fx_clamped)
            self._total_fy_delivered += abs(fy_clamped)
            vx = seat.linearVelocity.x
            vy = seat.linearVelocity.y
            dot = fx_clamped * vx + fy_clamped * vy
            self._phase_alignment_samples += 1
            if dot > 1e-9:
                self._phase_aligned_count += 1
            elif dot < -1e-9:
                self._phase_antialigned_count += 1
            try:
                seat.ApplyForceToCenter((fx_clamped, fy_clamped), True)
            except TypeError:
                seat.ApplyForceToCenter((fx_clamped, fy_clamped), wake=True)
    def apply_impulse_to_seat(self, ix, iy):
        seat = self._terrain_bodies.get("swing_seat")
        if seat is not None:
            ix_req = float(ix)
            iy_req = float(iy)
            self._total_fx_requested += abs(ix_req)
            self._total_fy_requested += abs(iy_req)
            self._force_calls += 1
            max_i = getattr(self, "MAX_PUMP_FORCE", 42.0) * 0.1
            ix_clamped = max(-max_i, min(max_i, float(ix_req)))
            iy_clamped = max(-max_i, min(max_i, float(iy_req)))
            if abs(ix_clamped) < abs(ix_req) - 1e-9 or abs(iy_clamped) < abs(iy_req) - 1e-9:
                self._impulse_clamped_count += 1
            ix_before_fault = ix_clamped
            fault = self._terrain_config.get("actuator_fault")
            if fault == "right_only" and ix_clamped < 0: ix_clamped = 0.0
            if fault == "left_only" and ix_clamped > 0: ix_clamped = 0.0
            if abs(ix_before_fault - ix_clamped) > 1e-9:
                self._force_suppressed_fault += 1
            ix_before_dz = ix_clamped
            iy_before_dz = iy_clamped
            dead_zone = self._terrain_config.get("dead_zone")
            if dead_zone:
                in_dz = dead_zone[0] <= seat.position.x <= dead_zone[1]
                if in_dz:
                    min_speed = self._terrain_config.get("dead_zone_min_speed")
                    if min_speed is not None:
                        vx = seat.linearVelocity.x
                        if abs(vx) < float(min_speed):
                            ix_clamped = 0.0
                            iy_clamped = 0.0
                    else:
                        ix_clamped = 0.0
                        iy_clamped = 0.0
            if abs(ix_before_dz - ix_clamped) > 1e-9 or abs(iy_before_dz - iy_clamped) > 1e-9:
                self._force_suppressed_deadzone += 1
            if abs(ix_clamped) > 1e-9 or abs(iy_clamped) > 1e-9:
                self._force_applied_count += 1
            else:
                self._force_suppressed_count += 1
            self._total_fx_delivered += abs(ix_clamped)
            self._total_fy_delivered += abs(iy_clamped)
            vx = seat.linearVelocity.x
            vy = seat.linearVelocity.y
            dot = ix_clamped * vx + iy_clamped * vy
            self._phase_alignment_samples += 1
            if dot > 1e-9:
                self._phase_aligned_count += 1
            elif dot < -1e-9:
                self._phase_antialigned_count += 1
            try:
                seat.ApplyLinearImpulse((ix_clamped, iy_clamped), seat.worldCenter, True)
            except TypeError:
                seat.ApplyLinearImpulse((ix_clamped, iy_clamped), seat.worldCenter, wake=True)
    def get_sim_time(self):
        return self._sim_time
    def get_force_delivery_stats(self):
        total_requested = self._total_fx_requested + self._total_fy_requested
        total_delivered = self._total_fx_delivered + self._total_fy_delivered
        delivery_pct = (100.0 * total_delivered / total_requested) if total_requested > 0 else 100.0
        return {
            "force_calls": self._force_calls,
            "force_applied_count": self._force_applied_count,
            "force_suppressed_count": self._force_suppressed_count,
            "force_suppressed_deadzone": self._force_suppressed_deadzone,
            "force_suppressed_fault": self._force_suppressed_fault,
            "force_clamped_count": self._force_clamped_count,
            "impulse_clamped_count": self._impulse_clamped_count,
            "total_fx_requested": self._total_fx_requested,
            "total_fy_requested": self._total_fy_requested,
            "total_fx_delivered": self._total_fx_delivered,
            "total_fy_delivered": self._total_fy_delivered,
            "delivery_pct": delivery_pct,
        }
    def get_wind_params(self):
        return {
            "wind_enabled": self._wind_enabled,
            "wind_strength": self._wind_strength,
            "wind_period": self._wind_period,
            "gust_prob": self._gust_prob,
            "gust_force": self._gust_force,
            "last_wind_fx": self._last_wind_fx,
        }
    def get_seat_mass(self):
        seat = self._terrain_bodies.get("swing_seat")
        if seat is not None:
            try:
                return float(seat.mass)
            except (AttributeError, TypeError):
                pass
        return 0.0
    def get_energy_state(self):
        seat = self._terrain_bodies.get("swing_seat")
        if seat is None:
            return {"kinetic_energy": 0.0, "potential_energy": 0.0, "total_mechanical_energy": 0.0}
        mass = self.get_seat_mass()
        g = float(self._world.gravity.y) if hasattr(self._world, "gravity") else -10.0
        g = abs(g) if g != 0 else 10.0
        vx = float(seat.linearVelocity.x)
        vy = float(seat.linearVelocity.y)
        y = float(seat.position.y)
        ke = 0.5 * mass * (vx * vx + vy * vy)
        ground_y = self._ground_y
        pe = mass * g * max(0.0, y - ground_y)
        return {"kinetic_energy": ke, "potential_energy": pe, "total_mechanical_energy": ke + pe}
    def get_speed_stats(self):
        return {
            "max_speed_seen": self._max_speed_seen,
            "extreme_velocity_detected": self._extreme_velocity_detected,
        }
    def get_phase_alignment_stats(self):
        total = self._phase_alignment_samples
        if total == 0:
            return {"aligned_pct": 0.0, "antialigned_pct": 0.0, "neutral_pct": 100.0,
                    "aligned_count": 0, "antialigned_count": 0, "total_samples": 0}
        aligned = self._phase_aligned_count
        antialigned = self._phase_antialigned_count
        neutral = total - aligned - antialigned
        return {
            "aligned_pct": 100.0 * aligned / total,
            "antialigned_pct": 100.0 * antialigned / total,
            "neutral_pct": 100.0 * neutral / total,
            "aligned_count": aligned,
            "antialigned_count": antialigned,
            "total_samples": total,
        }
    def get_wind_force_history(self):
        return list(self._wind_force_history)
    def get_environment_params(self):
        return {
            "rope_length": getattr(self, "_rope_length", 4.0),
            "pivot_x": self._pivot_x,
            "pivot_y": self._pivot_y,
            "ground_y": self._ground_y,
            "quadratic_damping": self._terrain_config.get("quadratic_damping", 0.0),
            "actuator_fault": self._terrain_config.get("actuator_fault"),
            "dead_zone": self._terrain_config.get("dead_zone"),
            "dead_zone_min_speed": self._terrain_config.get("dead_zone_min_speed"),
            "seat_linear_damping": self._terrain_config.get("seat_linear_damping", 0.1),
            "seat_angular_damping": self._terrain_config.get("seat_angular_damping", 0.1),
        }
