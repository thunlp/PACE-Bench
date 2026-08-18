from __future__ import annotations

import Box2D

from Box2D.b2 import (world, polygonShape, circleShape, staticBody, dynamicBody, revoluteJoint, weldJoint)

import math

from typing import List, Dict, Any

class Sandbox:
    def __init__(self, *, terrain_config=None, physics_config=None):
        terrain_config = terrain_config or {}
        physics_config = physics_config or {}
        self._terrain_config = dict(terrain_config)
        self._physics_config = dict(physics_config)
        gravity = tuple(physics_config.get("gravity", (0, -8)))
        self._default_linear_damping = float(physics_config.get("linear_damping", 0.0))
        self._default_angular_damping = float(physics_config.get("angular_damping", 0.0))
        self._wind_force = float(terrain_config.get("wind_force", 0.0))
        self._wind_oscillation = float(terrain_config.get("wind_oscillation", 0.0))
        self._max_joint_force = float(physics_config.get("max_joint_force", float('inf')))
        self._max_joint_torque = float(physics_config.get("max_joint_torque", float('inf')))
        self._gravity_evolution = float(physics_config.get("gravity_evolution", 0.0))
        self._initial_gravity_y = gravity[1]
        self._destroy_ground_time = float(terrain_config.get("destroy_ground_time", -1.0))
        self._boulder_interval = float(terrain_config.get("boulder_interval", -1.0))
        self._wall_oscillation_amp = float(terrain_config.get("wall_oscillation_amp", 0.0))
        self._wall_oscillation_freq = float(terrain_config.get("wall_oscillation_freq", 0.0))
        self._vortex_y = float(terrain_config.get("vortex_y", 100.0))
        self._vortex_force_x = float(terrain_config.get("vortex_force_x", 0.0))
        self._vortex_force_y = float(terrain_config.get("vortex_force_y", 0.0))
        self._suction_zones = terrain_config.get("suction_zones", None)
        self._world = world(gravity=gravity, doSleep=True)
        self._bodies = []
        self._joints = []
        self._terrain_bodies = {}
        self._climber_bodies = {}
        self._climber_joints = []
        self._pads = []
        self._pad_active = {}
        self._physics_history: List[dict] = []
        self._current_step: int = 0
        self._last_joint_stress: List[dict] = []
        self._joint_failure_events: List[dict] = []
        self._observation_error_count: int = 0
        self._last_observation_error: str | None = None
        self.world = self._world
        self.bodies = self._bodies
        self.joints = self._joints
        self._create_terrain(terrain_config)
        self.BUILD_ZONE_X_MIN = 0.0
        self.BUILD_ZONE_X_MAX = 5.0
        self.BUILD_ZONE_Y_MIN = 0.0
        self.BUILD_ZONE_Y_MAX = float(terrain_config.get("build_zone_y_max", 25.0))
        self.MAX_STRUCTURE_MASS = float(terrain_config.get("max_structure_mass", 50.0))
        self.MIN_STRUCTURE_MASS = float(terrain_config.get("min_structure_mass", 0.0))
        self.TARGET_HEIGHT = float(terrain_config.get("target_height", 20.0))
        self.FELL_HEIGHT_THRESHOLD = float(terrain_config.get("fell_height_threshold", 0.5))
        self._create_initial_climber_template(terrain_config)
    def _create_terrain(self, terrain_config: dict):
        wall_friction = float(terrain_config.get("wall_friction", 1.0))
        wall_x = 5.0
        wall_height = 30.0
        wall_thickness = 0.5
        wall_type = Box2D.b2_kinematicBody if self._wall_oscillation_amp > 0 else Box2D.b2_staticBody
        wall = self._world.CreateBody(
            type=wall_type,
            position=(wall_x + wall_thickness / 2, wall_height / 2),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(wall_thickness / 2, wall_height / 2)),
                friction=wall_friction,
                restitution=0.1,
            ),
        )
        self._terrain_bodies["wall"] = wall
        self._wall_x = wall_x
        self._wall_height = wall_height
        ground_length = 10.0
        ground_height = 1.0
        ground = self._world.CreateStaticBody(
            position=(ground_length / 2, ground_height / 2),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(ground_length / 2, ground_height / 2)),
                friction=0.8,
            ),
        )
        self._terrain_bodies["ground"] = ground
        self._ground_y = ground_height
    def _create_initial_climber_template(self, terrain_config: dict):
        spawn_x = 3.0
        spawn_y = 2.0
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
        self._climber_bodies["body_template"] = body
    MIN_BEAM_SIZE = 0.05
    MAX_BEAM_SIZE = 3.0
    MIN_PAD_RADIUS = 0.05
    MAX_PAD_RADIUS = 0.25
    MIN_JOINT_LIMIT = -math.pi
    MAX_JOINT_LIMIT = math.pi
    BUILD_ZONE_X_MIN = 0.0
    BUILD_ZONE_X_MAX = 5.0
    BUILD_ZONE_Y_MIN = 0.0
    BUILD_ZONE_Y_MAX = 25.0
    MAX_STRUCTURE_MASS = 50.0
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
    def add_pad(self, x, y, radius=0.12, density=0.8):
        radius = max(self.MIN_PAD_RADIUS, min(radius, self.MAX_PAD_RADIUS))
        body = self._world.CreateDynamicBody(
            position=(x, y),
            fixtures=Box2D.b2FixtureDef(
                shape=circleShape(radius=radius),
                density=density,
                friction=1.5,
            )
        )
        body.linearDamping = self._default_linear_damping
        body.angularDamping = self._default_angular_damping
        self._bodies.append(body)
        self._pads.append(body)
        self._pad_active[body] = False
        return body
    def set_pad_active(self, pad, active):
        if pad in self._pads:
            self._pad_active[pad] = bool(active)
    def add_joint(self, body_a, body_b, anchor_point, type='pivot', lower_limit=None, upper_limit=None):
        if body_a is None:
            raise ValueError("add_joint: body_a cannot be None. You must provide a valid body object (e.g., from add_beam).")
        if body_b is None:
            raise ValueError("add_joint: body_b cannot be None. You must provide a valid body object.")
        anchor_x, anchor_y = anchor_point[0], anchor_point[1]
        if type == 'rigid':
            joint = self._world.CreateWeldJoint(
                bodyA=body_a,
                bodyB=body_b,
                anchor=(anchor_x, anchor_y),
                collideConnected=False
            )
        elif type == 'pivot':
            joint_kwargs = {
                'bodyA': body_a,
                'bodyB': body_b,
                'anchor': (anchor_x, anchor_y),
                'collideConnected': False
            }
            if lower_limit is not None and upper_limit is not None:
                joint_kwargs['lowerAngle'] = max(self.MIN_JOINT_LIMIT, min(lower_limit, self.MAX_JOINT_LIMIT))
                joint_kwargs['upperAngle'] = min(self.MAX_JOINT_LIMIT, max(upper_limit, self.MIN_JOINT_LIMIT))
                joint_kwargs['enableLimit'] = True
            joint = self._world.CreateRevoluteJoint(**joint_kwargs)
        else:
            raise ValueError(f"Unknown joint type: {type}")
        self._joints.append(joint)
        return joint
    def set_motor(self, joint, motor_speed, max_torque=100.0):
        if not isinstance(joint, Box2D.b2RevoluteJoint):
            raise ValueError("set_motor: joint must be a pivot/revolute joint")
        joint.enableMotor = True
        joint.motorSpeed = float(motor_speed)
        joint.maxMotorTorque = float(max_torque)
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
    def step(self, time_step):
        wall_x = getattr(self, '_wall_x', 5.0)
        for pad in self._pads:
            is_in_zone = True
            if self._suction_zones:
                is_in_zone = any(z[0] <= pad.position.y <= z[1] for z in self._suction_zones)
            if self._pad_active.get(pad, False) and is_in_zone:
                pad.type = Box2D.b2_staticBody
                pad.position = (wall_x, pad.position.y + 1.5 * time_step)
            else:
                pad.type = Box2D.b2_dynamicBody
        if self._wind_force != 0:
            if self._bodies:
                b = self._bodies[0]
                force_x = self._wind_force
                if self._wind_oscillation > 0:
                    t = getattr(self, '_time', 0.0)
                    force_x *= (0.5 + 0.5 * math.sin(self._wind_oscillation * t))
                b.ApplyForce((force_x, 0), b.worldCenter, True)
        if not hasattr(self, '_time'): self._time = 0.0
        self._time += time_step
        if self._destroy_ground_time > 0 and self._time >= self._destroy_ground_time:
            if "ground" in self._terrain_bodies:
                self._world.DestroyBody(self._terrain_bodies["ground"])
                del self._terrain_bodies["ground"]
        if self._boulder_interval > 0:
            if not hasattr(self, '_last_boulder_time'): self._last_boulder_time = 0.0
            if self._time - self._last_boulder_time >= self._boulder_interval:
                boulder = self._world.CreateDynamicBody(position=(4.6, 28.0))
                boulder.CreateCircleFixture(radius=0.3, density=20.0, friction=0.5, restitution=0.1)
                self._last_boulder_time = self._time
        if self._wall_oscillation_amp > 0:
            wall = self._terrain_bodies["wall"]
            vx = self._wall_oscillation_amp * self._wall_oscillation_freq * math.cos(self._wall_oscillation_freq * self._time)
            wall.linearVelocity = (vx, 0)
            self._wall_x = wall.position.x
        if self._vortex_y < 100.0:
            for b in self._bodies:
                if b.position.y > self._vortex_y:
                    b.ApplyForceToCenter((self._vortex_force_x * b.mass, self._vortex_force_y * b.mass), True)
        if self._gravity_evolution != 0:
            new_g = self._initial_gravity_y + self._gravity_evolution * self._time
            self._world.gravity = (0, new_g)
        self._world.Step(time_step, 10, 10)
        self._last_joint_stress = self._compute_joint_stress(time_step)
        if self._max_joint_force < float('inf') or self._max_joint_torque < float('inf'):
            to_destroy = []
            failure_records = []
            for j in self._joints:
                try:
                    reaction_force = j.GetReactionForce(1.0/time_step).length
                    reaction_torque = abs(j.GetReactionTorque(1.0/time_step))
                    if reaction_force > self._max_joint_force or reaction_torque > self._max_joint_torque:
                        to_destroy.append(j)
                        force_pct = (reaction_force / self._max_joint_force * 100.0) if self._max_joint_force < float("inf") else float("inf")
                        torque_pct = (reaction_torque / self._max_joint_torque * 100.0) if self._max_joint_torque < float("inf") else float("inf")
                        failure_records.append({
                            "step": self._current_step,
                            "time": round(getattr(self, "_time", 0.0), 3),
                            "force_N": round(reaction_force, 3),
                            "force_pct": round(force_pct, 2),
                            "torque_Nm": round(reaction_torque, 3),
                            "torque_pct": round(torque_pct, 2),
                            "force_limit_N": self._max_joint_force,
                            "torque_limit_Nm": self._max_joint_torque,
                        })
                except Exception as exc:
                    self._record_observation_error("step.joint_failure_check", exc)
                    continue
            self._joint_failure_events.extend(failure_records)
            for j in to_destroy:
                if j in self._joints:
                    self._world.DestroyJoint(j)
                    self._joints.remove(j)
        self._physics_history.append(self._snapshot_step_state(time_step))
        self._current_step += 1
    def _record_observation_error(self, source: str, exc: Exception) -> None:
        self._observation_error_count += 1
        self._last_observation_error = f"{source}: {type(exc).__name__}: {exc}"
    def get_terrain_bounds(self):
        wall_x = getattr(self, '_wall_x', 5.0)
        wall_contact_x = [wall_x - 1.5, wall_x + 2.5]
        return {
            "wall": {"x": self._wall_x, "height": self._wall_height},
            "ground": {"y": self._ground_y},
            "target_height": self.TARGET_HEIGHT,
            "fell_height_threshold": getattr(self, "FELL_HEIGHT_THRESHOLD", 0.5),
            "wall_contact_x": wall_contact_x,
            "build_zone": {
                "x": [self.BUILD_ZONE_X_MIN, self.BUILD_ZONE_X_MAX],
                "y": [self.BUILD_ZONE_Y_MIN, self.BUILD_ZONE_Y_MAX]
            }
        }
    def get_climber_position(self):
        if not self._bodies:
            return None
        if self._bodies:
            body = self._bodies[0]
            return (body.position.x, body.position.y)
        return None
    def _compute_joint_stress(self, time_step: float) -> List[Dict[str, Any]]:
        results: List[Dict[str, Any]] = []
        for i, j in enumerate(self._joints):
            try:
                rf = j.GetReactionForce(1.0 / time_step).length
                rt = abs(j.GetReactionTorque(1.0 / time_step))
                force_pct = (rf / self._max_joint_force * 100.0
                             if self._max_joint_force < float("inf") else 0.0)
                torque_pct = (rt / self._max_joint_torque * 100.0
                              if self._max_joint_torque < float("inf") else 0.0)
                results.append({
                    "joint_index": i,
                    "force_N": round(rf, 3),
                    "force_pct": round(force_pct, 2),
                    "torque_Nm": round(rt, 3),
                    "torque_pct": round(torque_pct, 2),
                })
            except Exception as exc:
                self._record_observation_error("_compute_joint_stress", exc)
        return results
    def _snapshot_step_state(self, time_step: float) -> Dict[str, Any]:
        grav_y = self._world.gravity.y
        wind_this_step = 0.0
        if self._wind_force != 0 and self._bodies:
            wind_this_step = self._wind_force
            if self._wind_oscillation > 0:
                t = getattr(self, "_time", 0.0) - time_step
                wind_this_step *= (0.5 + 0.5 * math.sin(self._wind_oscillation * t))
        stress_list = self._last_joint_stress
        peak_force_pct = max((s["force_pct"] for s in stress_list), default=0.0)
        peak_torque_pct = max((s["torque_pct"] for s in stress_list), default=0.0)
        body_vels = []
        for b in self._bodies:
            try:
                body_vels.append({
                    "id": id(b),
                    "vx": round(b.linearVelocity.x, 3),
                    "vy": round(b.linearVelocity.y, 3),
                    "ang_vel": round(b.angularVelocity, 3),
                })
            except Exception as exc:
                self._record_observation_error("_snapshot_step_state.body_velocity", exc)
        pad_states = []
        for pad in self._pads:
            try:
                pad_states.append({
                    "y": round(pad.position.y, 3),
                    "x": round(pad.position.x, 3),
                    "active": self._pad_active.get(pad, False),
                })
            except Exception as exc:
                self._record_observation_error("_snapshot_step_state.pad", exc)
        total_ke = 0.0
        total_pe = 0.0
        g_mag = abs(grav_y)
        for b in self._bodies:
            try:
                vx = b.linearVelocity.x
                vy = b.linearVelocity.y
                total_ke += 0.5 * b.mass * (vx * vx + vy * vy)
                total_pe += b.mass * g_mag * max(b.position.y, 0.0)
            except Exception as exc:
                self._record_observation_error("_snapshot_step_state.energy", exc)
        max_body_speed = 0.0
        max_body_ang_vel = 0.0
        for b in self._bodies:
            try:
                spd = math.hypot(b.linearVelocity.x, b.linearVelocity.y)
                if spd > max_body_speed:
                    max_body_speed = spd
                av = abs(b.angularVelocity)
                if av > max_body_ang_vel:
                    max_body_ang_vel = av
            except Exception as exc:
                self._record_observation_error("_snapshot_step_state.speed", exc)
        return {
            "step": self._current_step,
            "time": round(getattr(self, "_time", 0.0), 3),
            "gravity_y": round(grav_y, 4),
            "gravity_evolution_rate": self._gravity_evolution,
            "wind_force_x": round(wind_this_step, 3),
            "num_joints_remaining": len(self._joints),
            "peak_joint_force_pct": round(peak_force_pct, 2),
            "peak_joint_torque_pct": round(peak_torque_pct, 2),
            "num_pads": len(self._pads),
            "num_active_pads": sum(1 for p in self._pads if self._pad_active.get(p, False)),
            "body_velocities": body_vels,
            "pad_states": pad_states,
            "joint_stress_per_joint": stress_list,
            "total_ke": round(total_ke, 3),
            "total_pe": round(total_pe, 3),
            "max_body_speed": round(max_body_speed, 3),
            "max_body_ang_vel": round(max_body_ang_vel, 3),
        }
    def get_physics_state(self) -> Dict[str, Any]:
        if self._physics_history:
            return dict(self._physics_history[-1])
        return {}
    def get_physics_history(self) -> List[Dict[str, Any]]:
        return list(self._physics_history)
    def get_current_step(self) -> int:
        return self._current_step
    def get_suction_zones(self) -> Any:
        return self._suction_zones
    def get_wind_config(self) -> Dict[str, Any]:
        return {
            "wind_force": self._wind_force,
            "wind_oscillation": self._wind_oscillation,
        }
    def get_vortex_config(self) -> Dict[str, Any]:
        return {
            "vortex_y": self._vortex_y,
            "vortex_force_x": self._vortex_force_x,
            "vortex_force_y": self._vortex_force_y,
        }
    def get_gravity_config(self) -> Dict[str, Any]:
        return {
            "initial_gravity_y": self._initial_gravity_y,
            "gravity_evolution": self._gravity_evolution,
        }
    def get_joint_failure_events(self) -> List[Dict[str, Any]]:
        return list(self._joint_failure_events)
    def get_joint_limits(self) -> Dict[str, float]:
        return {
            "max_joint_force": self._max_joint_force,
            "max_joint_torque": self._max_joint_torque,
        }
    def get_energy_state(self) -> Dict[str, Any]:
        total_ke = 0.0
        total_pe = 0.0
        grav_y = self._world.gravity.y
        g_mag = abs(grav_y)
        for b in self._bodies:
            try:
                vx = b.linearVelocity.x
                vy = b.linearVelocity.y
                total_ke += 0.5 * b.mass * (vx * vx + vy * vy)
                total_pe += b.mass * g_mag * max(b.position.y, 0.0)
            except Exception as exc:
                self._record_observation_error("get_energy_state", exc)
        return {
            "total_ke": round(total_ke, 3),
            "total_pe": round(total_pe, 3),
        }
