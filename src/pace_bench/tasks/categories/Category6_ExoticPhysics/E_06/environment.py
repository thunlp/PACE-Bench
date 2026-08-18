import math

import random

import Box2D

from Box2D.b2 import polygonShape, world

TIME_STEP = 1.0 / 60.0
INV_TIME_STEP = 1.0 / TIME_STEP

class Sandbox:
    MAX_STEPS = 500
    BUILD_ZONE_X_MIN = 5.0
    BUILD_ZONE_X_MAX = 15.0
    BUILD_ZONE_Y_MIN = 1.5
    BUILD_ZONE_Y_MAX = 8.0
    MAX_STRUCTURE_MASS = 120.0
    MAX_GROUND_ANCHORS = 1
    MAX_BEAMS = 48
    MAX_JOINTS = 75
    MIN_ANCHOR_SPACING = 0.7
    FORBIDDEN_ZONE_X_LO = 9.7
    FORBIDDEN_ZONE_X_HI = 10.3
    DAMAGE_HOTSPOT_X_LO = 8.5
    DAMAGE_HOTSPOT_X_HI = 11.0
    DAMAGE_HOTSPOT_MULT = 1.35
    MIN_BEAM_SIZE = 0.1
    MAX_BEAM_SIZE = 4.0
    ALLOWED_ANCHOR_X_LO = 5.0
    ALLOWED_ANCHOR_X_HI = 6.5
    SUPPORT_CENTER_X = 5.75
    DISTANCE_SCALE_FACTOR = 0.6
    NOISE_STRENGTH = 42.0
    JOINT_BREAK_FORCE = 280800.0
    JOINT_BREAK_TORQUE = 414000.0
    DAMAGE_FORCE_THRESH = 43200.0
    DAMAGE_TORQUE_THRESH = 64800.0
    DAMAGE_FORCE_RATE = 2.9 / 3750.0
    DAMAGE_TORQUE_RATE = 2.1 / 3750.0
    DAMAGE_LIMIT = 100.0
    CASCADE_SHOCK_DAMAGE = 26.0
    CASCADE_RADIUS = 2.2
    GROUND_DAMAGE_FORCE_THRESH = 21600.0
    GROUND_DAMAGE_TORQUE_THRESH = 36000.0
    GROUND_DAMAGE_FORCE_RATE = 4.8 / 3750.0
    GROUND_DAMAGE_TORQUE_RATE = 3.5 / 3750.0
    BEAM_ANGVEL_THRESH = 2.2
    BEAM_ANGVEL_TOLERANCE_STEPS = 10
    PHASED_STORM_START = 100
    PHASED_STORM_END = 450
    PHASED_STORM_MULT = 1.9
    COHERENT_PULSE_INTERVAL = 58
    COHERENT_PULSE_FORCE = 36.0
    COHERENT_MOMENT_BASE = 18.0
    def __init__(self, *, terrain_config=None, physics_config=None):
        terrain_config = terrain_config or {}
        physics_config = physics_config or {}
        self._terrain_config = dict(terrain_config)
        self._physics_config = dict(physics_config)
        self._rng = random.Random(int(physics_config.get("random_seed", 123)))
        gravity = tuple(physics_config.get("gravity", (0, -10)))
        self._linear_damping = float(physics_config.get("linear_damping", 0.0))
        self._angular_damping = float(physics_config.get("angular_damping", 1.6))
        self._noise_strength = float(physics_config.get("noise_strength", self.NOISE_STRENGTH))
        _pc = physics_config
        if "coherent_pulse_interval" in _pc:
            self.COHERENT_PULSE_INTERVAL = int(_pc["coherent_pulse_interval"])
        if "coherent_pulse_force" in _pc:
            self.COHERENT_PULSE_FORCE = float(_pc["coherent_pulse_force"])
        if "joint_break_force" in _pc:
            self.JOINT_BREAK_FORCE = float(_pc["joint_break_force"])
        if "joint_break_torque" in _pc:
            self.JOINT_BREAK_TORQUE = float(_pc["joint_break_torque"])
        if "damage_limit" in _pc:
            self.DAMAGE_LIMIT = float(_pc["damage_limit"])
        if "damage_force_thresh" in _pc:
            self.DAMAGE_FORCE_THRESH = float(_pc["damage_force_thresh"])
        if "damage_torque_thresh" in _pc:
            self.DAMAGE_TORQUE_THRESH = float(_pc["damage_torque_thresh"])
        if "cascade_shock_damage" in _pc:
            self.CASCADE_SHOCK_DAMAGE = float(_pc["cascade_shock_damage"])
        if "beam_angvel_thresh" in _pc:
            self.BEAM_ANGVEL_THRESH = float(_pc["beam_angvel_thresh"])
        if "beam_angvel_tolerance_steps" in _pc:
            self.BEAM_ANGVEL_TOLERANCE_STEPS = int(_pc["beam_angvel_tolerance_steps"])
        if "phased_storm_mult" in _pc:
            self.PHASED_STORM_MULT = float(_pc["phased_storm_mult"])
        if "phased_storm_start" in _pc:
            self.PHASED_STORM_START = int(_pc["phased_storm_start"])
        if "phased_storm_end" in _pc:
            self.PHASED_STORM_END = int(_pc["phased_storm_end"])
        if "burst_prob" in _pc:
            self._burst_prob = float(_pc["burst_prob"])
        else:
            self._burst_prob = 0.026
        self._max_structure_mass = float(terrain_config.get("max_structure_mass", self.MAX_STRUCTURE_MASS))
        self._max_ground_anchors = int(terrain_config.get("max_ground_anchors", self.MAX_GROUND_ANCHORS))
        self._forbidden_zone_x_lo = float(terrain_config.get("forbidden_zone_x_lo", self.FORBIDDEN_ZONE_X_LO))
        self._forbidden_zone_x_hi = float(terrain_config.get("forbidden_zone_x_hi", self.FORBIDDEN_ZONE_X_HI))
        self._allowed_anchor_x_lo = float(terrain_config.get("allowed_anchor_x_lo", self.ALLOWED_ANCHOR_X_LO))
        self._allowed_anchor_x_hi = float(terrain_config.get("allowed_anchor_x_hi", self.ALLOWED_ANCHOR_X_HI))
        self._world = world(gravity=gravity, doSleep=True)
        self._bodies = []
        self._joints = []
        self._joint_is_ground = {}
        self._joint_anchor = {}
        self._terrain_bodies = {}
        self._joint_peak_forces = {}
        self._joint_peak_torques = {}
        self._joint_damage = {}
        self._beam_high_spin_steps = {}
        self._joint_fail_step = {}
        self._joint_fail_pos  = {}
        self._joint_fail_type = {}
        self._joint_fail_force = {}
        self._joint_fail_torque = {}
        self._joint_fail_damage = {}
        self._body_destroy_step  = {}
        self._body_destroy_pos   = {}
        self._body_destroy_reason = {}
        self._beam_peak_angvel   = {}
        self._peak_body_angvel   = 0.0
        self._num_joints_removed_force_torque = 0
        self._num_joints_removed_damage = 0
        self._num_bodies_destroyed_spin = 0
        self._num_bodies_destroyed_orphan = 0
        self._current_phase = "STABILIZATION"
        self._step_counter = 0
        self._burst_remaining = 0
        self._ground_anchor_count = 0
        self._ground_anchor_x = []
        self.world = self._world
        self.bodies = self._bodies
        self.joints = self._joints
        self._create_terrain(terrain_config)
    def _create_terrain(self, terrain_config: dict):
        ground_length = 40.0
        ground_height = 1.0
        ground = self._world.CreateStaticBody(
            position=(ground_length / 2, ground_height / 2),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(ground_length / 2, ground_height / 2)),
                friction=0.6,
            ),
        )
        self._terrain_bodies["ground"] = ground
        self._ground_y = ground_height
    def _get_noise_multiplier(self):
        if self._burst_remaining > 0:
            self._burst_remaining -= 1
            return 3.8
        if self._step_counter > 0 and self._rng.random() < getattr(self, "_burst_prob", 0.026):
            self._burst_remaining = 12
        mult = 1.0
        if self.PHASED_STORM_START <= self._step_counter <= self.PHASED_STORM_END:
            mult *= self.PHASED_STORM_MULT
        return mult
    def _apply_impulses(self, sigma_mult):
        sigma = self._noise_strength * sigma_mult
        total_mass = sum(b.mass for b in self._bodies) or 1.0
        mass_factor = 1.0 + 0.25 * math.sqrt(total_mass / 60.0)
        if self._step_counter > 0 and self._step_counter % self.COHERENT_PULSE_INTERVAL == 0:
            sign = 1 if (self._step_counter // self.COHERENT_PULSE_INTERVAL) % 2 == 0 else -1
            for body in self._bodies:
                dx = body.position.x - self.SUPPORT_CENTER_X
                tau = sign * self.COHERENT_MOMENT_BASE * mass_factor * max(0, dx) / 5.0
                body.ApplyTorque(tau, wake=True)
                fx = sign * self.COHERENT_PULSE_FORCE * mass_factor * (0.3 + 0.7 * max(0, dx) / 10.0)
                body.ApplyForceToCenter((fx, 0), wake=True)
        span = self.BUILD_ZONE_X_MAX - self.SUPPORT_CENTER_X
        for i, body in enumerate(self._bodies):
            dx = body.position.x - self.SUPPORT_CENTER_X
            dist_mult = 1.0 + self.DISTANCE_SCALE_FACTOR * max(0, dx) / max(span, 0.1)
            s = sigma * dist_mult
            if self._rng.random() < 0.28:
                if i % 2 == 0:
                    fx = self._rng.gauss(0, s) + 1.6 * s
                    fy = self._rng.gauss(0, s * 0.8)
                else:
                    fx = self._rng.gauss(0, s) - 1.6 * s
                    fy = self._rng.gauss(0, s * 0.8)
            else:
                fx = self._rng.gauss(0, s)
                fy = self._rng.gauss(0, s)
            body.ApplyForceToCenter((fx * mass_factor, fy * mass_factor), wake=True)
    def _get_joint_anchor_pos(self, joint):
        return self._joint_anchor.get(joint, (0, 0))
    def _record_and_destroy_joint(self, joint, fail_reason):
        ax, ay = self._get_joint_anchor_pos(joint)
        self._joint_fail_step[joint] = self._step_counter
        self._joint_fail_pos[joint] = (ax, ay)
        self._joint_fail_type[joint] = fail_reason
        self._joint_fail_force[joint] = self._joint_peak_forces.get(joint, 0.0)
        self._joint_fail_torque[joint] = self._joint_peak_torques.get(joint, 0.0)
        self._joint_fail_damage[joint] = self._joint_damage.get(joint, 0.0)
        self._world.DestroyJoint(joint)
        self._joints.remove(joint)
        self._joint_peak_forces.pop(joint, None)
        self._joint_peak_torques.pop(joint, None)
        self._joint_damage.pop(joint, None)
        if self._joint_is_ground.get(joint):
            self._ground_anchor_count = max(0, self._ground_anchor_count - 1)
            anchor_x = self._joint_anchor.get(joint, (0, 0))[0]
            if anchor_x in self._ground_anchor_x:
                self._ground_anchor_x.remove(anchor_x)
        self._joint_is_ground.pop(joint, None)
        self._joint_anchor.pop(joint, None)
        return ax, ay
    def _destroy_body_and_joints(self, body, fail_reason="orphan"):
        to_remove = [j for j in self._joints if j.bodyA == body or j.bodyB == body]
        for joint in to_remove:
            self._record_and_destroy_joint(joint, fail_reason)
        bx, by = body.position.x, body.position.y
        self._body_destroy_step[body] = self._step_counter
        self._body_destroy_pos[body] = (bx, by)
        self._body_destroy_reason[body] = fail_reason
        if fail_reason == "spin":
            self._num_bodies_destroyed_spin += 1
        else:
            self._num_bodies_destroyed_orphan += 1
        self._world.DestroyBody(body)
        self._bodies.remove(body)
        self._beam_high_spin_steps.pop(body, None)
    def step(self, time_step):
        if not math.isclose(float(time_step), TIME_STEP, rel_tol=0.0, abs_tol=1e-12):
            raise ValueError(f"E_06 requires a fixed time step of {TIME_STEP}")
        self._step_counter += 1
        sigma_mult = self._get_noise_multiplier()
        self._apply_impulses(sigma_mult)
        self._world.Step(time_step, 10, 10)
        if self._step_counter == 0:
            self._current_phase = "STABILIZATION"
        elif self.PHASED_STORM_START <= self._step_counter <= self.PHASED_STORM_END:
            self._current_phase = "STORM"
        elif self._step_counter > self.PHASED_STORM_END:
            self._current_phase = "POST_STORM"
        else:
            self._current_phase = "LOADING"
        for body in list(self._bodies):
            av = abs(body.angularVelocity)
            self._beam_peak_angvel[body] = max(self._beam_peak_angvel.get(body, 0.0), av)
            self._peak_body_angvel = max(self._peak_body_angvel, av)
            if av > self.BEAM_ANGVEL_THRESH:
                self._beam_high_spin_steps[body] = self._beam_high_spin_steps.get(body, 0) + 1
                if self._beam_high_spin_steps[body] >= self.BEAM_ANGVEL_TOLERANCE_STEPS:
                    self._destroy_body_and_joints(body, fail_reason="spin")
            else:
                self._beam_high_spin_steps[body] = 0
        joints_to_remove = []
        for joint in list(self._joints):
            force = joint.GetReactionForce(INV_TIME_STEP)
            force_mag = math.hypot(force.x, force.y)
            torque_mag = abs(joint.GetReactionTorque(INV_TIME_STEP))
            self._joint_peak_forces[joint] = max(
                self._joint_peak_forces.get(joint, 0.0), force_mag
            )
            self._joint_peak_torques[joint] = max(
                self._joint_peak_torques.get(joint, 0.0), torque_mag
            )
            if force_mag > self.JOINT_BREAK_FORCE or torque_mag > self.JOINT_BREAK_TORQUE:
                joints_to_remove.append((joint, "force_torque"))
                continue
            is_ground = self._joint_is_ground.get(joint, False)
            f_thresh = self.GROUND_DAMAGE_FORCE_THRESH if is_ground else self.DAMAGE_FORCE_THRESH
            t_thresh = self.GROUND_DAMAGE_TORQUE_THRESH if is_ground else self.DAMAGE_TORQUE_THRESH
            f_rate = self.GROUND_DAMAGE_FORCE_RATE if is_ground else self.DAMAGE_FORCE_RATE
            t_rate = self.GROUND_DAMAGE_TORQUE_RATE if is_ground else self.DAMAGE_TORQUE_RATE
            joint_x, _ = self._get_joint_anchor_pos(joint)
            hotspot_mult = (
                self.DAMAGE_HOTSPOT_MULT
                if self.DAMAGE_HOTSPOT_X_LO <= joint_x <= self.DAMAGE_HOTSPOT_X_HI
                else 1.0
            )
            damage = self._joint_damage.get(joint, 0.0)
            if force_mag > f_thresh:
                damage += (force_mag - f_thresh) * f_rate * TIME_STEP * hotspot_mult
            if torque_mag > t_thresh:
                damage += (torque_mag - t_thresh) * t_rate * TIME_STEP * hotspot_mult
            self._joint_damage[joint] = damage
            if damage >= self.DAMAGE_LIMIT:
                joints_to_remove.append((joint, "damage"))
        for entry in joints_to_remove:
            joint, fail_reason = entry
            if fail_reason == "damage":
                self._num_joints_removed_damage += 1
            else:
                self._num_joints_removed_force_torque += 1
            anchor_x, anchor_y = self._record_and_destroy_joint(joint, fail_reason)
            for neighbor in list(self._joints):
                neighbor_x, neighbor_y = self._get_joint_anchor_pos(neighbor)
                distance = math.hypot(
                    neighbor_x - anchor_x, neighbor_y - anchor_y
                )
                if 0.01 < distance <= self.CASCADE_RADIUS:
                    self._joint_damage[neighbor] = min(
                        self.DAMAGE_LIMIT,
                        self._joint_damage.get(neighbor, 0.0)
                        + self.CASCADE_SHOCK_DAMAGE,
                    )
    def add_beam(self, x, y, width, height, angle=0, density=1.0):
        if len(self._bodies) >= self.MAX_BEAMS:
            raise ValueError(f"Maximum {self.MAX_BEAMS} beams allowed")
        if self._forbidden_zone_x_lo <= x <= self._forbidden_zone_x_hi:
            raise ValueError(
                f"Forbidden zone: beam center x={x:.2f} falls in [{self._forbidden_zone_x_lo}, {self._forbidden_zone_x_hi}]"
            )
        values = (x, y, width, height, angle, density)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("Beam parameters must all be finite")
        if not self.MIN_BEAM_SIZE <= width <= self.MAX_BEAM_SIZE:
            raise ValueError(
                f"Beam width must be in [{self.MIN_BEAM_SIZE}, {self.MAX_BEAM_SIZE}]"
            )
        if not self.MIN_BEAM_SIZE <= height <= self.MAX_BEAM_SIZE:
            raise ValueError(
                f"Beam height must be in [{self.MIN_BEAM_SIZE}, {self.MAX_BEAM_SIZE}]"
            )
        if density <= 0.0:
            raise ValueError("Beam density must be positive")
        body = self._world.CreateDynamicBody(
            position=(x, y),
            angle=angle,
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(width / 2, height / 2)),
                density=density,
                friction=0.5,
            ),
        )
        body.linearDamping = self._linear_damping
        body.angularDamping = self._angular_damping
        body.userData = {"width": width, "height": height}
        self._bodies.append(body)
        return body
    def add_joint(self, body_a, body_b, anchor_point, type="rigid"):
        if len(self._joints) >= self.MAX_JOINTS:
            raise ValueError(f"Maximum {self.MAX_JOINTS} joints allowed")
        if body_a is None:
            raise ValueError("add_joint: body_a cannot be None.")
        anchor_x, anchor_y = anchor_point[0], anchor_point[1]
        if not math.isfinite(float(anchor_x)) or not math.isfinite(float(anchor_y)):
            raise ValueError("Joint anchor coordinates must be finite")
        if body_a not in self._bodies:
            raise ValueError("add_joint: body_a must be an active agent beam")
        if body_b is not None and body_b not in self._bodies:
            raise ValueError("add_joint: body_b must be an active agent beam or None")
        is_ground = body_b is None
        if is_ground:
            if self._ground_anchor_count >= self._max_ground_anchors:
                raise ValueError(f"Maximum {self._max_ground_anchors} ground anchors allowed")
            if not (self._allowed_anchor_x_lo <= anchor_x <= self._allowed_anchor_x_hi):
                raise ValueError(
                    f"Ground anchor x={anchor_x:.2f} outside allowed zone [{self._allowed_anchor_x_lo}, {self._allowed_anchor_x_hi}]"
                )
            if not math.isclose(
                float(anchor_y), self._ground_y, rel_tol=0.0, abs_tol=1e-9
            ):
                raise ValueError(
                    f"Ground-anchor y must equal the ground surface y={self._ground_y}"
                )
            for gx in self._ground_anchor_x:
                if abs(anchor_x - gx) < self.MIN_ANCHOR_SPACING:
                    raise ValueError(f"Ground anchors must be at least {self.MIN_ANCHOR_SPACING} m apart")
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
        self._joint_peak_forces[joint] = 0.0
        self._joint_peak_torques[joint] = 0.0
        self._joint_damage[joint] = 0.0
        self._joint_is_ground[joint] = is_ground
        self._joint_anchor[joint] = (anchor_x, anchor_y)
        if is_ground:
            self._ground_anchor_count += 1
            self._ground_anchor_x.append(anchor_x)
        return joint
    def get_structure_mass(self):
        return sum(b.mass for b in self._bodies)
    def get_build_zone(self):
        return (self.BUILD_ZONE_X_MIN, self.BUILD_ZONE_X_MAX, self.BUILD_ZONE_Y_MIN, self.BUILD_ZONE_Y_MAX)
    def get_structure_mass_limit(self):
        return self._max_structure_mass
    def _get_per_body_angvel_data(self):
        result = []
        seen_body_ids = set()
        for body in self._bodies:
            body_id = id(body)
            seen_body_ids.add(body_id)
            dx = body.position.x - self.SUPPORT_CENTER_X
            peak_av = self._beam_peak_angvel.get(body, 0.0)
            consec = self._beam_high_spin_steps.get(body, 0)
            w = 0.0
            h = 0.0
            if hasattr(body, "userData") and body.userData:
                w = float(body.userData.get("width", 0.0))
                h = float(body.userData.get("height", 0.0))
            result.append({
                "pos_x": body.position.x,
                "pos_y": body.position.y,
                "dist_from_support": dx,
                "peak_angvel": peak_av,
                "consec_high_steps": consec,
                "beam_width": w,
                "beam_height": h,
                "mass": body.mass,
                "status": "active",
            })
        for body, pav in self._beam_peak_angvel.items():
            if id(body) in seen_body_ids:
                continue
            destroy_pos = self._body_destroy_pos.get(body)
            destroy_reason = self._body_destroy_reason.get(body, "unknown")
            px, py = destroy_pos if destroy_pos else (0.0, 0.0)
            dx = px - self.SUPPORT_CENTER_X
            result.append({
                "pos_x": px,
                "pos_y": py,
                "dist_from_support": dx,
                "peak_angvel": pav,
                "consec_high_steps": 0,
                "beam_width": 0.0,
                "beam_height": 0.0,
                "mass": 0.0,
                "status": "destroyed",
                "destroy_reason": destroy_reason,
            })
        result.sort(key=lambda d: d["peak_angvel"], reverse=True)
        return result
    def _get_per_joint_stress_data(self):
        result = []
        for joint in self._joints:
            anchor = self._joint_anchor.get(joint, (0.0, 0.0))
            dx = anchor[0] - self.SUPPORT_CENTER_X
            result.append({
                "anchor_x": anchor[0],
                "anchor_y": anchor[1],
                "dist_from_support": dx,
                "peak_force": self._joint_peak_forces.get(joint, 0.0),
                "peak_torque": self._joint_peak_torques.get(joint, 0.0),
                "damage": self._joint_damage.get(joint, 0.0),
                "is_ground": self._joint_is_ground.get(joint, False),
                "status": "active",
            })
        for joint, step in self._joint_fail_step.items():
            pos = self._joint_fail_pos.get(joint, (0.0, 0.0))
            ftype = self._joint_fail_type.get(joint, "unknown")
            dx = pos[0] - self.SUPPORT_CENTER_X
            result.append({
                "anchor_x": pos[0],
                "anchor_y": pos[1],
                "dist_from_support": dx,
                "fail_step": step,
                "fail_type": ftype,
                "peak_force": self._joint_fail_force.get(joint, 0.0),
                "peak_torque": self._joint_fail_torque.get(joint, 0.0),
                "damage": self._joint_fail_damage.get(joint, 0.0),
                "status": "failed",
            })
        return result
    def _get_failure_event_timeline(self):
        events = []
        for j, step in self._joint_fail_step.items():
            pos = self._joint_fail_pos.get(j, (0.0, 0.0))
            ftype = self._joint_fail_type.get(j, "unknown")
            dx = pos[0] - self.SUPPORT_CENTER_X
            events.append({
                "step": step,
                "event_type": "joint_failure",
                "pos_x": pos[0],
                "pos_y": pos[1],
                "dist_from_support": dx,
                "fail_type": ftype,
                "peak_force": self._joint_fail_force.get(j, 0.0),
                "peak_torque": self._joint_fail_torque.get(j, 0.0),
                "damage": self._joint_fail_damage.get(j, 0.0),
            })
        for b, step in self._body_destroy_step.items():
            pos = self._body_destroy_pos.get(b, (0.0, 0.0))
            reason = self._body_destroy_reason.get(b, "unknown")
            dx = pos[0] - self.SUPPORT_CENTER_X
            events.append({
                "step": step,
                "event_type": "body_destroy",
                "pos_x": pos[0],
                "pos_y": pos[1],
                "dist_from_support": dx,
                "fail_reason": reason,
            })
        events.sort(key=lambda e: e["step"])
        return events
    def _get_forensic_summary(self):
        first_joint_fail_step = None
        first_joint_fail_pos  = None
        first_joint_fail_type = None
        first_body_fail_step  = None
        first_body_fail_pos   = None
        first_body_fail_reason = None
        for j, step in self._joint_fail_step.items():
            if step is not None and (first_joint_fail_step is None or step < first_joint_fail_step):
                first_joint_fail_step = step
                first_joint_fail_pos  = self._joint_fail_pos.get(j)
                first_joint_fail_type = self._joint_fail_type.get(j)
        for b, step in self._body_destroy_step.items():
            if step is not None and (first_body_fail_step is None or step < first_body_fail_step):
                first_body_fail_step   = step
                first_body_fail_pos    = self._body_destroy_pos.get(b)
                first_body_fail_reason = self._body_destroy_reason.get(b)
        peak_angvel = self._peak_body_angvel
        worst_spin_body     = None
        worst_spin_pos      = None
        worst_spin_peak     = 0.0
        worst_spin_consec   = 0
        for body, pav in self._beam_peak_angvel.items():
            if pav > worst_spin_peak:
                worst_spin_peak   = pav
                worst_spin_body   = body
                worst_spin_consec = self._beam_high_spin_steps.get(body, 0)
        if worst_spin_body is not None:
            if worst_spin_body in self._bodies:
                worst_spin_pos = (
                    worst_spin_body.position.x,
                    worst_spin_body.position.y,
                )
            else:
                worst_spin_pos = self._body_destroy_pos.get(worst_spin_body)
        beam_angvel_thresh = self.BEAM_ANGVEL_THRESH
        beam_angvel_tol    = self.BEAM_ANGVEL_TOLERANCE_STEPS
        return {
            "first_joint_fail_step":   first_joint_fail_step,
            "first_joint_fail_pos":   first_joint_fail_pos,
            "first_joint_fail_type":  first_joint_fail_type,
            "first_body_fail_step":   first_body_fail_step,
            "first_body_fail_pos":    first_body_fail_pos,
            "first_body_fail_reason": first_body_fail_reason,
            "num_joints_removed_force_torque": self._num_joints_removed_force_torque,
            "num_joints_removed_damage":      self._num_joints_removed_damage,
            "num_bodies_destroyed_spin":      self._num_bodies_destroyed_spin,
            "num_bodies_destroyed_orphan":    self._num_bodies_destroyed_orphan,
            "total_joints_removed": len(self._joint_fail_step),
            "total_bodies_destroyed": (
                self._num_bodies_destroyed_spin + self._num_bodies_destroyed_orphan
            ),
            "peak_body_angvel":       peak_angvel,
            "beam_angvel_thresh":     beam_angvel_thresh,
            "beam_angvel_tolerance_steps": beam_angvel_tol,
            "worst_spin_body_pos":    worst_spin_pos,
            "worst_spin_body_peak":   worst_spin_peak,
            "worst_spin_consec_steps": worst_spin_consec,
            "current_phase":          self._current_phase,
            "step_counter":           self._step_counter,
            "failure_event_timeline": self._get_failure_event_timeline(),
            "per_body_angvel_data": self._get_per_body_angvel_data(),
            "per_joint_stress_data": self._get_per_joint_stress_data(),
        }
    def get_terrain_bounds(self):
        return {
            "ground_y": self._ground_y,
            "build_zone": {
                "x": [self.BUILD_ZONE_X_MIN, self.BUILD_ZONE_X_MAX],
                "y": [self.BUILD_ZONE_Y_MIN, self.BUILD_ZONE_Y_MAX],
            },
            "max_structure_mass": self._max_structure_mass,
            "forbidden_zone": [self._forbidden_zone_x_lo, self._forbidden_zone_x_hi],
            "allowed_anchor_zone": [self._allowed_anchor_x_lo, self._allowed_anchor_x_hi],
            "max_ground_anchors": self._max_ground_anchors,
        }
