import math

APEX_SPEED_THRESHOLD = 1.0

VERTICAL_FALL_VX_THRESHOLD = 1.35

from pace_bench.core.primitives import compute_constraint_penalty

class Evaluator:
    def __init__(self, terrain_bounds, environment=None):
        self.terrain_bounds = terrain_bounds
        self.environment = environment
        self._target_y_min = terrain_bounds.get("target_y_min", 11.7)
        self._target_x_min = terrain_bounds.get("target_x_min", 9.35)
        self._target_x_max = terrain_bounds.get("target_x_max", 10.65)
        self._touched_target = False
        self._design_constraints_checked = False
        self._max_seat_y_reached = 0.0
        self._apex_reached = False
        self._apex_events = []
        self._peak_seat_speed = 0.0
        self._peak_kinetic_energy = 0.0
        self._peak_kinetic_energy_step = 0
        self._peak_potential_energy = 0.0
        if environment is None:
            raise ValueError("Evaluator requires environment instance")
        self.MAX_STRUCTURE_MASS = getattr(environment, "MAX_STRUCTURE_MASS", 100.0)
        self.BUILD_ZONE_X_MIN = environment.BUILD_ZONE_X_MIN
        self.BUILD_ZONE_X_MAX = environment.BUILD_ZONE_X_MAX
        self.BUILD_ZONE_Y_MIN = environment.BUILD_ZONE_Y_MIN
        self.BUILD_ZONE_Y_MAX = environment.BUILD_ZONE_Y_MAX
    def evaluate(self, agent_body, step_count, max_steps):
        if self.environment is None:
            return True, 0.0, {"error": "Environment not available"}
        pos = self.environment.get_swing_seat_position()
        vel = self.environment.get_swing_seat_velocity()
        if pos is None:
            return True, 0.0, {"error": "Swing seat not found"}
        px, py = pos
        vx = vel[0] if vel else 0.0
        vy = vel[1] if vel else 0.0
        speed = (vx * vx + vy * vy) ** 0.5
        if py > self._max_seat_y_reached:
            self._max_seat_y_reached = py
        if speed < APEX_SPEED_THRESHOLD:
            self._apex_reached = True
        if speed > self._peak_seat_speed:
            self._peak_seat_speed = speed
        if speed < APEX_SPEED_THRESHOLD and py > self._target_y_min * 0.3:
            if not self._apex_events or abs(step_count - self._apex_events[-1][0]) > 50:
                self._apex_events.append((step_count, px, py, speed))
                if len(self._apex_events) > 10:
                    self._apex_events.pop(0)
        energy = self.environment.get_energy_state() if self.environment else {}
        ke = energy.get("kinetic_energy", 0.0)
        pe = energy.get("potential_energy", 0.0)
        if ke > self._peak_kinetic_energy:
            self._peak_kinetic_energy = ke
            self._peak_kinetic_energy_step = step_count
        if pe > self._peak_potential_energy:
            self._peak_potential_energy = pe
        in_zone = py >= self._target_y_min and self._target_x_min <= px <= self._target_x_max
        if in_zone and speed < APEX_SPEED_THRESHOLD:
            self._touched_target = True
        if self._apex_reached and in_zone and vy <= 0 and abs(vx) < VERTICAL_FALL_VX_THRESHOLD:
            self._touched_target = True
        if not self._design_constraints_checked and step_count == 0:
            violations = self._check_design_constraints()
            if violations:
                self._design_constraints_checked = True
                metrics = self._make_metrics(pos, vel, step_count, False, True,
                    "Design constraint violated: " + "; ".join(violations), max_steps)
                return True, 0.0, metrics
            self._design_constraints_checked = True
        success = self._touched_target
        failed = False
        failure_reason = None
        if step_count >= max_steps - 1 and not self._touched_target:
            failed = True
            failure_reason = "Did not reach the target zone within the simulation step limit"
        done = failed or success or step_count >= max_steps - 1
        score = 100.0 if success else (0.0 if failed else 0.0)
        metrics = self._make_metrics(pos, vel, step_count, success, failed, failure_reason, max_steps)
        return done, score, metrics
    def _check_design_constraints(self):
        violations = []
        if self.environment is None:
            return ["Environment not available"]
        mass = self.environment.get_structure_mass()
        if mass > self.MAX_STRUCTURE_MASS:
            violations.append(f"Structure mass {mass:.2f} kg exceeds maximum {self.MAX_STRUCTURE_MASS} kg")
        for body in self.environment.bodies:
            x, y = body.position.x, body.position.y
            if not (self.BUILD_ZONE_X_MIN <= x <= self.BUILD_ZONE_X_MAX and
                    self.BUILD_ZONE_Y_MIN <= y <= self.BUILD_ZONE_Y_MAX):
                violations.append(
                    f"Beam center at ({x:.2f}, {y:.2f}) outside build zone "
                    f"x=[{self.BUILD_ZONE_X_MIN}, {self.BUILD_ZONE_X_MAX}], "
                    f"y=[{self.BUILD_ZONE_Y_MIN}, {self.BUILD_ZONE_Y_MAX}]"
                )
        return violations
    def _make_metrics(
        self, pos, vel, step_count, success=False, failed=False,
        failure_reason=None, max_steps=None,
    ):
        px, py = pos if pos else (0, 0)
        vx, vy = (vel[0], vel[1]) if vel else (0, 0)
        speed = (vx * vx + vy * vy) ** 0.5
        pivot_x = getattr(self.environment, "_pivot_x", 10.0)
        pivot_y = getattr(self.environment, "_pivot_y", 10.0)
        rope_length = getattr(self.environment, "_rope_length", 4.0)
        swing_bottom_y = pivot_y - rope_length
        dx = px - pivot_x
        dy = py - pivot_y
        swing_angle_rad = math.atan2(dx, pivot_y - py) if (pivot_y - py) != 0 else 0.0
        swing_angle_deg = math.degrees(swing_angle_rad)
        height_gap_to_target = max(0.0, self._target_y_min - py)
        target_center_x = (self._target_x_min + self._target_x_max) / 2
        distance_to_target_x = abs(px - target_center_x) if not (self._target_x_min <= px <= self._target_x_max) else 0.0
        in_zone = py >= self._target_y_min and self._target_x_min <= px <= self._target_x_max
        if self._target_y_min > swing_bottom_y:
            progress_pct = max(0.0, min(100.0, 100.0 * (py - swing_bottom_y) / (self._target_y_min - swing_bottom_y)))
        else:
            progress_pct = 100.0 if py >= self._target_y_min else 0.0
        env = self.environment
        force_stats = env.get_force_delivery_stats() if env else {}
        speed_stats = env.get_speed_stats() if env else {}
        phase_stats = env.get_phase_alignment_stats() if env else {}
        env_params = env.get_environment_params() if env else {}
        max_pump_force = getattr(env, "MAX_PUMP_FORCE", 42.0) if env else 42.0
        max_impulse = max_pump_force * 0.1
        force_ok = speed >= 0 and True
        force_clamped = force_stats.get("force_clamped_count", 0)
        constraint_force = {
            "label": "Force magnitude ≤ {:.1f} N per step".format(max_pump_force),
            "limit": max_pump_force,
            "pass": force_clamped == 0,
            "detail": "{} call(s) had force clamped to limit".format(force_clamped) if force_clamped else "All forces within limit",
        }
        impulse_clamped = force_stats.get("impulse_clamped_count", 0)
        constraint_impulse = {
            "label": "Impulse magnitude ≤ {:.2f} N·s per step".format(max_impulse),
            "limit": max_impulse,
            "pass": impulse_clamped == 0,
            "detail": "Impulse limit enforced per-step" if impulse_clamped == 0 else "{} call(s) clamped".format(impulse_clamped),
        }
        struct_mass = env.get_structure_mass() if env else 0.0
        mass_margin = self.MAX_STRUCTURE_MASS - struct_mass
        constraint_mass = {
            "label": "Structure mass ≤ {:.1f} kg".format(self.MAX_STRUCTURE_MASS),
            "limit": self.MAX_STRUCTURE_MASS,
            "value": struct_mass,
            "margin": mass_margin,
            "pass": struct_mass <= self.MAX_STRUCTURE_MASS,
            "detail": "{:.2f} kg used of {:.1f} kg budget ({:.1f}% remaining)".format(
                struct_mass, self.MAX_STRUCTURE_MASS, 100.0 * max(0, mass_margin) / max(1, self.MAX_STRUCTURE_MASS)),
        }
        constraint_zone = {
            "label": "Build zone x∈[{:.1f},{:.1f}] y∈[{:.1f},{:.1f}]".format(
                self.BUILD_ZONE_X_MIN, self.BUILD_ZONE_X_MAX, self.BUILD_ZONE_Y_MIN, self.BUILD_ZONE_Y_MAX),
            "pass": True,
            "detail": "Checked at build time" if self._design_constraints_checked else "Not yet checked",
        }
        apex_list = sorted(self._apex_events, key=lambda a: a[2], reverse=True)[:5]
        metrics = {
            "seat_x": px, "seat_y": py,
            "seat_vx": vx, "seat_vy": vy, "seat_speed": speed,
            "target_y_min": self._target_y_min,
            "target_x_min": self._target_x_min,
            "target_x_max": self._target_x_max,
            "success": success, "failed": failed, "failure_reason": failure_reason,
            "step_count": step_count,
            "max_steps": max_steps,
            "structure_mass": env.get_structure_mass() if env else 0.0,
            "max_structure_mass": self.MAX_STRUCTURE_MASS,
            "touched_target": self._touched_target,
            "apex_reached": self._apex_reached,
            "max_seat_y_reached": self._max_seat_y_reached,
            "height_gap_to_target": height_gap_to_target,
            "swing_angle_deg": swing_angle_deg,
            "progress_pct": progress_pct,
            "distance_to_target_x": distance_to_target_x,
            "in_zone_at_final": in_zone,
            "force_calls": force_stats.get("force_calls", 0),
            "force_applied_count": force_stats.get("force_applied_count", 0),
            "force_suppressed_count": force_stats.get("force_suppressed_count", 0),
            "force_suppressed_deadzone": force_stats.get("force_suppressed_deadzone", 0),
            "force_suppressed_fault": force_stats.get("force_suppressed_fault", 0),
            "force_clamped_count": force_stats.get("force_clamped_count", 0),
            "impulse_clamped_count": force_stats.get("impulse_clamped_count", 0),
            "force_delivery_pct": force_stats.get("delivery_pct", 100.0),
            "force_total_requested": force_stats.get("total_fx_requested", 0.0) + force_stats.get("total_fy_requested", 0.0),
            "force_total_delivered": force_stats.get("total_fx_delivered", 0.0) + force_stats.get("total_fy_delivered", 0.0),
            "max_pump_force": max_pump_force,
            "max_impulse": max_impulse,
            "phase_alignment_pct": phase_stats.get("aligned_pct", 0.0),
            "phase_antialigned_pct": phase_stats.get("antialigned_pct", 0.0),
            "phase_neutral_pct": phase_stats.get("neutral_pct", 0.0),
            "phase_aligned_count": phase_stats.get("aligned_count", 0),
            "phase_antialigned_count": phase_stats.get("antialigned_count", 0),
            "phase_total_samples": phase_stats.get("total_samples", 0),
            "peak_seat_speed": self._peak_seat_speed,
            "max_speed_seen": speed_stats.get("max_speed_seen", 0.0),
            "extreme_velocity_detected": speed_stats.get("extreme_velocity_detected", False),
            "rope_length": env_params.get("rope_length", 4.0),
            "pivot_x": env_params.get("pivot_x", 10.0),
            "pivot_y": env_params.get("pivot_y", 10.0),
            "swing_bottom_y": swing_bottom_y,
            "actuator_fault": env_params.get("actuator_fault"),
            "dead_zone": env_params.get("dead_zone"),
            "dead_zone_min_speed": env_params.get("dead_zone_min_speed"),
            "apex_events": apex_list,
            "constraint_force_limit": constraint_force,
            "constraint_impulse_limit": constraint_impulse,
            "constraint_structure_mass": constraint_mass,
            "constraint_build_zone": constraint_zone,
            "constraint_apex_in_zone": {
                "label": "Apex in zone (y≥{:.2f}, x∈[{:.2f},{:.2f}], speed<{:.1f})".format(
                    self._target_y_min, self._target_x_min, self._target_x_max, APEX_SPEED_THRESHOLD),
                "pass": self._touched_target,
                "detail": "Achieved" if self._touched_target else "Not achieved",
            },
            "constraint_vertical_fall": {
                "label": "Vertical fall into zone (|vx|<{:.2f}, vy≤0)".format(VERTICAL_FALL_VX_THRESHOLD),
                "pass": self._touched_target and speed < APEX_SPEED_THRESHOLD and vy <= 0 and abs(vx) < VERTICAL_FALL_VX_THRESHOLD and in_zone,
                "detail": "Satisifed at final step" if (self._touched_target and speed < APEX_SPEED_THRESHOLD and vy <= 0 and abs(vx) < VERTICAL_FALL_VX_THRESHOLD and in_zone) else "Not satisfied at final step",
            },
            "apex_speed_threshold": APEX_SPEED_THRESHOLD,
            "vertical_fall_vx_threshold": VERTICAL_FALL_VX_THRESHOLD,
            "lateral_margin_to_zone_left": px - self._target_x_min,
            "lateral_margin_to_zone_right": self._target_x_max - px,
            "vertical_margin_to_target": py - self._target_y_min,
        }
        return metrics
    def compute_score_with_penalty(self, score: float, metrics: dict) -> float:
        if score > 0:
            return score
        constraint_info = self.get_constraint_info()
        penalty = compute_constraint_penalty("D_04", score, metrics, constraint_info)
        return penalty
    def get_constraint_info(self):
        return {
            'target_y_min': self._target_y_min,
            'target_x_min': self._target_x_min,
            'target_x_max': self._target_x_max,
            'max_structure_mass': self.MAX_STRUCTURE_MASS,
            'build_zone_x_min': self.BUILD_ZONE_X_MIN,
            'build_zone_x_max': self.BUILD_ZONE_X_MAX,
            'build_zone_y_min': self.BUILD_ZONE_Y_MIN,
            'build_zone_y_max': self.BUILD_ZONE_Y_MAX,
        }
    def get_task_description(self):
        return {
            "task": "D-04: The Swing",
            "description": "Pump swing so apex (v≈0) is in target zone or seat falls vertically into zone; wind/damping/force limit.",
            "success_criteria": {
                "primary": f"(1) Apex in zone: seat in zone (y>={self._target_y_min}, x in [{self._target_x_min},{self._target_x_max}]) with speed < {APEX_SPEED_THRESHOLD} m/s, OR (2) After apex, fall vertically (|vx|<{VERTICAL_FALL_VX_THRESHOLD}, vy<=0) into zone",
                "failure": "Did not satisfy apex-in-zone or vertical-fall-into-zone",
            },
            "evaluation": {"score_range": "0-100", "success_score": 100, "failure_score": 0},
        }
