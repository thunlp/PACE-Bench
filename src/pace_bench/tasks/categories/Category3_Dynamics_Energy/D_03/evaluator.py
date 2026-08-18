import math

from pace_bench.core.simulator import TIME_STEP

from pace_bench.core.primitives import compute_constraint_penalty

class Evaluator:
    def __init__(self, terrain_bounds, environment=None):
        self.terrain_bounds = terrain_bounds
        self.environment = environment
        self._target_x_min = float(terrain_bounds.get("target_x_min", 11.75))
        self._target_speed_min = float(terrain_bounds.get("target_speed_min", 0.45))
        self._target_speed_max = float(terrain_bounds.get("target_speed_max", 2.6))
        self._speed_trap_x = float(terrain_bounds.get("speed_trap_x", 9.0))
        self._speed_trap_min = float(terrain_bounds.get("speed_trap_min", 2.8))
        self._checkpoint_11_x = float(terrain_bounds.get("checkpoint_11_x", 11.0))
        self._checkpoint_11_speed_min = float(terrain_bounds.get("checkpoint_11_speed_min", 1.1))
        self._checkpoint_11_speed_max = float(terrain_bounds.get("checkpoint_11_speed_max", 2.7))
        self.design_constraints_checked = False
    def evaluate(self, agent_body, step_count, max_steps):
        if not self.environment:
            return True, 0.0, {"error": "Environment not available"}
        if not self.design_constraints_checked and step_count == 0:
            violations = self._check_design_constraints()
            if violations:
                self.design_constraints_checked = True
                return True, 0.0, {"failed": True, "failure_reason": "Design constraint violated: " + "; ".join(violations)}
            self.design_constraints_checked = True
        cabin_pos = self.environment.get_vehicle_position()
        cabin_vel = self.environment.get_vehicle_velocity()
        if cabin_pos is None or cabin_vel is None:
            return False, 0.0, {"failed": True, "failure_reason": "Cart not found"}
        current_x, current_y = cabin_pos
        vx, vy = cabin_vel
        speed = math.sqrt(vx*vx + vy*vy)
        failed = False
        failure_reason = None
        if getattr(self.environment, "get_gate_collision", lambda: False)():
            return True, 0.0, {"failed": True, "failure_reason": "Gate collision"}
        if getattr(self.environment, "_speed_trap_failed", False):
            return True, 0.0, {"failed": True, "failure_reason": f"Speed trap failed (too slow at x={self._speed_trap_x:.1f})"}
        if getattr(self.environment, "_checkpoint_11_failed", False):
            return True, 0.0, {"failed": True, "failure_reason": f"Checkpoint failed (speed at x={self._checkpoint_11_x:.1f} out of band)"}
        is_end = (step_count >= max_steps - 1)
        success = False
        if not failed:
            if current_x >= self._target_x_min:
                if self._target_speed_min <= speed <= self._target_speed_max:
                    success = True
                elif is_end:
                    failed, failure_reason = True, f"Final speed out of band ({speed:.2f} m/s)"
            elif is_end:
                failed, failure_reason = True, f"Did not reach target zone (x={current_x:.2f} < {self._target_x_min})"
        done = failed or success or is_end
        score = 100.0 if success else 0.0
        if not done:
            score = min(current_x / self._target_x_min, 1.0) * 80.0
        env = self.environment
        speed_trap_actual = getattr(env, "get_speed_trap_actual_speed", lambda: None)()
        checkpoint_11_actual = getattr(env, "get_checkpoint_11_actual_speed", lambda: None)()
        gate_arrivals = getattr(env, "get_gate_arrival_events", lambda: [])()
        gate_collisions = getattr(env, "get_gate_collision_details", lambda: [])()
        zone_crossings = getattr(env, "get_zone_crossings", lambda: [])()
        peak_speed = getattr(env, "get_peak_speed", lambda: None)()
        speed_trace = getattr(env, "get_speed_trace", lambda: [])()
        observation_errors = getattr(env, "get_observation_errors", lambda: [])()
        speed_trap_margin = None
        if speed_trap_actual is not None:
            speed_trap_margin = speed_trap_actual - self._speed_trap_min
        cp_lo_margin = None
        cp_hi_margin = None
        if checkpoint_11_actual is not None:
            cp_lo_margin = checkpoint_11_actual - self._checkpoint_11_speed_min
            cp_hi_margin = self._checkpoint_11_speed_max - checkpoint_11_actual
        return done, score, {
            "x": current_x,
            "speed": speed,
            "vx": vx,
            "vy": vy,
            "success": success,
            "failed": failed,
            "failure_reason": failure_reason,
            "step_count": step_count,
            "max_steps": max_steps,
            "target_x_min": self._target_x_min,
            "target_speed_min": self._target_speed_min,
            "target_speed_max": self._target_speed_max,
            "speed_trap_x": self._speed_trap_x,
            "speed_trap_min": self._speed_trap_min,
            "speed_trap_actual_speed": speed_trap_actual,
            "speed_trap_speed_margin": speed_trap_margin,
            "checkpoint_11_x": self._checkpoint_11_x,
            "checkpoint_11_speed_min": self._checkpoint_11_speed_min,
            "checkpoint_11_speed_max": self._checkpoint_11_speed_max,
            "checkpoint_11_actual_speed": checkpoint_11_actual,
            "checkpoint_11_speed_margin_low": cp_lo_margin,
            "checkpoint_11_speed_margin_high": cp_hi_margin,
            "beam_count": len(self.environment._bodies),
            "min_beam_count": self.terrain_bounds.get("min_beam_count", 4),
            "max_beam_count": self.terrain_bounds.get("max_beam_count", 5),
            "structure_mass": self.environment.get_structure_mass(),
            "max_structure_mass": self.terrain_bounds.get("max_structure_mass", 14.0),
            "build_zone": self.terrain_bounds.get("build_zone", {}),
            "gate_x": self.terrain_bounds.get("gate_pivot_x", 10.0),
            "gate2_x": self.terrain_bounds.get("gate2_pivot_x", 11.5),
            "gate3_x": self.terrain_bounds.get("gate3_pivot_x", 11.75),
            "gate4_x": self.terrain_bounds.get("gate4_pivot_x", 12.5),
            "gate_arrival_events": gate_arrivals,
            "gate_collision_details": gate_collisions,
            "zone_crossings": zone_crossings,
            "peak_speed": peak_speed,
            "speed_trace": speed_trace,
            "observation_errors": observation_errors,
            "time_step": TIME_STEP,
            "mud_zone": self.terrain_bounds.get("mud_zone", [5.5, 7.5]),
            "impulse_zone": self.terrain_bounds.get("impulse_zone", [8.0, 9.0]),
            "impulse2_zone": self.terrain_bounds.get("impulse2_zone", [10.5, 11.0]),
            "decel_zone": self.terrain_bounds.get("decel_zone", [9.5, 11.0]),
            "brake_zone": self.terrain_bounds.get("brake_zone", [12.0, 15.0]),
            "cart_spawn_x": self.terrain_bounds.get("cart_spawn", (4.0, 2.5))[0],
            "cart_initial_speed": self.terrain_bounds.get("cart_initial_speed", 10.0),
        }
    def _check_design_constraints(self):
        violations = []
        if self.environment is None:
            return ["Environment not available"]
        min_beams = self.terrain_bounds.get("min_beam_count", 4)
        max_beams = self.terrain_bounds.get("max_beam_count", 5)
        num_beams = len(self.environment._bodies)
        if num_beams < min_beams or num_beams > max_beams:
            violations.append(f"Beam count {num_beams} is outside allowed range [{min_beams}, {max_beams}]")
        bz = self.terrain_bounds.get("build_zone", {})
        bx_min, bx_max = bz.get("x", [4.8, 9.0])
        by_min, by_max = bz.get("y", [2.0, 3.2])
        for body in self.environment._bodies:
            x, y = body.position.x, body.position.y
            if not (bx_min <= x <= bx_max and by_min <= y <= by_max):
                violations.append(f"Beam at ({x:.2f}, {y:.2f}) is outside build zone")
        max_mass = self.terrain_bounds.get("max_structure_mass", 14.0)
        mass = self.environment.get_structure_mass()
        if mass > max_mass:
            violations.append(f"Structure mass {mass:.2f} kg exceeds limit {max_mass} kg")
        return violations
    def compute_score_with_penalty(self, score: float, metrics: dict) -> float:
        if score > 0:
            return score
        constraint_info = self.get_constraint_info()
        penalty = compute_constraint_penalty("D_03", score, metrics, constraint_info)
        return penalty
    def get_constraint_info(self):
        return {
            'target_x_min': self._target_x_min,
            'target_speed_min': self._target_speed_min,
            'target_speed_max': self._target_speed_max,
            'speed_trap_x': self._speed_trap_x,
            'speed_trap_min': self._speed_trap_min,
            'checkpoint_11_x': self._checkpoint_11_x,
            'checkpoint_11_speed_min': self._checkpoint_11_speed_min,
            'checkpoint_11_speed_max': self._checkpoint_11_speed_max,
            'min_beam_count': self.terrain_bounds.get("min_beam_count", 4),
            'max_beam_count': self.terrain_bounds.get("max_beam_count", 5),
        }
    def get_task_description(self):
        return {
            "task": "D-03: Phase-Locked Gate",
            "success_criteria": {
                "primary": f"Pass gate and reach x >= {self._target_x_min}m",
                "secondary": f"Final speed in [{self._target_speed_min}, {self._target_speed_max}] m/s"
            }
        }
