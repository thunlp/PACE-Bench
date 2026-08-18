import sys

import os

from pace_bench.core.primitives import compute_constraint_penalty

class Evaluator:
    def __init__(self, terrain_bounds, environment=None):
        self.terrain_bounds = terrain_bounds
        self.environment = environment
        self._design_constraints_checked = False
        if environment is None:
            raise ValueError("Evaluator requires environment instance")
        self.MAX_STRUCTURE_MASS = getattr(environment, "MAX_STRUCTURE_MASS", 70.0)
        self.BUILD_ZONE_X_MIN = environment.BUILD_ZONE_X_MIN
        self.BUILD_ZONE_X_MAX = environment.BUILD_ZONE_X_MAX
        self.BUILD_ZONE_Y_MIN = environment.BUILD_ZONE_Y_MIN
        self.BUILD_ZONE_Y_MAX = environment.BUILD_ZONE_Y_MAX
    def evaluate(self, agent_body, step_count, max_steps):
        if self.environment is None:
            return True, 0.0, {"error": "Environment not available"}
        if not self._design_constraints_checked and step_count == 0:
            violations = self._check_design_constraints()
            if violations:
                self._design_constraints_checked = True
                metrics = self._make_metrics(step_count, False, True,
                    "Design constraint violated: " + "; ".join(violations), agent_body, max_steps)
                return True, 0.0, metrics
            self._design_constraints_checked = True
        shell_broken = self.environment.is_shell_broken()
        hit_pendulum = getattr(self.environment, "hammer_hit_pendulum_before_shell", lambda: False)
        hit_pendulum = hit_pendulum() if callable(hit_pendulum) else hit_pendulum
        hit_gate = getattr(self.environment, "hammer_hit_gate_before_shell", lambda: False)
        hit_gate = hit_gate() if callable(hit_gate) else hit_gate
        hit_gate2 = getattr(self.environment, "hammer_hit_gate2_before_shell", lambda: False)
        hit_gate2 = hit_gate2() if callable(hit_gate2) else hit_gate2
        hit_wall = getattr(self.environment, "hammer_hit_wall_before_shell", lambda: False)
        hit_wall = hit_wall() if callable(hit_wall) else hit_wall
        hit_slot_wall = getattr(self.environment, "hammer_hit_slot_wall_before_shell", lambda: False)
        hit_slot_wall = hit_slot_wall() if callable(hit_slot_wall) else hit_slot_wall
        hit_slot_bar = getattr(self.environment, "hammer_hit_slot_bar_before_shell", lambda: False)
        hit_slot_bar = hit_slot_bar() if callable(hit_slot_bar) else hit_slot_bar
        success = shell_broken and not hit_pendulum and not hit_gate and not hit_gate2 and not hit_wall and not hit_slot_wall and not hit_slot_bar
        failed = False
        failure_reason = None
        if hit_slot_bar:
            failed = True
            failure_reason = "Hammer contacted the oscillating bar before shell breakage"
        elif hit_slot_wall:
            failed = True
            failure_reason = "Hammer contacted a slot wall before shell breakage"
        elif hit_pendulum:
            failed = True
            has_second_pendulum = "pendulum_rod_2" in self.environment._terrain_bodies
            failure_reason = "Hammer contacted the pendulum before shell breakage"
        elif hit_gate:
            failed = True
            has_second_gate = "gate2" in self.environment._terrain_bodies
            failure_reason = "Hammer contacted the first gate before shell breakage"
        elif hit_gate2:
            failed = True
            failure_reason = "Hammer contacted the second gate before shell breakage"
        elif hit_wall:
            failed = True
            failure_reason = "Hammer contacted the central wall before shell breakage"
        elif step_count >= max_steps - 1 and not shell_broken:
            failed = True
            failure_reason = "Shell remained intact at the simulation step limit"
        done = failed or success or step_count >= max_steps - 1
        score = 100.0 if success else (0.0 if failed else 0.0)
        metrics = self._make_metrics(step_count, success, failed, failure_reason, agent_body, max_steps)
        return done, score, metrics
    def _check_design_constraints(self):
        violations = []
        if self.environment is None:
            return ["Environment not available"]
        mass = self.environment.get_structure_mass()
        if mass >= self.MAX_STRUCTURE_MASS:
            violations.append(f"Structure mass {mass:.2f} kg must be strictly less than {self.MAX_STRUCTURE_MASS} kg")
        for body in self.environment._bodies:
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
        self, step_count, success=False, failed=False, failure_reason=None,
        agent_body=None, max_steps=None,
    ):
        hit_pendulum = getattr(self.environment, "hammer_hit_pendulum_before_shell", lambda: False)
        hit_pendulum = hit_pendulum() if callable(hit_pendulum) else hit_pendulum
        hit_gate = getattr(self.environment, "hammer_hit_gate_before_shell", lambda: False)
        hit_gate = hit_gate() if callable(hit_gate) else hit_gate
        hit_gate2 = getattr(self.environment, "hammer_hit_gate2_before_shell", lambda: False)
        hit_gate2 = hit_gate2() if callable(hit_gate2) else hit_gate2
        hit_wall = getattr(self.environment, "hammer_hit_wall_before_shell", lambda: False)
        hit_wall = hit_wall() if callable(hit_wall) else hit_wall
        hit_slot_wall = getattr(self.environment, "hammer_hit_slot_wall_before_shell", lambda: False)
        hit_slot_wall = hit_slot_wall() if callable(hit_slot_wall) else hit_slot_wall
        hit_slot_bar = getattr(self.environment, "hammer_hit_slot_bar_before_shell", lambda: False)
        hit_slot_bar = hit_slot_bar() if callable(hit_slot_bar) else hit_slot_bar
        metrics = {
            "success": success,
            "failed": failed,
            "failure_reason": failure_reason,
            "step_count": step_count,
            "max_steps": max_steps,
            "structure_mass": self.environment.get_structure_mass(),
            "max_structure_mass": self.MAX_STRUCTURE_MASS,
            "shell_broken": self.environment.is_shell_broken(),
            "hammer_hit_pendulum": hit_pendulum,
            "hammer_hit_gate": hit_gate,
            "hammer_hit_gate2": hit_gate2,
            "hammer_hit_wall": hit_wall,
            "hammer_hit_slot_wall": hit_slot_wall,
            "hammer_hit_slot_bar": hit_slot_bar,
            "build_zone_x_min": self.BUILD_ZONE_X_MIN,
            "build_zone_x_max": self.BUILD_ZONE_X_MAX,
            "build_zone_y_min": self.BUILD_ZONE_Y_MIN,
            "build_zone_y_max": self.BUILD_ZONE_Y_MAX,
            "peak_kinetic_energy": getattr(self.environment, '_peak_ke', 0.0),
            "peak_ke_step": getattr(self.environment, '_peak_ke_step', 0),
            "peak_speed": getattr(self.environment, '_peak_speed', 0.0),
            "max_shell_joint_force": getattr(self.environment, '_max_shell_joint_force', 0.0),
            "contact_events": getattr(self.environment, 'get_contact_events', lambda: [])() if hasattr(self.environment, 'get_contact_events') else [],
            "observation_errors": getattr(
                self.environment, 'get_observation_errors', lambda: []
            )(),
        }
        if hasattr(self.environment, 'get_slot_entry_info'):
            slot_info = self.environment.get_slot_entry_info()
            metrics["slot_entry_step"] = slot_info.get('step')
            metrics["slot_entry_hammer_y"] = slot_info.get('hammer_y')
            metrics["slot_entry_bar_y"] = slot_info.get('bar_y')
        if hasattr(self.environment, 'get_slot_gap_bounds'):
            gap_low, gap_high = self.environment.get_slot_gap_bounds()
            metrics["slot_gap_y_low"] = gap_low
            metrics["slot_gap_y_high"] = gap_high
        pendulum_rod = self.environment._terrain_bodies.get("pendulum_rod")
        if pendulum_rod is not None:
            metrics["pendulum_x"] = float(pendulum_rod.position.x)
            metrics["pendulum_y"] = float(pendulum_rod.position.y)
            metrics["pendulum_angle"] = float(pendulum_rod.angle)
            metrics["pendulum_angular_velocity"] = float(pendulum_rod.angularVelocity)
        body_positions = []
        for body in self.environment._bodies:
            body_positions.append({
                'x': float(body.position.x),
                'y': float(body.position.y),
            })
        metrics["agent_body_positions"] = body_positions
        tb = self.terrain_bounds if hasattr(self, "terrain_bounds") else {}
        if tb:
            metrics["shell_x"] = tb.get("shell_x", 16.0)
            metrics["shell_y"] = tb.get("shell_y", 2.6)
            metrics["shell_break_force"] = tb.get("shell_break_force", 5000.0)
            if "pendulum_pivot" in tb:
                metrics["pendulum_pivot"] = tb["pendulum_pivot"]
                metrics["pendulum_rod_length"] = tb.get("pendulum_rod_length", 3.5)
            if "shield_has_window" in tb:
                metrics["shield_has_window"] = tb["shield_has_window"]
            if tb.get("central_wall"):
                metrics["central_wall"] = True
        if agent_body is not None:
            metrics["hammer_x"] = float(agent_body.position.x)
            metrics["hammer_y"] = float(agent_body.position.y)
            vx = float(agent_body.linearVelocity.x)
            vy = float(agent_body.linearVelocity.y)
            metrics["velocity_x"] = vx
            metrics["velocity_y"] = vy
            speed = (vx**2 + vy**2) ** 0.5
            metrics["speed"] = speed
            metrics["angular_velocity"] = float(agent_body.angularVelocity)
            metrics["kinetic_energy"] = 0.5 * agent_body.mass * (speed**2) + 0.5 * agent_body.inertia * (metrics["angular_velocity"]**2)
        return metrics
    def compute_score_with_penalty(self, score: float, metrics: dict) -> float:
        if score > 0:
            return score
        constraint_info = self.get_constraint_info()
        penalty = compute_constraint_penalty("D_05", score, metrics, constraint_info)
        return penalty
    def get_constraint_info(self):
        return {
            'max_structure_mass': self.MAX_STRUCTURE_MASS,
            'build_zone_x_min': self.BUILD_ZONE_X_MIN,
            'build_zone_x_max': self.BUILD_ZONE_X_MAX,
            'build_zone_y_min': self.BUILD_ZONE_Y_MIN,
            'build_zone_y_max': self.BUILD_ZONE_Y_MAX,
        }
    def get_task_description(self):
        return {
            "task": "D-05: The Hammer",
            "description": "Design a hammer to break the hard shell with a large instantaneous force",
            "success_criteria": {"primary": "Shell is broken", "failure": "Shell not broken"},
            "evaluation": {"score_range": "0-100", "success_score": 100, "failure_score": 0},
        }
