"""Evaluator for E-05 magnetic-field navigation."""

from __future__ import annotations

import math
from typing import Any, Dict


class Evaluator:
    UPPER_DIAGNOSTIC_BAND_Y = 9.7

    def __init__(self, terrain_bounds, environment=None):
        if environment is None:
            raise ValueError("Evaluator requires environment instance")
        self.terrain_bounds = terrain_bounds
        self.environment = environment
        target = terrain_bounds.get("target_zone", {})
        self.target_x_min = float(target.get("x_min", 28.0))
        self.target_x_max = float(target.get("x_max", 32.0))
        self.target_y_min = float(target.get("y_min", 6.0))
        self.target_y_max = float(target.get("y_max", 9.0))
        pit = terrain_bounds.get("pit_zone", {})
        self.pit_x_min = float(pit.get("x_min", 16.0))
        self.pit_x_max = float(pit.get("x_max", 24.0))
        self.pit_y_max = float(pit.get("y_max", 5.5))
        self.ground_y = float(terrain_bounds.get("ground_y", 1.0))
        start = terrain_bounds.get("body_start", {})
        self.body_start_x = float(start.get("x", 8.0))
        self.body_start_y = float(start.get("y", 5.0))
        self.reached_target = False
        self.first_target_entry_step = None

    @staticmethod
    def _interval_margin(value: float, lower: float, upper: float) -> float:
        return min(value - lower, upper - value)

    def _pit_margin(self, x: float, y: float) -> float:
        if self.pit_x_min <= x <= self.pit_x_max:
            return y - self.pit_y_max
        return min(abs(x - self.pit_x_min), abs(x - self.pit_x_max))

    def evaluate(self, agent_body, step_count, max_steps):
        del agent_body
        position = self.environment.get_body_position()
        if position is None:
            return True, 0.0, {
                "success": False,
                "failed": True,
                "failure_reason": "Controlled body is unavailable",
                "step_count": int(step_count),
                "max_steps": min(int(max_steps), int(self.environment.MAX_STEPS)),
            }

        x, y = map(float, position)
        in_target_x = self.target_x_min <= x <= self.target_x_max
        in_target_y = self.target_y_min <= y <= self.target_y_max
        if in_target_x and in_target_y:
            self.reached_target = True
            if self.first_target_entry_step is None:
                self.first_target_entry_step = int(step_count)

        in_pit = (
            self.pit_x_min <= x <= self.pit_x_max and y < self.pit_y_max
        )
        step_limit = min(int(max_steps), int(self.environment.MAX_STEPS))
        success = self.reached_target
        if in_pit and not success:
            failed = True
            failure_reason = "Body center entered the forbidden pit region"
        else:
            failed = step_count >= step_limit and not success
            failure_reason = (
                "Target zone was not reached before the simulation-step limit"
                if failed
                else None
            )

        maximum_distance = self.target_x_min - self.body_start_x
        progress_x = (
            max(0.0, min(1.0, (x - self.body_start_x) / maximum_distance))
            if maximum_distance > 0.0
            else 0.0
        )
        if success:
            score = 100.0
        elif failed:
            score = 0.0
        else:
            score = progress_x * 80.0

        vx, vy = map(float, self.environment.get_body_velocity() or (0.0, 0.0))
        speed = math.hypot(vx, vy)
        closest_x = min(max(x, self.target_x_min), self.target_x_max)
        closest_y = min(max(y, self.target_y_min), self.target_y_max)
        distance_to_target = math.hypot(x - closest_x, y - closest_y)

        summary = self.environment._get_forensic_summary()
        physics = self.environment.get_physics_params()
        magnetic = self.environment._get_magnetic_force_summary()
        energy = self.environment._get_energy_summary()
        forces = self.environment._get_force_decomposition()
        plateau = self.environment._get_progress_plateau_info()
        temporal_events = self.environment._get_temporal_events()
        reversals = self.environment._get_velocity_reversals()

        metrics: Dict[str, Any] = {
            "step_count": int(step_count),
            "max_steps": step_limit,
            "success": bool(success),
            "failed": bool(failed),
            "failure_reason": failure_reason,
            "body_x": x,
            "body_y": y,
            "target_x_min": self.target_x_min,
            "target_x_max": self.target_x_max,
            "target_y_min": self.target_y_min,
            "target_y_max": self.target_y_max,
            "target_x_margin": self._interval_margin(
                x, self.target_x_min, self.target_x_max
            ),
            "target_y_margin": self._interval_margin(
                y, self.target_y_min, self.target_y_max
            ),
            "reached_target": self.reached_target,
            "first_target_entry_step": self.first_target_entry_step,
            "velocity_x": vx,
            "velocity_y": vy,
            "speed": speed,
            "progress_x": progress_x,
            "dist_to_target": distance_to_target,
            "in_target_x": in_target_x,
            "in_target_y": in_target_y,
            "start_x": self.body_start_x,
            "start_y": self.body_start_y,
            "ceiling_clearance": self.UPPER_DIAGNOSTIC_BAND_Y - y,
            "ground_clearance": y - self.ground_y,
            "hx_remaining_to_target": max(self.target_x_min - x, 0.0),
            "in_pit_zone": in_pit,
            "pit_zone_margin": self._pit_margin(x, y),
            "pit_x_min": self.pit_x_min,
            "pit_x_max": self.pit_x_max,
            "pit_y_max": self.pit_y_max,
            # Kept as compatibility aliases; this is a diagnostic field band,
            # not an evaluator-enforced ceiling.
            "ceiling_y": self.UPPER_DIAGNOSTIC_BAND_Y,
            "upper_diagnostic_band_y": self.UPPER_DIAGNOSTIC_BAND_Y,
            "ground_y": self.ground_y,
            "in_ceiling_zone": y > self.UPPER_DIAGNOSTIC_BAND_Y,
            "max_body_x": max(
                value
                for value in (summary.get("max_body_x"), x)
                if value is not None
            ),
            "min_body_x": min(
                value
                for value in (summary.get("min_body_x"), x)
                if value is not None
            ),
            "max_body_y": max(
                value
                for value in (summary.get("max_body_y"), y)
                if value is not None
            ),
            "min_body_y": min(
                value
                for value in (summary.get("min_body_y"), y)
                if value is not None
            ),
            "max_speed": summary.get("max_speed"),
            "max_x_reached": max(
                value
                for value in (summary.get("max_x_reached"), x)
                if value is not None
            ),
            "total_dx": summary.get("total_dx"),
            "total_dy": summary.get("total_dy"),
            "steps_near_ceiling": summary.get("steps_near_ceiling", 0),
            "steps_in_pit_zone": summary.get("steps_in_pit_zone", 0),
            "steps_in_ground_zone": summary.get("steps_in_ground_zone", 0),
            "steps_stationary": summary.get("steps_stationary", 0),
            "first_ceiling_entry_step": summary.get("first_ceiling_entry_step"),
            "first_pit_entry_step": summary.get("first_pit_entry_step"),
            "first_ground_entry_step": summary.get("first_ground_entry_step"),
            "vertical_zone_samples": summary.get("vertical_zone_samples", {}),
            "temporal_events": temporal_events,
            "velocity_reversal_events": reversals,
            "velocity_reversal_count_x": sum(
                event.get("axis") == "x"
                for event in reversals
                if isinstance(event, dict)
            ),
            "velocity_reversal_count_y": sum(
                event.get("axis") == "y"
                for event in reversals
                if isinstance(event, dict)
            ),
            "peak_vertical_accel": self.environment._get_peak_vertical_acceleration(),
            "gravity_y": physics.get("gravity_y"),
            "linear_damping": physics.get("linear_damping"),
            "max_thrust": physics.get("max_thrust"),
            "magnet_count": physics.get("magnet_count"),
            **magnetic,
            **energy,
            **forces,
            **plateau,
        }
        metrics["net_magnetic_force_x_terminal"] = metrics.get(
            "net_magnetic_force_x"
        )
        metrics["thrust_applied_x_terminal"] = metrics.get("thrust_applied_x")
        return success or failed, score, metrics

    def get_task_description(self):
        return {
            "task": "E-05: Magnetic Navigation",
            "description": (
                "Guide the controlled body into the target zone without entering "
                "the forbidden pit region."
            ),
            "terrain": self.terrain_bounds,
            "success_criteria": {
                "primary": (
                    f"Body center enters x=[{self.target_x_min}, "
                    f"{self.target_x_max}], y=[{self.target_y_min}, "
                    f"{self.target_y_max}]"
                ),
                "failure": (
                    f"Before success, body center must not enter "
                    f"x=[{self.pit_x_min}, {self.pit_x_max}] with "
                    f"y<{self.pit_y_max}"
                ),
            },
            "evaluation": {
                "score_range": "0-100",
                "success_score": 100,
                "failure_score": 0,
            },
        }
