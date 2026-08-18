from __future__ import annotations

import math


def _box_distance(x, y, x_min, x_max, y_min, y_max):
    dx = max(x_min - x, 0.0, x - x_max)
    dy = max(y_min - y, 0.0, y - y_max)
    return math.sqrt(dx * dx + dy * dy)


class Evaluator:
    def __init__(self, terrain_bounds, environment=None):
        self.terrain_bounds = terrain_bounds if isinstance(terrain_bounds, dict) else {}
        self.environment = environment
        self._configuration_error = None
        try:
            target = self.terrain_bounds["target_zone"]
            checkpoint_a = self.terrain_bounds["checkpoint_zone"]
            checkpoint_b = self.terrain_bounds["checkpoint_b_zone"]
            start = self.terrain_bounds["sled_start"]
            self.target_bounds = tuple(
                float(target[key]) for key in ("x_min", "x_max", "y_min", "y_max")
            )
            self.checkpoint_a_bounds = tuple(
                float(checkpoint_a[key])
                for key in ("x_min", "x_max", "y_min", "y_max")
            )
            self.checkpoint_b_bounds = tuple(
                float(checkpoint_b[key])
                for key in ("x_min", "x_max", "y_min", "y_max")
            )
            self.sled_start_x = float(start["x"])
            self.sled_start_y = float(start["y"])
        except (KeyError, TypeError, ValueError) as exc:
            self._configuration_error = f"Invalid terrain bounds: {exc}"
            self.target_bounds = (0.0, 0.0, 0.0, 0.0)
            self.checkpoint_a_bounds = (0.0, 0.0, 0.0, 0.0)
            self.checkpoint_b_bounds = (0.0, 0.0, 0.0, 0.0)
            self.sled_start_x = 0.0
            self.sled_start_y = 0.0

    def _terminal_error(self, step_count, max_steps, reason):
        return True, 0.0, {
            "success": False,
            "failed": True,
            "failure_reason": reason,
            "step_count": step_count,
            "max_steps": max_steps,
            "stop_reason": "evaluator_error",
            "numerical_health": {"all_finite": False, "non_finite_fields": []},
        }

    def evaluate(self, agent_body, step_count, max_steps):
        if self._configuration_error:
            return self._terminal_error(
                step_count, max_steps, self._configuration_error
            )
        if self.environment is None:
            return self._terminal_error(
                step_count, max_steps, "Environment not available"
            )
        pos = self.environment.get_sled_position()
        if pos is None:
            return self._terminal_error(step_count, max_steps, "Sled not found")

        x, y = float(pos[0]), float(pos[1])
        velocity = self.environment.get_sled_velocity() or (0.0, 0.0)
        vx, vy = float(velocity[0]), float(velocity[1])
        checkpoint_a = bool(self.environment.get_checkpoint_a_reached())
        checkpoint_b = bool(self.environment.get_checkpoint_b_reached())
        checkpoint_reached = bool(self.environment.get_checkpoint_reached())
        reached_target = bool(self.environment.get_target_reached())
        success = checkpoint_reached and reached_target
        failed = step_count >= max_steps and not success

        if failed:
            if not checkpoint_a:
                failure_reason = (
                    "Time limit reached before the sled entered checkpoint Alpha."
                )
            elif not checkpoint_b:
                failure_reason = (
                    "Time limit reached before the sled entered checkpoint Beta "
                    "after Alpha."
                )
            else:
                failure_reason = (
                    "Time limit reached before the sled entered the target after "
                    "both checkpoints."
                )
        else:
            failure_reason = None

        tx_min, tx_max, ty_min, ty_max = self.target_bounds
        distance_to_target = _box_distance(
            x, y, tx_min, tx_max, ty_min, ty_max
        )
        progress_span = tx_min - self.sled_start_x
        progress = (
            min(max((x - self.sled_start_x) / progress_span, 0.0), 1.0)
            if progress_span > 0.0
            else 0.0
        )
        if success:
            score = 100.0
        elif failed:
            score = 0.0
        else:
            score = (40.0 if checkpoint_reached else 0.0) + progress * 50.0

        ca_x_min, ca_x_max, ca_y_min, ca_y_max = self.checkpoint_a_bounds
        cb_x_min, cb_x_max, cb_y_min, cb_y_max = self.checkpoint_b_bounds
        finite_fields = {
            "sled_x": x,
            "sled_y": y,
            "velocity_x": vx,
            "velocity_y": vy,
            "distance_to_target": distance_to_target,
        }
        non_finite_fields = [
            name for name, value in finite_fields.items() if not math.isfinite(value)
        ]
        metrics = {
            "step_count": step_count,
            "max_steps": max_steps,
            "success": success,
            "failed": failed,
            "failure_reason": failure_reason,
            "stop_reason": (
                "success" if success else ("time_limit" if failed else "running")
            ),
            "checkpoint_reached": checkpoint_reached,
            "checkpoint_a_reached": checkpoint_a,
            "checkpoint_b_reached": checkpoint_b,
            "reached_target": reached_target,
            "sled_x": x,
            "sled_y": y,
            "sled_start_x": self.sled_start_x,
            "sled_start_y": self.sled_start_y,
            "velocity_x": vx,
            "velocity_y": vy,
            "velocity_magnitude": math.sqrt(vx * vx + vy * vy),
            "distance_to_target": distance_to_target,
            "progress_pct": progress * 100.0,
            "target_x_min": tx_min,
            "target_x_max": tx_max,
            "target_y_min": ty_min,
            "target_y_max": ty_max,
            "checkpoint_a_x_lo": ca_x_min,
            "checkpoint_a_x_hi": ca_x_max,
            "checkpoint_a_y_lo": ca_y_min,
            "checkpoint_a_y_hi": ca_y_max,
            "checkpoint_b_x_lo": cb_x_min,
            "checkpoint_b_x_hi": cb_x_max,
            "checkpoint_b_y_lo": cb_y_min,
            "checkpoint_b_y_hi": cb_y_max,
            "numerical_health": {
                "all_finite": not non_finite_fields,
                "non_finite_fields": non_finite_fields,
            },
        }

        zone_data = self.environment.get_zone_forensics()
        if isinstance(zone_data, dict):
            metrics["zone_forensics"] = zone_data
        thrust_data = self.environment.get_thrust_forensics()
        if isinstance(thrust_data, dict):
            metrics["thrust_forensics"] = {
                key: thrust_data.get(key)
                for key in (
                    "commanded_fx",
                    "commanded_fy",
                    "commanded_magnitude",
                    "peak_commanded_thrust",
                    "near_running_peak_command_steps",
                    "total_steps",
                )
            }
        stuck_data = self.environment.get_stuck_forensics()
        if isinstance(stuck_data, dict):
            metrics["stuck_forensics"] = stuck_data

        return success or failed, score, metrics

    def get_task_description(self):
        return {
            "task": "E-03: Slippery World",
            "description": (
                "Move the sled through checkpoint Alpha, checkpoint Beta, and "
                "the final target in that order."
            ),
            "terrain": self.terrain_bounds,
            "success_criteria": {
                "primary": (
                    "The sled center must enter Alpha, then Beta, then the target "
                    "within the simulation-step limit."
                ),
            },
            "evaluation": {
                "score_range": "0-100",
                "success_score": 100,
                "failure_score": 0,
            },
        }
