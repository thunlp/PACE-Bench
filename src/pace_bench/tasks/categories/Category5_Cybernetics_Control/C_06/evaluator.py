import math

from pace_bench.core.primitives import compute_constraint_penalty

from environment import (
    MEAN_SPEED_ERROR_THRESHOLD,
    REGULATION_START_STEP,
    STALL_SPEED_THRESHOLD,
    STALL_STEPS_THRESHOLD,
    TARGET_SPEED_RAD_S,

)

def _stall_steps_from_bounds(terrain_bounds):
    return int(terrain_bounds.get("stall_steps_threshold", STALL_STEPS_THRESHOLD))

class Evaluator:
    def __init__(self, terrain_bounds, environment=None):
        self.terrain_bounds = terrain_bounds
        self.environment = environment
        self._target_speed_time_varying = bool(terrain_bounds.get("target_speed_time_varying", False))
        self._regulation_start = int(terrain_bounds.get("regulation_start_step", REGULATION_START_STEP))
        self._stall_threshold = float(terrain_bounds.get("stall_speed_threshold", STALL_SPEED_THRESHOLD))
        self._stall_steps_threshold = _stall_steps_from_bounds(terrain_bounds)
        self._mean_speed_error_threshold = float(terrain_bounds.get("mean_speed_error_threshold", MEAN_SPEED_ERROR_THRESHOLD))
        self._stall_count = 0
        self._maximum_stall_count = 0
        self._first_stall_step = None
        self._speed_error_sum = 0.0
        self._speed_error_count = 0
        self._peak_reported_speed_error = 0.0
        self._last_target = None
        self._target_change_events = []
    def evaluate(self, agent_body, step_count, max_steps):
        if not self.environment:
            return True, 0.0, {"error": "Environment not available"}
        omega_true = self.environment._get_wheel_angular_velocity_actual()
        omega_reported = self.environment.get_wheel_angular_velocity()
        target = self.environment.get_target_speed()
        scoring_speed_error = abs(omega_true - target)
        reported_speed_error = abs(omega_reported - target)
        self._peak_reported_speed_error = max(
            self._peak_reported_speed_error, reported_speed_error
        )
        if self._last_target is None:
            self._last_target = target
        elif not math.isclose(target, self._last_target, rel_tol=0.0, abs_tol=1e-12):
            self._target_change_events.append(
                {
                    "step": int(step_count),
                    "from": float(self._last_target),
                    "to": float(target),
                }
            )
            self._last_target = target
        if step_count >= self._regulation_start:
            self._speed_error_sum += scoring_speed_error
            self._speed_error_count += 1
        if abs(omega_true) < self._stall_threshold:
            self._stall_count += 1
            if self._first_stall_step is None:
                self._first_stall_step = int(step_count)
        else:
            self._stall_count = 0
        self._maximum_stall_count = max(self._maximum_stall_count, self._stall_count)
        failed = False
        failure_reason = None
        if self._stall_count >= self._stall_steps_threshold:
            failed = True
            failure_reason = (
                f"Wheel stalled: |ω| below {self._stall_threshold} rad/s for "
                f"{self._stall_steps_threshold} consecutive steps"
            )
        mean_speed_error = (
            self._speed_error_sum / self._speed_error_count
            if self._speed_error_count > 0
            else 0.0
        )
        if (step_count >= max_steps - 1) and not failed:
            if max_steps <= self._regulation_start:
                failed = True
                failure_reason = (
                    f"Simulation horizon ({max_steps} steps) does not exceed "
                    f"regulation start step ({self._regulation_start}); "
                    f"regulation phase was never reached."
                )
            elif self._speed_error_count == 0:
                failed = True
                failure_reason = (
                    "No regulation-phase speed samples were collected."
                )
            elif mean_speed_error > self._mean_speed_error_threshold:
                failed = True
                failure_reason = (
                    f"Mean speed tracking error {mean_speed_error:.4f} rad/s exceeds threshold "
                    f"{self._mean_speed_error_threshold:.4f} rad/s"
                )
        success = (step_count >= max_steps - 1) and not failed
        if success and not failed:
            score = 100.0
        elif failed:
            score = 0.0
        else:
            progress = step_count / max_steps if max_steps > 0 else 0.0
            score = progress * 80.0
        metrics = {
            "wheel_angular_velocity": omega_reported,
            "target_speed": target,
            "speed_error": reported_speed_error,
            "reported_speed_error": reported_speed_error,
            "mean_speed_error": mean_speed_error,
            "peak_reported_speed_error": self._peak_reported_speed_error,
            "stall_count": self._stall_count,
            "maximum_stall_count": self._maximum_stall_count,
            "first_stall_step": self._first_stall_step,
            "stall_speed_threshold": self._stall_threshold,
            "stall_steps_threshold": self._stall_steps_threshold,
            "mean_speed_error_threshold": self._mean_speed_error_threshold,
            "regulation_start_step": self._regulation_start,
            "step_count": step_count,
            "max_steps": max_steps,
            "success": success,
            "failed": failed,
            "failure_reason": failure_reason,
            "target_change_events": list(self._target_change_events),
        }
        metrics["commanded_torque"] = self.environment._get_last_commanded_torque()
        done = failed or (step_count >= max_steps - 1)
        return done, score, metrics
    def compute_score_with_penalty(self, score: float, metrics: dict) -> float:
        if score > 0:
            return score
        constraint_info = self.get_constraint_info()
        penalty = compute_constraint_penalty("C_06", score, metrics, constraint_info)
        return penalty
    def get_constraint_info(self):
        return {
            'target_speed_rad_s': float(self.terrain_bounds.get("target_speed_rad_s", TARGET_SPEED_RAD_S)),
            'target_speed_time_varying': self._target_speed_time_varying,
            'regulation_start': self._regulation_start,
            'stall_threshold': self._stall_threshold,
            'stall_steps_threshold': self._stall_steps_threshold,
            'mean_speed_error_threshold': self._mean_speed_error_threshold,
        }
    def get_task_description(self):
        initial_target = float(self.terrain_bounds.get("target_speed_rad_s", TARGET_SPEED_RAD_S))
        return {
            "task": "C-06: The Governor",
            "description": "Maintain wheel speed at the commanded target under load (load may vary with speed and time).",
            "target_speed_rad_s": initial_target,
            "target_speed_time_varying": self._target_speed_time_varying,
            "stall_speed_threshold": self._stall_threshold,
            "stall_steps_threshold": self._stall_steps_threshold,
            "mean_speed_error_threshold_rad_s": self._mean_speed_error_threshold,
            "regulation_start_step": self._regulation_start,
            "success_criteria": {
                "primary": (
                    f"Through the episode after regulation start, keep mean |ω−target| below "
                    f"{self._mean_speed_error_threshold} rad/s and avoid prolonged stall"
                ),
                "failure": (
                    f"Mean |ω−target| exceeds {self._mean_speed_error_threshold} rad/s over "
                    f"the regulation phase, or wheel stalls (|ω| below "
                    f"{self._stall_threshold} rad/s for {self._stall_steps_threshold} consecutive steps)."
                ),
            },
            "evaluation": {
                "score_range": "0-100",
                "success_score": 100,
                "failure_score": 0,
            },
        }
