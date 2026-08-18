from pace_bench.core.primitives import compute_constraint_penalty

from pace_bench.tasks.categories.Category5_Cybernetics_Control.C_04.environment import (
    ACTIVATION_X_MAX,
    ACTIVATION_X_MIN,
    MAX_STEPS as TASK_MAX_STEPS,
    EXIT_X_MIN,
    EXIT_Y_MIN,
    EXIT_Y_MAX,
    HOLD_STEPS,

)

from pace_bench.tasks.categories.Category5_Cybernetics_Control.C_04 import prompt as c04_prompt

from pace_bench.tasks.categories.Category5_Cybernetics_Control.C_04 import stages as c04_stages

CONSECUTIVE_EXIT_STEPS_REQUIRED = HOLD_STEPS

def _exit_hold_steps_required(environment) -> int:
    if environment is not None and hasattr(environment, "_backward_steps_required"):
        return int(getattr(environment, "_backward_steps_required"))
    return int(HOLD_STEPS)

def _environment_max_steps(environment) -> int:
    if environment is not None and hasattr(environment, "MAX_STEPS"):
        return int(environment.MAX_STEPS)
    return int(TASK_MAX_STEPS)

def _activation_bounds(environment):
    if environment is not None and hasattr(environment, "_activation_x_min"):
        return float(environment._activation_x_min), float(environment._activation_x_max)
    return float(ACTIVATION_X_MIN), float(ACTIVATION_X_MAX)

class Evaluator:
    def __init__(self, terrain_bounds, environment=None, task_description=None, **kwargs):
        self.terrain_bounds = terrain_bounds
        self.environment = environment
        self._exit_x_min = float(terrain_bounds.get("exit_x_min", EXIT_X_MIN))
        self._exit_y_min = float(terrain_bounds.get("exit_y_min", EXIT_Y_MIN))
        self._exit_y_max = float(terrain_bounds.get("exit_y_max", EXIT_Y_MAX))
        self._task_description_override = task_description
        if self._task_description_override is None and self.environment is not None:
            if hasattr(self.environment, "physics_config"):
                self._task_description_override = self.environment.physics_config.get("task_description")
    def evaluate(self, agent_body, step_count, max_steps):
        if not self.environment:
            return True, 0.0, {
                "error": "Environment not available",
                "success": False,
                "failed": True,
                "configuration_error": True,
                "failure_reason": "Evaluator requires an environment instance",
                "stop_reason": "evaluator_missing_environment",
            }
        reached_exit = self.environment.has_reached_exit()
        unlocked = bool(self.environment.get_metrics().get("unlocked", False))
        dwell_status = self.environment.get_exit_dwell_status()
        x, y = self.environment.get_agent_position()
        vx, vy = self.environment.get_agent_velocity()
        whisker = self.environment.get_whisker_readings()
        exit_hold_need = int(dwell_status["required_steps"])
        consecutive_in_exit = int(dwell_status["consecutive_steps"])
        max_consecutive_in_exit = int(dwell_status["max_consecutive_steps"])
        success = unlocked and max_consecutive_in_exit >= exit_hold_need
        failed = False
        failure_reason = None
        if self.environment and self.environment.is_destroyed():
            failed = True
            failure_reason = self.environment.get_destruction_reason()
        elif max_steps > 0 and step_count >= max_steps and not success:
            failed = True
            if not unlocked:
                failure_reason = "Time limit reached before unlocking the exit gate"
            else:
                failure_reason = "Time limit reached before satisfying exit hold requirement"
        progress_x = (x / self._exit_x_min) if self._exit_x_min > 0 else 0.0
        if success:
            score = 100.0
        elif failed:
            score = 0.0
        else:
            score = min(80.0, progress_x * 80.0)
        distance_to_exit_x = max(0.0, self._exit_x_min - x)
        distance_y_to_band = 0.0
        if y < self._exit_y_min:
            distance_y_to_band = self._exit_y_min - y
        elif y > self._exit_y_max:
            distance_y_to_band = y - self._exit_y_max
        act_lo, act_hi = _activation_bounds(self.environment)
        metrics = {
            "agent_x": x,
            "agent_y": y,
            "agent_vx": vx,
            "agent_vy": vy,
            "whisker_front": whisker[0] if len(whisker) > 0 else 0.0,
            "whisker_up": whisker[1] if len(whisker) > 1 else 0.0,
            "whisker_down": whisker[2] if len(whisker) > 2 else 0.0,
            "unlocked": unlocked,
            "reached_exit": reached_exit,
            "consecutive_steps_in_exit": consecutive_in_exit,
            "max_consecutive_steps_in_exit": max_consecutive_in_exit,
            "first_qualified_exit_step": dwell_status["first_qualified_step"],
            "exit_hold_completion_step": dwell_status["completion_step"],
            "step_count": step_count,
            "success": success,
            "failed": failed,
            "failure_reason": failure_reason,
            "distance_to_exit_x": distance_to_exit_x,
            "progress_x_pct": min(100.0, progress_x * 100.0),
            "distance_y_to_exit_band": distance_y_to_band,
            "exit_x_min": self._exit_x_min,
            "exit_y_min": self._exit_y_min,
            "exit_y_max": self._exit_y_max,
            "activation_x_min": act_lo,
            "activation_x_max": act_hi,
            "consecutive_exit_steps_required": exit_hold_need,
            "max_steps": int(max_steps),
            "stop_reason": (
                "success" if success else "failure" if failed else "running"
            ),
        }
        if self.environment is not None:
            metrics["force_ledger"] = self.environment.get_force_ledger()
            metrics["unlock_condition_status"] = self.environment.get_unlock_condition_status()
            metrics["wall_clearance_map"] = self.environment.get_wall_clearance_map()
            metrics["control_lag_info"] = self.environment.get_control_lag_info()
            metrics["diagnostic_timeline"] = self.environment.get_diagnostic_timeline()
            metrics["whisker_max_range"] = self.environment.get_whisker_max_range()
            metrics["structural_impulse_limit_ns"] = self.environment.get_structural_impulse_limit()
            metrics["agent_destroyed"] = self.environment.is_destroyed()
        done = success or failed
        return done, score, metrics
    def compute_score_with_penalty(self, score: float, metrics: dict) -> float:
        if score > 0:
            return score
        constraint_info = self.get_constraint_info()
        penalty = compute_constraint_penalty("C_04", score, metrics, constraint_info)
        return penalty
    def get_constraint_info(self):
        return {
            'exit_x_min': self._exit_x_min,
            'exit_y_min': self._exit_y_min,
            'exit_y_max': self._exit_y_max,
            'structural_impulse_limit_ns': (
                self.environment.get_structural_impulse_limit()
                if self.environment is not None else None
            ),
            'consecutive_exit_steps_required': _exit_hold_steps_required(self.environment),
            'max_steps': _environment_max_steps(self.environment),
        }
    def get_task_description(self):
        max_steps_meta = _environment_max_steps(self.environment)
        tc: dict = {}
        pc: dict = {}
        if self.environment is not None:
            tc = dict(getattr(self.environment, "terrain_config", None) or {})
            pc = dict(getattr(self.environment, "physics_config", None) or {})
            pc.pop("task_description", None)
        base_physics = c04_stages.get_source_base_physics_config()
        base_terrain = c04_stages.get_source_base_terrain_config()
        if self._task_description_override is not None:
            desc = self._task_description_override
        else:
            base_desc = c04_prompt.TASK_PROMPT["task_description"]
            desc = c04_stages.update_task_description_for_visible_changes(
                base_desc, tc, base_terrain, pc, base_physics
            )
        base_success = c04_prompt.TASK_PROMPT["success_criteria"]
        success_markdown = c04_stages.update_success_criteria_for_visible_changes(
            base_success, tc, base_terrain, pc, base_physics
        )
        hold_steps_meta = _exit_hold_steps_required(self.environment)
        return {
            "task": "C-04: The Escaper",
            "description": desc,
            "exit_x_min": self._exit_x_min,
            "exit_y_min": self._exit_y_min,
            "exit_y_max": self._exit_y_max,
            "time_limit_steps": max_steps_meta,
            "success_criteria": {
                "primary": (
                    f"Unlock, enter x >= {self._exit_x_min:.1f} m and "
                    f"y in [{self._exit_y_min:.1f}, {self._exit_y_max:.1f}] m, "
                    f"then remain for {hold_steps_meta} consecutive simulation steps."
                ),
                "failure": "Structural destruction or expiration of the step limit.",
                "detail_markdown": success_markdown.strip(),
            },
            "evaluation": {
                "score_range": "0-100",
                "success_score": 100,
                "failure_score": 0,
            },
        }
