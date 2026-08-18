import math


class Evaluator:
    def __init__(self, terrain_bounds, environment=None):
        if environment is None:
            raise ValueError("Evaluator requires environment instance")
        self.terrain_bounds = terrain_bounds
        self.environment = environment
        target = terrain_bounds.get("target_zone", {})
        self.target_x_min = float(target.get("x_min", 28.0))
        self.target_x_max = float(target.get("x_max", 32.0))
        self.target_y_min = float(target.get("y_min", 2.0))
        self.target_y_max = float(target.get("y_max", 5.0))
        start = terrain_bounds.get("craft_start", {})
        self.craft_start_x = float(start.get("x", 8.0))
        self.craft_start_y = float(start.get("y", 2.0))
        self.overheat_limit = float(environment.get_overheat_limit())
        self.reached_target = False
        self.first_target_step = None
        self.first_overheat_step = None
        self.closest_target_gap = math.inf
        self.closest_target_step = None
        self.peak_speed = 0.0
        self.peak_heat = 0.0

    def _target_gap(self, x, y):
        gap_x = max(self.target_x_min - x, 0.0, x - self.target_x_max)
        gap_y = max(self.target_y_min - y, 0.0, y - self.target_y_max)
        return gap_x, gap_y, math.hypot(gap_x, gap_y)

    def evaluate(self, agent_body, step_count, max_steps):
        del agent_body
        position = self.environment.get_craft_position()
        velocity = self.environment.get_craft_velocity()
        heat = float(self.environment.get_heat())
        overheated = bool(self.environment.is_overheated())

        if position is None or velocity is None:
            return True, 0.0, {
                "success": False,
                "failed": True,
                "failure_reason": "Craft state is unavailable",
                "step_count": step_count,
                "max_steps": max_steps,
            }

        x, y = map(float, position)
        vx, vy = map(float, velocity)
        state_values = (x, y, vx, vy, heat, self.overheat_limit)
        numerical_failure = not all(math.isfinite(value) for value in state_values)
        speed = math.hypot(vx, vy) if not numerical_failure else math.nan
        gap_x, gap_y, target_gap = (
            self._target_gap(x, y)
            if math.isfinite(x) and math.isfinite(y)
            else (math.nan, math.nan, math.nan)
        )

        if math.isfinite(target_gap) and target_gap < self.closest_target_gap:
            self.closest_target_gap = target_gap
            self.closest_target_step = step_count
        if math.isfinite(speed):
            self.peak_speed = max(self.peak_speed, speed)
        if math.isfinite(heat):
            self.peak_heat = max(self.peak_heat, heat)

        inside_target = target_gap == 0.0
        if inside_target and not self.reached_target:
            self.reached_target = True
            self.first_target_step = step_count
        if overheated and self.first_overheat_step is None:
            self.first_overheat_step = step_count

        time_exhausted = step_count >= max_steps - 1
        failed = numerical_failure or overheated or (
            time_exhausted and not self.reached_target
        )
        success = self.reached_target and not overheated and not numerical_failure

        if numerical_failure:
            failure_reason = "Non-finite craft state detected"
        elif overheated:
            failure_reason = (
                "Thermal limit reached before evaluation completed "
                f"({heat:.1f}/{self.overheat_limit:.1f} N·s)"
            )
        elif time_exhausted and not self.reached_target:
            failure_reason = "Target zone was not reached before the step budget expired"
        else:
            failure_reason = None

        if success:
            score = 100.0
        elif failed:
            score = 0.0
        else:
            initial_gap = max(self.target_x_min - self.craft_start_x, 1.0)
            score = 80.0 * min(max(1.0 - target_gap / initial_gap, 0.0), 1.0)

        step_budget_remaining = max(0, max_steps - step_count)
        metrics = {
            "step_count": step_count,
            "max_steps": max_steps,
            "step_budget_remaining": step_budget_remaining,
            "success": success,
            "failed": failed,
            "failure_reason": failure_reason,
            "craft_x": x,
            "craft_y": y,
            "velocity_x": vx,
            "velocity_y": vy,
            "speed": speed,
            "peak_speed": self.peak_speed,
            "target_x_min": self.target_x_min,
            "target_x_max": self.target_x_max,
            "target_y_min": self.target_y_min,
            "target_y_max": self.target_y_max,
            "target_gap_x": gap_x,
            "target_gap_y": gap_y,
            "target_gap": target_gap,
            "closest_target_gap": (
                self.closest_target_gap
                if math.isfinite(self.closest_target_gap)
                else None
            ),
            "closest_target_step": self.closest_target_step,
            "reached_target": self.reached_target,
            "first_target_step": self.first_target_step,
            "heat": heat,
            "peak_heat": self.peak_heat,
            "overheated": overheated,
            "first_overheat_step": self.first_overheat_step,
            "overheat_limit": self.overheat_limit,
            "heat_remaining": max(0.0, self.overheat_limit - heat)
            if math.isfinite(heat)
            else math.nan,
        }
        done = failed or time_exhausted
        return done, score, metrics

    def get_task_description(self):
        return {
            "task": "E-02: Thick Air",
            "description": (
                "Move the craft into the target zone without reaching its "
                "thermal limit."
            ),
            "terrain": self.terrain_bounds,
            "success_criteria": {
                "primary": (
                    "Craft center enters target zone "
                    f"(x in [{self.target_x_min:.1f}, {self.target_x_max:.1f}], "
                    f"y in [{self.target_y_min:.1f}, {self.target_y_max:.1f}])"
                ),
                "secondary": (
                    f"Heat remains below {self.overheat_limit:.0f} N·s"
                ),
            },
            "evaluation": {
                "score_range": "0-100",
                "success_score": 100,
                "failure_score": 0,
            },
        }
