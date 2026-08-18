from pace_bench.core.primitives import compute_constraint_penalty
from pace_bench.core.simulator import TIME_STEP

class Evaluator:
    def __init__(self, terrain_bounds, environment=None):
        self.terrain_bounds = terrain_bounds
        self.environment = environment
        self._right_platform_start_x = terrain_bounds.get("right_platform_start_x", 26.0)
        self._pit_bottom_y = terrain_bounds.get("pit_bottom_y", 0.0)
        self._landing_min_y = float(terrain_bounds.get("landing_min_y", 1.0))
        self._pit_fail_y = self._pit_bottom_y
        spawn = terrain_bounds.get("jumper_spawn", (5.0, 5.0))
        self._jumper_spawn_x = float(spawn[0]) if len(spawn) >= 1 else 5.0
        jw = float(terrain_bounds.get("jumper_width", 0.8))
        jh = float(terrain_bounds.get("jumper_height", 0.6))
        self._jumper_half_w = jw / 2.0
        self._jumper_half_h = jh / 2.0
        self._landed = False
        self._design_constraints_checked = False
        self._slots = list(terrain_bounds.get("slots", []))
        self._barrier_x_min = terrain_bounds.get("barrier_x_min")
        self._barrier_x_max = terrain_bounds.get("barrier_x_max")
        self._barrier_y_max = terrain_bounds.get("barrier_y_max")
        self._barrier2_x_min = terrain_bounds.get("barrier2_x_min")
        self._barrier2_x_max = terrain_bounds.get("barrier2_x_max")
        self._barrier2_y_max = terrain_bounds.get("barrier2_y_max")
        self._barrier3_x_min = terrain_bounds.get("barrier3_x_min")
        self._barrier3_x_max = terrain_bounds.get("barrier3_x_max")
        self._barrier3_y_max = terrain_bounds.get("barrier3_y_max")
        if environment is None:
            raise ValueError("Evaluator requires environment instance")
        self.MAX_STRUCTURE_MASS = getattr(environment, "MAX_STRUCTURE_MASS", 180.0)
        self.BUILD_ZONE_X_MIN = environment.BUILD_ZONE_X_MIN
        self.BUILD_ZONE_X_MAX = environment.BUILD_ZONE_X_MAX
        self.BUILD_ZONE_Y_MIN = environment.BUILD_ZONE_Y_MIN
        self.BUILD_ZONE_Y_MAX = environment.BUILD_ZONE_Y_MAX
        self._initial_state = None
        self._trajectory_events = []
        self._peak_speed = 0.0
        self._peak_speed_step = 0
        self._peak_angular_vel = 0.0
        self._peak_angular_vel_step = 0
        self._prev_vx = None
        self._prev_vy = None
        self._prev_px = None
        self._prev_py = None
        self._prev_step = None
        self._eff_ax_samples = []
        self._eff_ay_samples = []
        self._ten_step_traj = []
        self._slot_approach = {}
        self._design_pass = None
        self._mass_at_check = None
        self._observation_errors = []
    def evaluate(self, agent_body, step_count, max_steps):
        if self.environment is None:
            return True, 0.0, {"error": "Environment not available"}
        pos = self.environment.get_jumper_position()
        vel = self.environment.get_jumper_velocity()
        if pos is None:
            return True, 0.0, {"error": "Jumper not found"}
        px, py = pos
        vx = vel[0] if vel else 0.0
        vy = vel[1] if vel else 0.0
        self._accumulate_diagnostics(px, py, vx, vy, step_count)
        if px >= self._right_platform_start_x and py >= self._landing_min_y:
            self._landed = True
        if not self._design_constraints_checked and step_count <= 1:
            violations = self._check_design_constraints()
            if violations:
                self._design_constraints_checked = True
                metrics = self._make_metrics(
                    pos, vel, step_count, success=False, failed=True,
                    failure_reason="Design constraint violated: " + "; ".join(violations),
                    max_steps=max_steps,
                )
                return True, 0.0, metrics
            self._design_constraints_checked = True
        success = self._landed
        failed = False
        failure_reason = None
        SLOT_MARGIN = 0.05
        for i, slot in enumerate(self._slots):
            if len(slot) != 4:
                continue
            bx_min, bx_max, floor_y, ceil_y = slot
            if bx_min is None or bx_max is None or floor_y is None or ceil_y is None:
                continue
            in_x_range = bx_min <= px <= bx_max
            if not in_x_range:
                continue
            slot_num = i + 1
            if py - self._jumper_half_h <= floor_y + SLOT_MARGIN:
                failed = True
                failure_reason = f"Hit lower red bar in slot {slot_num}: trajectory must pass through the gap between lower and upper red bars"
                break
            if py + self._jumper_half_h >= ceil_y - SLOT_MARGIN:
                failed = True
                failure_reason = f"Hit upper red bar in slot {slot_num}: trajectory must pass through the gap between lower and upper red bars"
                break
        if py < self._pit_fail_y:
            failed = True
            failure_reason = f"Fall into pit: jumper fell into the pit (y < {self._pit_fail_y} m)"
        done = False
        if failed:
            done = True
            score = 0.0
        elif success:
            done = True
            score = 100.0
        elif step_count >= max_steps - 1:
            done = True
            if self._landed:
                score = 100.0
                success = True
            else:
                failed = True
                failure_reason = "Jumper did not reach the right platform (fell into pit or insufficient jump)"
                score = 0.0
        else:
            score = 100.0 if self._landed else 0.0
        metrics = self._make_metrics(
            pos, vel, step_count, success=success, failed=failed,
            failure_reason=failure_reason,
            max_steps=max_steps,
        )
        return done, score, metrics
    def _check_design_constraints(self):
        violations = []
        if self.environment is None:
            return ["Environment not available"]
        mass = self.environment.get_structure_mass()
        if mass > self.MAX_STRUCTURE_MASS:
            violations.append(
                f"Structure mass {mass:.2f} kg exceeds maximum {self.MAX_STRUCTURE_MASS} kg"
            )
        for body in self.environment._bodies:
            x, y = body.position.x, body.position.y
            if not (
                self.BUILD_ZONE_X_MIN <= x <= self.BUILD_ZONE_X_MAX
                and self.BUILD_ZONE_Y_MIN <= y <= self.BUILD_ZONE_Y_MAX
            ):
                violations.append(
                    f"Beam center at ({x:.2f}, {y:.2f}) outside build zone "
                    f"x=[{self.BUILD_ZONE_X_MIN}, {self.BUILD_ZONE_X_MAX}], "
                    f"y=[{self.BUILD_ZONE_Y_MIN}, {self.BUILD_ZONE_Y_MAX}]"
                )
        return violations
    def _accumulate_diagnostics(self, px, py, vx, vy, step_count):
        import math
        speed = math.sqrt(vx * vx + vy * vy)
        if speed > self._peak_speed:
            self._peak_speed = speed
            self._peak_speed_step = step_count
        try:
            jb = self.environment._terrain_bodies.get("jumper") if self.environment else None
            ang_vel = float(jb.angularVelocity) if jb else 0.0
        except Exception as exc:
            self._observation_errors.append(
                f"jumper angular velocity unavailable: {type(exc).__name__}: {exc}"
            )
            ang_vel = 0.0
        abs_ang = abs(ang_vel)
        if abs_ang > self._peak_angular_vel:
            self._peak_angular_vel = abs_ang
            self._peak_angular_vel_step = step_count
        if self._initial_state is None and step_count >= 1 and (vx != 0.0 or vy != 0.0):
            self._initial_state = (step_count, px, py, vx, vy)
            self._trajectory_events.append({
                "step": step_count, "event": "launch",
                "px": round(px, 3), "py": round(py, 3),
                "vx": round(vx, 3), "vy": round(vy, 3),
            })
        if self._prev_vx is not None and self._prev_vx * vx < 0 and abs(vx) > 0.01:
            self._trajectory_events.append({
                "step": step_count, "event": "vx_reversal",
                "prev_vx": round(self._prev_vx, 3), "new_vx": round(vx, 3),
                "px": round(px, 3), "py": round(py, 3),
            })
        if self._prev_vy is not None and self._prev_vy * vy < 0 and abs(vy) > 0.01:
            self._trajectory_events.append({
                "step": step_count, "event": "vy_reversal",
                "prev_vy": round(self._prev_vy, 3), "new_vy": round(vy, 3),
                "px": round(px, 3), "py": round(py, 3),
            })
        if self._peak_angular_vel < 3.0 and abs_ang >= 3.0:
            self._trajectory_events.append({
                "step": step_count, "event": "tumbling_onset",
                "angular_vel": round(abs_ang, 3),
            })
        if self._prev_vx is not None and self._prev_step is not None:
            dt = (step_count - self._prev_step) * TIME_STEP
            if dt > 0.0001:
                ax_eff = (vx - self._prev_vx) / dt
                ay_eff = (vy - self._prev_vy) / dt
                self._eff_ax_samples.append({"step": step_count, "ax": round(ax_eff, 3)})
                self._eff_ay_samples.append({"step": step_count, "ay": round(ay_eff, 3)})
        if step_count % 10 == 0 and step_count > 0:
            self._ten_step_traj.append({
                "step": step_count,
                "px": round(px, 3), "py": round(py, 3),
                "vx": round(vx, 3), "vy": round(vy, 3),
                "speed": round(speed, 3),
                "angular_vel": round(abs_ang, 3),
            })
        SLOT_MARGIN = 0.05
        for i, slot in enumerate(self._slots):
            if len(slot) != 4:
                continue
            bx_min, bx_max, floor_y, ceil_y = slot
            if bx_min is None or bx_max is None or floor_y is None or ceil_y is None:
                continue
            slot_key = f"slot_{i+1}"
            if bx_min <= px <= bx_max:
                floor_m = (py - self._jumper_half_h) - (floor_y + SLOT_MARGIN)
                ceil_m = (ceil_y - SLOT_MARGIN) - (py + self._jumper_half_h)
                prev = self._slot_approach.get(slot_key)
                if prev is None:
                    self._slot_approach[slot_key] = {
                        "bx_min": bx_min, "bx_max": bx_max,
                        "floor_y": floor_y, "ceil_y": ceil_y,
                        "floor_margin": round(floor_m, 3),
                        "ceil_margin": round(ceil_m, 3),
                        "step": step_count, "px": round(px, 3), "py": round(py, 3),
                    }
                else:
                    if floor_m < prev["floor_margin"]:
                        prev["floor_margin"] = round(floor_m, 3)
                        prev["step"] = step_count
                        prev["px"] = round(px, 3)
                        prev["py"] = round(py, 3)
                    if ceil_m < prev["ceil_margin"]:
                        prev["ceil_margin"] = round(ceil_m, 3)
                        prev["step"] = step_count
                        prev["px"] = round(px, 3)
                        prev["py"] = round(py, 3)
        self._prev_vx = vx
        self._prev_vy = vy
        self._prev_px = px
        self._prev_py = py
        self._prev_step = step_count
    def _make_metrics(
        self, pos, vel, step_count, success=False, failed=False, failure_reason=None,
        max_steps=None,
    ):
        px, py = pos if pos else (0, 0)
        vx, vy = (vel[0], vel[1]) if vel else (0, 0)
        speed = (vx * vx + vy * vy) ** 0.5
        progress = max(0.0, (px - self._jumper_spawn_x) / (self._right_platform_start_x - self._jumper_spawn_x)) * 100.0
        progress = min(100.0, progress)
        jumper_body = self.environment._terrain_bodies.get("jumper")
        angular_velocity = float(jumper_body.angularVelocity) if jumper_body else 0.0
        angle = float(jumper_body.angle) if jumper_body else 0.0
        distance_from_platform = max(0.0, self._right_platform_start_x - px)
        m = {
            "jumper_x": px,
            "jumper_y": py,
            "jumper_vx": vx,
            "jumper_vy": vy,
            "jumper_speed": speed,
            "right_platform_start_x": self._right_platform_start_x,
            "progress": progress,
            "success": success,
            "failed": failed,
            "failure_reason": failure_reason,
            "step_count": step_count,
            "max_steps": max_steps,
            "time_step": TIME_STEP,
            "structure_mass": self.environment.get_structure_mass(),
            "max_structure_mass": self.MAX_STRUCTURE_MASS,
            "landed": self._landed,
            "angular_velocity": angular_velocity,
            "angle": angle,
            "distance_from_platform": distance_from_platform,
            "pit_fail_y": self._pit_fail_y,
            "landing_min_y": self._landing_min_y,
        }
        if self._initial_state:
            istep, ipx, ipy, ivx, ivy = self._initial_state
        if self._eff_ax_samples:
            win = self._eff_ax_samples[-max(1, min(20, len(self._eff_ax_samples))):]
            m["effective_ax_mean"] = round(sum(s["ax"] for s in win) / len(win), 3)
            m["effective_ax_last"] = self._eff_ax_samples[-1]["ax"]
        if self._eff_ay_samples:
            win = self._eff_ay_samples[-max(1, min(20, len(self._eff_ay_samples))):]
            m["effective_ay_mean"] = round(sum(s["ay"] for s in win) / len(win), 3)
            m["effective_ay_last"] = self._eff_ay_samples[-1]["ay"]
        m["peak_speed"] = round(self._peak_speed, 3)
        m["peak_speed_step"] = self._peak_speed_step
        m["peak_angular_vel"] = round(self._peak_angular_vel, 3)
        m["peak_angular_vel_step"] = self._peak_angular_vel_step
        if self._initial_state:
            m["initial_step"] = self._initial_state[0]
            m["initial_px"] = self._initial_state[1]
            m["initial_py"] = self._initial_state[2]
            m["initial_vx"] = self._initial_state[3]
            m["initial_vy"] = self._initial_state[4]
        m["trajectory_events"] = list(self._trajectory_events)
        m["trajectory_snapshots"] = list(self._ten_step_traj)
        m["slot_closest_approach"] = dict(self._slot_approach)
        slot_defs = []
        for i, slot in enumerate(self._slots):
            if len(slot) == 4:
                slot_defs.append({
                    "slot_num": i + 1,
                    "x_min": slot[0], "x_max": slot[1],
                    "floor_y": slot[2], "ceil_y": slot[3],
                })
        m["slot_definitions"] = slot_defs
        if self._design_pass is not None:
            m["design_constraint_pass"] = self._design_pass
        m["build_zone_x_min"] = self.BUILD_ZONE_X_MIN
        m["build_zone_x_max"] = self.BUILD_ZONE_X_MAX
        m["build_zone_y_min"] = self.BUILD_ZONE_Y_MIN
        m["build_zone_y_max"] = self.BUILD_ZONE_Y_MAX
        m["jumper_spawn_x"] = self._jumper_spawn_x
        m["jumper_half_w"] = self._jumper_half_w
        m["jumper_half_h"] = self._jumper_half_h
        m["observation_errors"] = list(self._observation_errors)
        return m
    def compute_score_with_penalty(self, score: float, metrics: dict) -> float:
        if score > 0:
            return score
        constraint_info = self.get_constraint_info()
        penalty = compute_constraint_penalty("D_02", score, metrics, constraint_info)
        return penalty
    def get_constraint_info(self):
        return {
            'right_platform_start_x': self._right_platform_start_x,
            'pit_bottom_y': self._pit_bottom_y,
            'landing_min_y': self._landing_min_y,
            'max_structure_mass': self.MAX_STRUCTURE_MASS,
            'build_zone_x_min': self.BUILD_ZONE_X_MIN,
            'build_zone_x_max': self.BUILD_ZONE_X_MAX,
            'build_zone_y_min': self.BUILD_ZONE_Y_MIN,
            'build_zone_y_max': self.BUILD_ZONE_Y_MAX,
        }
    def get_task_description(self):
        return {
            "task": "D-02: The Jumper",
            "description": "Launch a jumper across a pit with an obstacle; trajectory must go OVER the barrier to the right platform",
            "success_criteria": {
                "primary": f"Jumper reaches right platform (x >= {self._right_platform_start_x} m, y >= {self._landing_min_y} m)",
                "failure": "Fall into pit or insufficient jump",
            },
            "evaluation": {"score_range": "0-100", "success_score": 100, "failure_score": 0},
        }
