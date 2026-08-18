import math

from pace_bench.core.primitives import compute_constraint_penalty
from pace_bench.core.simulator import TIME_STEP

def _segment_intersects_rect(x0, y0, x1, y1, rx_min, ry_min, rx_max, ry_max):
    if rx_min <= x0 <= rx_max and ry_min <= y0 <= ry_max:
        return True
    if rx_min <= x1 <= rx_max and ry_min <= y1 <= ry_max:
        return True
    dx, dy = x1 - x0, y1 - y0
    for edge in ("left", "right", "bottom", "top"):
        if edge == "left" and dx != 0:
            t = (rx_min - x0) / dx
            if 0 <= t <= 1 and ry_min <= y0 + t * dy <= ry_max:
                return True
        if edge == "right" and dx != 0:
            t = (rx_max - x0) / dx
            if 0 <= t <= 1 and ry_min <= y0 + t * dy <= ry_max:
                return True
        if edge == "bottom" and dy != 0:
            t = (ry_min - y0) / dy
            if 0 <= t <= 1 and rx_min <= x0 + t * dx <= rx_max:
                return True
        if edge == "top" and dy != 0:
            t = (ry_max - y0) / dy
            if 0 <= t <= 1 and rx_min <= x0 + t * dx <= rx_max:
                return True
    return False

class Evaluator:
    def __init__(self, terrain_bounds, environment=None):
        self.terrain_bounds = terrain_bounds
        self.environment = environment
        spawn = terrain_bounds.get("projectile_spawn", (10.0, 3.0))
        self._spawn_x = float(spawn[0]) if spawn else 10.0
        self._spawn_y = float(spawn[1]) if spawn else 3.0
        tz = terrain_bounds.get("target_zone", {})
        self.target_x_min = tz.get("x_min", 40.0)
        self.target_x_max = tz.get("x_max", 45.0)
        self.target_y_min = tz.get("y_min", 2.0)
        self.target_y_max = tz.get("y_max", 5.0)
        self._eff_y_min = self.target_y_min
        self._eff_y_max = self.target_y_max
        self._hit_occurred = False
        self._design_constraints_checked = False
        self._last_pos = None
        self._max_y_in_target_x = None
        self._arm_proj_contact_step = None
        self._proj_launched_step = None
        self._proj_launch_vx = None
        self._proj_launch_vy = None
        self._peak_arm_ke = 0.0
        self._peak_arm_ke_step = 0
        self._peak_arm_ang_vel = 0.0
        self._peak_arm_ang_vel_step = 0
        self._peak_proj_speed = 0.0
        self._peak_proj_speed_step = 0
        self._peak_proj_y = -1e9
        self._peak_proj_y_step = 0
        self._arm_awake_first_step = None
        self._spring_ever_compressed = False
        self._spring_compressed_first_step = None
        self._entered_target_x_step = None
        self._exited_target_x_step = None
        self._proj_in_target_x = False
        if environment is None:
            raise ValueError("Evaluator requires environment instance")
        self.MAX_STRUCTURE_MASS = getattr(environment, "MAX_STRUCTURE_MASS", 500.0)
        self.BUILD_ZONE_X_MIN = environment.BUILD_ZONE_X_MIN
        self.BUILD_ZONE_X_MAX = environment.BUILD_ZONE_X_MAX
        self.BUILD_ZONE_Y_MIN = environment.BUILD_ZONE_Y_MIN
        self.BUILD_ZONE_Y_MAX = environment.BUILD_ZONE_Y_MAX
    def evaluate(self, agent_body, step_count, max_steps):
        if self.environment is None:
            return True, 0.0, {"error": "Environment not available"}
        pos = self.environment.get_projectile_position()
        vel = self.environment.get_projectile_velocity()
        if pos is None:
            return True, 0.0, {"error": "Projectile not found"}
        px, py = pos
        vx = vel[0] if vel else 0.0
        vy = vel[1] if vel else 0.0
        in_zone = (
            self.target_x_min <= px <= self.target_x_max
            and self._eff_y_min <= py <= self._eff_y_max
        )
        if in_zone:
            self._hit_occurred = True
        if self.target_x_min <= px <= self.target_x_max:
            if self._max_y_in_target_x is None or py > self._max_y_in_target_x:
                self._max_y_in_target_x = py
        if not self._hit_occurred and self._last_pos is not None:
            x0, y0 = self._last_pos
            if _segment_intersects_rect(x0, y0, px, py,
                    self.target_x_min, self._eff_y_min, self.target_x_max, self._eff_y_max):
                self._hit_occurred = True
        self._last_pos = (px, py)
        if not self._design_constraints_checked and step_count == 0:
            violations = self._check_design_constraints()
            if violations:
                self._design_constraints_checked = True
                metrics = self._make_metrics(
                    pos, vel, step_count, success=False, failed=True,
                    failure_reason="Design constraint violated: " + "; ".join(violations),
                )
                return True, 0.0, metrics
            self._design_constraints_checked = True
        success = self._hit_occurred
        failed = False
        failure_reason = None
        sim_x_min = getattr(type(self.environment), "SIM_BOUNDS_X_MIN", -10.0)
        sim_x_max = getattr(type(self.environment), "SIM_BOUNDS_X_MAX", 60.0)
        sim_y_min = getattr(type(self.environment), "SIM_BOUNDS_Y_MIN", -5.0)
        if not self._hit_occurred and (py < sim_y_min or px < sim_x_min or px > sim_x_max):
            failed = True
            failure_reason = "Projectile left simulation bounds"
        done = False
        if failed:
            done = True
            score = 0.0
        elif success:
            done = True
            score = 100.0
        elif step_count >= max_steps - 1:
            done = True
            if self._hit_occurred:
                score = 100.0
                success = True
            else:
                if px < self.target_x_min:
                    failed = True
                    failure_reason = "Insufficient distance: projectile did not reach target zone"
                else:
                    failed = True
                    failure_reason = "Miss: projectile did not land inside target zone (wrong y or overshoot)"
                score = 0.0
        else:
            score = 0.0 if failed else (100.0 if self._hit_occurred else 0.0)
        self._update_temporal_tracking(pos, vel, step_count)
        metrics = self._make_metrics(
            pos, vel, step_count, success=success, failed=failed,
            failure_reason=failure_reason,
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
    def _update_temporal_tracking(self, pos, vel, step_count):
        if self.environment is None:
            return
        px, py = pos if pos else (0, 0)
        vx, vy = vel if vel else (0, 0)
        speed = math.sqrt(vx * vx + vy * vy)
        dist_from_spawn = math.sqrt(
            (px - self._spawn_x) ** 2 + (py - self._spawn_y) ** 2
        )
        if self._proj_launched_step is None and speed > 0.1 and dist_from_spawn > 0.1:
            self._proj_launched_step = step_count
            self._proj_launch_vx = vx
            self._proj_launch_vy = vy
        if speed > self._peak_proj_speed:
            self._peak_proj_speed = speed
            self._peak_proj_speed_step = step_count
        if py > self._peak_proj_y:
            self._peak_proj_y = py
            self._peak_proj_y_step = step_count
        in_target_x = self.target_x_min <= px <= self.target_x_max
        if in_target_x and not self._proj_in_target_x:
            if self._entered_target_x_step is None:
                self._entered_target_x_step = step_count
            self._proj_in_target_x = True
        elif not in_target_x and self._proj_in_target_x:
            if self._exited_target_x_step is None:
                self._exited_target_x_step = step_count
            self._proj_in_target_x = False
        elif not in_target_x and not self._proj_in_target_x:
            pass
        try:
            arm_state = self.environment.get_arm_state()
            if arm_state is not None:
                arm_ke = float(arm_state.get("kinetic_energy", 0.0))
                arm_av = float(arm_state.get("angular_velocity", 0.0))
                arm_awake = bool(arm_state.get("awake", False))
                if arm_ke > self._peak_arm_ke:
                    self._peak_arm_ke = arm_ke
                    self._peak_arm_ke_step = step_count
                if abs(arm_av) > abs(self._peak_arm_ang_vel):
                    self._peak_arm_ang_vel = arm_av
                    self._peak_arm_ang_vel_step = step_count
                if arm_awake and self._arm_awake_first_step is None:
                    self._arm_awake_first_step = step_count
        except Exception:
            pass
        try:
            springs = self.environment.get_spring_states()
            for s in springs:
                if float(s.get("elastic_pe", 0.0)) > 1e-6:
                    self._spring_ever_compressed = True
                    if self._spring_compressed_first_step is None:
                        self._spring_compressed_first_step = step_count
                    break
        except Exception:
            pass
        if self._arm_proj_contact_step is None:
            try:
                if len(self.environment._bodies) > 0:
                    arm_body = self.environment._bodies[0]
                    proj_body = self.environment.get_projectile()
                    if proj_body is not None:
                        contacts = self.environment.get_contacts_involving(
                            arm_body, proj_body
                        )
                        for c in contacts:
                            if c.get("touching"):
                                self._arm_proj_contact_step = step_count
                                break
            except Exception:
                pass
    def _make_metrics(
        self, pos, vel, step_count, success=False, failed=False, failure_reason=None
    ):
        px, py = pos if pos else (0, 0)
        vx, vy = (vel[0], vel[1]) if vel else (0, 0)
        speed = (vx * vx + vy * vy) ** 0.5
        progress = max(
            0.0,
            (px - self._spawn_x) / (self.target_x_min - self._spawn_x),
        ) if (self.target_x_min - self._spawn_x) > 0 else 0.0
        progress = min(1.0, progress) * 100.0
        metrics = {
            "projectile_x": px,
            "projectile_y": py,
            "projectile_vx": vx,
            "projectile_vy": vy,
            "projectile_speed": speed,
            "target_x_min": self.target_x_min,
            "target_x_max": self.target_x_max,
            "target_y_min": self.target_y_min,
            "target_y_max": self.target_y_max,
            "progress": progress,
            "success": success,
            "failed": failed,
            "failure_reason": failure_reason,
            "step_count": step_count,
            "time_step": TIME_STEP,
            "structure_mass": self.environment.get_structure_mass(),
            "max_structure_mass": self.MAX_STRUCTURE_MASS,
            "hit_occurred": self._hit_occurred,
            "max_y_in_target_x": self._max_y_in_target_x,
            "sim_x_min": getattr(type(self.environment), "SIM_BOUNDS_X_MIN", -10.0),
            "sim_x_max": getattr(type(self.environment), "SIM_BOUNDS_X_MAX", 60.0),
            "sim_y_min": getattr(type(self.environment), "SIM_BOUNDS_Y_MIN", -5.0),
            "constraint_info": self.get_constraint_info(),
            "observation_error_count": getattr(self.environment, "_observation_error_count", 0),
            "last_observation_error": getattr(self.environment, "_last_observation_error", None),
        }
        try:
            proj_body = self.environment.get_projectile()
            if proj_body is not None:
                metrics["projectile_awake"] = bool(proj_body.awake)
        except Exception:
            pass
        try:
            metrics["projectile_spawn_x"] = self._spawn_x
            metrics["projectile_spawn_y"] = self._spawn_y
        except Exception:
            pass
        try:
            metrics["spring_states"] = self.environment.get_spring_states()
        except Exception:
            metrics["spring_states"] = []
        try:
            metrics["joint_topology"] = self.environment.get_joint_topology()
        except Exception:
            metrics["joint_topology"] = []
        try:
            arm_state = self.environment.get_arm_state()
            if arm_state is not None:
                metrics["arm_angle"] = arm_state["angle"]
                metrics["arm_angular_velocity"] = arm_state["angular_velocity"]
                metrics["arm_kinetic_energy"] = arm_state["kinetic_energy"]
                metrics["arm_speed"] = arm_state["speed"]
                metrics["arm_position_x"] = arm_state["position"][0]
                metrics["arm_position_y"] = arm_state["position"][1]
                metrics["arm_mass"] = arm_state["mass"]
                metrics["arm_awake"] = arm_state["awake"]
                if arm_state.get("pivot") is not None:
                    metrics["arm_pivot_x"] = arm_state["pivot"][0]
                    metrics["arm_pivot_y"] = arm_state["pivot"][1]
        except Exception:
            pass
        try:
            arm_to_proj_dx = px - metrics.get("arm_position_x", px)
            arm_to_proj_dy = py - metrics.get("arm_position_y", py)
            metrics["arm_to_projectile_distance"] = math.sqrt(
                arm_to_proj_dx ** 2 + arm_to_proj_dy ** 2
            )
        except Exception:
            pass
        try:
            metrics["beam_count"] = len(self.environment._bodies)
        except Exception:
            pass
        try:
            metrics["joint_count"] = len(self.environment._joints)
        except Exception:
            pass
        metrics["arm_proj_contact_step"] = self._arm_proj_contact_step
        metrics["proj_launched_step"] = self._proj_launched_step
        metrics["proj_launch_vx"] = self._proj_launch_vx
        metrics["proj_launch_vy"] = self._proj_launch_vy
        metrics["peak_arm_ke"] = self._peak_arm_ke
        metrics["peak_arm_ke_step"] = self._peak_arm_ke_step
        metrics["peak_arm_ang_vel"] = self._peak_arm_ang_vel
        metrics["peak_arm_ang_vel_step"] = self._peak_arm_ang_vel_step
        metrics["peak_proj_speed"] = self._peak_proj_speed
        metrics["peak_proj_speed_step"] = self._peak_proj_speed_step
        metrics["peak_proj_y"] = self._peak_proj_y if self._peak_proj_y > -1e8 else None
        metrics["peak_proj_y_step"] = self._peak_proj_y_step
        metrics["arm_awake_first_step"] = self._arm_awake_first_step
        metrics["spring_ever_compressed"] = self._spring_ever_compressed
        metrics["spring_compressed_first_step"] = self._spring_compressed_first_step
        metrics["entered_target_x_step"] = self._entered_target_x_step
        metrics["exited_target_x_step"] = self._exited_target_x_step
        return metrics
    def compute_score_with_penalty(self, score: float, metrics: dict) -> float:
        if score > 0:
            return score
        constraint_info = self.get_constraint_info()
        penalty = compute_constraint_penalty("D_01", score, metrics, constraint_info)
        return penalty
    def get_constraint_info(self):
        return {
            'target_x_min': self.target_x_min,
            'target_x_max': self.target_x_max,
            'target_y_min': self.target_y_min,
            'target_y_max': self.target_y_max,
            'max_structure_mass': self.MAX_STRUCTURE_MASS,
            'build_zone_x_min': self.BUILD_ZONE_X_MIN,
            'build_zone_x_max': self.BUILD_ZONE_X_MAX,
            'build_zone_y_min': self.BUILD_ZONE_Y_MIN,
            'build_zone_y_max': self.BUILD_ZONE_Y_MAX,
        }
    def get_task_description(self):
        return {
            "task": "D-01: The Launcher",
            "description": "Design a launcher to propel a projectile into a distant target zone",
            "target_zone": {
                "x": [self.target_x_min, self.target_x_max],
                "y": [self.target_y_min, self.target_y_max],
            },
            "success_criteria": {
                "primary": f"Projectile center enters target zone (x in [{self.target_x_min}, {self.target_x_max}] m, y in [{self.target_y_min}, {self.target_y_max}] m)",
                "failure_miss": "Projectile does not land inside target zone",
                "failure_insufficient_distance": "Projectile does not reach target x range",
            },
            "evaluation": {
                "score_range": "0-100",
                "success_score": 100,
                "failure_score": 0,
            },
        }
