from pace_bench.core.primitives import compute_constraint_penalty

from pace_bench.core.simulator import TARGET_FPS

class Evaluator:
    def __init__(self, terrain_bounds, environment=None):
        self.terrain_bounds = terrain_bounds
        self.environment = environment
        self.initial_joint_count = 0
        self.structure_broken = False
        self.design_constraints_checked = False
        if not environment:
            raise ValueError("Evaluator requires environment instance")
        env_class = type(environment)
        self.MAX_STRUCTURE_MASS = getattr(environment, 'MAX_STRUCTURE_MASS', getattr(env_class, 'MAX_STRUCTURE_MASS', 380.0))
        self.BUILD_ZONE_X_MIN = getattr(environment, 'BUILD_ZONE_X_MIN', getattr(env_class, 'BUILD_ZONE_X_MIN', 6.0))
        self.BUILD_ZONE_X_MAX = getattr(environment, 'BUILD_ZONE_X_MAX', getattr(env_class, 'BUILD_ZONE_X_MAX', 18.0))
        self.BUILD_ZONE_Y_MIN = getattr(environment, 'BUILD_ZONE_Y_MIN', getattr(env_class, 'BUILD_ZONE_Y_MIN', 0.0))
        self.BUILD_ZONE_Y_MAX = getattr(environment, 'BUILD_ZONE_Y_MAX', getattr(env_class, 'BUILD_ZONE_Y_MAX', 6.0))
        self.BUILD_ZONE_TOLERANCE = 0.1
        self.MIN_DELIVERY_RATIO = getattr(environment, 'MIN_DELIVERY_RATIO', 0.90)
        self.FORCE_BUDGET = getattr(environment, 'FORCE_BUDGET_PER_STEP', 12000.0)
        self.MAX_TIME_SECONDS = getattr(environment, 'MAX_TIME_SECONDS', 40.0)
        self.MAX_STEPS = getattr(environment, 'MAX_STEPS', None)
        if self.MAX_STEPS is None:
            self.MAX_STEPS = int(self.MAX_TIME_SECONDS * TARGET_FPS)
        else:
            self.MAX_STEPS = int(self.MAX_STEPS)
    def evaluate(self, agent_body, step_count, max_steps):
        if not self.environment:
            return True, 0.0, {
                "success": False,
                "failed": True,
                "failure_reason": "Environment not available",
                "error": "Environment not available",
                "step_count": step_count,
            }
        self._effective_max_steps = min(int(max_steps), self.MAX_STEPS)
        if not self.design_constraints_checked:
            violations = self._check_design_constraints()
            if violations:
                self.design_constraints_checked = True
                return True, 0.0, {
                    "success": False,
                    "failed": True,
                    "failure_reason": "Design constraint violated: " + "; ".join(violations),
                    "step_count": step_count,
                    "constraint_violations": violations,
                    "structure_mass": self.environment.get_structure_mass(),
                    "max_structure_mass": self.MAX_STRUCTURE_MASS,
                    "structure_broken": False,
                    "force_budget": self.FORCE_BUDGET,
                    "min_delivery_ratio_percent": self.MIN_DELIVERY_RATIO * 100.0,
                    "target_x_min": self.environment.TARGET_X_MIN,
                    "target_x_max": self.environment.TARGET_X_MAX,
                    "target_y_min": self.environment.TARGET_Y_MIN,
                    "target_y_max": self.environment.TARGET_Y_MAX,
                    "build_zone_x_min": self.BUILD_ZONE_X_MIN,
                    "build_zone_x_max": self.BUILD_ZONE_X_MAX,
                    "build_zone_y_min": self.BUILD_ZONE_Y_MIN,
                    "build_zone_y_max": self.BUILD_ZONE_Y_MAX,
                    "build_zone_tolerance": self.BUILD_ZONE_TOLERANCE,
                    "max_steps": self._effective_max_steps,
                }
            self.design_constraints_checked = True
            self.initial_joint_count = len(self.environment._joints)
        current_joint_count = len(self.environment._joints)
        if current_joint_count < self.initial_joint_count:
            self.structure_broken = True
        done = step_count >= self._effective_max_steps
        delivery_ratio = self.environment.get_delivery_ratio()
        failed = False
        failure_reason = None
        if done and delivery_ratio < self.MIN_DELIVERY_RATIO:
            failed = True
            failure_reason = f"Delivery efficiency {delivery_ratio*100:.1f}% below {self.MIN_DELIVERY_RATIO*100:.0f}% target"
        if done and self.structure_broken:
            failed = True
            failure_reason = (failure_reason or "") + ("; " if failure_reason else "") + "Structure integrity lost"
        success = done and not failed
        score = 100.0 if success else 0.0
        metrics = self._collect_metrics(
            step_count,
            success=success,
            failed=failed,
            failure_reason=failure_reason,
        )
        return done, score, metrics

    def _collect_metrics(self, step_count, *, success, failed, failure_reason):
        delivery_ratio = self.environment.get_delivery_ratio()
        initial_count = self.environment.get_initial_particle_count()
        in_target_count = self.environment.get_particles_in_target_count()
        particle_stats = self.environment.get_particle_stats()
        closest_info = self.environment.get_closest_particle_to_target()
        hazard_losses = self.environment.get_hazard_losses()
        hazard_loss_positions = self.environment.get_hazard_loss_positions()
        hazard_zone_bounds = self.environment.get_hazard_zone_bounds()
        transport_timeline = self.environment.get_transport_timeline()
        budget_info = self.environment.get_force_budget_utilization()
        zone_velocity_stats = self.environment.get_zone_velocity_stats()
        particles_in_source = self.environment.get_particles_in_source_count()
        particles_in_build_zone = self.environment.get_particles_in_build_zone_count()
        return {
            "delivery_ratio": delivery_ratio,
            "delivery_ratio_percent": delivery_ratio * 100.0,
            "min_delivery_ratio_percent": self.MIN_DELIVERY_RATIO * 100.0,
            "initial_particle_count": initial_count,
            "particles_in_target": in_target_count,
            "particles_in_source": particles_in_source,
            "particles_in_build_zone": particles_in_build_zone,
            "success": success,
            "failed": failed,
            "failure_reason": failure_reason,
            "structure_mass": self.environment.get_structure_mass(),
            "max_structure_mass": self.MAX_STRUCTURE_MASS,
            "structure_broken": self.structure_broken,
            "force_budget": self.FORCE_BUDGET,
            "force_budget_peak_used": budget_info.get("peak_used", 0.0),
            "force_budget_last_used": budget_info.get("last_step_used", 0.0),
            "force_budget_peak_utilization_pct": budget_info.get("peak_utilization_pct", 0.0),
            "force_budget_last_utilization_pct": budget_info.get("last_utilization_pct", 0.0),
            "step_count": step_count,
            "particle_mean_x": particle_stats.get("mean_x", 0.0),
            "particle_mean_y": particle_stats.get("mean_y", 0.0),
            "particle_max_x": particle_stats.get("max_x", 0.0),
            "particle_min_x": particle_stats.get("min_x", 0.0),
            "particle_active_count": particle_stats.get("active_count", 0),
            "closest_particle_distance_to_target": closest_info.get("distance", -1.0),
            "closest_particle_position": closest_info.get("position", (0.0, 0.0)),
            "hazard_losses": hazard_losses,
            "hazard_loss_positions": hazard_loss_positions,
            "hazard_zone_bounds": hazard_zone_bounds,
            "transport_timeline": transport_timeline,
            "zone_velocity_stats": zone_velocity_stats,
            "target_x_min": self.environment.TARGET_X_MIN,
            "target_x_max": self.environment.TARGET_X_MAX,
            "target_y_min": self.environment.TARGET_Y_MIN,
            "target_y_max": self.environment.TARGET_Y_MAX,
            "build_zone_x_min": self.BUILD_ZONE_X_MIN,
            "build_zone_x_max": self.BUILD_ZONE_X_MAX,
            "build_zone_y_min": self.BUILD_ZONE_Y_MIN,
            "build_zone_y_max": self.BUILD_ZONE_Y_MAX,
            "build_zone_tolerance": self.BUILD_ZONE_TOLERANCE,
            "max_steps": getattr(self, '_effective_max_steps', self.MAX_STEPS),
        }
    def _check_design_constraints(self):
        violations = []
        if not self.environment:
            return ["Environment not available"]
        mass = self.environment.get_structure_mass()
        if mass > self.MAX_STRUCTURE_MASS:
            violations.append(f"Mass {mass:.2f}kg exceeds {self.MAX_STRUCTURE_MASS}kg")
        for body in self.environment._bodies:
            x, y = body.position.x, body.position.y
            if not (self.BUILD_ZONE_X_MIN - self.BUILD_ZONE_TOLERANCE <= x <= self.BUILD_ZONE_X_MAX + self.BUILD_ZONE_TOLERANCE and
                    self.BUILD_ZONE_Y_MIN - self.BUILD_ZONE_TOLERANCE <= y <= self.BUILD_ZONE_Y_MAX + self.BUILD_ZONE_TOLERANCE):
                violations.append(f"Component at ({x:.2f}, {y:.2f}) outside build zone")
        if len(self.environment._joints) == 0:
            violations.append("Structure must be anchored to the ground using joints")
        return violations
    def compute_score_with_penalty(self, score: float, metrics: dict) -> float:
        if score > 0:
            return score
        constraint_info = self.get_constraint_info()
        penalty = compute_constraint_penalty("F_06", score, metrics, constraint_info)
        return penalty
    def get_constraint_info(self):
        return {
            'max_structure_mass': self.MAX_STRUCTURE_MASS,
            'build_zone_x_min': self.BUILD_ZONE_X_MIN,
            'build_zone_x_max': self.BUILD_ZONE_X_MAX,
            'build_zone_y_min': self.BUILD_ZONE_Y_MIN,
            'build_zone_y_max': self.BUILD_ZONE_Y_MAX,
            'build_zone_tolerance': self.BUILD_ZONE_TOLERANCE,
            'min_delivery_ratio': self.MIN_DELIVERY_RATIO,
            'force_budget': self.FORCE_BUDGET,
            'max_time_seconds': self.MAX_TIME_SECONDS,
        }
    def get_task_description(self):
        return {
            "task": "F-06: The Pipeline (hard)",
            "description": f"Transport fluid to target; {self.MIN_DELIVERY_RATIO*100:.0f}% delivery; {self.FORCE_BUDGET} N/step budget",
            "success_criteria": {
                "primary": f"Delivery efficiency >= {self.MIN_DELIVERY_RATIO*100:.0f}%",
                "secondary": "Structure intact"
            }
        }
