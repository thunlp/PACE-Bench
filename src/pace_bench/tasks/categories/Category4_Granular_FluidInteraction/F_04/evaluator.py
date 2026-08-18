from pace_bench.core.primitives import compute_constraint_penalty

from Box2D.b2 import staticBody

def _f04_fmt_m(x: float) -> str:
    s = f"{float(x):.4f}".rstrip("0").rstrip(".")
    return s if s else "0"

class Evaluator:
    def __init__(self, terrain_bounds, environment=None):
        self.terrain_bounds = terrain_bounds
        self.environment = environment
        self.initial_joint_count = 0
        self.structure_broken = False
        self.design_constraints_checked = False
        self._initial_structure_pose = {}
        self._pose_break_detected = False
        self._structure_break_step = None
        if not environment:
            raise ValueError("Evaluator requires environment instance")
        env_class = type(environment)
        self.MAX_STRUCTURE_MASS = getattr(environment, 'MAX_STRUCTURE_MASS', getattr(env_class, 'MAX_STRUCTURE_MASS', 75.0))
        self.MAX_BEAMS = getattr(environment, 'MAX_BEAMS', getattr(env_class, 'MAX_BEAMS', 6))
        self.BUILD_ZONE_X_MIN = getattr(environment, 'BUILD_ZONE_X_MIN', getattr(env_class, 'BUILD_ZONE_X_MIN', 5.20))
        self.BUILD_ZONE_X_MAX = getattr(environment, 'BUILD_ZONE_X_MAX', getattr(env_class, 'BUILD_ZONE_X_MAX', 6.90))
        self.BUILD_ZONE_Y_MIN = getattr(environment, 'BUILD_ZONE_Y_MIN', getattr(env_class, 'BUILD_ZONE_Y_MIN', 1.72))
        self.BUILD_ZONE_Y_MAX = getattr(environment, 'BUILD_ZONE_Y_MAX', getattr(env_class, 'BUILD_ZONE_Y_MAX', 2.45))
        self.MIN_PURITY = getattr(environment, 'MIN_PURITY', getattr(env_class, 'MIN_PURITY', 0.35))
    def evaluate(self, agent_body, step_count, max_steps):
        if not self.environment:
            return True, 0.0, {"error": "Environment not available"}
        if not self.design_constraints_checked and step_count == 0:
            violations = self._check_design_constraints()
            if violations:
                self.design_constraints_checked = True
                return True, 0.0, {
                    "success": False,
                    "failed": True,
                    "failure_reason": "Design constraint violated: " + "; ".join(violations),
                    "step_count": step_count,
                    "constraint_violations": violations,
                }
            self.design_constraints_checked = True
            self.initial_joint_count = len(self.environment._joints)
            self._initial_structure_pose = self._capture_structure_pose()
        current_joint_count = len(self.environment._joints)
        if current_joint_count < self.initial_joint_count:
            self.structure_broken = True
            if self._structure_break_step is None:
                self._structure_break_step = step_count
        if self._structure_pose_changed():
            self.structure_broken = True
            self._pose_break_detected = True
            if self._structure_break_step is None:
                self._structure_break_step = step_count
        violations = self._check_design_constraints()
        if violations:
            return True, 0.0, {
                "success": False,
                "failed": True,
                "failure_reason": "Design constraint violated: " + "; ".join(violations),
                "step_count": step_count,
                "constraint_violations": violations,
            }
        done = step_count >= max_steps
        if not done:
            metrics = self._collect_metrics(
                step_count,
                max_steps=max_steps,
                success=False,
                failed=False,
                failure_reason=None,
            )
            return False, 0.0, metrics
        purity = self.environment.get_classification_purity()
        spawned_total = (
            self.environment.get_spawned_particle_count()
            if hasattr(self.environment, "get_spawned_particle_count")
            else self.environment.get_initial_particle_count()
        )
        planned_total = self.environment.get_initial_particle_count()
        small_in_small = self.environment.get_small_in_small_zone_count()
        medium_in_medium = self.environment.get_medium_in_medium_zone_count()
        large_in_large = self.environment.get_large_in_large_zone_count()
        contaminated = getattr(self.environment, 'has_contamination', lambda: False)()
        failed = False
        failure_reason = None
        if purity < self.MIN_PURITY:
            failed = True
            failure_reason = f"Classification purity {purity*100:.1f}% below {self.MIN_PURITY*100:.0f}% target"
        if self.structure_broken:
            failed = True
            detail_parts = []
            if current_joint_count < self.initial_joint_count:
                detail_parts.append("joint count decreased")
            if self._pose_break_detected:
                detail_parts.append("beam pose drifted from initial build state")
            detail = "; ".join(detail_parts) if detail_parts else "integrity condition violated"
            failure_reason = (failure_reason or "") + ("; " if failure_reason else "") + f"Structure integrity lost ({detail})"
        success = purity >= self.MIN_PURITY and not self.structure_broken and not failed
        score = 100.0 if success else 0.0
        metrics = self._collect_metrics(
            step_count,
            max_steps=max_steps,
            success=success,
            failed=failed,
            failure_reason=failure_reason,
            purity=purity,
            spawned_total=spawned_total,
            planned_total=planned_total,
            small_in_small=small_in_small,
            medium_in_medium=medium_in_medium,
            large_in_large=large_in_large,
            contaminated=contaminated,
        )
        return True, score, metrics
    def _collect_metrics(self, step_count, max_steps=None, success=False, failed=False, failure_reason=None,
                         purity=None, spawned_total=None, planned_total=None, small_in_small=None,
                         medium_in_medium=None, large_in_large=None, contaminated=None):
        if spawned_total is None:
            spawned_total = (
                self.environment.get_spawned_particle_count()
                if hasattr(self.environment, "get_spawned_particle_count")
                else self.environment.get_initial_particle_count()
            )
        if planned_total is None:
            planned_total = self.environment.get_initial_particle_count()
        if small_in_small is None:
            small_in_small = self.environment.get_small_in_small_zone_count()
        if medium_in_medium is None:
            medium_in_medium = getattr(self.environment, 'get_medium_in_medium_zone_count', lambda: 0)()
        if large_in_large is None:
            large_in_large = self.environment.get_large_in_large_zone_count()
        if purity is None:
            purity = self.environment.get_classification_purity()
        if contaminated is None:
            contaminated = getattr(self.environment, 'has_contamination', lambda: False)()
        small_above = self.environment.get_small_above_sieve_count()
        small_in_band = self.environment.get_small_in_sieve_band_count()
        large_below = self.environment.get_large_below_sieve_count()
        large_in_band = self.environment.get_large_in_sieve_band_count()
        large_in_small = getattr(self.environment, 'get_large_in_small_zone_count', lambda: 0)()
        small_in_large = getattr(self.environment, 'get_small_in_large_zone_count', lambda: 0)()
        medium_in_small = getattr(self.environment, 'get_medium_in_small_zone_count', lambda: 0)()
        medium_in_large = getattr(self.environment, 'get_medium_in_large_zone_count', lambda: 0)()
        base_metrics = {
            "step_count": step_count,
            "max_steps": max_steps,
            "progress_pct": (
                min(100.0, max(0.0, 100.0 * step_count / max_steps))
                if max_steps
                else None
            ),
            "second_wave_step": int(getattr(self.environment, "SECOND_WAVE_STEP", 1800)),
            "third_wave_step": int(getattr(self.environment, "THIRD_WAVE_STEP", 3600)),
            "spawned_particle_count": spawned_total,
            "planned_total_particle_count": planned_total,
            "small_in_small_zone": small_in_small,
            "medium_in_medium_zone": medium_in_medium,
            "large_in_large_zone": large_in_large,
            "small_above_sieve": small_above,
            "small_in_sieve_band": small_in_band,
            "large_below_sieve": large_below,
            "large_in_sieve_band": large_in_band,
            "large_in_small_zone": large_in_small,
            "small_in_large_zone": small_in_large,
            "medium_in_small_zone": medium_in_small,
            "medium_in_large_zone": medium_in_large,
            "classification_purity": purity,
            "purity_percent": purity * 100.0,
            "min_purity": self.MIN_PURITY,
            "min_purity_percent": self.MIN_PURITY * 100.0,
            "contaminated": contaminated,
            "success": success,
            "failed": failed,
            "failure_reason": failure_reason,
            "structure_mass": self.environment.get_structure_mass(),
            "max_structure_mass": self.MAX_STRUCTURE_MASS,
            "structure_broken": self.structure_broken,
            "structure_break_step": self._structure_break_step,
            "joint_count": len(self.environment._joints),
            "initial_joint_count": self.initial_joint_count,
            "beam_count": len(self.environment._bodies),
            "max_beams": self.MAX_BEAMS,
            "feed_y_min": float(getattr(self.environment, "FEED_Y_MIN", 3.0)),
        }
        env = self.environment
        if hasattr(env, "get_particle_y_stats"):
            base_metrics["particle_y_stats"] = env.get_particle_y_stats()
        if hasattr(env, "get_wave_events"):
            base_metrics["wave_events"] = env.get_wave_events()
        if hasattr(env, "get_particle_velocity_stats"):
            base_metrics["particle_velocity_stats"] = env.get_particle_velocity_stats()
        if hasattr(env, "get_beam_build_zone_margins"):
            base_metrics["beam_build_zone_margins"] = env.get_beam_build_zone_margins()
        if hasattr(env, "get_numerical_health"):
            base_metrics["numerical_health"] = env.get_numerical_health()
        base_metrics["zone_boundaries"] = {
            "small_zone_y_max": getattr(env, "SMALL_ZONE_Y_MAX", 1.92),
            "medium_zone_y_min": getattr(env, "MEDIUM_ZONE_Y_MIN", 1.92),
            "medium_zone_y_max": getattr(env, "MEDIUM_ZONE_Y_MAX", 2.52),
            "large_zone_y_min": getattr(env, "LARGE_ZONE_Y_MIN", 2.52),
            "build_zone_x_min": getattr(env, "BUILD_ZONE_X_MIN", 5.20),
            "build_zone_x_max": getattr(env, "BUILD_ZONE_X_MAX", 6.90),
            "build_zone_y_min": getattr(env, "BUILD_ZONE_Y_MIN", 1.72),
            "build_zone_y_max": getattr(env, "BUILD_ZONE_Y_MAX", 2.45),
            "feed_y_min": float(getattr(env, "FEED_Y_MIN", 3.0)),
        }
        return base_metrics
    def _check_design_constraints(self):
        violations = []
        if not self.environment:
            return ["Environment not available"]
        n_beams = len(self.environment._bodies)
        if n_beams > self.MAX_BEAMS:
            violations.append(f"Number of beams {n_beams} exceeds maximum {self.MAX_BEAMS}")
        structure_mass = self.environment.get_structure_mass()
        if structure_mass > self.MAX_STRUCTURE_MASS:
            violations.append(f"Structure mass {structure_mass:.2f} kg exceeds maximum {self.MAX_STRUCTURE_MASS} kg")
        for body in self.environment._bodies:
            if body.type != staticBody:
                violations.append("Structure contains non-static beam(s); all structural beams must be static")
            ok, vx, vy = self._beam_footprint_inside_build_zone(body)
            if not ok:
                violations.append(
                    f"Beam footprint extends outside build zone (witness vertex ~({vx:.2f}, {vy:.2f}))"
                )
        return violations
    def _beam_footprint_inside_build_zone(self, body):
        eps = 1e-4
        x0, x1 = self.BUILD_ZONE_X_MIN - eps, self.BUILD_ZONE_X_MAX + eps
        y0, y1 = self.BUILD_ZONE_Y_MIN - eps, self.BUILD_ZONE_Y_MAX + eps
        saw_vertex = False
        for fixture in body.fixtures:
            shape = fixture.shape
            if not hasattr(shape, "vertices"):
                continue
            for vertex in shape.vertices:
                saw_vertex = True
                w = body.GetWorldPoint(vertex)
                if not (x0 <= w.x <= x1 and y0 <= w.y <= y1):
                    return False, w.x, w.y
        if not saw_vertex:
            x, y = body.position.x, body.position.y
            if not (x0 <= x <= x1 and y0 <= y <= y1):
                return False, x, y
        return True, None, None
    def _capture_structure_pose(self):
        pose = {}
        for body in self.environment._bodies:
            pose[id(body)] = (float(body.position.x), float(body.position.y), float(body.angle))
        return pose
    def _structure_pose_changed(self):
        if not self._initial_structure_pose:
            return False
        pos_tol = 1e-3
        ang_tol = 1e-3
        for body in self.environment._bodies:
            key = id(body)
            if key not in self._initial_structure_pose:
                return True
            x0, y0, a0 = self._initial_structure_pose[key]
            if abs(float(body.position.x) - x0) > pos_tol:
                return True
            if abs(float(body.position.y) - y0) > pos_tol:
                return True
            if abs(float(body.angle) - a0) > ang_tol:
                return True
        return False
    def compute_score_with_penalty(self, score: float, metrics: dict) -> float:
        if score > 0:
            return score
        constraint_info = self.get_constraint_info()
        penalty = compute_constraint_penalty("F_04", score, metrics, constraint_info)
        return penalty
    def get_constraint_info(self):
        return {
            'max_structure_mass': self.MAX_STRUCTURE_MASS,
            'max_beams': self.MAX_BEAMS,
            'build_zone_x_min': self.BUILD_ZONE_X_MIN,
            'build_zone_x_max': self.BUILD_ZONE_X_MAX,
            'build_zone_y_min': self.BUILD_ZONE_Y_MIN,
            'build_zone_y_max': self.BUILD_ZONE_Y_MAX,
            'min_purity': self.MIN_PURITY,
        }
    def get_task_description(self):
        return {
            "task": "F-04: The Filter (Three-way)",
            "description": f"Separate small, medium, and large balls into three zones; purity >= {self.MIN_PURITY*100:.0f}%, structure intact",
            "terrain": self.terrain_bounds,
            "success_criteria": {
                "primary": f"Classification purity >= {self.MIN_PURITY*100:.0f}%",
                "secondary": (
                ),
            },
            "evaluation": {"score_range": "0-100", "success_score": 100, "failure_score": 0},
        }
