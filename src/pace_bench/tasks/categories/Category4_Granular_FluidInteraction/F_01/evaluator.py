from pace_bench.core.primitives import compute_constraint_penalty

class Evaluator:
    MAX_LEAKAGE_RATE = 0.001
    def __init__(self, terrain_bounds, environment=None):
        self.terrain_bounds = terrain_bounds
        self.environment = environment
        self.initial_joint_count = 0
        self.initial_beam_to_beam_joint_count = 0
        self.structure_broken = False
        self.design_constraints_checked = False
        if not environment:
            raise ValueError("Evaluator requires environment instance")
        env_class = type(environment)
        self.MAX_LEAKAGE_RATE = getattr(environment, 'MAX_LEAKAGE_RATE', 0.001)
        self.MAX_STRUCTURE_MASS = getattr(environment, 'MAX_STRUCTURE_MASS', getattr(env_class, 'MAX_STRUCTURE_MASS', 380.0))
        self.MAX_TERRAIN_ANCHORS = getattr(environment, 'MAX_TERRAIN_ANCHORS', getattr(env_class, 'MAX_TERRAIN_ANCHORS', 0))
        self.MAX_BEAM_COUNT = getattr(environment, 'MAX_BEAM_COUNT', getattr(env_class, 'MAX_BEAM_COUNT', 18))
        self.MIN_BEAM_COUNT = getattr(environment, 'MIN_BEAM_COUNT', getattr(env_class, 'MIN_BEAM_COUNT', 10))
        self.MAX_BEAMS_RIGHT_STRIP = getattr(environment, 'MAX_BEAMS_RIGHT_STRIP', getattr(env_class, 'MAX_BEAMS_RIGHT_STRIP', 2))
        self.MAX_BEAMS_MIDDLE_STRIP = getattr(environment, 'MAX_BEAMS_MIDDLE_STRIP', getattr(env_class, 'MAX_BEAMS_MIDDLE_STRIP', 1))
        self.BUILD_ZONE_LEFT_X_MIN = getattr(environment, 'BUILD_ZONE_LEFT_X_MIN', 12.4)
        self.BUILD_ZONE_LEFT_X_MAX = getattr(environment, 'BUILD_ZONE_LEFT_X_MAX', 12.6)
        self.BUILD_ZONE_MIDDLE_X_MIN = getattr(environment, 'BUILD_ZONE_MIDDLE_X_MIN', 12.9)
        self.BUILD_ZONE_MIDDLE_X_MAX = getattr(environment, 'BUILD_ZONE_MIDDLE_X_MAX', 13.1)
        self.BUILD_ZONE_RIGHT_X_MIN = getattr(environment, 'BUILD_ZONE_RIGHT_X_MIN', 13.4)
        self.BUILD_ZONE_RIGHT_X_MAX = getattr(environment, 'BUILD_ZONE_RIGHT_X_MAX', 13.6)
        self.BUILD_ZONE_X_MIN = getattr(environment, 'BUILD_ZONE_X_MIN', 12.4)
        self.BUILD_ZONE_X_MAX = getattr(environment, 'BUILD_ZONE_X_MAX', 13.6)
        self.BUILD_ZONE_Y_MIN = getattr(environment, 'BUILD_ZONE_Y_MIN', 0.0)
        self.BUILD_ZONE_Y_MAX = getattr(environment, 'BUILD_ZONE_Y_MAX', 7.5)
        self.MIN_BEAM_BOTTOM_Y = getattr(environment, 'MIN_BEAM_BOTTOM_Y', 0.5)
        self.MAX_JOINT_COUNT = getattr(environment, 'MAX_JOINT_COUNT', getattr(env_class, 'MAX_JOINT_COUNT', 15))
    def evaluate(self, agent_body, step_count, max_steps):
        if not self.environment:
            return True, 0.0, {
                "success": False,
                "failed": True,
                "failure_reason": "Environment not available",
                "error": "Environment not available",
                "step_count": step_count,
            }
        self._current_max_steps = int(max_steps)
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
                }
            self.design_constraints_checked = True
            self.initial_joint_count = len(self.environment._joints)
            terrain_joints = self.environment.get_terrain_joint_count() if hasattr(self.environment, 'get_terrain_joint_count') else 0
            self.initial_beam_to_beam_joint_count = self.initial_joint_count - terrain_joints
        current_joint_count = len(self.environment._joints)
        terrain_joints = self.environment.get_terrain_joint_count() if hasattr(self.environment, 'get_terrain_joint_count') else 0
        current_beam_to_beam_joint_count = current_joint_count - terrain_joints
        if current_beam_to_beam_joint_count < self.initial_beam_to_beam_joint_count:
            self.structure_broken = True
        done = step_count >= max_steps
        if not done:
            metrics = self._collect_metrics(step_count, success=False, failed=False, failure_reason=None)
            return False, 0.0, metrics
        initial_count = self.environment.get_initial_particle_count()
        leaked_count = self.environment.get_leaked_particle_count()
        leakage_rate = (leaked_count / initial_count) if initial_count > 0 else 0.0
        failed = False
        failure_reason = None
        if leakage_rate > self.MAX_LEAKAGE_RATE:
            failed = True
            limit_pct = self.MAX_LEAKAGE_RATE * 100
            failure_reason = f"Leakage rate {leakage_rate * 100:.1f}% exceeds {limit_pct:.2f}% limit"
        if self.structure_broken:
            failed = True
            failure_reason = (failure_reason or "") + ("; " if failure_reason else "") + "Structure integrity lost (joints broke)"
        success = (leakage_rate <= self.MAX_LEAKAGE_RATE) and not self.structure_broken and not failed
        score = 100.0 if success else 0.0
        metrics = self._collect_metrics(
            step_count,
            success=success,
            failed=failed,
            failure_reason=failure_reason,
            initial_count=initial_count,
            leaked_count=leaked_count,
            leakage_rate=leakage_rate,
        )
        return True, score, metrics
    def _collect_metrics(self, step_count, success=False, failed=False, failure_reason=None,
                         initial_count=None, leaked_count=None, leakage_rate=None):
        if initial_count is None:
            initial_count = self.environment.get_initial_particle_count()
        if leaked_count is None:
            leaked_count = self.environment.get_leaked_particle_count()
        if leakage_rate is None and initial_count > 0:
            leakage_rate = leaked_count / initial_count
        elif leakage_rate is None:
            leakage_rate = 0.0
        current_total = self.environment.get_particle_count()
        retained_count = initial_count - leaked_count
        containment_percent = (1.0 - leakage_rate) * 100.0 if initial_count > 0 else 100.0
        beam_count = len(self.environment._bodies)
        max_beam_count = self.MAX_BEAM_COUNT
        joint_count = len(self.environment._joints)
        terrain_joint_count = self.environment.get_terrain_joint_count() if hasattr(self.environment, 'get_terrain_joint_count') else 0
        initial_joint_count = getattr(self, 'initial_joint_count', joint_count)
        joint_break_events = self.environment.get_joint_break_events() if hasattr(self.environment, 'get_joint_break_events') else []
        joint_peak_forces = self.environment.get_joint_peak_forces() if hasattr(self.environment, 'get_joint_peak_forces') else []
        joint_force_limit = self.environment.get_joint_force_limit() if hasattr(self.environment, 'get_joint_force_limit') else 50000.0
        joint_break_steps = self.environment.get_joint_break_consecutive_steps() if hasattr(self.environment, 'get_joint_break_consecutive_steps') else 3
        beam_coverage = self.environment.get_beam_coverage_envelope() if hasattr(self.environment, 'get_beam_coverage_envelope') else {}
        leak_height = self.environment.get_leak_height_distribution() if hasattr(self.environment, 'get_leak_height_distribution') else {}
        numerical_warnings = self.environment.get_numerical_health_warnings() if hasattr(self.environment, 'get_numerical_health_warnings') else []
        observation_errors = self.environment.get_observation_errors() if hasattr(self.environment, 'get_observation_errors') else []
        max_steps = getattr(self, '_current_max_steps', 10000)
        progress_pct = (float(step_count) / float(max(max_steps, 1))) * 100.0
        structure_mass = self.environment.get_structure_mass()
        max_structure_mass = self.MAX_STRUCTURE_MASS
        dam_x_left = getattr(self.environment, 'DAM_X_LEFT', 12.0)
        dam_x_right = getattr(self.environment, 'DAM_X_RIGHT', 14.0)
        reservoir_fill_height = getattr(self.environment, 'RESERVOIR_FILL_HEIGHT', 7.0)
        return {
            "step_count": step_count,
            "max_steps": max_steps,
            "progress_pct": progress_pct,
            "initial_particle_count": initial_count,
            "leaked_particle_count": leaked_count,
            "leakage_rate": leakage_rate,
            "leakage_rate_percent": leakage_rate * 100.0,
            "leakage_limit_percent": self.MAX_LEAKAGE_RATE * 100.0,
            "retained_particle_count": retained_count,
            "containment_percent": containment_percent,
            "current_particle_count": current_total,
            "beam_count": beam_count,
            "max_beam_count": max_beam_count,
            "min_beam_count": self.MIN_BEAM_COUNT,
            "success": success,
            "failed": failed,
            "failure_reason": failure_reason,
            "structure_mass": structure_mass,
            "max_structure_mass": max_structure_mass,
            "structure_broken": self.structure_broken,
            "joint_count": joint_count,
            "initial_joint_count": initial_joint_count,
            "terrain_joint_count": terrain_joint_count,
            "min_beam_bottom_y": self.MIN_BEAM_BOTTOM_Y,
            "dam_x_left": dam_x_left,
            "dam_x_right": dam_x_right,
            "reservoir_fill_height": reservoir_fill_height,
            "joint_break_events": joint_break_events,
            "joint_peak_forces": joint_peak_forces,
            "joint_force_limit": joint_force_limit,
            "joint_break_consecutive_steps": joint_break_steps,
            "beam_coverage_envelope": beam_coverage,
            "leak_height_distribution": leak_height,
            "numerical_health_warnings": numerical_warnings,
            "observation_errors": observation_errors,
            "build_zone_left_x_min": self.BUILD_ZONE_LEFT_X_MIN,
            "build_zone_left_x_max": self.BUILD_ZONE_LEFT_X_MAX,
            "build_zone_middle_x_min": self.BUILD_ZONE_MIDDLE_X_MIN,
            "build_zone_middle_x_max": self.BUILD_ZONE_MIDDLE_X_MAX,
            "build_zone_right_x_min": self.BUILD_ZONE_RIGHT_X_MIN,
            "build_zone_right_x_max": self.BUILD_ZONE_RIGHT_X_MAX,
            "max_beams_right_strip": self.MAX_BEAMS_RIGHT_STRIP,
            "max_beams_middle_strip": self.MAX_BEAMS_MIDDLE_STRIP,
            "max_joint_count": self.MAX_JOINT_COUNT,
            "min_beams_per_band": getattr(self.environment, 'MIN_BEAMS_PER_BAND', 3),
        }
    def _check_design_constraints(self):
        violations = []
        if not self.environment:
            return ["Environment not available"]
        self.MIN_BEAM_BOTTOM_Y = getattr(self.environment, 'MIN_BEAM_BOTTOM_Y', 0.5)
        self.MAX_BEAM_HEIGHT = getattr(self.environment, 'MAX_BEAM_HEIGHT', 1.5)
        self.MAX_BEAM_WIDTH = getattr(self.environment, 'MAX_BEAM_WIDTH', 0.6)
        structure_mass = self.environment.get_structure_mass()
        if structure_mass > self.MAX_STRUCTURE_MASS:
            violations.append(f"Structure mass {structure_mass:.2f} kg exceeds maximum {self.MAX_STRUCTURE_MASS} kg")
        terrain_joints = self.environment.get_terrain_joint_count() if hasattr(self.environment, 'get_terrain_joint_count') else 0
        if terrain_joints > self.MAX_TERRAIN_ANCHORS:
            note = " (terrain anchors not allowed)" if self.MAX_TERRAIN_ANCHORS == 0 else ""
            violations.append(
                f"Too many terrain anchors: {terrain_joints} (max {self.MAX_TERRAIN_ANCHORS}) {note}"
            )
        beam_count = len(self.environment._bodies)
        if beam_count > self.MAX_BEAM_COUNT:
            violations.append(f"Beam count {beam_count} exceeds maximum {self.MAX_BEAM_COUNT}")
        if beam_count < self.MIN_BEAM_COUNT:
            violations.append(f"Beam count {beam_count} is below minimum {self.MIN_BEAM_COUNT}")
        y_min = getattr(self.environment, 'MIN_BEAM_BOTTOM_Y', 0.5)
        y_max = getattr(self.environment, 'BUILD_ZONE_Y_MAX', 7.5)
        band_limits = [(y_min, 2.5), (2.5, 5.0), (5.0, y_max)]
        min_per_band = getattr(self.environment, 'MIN_BEAMS_PER_BAND', 3)
        for y_lo, y_hi in band_limits:
            n_in_band = sum(1 for b in self.environment._bodies
                           if y_lo <= b.position.y <= y_hi)
            if n_in_band < min_per_band:
                violations.append(
                    f"Vertical band y=[{y_lo}, {y_hi}] has {n_in_band} beam(s); "
                    f"need at least {min_per_band}"
                )
        right_strip_count = sum(1 for b in self.environment._bodies
                               if self.BUILD_ZONE_RIGHT_X_MIN <= b.position.x <= self.BUILD_ZONE_RIGHT_X_MAX)
        if right_strip_count > self.MAX_BEAMS_RIGHT_STRIP:
            violations.append(
                f"Right strip x=[{self.BUILD_ZONE_RIGHT_X_MIN}, {self.BUILD_ZONE_RIGHT_X_MAX}] "
                f"has {right_strip_count} beam(s); max {self.MAX_BEAMS_RIGHT_STRIP}"
            )
        middle_strip_count = sum(1 for b in self.environment._bodies
                                 if self.BUILD_ZONE_MIDDLE_X_MIN <= b.position.x <= self.BUILD_ZONE_MIDDLE_X_MAX)
        if middle_strip_count > self.MAX_BEAMS_MIDDLE_STRIP:
            violations.append(
                f"Middle strip has {middle_strip_count} beam(s); max {self.MAX_BEAMS_MIDDLE_STRIP}"
            )
        if middle_strip_count < 1:
            violations.append(
                f"Middle strip has no beams; need at least 1"
            )
        beam_to_beam_joints = len(self.environment._joints) - terrain_joints
        if beam_to_beam_joints > self.MAX_JOINT_COUNT:
            violations.append(
                f"Beam-to-beam joints {beam_to_beam_joints} exceed maximum {self.MAX_JOINT_COUNT}"
            )
        bodies_set = set(self.environment._bodies)
        floor_body = self.environment._terrain_bodies.get("floor") if hasattr(self.environment, '_terrain_bodies') else None
        if len(self.environment._bodies) > 0:
            adj = {b: set() for b in self.environment._bodies}
            for joint in self.environment._joints:
                a, b = joint.bodyA, joint.bodyB
                if a in bodies_set and b in bodies_set and b != floor_body and a != floor_body:
                    adj[a].add(b)
                    adj[b].add(a)
            from collections import deque
            start = next(iter(self.environment._bodies))
            visited = set()
            q = deque([start])
            visited.add(start)
            while q:
                u = q.popleft()
                for v in adj[u]:
                    if v not in visited:
                        visited.add(v)
                        q.append(v)
            if len(visited) != len(self.environment._bodies):
                violations.append(
                    f"Structure is not fully connected: {len(visited)} of "
                    f"{len(self.environment._bodies)} beams are in the main component "
                    f"({len(self.environment._bodies) - len(visited)} isolated)"
                )
        left_count = sum(1 for b in self.environment._bodies
                        if self.BUILD_ZONE_LEFT_X_MIN <= b.position.x <= self.BUILD_ZONE_LEFT_X_MAX)
        right_count = sum(1 for b in self.environment._bodies
                         if self.BUILD_ZONE_RIGHT_X_MIN <= b.position.x <= self.BUILD_ZONE_RIGHT_X_MAX)
        if left_count < 1 or right_count < 1:
            violations.append(
                f"Need at least one beam in left strip and one in right strip (left={left_count}, right={right_count})"
            )
        def _world_bottom_y(body):
            try:
                mins = []
                for fx in getattr(body, "fixtures", []):
                    shape = getattr(fx, "shape", None)
                    verts = getattr(shape, "vertices", None)
                    if verts:
                        for v in verts:
                            wv = body.GetWorldPoint(v)
                            mins.append(float(wv[1]))
                if mins:
                    return min(mins)
            except (AttributeError, TypeError, ValueError) as exc:
                violations.append(f"Unable to inspect beam geometry: {exc}")
            return None
        for body in self.environment._bodies:
            x, y = body.position.x, body.position.y
            in_left = self.BUILD_ZONE_LEFT_X_MIN <= x <= self.BUILD_ZONE_LEFT_X_MAX
            in_middle = self.BUILD_ZONE_MIDDLE_X_MIN <= x <= self.BUILD_ZONE_MIDDLE_X_MAX
            in_right = self.BUILD_ZONE_RIGHT_X_MIN <= x <= self.BUILD_ZONE_RIGHT_X_MAX
            in_y = self.BUILD_ZONE_Y_MIN <= y <= self.BUILD_ZONE_Y_MAX
            if not ((in_left or in_middle or in_right) and in_y):
                violations.append(
                    f"Beam at ({x:.2f}, {y:.2f}) is outside allowed build strips / vertical range"
                )
            try:
                if body.fixtures:
                    shape = body.fixtures[0].shape
                    hx, hy = None, None
                    try:
                        if hasattr(shape, 'box'):
                            hx, hy = shape.box
                    except (AttributeError, TypeError, ValueError):
                        hx, hy = None, None
                    if hx is None and getattr(shape, 'vertices', None):
                        verts = shape.vertices
                        if len(verts) >= 2:
                            hx = max(abs(v[0]) for v in verts)
                            hy = max(abs(v[1]) for v in verts)
                    if hx is not None and hy is not None:
                        bottom = _world_bottom_y(body)
                        if bottom is None:
                            bottom = body.position.y - hy
                        if bottom < self.MIN_BEAM_BOTTOM_Y:
                            violations.append(
                                f"Beam bottom y={bottom:.2f} is below minimum allowed {self.MIN_BEAM_BOTTOM_Y}"
                            )
                        beam_height = 2.0 * hy
                        beam_width = 2.0 * hx
                        if beam_height > self.MAX_BEAM_HEIGHT + 1e-6:
                            violations.append(
                                f"Beam height {beam_height:.2f} m exceeds maximum {self.MAX_BEAM_HEIGHT} m"
                            )
                        if beam_width > self.MAX_BEAM_WIDTH + 1e-6:
                            violations.append(
                                f"Beam width {beam_width:.2f} m exceeds maximum {self.MAX_BEAM_WIDTH} m"
                            )
            except (IndexError, TypeError, AttributeError, ValueError) as exc:
                violations.append(
                    f"Unable to validate beam geometry at ({x:.2f}, {y:.2f}): {exc}"
                )
        return violations
    def get_constraint_info(self):
        return {
            'max_leakage_rate': self.MAX_LEAKAGE_RATE,
            'max_structure_mass': self.MAX_STRUCTURE_MASS,
            'max_terrain_anchors': self.MAX_TERRAIN_ANCHORS,
            'max_beam_count': self.MAX_BEAM_COUNT,
            'min_beam_count': self.MIN_BEAM_COUNT,
            'max_beams_right_strip': self.MAX_BEAMS_RIGHT_STRIP,
            'max_beams_middle_strip': self.MAX_BEAMS_MIDDLE_STRIP,
            'build_zone_left_x_min': self.BUILD_ZONE_LEFT_X_MIN,
            'build_zone_left_x_max': self.BUILD_ZONE_LEFT_X_MAX,
            'build_zone_middle_x_min': self.BUILD_ZONE_MIDDLE_X_MIN,
            'build_zone_middle_x_max': self.BUILD_ZONE_MIDDLE_X_MAX,
            'build_zone_right_x_min': self.BUILD_ZONE_RIGHT_X_MIN,
            'build_zone_right_x_max': self.BUILD_ZONE_RIGHT_X_MAX,
            'build_zone_x_min': self.BUILD_ZONE_X_MIN,
            'build_zone_x_max': self.BUILD_ZONE_X_MAX,
            'build_zone_y_min': self.BUILD_ZONE_Y_MIN,
            'build_zone_y_max': self.BUILD_ZONE_Y_MAX,
            'min_beam_bottom_y': self.MIN_BEAM_BOTTOM_Y,
            'max_joint_count': self.MAX_JOINT_COUNT,
        }
    def compute_score_with_penalty(self, score: float, metrics: dict) -> float:
        if score > 0:
            return score
        constraint_info = self.get_constraint_info()
        penalty = compute_constraint_penalty("F_01", score, metrics, constraint_info)
        return penalty
    def get_task_description(self):
        limit_pct = self.MAX_LEAKAGE_RATE * 100
        return {
            "task": "F-01: The Dam (extreme)",
            "description": f"Design a dam to block water particles; leakage rate must not exceed {limit_pct:.2f}%",
            "terrain": self.terrain_bounds,
            "success_criteria": {
                "primary": f"Leakage rate <= {limit_pct:.2f}%",
                "secondary": "Dam structure remains intact (no broken joints)",
            },
            "evaluation": {
                "score_range": "0-100",
                "success_score": 100,
                "failure_score": 0,
            },
        }
