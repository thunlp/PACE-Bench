import math

from pace_bench.core.primitives import compute_constraint_penalty

def _b2_same_body(a, b):
    if a is None or b is None:
        return False
    if a is b:
        return True
    ta = getattr(a, "this", None)
    tb = getattr(b, "this", None)
    return ta is not None and tb is not None and ta == tb

class Evaluator:
    def __init__(self, terrain_bounds, environment=None):
        self.environment = environment
        self._caught_speed_threshold = 0.35
        self._pit_y_threshold = 0.72
        self._pit_speed_threshold = 1.0
        self._ball_ever_in_pit_fast = False
        self._approach_x = 7.4
        self._ball_arrived = set()
        self._sequential_violation = False
        self._balls_caught = set()
        self._design_constraints_checked = False
        self._ball_approach_step = {}
        self._ball_speed_at_approach = {}
        self._observation_errors = []
        if environment is None:
            raise ValueError("Evaluator requires environment instance")
        self.MAX_STRUCTURE_MASS = getattr(environment, "MAX_STRUCTURE_MASS", 10.0)
        self.BUILD_ZONE_X_MIN = environment.BUILD_ZONE_X_MIN
        self.BUILD_ZONE_X_MAX = environment.BUILD_ZONE_X_MAX
        self.BUILD_ZONE_Y_MIN = environment.BUILD_ZONE_Y_MIN
        self.BUILD_ZONE_Y_MAX = environment.BUILD_ZONE_Y_MAX
        self._target_x_min = self.BUILD_ZONE_X_MIN
        self._target_x_max = self.BUILD_ZONE_X_MAX
        self._target_y_min = self.BUILD_ZONE_Y_MIN
        self._target_y_max = self.BUILD_ZONE_Y_MAX
        self.FORBIDDEN_ZONE_X_MIN = getattr(environment, "FORBIDDEN_ZONE_X_MIN", 8.5)
        self.FORBIDDEN_ZONE_X_MAX = getattr(environment, "FORBIDDEN_ZONE_X_MAX", 9.5)
        self.FORBIDDEN_ZONE_2_X_MIN = getattr(environment, "FORBIDDEN_ZONE_2_X_MIN", 7.35)
        self.FORBIDDEN_ZONE_2_X_MAX = getattr(environment, "FORBIDDEN_ZONE_2_X_MAX", 7.75)
        self.FORBIDDEN_ZONE_3_X_MIN = getattr(environment, "FORBIDDEN_ZONE_3_X_MIN", 7.78)
        self.FORBIDDEN_ZONE_3_X_MAX = getattr(environment, "FORBIDDEN_ZONE_3_X_MAX", 8.55)
        self.FORBIDDEN_ZONE_4_X_MIN = getattr(environment, "FORBIDDEN_ZONE_4_X_MIN", 10.0)
        self.FORBIDDEN_ZONE_4_X_MAX = getattr(environment, "FORBIDDEN_ZONE_4_X_MAX", 10.5)
        self.FORBIDDEN_ZONE_5_X_MIN = getattr(environment, "FORBIDDEN_ZONE_5_X_MIN", 7.18)
        self.FORBIDDEN_ZONE_5_X_MAX = getattr(environment, "FORBIDDEN_ZONE_5_X_MAX", 7.34)
        self.MAX_BEAM_COUNT = getattr(environment, "MAX_BEAM_COUNT", 9)
        self.SWEEPER_BAND_1_Y_MIN = getattr(environment, "SWEEPER_BAND_1_Y_MIN", 2.95)
        self.SWEEPER_BAND_1_Y_MAX = getattr(environment, "SWEEPER_BAND_1_Y_MAX", 3.55)
        self.SWEEPER_BAND_2_Y_MIN = getattr(environment, "SWEEPER_BAND_2_Y_MIN", 4.15)
        self.SWEEPER_BAND_2_Y_MAX = getattr(environment, "SWEEPER_BAND_2_Y_MAX", 4.75)
        self.SWEEPER_BAND_3_Y_MIN = getattr(environment, "SWEEPER_BAND_3_Y_MIN", 1.0)
        self.SWEEPER_BAND_3_Y_MAX = getattr(environment, "SWEEPER_BAND_3_Y_MAX", 1.5)
        self.SWEEPER_BAND_4_Y_MIN = getattr(environment, "SWEEPER_BAND_4_Y_MIN", 2.0)
        self.SWEEPER_BAND_4_Y_MAX = getattr(environment, "SWEEPER_BAND_4_Y_MAX", 2.5)
        tb = environment.get_terrain_bounds() if hasattr(environment, "get_terrain_bounds") else {}
        self.terrain_bounds = tb
        self._max_joint_force = tb.get("max_joint_force", 880.0)
        self._joint_fatigue_threshold = tb.get("joint_fatigue_threshold", 760.0)
    def evaluate(self, agent_body, step_count, max_steps):
        if self.environment is None:
            return True, 0.0, {"error": "Environment not available"}
        positions = self.environment.get_all_balls_positions() if hasattr(self.environment, "get_all_balls_positions") else [self.environment.get_ball_position()]
        velocities = self.environment.get_all_balls_velocities() if hasattr(self.environment, "get_all_balls_velocities") else [self.environment.get_ball_velocity()]
        if not positions or not velocities:
            return True, 0.0, {"error": "No balls found"}
        for i, (pos, vel) in enumerate(zip(positions, velocities)):
            if pos is None or vel is None:
                continue
            px, py = pos[0], pos[1]
            vx, vy = vel[0], vel[1]
            speed = (vx * vx + vy * vy) ** 0.5
            in_target_box = (
                self._target_x_min <= px <= self._target_x_max
                and self._target_y_min <= py <= self._target_y_max
            )
            if speed < self._caught_speed_threshold and in_target_box:
                self._balls_caught.add(i)
            if (
                i not in self._balls_caught
                and py < self._pit_y_threshold
                and speed > self._pit_speed_threshold
            ):
                self._ball_ever_in_pit_fast = True
            if px < self._approach_x and i not in self._ball_arrived:
                self._ball_arrived.add(i)
                if i not in self._ball_approach_step:
                    self._ball_approach_step[i] = step_count
                    self._ball_speed_at_approach[i] = speed
                for j in range(i):
                    if j not in self._balls_caught:
                        self._sequential_violation = True
                        break
        n_balls = len(positions)
        all_caught = len(self._balls_caught) >= n_balls and n_balls >= 7
        if not hasattr(self.environment, "get_all_balls_positions") or n_balls < 7:
            all_caught = False
        if n_balls >= 7 and len(self._balls_caught) < n_balls:
            all_caught = False
        if not self._design_constraints_checked and step_count == 0:
            violations = self._check_design_constraints()
            if violations:
                self._design_constraints_checked = True
                metrics = self._make_metrics(positions, velocities, step_count, False, True,
                    "Design constraint violated: " + "; ".join(violations), max_steps)
                return True, 0.0, metrics
            self._design_constraints_checked = True
        structure_smashed = self.environment.is_structure_smashed()
        success = all_caught and not structure_smashed and not self._ball_ever_in_pit_fast and not self._sequential_violation
        failed = False
        failure_reason = None
        if self._ball_ever_in_pit_fast:
            failed = True
            failure_reason = "Pit failure: an uncaught ball dropped below y=0.72 m with speed exceeding 1.0 m/s"
        elif self._sequential_violation:
            failed = True
            failure_reason = "Sequential violation: a ball crossed the approach line (x < 7.4 m) before all lower-index balls were caught"
        elif structure_smashed:
            failed = True
            failure_reason = "Structure smashed: a joint exceeded the peak or fatigue force limit"
        elif step_count >= max_steps - 1 and not all_caught:
            failed = True
            failure_reason = "Time limit reached: not all balls were caught before the maximum step count"
        done = failed or success or step_count >= max_steps - 1
        score = 100.0 if success else (0.0 if failed else 0.0)
        metrics = self._make_metrics(
            positions, velocities, step_count, success, failed, failure_reason, max_steps
        )
        return done, score, metrics
    def _check_design_constraints(self):
        violations = []
        if self.environment is None:
            return ["Environment not available"]
        ground = None
        if hasattr(self.environment, "_terrain_bodies"):
            ground = self.environment._terrain_bodies.get("ground")
        has_rigid_ground_anchor = False
        for joint in getattr(self.environment, "_joints", []):
            body_a = getattr(joint, "bodyA", None)
            body_b = getattr(joint, "bodyB", None)
            touches_ground = ground is not None and (
                _b2_same_body(body_a, ground) or _b2_same_body(body_b, ground)
            )
            is_rigid = "weld" in joint.__class__.__name__.lower()
            if touches_ground and is_rigid:
                has_rigid_ground_anchor = True
                break
        if not has_rigid_ground_anchor:
            violations.append("At least one rigid joint must anchor an agent beam to the ground")
        mass = self.environment.get_structure_mass()
        if mass >= self.MAX_STRUCTURE_MASS:
            violations.append(
                f"Structure mass {mass:.2f} kg must be strictly below maximum {self.MAX_STRUCTURE_MASS} kg"
            )
        n_beams = len(self.environment._bodies)
        if n_beams > self.MAX_BEAM_COUNT:
            violations.append(f"Beam count {n_beams} exceeds maximum {self.MAX_BEAM_COUNT}")
        for body in self.environment._bodies:
            x, y = body.position.x, body.position.y
            if not (self.BUILD_ZONE_X_MIN <= x <= self.BUILD_ZONE_X_MAX and
                    self.BUILD_ZONE_Y_MIN <= y <= self.BUILD_ZONE_Y_MAX):
                violations.append(
                    f"Beam center at ({x:.2f}, {y:.2f}) outside build zone "
                    f"x=[{self.BUILD_ZONE_X_MIN}, {self.BUILD_ZONE_X_MAX}], "
                    f"y=[{self.BUILD_ZONE_Y_MIN}, {self.BUILD_ZONE_Y_MAX}]"
                )
            if self.FORBIDDEN_ZONE_X_MIN <= x <= self.FORBIDDEN_ZONE_X_MAX:
                violations.append(
                    f"Beam center at ({x:.2f}, {y:.2f}) in forbidden zone 1 "
                    f"x=[{self.FORBIDDEN_ZONE_X_MIN}, {self.FORBIDDEN_ZONE_X_MAX}]"
                )
            if self.FORBIDDEN_ZONE_2_X_MIN <= x <= self.FORBIDDEN_ZONE_2_X_MAX:
                violations.append(
                    f"Beam center at ({x:.2f}, {y:.2f}) in forbidden zone 2 "
                    f"x=[{self.FORBIDDEN_ZONE_2_X_MIN}, {self.FORBIDDEN_ZONE_2_X_MAX}]"
                )
            if self.FORBIDDEN_ZONE_3_X_MIN <= x <= self.FORBIDDEN_ZONE_3_X_MAX:
                violations.append(
                    f"Beam center at ({x:.2f}, {y:.2f}) in forbidden zone 3 "
                    f"x=[{self.FORBIDDEN_ZONE_3_X_MIN}, {self.FORBIDDEN_ZONE_3_X_MAX}]"
                )
            if self.FORBIDDEN_ZONE_4_X_MIN <= x <= self.FORBIDDEN_ZONE_4_X_MAX:
                violations.append(
                    f"Beam center at ({x:.2f}, {y:.2f}) in forbidden zone 4 "
                    f"x=[{self.FORBIDDEN_ZONE_4_X_MIN}, {self.FORBIDDEN_ZONE_4_X_MAX}]"
                )
            if self.FORBIDDEN_ZONE_5_X_MIN <= x <= self.FORBIDDEN_ZONE_5_X_MAX:
                violations.append(
                    f"Beam center at ({x:.2f}, {y:.2f}) in forbidden zone 5 "
                    f"x=[{self.FORBIDDEN_ZONE_5_X_MIN}, {self.FORBIDDEN_ZONE_5_X_MAX}]"
                )
            if self.SWEEPER_BAND_1_Y_MIN <= y <= self.SWEEPER_BAND_1_Y_MAX:
                violations.append(
                    f"Beam center at ({x:.2f}, {y:.2f}) in sweeper band 1 "
                    f"y=[{self.SWEEPER_BAND_1_Y_MIN}, {self.SWEEPER_BAND_1_Y_MAX}]"
                )
            if self.SWEEPER_BAND_2_Y_MIN <= y <= self.SWEEPER_BAND_2_Y_MAX:
                violations.append(
                    f"Beam center at ({x:.2f}, {y:.2f}) in sweeper band 2 "
                    f"y=[{self.SWEEPER_BAND_2_Y_MIN}, {self.SWEEPER_BAND_2_Y_MAX}]"
                )
            if self.SWEEPER_BAND_3_Y_MIN <= y <= self.SWEEPER_BAND_3_Y_MAX:
                violations.append(
                    f"Beam center at ({x:.2f}, {y:.2f}) in sweeper band 3 "
                    f"y=[{self.SWEEPER_BAND_3_Y_MIN}, {self.SWEEPER_BAND_3_Y_MAX}]"
                )
            if self.SWEEPER_BAND_4_Y_MIN <= y <= self.SWEEPER_BAND_4_Y_MAX:
                violations.append(
                    f"Beam center at ({x:.2f}, {y:.2f}) in sweeper band 4 "
                    f"y=[{self.SWEEPER_BAND_4_Y_MIN}, {self.SWEEPER_BAND_4_Y_MAX}]"
                )
        return violations
    def _make_metrics(
        self, positions, velocities, step_count, success=False, failed=False,
        failure_reason=None, max_steps=None,
    ):
        if not isinstance(positions, list):
            positions = [positions] if positions else [(0, 0)]
        if not isinstance(velocities, list):
            velocities = [velocities] if velocities else [(0, 0)]
        pos = positions[0] if positions else (0, 0)
        vel = velocities[0] if velocities else (0, 0)
        px, py = pos[0], pos[1]
        vx, vy = vel[0], vel[1]
        speed = (vx * vx + vy * vy) ** 0.5
        in_target = (
            self._target_x_min <= px <= self._target_x_max and
            self._target_y_min <= py <= self._target_y_max
        ) if pos else False
        all_caught = (
            hasattr(self.environment, "get_all_balls_positions") and len(positions) >= 7 and
            len(self._balls_caught) >= len(positions)
        )
        uncaptured_positions = []
        if not all_caught and positions:
            for i in range(len(positions)):
                if i not in self._balls_caught and i < len(positions):
                    pos_i = positions[i]
                    if pos_i:
                        uncaptured_positions.append((i + 1, float(pos_i[0]), float(pos_i[1])))
        structure_mass = self.environment.get_structure_mass()
        mass_budget_used = (structure_mass / self.MAX_STRUCTURE_MASS * 100) if self.MAX_STRUCTURE_MASS > 0 else 0
        terrain = self.environment.get_terrain_bounds() if hasattr(self.environment, "get_terrain_bounds") else {}
        max_joint_force = terrain.get("max_joint_force", 880.0)
        joint_fatigue = terrain.get("joint_fatigue_threshold", 760.0)
        n_beams = len(getattr(self.environment, "_bodies", []))
        per_ball_positions = {}
        per_ball_speeds = {}
        per_ball_caught = {}
        for i in range(len(positions)):
            per_ball_caught[i] = i in self._balls_caught
            if i < len(positions) and positions[i] is not None:
                per_ball_positions[i] = (float(positions[i][0]), float(positions[i][1]))
            if i < len(velocities) and velocities[i] is not None:
                vxi, vyi = float(velocities[i][0]), float(velocities[i][1])
                per_ball_speeds[i] = math.sqrt(vxi * vxi + vyi * vyi)
        ball_margins = []
        for i in range(len(positions)):
            if i in self._balls_caught:
                continue
            if i >= len(positions) or positions[i] is None:
                continue
            bxi, byi = float(positions[i][0]), float(positions[i][1])
            margin_right = self._target_x_max - bxi
            margin_left = bxi - self._target_x_min
            margin_top = self._target_y_max - byi
            margin_bottom = byi - self._target_y_min
            margin_pit = byi - self._pit_y_threshold
            ball_margins.append({
                "ball_idx": i + 1,
                "x": round(bxi, 3),
                "y": round(byi, 3),
                "margin_right": round(margin_right, 3),
                "margin_left": round(margin_left, 3),
                "margin_top": round(margin_top, 3),
                "margin_bottom": round(margin_bottom, 3),
                "margin_pit": round(margin_pit, 3),
            })
        joint_force_data = []
        peak_joint_force = 0.0
        if hasattr(self.environment, "get_joint_reaction_forces"):
            try:
                joint_force_data = self.environment.get_joint_reaction_forces()
                for (_, _, _, mag, _) in joint_force_data:
                    if mag > peak_joint_force:
                        peak_joint_force = mag
            except Exception as exc:
                self._observation_errors.append(
                    f"joint reaction forces unavailable: {type(exc).__name__}: {exc}"
                )
        if hasattr(self.environment, "get_peak_joint_force_seen"):
            try:
                hist_peak = self.environment.get_peak_joint_force_seen()
                if hist_peak > peak_joint_force:
                    peak_joint_force = hist_peak
            except Exception as exc:
                self._observation_errors.append(
                    f"peak joint force unavailable: {type(exc).__name__}: {exc}"
                )
        joint_fatigue_data = []
        if hasattr(self.environment, "get_joint_fatigue_state"):
            try:
                joint_fatigue_data = self.environment.get_joint_fatigue_state()
            except Exception as exc:
                self._observation_errors.append(
                    f"joint fatigue state unavailable: {type(exc).__name__}: {exc}"
                )
        approach_order = sorted(self._ball_approach_step.keys())
        sequential_detail = []
        for idx in approach_order:
            at_step = self._ball_approach_step.get(idx)
            at_speed = self._ball_speed_at_approach.get(idx)
            predecessors_uncaught = []
            for j in range(idx):
                if j not in self._balls_caught:
                    pred_speed = per_ball_speeds.get(j)
                    predecessors_uncaught.append({
                        "predecessor_idx": j + 1,
                        "speed": round(pred_speed, 3) if pred_speed is not None else None,
                    })
            sequential_detail.append({
                "ball_idx": idx + 1,
                "approach_step": at_step,
                "speed_at_approach": round(at_speed, 3) if at_speed is not None else None,
                "caught": idx in self._balls_caught,
                "predecessors_uncaught": predecessors_uncaught,
            })
        metrics = {
            "ball_x": px, "ball_y": py,
            "ball_vx": vx, "ball_vy": vy, "ball_speed": speed,
            "success": success, "failed": failed, "failure_reason": failure_reason,
            "step_count": step_count,
            "max_steps": max_steps,
            "target_x_min": self._target_x_min,
            "target_x_max": self._target_x_max,
            "target_y_min": self._target_y_min,
            "target_y_max": self._target_y_max,
            "caught_speed_threshold": self._caught_speed_threshold,
            "pit_y_threshold": self._pit_y_threshold,
            "pit_speed_threshold": self._pit_speed_threshold,
            "structure_mass": structure_mass,
            "max_structure_mass": self.MAX_STRUCTURE_MASS,
            "mass_budget_used_pct": mass_budget_used,
            "structure_smashed": self.environment.is_structure_smashed(),
            "ball_caught": all_caught,
            "ball_in_catch_zone": in_target,
            "joint_count": len(self.environment._joints),
            "beam_count": n_beams,
            "max_joint_force_limit": max_joint_force,
            "joint_fatigue_threshold": joint_fatigue,
            "ball_speed_vs_threshold": speed - self._caught_speed_threshold,
            "balls_caught_count": len(self._balls_caught),
            "balls_required_count": len(positions),
            "uncaptured_positions": uncaptured_positions if uncaptured_positions else None,
            "pit_failure": getattr(self, "_ball_ever_in_pit_fast", False),
            "sequential_violation": getattr(self, "_sequential_violation", False),
            "approach_x_m": float(self._approach_x),
            "per_ball_positions": per_ball_positions,
            "per_ball_speeds": per_ball_speeds,
            "per_ball_caught": per_ball_caught,
            "ball_margins": ball_margins,
            "joint_force_data": joint_force_data,
            "peak_joint_force": round(peak_joint_force, 3),
            "joint_fatigue_data": joint_fatigue_data,
            "sequential_detail": sequential_detail,
            "observation_errors": list(self._observation_errors) + getattr(
                self.environment, "get_observation_errors", lambda: []
            )(),
        }
        return metrics
    def compute_score_with_penalty(self, score: float, metrics: dict) -> float:
        if score > 0:
            return score
        constraint_info = self.get_constraint_info()
        penalty = compute_constraint_penalty("D_06", score, metrics, constraint_info)
        return penalty
    def get_constraint_info(self):
        return {
            'caught_speed_threshold': self._caught_speed_threshold,
            'pit_y_threshold': self._pit_y_threshold,
            'pit_speed_threshold': self._pit_speed_threshold,
            'max_structure_mass': self.MAX_STRUCTURE_MASS,
            'build_zone_x_min': self.BUILD_ZONE_X_MIN,
            'build_zone_x_max': self.BUILD_ZONE_X_MAX,
            'build_zone_y_min': self.BUILD_ZONE_Y_MIN,
            'build_zone_y_max': self.BUILD_ZONE_Y_MAX,
            'target_x_min': self._target_x_min,
            'target_x_max': self._target_x_max,
            'target_y_min': self._target_y_min,
            'target_y_max': self._target_y_max,
            'forbidden_zone_x_min': self.FORBIDDEN_ZONE_X_MIN,
            'forbidden_zone_x_max': self.FORBIDDEN_ZONE_X_MAX,
            'forbidden_zone_2_x_min': self.FORBIDDEN_ZONE_2_X_MIN,
            'forbidden_zone_2_x_max': self.FORBIDDEN_ZONE_2_X_MAX,
            'forbidden_zone_3_x_min': self.FORBIDDEN_ZONE_3_X_MIN,
            'forbidden_zone_3_x_max': self.FORBIDDEN_ZONE_3_X_MAX,
            'forbidden_zone_4_x_min': self.FORBIDDEN_ZONE_4_X_MIN,
            'forbidden_zone_4_x_max': self.FORBIDDEN_ZONE_4_X_MAX,
            'forbidden_zone_5_x_min': self.FORBIDDEN_ZONE_5_X_MIN,
            'forbidden_zone_5_x_max': self.FORBIDDEN_ZONE_5_X_MAX,
            'max_beam_count': self.MAX_BEAM_COUNT,
            'sweeper_band_1_y_min': self.SWEEPER_BAND_1_Y_MIN,
            'sweeper_band_1_y_max': self.SWEEPER_BAND_1_Y_MAX,
            'sweeper_band_2_y_min': self.SWEEPER_BAND_2_Y_MIN,
            'sweeper_band_2_y_max': self.SWEEPER_BAND_2_Y_MAX,
            'sweeper_band_3_y_min': self.SWEEPER_BAND_3_Y_MIN,
            'sweeper_band_3_y_max': self.SWEEPER_BAND_3_Y_MAX,
            'sweeper_band_4_y_min': self.SWEEPER_BAND_4_Y_MIN,
            'sweeper_band_4_y_max': self.SWEEPER_BAND_4_Y_MAX,
            'max_joint_force': self._max_joint_force,
            'joint_fatigue_threshold': self._joint_fatigue_threshold,
        }
    def get_task_description(self):
        tx0, tx1 = self._target_x_min, self._target_x_max
        ty0, ty1 = self._target_y_min, self._target_y_max
        return {
            "task": "D-06: The Catch (Essential)",
            "description": "Catch seven sequential projectiles inside the target box without structural failure.",
            "success_criteria": {
                "primary": f"All seven balls stabilized in x=[{tx0}, {tx1}], y=[{ty0}, {ty1}] m.",
                "failure": "Pit event, sequential-order violation, structural breakage, or timeout.",
            },
            "evaluation": {"score_range": "0-100", "success_score": 100, "failure_score": 0},
        }
