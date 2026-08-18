"""Evaluator for E-06 cantilever endurance."""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple


class Evaluator:
    SPAN_X_LEFT = 7.0
    SPAN_X_RIGHT = 13.0
    MIN_HEIGHT_Y = 5.0
    def __init__(self, terrain_bounds, environment=None):
        if environment is None:
            raise ValueError("Evaluator requires environment instance")
        self.terrain_bounds = terrain_bounds
        self.environment = environment
        self.initial_joint_count: Optional[int] = None
        self.initial_body_count: Optional[int] = None
        self.initial_structure_mass: Optional[float] = None
        self.initial_ground_anchor_count: Optional[int] = None
        self.initial_min_x: Optional[float] = None
        self.initial_max_x: Optional[float] = None
        self.initial_max_y: Optional[float] = None
        self.initial_span_check_passed: Optional[bool] = None
        self.structure_broken = False
        self.design_constraints_checked = False
        self.max_structure_mass = float(
            terrain_bounds.get(
                "max_structure_mass", environment.MAX_STRUCTURE_MASS
            )
        )
        build_zone = terrain_bounds.get("build_zone", {})
        build_x = build_zone.get(
            "x", [environment.BUILD_ZONE_X_MIN, environment.BUILD_ZONE_X_MAX]
        )
        build_y = build_zone.get(
            "y", [environment.BUILD_ZONE_Y_MIN, environment.BUILD_ZONE_Y_MAX]
        )
        self.build_x_min, self.build_x_max = map(float, build_x)
        self.build_y_min, self.build_y_max = map(float, build_y)
        self.forbidden_zone = list(
            terrain_bounds.get("forbidden_zone", [9.7, 10.3])
        )
        self.allowed_anchor_zone = list(
            terrain_bounds.get("allowed_anchor_zone", [5.0, 6.5])
        )
        self.max_ground_anchors = int(
            terrain_bounds.get("max_ground_anchors", 1)
        )

    def evaluate(self, agent_body, step_count, max_steps):
        del agent_body
        if not self.design_constraints_checked:
            self.initial_joint_count = len(self.environment._joints)
            self.initial_body_count = len(self.environment._bodies)
            self.initial_structure_mass = self.environment.get_structure_mass()
            self.initial_ground_anchor_count = self.environment._ground_anchor_count
            self.initial_min_x = min(
                (float(body.position.x) for body in self.environment._bodies),
                default=None,
            )
            self.initial_max_x = max(
                (float(body.position.x) for body in self.environment._bodies),
                default=None,
            )
            self.initial_max_y = max(
                (float(body.position.y) for body in self.environment._bodies),
                default=None,
            )
            self.initial_span_check_passed = self._check_span()[0]
            violations = self._check_design_constraints()
            self.design_constraints_checked = True
            if violations:
                metrics = self._metrics(
                    step_count,
                    max_steps,
                    success=False,
                    failed=True,
                    failure_reason=(
                        "Design constraint violation: one or more build-time "
                        "requirements were not met"
                    ),
                )
                metrics["constraint_violations"] = violations
                return True, 0.0, metrics

        current_joint_count = len(self.environment._joints)
        current_body_count = len(self.environment._bodies)
        if (
            current_joint_count < (self.initial_joint_count or 0)
            or current_body_count < (self.initial_body_count or 0)
        ):
            self.structure_broken = True

        span_ok, span_message = self._check_span()
        step_limit = min(int(max_steps), int(self.environment.MAX_STEPS))
        run_complete = step_count >= step_limit
        failed = self.structure_broken
        success = run_complete and not failed
        if self.structure_broken:
            failure_reason = (
                "Structure lost one or more joints or beams during the endurance run"
            )
        else:
            failure_reason = None

        if success:
            score = 100.0
        elif failed:
            score = 0.0
        else:
            score = 80.0 * step_count / max(step_limit, 1)
        metrics = self._metrics(
            step_count,
            step_limit,
            success=success,
            failed=failed,
            failure_reason=failure_reason,
        )
        return success or failed, score, metrics

    def _metrics(
        self,
        step_count: int,
        max_steps: int,
        *,
        success: bool,
        failed: bool,
        failure_reason: Optional[str],
    ) -> Dict[str, Any]:
        current_joint_count = len(self.environment._joints)
        current_body_count = len(self.environment._bodies)
        span_ok, span_message = self._check_span()
        forensic = self.environment._get_forensic_summary()
        per_joint = forensic.get("per_joint_stress_data", [])
        force_values = [
            float(item.get("peak_force", 0.0))
            for item in per_joint
            if isinstance(item, dict)
        ]
        torque_values = [
            float(item.get("peak_torque", 0.0))
            for item in per_joint
            if isinstance(item, dict)
        ]
        damage_values = [
            float(item.get("damage", 0.0))
            for item in per_joint
            if isinstance(item, dict)
        ]
        return {
            "step_count": int(step_count),
            "max_steps": min(int(max_steps), int(self.environment.MAX_STEPS)),
            "success": bool(success),
            "failed": bool(failed),
            "failure_reason": failure_reason,
            "structure_broken": self.structure_broken,
            "joint_count": current_joint_count,
            "initial_joint_count": self.initial_joint_count,
            "body_count": current_body_count,
            "initial_body_count": self.initial_body_count,
            "structure_mass": self.environment.get_structure_mass(),
            "initial_structure_mass": self.initial_structure_mass,
            "max_structure_mass": self.max_structure_mass,
            "ground_anchor_count": self.environment._ground_anchor_count,
            "initial_ground_anchor_count": self.initial_ground_anchor_count,
            "required_ground_anchor_count": 1,
            "allowed_anchor_x_min": float(self.allowed_anchor_zone[0]),
            "allowed_anchor_x_max": float(self.allowed_anchor_zone[1]),
            "initial_min_x": self.initial_min_x,
            "initial_max_x": self.initial_max_x,
            "initial_max_y": self.initial_max_y,
            "max_joint_force": max(force_values, default=0.0),
            "max_joint_torque": max(torque_values, default=0.0),
            "joint_break_force": float(self.environment.JOINT_BREAK_FORCE),
            "joint_break_torque": float(self.environment.JOINT_BREAK_TORQUE),
            "max_joint_damage": max(damage_values, default=0.0),
            "damage_limit": float(self.environment.DAMAGE_LIMIT),
            "span_check_passed": span_ok,
            "initial_span_check_passed": self.initial_span_check_passed,
            "span_check_message": span_message,
            "span_x_left_required": self.SPAN_X_LEFT,
            "span_x_right_required": self.SPAN_X_RIGHT,
            "minimum_height_required": self.MIN_HEIGHT_Y,
            "current_min_x": min(
                (float(body.position.x) for body in self.environment._bodies),
                default=None,
            ),
            "current_max_x": max(
                (float(body.position.x) for body in self.environment._bodies),
                default=None,
            ),
            "current_max_y": max(
                (float(body.position.y) for body in self.environment._bodies),
                default=None,
            ),
            "first_joint_fail_step": forensic.get("first_joint_fail_step"),
            "first_joint_fail_pos": forensic.get("first_joint_fail_pos"),
            "first_joint_fail_type": forensic.get("first_joint_fail_type"),
            "first_body_fail_step": forensic.get("first_body_fail_step"),
            "first_body_fail_pos": forensic.get("first_body_fail_pos"),
            "first_body_fail_reason": forensic.get("first_body_fail_reason"),
            "num_joints_removed_force_torque": forensic.get(
                "num_joints_removed_force_torque", 0
            ),
            "num_joints_removed_damage": forensic.get(
                "num_joints_removed_damage", 0
            ),
            "total_joints_removed": forensic.get("total_joints_removed", 0),
            "num_bodies_destroyed_spin": forensic.get(
                "num_bodies_destroyed_spin", 0
            ),
            "num_bodies_destroyed_orphan": forensic.get(
                "num_bodies_destroyed_orphan", 0
            ),
            "total_bodies_destroyed": forensic.get("total_bodies_destroyed", 0),
            "peak_body_angvel": forensic.get("peak_body_angvel", 0.0),
            "beam_angvel_thresh": float(self.environment.BEAM_ANGVEL_THRESH),
            "beam_angvel_tolerance_steps": int(
                self.environment.BEAM_ANGVEL_TOLERANCE_STEPS
            ),
            "worst_spin_body_pos": forensic.get("worst_spin_body_pos"),
            "worst_spin_body_peak": forensic.get("worst_spin_body_peak", 0.0),
            "worst_spin_consec_steps": forensic.get(
                "worst_spin_consec_steps", 0
            ),
            "forensic_step_counter": forensic.get("step_counter", step_count),
            "failure_event_timeline": forensic.get(
                "failure_event_timeline", []
            ),
            "per_body_angvel_data": forensic.get("per_body_angvel_data", []),
            "per_joint_stress_data": per_joint,
        }

    def _check_span(self) -> Tuple[bool, str]:
        if not self.environment._bodies:
            return False, "Structure has no active beams"
        xs = [float(body.position.x) for body in self.environment._bodies]
        ys = [float(body.position.y) for body in self.environment._bodies]
        if not any(x <= self.SPAN_X_LEFT for x in xs):
            return False, f"Structure does not reach x <= {self.SPAN_X_LEFT}"
        if not any(x >= self.SPAN_X_RIGHT for x in xs):
            return False, f"Structure does not reach x >= {self.SPAN_X_RIGHT}"
        if not any(y >= self.MIN_HEIGHT_Y for y in ys):
            return False, f"Structure does not reach y >= {self.MIN_HEIGHT_Y}"
        return True, "All span and height requirements are met"

    def _check_design_constraints(self) -> List[str]:
        violations = []
        mass = self.environment.get_structure_mass()
        if mass > self.max_structure_mass:
            violations.append(
                f"Structure mass {mass:.6g} kg exceeds maximum "
                f"{self.max_structure_mass:.6g} kg"
            )
        if self.environment._ground_anchor_count != 1:
            violations.append(
                f"Structure has {self.environment._ground_anchor_count} ground "
                "anchors; exactly 1 is required"
            )
        for body in self.environment._bodies:
            x, y = float(body.position.x), float(body.position.y)
            if not (
                self.build_x_min <= x <= self.build_x_max
                and self.build_y_min <= y <= self.build_y_max
            ):
                violations.append(
                    f"Beam center ({x:.2f}, {y:.2f}) is outside build zone"
                )
        span_ok, span_message = self._check_span()
        if not span_ok:
            violations.append(span_message)
        return violations

    def get_task_description(self):
        return {
            "task": "E-06: Cantilever Endurance",
            "description": (
                "Build a one-anchor cantilever that retains every beam and joint "
                "through the endurance interval."
            ),
            "terrain": self.terrain_bounds,
            "success_criteria": {
                "primary": "No joint or beam is lost",
                "span": (
                    f"Beam centers reach x<={self.SPAN_X_LEFT} and "
                    f"x>={self.SPAN_X_RIGHT}"
                ),
                "height": f"At least one beam center reaches y>={self.MIN_HEIGHT_Y}",
                "mass": f"Initial total mass <= {self.max_structure_mass} kg",
                "anchors": "Exactly one ground anchor",
            },
            "evaluation": {
                "score_range": "0-100",
                "success_score": 100,
                "failure_score": 0,
            },
        }
