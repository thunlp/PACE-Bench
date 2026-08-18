import math

from pace_bench.core.primitives import compute_constraint_penalty


class Evaluator:
    """Evaluate the three published F-05 success conditions.

    Cargo loss, capsize, and weld failure are episode-history conditions.  A
    late recovery therefore cannot erase an earlier failure.
    """

    def __init__(self, terrain_bounds, environment=None):
        if environment is None:
            raise ValueError("Evaluator requires environment instance")
        self.terrain_bounds = terrain_bounds
        self.environment = environment
        self.initial_joint_count = 0
        self.structure_broken = False
        self.design_constraints_checked = False
        self.MAX_STRUCTURE_MASS = float(environment.MAX_STRUCTURE_MASS)
        self.BUILD_ZONE_X_MIN = float(environment.BUILD_ZONE_X_MIN)
        self.BUILD_ZONE_X_MAX = float(environment.BUILD_ZONE_X_MAX)
        self.BUILD_ZONE_Y_MIN = float(environment.BUILD_ZONE_Y_MIN)
        self.BUILD_ZONE_Y_MAX = float(environment.BUILD_ZONE_Y_MAX)
        self.BOAT_MAX_ANGLE_RAD = float(environment.BOAT_MAX_ANGLE_RAD)
        self.CARGO_WATER_Y = float(environment.CARGO_WATER_Y)

    def evaluate(self, agent_body, step_count, max_steps):
        if not self.design_constraints_checked:
            violations = self._check_design_constraints()
            self.design_constraints_checked = True
            self.initial_joint_count = len(self.environment._joints)
            if violations:
                metrics = self._collect_metrics(
                    step_count,
                    max_steps=max_steps,
                    success=False,
                    failed=True,
                    failure_reason=(
                        "Design constraint violated: " + "; ".join(violations)
                    ),
                )
                metrics["constraint_violations"] = violations
                return True, 0.0, metrics

        if len(self.environment._joints) < self.initial_joint_count:
            self.structure_broken = True

        if step_count < max_steps:
            return False, 0.0, self._collect_metrics(
                step_count,
                max_steps=max_steps,
                success=False,
                failed=False,
                failure_reason=None,
            )

        initial_cargo = self.environment.get_initial_cargo_count()
        cargo_lost = self.environment.get_cargo_ever_below_loss_plane_count()
        peak_angle = self.environment.get_peak_abs_boat_angle_rad()
        capsize_step = self.environment.get_capsize_first_step()
        cargo_failed = cargo_lost > 0
        capsize_failed = (
            capsize_step is not None or peak_angle > self.BOAT_MAX_ANGLE_RAD
        )

        reasons = []
        if cargo_failed:
            reasons.append(
                f"{cargo_lost}/{initial_cargo} cargo particles crossed below "
                f"y={self.CARGO_WATER_Y:.2f} m after the grace window"
            )
        if capsize_failed:
            reasons.append(
                f"Peak hull roll {math.degrees(peak_angle):.2f} deg exceeded "
                f"{math.degrees(self.BOAT_MAX_ANGLE_RAD):.2f} deg"
            )
        if self.structure_broken:
            reasons.append("Structure integrity lost (one or more welds broke)")

        failed = bool(reasons)
        success = not failed
        failure_reason = "; ".join(reasons) if reasons else None
        metrics = self._collect_metrics(
            step_count,
            max_steps=max_steps,
            success=success,
            failed=failed,
            failure_reason=failure_reason,
        )
        return True, 100.0 if success else 0.0, metrics

    def _collect_metrics(
        self,
        step_count,
        *,
        max_steps,
        success=False,
        failed=False,
        failure_reason=None,
    ):
        environment = self.environment
        initial_cargo = environment.get_initial_cargo_count()
        cargo_currently_below = environment.get_cargo_in_water_count()
        cargo_lost = environment.get_cargo_ever_below_loss_plane_count()
        cargo_retained = max(0, initial_cargo - cargo_lost)
        cargo_retained_ratio = (
            cargo_retained / initial_cargo if initial_cargo > 0 else None
        )
        boat_angle = environment.get_boat_angle()
        boat_angle_deg = (
            math.degrees(abs(float(boat_angle))) if boat_angle is not None else None
        )
        peak_angle = environment.get_peak_abs_boat_angle_rad()
        boat_position = environment.get_boat_position()
        cargo_lowest_y = environment.get_cargo_lowest_y()
        cargo_loss_margin = (
            cargo_lowest_y - self.CARGO_WATER_Y
            if cargo_lowest_y is not None and math.isfinite(cargo_lowest_y)
            else None
        )
        joint_peak_force = environment.get_joint_peak_force()
        joint_peak_torque = environment.get_joint_peak_torque()
        joint_max_force = environment.get_joint_max_force()
        joint_max_torque = environment.get_joint_max_torque()
        joint_peak_force_pct = (
            joint_peak_force / joint_max_force * 100.0
            if math.isfinite(joint_max_force) and joint_max_force > 0.0
            else None
        )
        joint_peak_torque_pct = (
            joint_peak_torque / joint_max_torque * 100.0
            if math.isfinite(joint_max_torque) and joint_max_torque > 0.0
            else None
        )
        current_joint_count = len(environment._joints)
        initial_joint_count = max(self.initial_joint_count, current_joint_count)

        return {
            "step_count": step_count,
            "max_steps": max_steps,
            "success": bool(success),
            "failed": bool(failed),
            "failure_reason": failure_reason,
            "initial_cargo_count": initial_cargo,
            # Backward-compatible decisive count: particles that ever breached.
            "cargo_in_water": cargo_lost,
            "cargo_lost_count": cargo_lost,
            "cargo_currently_below_loss_plane": cargo_currently_below,
            "cargo_retained": cargo_retained,
            "cargo_retained_ratio": cargo_retained_ratio,
            "cargo_water_y": self.CARGO_WATER_Y,
            "cargo_ever_below_loss_plane": cargo_lost > 0,
            "cargo_loss_first_step": environment.get_cargo_loss_first_step(),
            "cargo_lowest_y": cargo_lowest_y,
            "cargo_lowest_y_step": environment.get_cargo_lowest_y_step(),
            "cargo_loss_margin": cargo_loss_margin,
            "cargo_retention_milestones": environment.get_cargo_retention_milestones(),
            "cargo_grace_steps": environment.get_grace_steps(),
            "boat_angle_rad": boat_angle,
            "boat_angle_deg": boat_angle_deg,
            "peak_abs_boat_angle_rad": peak_angle,
            "peak_abs_boat_angle_deg": math.degrees(peak_angle),
            "boat_max_angle_deg": math.degrees(self.BOAT_MAX_ANGLE_RAD),
            "capsize_first_step": environment.get_capsize_first_step(),
            "capsize_margin_at_grace_end_rad": (
                environment.get_capsize_margin_at_grace_end()
            ),
            "peak_angular_velocity_rad_s": (
                environment.get_peak_angular_velocity_rad_s()
            ),
            "boat_x": boat_position[0] if boat_position else None,
            "boat_y": boat_position[1] if boat_position else None,
            "structure_mass": environment.get_structure_mass(),
            "max_structure_mass": self.MAX_STRUCTURE_MASS,
            "structure_broken": self.structure_broken,
            "initial_joint_count": initial_joint_count,
            "joint_count": current_joint_count,
            "broken_joint_count": max(0, initial_joint_count - current_joint_count),
            "first_joint_break_step": environment.get_first_joint_break_step(),
            "joint_peak_force_N": joint_peak_force,
            "joint_max_force_N": joint_max_force,
            "joint_peak_force_pct": joint_peak_force_pct,
            "joint_peak_torque_Nm": joint_peak_torque,
            "joint_max_torque_Nm": joint_max_torque,
            "joint_peak_torque_pct": joint_peak_torque_pct,
            "lowest_beam_y_floor_margin": (
                environment.get_lowest_beam_y_floor_margin()
            ),
            "build_zone_x_min": self.BUILD_ZONE_X_MIN,
            "build_zone_x_max": self.BUILD_ZONE_X_MAX,
            "build_zone_y_min": self.BUILD_ZONE_Y_MIN,
            "build_zone_y_max": self.BUILD_ZONE_Y_MAX,
            "constraint_info": self.get_constraint_info(),
        }

    def _check_design_constraints(self):
        violations = []
        structure_mass = self.environment.get_structure_mass()
        if structure_mass > self.MAX_STRUCTURE_MASS:
            violations.append(
                f"Structure mass {structure_mass:.2f} kg exceeds maximum "
                f"{self.MAX_STRUCTURE_MASS:.2f} kg"
            )
        for body in self.environment._bodies:
            x, y = body.position.x, body.position.y
            if not (
                self.BUILD_ZONE_X_MIN <= x <= self.BUILD_ZONE_X_MAX
                and self.BUILD_ZONE_Y_MIN <= y <= self.BUILD_ZONE_Y_MAX
            ):
                violations.append(
                    f"Beam at ({x:.2f}, {y:.2f}) is outside build zone "
                    f"x=[{self.BUILD_ZONE_X_MIN}, {self.BUILD_ZONE_X_MAX}], "
                    f"y=[{self.BUILD_ZONE_Y_MIN}, {self.BUILD_ZONE_Y_MAX}]"
                )
        return violations

    def compute_score_with_penalty(self, score: float, metrics: dict) -> float:
        if score > 0:
            return score
        return compute_constraint_penalty(
            "F_05", score, metrics, self.get_constraint_info()
        )

    def get_constraint_info(self):
        return {
            "max_structure_mass": self.MAX_STRUCTURE_MASS,
            "build_zone_x_min": self.BUILD_ZONE_X_MIN,
            "build_zone_x_max": self.BUILD_ZONE_X_MAX,
            "build_zone_y_min": self.BUILD_ZONE_Y_MIN,
            "build_zone_y_max": self.BUILD_ZONE_Y_MAX,
            "boat_max_angle_rad": self.BOAT_MAX_ANGLE_RAD,
            "cargo_water_y": self.CARGO_WATER_Y,
        }

    def get_task_description(self):
        return {
            "task": "F-05: The Boat",
            "description": "Keep all cargo aboard and the hull upright.",
            "terrain": self.terrain_bounds,
            "success_criteria": {
                "primary": (
                    "No cargo crosses below y={:.2f} m after grace".format(
                        self.CARGO_WATER_Y
                    )
                ),
                "secondary": (
                    "Peak hull roll <= "
                    f"{math.degrees(self.BOAT_MAX_ANGLE_RAD):.0f} deg"
                ),
                "tertiary": "All welds survive",
            },
            "evaluation": {
                "score_range": "0-100",
                "success_score": 100,
                "failure_score": 0,
            },
        }
