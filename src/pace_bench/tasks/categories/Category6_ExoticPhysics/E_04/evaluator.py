import math


class Evaluator:
    def __init__(self, terrain_bounds, environment=None):
        if environment is None:
            raise ValueError("Evaluator requires environment instance")
        self.terrain_bounds = terrain_bounds
        self.environment = environment
        self.max_structure_mass = float(terrain_bounds["max_structure_mass"])
        build_zone = terrain_bounds["build_zone"]
        self.build_x_min, self.build_x_max = map(float, build_zone["x"])
        self.build_y_min, self.build_y_max = map(float, build_zone["y"])
        self.min_beams = int(terrain_bounds["min_beams"])
        self.min_joints = int(terrain_bounds["min_joints"])
        self.span_left = float(terrain_bounds["span_left_x"])
        self.span_right = float(terrain_bounds["span_right_x"])
        self.initial_joint_count = None
        self.design_violations = None
        self.structure_broken = False

    def _design_state(self):
        beam_count, joint_count = self.environment.get_structure_counts()
        positions = self.environment.get_structure_positions()
        joint_types = self.environment.get_joint_types()
        xs = [position[0] for position in positions]
        out_of_zone = sum(
            not (
                self.build_x_min <= x <= self.build_x_max
                and self.build_y_min <= y <= self.build_y_max
            )
            for x, y in positions
        )
        return {
            "beam_count": beam_count,
            "joint_count": joint_count,
            "pivot_joint_count": sum(kind == "pivot" for kind in joint_types),
            "span_min_x": min(xs) if xs else None,
            "span_max_x": max(xs) if xs else None,
            "out_of_zone_beam_count": out_of_zone,
        }

    def _check_design_constraints(self, state, mass):
        violations = []
        if not math.isfinite(mass):
            violations.append("structure mass is non-finite")
        elif mass > self.max_structure_mass:
            violations.append(
                f"structure mass {mass:.3f} kg exceeds "
                f"{self.max_structure_mass:.3f} kg"
            )
        if state["beam_count"] < self.min_beams:
            violations.append(
                f"{state['beam_count']} beams provided; {self.min_beams} required"
            )
        if state["joint_count"] < self.min_joints:
            violations.append(
                f"{state['joint_count']} joints provided; {self.min_joints} required"
            )
        if state["pivot_joint_count"] < 1:
            violations.append("no pivot joint was provided")
        if (
            state["span_min_x"] is None
            or state["span_min_x"] > self.span_left
        ):
            violations.append(
                f"no beam center reaches x <= {self.span_left:.1f} m"
            )
        if (
            state["span_max_x"] is None
            or state["span_max_x"] < self.span_right
        ):
            violations.append(
                f"no beam center reaches x >= {self.span_right:.1f} m"
            )
        if state["out_of_zone_beam_count"]:
            violations.append(
                f"{state['out_of_zone_beam_count']} beam center(s) outside build zone"
            )
        return violations

    def evaluate(self, agent_body, step_count, max_steps):
        del agent_body
        effective_max_steps = min(int(max_steps), int(self.environment.MAX_STEPS))
        state = self._design_state()
        mass = float(self.environment.get_structure_mass())
        peak_mass = float(self.environment.get_peak_structure_mass())
        first_mass_violation_step = (
            self.environment.get_first_mass_violation_step()
        )

        if self.design_violations is None:
            self.design_violations = self._check_design_constraints(state, mass)
            self.initial_joint_count = state["joint_count"]

        mass_over_budget = (
            not math.isfinite(mass)
            or not math.isfinite(peak_mass)
            or first_mass_violation_step is not None
            or peak_mass > self.max_structure_mass
        )
        if (
            self.initial_joint_count is not None
            and state["joint_count"] < self.initial_joint_count
        ):
            self.structure_broken = True

        peak_force = float(self.environment.get_max_joint_reaction_force())
        peak_torque = float(self.environment.get_max_joint_reaction_torque())
        peak_speed = float(self.environment.get_peak_body_speed())
        numerical_failure = not all(
            math.isfinite(value)
            for value in (mass, peak_mass, peak_force, peak_torque, peak_speed)
        )
        design_failed = bool(self.design_violations)
        failed = (
            design_failed
            or self.structure_broken
            or mass_over_budget
            or numerical_failure
        )
        terminal_step = step_count >= effective_max_steps - 1
        success = terminal_step and not failed

        if design_failed:
            failure_reason = "Design constraint violation: " + "; ".join(
                self.design_violations
            )
        elif numerical_failure:
            failure_reason = "Non-finite structure state detected"
        elif mass_over_budget:
            failure_reason = (
                f"Structure mass exceeded {self.max_structure_mass:.3f} kg"
            )
        elif self.structure_broken:
            failure_reason = "One or more joints broke"
        else:
            failure_reason = None

        score = 100.0 if success else (
            0.0
            if failed
            else 80.0 * step_count / max(effective_max_steps, 1)
        )
        metrics = {
            "step_count": step_count,
            "max_steps": effective_max_steps,
            "steps_remaining": max(0, effective_max_steps - step_count),
            "success": success,
            "failed": failed,
            "failure_reason": failure_reason,
            "design_violations": list(self.design_violations),
            "structure_broken": self.structure_broken,
            "beam_count": state["beam_count"],
            "joint_count": state["joint_count"],
            "initial_joint_count": self.initial_joint_count,
            "pivot_joint_count": state["pivot_joint_count"],
            "min_beams": self.min_beams,
            "min_joints": self.min_joints,
            "span_min_x": state["span_min_x"],
            "span_max_x": state["span_max_x"],
            "required_span_left_x": self.span_left,
            "required_span_right_x": self.span_right,
            "out_of_zone_beam_count": state["out_of_zone_beam_count"],
            "structure_mass": mass,
            "peak_structure_mass": peak_mass,
            "max_structure_mass": self.max_structure_mass,
            "first_mass_violation_step": first_mass_violation_step,
            "max_joint_reaction_force": peak_force,
            "max_joint_reaction_torque": peak_torque,
            "effective_joint_force_limit": (
                self.environment.get_effective_joint_force_limit()
            ),
            "effective_joint_torque_limit": (
                self.environment.get_effective_joint_torque_limit()
            ),
            "simulation_time_s": self.environment.get_simulation_time(),
            "fatigue_factor": self.environment.get_current_fatigue_factor(),
            "peak_body_speed": peak_speed,
            "joints_ever_broken": self.environment.get_joints_ever_broken(),
            "per_joint_peaks": self.environment.get_per_joint_peaks(),
            "closest_joint_margin_events": (
                self.environment.get_closest_joint_margin_events()
            ),
        }
        return failed or terminal_step, score, metrics

    def get_task_description(self):
        return {
            "task": "E-04: Variable Mass",
            "description": (
                "Design a structure that remains intact for the full run "
                "under time-varying loading."
            ),
            "terrain": self.terrain_bounds,
            "success_criteria": {
                "primary": "All joints remain intact for the full run",
                "span": (
                    f"Beam centers span x <= {self.span_left:.1f} m "
                    f"through x >= {self.span_right:.1f} m"
                ),
                "complexity": (
                    f"At least {self.min_beams} beams, {self.min_joints} joints, "
                    "and one pivot joint"
                ),
                "mass": (
                    f"Instantaneous structure mass <= "
                    f"{self.max_structure_mass:.1f} kg"
                ),
            },
            "evaluation": {
                "score_range": "0-100",
                "success_score": 100,
                "failure_score": 0,
            },
        }
