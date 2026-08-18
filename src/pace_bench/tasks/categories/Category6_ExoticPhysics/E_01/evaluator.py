"""Evaluator for E-01 inverted-gravity containment."""

from __future__ import annotations

import math
from typing import Any, Dict, Iterable, List, Optional, Tuple


class Evaluator:
    def __init__(self, terrain_bounds, environment=None):
        if environment is None:
            raise ValueError("Evaluator requires environment instance")
        self.terrain_bounds = terrain_bounds
        self.environment = environment
        arena = terrain_bounds.get("arena", {})
        self.arena_x_min = float(arena.get("x_min", 0.0))
        self.arena_x_max = float(arena.get("x_max", 40.0))
        self.arena_y_min = float(arena.get("y_min", 0.0))
        self.arena_y_max = float(arena.get("y_max", 20.0))
        build_zone = terrain_bounds.get("build_zone", {})
        build_x = build_zone.get("x", [12.0, 28.0])
        build_y = build_zone.get("y", [6.0, 18.0])
        self.build_x_min, self.build_x_max = map(float, build_x)
        self.build_y_min, self.build_y_max = map(float, build_y)
        self.max_structure_mass = float(environment.MAX_STRUCTURE_MASS)
        self.max_beam_count = int(environment.MAX_BEAM_COUNT)
        self.obstacle_zones = self._rectangles(terrain_bounds.get("obstacles", []))
        self.forbidden_zones = self._rectangles(
            terrain_bounds.get("forbidden_zones", [])
        )
        self.initial_joint_count = 0
        self.structure_broken = False
        self.design_constraints_checked = False
        self.first_structure_break_step = None
        self.first_out_of_bounds_step = None
        self.minimum_arena_margin = float("inf")
        self.initial_build_zone_tightest_margin = None

    @staticmethod
    def _rectangles(raw: Iterable[Dict[str, Any]]) -> List[Dict[str, float]]:
        return [
            {
                "x_min": float(item.get("x_min", 0.0)),
                "x_max": float(item.get("x_max", 0.0)),
                "y_min": float(item.get("y_min", 0.0)),
                "y_max": float(item.get("y_max", 0.0)),
            }
            for item in raw
        ]

    def _dynamic_bodies(self) -> List[Any]:
        bodies = list(self.environment._bodies)
        bodies.extend(
            body
            for name, body in self.environment._terrain_bodies.items()
            if name.startswith("demonstrator_")
        )
        return bodies

    @staticmethod
    def _body_bounds(body: Any) -> Tuple[float, float, float, float]:
        xs: List[float] = []
        ys: List[float] = []
        for fixture in body.fixtures:
            shape = fixture.shape
            if hasattr(shape, "vertices"):
                for vertex in shape.vertices:
                    world_vertex = body.GetWorldPoint(vertex)
                    xs.append(float(world_vertex.x))
                    ys.append(float(world_vertex.y))
            elif hasattr(shape, "radius"):
                local_center = getattr(shape, "pos", (0.0, 0.0))
                center = body.GetWorldPoint(local_center)
                radius = float(shape.radius)
                xs.extend((float(center.x) - radius, float(center.x) + radius))
                ys.extend((float(center.y) - radius, float(center.y) + radius))
        if not xs:
            xs.append(float(body.position.x))
            ys.append(float(body.position.y))
        return min(xs), max(xs), min(ys), max(ys)

    @staticmethod
    def _point_rectangle_margin(
        x: float, y: float, rectangle: Dict[str, float]
    ) -> float:
        if (
            rectangle["x_min"] <= x <= rectangle["x_max"]
            and rectangle["y_min"] <= y <= rectangle["y_max"]
        ):
            return -min(
                x - rectangle["x_min"],
                rectangle["x_max"] - x,
                y - rectangle["y_min"],
                rectangle["y_max"] - y,
            )
        closest_x = min(max(x, rectangle["x_min"]), rectangle["x_max"])
        closest_y = min(max(y, rectangle["y_min"]), rectangle["y_max"])
        return math.hypot(x - closest_x, y - closest_y)

    def evaluate(self, agent_body, step_count, max_steps):
        del agent_body
        if not self.design_constraints_checked:
            self.initial_joint_count = len(self.environment._joints)
            self.initial_build_zone_tightest_margin = min(
                (
                    min(
                        float(body.position.x) - self.build_x_min,
                        self.build_x_max - float(body.position.x),
                        float(body.position.y) - self.build_y_min,
                        self.build_y_max - float(body.position.y),
                    )
                    for body in self.environment._bodies
                ),
                default=None,
            )
            violations = self._check_design_constraints()
            self.design_constraints_checked = True
            if violations:
                metrics = self._collect_metrics(
                    step_count,
                    max_steps,
                    success=False,
                    failed=True,
                    failure_reason="One or more build-time design constraints were violated",
                )
                metrics["constraint_violations"] = violations
                return True, 0.0, metrics

        current_joint_count = len(self.environment._joints)
        if current_joint_count < self.initial_joint_count:
            self.structure_broken = True
            if self.first_structure_break_step is None:
                self.first_structure_break_step = int(step_count)

        out_of_bounds = False
        offending_bounds = []
        for body in self._dynamic_bodies():
            bounds = self._body_bounds(body)
            x_min, x_max, y_min, y_max = bounds
            margin = min(
                x_min - self.arena_x_min,
                self.arena_x_max - x_max,
                y_min - self.arena_y_min,
                self.arena_y_max - y_max,
            )
            self.minimum_arena_margin = min(self.minimum_arena_margin, margin)
            if margin < 0.0:
                out_of_bounds = True
                offending_bounds.append(bounds)
        if out_of_bounds and self.first_out_of_bounds_step is None:
            self.first_out_of_bounds_step = int(step_count)

        obstacle_overlap, obstacle_offending = self._center_overlaps(
            self.obstacle_zones
        )
        forbidden_violation, forbidden_offending = self._center_overlaps(
            self.forbidden_zones
        )
        failed = (
            out_of_bounds
            or self.structure_broken
            or obstacle_overlap
            or forbidden_violation
        )
        step_limit = min(int(max_steps), int(self.environment.MAX_STEPS))
        success = not failed and step_count >= step_limit - 1
        if out_of_bounds:
            reason = "At least one dynamic body fixture left the arena bounds"
        elif forbidden_violation:
            reason = "A beam center entered a forbidden zone"
        elif obstacle_overlap:
            reason = "A beam center entered an obstacle zone"
        elif self.structure_broken:
            reason = "Structure integrity lost (one or more joints broke)"
        else:
            reason = None
        metrics = self._collect_metrics(
            step_count,
            step_limit,
            success=success,
            failed=failed,
            failure_reason=reason,
        )
        metrics["out_of_bounds"] = out_of_bounds
        metrics["offending_body_bounds"] = offending_bounds[:5]
        metrics["obstacle_overlap"] = obstacle_overlap
        metrics["obstacle_offending"] = obstacle_offending[:5]
        metrics["forbidden_zone_violation"] = forbidden_violation
        metrics["forbidden_offending"] = forbidden_offending[:5]
        if success:
            score = 100.0
        elif failed:
            score = 0.0
        else:
            score = step_count / max(step_limit, 1) * 80.0
        return success or failed, score, metrics

    def _center_overlaps(
        self, rectangles: List[Dict[str, float]]
    ) -> Tuple[bool, List[Tuple[float, float]]]:
        offending = []
        for body in self.environment._bodies:
            x, y = float(body.position.x), float(body.position.y)
            if any(
                rectangle["x_min"] <= x <= rectangle["x_max"]
                and rectangle["y_min"] <= y <= rectangle["y_max"]
                for rectangle in rectangles
            ):
                offending.append((x, y))
        return bool(offending), offending

    def _collect_metrics(
        self,
        step_count: int,
        max_steps: int,
        *,
        success: bool,
        failed: bool,
        failure_reason: Optional[str],
    ) -> Dict[str, Any]:
        bodies = self._dynamic_bodies()
        body_bounds = [self._body_bounds(body) for body in bodies]
        centers = [
            (float(body.position.x), float(body.position.y))
            for body in self.environment._bodies
        ]
        obstacle_margins = [
            self._point_rectangle_margin(x, y, rectangle)
            for x, y in centers
            for rectangle in self.obstacle_zones
        ]
        forbidden_margins = [
            self._point_rectangle_margin(x, y, rectangle)
            for x, y in centers
            for rectangle in self.forbidden_zones
        ]
        build_margins = [
            min(
                x - self.build_x_min,
                self.build_x_max - x,
                y - self.build_y_min,
                self.build_y_max - y,
            )
            for x, y in centers
        ]
        obstacle_body_margins = [
            (
                x,
                y,
                min(
                    (
                        self._point_rectangle_margin(x, y, rectangle)
                        for rectangle in self.obstacle_zones
                    ),
                    default=float("inf"),
                ),
            )
            for x, y in centers
        ]
        forbidden_body_margins = [
            (
                x,
                y,
                min(
                    (
                        self._point_rectangle_margin(x, y, rectangle)
                        for rectangle in self.forbidden_zones
                    ),
                    default=float("inf"),
                ),
            )
            for x, y in centers
        ]
        build_body_margins = [
            {
                "pos": (x, y),
                "left_margin": x - self.build_x_min,
                "right_margin": self.build_x_max - x,
                "bottom_margin": y - self.build_y_min,
                "top_margin": self.build_y_max - y,
                "tightest": min(
                    x - self.build_x_min,
                    self.build_x_max - x,
                    y - self.build_y_min,
                    self.build_y_max - y,
                ),
            }
            for x, y in centers
        ]
        tracking = self.environment.get_joint_force_tracking()
        force_history = tracking.get("joint_force_history", [])
        latest_force = force_history[-1] if force_history else None
        energy_history = self.environment.get_kinetic_energy_history()
        energy_values = [
            float(item.get("kinetic_energy", 0.0))
            for item in energy_history
            if isinstance(item, dict)
        ]
        current_arena_margin = min(
            (
                min(
                    bounds[0] - self.arena_x_min,
                    self.arena_x_max - bounds[1],
                    bounds[2] - self.arena_y_min,
                    self.arena_y_max - bounds[3],
                )
                for bounds in body_bounds
            ),
            default=None,
        )
        if current_arena_margin is not None:
            self.minimum_arena_margin = min(
                self.minimum_arena_margin, current_arena_margin
            )
        offending_positions = [
            (float(body.position.x), float(body.position.y))
            for body in bodies
            if min(
                self._body_bounds(body)[0] - self.arena_x_min,
                self.arena_x_max - self._body_bounds(body)[1],
                self._body_bounds(body)[2] - self.arena_y_min,
                self.arena_y_max - self._body_bounds(body)[3],
            )
            < 0.0
        ]
        return {
            "step_count": int(step_count),
            "max_steps": int(max_steps),
            "success": bool(success),
            "failed": bool(failed),
            "failure_reason": failure_reason,
            "structure_broken": self.structure_broken,
            "first_structure_break_step": self.first_structure_break_step,
            "first_out_of_bounds_step": self.first_out_of_bounds_step,
            "joint_count": len(self.environment._joints),
            "initial_joint_count": self.initial_joint_count,
            "beam_count": len(self.environment._bodies),
            "max_beam_count": self.max_beam_count,
            "structure_mass": self.environment.get_structure_mass(),
            "max_structure_mass": self.max_structure_mass,
            "arena_x_min": self.arena_x_min,
            "arena_x_max": self.arena_x_max,
            "arena_y_min": self.arena_y_min,
            "arena_y_max": self.arena_y_max,
            "body_count": len(bodies),
            "body_x_min": min((bounds[0] for bounds in body_bounds), default=None),
            "body_x_max": max((bounds[1] for bounds in body_bounds), default=None),
            "body_y_min": min((bounds[2] for bounds in body_bounds), default=None),
            "body_y_max": max((bounds[3] for bounds in body_bounds), default=None),
            "minimum_arena_margin": (
                self.minimum_arena_margin
                if math.isfinite(self.minimum_arena_margin)
                else None
            ),
            "out_of_bounds": bool(offending_positions),
            "offending_positions": offending_positions[:5],
            "offending_body_bounds": [
                self._body_bounds(body)
                for body in bodies
                if min(
                    self._body_bounds(body)[0] - self.arena_x_min,
                    self.arena_x_max - self._body_bounds(body)[1],
                    self._body_bounds(body)[2] - self.arena_y_min,
                    self.arena_y_max - self._body_bounds(body)[3],
                )
                < 0.0
            ][:5],
            "obstacle_zone_min_margin": min(obstacle_margins, default=None),
            "forbidden_zone_min_margin": min(forbidden_margins, default=None),
            "obstacle_zone_all_margins": [
                item for item in obstacle_body_margins if math.isfinite(item[2])
            ][:5],
            "forbidden_zone_all_margins": [
                item for item in forbidden_body_margins if math.isfinite(item[2])
            ][:5],
            "build_zone_tightest_margin": self.initial_build_zone_tightest_margin,
            "build_zone_body_margins": build_body_margins[:5],
            "build_zone_x_min": self.build_x_min,
            "build_zone_x_max": self.build_x_max,
            "build_zone_y_min": self.build_y_min,
            "build_zone_y_max": self.build_y_max,
            "joint_failure_events": list(
                tracking.get("joint_failure_events", [])
            ),
            "joint_tracking": tracking,
            "latest_joint_force_summary": latest_force,
            "peak_reaction_force_ever": self.environment.get_peak_reaction_force_ever(),
            "peak_body_velocity": self.environment.get_peak_body_velocity(),
            "kinetic_energy_initial": energy_values[0] if energy_values else None,
            "kinetic_energy_current": energy_values[-1] if energy_values else None,
            "kinetic_energy_peak": max(energy_values) if energy_values else None,
            "kinetic_energy_history": energy_history,
            "progress_pct": 100.0 * step_count / max(max_steps, 1),
        }

    def _check_design_constraints(self) -> List[str]:
        violations = []
        mass = self.environment.get_structure_mass()
        if mass > self.max_structure_mass:
            violations.append(
                f"Structure mass {mass:.6g} kg exceeds maximum "
                f"{self.max_structure_mass:.6g} kg"
            )
        beam_count = len(self.environment._bodies)
        if beam_count > self.max_beam_count:
            violations.append(
                f"Structure has {beam_count} beams, exceeds maximum "
                f"{self.max_beam_count}"
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
        return violations

    def get_task_description(self):
        return {
            "task": "E-01: Inverted Gravity",
            "description": (
                "Keep every dynamic fixture inside the arena while preserving all "
                "agent-created joints under time-varying gravity."
            ),
            "terrain": self.terrain_bounds,
            "success_criteria": {
                "primary": (
                    f"No dynamic fixture leaves x=[{self.arena_x_min}, "
                    f"{self.arena_x_max}], y=[{self.arena_y_min}, "
                    f"{self.arena_y_max}]"
                ),
                "secondary": "All agent-created joints remain intact",
            },
            "evaluation": {
                "score_range": "0-100",
                "success_score": 100,
                "failure_score": 0,
            },
        }
