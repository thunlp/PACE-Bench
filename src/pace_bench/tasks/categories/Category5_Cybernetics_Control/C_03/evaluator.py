import math

import os

import importlib.util

_c03_eval_dir = os.path.dirname(os.path.abspath(__file__))

_spec_c03_env = importlib.util.spec_from_file_location(
    "c03_environment_eval", os.path.join(_c03_eval_dir, "environment.py")

)

_c03_environment = importlib.util.module_from_spec(_spec_c03_env)

_spec_c03_env.loader.exec_module(_c03_environment)

ACTIVATION_ZONE_X_MIN = _c03_environment.ACTIVATION_ZONE_X_MIN

ACTIVATION_ZONE_X_MAX = _c03_environment.ACTIVATION_ZONE_X_MAX

ACTIVATION_REQUIRED_STEPS = _c03_environment.ACTIVATION_REQUIRED_STEPS

SLOTS_PHASE1 = _c03_environment.SLOTS_PHASE1

SLOTS_PHASE2 = _c03_environment.SLOTS_PHASE2

RENDEZVOUS_ZONE_X_MIN = _c03_environment.RENDEZVOUS_ZONE_X_MIN

RENDEZVOUS_ZONE_X_MAX = _c03_environment.RENDEZVOUS_ZONE_X_MAX

RENDEZVOUS_DISTANCE_DEFAULT = _c03_environment.RENDEZVOUS_DISTANCE_DEFAULT

RENDEZVOUS_REL_SPEED_DEFAULT = _c03_environment.RENDEZVOUS_REL_SPEED_DEFAULT

TRACK_DISTANCE_DEFAULT = _c03_environment.TRACK_DISTANCE_DEFAULT

RENDEZVOUS_HEADING_TOLERANCE_DEG_DEFAULT = _c03_environment.RENDEZVOUS_HEADING_TOLERANCE_DEG_DEFAULT

HEADING_REFERENCE_MIN_TARGET_SPEED = _c03_environment.HEADING_REFERENCE_MIN_TARGET_SPEED

RENDEZVOUS_DISTANCE_DEF = RENDEZVOUS_DISTANCE_DEFAULT

RENDEZVOUS_REL_SPEED_DEF = RENDEZVOUS_REL_SPEED_DEFAULT

TRACK_DISTANCE_DEF = TRACK_DISTANCE_DEFAULT

RENDEZVOUS_HEADING_TOLERANCE_DEG_DEF = RENDEZVOUS_HEADING_TOLERANCE_DEG_DEFAULT

from pace_bench.core.primitives import compute_constraint_penalty

class Evaluator:
    def __init__(self, terrain_bounds, environment=None):
        self.environment = environment
        if hasattr(terrain_bounds, "get_terrain_bounds"):
            self.terrain_bounds = terrain_bounds.get_terrain_bounds()
            if self.environment is None:
                self.environment = terrain_bounds
        else:
            self.terrain_bounds = terrain_bounds or {}
        self.rendezvous_distance = float(
            self.terrain_bounds.get("rendezvous_distance", RENDEZVOUS_DISTANCE_DEF)
        )
        self.rendezvous_rel_speed = float(
            self.terrain_bounds.get("rendezvous_rel_speed", RENDEZVOUS_REL_SPEED_DEF)
        )
        self.slots_phase1 = self.terrain_bounds.get("slots_phase1", SLOTS_PHASE1)
        self.slots_phase2 = self.terrain_bounds.get("slots_phase2", SLOTS_PHASE2)
        self.window1_lo = min(s[0] for s in self.slots_phase1)
        self.window1_hi = max(s[1] for s in self.slots_phase1)
        self.window2_lo = min(s[0] for s in self.slots_phase2)
        self.window2_hi = max(s[1] for s in self.slots_phase2)
        heading_deg = float(
            self.terrain_bounds.get(
                "rendezvous_heading_tolerance_deg", RENDEZVOUS_HEADING_TOLERANCE_DEG_DEF
            )
        )
        self.heading_tolerance_rad = math.radians(heading_deg)
        self.heading_tolerance_deg = heading_deg
        self.track_distance = float(
            self.terrain_bounds.get("track_distance", TRACK_DISTANCE_DEF)
        )
        self.rendezvous_zone_x_min = float(
            self.terrain_bounds.get("rendezvous_zone_x_min", RENDEZVOUS_ZONE_X_MIN)
        )
        self.rendezvous_zone_x_max = float(
            self.terrain_bounds.get("rendezvous_zone_x_max", RENDEZVOUS_ZONE_X_MAX)
        )
        self.activation_zone_x_min = float(
            self.terrain_bounds.get("activation_zone_x_min", ACTIVATION_ZONE_X_MIN)
        )
        self.activation_zone_x_max = float(
            self.terrain_bounds.get("activation_zone_x_max", ACTIVATION_ZONE_X_MAX)
        )
        self.activation_required_steps = int(
            self.terrain_bounds.get("activation_required_steps", ACTIVATION_REQUIRED_STEPS)
        )
        self.heading_ref_min_target_speed = float(
            self.terrain_bounds.get(
                "heading_reference_min_target_speed", HEADING_REFERENCE_MIN_TARGET_SPEED
            )
        )
        self._rendezvous_count = 0
        self._rendezvous_steps = []
        self._slot_best = {
            1: {"distance": None, "relative_speed": None, "heading_error_deg": None, "step": None},
            2: {"distance": None, "relative_speed": None, "heading_error_deg": None, "step": None},
        }
        self._post_rendezvous_peak_distance = None
        self._post_rendezvous_peak_distance_step = None
    def evaluate(self, agent_body, step_count, max_steps):
        if not self.environment:
            return True, 0.0, {"error": "Environment not available"}
        distance = self.environment.get_distance_to_target()
        sx, sy = self.environment.get_seeker_position()
        vx, vy = self.environment.get_seeker_velocity()
        tx, ty = (
            self.environment.get_target_position_true()
            if hasattr(self.environment, "get_target_position_true")
            else self.environment.get_target_position()
        )
        tvx, tvy = (
            self.environment.get_target_velocity_true()
            if hasattr(self.environment, "get_target_velocity_true")
            else (0.0, 0.0)
        )
        rel_vx = vx - tvx
        rel_vy = vy - tvy
        relative_speed = math.sqrt(rel_vx * rel_vx + rel_vy * rel_vy)
        activation_achieved = getattr(
            self.environment, "get_activation_achieved", lambda: False
        )()
        in_rendezvous_zone = (
            self.rendezvous_zone_x_min <= sx <= self.rendezvous_zone_x_max
        )
        in_any_slot1 = any(lo <= step_count <= hi for (lo, hi) in self.slots_phase1)
        in_any_slot2 = any(lo <= step_count <= hi for (lo, hi) in self.slots_phase2)
        seeker_heading = getattr(self.environment, "get_seeker_heading", lambda: 0.0)()
        target_speed = math.sqrt(tvx * tvx + tvy * tvy)
        if target_speed >= self.heading_ref_min_target_speed:
            target_dir = math.atan2(tvy, tvx)
        else:
            target_dir = math.atan2(ty - sy, tx - sx)
        angle_diff = seeker_heading - target_dir
        while angle_diff > math.pi: angle_diff -= 2 * math.pi
        while angle_diff < -math.pi: angle_diff += 2 * math.pi
        heading_error_deg = math.degrees(abs(angle_diff))
        heading_aligned = abs(angle_diff) <= self.heading_tolerance_rad
        active_phase = 1 if in_any_slot1 else (2 if in_any_slot2 else None)
        if active_phase is not None:
            best = self._slot_best[active_phase]
            if best["distance"] is None or distance < best["distance"]:
                best.update(
                    {
                        "distance": distance,
                        "relative_speed": relative_speed,
                        "heading_error_deg": heading_error_deg,
                        "step": step_count,
                    }
                )
        conditions_met = (
            activation_achieved
            and distance <= self.rendezvous_distance
            and relative_speed < self.rendezvous_rel_speed
            and in_rendezvous_zone
            and heading_aligned
        )
        previous_rendezvous_count = self._rendezvous_count
        if conditions_met and in_any_slot1:
            self._rendezvous_count = max(self._rendezvous_count, 1)
        if conditions_met and in_any_slot2 and self._rendezvous_count >= 1:
            self._rendezvous_count = 2
        if self._rendezvous_count > previous_rendezvous_count:
            self._rendezvous_steps.append(step_count)
        if self._rendezvous_count >= 2:
            if (
                self._post_rendezvous_peak_distance is None
                or distance > self._post_rendezvous_peak_distance
            ):
                self._post_rendezvous_peak_distance = distance
                self._post_rendezvous_peak_distance_step = step_count
        failed = False
        failure_reason = None
        if getattr(self.environment, "get_out_of_fuel", lambda: False)():
            failed = True
            failure_reason = "Thrust budget exceeded (out of fuel)"
        elif getattr(self.environment, "get_corridor_violation", lambda: False)():
            failed = True
            failure_reason = "Left the allowed moving corridor"
        elif getattr(self.environment, "get_obstacle_collision", lambda: False)():
            failed = True
            failure_reason = "Collision with corridor obstacle"
        elif step_count > self.window1_hi and self._rendezvous_count < 1:
            failed = True
            if not activation_achieved:
                failure_reason = "First rendezvous missed: activation not achieved."
            else:
                failure_reason = "First rendezvous window closed: rendezvous not completed."
        elif step_count > self.window2_hi and self._rendezvous_count < 2:
            failed = True
            failure_reason = "Second rendezvous window closed: only {} of 2 rendezvous completed.".format(self._rendezvous_count)
        elif self._rendezvous_count >= 2 and distance > self.track_distance:
            failed = True
            failure_reason = f"Target lost after rendezvous: distance {distance:.2f} m exceeds track limit {self.track_distance:.1f} m"
        success = (step_count >= max_steps - 1) and self._rendezvous_count >= 2 and not failed
        if success and not failed:
            score = 100.0
        elif failed:
            score = 0.0
        else:
            progress = step_count / max_steps if max_steps > 0 else 0.0
            if self._rendezvous_count >= 2:
                score = 90.0 + progress * 10.0
            elif self._rendezvous_count >= 1:
                score = 50.0 + progress * 40.0
            else:
                score = progress * 50.0
        metrics = {
            "seeker_x": sx,
            "seeker_y": sy,
            "seeker_vx": vx,
            "seeker_vy": vy,
            "target_x": tx,
            "target_y": ty,
            "distance_to_target": distance,
            "relative_speed": relative_speed,
            "rendezvous_distance": self.rendezvous_distance,
            "rendezvous_rel_speed": self.rendezvous_rel_speed,
            "track_distance": self.track_distance,
            "step_count": step_count,
            "max_steps": max_steps,
            "success": success,
            "failed": failed,
            "failure_reason": failure_reason,
            "rendezvous_count": self._rendezvous_count,
            "required_rendezvous_count": 2,
            "heading_aligned": heading_aligned,
            "heading_error_deg": heading_error_deg,
        }
        if hasattr(self.environment, "get_remaining_impulse_budget"):
            metrics["remaining_impulse_budget"] = self.environment.get_remaining_impulse_budget()
        if hasattr(self.environment, "get_out_of_fuel"):
            metrics["out_of_fuel"] = self.environment.get_out_of_fuel()
        if hasattr(self.environment, "get_corridor_violation"):
            metrics["corridor_violation"] = self.environment.get_corridor_violation()
        if hasattr(self.environment, "get_activation_achieved"):
            metrics["activation_achieved"] = self.environment.get_activation_achieved()
        if hasattr(self.environment, "get_activation_progress"):
            activation_progress = self.environment.get_activation_progress()
            metrics["activation_current_consecutive_steps"] = int(
                activation_progress.get("current_consecutive_steps", 0)
            )
            metrics["activation_max_consecutive_steps"] = int(
                activation_progress.get("max_consecutive_steps", 0)
            )
            metrics["activation_achieved_step"] = activation_progress.get("achieved_step")
        if hasattr(self.environment, "get_obstacle_collision"):
            metrics["obstacle_collision"] = self.environment.get_obstacle_collision()
        metrics["activation_zone_x_min"] = self.activation_zone_x_min
        metrics["activation_zone_x_max"] = self.activation_zone_x_max
        metrics["activation_required_steps"] = self.activation_required_steps
        metrics["heading_reference_min_target_speed"] = self.heading_ref_min_target_speed
        metrics["slots_phase1"] = self.slots_phase1
        metrics["slots_phase2"] = self.slots_phase2
        current_step = step_count
        next_p1_lo, next_p1_hi = None, None
        next_p2_lo, next_p2_hi = None, None
        for lo, hi in self.slots_phase1:
            if lo > current_step:
                if next_p1_lo is None or lo < next_p1_lo:
                    next_p1_lo, next_p1_hi = lo, hi
        for lo, hi in self.slots_phase2:
            if lo > current_step:
                if next_p2_lo is None or lo < next_p2_lo:
                    next_p2_lo, next_p2_hi = lo, hi
        steps_to_p1 = (next_p1_lo - current_step) if next_p1_lo is not None else -1
        steps_to_p2 = (next_p2_lo - current_step) if next_p2_lo is not None else -1
        in_slot = in_any_slot1 or in_any_slot2
        metrics["steps_until_next_p1_slot"] = steps_to_p1
        metrics["steps_until_next_p2_slot"] = steps_to_p2
        metrics["in_active_slot"] = in_slot
        metrics["seeker_heading_rad"] = seeker_heading
        metrics["target_velocity_dir_rad"] = math.atan2(tvy, tvx) if abs(tvx) > 1e-9 or abs(tvy) > 1e-9 else 0.0
        metrics["seeker_to_target_dir_rad"] = math.atan2(ty - sy, tx - sx)
        metrics["reference_heading_dir_rad"] = target_dir
        metrics["target_speed_mps"] = target_speed
        metrics["heading_tolerance_deg"] = self.heading_tolerance_deg
        if self.heading_tolerance_deg > 0:
            metrics["heading_margin_deg"] = self.heading_tolerance_deg - math.degrees(abs(angle_diff))
        else:
            metrics["heading_margin_deg"] = 0.0
        if hasattr(self.environment, "get_corridor_bounds"):
            c_lo, c_hi = self.environment.get_corridor_bounds()
            metrics["corridor_x_lo"] = float(c_lo)
            metrics["corridor_x_hi"] = float(c_hi)
            margin_lo = sx - c_lo
            margin_hi = c_hi - sx
            metrics["corridor_margin_lo"] = margin_lo
            metrics["corridor_margin_hi"] = margin_hi
            metrics["corridor_width"] = c_hi - c_lo
            metrics["corridor_min_margin"] = min(margin_lo, margin_hi)
        rendezvous_dist_limit = self.rendezvous_distance
        rendezvous_rel_limit = self.rendezvous_rel_speed
        track_dist_limit = self.track_distance
        impulse_budget_total = float(self.environment.get_impulse_budget())
        impulse_used = float(self.environment.get_impulse_used())
        metrics["constraint_distance_margin"] = rendezvous_dist_limit - distance
        metrics["constraint_rel_speed_margin"] = rendezvous_rel_limit - relative_speed
        metrics["constraint_track_distance_margin"] = track_dist_limit - distance
        metrics["constraint_distance_pct"] = (distance / rendezvous_dist_limit * 100.0) if rendezvous_dist_limit > 0 else 0.0
        metrics["constraint_rel_speed_pct"] = (relative_speed / rendezvous_rel_limit * 100.0) if rendezvous_rel_limit > 0 else 0.0
        metrics["impulse_used"] = impulse_used
        metrics["impulse_budget"] = impulse_budget_total
        metrics["impulse_used_pct"] = (impulse_used / impulse_budget_total * 100.0) if impulse_budget_total > 0 else 0.0
        metrics["remaining_impulse_budget"] = max(0.0, impulse_budget_total - impulse_used)
        metrics["cooldown_remaining_steps"] = int(
            self.environment.get_cooldown_remaining_steps()
        )
        metrics["last_applied_thrust_magnitude"] = float(
            self.environment.get_last_applied_thrust_magnitude()
        )
        for key in (
            "max_thrust_magnitude",
            "cooldown_threshold",
            "cooldown_max_thrust",
            "cooldown_steps",
            "corridor_violation_tolerance",
            "obstacle_penetration_limit",
        ):
            if key in self.terrain_bounds:
                metrics[key] = self.terrain_bounds[key]
        peak_speed = float(self.environment.get_peak_seeker_speed())
        peak_accel = float(self.environment.get_peak_acceleration())
        metrics["peak_seeker_speed"] = peak_speed
        metrics["peak_acceleration"] = peak_accel
        seeker_speed = math.sqrt(vx * vx + vy * vy)
        metrics["seeker_speed_current"] = seeker_speed
        any_nonfinite = False
        for fkey in ("seeker_x", "seeker_y", "seeker_vx", "seeker_vy", "target_x", "target_y",
                      "distance_to_target", "relative_speed"):
            vv = metrics.get(fkey)
            if vv is not None:
                try:
                    if not math.isfinite(float(vv)):
                        any_nonfinite = True
                        break
                except (TypeError, ValueError):
                    any_nonfinite = True
                    break
        metrics["numerical_nonfinite_detected"] = any_nonfinite
        fe = list(self.environment.get_failure_events())
        metrics["failure_events"] = fe
        if metrics.get("corridor_violation"):
            for ev in reversed(fe):
                if ev.get("type") == "corridor_violation":
                    metrics["violation_seeker_x"] = ev.get("seeker_x", sx)
                    metrics["violation_bound_lo"] = ev.get("bound_lo", 0.0)
                    metrics["violation_bound_hi"] = ev.get("bound_hi", 0.0)
                    metrics["violation_boundary"] = ev.get("boundary", "unknown")
                    metrics["violation_overflow"] = ev.get("overflow", 0.0)
                    break
        if metrics.get("obstacle_collision"):
            for ev in reversed(fe):
                if ev.get("type") == "obstacle_collision":
                    metrics["collision_event_detail"] = ev.get("detail", "")
                    break
        metrics["rendezvous_steps"] = list(self._rendezvous_steps)
        for phase in (1, 2):
            best = self._slot_best[phase]
            metrics[f"phase{phase}_best_distance"] = best["distance"]
            metrics[f"phase{phase}_best_relative_speed"] = best["relative_speed"]
            metrics[f"phase{phase}_best_heading_error_deg"] = best["heading_error_deg"]
            metrics[f"phase{phase}_best_step"] = best["step"]
        metrics["post_rendezvous_peak_distance"] = self._post_rendezvous_peak_distance
        metrics["post_rendezvous_peak_distance_step"] = (
            self._post_rendezvous_peak_distance_step
        )
        metrics["rendezvous_conditions_met"] = {
            "activation": activation_achieved,
            "distance_ok": distance <= self.rendezvous_distance,
            "rel_speed_ok": relative_speed < self.rendezvous_rel_speed,
            "in_zone": in_rendezvous_zone,
            "heading_aligned": heading_aligned,
            "in_slot": in_slot,
        }
        done = failed or (step_count >= max_steps - 1)
        return done, score, metrics
    def compute_score_with_penalty(self, score: float, metrics: dict) -> float:
        if score > 0:
            return score
        constraint_info = self.get_constraint_info()
        penalty = compute_constraint_penalty("C_03", score, metrics, constraint_info)
        return penalty
    def get_constraint_info(self):
        return {
            'rendezvous_distance': self.rendezvous_distance,
            'rendezvous_rel_speed': self.rendezvous_rel_speed,
            'track_distance': self.track_distance,
            'heading_tolerance_rad': self.heading_tolerance_rad,
            'heading_tolerance_deg': self.heading_tolerance_deg,
            'rendezvous_zone_x_min': self.rendezvous_zone_x_min,
            'rendezvous_zone_x_max': self.rendezvous_zone_x_max,
            'activation_zone_x_min': self.activation_zone_x_min,
            'activation_zone_x_max': self.activation_zone_x_max,
            'activation_required_steps': self.activation_required_steps,
            'heading_ref_min_target_speed': self.heading_ref_min_target_speed,
        }
    def get_task_description(self):
        return {
            "task": "C-03: The Seeker (Slotted Rendezvous)",
            "description": (
                "Activate the seeker, complete one heading-aligned rendezvous in "
                "each slot phase, then retain the target without violating safety "
                "or impulse constraints."
            ),
            "rendezvous_distance": self.rendezvous_distance,
            "rendezvous_rel_speed": self.rendezvous_rel_speed,
            "track_distance": self.track_distance,
            "activation_zone_x": [self.activation_zone_x_min, self.activation_zone_x_max],
            "activation_required_consecutive_steps": self.activation_required_steps,
            "rendezvous_zone_x": [self.rendezvous_zone_x_min, self.rendezvous_zone_x_max],
            "heading_tolerance_deg": self.heading_tolerance_deg,
            "heading_reference_min_target_speed": self.heading_ref_min_target_speed,
            "success_criteria": {
                "phase1": "First rendezvous in a phase-1 slot before phase-1 window ends",
                "phase2": "Second rendezvous in a phase-2 slot before phase-2 window ends",
                "phase3": f"After second rendezvous, distance <= {self.track_distance} m until episode end",
                "capture": (
                    f"Activation complete; seeker x in "
                    f"[{self.rendezvous_zone_x_min}, {self.rendezvous_zone_x_max}] m; "
                    f"distance <= {self.rendezvous_distance} m; relative speed < "
                    f"{self.rendezvous_rel_speed} m/s; heading error <= "
                    f"{self.heading_tolerance_deg} degrees."
                ),
                "failure": "Miss slots, obstacles, corridor exit, impulse budget, or lose target after rendezvous",
            },
            "evaluation": {
                "score_range": "0-100",
                "success_score": 100,
                "failure_score": 0,
            },
        }
