import math

_PARTIAL_SCORE_MAX = 80.0

from pace_bench.core.primitives import compute_constraint_penalty

def _distance_point_to_switch_zone(x: float, y: float, cx: float, cy: float, hw: float, hh: float) -> float:
    closest_x = min(max(x, cx - hw), cx + hw)
    closest_y = min(max(y, cy - hh), cy + hh)
    return math.hypot(x - closest_x, y - closest_y)

def _agent_center_in_zone(x: float, y: float, cx: float, cy: float, hw: float, hh: float) -> bool:
    return (cx - hw <= x <= cx + hw) and (cy - hh <= y <= cy + hh)

class Evaluator:
    def __init__(self, terrain_bounds, environment=None):
        self.terrain_bounds = terrain_bounds
        self.environment = environment
        self._required_order = list(terrain_bounds.get("required_order", ["A", "B", "C"]))
    def evaluate(self, agent_body, step_count, max_steps):
        if not self.environment:
            return True, 0.0, {"error": "Environment not available"}
        sequence_correct = self.environment.get_sequence_correct()
        wrong_order = self.environment.get_wrong_order()
        triggered = self.environment.get_triggered_switches()
        next_req = self.environment.get_next_required_switch()
        x, y = self.environment.get_agent_position()
        vx, vy = self.environment.get_agent_velocity()
        steps_in_current_zone = getattr(
            self.environment, "get_steps_in_current_zone", lambda: 0
        )()
        steps_required_to_trigger = getattr(
            self.environment, "get_steps_required_to_trigger", lambda: 1
        )()
        cooldown_remaining = getattr(
            self.environment, "get_cooldown_remaining", lambda: 0
        )()
        failed = False
        failure_reason = None
        timed_out = False
        if wrong_order:
            failed = True
            failure_reason = "Switches triggered in wrong order relative to required sequence"
        elif max_steps > 0 and step_count >= max_steps and not sequence_correct:
            failed = True
            timed_out = True
            failure_reason = "Time limit reached before completing the required switch sequence"
        success = sequence_correct and not failed
        n_milestones = max(1, len(self._required_order))
        milestone_weight = _PARTIAL_SCORE_MAX / float(n_milestones)
        if success:
            score = 100.0
        elif failed:
            score = 0.0
        else:
            score = (
                len(triggered) * milestone_weight
                if self._required_order
                else 0.0
            )
        zones = self.terrain_bounds.get("zones", {})
        tb_fn = getattr(self.environment, "get_terrain_bounds", None)
        if callable(tb_fn):
            live_tb = tb_fn()
            if isinstance(live_tb, dict) and live_tb.get("zones"):
                zones = live_tb["zones"]
        distance_to_next = None
        inside_next_required_zone = False
        if next_req and next_req in zones:
            cx, cy, hw, hh = zones[next_req]
            distance_to_next = _distance_point_to_switch_zone(x, y, cx, cy, hw, hh)
            inside_next_required_zone = _agent_center_in_zone(x, y, cx, cy, hw, hh)
        speed = math.sqrt(vx * vx + vy * vy)
        progress_percent = (
            (len(triggered) / float(n_milestones) * 100.0) if self._required_order else 0.0
        )
        metrics = {
            "max_steps": max_steps,
            "agent_x": x,
            "agent_y": y,
            "agent_vx": vx,
            "agent_vy": vy,
            "triggered_switches": list(triggered),
            "next_required": next_req,
            "sequence_correct": sequence_correct,
            "wrong_order": wrong_order,
            "step_count": step_count,
            "success": success,
            "failed": failed,
            "failure_reason": failure_reason,
            "distance_to_next_zone": distance_to_next,
            "inside_next_required_zone": inside_next_required_zone,
            "speed": speed,
            "progress_percent": progress_percent,
            "steps_in_current_zone": steps_in_current_zone,
            "steps_required_to_trigger": steps_required_to_trigger,
            "cooldown_remaining": cooldown_remaining,
            "timed_out": timed_out,
            "zones": zones,
        }
        env = self.environment
        rfx, rfy, rmag = env.get_repulsion_at_agent()
        metrics.update(
            {
                "repulsion_fx": rfx,
                "repulsion_fy": rfy,
                "repulsion_magnitude": rmag,
            }
        )
        barrier = env.get_barrier_status()
        metrics["barrier_active"] = barrier.get("active", False)
        metrics["barrier_steps_until_open"] = barrier.get("steps_until_open", 0)
        metrics["cooldown_total"] = env.get_cooldown_total()
        temporal = env.get_temporal_window_status()
        metrics.update(
            {
                "A_visited": temporal.get("A_visited", False),
                "B_visited": temporal.get("B_visited", False),
                "steps_since_last_A": temporal.get("steps_since_last_A", -1),
                "steps_since_last_B": temporal.get("steps_since_last_B", -1),
                "temporal_window_A_to_B": temporal.get("window_A_to_B", 0),
                "temporal_window_B_to_C": temporal.get("window_B_to_C", 0),
            }
        )
        reset_stats = env.get_dwell_reset_stats()
        for source_key, metric_key in (
            ("zone_change", "dwell_reset_zone_change"),
            ("speed", "dwell_reset_speed"),
            ("force", "dwell_reset_force"),
            ("blocked_temporal", "dwell_blocked_temporal"),
            ("blocked_altitude", "dwell_blocked_altitude"),
            ("blocked_cooldown", "dwell_blocked_cooldown"),
        ):
            metrics[metric_key] = reset_stats.get(source_key, 0)
        afx, afy, afm = env.get_last_applied_force()
        metrics.update(
            {
                "applied_force_x": afx,
                "applied_force_y": afy,
                "applied_force_magnitude": afm,
            }
        )
        history = env.get_agent_y_history_stats()
        metrics.update(
            {
                "agent_y_max_recent": history.get("max_recent_y", 0.0),
                "agent_y_history_length": history.get("history_length", 0),
                "history_window": history.get("history_window", 0),
                "required_max_y_c": history.get("required_max_y", 0.0),
            }
        )
        peaks = env.get_peak_values()
        metrics["peak_speed_seen"] = peaks.get("peak_speed", 0.0)
        metrics["peak_force_applied_magnitude"] = peaks.get(
            "peak_force_applied", 0.0
        )
        metrics["force_limit_inside"] = env.get_in_zone_force_limit()
        metrics["speed_cap_inside"] = env.get_in_zone_speed_cap()
        repulsion = env.get_repulsion_params()
        metrics["repulsion_mag"] = repulsion.get("magnitude", 0.0)
        metrics["repulsion_range"] = repulsion.get("range", 0.0)
        metrics["trigger_stay_steps"] = env.get_trigger_stay_steps()
        metrics["barrier_delay_steps"] = env.get_barrier_delay_steps()
        metrics["max_agent_force_per_axis"] = env.get_max_agent_force()
        done = failed or sequence_correct
        return done, score, metrics
    def compute_score_with_penalty(self, score: float, metrics: dict) -> float:
        if score > 0:
            return score
        constraint_info = self.get_constraint_info()
        penalty = compute_constraint_penalty("C_05", score, metrics, constraint_info)
        return penalty
    def get_constraint_info(self):
        return {
            'required_order': self._required_order,
        }
    def get_task_description(self):
        desc = (
            "Navigate the agent through trigger zones A, B, and C in order while "
            "satisfying live dwell, speed, force, cooldown, and timing constraints."
        )
        order_s = ", ".join(self._required_order) if self._required_order else "(see environment)"
        primary = (
            f"Trigger switches in order: {order_s}"
        )
        return {
            "task": "C-05: The Logic Lock",
            "description": desc,
            "required_order": self._required_order,
            "success_criteria": {
                "primary": primary,
                "failure": "Wrong order or exhausting the episode before completion",
            },
            "evaluation": {
                "score_range": "0-100",
                "success_score": 100,
                "failure_score": 0,
            },
        }
