from __future__ import annotations

import math
from typing import Any, Dict, List, Optional


def _finite(metrics: Dict[str, Any], key: str) -> Optional[float]:
    value = metrics.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    value = float(value)
    return value if math.isfinite(value) else None


def _fmt(value: Optional[float], digits: int = 3) -> str:
    return "unavailable" if value is None else f"{value:.{digits}f}"


def format_task_metrics(metrics: Dict[str, Any]) -> List[str]:
    if not metrics:
        return ["**Evaluation data**: unavailable"]
    if "error" in metrics:
        return [f"**Evaluator error**: {metrics.get('error')}"]

    required = ("craft_x", "craft_y", "heat", "overheat_limit")
    invalid = [
        key
        for key in required
        if key in metrics and _finite(metrics, key) is None
    ]
    if invalid:
        return [
            "### Evaluation state",
            "Non-finite or invalid required fields: " + ", ".join(invalid),
        ]

    step = _finite(metrics, "step_count")
    max_steps = _finite(metrics, "max_steps")
    remaining_steps = _finite(metrics, "step_budget_remaining")
    x = _finite(metrics, "craft_x")
    y = _finite(metrics, "craft_y")
    vx = _finite(metrics, "velocity_x")
    vy = _finite(metrics, "velocity_y")
    speed = _finite(metrics, "speed")
    peak_speed = _finite(metrics, "peak_speed")
    heat = _finite(metrics, "heat")
    peak_heat = _finite(metrics, "peak_heat")
    limit = _finite(metrics, "overheat_limit")
    heat_remaining = _finite(metrics, "heat_remaining")
    target_gap = _finite(metrics, "target_gap")
    target_gap_x = _finite(metrics, "target_gap_x")
    target_gap_y = _finite(metrics, "target_gap_y")
    closest_gap = _finite(metrics, "closest_target_gap")
    closest_step = _finite(metrics, "closest_target_step")
    first_target_step = _finite(metrics, "first_target_step")
    first_overheat_step = _finite(metrics, "first_overheat_step")

    if metrics.get("success"):
        outcome = "success"
    elif metrics.get("failed"):
        outcome = "failure"
    else:
        outcome = "in progress"

    lines = ["### Outcome", f"Status: {outcome}."]
    reason = metrics.get("failure_reason")
    if reason:
        lines.append(f"Recorded reason: {reason}.")

    lines.extend(
        [
            "",
            "### Final measured state",
            (
                f"Step: {_fmt(step, 0)}/{_fmt(max_steps, 0)}; "
                f"remaining: {_fmt(remaining_steps, 0)}."
            ),
            (
                f"Position: ({_fmt(x)}, {_fmt(y)}) m; "
                f"velocity: ({_fmt(vx)}, {_fmt(vy)}) m/s; "
                f"speed: {_fmt(speed)} m/s."
            ),
            (
                f"Distance to target region: {_fmt(target_gap)} m "
                f"(horizontal gap {_fmt(target_gap_x)} m, "
                f"vertical gap {_fmt(target_gap_y)} m)."
            ),
            (
                f"Heat: {_fmt(heat, 1)}/{_fmt(limit, 1)} N·s; "
                f"remaining thermal margin: {_fmt(heat_remaining, 1)} N·s."
            ),
            "",
            "### Recorded trajectory extrema",
            (
                f"Closest target-region distance: {_fmt(closest_gap)} m "
                f"at step {_fmt(closest_step, 0)}."
            ),
            f"Peak speed: {_fmt(peak_speed)} m/s.",
            f"Peak heat: {_fmt(peak_heat, 1)} N·s.",
        ]
    )

    if first_target_step is not None:
        lines.append(f"First target entry: step {first_target_step:.0f}.")
    else:
        lines.append("First target entry: not recorded.")
    if first_overheat_step is not None:
        lines.append(f"First thermal-limit event: step {first_overheat_step:.0f}.")
    else:
        lines.append("First thermal-limit event: not recorded.")
    return lines


def get_improvement_suggestions(
    metrics: Dict[str, Any],
    score: float,
    success: bool,
    failed: bool,
    failure_reason: str = None,
    error: str = None,
) -> List[str]:
    del score, success, failed, failure_reason
    if error:
        return [f"- Execution error: {error[:200]}"]
    if not metrics:
        return []
    # Deliberately observational: controller design remains the agent's work.
    observations: List[str] = []
    target_gap = _finite(metrics, "target_gap")
    if target_gap is not None and not metrics.get("reached_target"):
        observations.append(
            f"- Final measured distance to the target region was {target_gap:.3f} m."
        )
    heat = _finite(metrics, "heat")
    limit = _finite(metrics, "overheat_limit")
    if heat is not None and limit is not None:
        observations.append(
            f"- Final measured heat was {heat:.1f} N·s against a "
            f"{limit:.1f} N·s limit."
        )
    return observations
