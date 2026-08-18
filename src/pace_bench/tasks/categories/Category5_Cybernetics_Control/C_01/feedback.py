"""Deterministic, observation-only feedback for C-01."""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional


def _number(value: Any) -> Optional[float]:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _fmt(value: Any, digits: int = 3) -> str:
    number = _number(value)
    return "unavailable" if number is None else f"{number:.{digits}f}"


def _outcome(metrics: Dict[str, Any]) -> str:
    step = metrics.get("step_count", "unavailable")
    maximum = metrics.get("max_steps")
    step_text = f"{step}/{maximum}" if maximum is not None else str(step)
    if metrics.get("success"):
        return f"**Outcome**: SUCCESS at step {step_text}."
    if metrics.get("failed"):
        reason = metrics.get("failure_reason") or metrics.get("reason") or "failure reason unavailable"
        return f"**Outcome**: FAILURE at step {step_text} — {reason}."
    return f"**Outcome**: IN PROGRESS at step {step_text}."


def format_task_metrics(metrics: Dict[str, Any]) -> List[str]:
    if not isinstance(metrics, dict) or not metrics:
        return ["**Outcome**: unavailable — no metrics were provided."]
    center = _number(metrics.get("track_center_x"))
    safe = _number(metrics.get("safe_half_range"))
    cart = _number(metrics.get("cart_x"))
    offset = abs(cart - center) if cart is not None and center is not None else None
    track_margin = safe - offset if safe is not None and offset is not None else None
    force = _number(metrics.get("applied_force"))
    force_limit = _number(metrics.get("force_limit"))
    force_margin = force_limit - abs(force) if force_limit is not None and force is not None else None
    upright = metrics.get("consecutive_upright_sim_steps", "unavailable")
    required = metrics.get("balance_hold_steps_required", "unavailable")
    lines = [
        "## C-01 Cart-Pole Feedback",
        _outcome(metrics),
        "### Reported sensor state",
        f"- Pole angle: {_fmt(metrics.get('pole_angle_deg'))}°; angular velocity "
        f"{_fmt(metrics.get('pole_angular_velocity'))} rad/s.",
        f"- Cart position: {_fmt(metrics.get('cart_x'))} m; velocity "
        f"{_fmt(metrics.get('cart_velocity_x'))} m/s.",
        f"- Observed extrema through this step: |reported angle| "
        f"{_fmt(metrics.get('peak_abs_reported_angle_deg'))}°; |reported angular velocity| "
        f"{_fmt(metrics.get('peak_abs_reported_angular_velocity'))} rad/s.",
        "### Public constraint state",
        f"- Track center: {_fmt(center)} m; safe half-range {_fmt(safe)} m; "
        f"absolute offset {_fmt(offset)} m; remaining margin {_fmt(track_margin)} m.",
        f"- Applied force: {_fmt(force)} / {_fmt(force_limit)} N; remaining "
        f"magnitude margin {_fmt(force_margin)} N.",
        f"- Worst observed track margin: {_fmt(metrics.get('minimum_track_margin'))} m; "
        f"force-limit saturation steps: {metrics.get('force_saturation_steps', 'unavailable')}.",
        f"- Upright grading steps: {upright} / {required}; lock-in achieved="
        f"{bool(metrics.get('balance_achieved'))}; first lock-in step="
        f"{metrics.get('balance_achieved_step', 'unavailable')}.",
        f"- Grading bands: upright ±{_fmt(metrics.get('grading_balance_angle_deg'))}°; "
        f"post-lock-in failure ±{_fmt(metrics.get('grading_failure_angle_deg'))}°.",
        "### Data health",
    ]
    invalid = []
    for key in (
        "pole_angle_deg", "pole_angular_velocity", "cart_x", "cart_velocity_x",
        "applied_force", "force_limit", "dist_from_center",
        "peak_abs_reported_angle_deg", "peak_abs_reported_angular_velocity",
        "minimum_track_margin",
    ):
        if key in metrics and metrics.get(key) is not None and _number(metrics.get(key)) is None:
            invalid.append(key)
    if invalid:
        lines.append("- Non-finite or invalid observations: " + ", ".join(invalid) + ".")
    else:
        lines.append("- All reported numeric observations are finite.")
    return lines


def get_improvement_suggestions(
    metrics: Dict[str, Any],
    score: float,
    success: bool,
    failed: bool,
    failure_reason: str = None,
    error: str = None,
) -> List[str]:
    if error:
        return [f"- Resolve the reported execution error: {error}"]
    return []
