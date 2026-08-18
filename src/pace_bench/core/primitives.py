"""
Reward utilities for constraint-aware reward computation.

Provides functions to compute violation penalties based on constraint metadata
from evaluator metrics. Used by the verifier to retain dense score signals.

The reward formula for failed solutions (score == 0):
    reward = (K / N - 1) * 100
where N = total constraints, K = satisfied constraints (N - violations).

This gives:
    - reward = 0 when all N constraints are satisfied (K/N = 1)
    - reward = -50 when half are violated (K/N = 0.5)
    - reward = -100 when all are violated (K/N = 0)

For successful solutions (score > 0), reward = score (no penalty applied).
"""

from typing import Any, Dict, Optional, Tuple


# ----------------------------------------------------------------------
# Constraint definition helpers
# ----------------------------------------------------------------------


def count_violations_S_01(
    metrics: Dict[str, Any], info: Dict[str, Any]
) -> Tuple[int, int]:
    """
    Count constraint violations for S-01 (Bridge) task.

    Returns (violated_count, total_count).
    """
    import math

    violations = 0
    total = 7  # 7 measurable hard constraints

    # 1. Mass budget
    mass = metrics.get("structure_mass", 0)
    max_mass = info.get("max_structure_mass", 2000.0)
    if mass > max_mass:
        violations += 1

    # 2. Structure integrity (joints didn't break)
    if metrics.get("structure_broken", False):
        violations += 1

    # 3. Vehicle didn't fall (y above fail zone)
    vehicle_y = metrics.get("vehicle_y", float("inf"))
    fail_zone_y = info.get("fail_zone_y", 0.5)
    if vehicle_y <= fail_zone_y:
        violations += 1

    # 4. Vertical acceleration within limit
    max_accel = metrics.get("max_vertical_accel_seen", 0)
    max_accel_limit = info.get("max_vertical_acceleration", 19.6)
    if max_accel > max_accel_limit:
        violations += 1

    # 5. Vehicle not unstable (high angular velocity)
    high_av_count = metrics.get("high_angular_velocity_count", 0)
    unstable_thresh = info.get("unstable_threshold", 5)
    if high_av_count >= unstable_thresh:
        violations += 1

    # 6. Vehicle didn't flip (angle within bounds)
    norm_angle = metrics.get("normalized_angle", 0.0)
    if abs(norm_angle) > math.pi / 2:
        violations += 1

    # 7. Airborne rotation within limit
    air_rot = metrics.get("airborne_rotation_accumulated", 0.0)
    max_air_rot = info.get("max_airborne_rotation", math.pi)
    if air_rot > max_air_rot:
        violations += 1

    return violations, total


def count_violations_K_01(
    metrics: Dict[str, Any], info: Dict[str, Any]
) -> Tuple[int, int]:
    """
    Count constraint violations for K-01 (Walker) task.

    Returns (violated_count, total_count).
    """
    violations = 0
    total = 4

    # 1. Mass budget
    mass = metrics.get("structure_mass")
    max_mass = info.get("max_structure_mass", 100.0)
    if mass is not None and mass > max_mass:
        violations += 1

    # 2. Torso didn't collapse (didn't touch ground)
    torso_touched = metrics.get("torso_touched_ground", False)
    if torso_touched:
        violations += 1

    # 3. Build zone not violated
    # K-01 doesn't have explicit build zone violations in the failure reason
    # since it checks via the bz_violated path
    if metrics.get("failed") and metrics.get("failure_reason"):
        fr = str(metrics.get("failure_reason", ""))
        if "build zone" in fr.lower():
            violations += 1

    # 4. Joint limits not exceeded
    joint_limit_hit = metrics.get("joint_limit_hit", False)
    if joint_limit_hit:
        violations += 1

    return violations, total


def count_violations_D_01(
    metrics: Dict[str, Any], info: Dict[str, Any]
) -> Tuple[int, int]:
    """
    Count constraint violations for D-01 (Launcher) task.

    Returns (violated_count, total_count).
    """
    violations = 0
    total = 3

    # 1. Mass budget
    mass = metrics.get("structure_mass", 0)
    max_mass = info.get("max_structure_mass", 500.0)
    if mass > max_mass:
        violations += 1

    # 2. Build zone (all beams within allowed region)
    # Check via failure reason
    if metrics.get("failed") and metrics.get("failure_reason"):
        fr = str(metrics.get("failure_reason", ""))
        if "build zone" in fr.lower() or "outside build zone" in fr.lower():
            violations += 1

    # 3. Projectile within simulation bounds
    if metrics.get("failed") and metrics.get("failure_reason"):
        fr = str(metrics.get("failure_reason", ""))
        if "bounds" in fr.lower() or "left simulation" in fr.lower():
            violations += 1

    return violations, total


def count_violations_generic(
    metrics: Dict[str, Any], info: Dict[str, Any]
) -> Tuple[int, int]:
    """
    Generic constraint violation counter.
    Uses available metrics and info fields to count violations.
    Falls back to 0/1 when specific fields are not available.
    """
    violations = 0
    total = 0

    # Mass constraint (common to all tasks)
    mass = metrics.get("structure_mass")
    max_mass = info.get("max_structure_mass")
    if max_mass is not None:
        total += 1
        if mass is not None and mass > max_mass:
            violations += 1

    # Structure broken constraint
    if "structure_broken" in metrics or "joint_failure_events" in metrics:
        total += 1
        jfe = metrics.get("joint_failure_events", 0)
        if (
            metrics.get("structure_broken", False)
            or (
                jfe
                if isinstance(jfe, (int, float))
                else len(jfe)
                if isinstance(jfe, (list, tuple))
                else 0
            )
            > 0
        ):
            violations += 1

    # Fail zone constraint
    fail_zone_y = info.get("fail_zone_y")
    vehicle_y = (
        metrics.get("vehicle_y")
        or metrics.get("walker_y")
        or metrics.get("projectile_y")
    )
    if fail_zone_y is not None and vehicle_y is not None:
        total += 1
        if vehicle_y <= fail_zone_y:
            violations += 1

    # Vertical acceleration constraint
    max_accel_seen = metrics.get("max_vertical_accel_seen")
    max_accel_limit = info.get("max_vertical_acceleration")
    if max_accel_limit is not None:
        total += 1
        if max_accel_seen is not None and max_accel_seen > max_accel_limit:
            violations += 1

    # Angular velocity / instability constraint
    high_av_count = metrics.get("high_angular_velocity_count")
    unstable_thresh = info.get("unstable_threshold")
    if unstable_thresh is not None:
        total += 1
        if high_av_count is not None and high_av_count >= unstable_thresh:
            violations += 1

    # Flip constraint
    norm_angle = metrics.get("normalized_angle")
    if norm_angle is not None:
        total += 1
        import math

        if abs(norm_angle) > math.pi / 2:
            violations += 1

    # Airborne rotation constraint
    air_rot = metrics.get("airborne_rotation_accumulated")
    max_air_rot = info.get("max_airborne_rotation")
    if max_air_rot is not None:
        total += 1
        if air_rot is not None and air_rot > max_air_rot:
            violations += 1

    # Build zone constraint (via failure reason)
    if metrics.get("failed") and metrics.get("failure_reason"):
        fr = str(metrics.get("failure_reason", "")).lower()
        build_zone_keywords = ["build zone", "outside build zone", "outside allowed"]
        if any(kw in fr for kw in build_zone_keywords):
            total += 1
            violations += 1

    # Ensure at least 1 total so we don't divide by zero
    if total == 0:
        total = 1

    return violations, total


# ----------------------------------------------------------------------
# Task-to-violation-counter dispatch
# ----------------------------------------------------------------------


def count_violations(
    task_name: str, metrics: Dict[str, Any], info: Dict[str, Any]
) -> Tuple[int, int]:
    """
    Dispatch to the correct violation counter based on task name.

    Args:
        task_name: Task identifier (e.g., 'S_01', 'K_01')
        metrics: Full metrics dict from verifier
        info: Constraint info dict from evaluator

    Returns:
        (violated_count, total_count)
    """
    prefix = task_name.split("_")[0].upper() if "_" in task_name else task_name.upper()

    if prefix == "S":
        return count_violations_S_01(metrics, info)
    elif prefix == "K":
        return count_violations_K_01(metrics, info)
    elif prefix == "D":
        return count_violations_D_01(metrics, info)
    else:
        # Generic fallback for F (granular/fluid), C (cybernetics), demo, etc.
        return count_violations_generic(metrics, info)


# ----------------------------------------------------------------------
# Main reward computation
# ----------------------------------------------------------------------


def compute_constraint_penalty(
    task_name: str,
    score: float,
    metrics: Dict[str, Any],
    constraint_info: Optional[Dict[str, Any]] = None,
) -> float:
    """
    Compute the constraint-aware reward.

    For successful solutions (score > 0): returns score unchanged.
    For failed solutions (score == 0): returns penalty based on constraint satisfaction.

    Penalty formula: (K/N - 1) * 100
        K = number of satisfied constraints
        N = total number of constraints

    Examples:
        - All 8/8 constraints satisfied: penalty = (8/8 - 1) * 100 = 0
        - 4/8 constraints satisfied: penalty = (4/8 - 1) * 100 = -50
        - 0/8 constraints satisfied: penalty = (0/8 - 1) * 100 = -100

    Args:
        task_name: Task identifier for dispatching to correct counter
        score: The raw score from verifier (0-100)
        metrics: Full metrics dict from verifier
        constraint_info: Optional constraint info dict. If not provided,
            extracted from metrics['constraint_info'].

    Returns:
        Reward value. For score > 0: returns score. For score == 0: returns
        penalty in range [-100, 0].
    """
    # No penalty for successful solutions
    if score > 0:
        return float(score)

    # Extract constraint info from metrics if not provided
    if constraint_info is None:
        constraint_info = metrics.get("constraint_info", {})

    # Count violations
    violated, total = count_violations(task_name, metrics, constraint_info)
    satisfied = total - violated
    k_over_n = satisfied / total if total > 0 else 0.0

    # Penalty: (K/N - 1) * 100
    penalty = (k_over_n - 1.0) * 100.0
    return penalty


def compute_reward_with_penalty(
    task_name: str,
    score: float,
    metrics: Dict[str, Any],
    constraint_info: Optional[Dict[str, Any]] = None,
) -> Tuple[float, int, int, float]:
    """
    Compute reward with full diagnostic info.

    Returns:
        (reward, violated_count, total_count, penalty_fraction)
        penalty_fraction = K/N (1.0 = all satisfied, 0.0 = none satisfied)
    """
    if score > 0:
        violated, total = count_violations(
            task_name, metrics, constraint_info or metrics.get("constraint_info", {})
        )
        return float(score), violated, total, 1.0

    if constraint_info is None:
        constraint_info = metrics.get("constraint_info", {})

    violated, total = count_violations(task_name, metrics, constraint_info)
    satisfied = total - violated
    k_over_n = satisfied / total if total > 0 else 0.0
    penalty = (k_over_n - 1.0) * 100.0
    return penalty, violated, total, k_over_n
