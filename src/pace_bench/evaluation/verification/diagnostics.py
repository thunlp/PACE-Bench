import math
from typing import Dict, Any, Optional, List

# Keys never printed by generic fallback (too large, internal, or redundant with error section)
_SKIP_GENERIC_METRIC_KEYS = frozenset(
    {
        "granular_snapshots",
        "error_traceback",
        "joint_stress_summary",
        "joint_failure_events",
        "joint_force_history",
    }
)


def _format_generic_execution_metrics(metrics: Dict[str, Any]) -> List[str]:
    """
    When task-specific format_task_metrics is missing, empty, or failed, still emit
    readable lines from common evaluator keys (bridge, inverted-gravity, vehicles, etc.).
    """
    if not metrics:
        return []

    parts: List[str] = []

    def _add(label: str, value: Any) -> None:
        parts.append(f"**{label}**: {value}")

    if metrics.get("success") is not None:
        _add("Success", metrics["success"])
    if metrics.get("failed") is not None:
        _add("Failed", metrics["failed"])
    fr = metrics.get("failure_reason")
    if fr is not None and str(fr).strip():
        _add("Failure reason (evaluator)", fr)

    sc = metrics.get("step_count")
    if sc is not None:
        line = f"**Simulation step**: {sc}"
        cs = metrics.get("current_sim_step")
        if cs is not None:
            try:
                line += f" (physics sub-step: {int(float(cs))})"
            except (TypeError, ValueError):
                line += f" (physics sub-step: {cs})"
        parts.append(line)

    pp = metrics.get("progress_pct")
    if pp is not None:
        try:
            _add("Temporal progress", f"{float(pp):.1f}%")
        except (TypeError, ValueError):
            _add("Temporal progress", str(pp))

    for key, label in (
        ("vehicle_x", "Vehicle x (m)"),
        ("vehicle_y", "Vehicle y (m)"),
        ("target_x", "Target x (m)"),
        ("vehicle_start_x", "Start x (m)"),
        ("velocity_x", "Velocity vx (m/s)"),
        ("velocity_y", "Velocity vy (m/s)"),
        ("angular_velocity", "Angular velocity (rad/s)"),
        ("fail_zone_y", "Fail zone y (m)"),
    ):
        v = metrics.get(key)
        if v is None:
            continue
        try:
            fv = float(v)
            if math.isfinite(fv):
                _add(label, f"{fv:.3f}" if key != "fail_zone_y" else f"{fv:.2f}")
            else:
                _add(label, str(v))
        except (TypeError, ValueError):
            _add(label, str(v))

    sm = metrics.get("structure_mass")
    msm = metrics.get("max_structure_mass")
    if sm is not None:
        try:
            s = f"{float(sm):.2f} kg"
            if msm is not None:
                mf = float(msm)
                if math.isfinite(mf) and mf > 0:
                    s += f" (budget {mf:.2f} kg, {100.0 * float(sm) / mf:.1f}% used)"
            _add("Structural mass", s)
        except (TypeError, ValueError):
            _add("Structural mass", str(sm))

    if (
        metrics.get("beam_count") is not None
        or metrics.get("max_beam_count") is not None
    ):
        _add(
            "Beam count",
            f"{metrics.get('beam_count', '—')} / max {metrics.get('max_beam_count', '—')}",
        )

    jc, ijc = metrics.get("joint_count"), metrics.get("initial_joint_count")
    if jc is not None:
        line = f"**Joint count (current)**: {jc}"
        if ijc is not None:
            try:
                line += f" / {ijc} initial ({int(ijc) - int(jc)} broken)"
            except (TypeError, ValueError):
                line += f" / {ijc} initial"
        parts.append(line)

    if metrics.get("body_count") is not None:
        _add("Dynamic body count (tracked)", metrics["body_count"])

    if metrics.get("structure_broken") is not None:
        _add("Structure broken (joint count dropped)", metrics["structure_broken"])

    g = metrics.get("gravity_current")
    if isinstance(g, (list, tuple)) and len(g) >= 2:
        try:
            gx, gy = float(g[0]), float(g[1])
            mag = math.hypot(gx, gy)
            _add(
                "Instantaneous gravity (m/s²)", f"({gx:.2f}, {gy:.2f}), |g| = {mag:.2f}"
            )
        except (TypeError, ValueError):
            pass

    for key, label in (
        ("joint_max_force_limit", "Structural joint force limit (N)"),
        ("joint_max_torque_limit", "Structural joint torque limit (Nm)"),
        ("anchor_max_force_limit", "Anchor force limit (N)"),
        ("anchor_max_torque_limit", "Anchor torque limit (Nm)"),
    ):
        v = metrics.get(key)
        if v is None:
            continue
        try:
            _add(label, f"{float(v):.2f}")
        except (TypeError, ValueError):
            _add(label, str(v))

    mva = metrics.get("max_vertical_accel")
    if mva is not None:
        try:
            line = f"**Peak vertical acceleration (m/s²)**: {float(mva):.2f}"
            lim = metrics.get("max_vertical_acceleration_limit")
            if lim is not None:
                lf = float(lim)
                if math.isfinite(lf):
                    line += f" (limit {lf:.2f})"
            parts.append(line)
        except (TypeError, ValueError):
            _add("Peak vertical acceleration", str(mva))

    jss = metrics.get("joint_stress_summary")
    if isinstance(jss, list) and jss:
        parts.append(
            f"**Joint stress records**: {len(jss)} joint(s) tracked (see task feedback for detail)"
        )

    jfe = metrics.get("joint_failure_events")
    if isinstance(jfe, list) and jfe:
        parts.append(f"**Joint failure events**: {len(jfe)} recorded")

    # Remaining scalars (short) for tasks with uncommon keys
    shown = {
        "success",
        "failed",
        "failure_reason",
        "step_count",
        "current_sim_step",
        "progress_pct",
        "vehicle_x",
        "vehicle_y",
        "target_x",
        "vehicle_start_x",
        "velocity_x",
        "velocity_y",
        "angular_velocity",
        "fail_zone_y",
        "structure_mass",
        "max_structure_mass",
        "beam_count",
        "max_beam_count",
        "joint_count",
        "initial_joint_count",
        "body_count",
        "structure_broken",
        "gravity_current",
        "joint_max_force_limit",
        "joint_max_torque_limit",
        "anchor_max_force_limit",
        "anchor_max_torque_limit",
        "max_vertical_accel",
        "max_vertical_acceleration_limit",
    }
    extras = []
    for k, v in sorted(metrics.items()):
        if k in shown or k in _SKIP_GENERIC_METRIC_KEYS:
            continue
        if v is None or isinstance(v, (dict, list)):
            continue
        if isinstance(v, str) and len(v) > 120:
            continue
        extras.append(f"{k}={v}")
    if extras:
        parts.append("**Other metrics**: " + "  |  ".join(extras[:24]))
        if len(extras) > 24:
            parts.append(f"  ... and {len(extras) - 24} more scalar fields")

    return parts


def format_generic_execution_metrics(metrics: Dict[str, Any]) -> List[str]:
    """
    Public helper for task `feedback.py` modules: reuse the same structured lines as the
    central evaluator fallback (safe to import from task code after evaluation.feedback loads).
    """
    return _format_generic_execution_metrics(metrics)


def _get_task_feedback_module(task_name: str):
    """
    Dynamically import task-specific feedback module
    Args:
        task_name: Task name (can be in various formats like 'category_1_01', 'Category1_Statics_Equilibrium/S_01', etc.)
    Returns:
        Task feedback module or None if not found
    """
    from pace_bench.tasks.registry import get_registry

    try:
        task = get_registry().resolve(task_name)
        return get_registry().load_module(task, "feedback")
    except (ImportError, ModuleNotFoundError, ValueError):
        return None


def format_feedback(
    metrics: Dict[str, Any],
    score: float,
    success: bool,
    failed: bool,
    failure_reason: str = None,
    iteration: int = 0,
    error: Optional[str] = None,
    task_name: str = None,
    include_suggestions: bool = False,
) -> str:
    """
    Generate feedback text based on evaluation metrics
    Args:
        metrics: Evaluation metrics dictionary
        score: Score (0-100)
        success: Whether successful
        failed: Whether failed
        failure_reason: Failure reason
        iteration: Current iteration number
        error: Error message if code execution failed
        task_name: Task name for loading task-specific feedback
        include_suggestions: Whether to include improvement suggestions (only for sys_feedback mode)
    Returns:
        str: Formatted feedback text
    """
    feedback_parts = []

    feedback_parts.append(f"## Iteration {iteration} Evaluation Results\n")

    # Code execution status section
    if error:
        feedback_parts.append("## Code Execution Status\n")
        feedback_parts.append("❌ **Code execution failed**\n\n")

        # Parse error type
        error_lower = error.lower()
        if "syntax error" in error_lower or "invalid syntax" in error_lower:
            error_type = "Syntax Error"
            feedback_parts.append(f"**Error Type**: {error_type}\n")
            feedback_parts.append(
                "The generated code contains syntax errors that prevent it from being executed.\n"
            )
        elif (
            "name 'sandbox' is not defined" in error_lower or "nameerror" in error_lower
        ):
            error_type = "Name Error"
            feedback_parts.append(f"**Error Type**: {error_type}\n")
            feedback_parts.append(
                "The code references undefined variables. Ensure all code is inside functions.\n"
            )
        elif "error building agent" in error_lower or "valueerror" in error_lower:
            error_type = "Agent Building Error"
            feedback_parts.append(f"**Error Type**: {error_type}\n")
            feedback_parts.append(
                "The code executed but failed during agent construction (e.g., constraint violations).\n"
            )
        elif "runtime error" in error_lower:
            error_type = "Runtime Error"
            feedback_parts.append(f"**Error Type**: {error_type}\n")
            feedback_parts.append(
                "The code executed but encountered a runtime error during execution.\n"
            )
        else:
            error_type = "Execution Error"
            feedback_parts.append(f"**Error Type**: {error_type}\n")
            feedback_parts.append("The code failed during execution.\n")

        # Full error details
        feedback_parts.append("\n**Error Details**:\n")
        feedback_parts.append("```")
        feedback_parts.append(error)
        feedback_parts.append("```\n")

        feedback_parts.append(f"\n**Score**: {score:.1f}/100 (Code execution failed)\n")

    elif success:
        feedback_parts.append("✅ **Task completed successfully!**\n")
        feedback_parts.append(f"**Score**: {score:.1f}/100\n")
    elif failed:
        feedback_parts.append(f"❌ **Task failed**: {failure_reason}\n")
        feedback_parts.append(f"**Score**: {score:.1f}/100\n")
    else:
        feedback_parts.append("⚠️ **Task not completed**\n")
        feedback_parts.append(f"**Score**: {score:.1f}/100\n")

    # Get task-specific feedback module once (if needed)
    task_feedback_module = None
    if task_name:
        task_feedback_module = _get_task_feedback_module(task_name)

    # Task execution results section (only if code executed successfully)
    if not error and metrics:
        feedback_parts.append("\n### Task Execution Results\n")

        # Get task-specific metrics formatting
        task_metric_parts: List[str] = []
        task_format_error: Optional[str] = None
        if task_feedback_module and hasattr(
            task_feedback_module, "format_task_metrics"
        ):
            try:
                raw_parts = task_feedback_module.format_task_metrics(metrics)
                if isinstance(raw_parts, list):
                    task_metric_parts = [
                        p for p in raw_parts if isinstance(p, str) and p.strip()
                    ]
                elif raw_parts:
                    task_metric_parts = [str(raw_parts)]
            except Exception as e:
                task_format_error = f"{type(e).__name__}: {e}"

        if task_metric_parts:
            feedback_parts.extend(task_metric_parts)
        else:
            if task_format_error:
                feedback_parts.append(
                    f"**Task-specific metrics formatting failed** ({task_format_error})."
                )
            elif not task_name:
                feedback_parts.append(
                    "**Metrics present** but no `task_name` was passed — cannot load task `feedback.py`. "
                    "Pass `task_name` (e.g. `category_1_01` or `Category1_Statics_Equilibrium/S_01`) to "
                    "`format_feedback` / `_compose_feedback`."
                )
            elif task_feedback_module is None:
                generic_parts = _format_generic_execution_metrics(metrics)
                if generic_parts:
                    feedback_parts.extend(generic_parts)
                else:
                    feedback_parts.append(
                        f"**No task feedback module** for `{task_name}` "
                        "(missing `tasks/.../feedback.py` or failed to load)."
                    )
            else:
                feedback_parts.append(
                    "**Metrics available but task-specific formatting not found**"
                )

    # Add improvement suggestions (only if include_suggestions is True)
    if include_suggestions:
        feedback_parts.append("\n## Improvement Suggestions\n")

        # Generic error suggestions (applicable to all tasks)
        if error:
            error_lower = error.lower()
            if "syntax error" in error_lower or "invalid syntax" in error_lower:
                feedback_parts.append(
                    "- Fix syntax errors in the code (check for missing parentheses, brackets, or quotes)"
                )
                feedback_parts.append("- Ensure code blocks are properly closed")
                feedback_parts.append(
                    "- Remove any markdown formatting or non-code text from the output"
                )
            elif "name 'sandbox' is not defined" in error_lower:
                feedback_parts.append(
                    "- Move all code that uses 'sandbox' inside the build_agent function"
                )
                feedback_parts.append("- Do not use 'sandbox' variable at module level")
            else:
                feedback_parts.append(
                    "- Review the error details above to identify the specific issue"
                )
                feedback_parts.append(
                    "- Ensure code follows the required function structure (build_agent and optionally agent_action)"
                )

        # Try to get task-specific suggestions
        task_suggestions = []
        if task_feedback_module and hasattr(
            task_feedback_module, "get_improvement_suggestions"
        ):
            try:
                task_suggestions = task_feedback_module.get_improvement_suggestions(
                    metrics, score, success, failed, failure_reason, error
                )
            except (TypeError, Exception):
                pass

        # Add task-specific suggestions
        if task_suggestions:
            for suggestion in task_suggestions:
                feedback_parts.append(suggestion)
        elif not error:
            # Fallback: if no task-specific suggestions and no error, provide generic guidance
            feedback_parts.append(
                "- Review the metrics above to identify areas for improvement"
            )

    return "\n".join(feedback_parts)


def format_granular_feedback(
    snapshot_entries,
    iteration: int,
    task_name: str = None,
    include_suggestions: bool = False,
) -> str:
    """
    Build concatenated feedback for process-level granularity.

    snapshot_entries item schema:
      {
        "moment_index": int,
        "total_moments": int,
        "step_count": int,
        "max_steps": int,
        "metrics": dict,
        "score": float,
        "success": bool,
        "failed": bool,
        "failure_reason": str,
        "error": Optional[str],
      }
    """
    if not snapshot_entries:
        return ""

    blocks = []
    total = snapshot_entries[0].get("total_moments", len(snapshot_entries))
    blocks.append(
        "## Multi-Moment Simulation Feedback\n\n"
        "The rollout is sampled at multiple moments to provide process-level supervision.\n"
    )

    for snap in snapshot_entries:
        idx = int(snap.get("moment_index", 1))
        total_i = int(snap.get("total_moments", total))
        step_count = int(snap.get("step_count", 0))
        max_steps = max(1, int(snap.get("max_steps", 1)))
        progress_pct = 100.0 * float(step_count) / float(max_steps)
        if idx >= total_i:
            moment_note = (
                "This is the **final** sampled moment for this run: the metrics below are the "
                "terminal state when the simulation stopped (success, failure, or step limit).\n"
            )
        else:
            moment_note = (
                "This is an **intermediate** sampled moment: the metrics below reflect partial "
                "progress along the rollout, not necessarily the final outcome.\n"
            )
        blocks.append(
            f"\n## Simulation Moment {idx}/{total_i} "
            f"(~{progress_pct:.1f}% of rollout, step {step_count}/{max_steps})\n"
            f"{moment_note}"
        )
        blocks.append(
            format_feedback(
                snap.get("metrics", {}) or {},
                float(snap.get("score", 0.0)),
                bool(snap.get("success", False)),
                bool(snap.get("failed", False)),
                snap.get("failure_reason"),
                iteration=iteration,
                error=snap.get("error"),
                task_name=task_name,
                include_suggestions=include_suggestions,
            )
        )

    return "\n".join(blocks)
