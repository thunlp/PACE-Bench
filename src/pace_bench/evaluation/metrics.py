"""Compute plot-agnostic trajectory and benchmark report metrics.

The metric families are kept in one place so persistence never depends on a
paper plotting script. The module is organized as follows:

1. per-trajectory score, token, error, and code-similarity diagnostics;
2. independent-run distillation and Pass@k/group aggregation;
3. model, strategy, category, stage, and task breakdowns;
4. source-derived task difficulty and reference-similarity analysis; and
5. the public :func:`aggregate` report builder.
"""

from __future__ import annotations

import io
import math
import re
import token
import tokenize
from collections import Counter, defaultdict
from collections.abc import Sequence
from typing import Any

from pace_bench.core.errors import PaceBenchError
from pace_bench.core.types import AttemptRecord, EvaluationResult
from pace_bench.tasks.registry import CATEGORIES, get_registry

INVISIBLE_PARAMETER_KEYWORDS = (
    "gravity",
    "damping",
    "friction",
    "wind",
    "viscosity",
    "restitution",
)

CATEGORY_NAMES_BY_NUMBER = {
    number: category_name
    for number, (category_name, _prefix, _title) in CATEGORIES.items()
}
CATEGORY_NAMES_BY_PREFIX = {
    prefix: category_name for category_name, prefix, _title in CATEGORIES.values()
}

# Audit map for the former plot/table metric families. It is documentation only:
# this package never imports or executes the local legacy analysis directory.
# Method-specific/VLM/CE figures are comparisons of the generic grouped values.
LEGACY_METRIC_PATHS = {
    "Pass@k across independent runs": "pass_at_k.Pass@k",
    "Pass@2 / pass rate": "metrics.pass_rate_percent",
    "Score-Avg": "metrics.score_mean",
    "Score-Std": "metrics.score_std",
    "Iteration-Avg": "metrics.attempts_used_mean",
    "Code-Avg": "metrics.best_code_tokens_mean",
    "Total-Tokens": "metrics.tokens.total",
    "Avg Tokens": "metrics.tokens.mean_per_pair",
    "Cost-Success": "metrics.tokens.per_successful_pair",
    "Error-Dist / Error-Pct": "metrics.error_taxonomy",
    "Pass@2@attempt": "metrics.attempt_discovery",
    "Score@attempt": "metrics.score_by_attempt",
    "Adaptation Efficiency": "metrics.adaptation_efficiency",
    "Budget saturation": "budget_sensitivity",
    "Code similarity / phase similarity / trend / radicality": "metrics.code_similarity",
    "Trajectory failure diagnostics": "metrics.trajectory_diagnostics",
    "Stage degradation": "by_stage",
    "Model x category": "by_model_and_category",
    "Model x stage": "by_model_and_stage",
    "Strategy x model": "by_strategy_and_model",
    "Strategy x category": "by_strategy_and_category",
    "Design fixation by strategy x model x category": (
        "by_strategy_and_model_and_category.*.*.*.error_taxonomy"
    ),
    "Category x stage": "by_category_and_stage",
    "Model parameter scale": "model_scale",
    "Token/pass Pareto frontier": "strategy_efficiency",
    "Strategy complementarity (Cohen's kappa)": "strategy_complementarity_cohens_kappa",
    "Mutation counts / invisible counts / reference tokens": "dataset_metrics",
    "Reference-solution Jaccard / token ratio": "dataset_metrics.*.stages",
    "Reference similarity stage correlations": "reference_similarity_correlations",
    "Reference similarity task summary/correlations": "reference_similarity_analysis",
}


# ---------------------------------------------------------------------------
# Per-trajectory metrics
# ---------------------------------------------------------------------------


def _pass_rate(results: Sequence[EvaluationResult]) -> float:
    return (
        0.0 if not results else sum(result.success for result in results) / len(results)
    )


def _mean_score(results: Sequence[EvaluationResult]) -> float:
    return (
        0.0
        if not results
        else sum(result.best_score for result in results) / len(results)
    )


def _score_std(results: Sequence[EvaluationResult]) -> float:
    if len(results) < 2:
        return 0.0
    mean = _mean_score(results)
    return (
        sum((result.best_score - mean) ** 2 for result in results) / (len(results) - 1)
    ) ** 0.5


def _sample_std(values: Sequence[float]) -> float:
    if len(values) < 2:
        return 0.0
    mean = sum(values) / len(values)
    return math.sqrt(sum((value - mean) ** 2 for value in values) / (len(values) - 1))


def _count_code_tokens(code: str) -> int:
    """Count Python tokens with the same rule used by the legacy result tables."""

    if not code.strip():
        return 0
    try:
        stream = tokenize.generate_tokens(io.StringIO(code).readline)
        ignored = {
            token.NEWLINE,
            token.NL,
            token.INDENT,
            token.DEDENT,
            token.ENCODING,
            token.ENDMARKER,
            token.COMMENT,
        }
        return sum(1 for item in stream if item.type not in ignored)
    except (tokenize.TokenError, IndentationError):
        return sum(
            bool(line.strip()) and not line.strip().startswith("#")
            for line in code.splitlines()
        )


def _ordered_attempts(result: EvaluationResult) -> list[AttemptRecord]:
    return sorted(result.attempts, key=lambda item: item.attempt)


def _score_at_iteration(result: EvaluationResult, iteration_count: int) -> float:
    """Return cumulative score after N verified records, including attempt 0.

    The former result tables treated the adaptation reference check as the
    first history entry. Keeping that ordinal convention here preserves
    Score@1..20 and Iteration-Avg for existing benchmark comparisons.
    """

    scores = [item.score for item in _ordered_attempts(result)[:iteration_count]]
    return max(scores, default=0.0)


def _trajectory_diagnostics(result: EvaluationResult) -> dict[str, Any]:
    """Return all trajectory signals consumed by the legacy analysis plots."""

    attempts = _ordered_attempts(result)
    if not attempts:
        return {
            "first_positive_index": -1,
            "score_trend": 0.0,
            "unique_failure_reasons": 0,
            "total_constraint_violations": 0,
            "persistent_constraint_violation": False,
            "code_length_trend": 0.0,
            "improved_late": False,
            "code_similarity_recent": 0.0,
            "code_similarity_early": 0.0,
            "code_similarity_middle": 0.0,
            "code_similarity_late": 0.0,
            "code_similarity_global": 0.0,
            "code_similarity_trend": 0.0,
            "code_radicality": 0.0,
        }

    scores = [item.score for item in attempts]
    first_positive = next(
        (index for index, score in enumerate(scores) if score > 0), -1
    )
    third = max(1, len(scores) // 3)
    early_mean = sum(scores[:third]) / third
    late_start = max(third * 2, len(scores) - third)
    late_scores = scores[late_start:]
    score_trend = sum(late_scores) / max(1, len(late_scores)) - early_mean
    improved_late = False
    if len(scores) >= 10:
        midpoint = len(scores) // 2
        improved_late = max(scores[midpoint:]) > max(scores[:midpoint]) + 2

    failure_reasons = {
        str(item.verification.metrics.get("failure_reason") or "").strip().lower()[:80]
        for item in attempts
        if item.verification.metrics.get("failure_reason")
    }
    violation_counts = [
        len(
            _constraint_violations(
                item.verification.metrics,
                _optional_text(item.verification.metrics.get("failure_reason")),
            )
        )
        for item in attempts
    ]
    persistent_violation = len(violation_counts) >= 3 and all(
        count > 0 for count in violation_counts[-3:]
    )
    code_lengths = [len(item.code) for item in attempts]
    code_length_trend = (
        (code_lengths[-1] - code_lengths[0]) / max(1, code_lengths[0])
        if code_lengths[0] > 0
        else 0.0
    )

    similarities: list[float] = []
    for previous, current in zip(attempts, attempts[1:]):
        previous_tokens = set(previous.code.split())
        current_tokens = set(current.code.split())
        similarities.append(
            len(previous_tokens & current_tokens)
            / max(1, len(previous_tokens | current_tokens))
            if previous_tokens and current_tokens
            else 0.0
        )
    recent = similarities[-5:]
    recent_similarity = sum(recent) / len(recent) if recent else 0.0
    if len(similarities) >= 9:
        similarity_third = max(1, len(similarities) // 3)
        early_similarity = sum(similarities[:similarity_third]) / similarity_third
        middle_similarity = (
            sum(similarities[similarity_third : 2 * similarity_third])
            / similarity_third
        )
        late_values = similarities[2 * similarity_third :]
        late_similarity = sum(late_values) / max(1, len(late_values))
        similarity_trend = late_similarity - early_similarity
    else:
        early_similarity = middle_similarity = late_similarity = recent_similarity
        similarity_trend = 0.0
    global_similarity = sum(similarities) / len(similarities) if similarities else 0.0
    return {
        "first_positive_index": first_positive,
        "score_trend": round(score_trend, 6),
        "unique_failure_reasons": len(failure_reasons),
        "total_constraint_violations": sum(violation_counts),
        "persistent_constraint_violation": persistent_violation,
        "code_length_trend": round(code_length_trend, 6),
        "improved_late": improved_late,
        "code_similarity_recent": round(recent_similarity, 6),
        "code_similarity_early": round(early_similarity, 6),
        "code_similarity_middle": round(middle_similarity, 6),
        "code_similarity_late": round(late_similarity, 6),
        "code_similarity_global": round(global_similarity, 6),
        "code_similarity_trend": round(similarity_trend, 6),
        "code_radicality": round(1.0 - global_similarity, 6),
    }


def trajectory_metrics(result: EvaluationResult) -> dict[str, Any]:
    """Compute the compact per-run metrics required by benchmark analysis."""

    attempts = _ordered_attempts(result)
    best = max(attempts, key=lambda item: item.score, default=None)
    prompt_tokens = completion_tokens = total_tokens = 0
    for attempt in attempts:
        usage = attempt.generation.token_usage if attempt.generation else {}
        prompt_tokens += int(usage.get("prompt_tokens", 0) or 0)
        completion_tokens += int(usage.get("completion_tokens", 0) or 0)
        total_tokens += int(
            usage.get(
                "total_tokens",
                (usage.get("prompt_tokens", 0) or 0)
                + (usage.get("completion_tokens", 0) or 0),
            )
            or 0
        )
    if not total_tokens:
        total_tokens = prompt_tokens + completion_tokens
    # Schema-2 strategy runs audit every code-generation call, including
    # unverified inner candidates used by methods such as Self-Refine. Prefer
    # that complete accounting when present; legacy files fall back to the
    # generation attached to each verified AttemptRecord above.
    strategy_runtime = dict(result.metadata.get("strategy_runtime") or {})
    candidate_usage = dict(strategy_runtime.get("candidate_usage") or {})
    candidate_calls = int(candidate_usage.get("calls", 0) or 0)
    candidate_tokens = dict(candidate_usage.get("token_usage") or {})
    if candidate_calls:
        prompt_tokens = int(candidate_tokens.get("prompt_tokens", 0) or 0)
        completion_tokens = int(candidate_tokens.get("completion_tokens", 0) or 0)
        total_tokens = int(
            candidate_tokens.get("total_tokens", prompt_tokens + completion_tokens) or 0
        )
        if not total_tokens:
            total_tokens = prompt_tokens + completion_tokens
    auxiliary_usage = dict(result.metadata.get("auxiliary_usage") or {})
    auxiliary_tokens = dict(auxiliary_usage.get("token_usage") or {})
    auxiliary_prompt_tokens = int(auxiliary_tokens.get("prompt_tokens", 0) or 0)
    auxiliary_completion_tokens = int(auxiliary_tokens.get("completion_tokens", 0) or 0)
    auxiliary_total_tokens = int(
        auxiliary_tokens.get(
            "total_tokens",
            auxiliary_prompt_tokens + auxiliary_completion_tokens,
        )
        or 0
    )
    if not auxiliary_total_tokens:
        auxiliary_total_tokens = auxiliary_prompt_tokens + auxiliary_completion_tokens

    best_metrics = best.verification.metrics if best else {}
    best_failure_reason = _optional_text(best_metrics.get("failure_reason"))
    best_violations = _constraint_violations(best_metrics, best_failure_reason)
    best_physics = _physical_summary(best_metrics)
    cumulative_scores: list[dict[str, float | int]] = []
    running_best = -math.inf
    for attempt in attempts:
        running_best = max(running_best, attempt.score)
        cumulative_scores.append(
            {"attempt": attempt.attempt, "best_score": running_best}
        )
    success_attempt = min(
        (item.attempt for item in attempts if item.success), default=None
    )
    return {
        "verified_attempts": len(attempts),
        "revision_attempts": sum(item.attempt > 0 for item in attempts),
        "success_attempt": success_attempt,
        "best_code_tokens": _count_code_tokens(best.code) if best else 0,
        "cumulative_best_scores": cumulative_scores,
        "tokens": {
            # These three compatibility fields count candidate-producing calls,
            # matching historical result files and plots.
            "prompt": prompt_tokens,
            "completion": completion_tokens,
            "total": total_tokens,
            # Auxiliary calls (reflection, memory induction, etc.) do not spend
            # sandbox attempts, but remain visible for complete cost analysis.
            "auxiliary_prompt": auxiliary_prompt_tokens,
            "auxiliary_completion": auxiliary_completion_tokens,
            "auxiliary_total": auxiliary_total_tokens,
            "all_calls_total": total_tokens + auxiliary_total_tokens,
        },
        "error_type": _trajectory_error_type(result),
        "best_attempt_diagnostics": {
            "failure_reason": best_failure_reason,
            "step_count": int(best_metrics.get("step_count", 0) or 0),
            "constraint_violation_count": len(best_violations),
            "structure_broken": bool(best_physics.get("structure_broken", False)),
            "joints_broken_count": int(best_physics.get("joints_broken_count", 0) or 0),
            "progress": float(best_physics.get("progress", 0.0) or 0.0),
        },
        "trajectory_diagnostics": _trajectory_diagnostics(result),
    }


# ---------------------------------------------------------------------------
# Independent-run and group aggregation
# ---------------------------------------------------------------------------


def _result_run_index(result: EvaluationResult) -> int:
    try:
        return int(result.config.get("run_index", 1))
    except (TypeError, ValueError):
        return 1


def _pair_key(result: EvaluationResult) -> tuple[str, ...]:
    environment = result.environment_pair or result.target_environment or "unknown"
    return (
        result.task_path,
        environment,
        result.mode,
        result.provider,
        result.model,
        result.strategy,
    )


def _pass_at_k(results: Sequence[EvaluationResult]) -> dict[str, Any]:
    grouped: dict[tuple[str, ...], list[EvaluationResult]] = defaultdict(list)
    for result in results:
        grouped[_pair_key(result)].append(result)
    for trajectories in grouped.values():
        trajectories.sort(key=_result_run_index)
    max_runs = max((len(items) for items in grouped.values()), default=0)
    values: dict[str, Any] = {}
    for k in range(1, max_runs + 1):
        eligible = [items for items in grouped.values() if len(items) >= k]
        passed = sum(any(item.success for item in items[:k]) for items in eligible)
        values[f"Pass@{k}"] = {
            "rate": 0.0 if not eligible else passed / len(eligible),
            "passed_pairs": passed,
            "pair_count": len(eligible),
        }
    return values


def pass_at_k_by_group(
    results: Sequence[EvaluationResult],
    key_function: Any,
    k: int,
) -> dict[str, dict[str, float | int]]:
    """Compute pair-aware Pass@k for arbitrary report groups.

    This is the grouped counterpart of :func:`_pass_at_k`, exposed for paper
    exporters that need model/category/method rows.  A pair is eligible only
    when it has at least ``k`` independently indexed runs, matching the
    top-level Pass@k definition.
    """

    grouped: dict[str, dict[tuple[str, ...], list[EvaluationResult]]] = defaultdict(
        lambda: defaultdict(list)
    )
    for result in results:
        grouped[str(key_function(result))][_pair_key(result)].append(result)

    report: dict[str, dict[str, float | int]] = {}
    for group, pairs in sorted(grouped.items()):
        eligible = 0
        passed = 0
        for trajectories in pairs.values():
            trajectories.sort(key=_result_run_index)
            if len(trajectories) < k:
                continue
            eligible += 1
            passed += int(any(item.success for item in trajectories[:k]))
        report[group] = {
            "rate": passed / eligible if eligible else 0.0,
            "passed_pairs": passed,
            "pair_count": eligible,
        }
    return report


def _run_count_summary(results: Sequence[EvaluationResult]) -> dict[str, int]:
    counts = Counter(_pair_key(result) for result in results)
    values = list(counts.values())
    return {
        "minimum": min(values, default=0),
        "maximum": max(values, default=0),
    }


def _error_taxonomy_summary(results: Sequence[EvaluationResult]) -> dict[str, Any]:
    counts = Counter(_trajectory_error_type(result) for result in results)
    total = len(results)
    failures = total - counts.get("success", 0)
    return {
        "counts": dict(sorted(counts.items())),
        "rates": {key: count / total for key, count in sorted(counts.items())}
        if total
        else {},
        "failure_only_rates": {
            key: count / failures
            for key, count in sorted(counts.items())
            if key != "success"
        }
        if failures
        else {},
    }


def _pair_error_taxonomy_summary(
    results: Sequence[EvaluationResult],
) -> dict[str, Any]:
    grouped: dict[tuple[str, ...], list[EvaluationResult]] = defaultdict(list)
    for result in results:
        grouped[_pair_key(result)].append(result)
    labels: list[str] = []
    for trajectories in grouped.values():
        trajectories.sort(key=_result_run_index)
        if any(item.success for item in trajectories):
            labels.append("success")
            continue
        labels.append(
            _majority(
                [_trajectory_error_type(item) for item in trajectories],
                default="stagnation",
            )
        )
    synthetic = Counter(labels)
    total = len(labels)
    failures = total - synthetic.get("success", 0)
    return {
        "counts": dict(sorted(synthetic.items())),
        "rates": {key: count / total for key, count in sorted(synthetic.items())}
        if total
        else {},
        "failure_only_rates": {
            key: count / failures
            for key, count in sorted(synthetic.items())
            if key != "success"
        }
        if failures
        else {},
    }


def _mean(values: Sequence[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def _majority(values: Sequence[str], *, default: str) -> str:
    """Return the mode, preserving DaVinci result.py's first-seen tie break."""

    counts = Counter(values)
    return max(counts, key=counts.get) if counts else default


def _target_stage(result: EvaluationResult) -> str:
    if result.target_environment:
        return result.target_environment
    if result.environment_pair and "_to_" in result.environment_pair:
        return result.environment_pair.split("_to_", 1)[1]
    return "unknown"


def _category(result: EvaluationResult) -> str:
    if "/" in result.task_path:
        return result.task_path.split("/", 1)[0]
    prefix = result.task_id.upper().split("_", 1)[0]
    if prefix in CATEGORY_NAMES_BY_PREFIX:
        return CATEGORY_NAMES_BY_PREFIX[prefix]
    match = re.search(r"category[_-]?(\d+)", result.task_path, re.IGNORECASE)
    if match:
        return CATEGORY_NAMES_BY_NUMBER.get(int(match.group(1)), "other")
    return "other"


def _pair_records(results: Sequence[EvaluationResult]) -> list[dict[str, Any]]:
    """Distill independent runs into one auditable record per benchmark pair."""

    grouped: dict[tuple[str, ...], list[EvaluationResult]] = defaultdict(list)
    for result in results:
        grouped[_pair_key(result)].append(result)
    records: list[dict[str, Any]] = []
    for key, runs in sorted(grouped.items()):
        runs.sort(key=_result_run_index)
        analyses = [trajectory_metrics(run) for run in runs]
        successful = [
            analysis
            for run, analysis in zip(runs, analyses)
            if run.success and analysis["success_attempt"] is not None
        ]
        attempts_used = min(
            (int(item["verified_attempts"]) for item in successful),
            default=min(
                (int(item["verified_attempts"]) for item in analyses), default=0
            ),
        )
        scores = [run.best_score for run in runs]
        best_run_index = max(range(len(runs)), key=lambda index: runs[index].best_score)
        token_keys = (
            "prompt",
            "completion",
            "total",
            "auxiliary_prompt",
            "auxiliary_completion",
            "auxiliary_total",
            "all_calls_total",
        )
        token_means = {
            name: _mean([float(analysis["tokens"][name]) for analysis in analyses])
            for name in token_keys
        }
        diagnostic_keys = tuple(
            analyses[0]["trajectory_diagnostics"].keys() if analyses else ()
        )
        diagnostics: dict[str, float] = {}
        for name in diagnostic_keys:
            values = [analysis["trajectory_diagnostics"][name] for analysis in analyses]
            diagnostics[name] = _mean([float(value) for value in values])
        max_budget = max(
            (int(run.config.get("attempts", 0) or 0) for run in runs),
            default=0,
        )
        max_budget = max(
            max_budget,
            max((item.attempt for run in runs for item in run.attempts), default=0),
        )
        score_curve = {
            str(attempt): _mean([_score_at_iteration(run, attempt) for run in runs])
            for attempt in range(1, max_budget + 1)
        }
        discovery = {
            str(attempt): any(
                run.success and len(run.attempts) <= attempt for run in runs
            )
            for attempt in range(1, max_budget + 1)
        }
        error_types = [
            str(analysis["error_type"])
            for run, analysis in zip(runs, analyses)
            if not run.success
        ]
        records.append(
            {
                "task": runs[0].task_path,
                "task_id": runs[0].task_id,
                "category": _category(runs[0]),
                "environment": key[1],
                "stage": _target_stage(runs[0]),
                "mode": runs[0].mode,
                "provider": runs[0].provider,
                "model": runs[0].model,
                "strategy": runs[0].strategy,
                "run_count": len(runs),
                "success_count": sum(run.success for run in runs),
                "success": any(run.success for run in runs),
                "best_score_mean": _mean(scores),
                "best_score_std": _sample_std(scores),
                "attempts_used": attempts_used,
                "best_code_tokens": analyses[best_run_index]["best_code_tokens"],
                "tokens_mean": token_means,
                "error_type": "success"
                if any(run.success for run in runs)
                else _majority(error_types, default="stagnation"),
                "score_by_attempt": score_curve,
                "success_by_attempt": discovery,
                "trajectory_diagnostics_mean": diagnostics,
            }
        )
    return records


def _group_metrics(
    results: Sequence[EvaluationResult],
    pair_records: Sequence[dict[str, Any]] | None = None,
) -> dict[str, Any]:
    """Compute every generic metric used by the legacy plots and tables."""

    pairs = list(pair_records if pair_records is not None else _pair_records(results))
    pair_count = len(pairs)
    successful_pairs = sum(bool(item["success"]) for item in pairs)
    max_attempt = max(
        (
            max((int(key) for key in item["score_by_attempt"]), default=0)
            for item in pairs
        ),
        default=0,
    )
    discovery: dict[str, dict[str, float | int]] = {}
    score_curve: dict[str, float] = {}
    for attempt in range(1, max_attempt + 1):
        eligible = [
            item for item in pairs if str(attempt) in item["success_by_attempt"]
        ]
        passed = sum(
            bool(item["success_by_attempt"][str(attempt)]) for item in eligible
        )
        discovery[str(attempt)] = {
            "rate": passed / len(eligible) if eligible else 0.0,
            "rate_percent": passed / len(eligible) * 100 if eligible else 0.0,
            "passed_pairs": passed,
            "pair_count": len(eligible),
        }
        scores = [
            float(item["score_by_attempt"][str(attempt)])
            for item in pairs
            if str(attempt) in item["score_by_attempt"]
        ]
        score_curve[str(attempt)] = _mean(scores)

    error_counts = Counter(str(item["error_type"]) for item in pairs)
    failure_count = pair_count - error_counts.get("success", 0)
    total_tokens = sum(float(item["tokens_mean"]["total"]) for item in pairs)
    prompt_tokens = sum(float(item["tokens_mean"]["prompt"]) for item in pairs)
    completion_tokens = sum(float(item["tokens_mean"]["completion"]) for item in pairs)
    auxiliary_prompt_tokens = sum(
        float(item["tokens_mean"]["auxiliary_prompt"]) for item in pairs
    )
    auxiliary_completion_tokens = sum(
        float(item["tokens_mean"]["auxiliary_completion"]) for item in pairs
    )
    auxiliary_total_tokens = sum(
        float(item["tokens_mean"]["auxiliary_total"]) for item in pairs
    )
    all_calls_tokens = sum(
        float(item["tokens_mean"]["all_calls_total"]) for item in pairs
    )
    mean_attempts = _mean([float(item["attempts_used"]) for item in pairs])
    mean_budget = _mean(
        [float(result.config.get("attempts", 0) or 0) for result in results]
    )
    adaptation_efficiency = (
        max(0.0, (mean_budget + 1.0 - mean_attempts) / mean_budget)
        if mean_budget
        else 0.0
    )
    similarity_names = (
        "code_similarity_recent",
        "code_similarity_early",
        "code_similarity_middle",
        "code_similarity_late",
        "code_similarity_global",
        "code_similarity_trend",
        "code_radicality",
    )
    trajectory_names = (
        "first_positive_index",
        "score_trend",
        "unique_failure_reasons",
        "total_constraint_violations",
        "persistent_constraint_violation",
        "code_length_trend",
        "improved_late",
    )
    return {
        "trajectory_count": len(results),
        "pair_count": pair_count,
        "successful_pairs": successful_pairs,
        "pass_rate": successful_pairs / pair_count if pair_count else 0.0,
        "pass_rate_percent": successful_pairs / pair_count * 100 if pair_count else 0.0,
        "score_mean": _mean([float(item["best_score_mean"]) for item in pairs]),
        "score_std": _mean(
            [
                float(item["best_score_std"])
                for item in pairs
                if int(item["run_count"]) > 1
            ]
        ),
        "attempts_used_mean": mean_attempts,
        "adaptation_efficiency": adaptation_efficiency,
        "best_code_tokens_mean": _mean(
            [float(item["best_code_tokens"]) for item in pairs]
        ),
        "tokens": {
            "prompt_total": prompt_tokens,
            "completion_total": completion_tokens,
            "auxiliary_prompt_total": auxiliary_prompt_tokens,
            "auxiliary_completion_total": auxiliary_completion_tokens,
            "auxiliary_total": auxiliary_total_tokens,
            "total": total_tokens,
            "mean_per_pair": total_tokens / pair_count if pair_count else 0.0,
            "per_successful_pair": total_tokens / successful_pairs
            if successful_pairs
            else None,
            "all_calls_total": all_calls_tokens,
            "all_calls_mean_per_pair": (
                all_calls_tokens / pair_count if pair_count else 0.0
            ),
            "all_calls_per_successful_pair": (
                all_calls_tokens / successful_pairs if successful_pairs else None
            ),
        },
        "attempt_discovery": discovery,
        "score_by_attempt": score_curve,
        "error_taxonomy": {
            "counts": dict(sorted(error_counts.items())),
            "rates": {
                name: count / pair_count for name, count in sorted(error_counts.items())
            }
            if pair_count
            else {},
            "rates_percent": {
                name: count / pair_count * 100
                for name, count in sorted(error_counts.items())
            }
            if pair_count
            else {},
            "failure_only_rates": {
                name: count / failure_count
                for name, count in sorted(error_counts.items())
                if name != "success"
            }
            if failure_count
            else {},
        },
        "code_similarity": {
            name: _mean(
                [
                    float(item["trajectory_diagnostics_mean"].get(name, 0.0))
                    for item in pairs
                ]
            )
            for name in similarity_names
        },
        "trajectory_diagnostics": {
            name: _mean(
                [
                    float(item["trajectory_diagnostics_mean"].get(name, 0.0))
                    for item in pairs
                ]
            )
            for name in trajectory_names
        },
    }


# ---------------------------------------------------------------------------
# One-, two-, and three-dimensional report breakdowns
# ---------------------------------------------------------------------------


def _breakdown(
    results: Sequence[EvaluationResult], key_function: Any
) -> dict[str, Any]:
    grouped: dict[str, list[EvaluationResult]] = defaultdict(list)
    for result in results:
        grouped[str(key_function(result))].append(result)
    return {key: _group_metrics(items) for key, items in sorted(grouped.items())}


def _cross_breakdown(
    results: Sequence[EvaluationResult], outer_function: Any, inner_function: Any
) -> dict[str, Any]:
    grouped: dict[str, dict[str, list[EvaluationResult]]] = defaultdict(
        lambda: defaultdict(list)
    )
    for result in results:
        grouped[str(outer_function(result))][str(inner_function(result))].append(result)
    return {
        outer: {
            inner: _group_metrics(items)
            for inner, items in sorted(inner_groups.items())
        }
        for outer, inner_groups in sorted(grouped.items())
    }


def _three_way_breakdown(
    results: Sequence[EvaluationResult],
    outer_function: Any,
    middle_function: Any,
    inner_function: Any,
) -> dict[str, Any]:
    """Group results across the three dimensions used by fixation analyses."""

    grouped: dict[str, dict[str, dict[str, list[EvaluationResult]]]] = defaultdict(
        lambda: defaultdict(lambda: defaultdict(list))
    )
    for result in results:
        grouped[str(outer_function(result))][str(middle_function(result))][
            str(inner_function(result))
        ].append(result)
    return {
        outer: {
            middle: {
                inner: _group_metrics(items)
                for inner, items in sorted(inner_groups.items())
            }
            for middle, inner_groups in sorted(middle_groups.items())
        }
        for outer, middle_groups in sorted(grouped.items())
    }


def _cohens_kappa(
    left: dict[tuple[str, ...], bool], right: dict[tuple[str, ...], bool]
) -> float | None:
    common = sorted(set(left) & set(right))
    if not common:
        return None
    both = left_only = right_only = neither = 0
    for identity in common:
        left_value, right_value = left[identity], right[identity]
        if left_value and right_value:
            both += 1
        elif left_value:
            left_only += 1
        elif right_value:
            right_only += 1
        else:
            neither += 1
    total = len(common)
    observed = (both + neither) / total
    expected = (
        (both + left_only) * (both + right_only)
        + (right_only + neither) * (left_only + neither)
    ) / (total**2)
    return (observed - expected) / (1 - expected) if expected < 1 else 1.0


def _strategy_complementarity(results: Sequence[EvaluationResult]) -> dict[str, Any]:
    outcomes: dict[str, dict[tuple[str, ...], bool]] = defaultdict(dict)
    grouped: dict[tuple[str, tuple[str, ...]], list[EvaluationResult]] = defaultdict(
        list
    )
    for result in results:
        identity = (
            result.task_path,
            result.environment_pair or result.target_environment or "unknown",
            result.mode,
            result.provider,
            result.model,
        )
        grouped[(result.strategy, identity)].append(result)
    for (strategy, identity), runs in grouped.items():
        outcomes[strategy][identity] = any(run.success for run in runs)
    strategies = sorted(outcomes)
    return {
        left: {
            right: (
                1.0 if left == right else _cohens_kappa(outcomes[left], outcomes[right])
            )
            for right in strategies
        }
        for left in strategies
    }


# ---------------------------------------------------------------------------
# Dataset difficulty and reference-solution similarity
# ---------------------------------------------------------------------------


def _model_parameter_billions(model: str) -> float | None:
    match = re.search(r"(\d+(?:\.\d+)?)B(?:\b|[-_])", model, re.IGNORECASE)
    return float(match.group(1)) if match else None


def _pearson(left: Sequence[float], right: Sequence[float]) -> float | None:
    if len(left) != len(right) or len(left) < 2:
        return None
    left_mean, right_mean = _mean(left), _mean(right)
    numerator = sum(
        (left_value - left_mean) * (right_value - right_mean)
        for left_value, right_value in zip(left, right)
    )
    left_scale = math.sqrt(sum((value - left_mean) ** 2 for value in left))
    right_scale = math.sqrt(sum((value - right_mean) ** 2 for value in right))
    denominator = left_scale * right_scale
    return numerator / denominator if denominator else None


def _reference_similarity(left: str, right: str) -> dict[str, float | int]:
    left_tokens, right_tokens = set(left.split()), set(right.split())
    union = left_tokens | right_tokens
    return {
        "jaccard": len(left_tokens & right_tokens) / len(union) if union else 0.0,
        "initial_unique_tokens": len(left_tokens),
        "stage_unique_tokens": len(right_tokens),
        "token_ratio": len(right_tokens) / max(1, len(left_tokens)),
    }


def _legacy_reference_code(source: str, environment: str) -> str:
    """Extract the same function bodies used by the former similarity plots."""

    def body(name: str) -> str:
        match = re.search(
            rf"^def {name}\(.+?\):(.+?)(?=\n(?:def |$))",
            source,
            re.DOTALL | re.MULTILINE,
        )
        return match.group(1) if match else ""

    if environment == "Initial":
        return body("build_agent") + body("agent_action")
    stage_number = environment.rsplit("-", 1)[-1]
    return body(f"build_agent_stage_{stage_number}") + body(
        f"agent_action_stage_{stage_number}"
    )


def _dataset_profiles(results: Sequence[EvaluationResult]) -> dict[str, Any]:
    """Count source-derived task metrics used by legacy difficulty analyses."""

    registry = get_registry()
    profiles: dict[str, Any] = {}
    for task_path in sorted({result.task_path for result in results}):
        try:
            task = registry.resolve(task_path)
            if not task.benchmark:
                continue
            environments = registry.environments(task)
            agent_source = (task.path / "agent.py").read_text(encoding="utf-8")
            initial_reference = _legacy_reference_code(agent_source, "Initial")
            stages: dict[str, Any] = {}
            parameter_counts: list[int] = []
            invisible_counts: list[int] = []
            reference_similarities: list[float] = []
            for environment in environments:
                if environment.environment_id.value == "Initial":
                    continue
                raw_terrain = dict(environment.raw.get("terrain_config") or {})
                raw_physics = dict(environment.raw.get("physics_config") or {})
                names = [*raw_terrain, *raw_physics]
                invisible = sum(
                    any(
                        keyword in name.lower()
                        for keyword in INVISIBLE_PARAMETER_KEYWORDS
                    )
                    for name in names
                )
                parameter_counts.append(len(names))
                invisible_counts.append(invisible)
                stage_reference = _legacy_reference_code(
                    agent_source, str(environment.environment_id)
                )
                similarity = _reference_similarity(initial_reference, stage_reference)
                reference_similarities.append(float(similarity["jaccard"]))
                stages[str(environment.environment_id)] = {
                    "mutated_parameter_count": len(names),
                    "legacy_invisible_parameter_count": invisible,
                    "reference_similarity": similarity,
                }
            profiles[task_path] = {
                "mutated_parameters_mean": _mean(
                    [float(value) for value in parameter_counts]
                ),
                "mutated_parameters_max": max(parameter_counts, default=0),
                "legacy_invisible_parameters_mean": _mean(
                    [float(value) for value in invisible_counts]
                ),
                "reference_file_tokens": len(agent_source.split()),
                "reference_similarity_mean": _mean(reference_similarities),
                "stages": stages,
            }
        except (
            OSError,
            ValueError,
            ImportError,
            AttributeError,
            PaceBenchError,
        ) as exc:
            profiles[task_path] = {"error": str(exc)}
    return profiles


def _reference_similarity_correlations(
    pair_records: Sequence[dict[str, Any]], profiles: dict[str, Any]
) -> dict[str, Any]:
    """Retain the schema-1.0 stage-level correlation view by model."""

    by_model: dict[str, list[dict[str, float]]] = defaultdict(list)
    for pair in pair_records:
        profile = profiles.get(str(pair["task"]), {})
        stage = profile.get("stages", {}).get(str(pair["stage"]), {})
        similarity = stage.get("reference_similarity", {}).get("jaccard")
        if similarity is None:
            continue
        by_model[str(pair["model"])].append(
            {
                "similarity": float(similarity),
                "pass": float(bool(pair["success"])),
                "score": float(pair["best_score_mean"]),
            }
        )
    return {
        model: {
            "point_count": len(points),
            "similarity_vs_pass_rate_pearson_r": _pearson(
                [point["similarity"] for point in points],
                [point["pass"] for point in points],
            ),
            "similarity_vs_score_pearson_r": _pearson(
                [point["similarity"] for point in points],
                [point["score"] for point in points],
            ),
        }
        for model, points in sorted(by_model.items())
    }


def _reference_similarity_analysis(
    pair_records: Sequence[dict[str, Any]], profiles: dict[str, Any]
) -> dict[str, Any]:
    """Build the stage- and task-level views used by legacy similarity plots.

    Results are separated by both model and strategy. This avoids mixing
    custom strategies when researchers add them to the same report while still
    supporting the former vanilla-only table and scatter plots.
    """

    stage_points: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    for pair in pair_records:
        profile = profiles.get(str(pair["task"]), {})
        stage_profile = profile.get("stages", {}).get(str(pair["stage"]), {})
        similarity = stage_profile.get("reference_similarity", {}).get("jaccard")
        if similarity is None:
            continue
        stage_points[(str(pair["model"]), str(pair["strategy"]))].append(
            {
                "task": str(pair["task"]),
                "category": str(pair["category"]),
                "stage": str(pair["stage"]),
                "reference_jaccard": float(similarity),
                "pass_rate": float(bool(pair["success"])),
                "score_mean": float(pair["best_score_mean"]),
            }
        )

    report: dict[str, dict[str, Any]] = defaultdict(dict)
    for (model, strategy), points in sorted(stage_points.items()):
        by_task: dict[str, list[dict[str, Any]]] = defaultdict(list)
        for point in points:
            by_task[str(point["task"])].append(point)
        task_rows = []
        for task, task_points in sorted(by_task.items()):
            task_rows.append(
                {
                    "task": task,
                    "category": str(task_points[0]["category"]),
                    "stage_count": len(task_points),
                    "reference_jaccard_mean": _mean(
                        [float(point["reference_jaccard"]) for point in task_points]
                    ),
                    "pass_rate_mean": _mean(
                        [float(point["pass_rate"]) for point in task_points]
                    ),
                    "score_mean": _mean(
                        [float(point["score_mean"]) for point in task_points]
                    ),
                }
            )
        report[model][strategy] = {
            "stage_level": {
                "point_count": len(points),
                "similarity_vs_pass_rate_pearson_r": _pearson(
                    [float(point["reference_jaccard"]) for point in points],
                    [float(point["pass_rate"]) for point in points],
                ),
                "similarity_vs_score_pearson_r": _pearson(
                    [float(point["reference_jaccard"]) for point in points],
                    [float(point["score_mean"]) for point in points],
                ),
            },
            "task_level": {
                "point_count": len(task_rows),
                "similarity_vs_pass_rate_pearson_r": _pearson(
                    [float(row["reference_jaccard_mean"]) for row in task_rows],
                    [float(row["pass_rate_mean"]) for row in task_rows],
                ),
                "similarity_vs_score_pearson_r": _pearson(
                    [float(row["reference_jaccard_mean"]) for row in task_rows],
                    [float(row["score_mean"]) for row in task_rows],
                ),
                "tasks": task_rows,
            },
        }
    return {model: dict(strategies) for model, strategies in sorted(report.items())}


# ---------------------------------------------------------------------------
# Public benchmark report
# ---------------------------------------------------------------------------


def aggregate(results: list[EvaluationResult]) -> dict[str, Any]:
    """Build the versioned, plot-agnostic benchmark analysis report."""

    pair_records = _pair_records(results)
    overall = _group_metrics(results, pair_records)
    profiles = _dataset_profiles(results)
    by_category = _breakdown(results, _category)
    by_stage = _breakdown(results, _target_stage)
    by_model = _breakdown(results, lambda item: item.model)
    by_strategy = _breakdown(results, lambda item: item.strategy)
    by_task = _breakdown(results, lambda item: item.task_path)

    strategy_efficiency = {
        strategy: {
            "pass_rate": values["pass_rate"],
            "score_mean": values["score_mean"],
            "mean_tokens_per_pair": values["tokens"]["all_calls_mean_per_pair"],
            "tokens_per_successful_pair": values["tokens"][
                "all_calls_per_successful_pair"
            ],
            "candidate_tokens_mean_per_pair": values["tokens"]["mean_per_pair"],
            "auxiliary_tokens_total": values["tokens"]["auxiliary_total"],
        }
        for strategy, values in by_strategy.items()
    }
    best_pass_rate = -1.0
    for strategy, values in sorted(
        strategy_efficiency.items(),
        key=lambda item: (item[1]["mean_tokens_per_pair"], item[0]),
    ):
        values["pareto_efficient"] = values["pass_rate"] > best_pass_rate
        best_pass_rate = max(best_pass_rate, values["pass_rate"])

    def budget_summary(values: dict[str, Any]) -> dict[str, Any]:
        curve = values["attempt_discovery"]
        final_key = max(curve, key=int) if curve else None
        final_rate = float(curve[final_key]["rate"]) if final_key else 0.0
        selected: dict[str, Any] = {}
        for budget in (1, 5, 10, 15, 20):
            point = curve.get(str(budget))
            if point is None:
                continue
            selected[str(budget)] = {
                **point,
                "saturation_of_final": (
                    float(point["rate"]) / final_rate if final_rate else 0.0
                ),
            }
        return {"final_attempt": int(final_key) if final_key else 0, "points": selected}

    return {
        "report_schema_version": "1.0",
        "legacy_metric_coverage": LEGACY_METRIC_PATHS,
        "result_schema_versions": sorted({result.schema_version for result in results}),
        "result_count": len(results),
        "trajectory_count": len(results),
        "pair_count": len(pair_records),
        # Stable top-level compatibility keys retained from schema 2.0 reports.
        "pass_rate": overall["pass_rate"],
        "trajectory_pass_rate": _pass_rate(results),
        "pass_at_k": _pass_at_k(results),
        "runs_per_pair": _run_count_summary(results),
        "error_taxonomy": _pair_error_taxonomy_summary(results),
        "trajectory_error_taxonomy": _error_taxonomy_summary(results),
        "mean_best_score": overall["score_mean"],
        "best_score_std": overall["score_std"],
        "mean_verified_attempts": (
            _mean([float(len(result.attempts)) for result in results])
        ),
        "stop_reasons": dict(
            sorted(Counter(item.stop_reason for item in results).items())
        ),
        "metrics": overall,
        "pairs": pair_records,
        "by_category": by_category,
        "by_stage": by_stage,
        "by_model": by_model,
        "by_strategy": by_strategy,
        "by_task": by_task,
        "by_model_and_category": _cross_breakdown(
            results, lambda item: item.model, _category
        ),
        "by_model_and_stage": _cross_breakdown(
            results, lambda item: item.model, _target_stage
        ),
        "by_strategy_and_model": _cross_breakdown(
            results, lambda item: item.strategy, lambda item: item.model
        ),
        "by_strategy_and_category": _cross_breakdown(
            results, lambda item: item.strategy, _category
        ),
        "by_strategy_and_model_and_category": _three_way_breakdown(
            results,
            lambda item: item.strategy,
            lambda item: item.model,
            _category,
        ),
        "by_category_and_stage": _cross_breakdown(results, _category, _target_stage),
        "strategy_efficiency": strategy_efficiency,
        "budget_sensitivity": {
            "overall": budget_summary(overall),
            "by_category": {
                category: budget_summary(values)
                for category, values in by_category.items()
            },
        },
        "strategy_complementarity_cohens_kappa": _strategy_complementarity(results),
        "model_scale": {
            model: {
                "parameter_billions": _model_parameter_billions(model),
                "pass_rate": values["pass_rate"],
                "score_mean": values["score_mean"],
                "adaptation_efficiency": values["adaptation_efficiency"],
            }
            for model, values in by_model.items()
        },
        "dataset_metrics": profiles,
        "reference_similarity_correlations": _reference_similarity_correlations(
            pair_records, profiles
        ),
        "reference_similarity_analysis": _reference_similarity_analysis(
            pair_records, profiles
        ),
    }


# ---------------------------------------------------------------------------
# Error taxonomy and compact physical diagnostics
# ---------------------------------------------------------------------------


def _constraint_violations(
    metrics: dict[str, Any], failure_reason: str | None
) -> list[str]:
    values: list[str] = []
    for key in ("constraint_violations", "design_violations", "violations"):
        raw = metrics.get(key)
        if isinstance(raw, str):
            values.append(raw)
        elif isinstance(raw, Sequence) and not isinstance(raw, (str, bytes)):
            values.extend(str(item) for item in raw if item is not None)
    if not values and failure_reason and "constraint" in failure_reason.lower():
        detail = (
            failure_reason.split(":", 1)[1] if ":" in failure_reason else failure_reason
        )
        values.extend(item.strip() for item in detail.split(";") if item.strip())
    return list(dict.fromkeys(values))


def _physical_summary(metrics: dict[str, Any]) -> dict[str, Any]:
    """Retain only physical scalars consumed by benchmark error analysis."""

    summary: dict[str, Any] = {}
    if "structure_broken" in metrics:
        summary["structure_broken"] = bool(metrics["structure_broken"])
    broken = metrics.get("joints_broken_count")
    if broken is None:
        events = metrics.get("joint_failure_events")
        if isinstance(events, Sequence) and not isinstance(events, (str, bytes)):
            broken = len(events)
        elif isinstance(events, (int, float)):
            broken = int(events)
    if broken is not None:
        summary["joints_broken_count"] = int(broken)
    progress = metrics.get("progress")
    if progress is None:
        progress = metrics.get("progress_pct")
    if isinstance(progress, (int, float)):
        summary["progress"] = float(progress)
    return summary


def _trajectory_error_type(result: EvaluationResult) -> str:
    """Classify a trajectory using the lightweight signals from old result.py."""

    if result.success:
        return "success"
    if not result.attempts:
        return "stagnation"
    best = max(result.attempts, key=lambda item: item.score)
    best_metrics = best.verification.metrics
    if result.best_score <= -60:
        return "catastrophic_collapse"
    if _constraint_violations(
        best_metrics, _optional_text(best_metrics.get("failure_reason"))
    ):
        return "constraint_violation"
    physics = _physical_summary(best_metrics)
    if physics.get("structure_broken") or physics.get("joints_broken_count", 0) > 0:
        return "structural_failure"
    if result.best_score < 0:
        return "numerical_instability"

    diagnostics = _trajectory_diagnostics(result)
    score_trend = diagnostics["score_trend"]
    improved_late = diagnostics["improved_late"]
    first_positive = diagnostics["first_positive_index"]
    unique_failures = diagnostics["unique_failure_reasons"]
    mean_recent_similarity = diagnostics["code_similarity_recent"]
    if (
        mean_recent_similarity > 0.85
        and abs(score_trend) < 3
        and result.best_score < 50
    ):
        return "design_fixation"
    if improved_late and result.best_score >= 30:
        return "late_convergence"
    if first_positive < 0 and abs(score_trend) < 2 and unique_failures <= 1:
        return "stagnation"
    if first_positive < 0 and unique_failures >= 3:
        return "exploration"
    if 0 < result.best_score < 100:
        return "late_convergence" if improved_late else "budget_exhaustion"
    return "stagnation"


def _optional_text(value: Any) -> str | None:
    if value is None:
        return None
    rendered = str(value).strip()
    return rendered or None
