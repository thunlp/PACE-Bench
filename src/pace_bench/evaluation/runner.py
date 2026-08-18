"""Deterministic work enumeration, persistence, parallelism, and reference checks."""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass, replace

from pace_bench.evaluation.config import RunConfig
from pace_bench.evaluation.engine import EvaluationEngine
from pace_bench.evaluation.results import (
    is_complete,
    load_result,
    result_path,
    result_path_candidates,
    save_result,
)
from pace_bench.evaluation.verification.verifier import PhysicsVerifier
from pace_bench.tasks.registry import get_reference_solution
from pace_bench.tasks.registry import TaskRegistry, get_registry
from pace_bench.tasks.registry import max_steps_for_task
from pace_bench.tasks.registry import EnvironmentSpec, TaskSpec
from pace_bench.core.types import EnvironmentId, EvaluationResult, RunMode


@dataclass(frozen=True)
class WorkItem:
    index: int
    config: RunConfig

    @property
    def identity(self) -> str:
        environment = (
            f"{self.config.source}_to_{self.config.target}"
            if self.config.mode == RunMode.ADAPTATION
            else str(self.config.target)
        )
        return (
            f"{self.config.task}:{environment}:run-{self.config.run_index}"
        )


@dataclass(frozen=True)
class WorkOutcome:
    work_item: WorkItem
    result: EvaluationResult | None
    error: str | None = None


def enumerate_work_items(
    base_config: RunConfig,
    *,
    task_selectors: list[str] | tuple[str, ...] | str,
    stages: list[str] | tuple[str, ...] | str = "all",
    runs: int = 1,
    registry: TaskRegistry | None = None,
) -> tuple[WorkItem, ...]:
    """Enumerate selected task/environment/run triples in canonical order."""

    if runs < 1:
        raise ValueError("runs must be at least 1")
    registry = registry or get_registry()
    tasks = registry.select(task_selectors)
    if isinstance(stages, str):
        stages = [stages]
    if any(value.lower() == "all" for value in stages):
        stage_values = (
            [EnvironmentId("Initial")]
            if base_config.mode == RunMode.FROM_SCRATCH
            else []
        ) + [EnvironmentId(f"Stage-{number}") for number in range(1, 5)]
    else:
        stage_values = [EnvironmentId(value) for value in stages]
    items: list[WorkItem] = []
    base_options = dict(base_config.provider_options)
    device_pool = tuple(base_options.pop("device_pool", ()))
    for task in tasks:
        if not task.benchmark:
            continue
        available = {
            environment.environment_id for environment in registry.environments(task)
        }
        for stage in stage_values:
            if stage not in available:
                continue
            if base_config.mode == RunMode.ADAPTATION and stage.value == "Initial":
                continue
            for run_index in range(1, runs + 1):
                options = dict(base_options)
                if device_pool:
                    options["device"] = device_pool[len(items) % len(device_pool)]
                config = replace(
                    base_config,
                    task=task.full_name,
                    source=EnvironmentId("Initial"),
                    target=stage,
                    run_index=run_index,
                    provider_options=options,
                )
                items.append(WorkItem(len(items), config))
    return tuple(items)


def run_single(
    config: RunConfig, *, registry: TaskRegistry | None = None
) -> EvaluationResult:
    registry = registry or get_registry()
    task = registry.resolve(config.task)
    identity = (
        f"{config.source}_to_{config.target}"
        if config.mode == RunMode.ADAPTATION
        else str(config.target)
    )
    path = result_path(config, task, environment_identity=identity)
    if config.resume:
        for candidate in result_path_candidates(
            config, task, environment_identity=identity
        ):
            if is_complete(candidate):
                return load_result(candidate)
    result = EvaluationEngine(config, registry=registry).run()
    save_result(path, result)
    return result


def run_work_items(
    items: tuple[WorkItem, ...],
    *,
    workers: int = 1,
    dry_run: bool = False,
) -> tuple[WorkOutcome, ...]:
    if dry_run:
        return tuple(WorkOutcome(item, None) for item in items)
    if workers <= 1:
        outcomes = [_run_item(item) for item in items]
    elif _uses_explicit_accelerators(items):
        outcomes = _run_accelerator_queues(items, workers)
    else:
        outcomes = []
        with ThreadPoolExecutor(
            max_workers=workers, thread_name_prefix="pace-bench"
        ) as executor:
            futures = [executor.submit(_run_item, item) for item in items]
            outcomes.extend(future.result() for future in as_completed(futures))
    return tuple(sorted(outcomes, key=lambda outcome: outcome.work_item.index))


def _accelerator_device(item: WorkItem) -> str | None:
    if item.config.provider not in {"local", "local-transformers", "transformers"}:
        return None
    device = str(item.config.provider_options.get("device", "auto")).lower()
    return None if device in {"auto", "cpu"} else device


def _uses_explicit_accelerators(items: tuple[WorkItem, ...]) -> bool:
    return bool(items) and all(_accelerator_device(item) is not None for item in items)


def _run_accelerator_queues(
    items: tuple[WorkItem, ...], workers: int
) -> list[WorkOutcome]:
    queues: dict[str, list[WorkItem]] = {}
    for item in items:
        device = _accelerator_device(item)
        if device is None:
            raise ValueError("Accelerator queue received a non-accelerator work item")
        queues.setdefault(device, []).append(item)
    outcomes: list[WorkOutcome] = []
    with ThreadPoolExecutor(
        max_workers=min(workers, len(queues)), thread_name_prefix="pace-bench-device"
    ) as executor:
        futures = [
            executor.submit(_run_queue, tuple(queue)) for queue in queues.values()
        ]
        for future in as_completed(futures):
            outcomes.extend(future.result())
    return outcomes


def _run_queue(items: tuple[WorkItem, ...]) -> list[WorkOutcome]:
    return [_run_item(item) for item in items]


def _run_item(item: WorkItem) -> WorkOutcome:
    try:
        return WorkOutcome(item, run_single(item.config))
    except Exception as exc:
        return WorkOutcome(item, None, f"{type(exc).__name__}: {exc}")


@dataclass(frozen=True)
class ReferenceCheck:
    task: str
    reference_environment: str
    execution_environment: str
    expected_success: bool
    actual_success: bool
    score: float
    error: str | None

    @property
    def passed(self) -> bool:
        return self.actual_success == self.expected_success


def validate_task_references(
    task: TaskSpec,
    *,
    registry: TaskRegistry | None = None,
    check_initial_failures: bool = True,
    max_steps: int | None = None,
) -> tuple[ReferenceCheck, ...]:
    registry = registry or get_registry()
    initial_code = get_reference_solution(task, EnvironmentId("Initial"))
    checks: list[ReferenceCheck] = []
    for environment in registry.environments(task):
        if environment.environment_id.value == "Initial":
            checks.append(
                _run_reference_check(
                    task,
                    environment,
                    "Initial",
                    initial_code,
                    True,
                    registry,
                    max_steps,
                )
            )
            continue
        if check_initial_failures:
            checks.append(
                _run_reference_check(
                    task,
                    environment,
                    "Initial",
                    initial_code,
                    False,
                    registry,
                    max_steps,
                )
            )
        checks.append(
            _run_reference_check(
                task,
                environment,
                str(environment.environment_id),
                get_reference_solution(task, environment.environment_id),
                True,
                registry,
                max_steps,
            )
        )
    return tuple(checks)


def _run_reference_check(
    task: TaskSpec,
    environment: EnvironmentSpec,
    reference_environment: str,
    code: str,
    expected: bool,
    registry: TaskRegistry,
    max_steps: int | None,
) -> ReferenceCheck:
    verifier = PhysicsVerifier(
        task,
        environment,
        max_steps=max_steps or max_steps_for_task(task),
        headless=True,
        save_gif=False,
        artifact_directory=None,
        registry=registry,
    )
    try:
        result = verifier.verify(code, 0)
    finally:
        verifier.close()
    return ReferenceCheck(
        task=task.full_name,
        reference_environment=reference_environment,
        execution_environment=str(environment.environment_id),
        expected_success=expected,
        actual_success=result.success,
        score=result.score,
        error=result.error,
    )
