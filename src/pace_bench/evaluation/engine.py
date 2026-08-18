"""One reusable loop for from-scratch, adaptation, and reference protocols."""

from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import replace
from typing import Protocol

from pace_bench.core.errors import ConfigurationError, ProviderError
from pace_bench.core.types import (
    AttemptRecord,
    EvaluationResult,
    GenerationRequest,
    GenerationResult,
    RunMode,
    VerificationResult,
    to_jsonable,
)
from pace_bench.evaluation.config import (
    CandidateSubmission,
    EvaluationStrategy,
    EvaluationStrategyV2,
    ModelProvider,
    StrategyRuntime,
    StrategyStep,
    StrategyContext,
)
from pace_bench.evaluation.config import RunConfig
from pace_bench.evaluation.method import VanillaMethod, load_method
from pace_bench.evaluation.prompts import PromptBuilder
from pace_bench.evaluation.providers import load_provider
from pace_bench.evaluation.results import artifact_directory
from pace_bench.evaluation.verification.safety import (
    extract_code,
    validate_solver_output,
)
from pace_bench.evaluation.verification.verifier import PhysicsVerifier
from pace_bench.tasks.registry import (
    EnvironmentSpec,
    TaskRegistry,
    TaskSpec,
    get_reference_solution,
    get_registry,
    max_steps_for_task,
)


class CandidateVerifier(Protocol):
    def verify(self, code: str, attempt: int) -> VerificationResult: ...

    def close(self) -> None: ...


VerifierFactory = Callable[[TaskSpec, EnvironmentSpec, RunConfig], CandidateVerifier]


class EvaluationEngine:
    """Execute exactly one task/environment protocol with explicit dependencies."""

    def __init__(
        self,
        config: RunConfig,
        *,
        registry: TaskRegistry | None = None,
        provider: ModelProvider | None = None,
        strategy: EvaluationStrategy | EvaluationStrategyV2 | None = None,
        verifier_factory: VerifierFactory | None = None,
    ) -> None:
        self.config = config
        self.registry = registry or get_registry()
        self.provider = provider or load_provider(
            config.provider, model=config.model, options=config.provider_options
        )
        if strategy is not None:
            self.strategy = strategy
        elif config.strategy in {"iterative", "vanilla"}:
            self.strategy = VanillaMethod(PromptBuilder(self.registry))
        else:
            self.strategy = load_method(config.strategy, options=config.method_options)
        self.strategy_runtime = StrategyRuntime(self.provider)
        self.verifier_factory = verifier_factory or self._physics_verifier_factory

    def run(self) -> EvaluationResult:
        started_wall = time.time()
        started = time.perf_counter()
        task = self.registry.resolve(self.config.task)
        environments = {
            str(item.environment_id): item for item in self.registry.environments(task)
        }
        target = environments[str(self.config.target)]
        task_context = PromptBuilder(self.registry).load_task_context(
            task,
            target,
            include_source_comparison=self.config.mode != RunMode.FROM_SCRATCH,
        )
        verifier = self.verifier_factory(task, target, self.config)
        history: list[AttemptRecord] = []
        stop_reason = "budget_exhausted"
        invalid_error: str | None = None
        try:
            reference_code = None
            reference_feedback = None
            if self.config.mode in {
                RunMode.ADAPTATION,
                RunMode.REFERENCE,
                RunMode.SINGLE,
            }:
                reference_environment = (
                    self.config.source
                    if self.config.mode == RunMode.ADAPTATION
                    else self.config.target
                )
                reference_code = get_reference_solution(task, reference_environment)
            if self.config.mode == RunMode.ADAPTATION:
                reference_attempt = self._verify_attempt(
                    verifier,
                    attempt=0,
                    code=reference_code or "",
                    phase="reference",
                )
                history.append(reference_attempt)
                reference_feedback = reference_attempt.verification.feedback
                if reference_attempt.success:
                    stop_reason = "reference_passes_target"
            strategy_context = StrategyContext(
                config=self.config,
                task_context=task_context,
                reference_code=reference_code,
                reference_feedback=reference_feedback,
            )
            is_v2 = isinstance(self.strategy, EvaluationStrategyV2)
            if is_v2:
                self.strategy.initialize(strategy_context, self.strategy_runtime)
            else:
                self.strategy.initialize(strategy_context)

            verified_submissions = 0
            next_attempt = 1
            while (
                stop_reason != "reference_passes_target"
                and verified_submissions < self.config.attempts
                and self.config.mode not in {RunMode.REFERENCE, RunMode.SINGLE}
            ):
                remaining_attempts = self.config.attempts - verified_submissions
                step = self._build_step(
                    history,
                    remaining_attempts=remaining_attempts,
                    is_v2=is_v2,
                )
                accepted = step.submissions[:remaining_attempts]
                self.strategy_runtime.record_step(
                    step,
                    remaining_attempts=remaining_attempts,
                    accepted_submissions=len(accepted),
                )
                observed: list[AttemptRecord] = []
                batch_invalid = False
                for submission in accepted:
                    generation, code, invalid_error = self._generate_valid_candidate(
                        submission, next_attempt
                    )
                    if generation is None or code is None:
                        stop_reason = "invalid_solver_output"
                        batch_invalid = True
                        break
                    attempt = self._verify_attempt(
                        verifier,
                        attempt=next_attempt,
                        code=code,
                        phase=(
                            "initial"
                            if self.config.mode == RunMode.FROM_SCRATCH
                            and verified_submissions == 0
                            else "revision"
                        ),
                        request=submission.request,
                        generation=generation,
                    )
                    history.append(attempt)
                    observed.append(attempt)
                    verified_submissions += 1
                    next_attempt += 1
                if observed:
                    if is_v2:
                        self.strategy.observe(tuple(observed))
                    else:
                        self.strategy.observe(observed[0])
                if batch_invalid:
                    break
                if any(attempt.success for attempt in observed):
                    stop_reason = "success"
                    break
            if self.config.mode in {RunMode.REFERENCE, RunMode.SINGLE} and not history:
                direct = self._verify_attempt(
                    verifier,
                    attempt=0,
                    code=reference_code or "",
                    phase="reference",
                )
                history.append(direct)
                stop_reason = "success" if direct.success else "reference_failed"
        finally:
            verifier.close()
            self.provider.close()

        best = max(history, key=lambda item: item.score) if history else None
        result = EvaluationResult(
            task_id=task.name,
            task_path=task.full_name,
            mode=self.config.mode.value,
            provider=self.config.provider,
            model=self.config.model,
            strategy=getattr(self.strategy, "name", self.config.strategy),
            attempts=history,
            source_environment=(
                str(self.config.source)
                if self.config.mode == RunMode.ADAPTATION
                else None
            ),
            target_environment=str(self.config.target),
            environment_pair=(
                f"{self.config.source}_to_{self.config.target}"
                if self.config.mode == RunMode.ADAPTATION
                else None
            ),
            success=any(attempt.success for attempt in history),
            best_score=best.score if best else 0.0,
            best_attempt=best.attempt if best else None,
            stop_reason=stop_reason,
            started_at=started_wall,
            finished_at=time.time(),
            total_time_seconds=time.perf_counter() - started,
            config=self.config.to_dict(),
            metadata={"invalid_solver_error": invalid_error} if invalid_error else {},
        )
        self.strategy.finalize(result)
        strategy_snapshot = self.strategy.snapshot() if is_v2 else {}
        result.metadata.update(
            {
                "strategy_snapshot": to_jsonable(strategy_snapshot),
                "strategy_runtime": self.strategy_runtime.snapshot(),
                "auxiliary_usage": self.strategy_runtime.auxiliary_usage,
            }
        )
        return result

    def _build_step(
        self,
        history: list[AttemptRecord],
        *,
        remaining_attempts: int,
        is_v2: bool,
    ) -> StrategyStep:
        if is_v2:
            step = self.strategy.build_step(history, remaining_attempts)
        else:
            request = (
                self.strategy.build_initial_request()
                if not history
                else self.strategy.build_revision_request(history)
            )
            step = StrategyStep.one(request)
        if not isinstance(step, StrategyStep):
            raise ConfigurationError(
                f"Strategy {getattr(self.strategy, 'name', '<unknown>')!r} "
                "build_step() must return StrategyStep"
            )
        return step

    def _generate_valid_candidate(
        self, submission: CandidateSubmission, attempt: int
    ) -> tuple[GenerationResult | None, str | None, str | None]:
        request = submission.request
        last_error = "provider produced no candidate"
        last_provider_error: ProviderError | None = None
        received_provider_response = submission.generation is not None
        first_retry = 0
        total_provider_calls = self.config.generation_retries + 1
        if submission.generation is not None:
            generation = submission.generation
            code = (
                generation.code
                if generation.code is not None
                else extract_code(generation.text)
            )
            valid, reason = validate_solver_output(generation.text, code)
            if valid:
                generation.code = code
                return generation, code, None
            last_error = reason
            # The strategy-owned generation was the initial call. Preserve the
            # configured policy by making only the remaining retry calls.
            first_retry = 1
            total_provider_calls = self.config.generation_retries + 1
        for retry in range(first_retry, total_provider_calls):
            retry_request = replace(
                request,
                # Preserve the legacy trial/iteration seed separation while
                # keeping an explicit user-controlled base seed.
                seed=(
                    self.config.seed
                    + self.config.run_index * 1000
                    + attempt
                    + retry * 100
                ),
                metadata={**request.metadata, "retry": retry},
            )
            try:
                generation = self.strategy_runtime.generate_candidate(
                    retry_request,
                    attempt=attempt,
                    retry=retry,
                    label=submission.label,
                )
            except ProviderError as exc:
                last_error = str(exc)
                last_provider_error = exc
                continue
            received_provider_response = True
            code = (
                generation.code
                if generation.code is not None
                else extract_code(generation.text)
            )
            valid, reason = validate_solver_output(generation.text, code)
            if valid:
                generation.code = code
                return generation, code, None
            last_error = reason
        if not received_provider_response and last_provider_error is not None:
            # A provider outage is an orchestration failure, not invalid solver
            # output. Propagate it so run_single() never persists a benchmark
            # result for a run in which the model produced no response.
            raise last_provider_error
        return None, None, last_error

    @staticmethod
    def _verify_attempt(
        verifier: CandidateVerifier,
        *,
        attempt: int,
        code: str,
        phase: str,
        request: GenerationRequest | None = None,
        generation: GenerationResult | None = None,
    ) -> AttemptRecord:
        return AttemptRecord(
            attempt=attempt,
            code=code,
            request=request,
            generation=generation,
            verification=verifier.verify(code, attempt),
            timestamp=time.time(),
            phase=phase,
        )

    def _physics_verifier_factory(
        self, task: TaskSpec, environment: EnvironmentSpec, config: RunConfig
    ) -> PhysicsVerifier:
        max_steps = config.max_steps or max_steps_for_task(task)
        gif_directory = None
        if config.save_gif:
            identity = (
                f"{config.source}_to_{config.target}"
                if config.mode == RunMode.ADAPTATION
                else str(config.target)
            )
            gif_directory = artifact_directory(
                config, task, environment_identity=identity
            )
        return PhysicsVerifier(
            task,
            environment,
            max_steps=max_steps,
            headless=config.headless,
            save_gif=config.save_gif,
            artifact_directory=gif_directory,
            registry=self.registry,
        )
