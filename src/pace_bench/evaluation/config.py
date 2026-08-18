"""Validated configuration for every evaluation protocol."""

from __future__ import annotations

import time
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Protocol, runtime_checkable

from pace_bench.core.errors import ConfigurationError
from pace_bench.core.types import (
    AttemptRecord,
    EnvironmentId,
    EvaluationResult,
    GenerationRequest,
    GenerationResult,
    RunMode,
    to_jsonable,
)


@dataclass(frozen=True)
class RunConfig:
    """Model-independent configuration consumed by the evaluation engine."""

    task: str
    mode: RunMode = RunMode.ADAPTATION
    source: EnvironmentId = field(default_factory=lambda: EnvironmentId("Initial"))
    target: EnvironmentId = field(default_factory=lambda: EnvironmentId("Stage-1"))
    provider: str = "mock"
    model: str = "mock"
    strategy: str = "vanilla"
    attempts: int = 20
    max_steps: int | None = None
    generation_retries: int = 2
    seed: int = 0
    temperature: float = 0.7
    top_p: float = 0.95
    max_tokens: int = 65_536
    headless: bool = True
    save_gif: bool = False
    output: Path | None = None
    resume: bool = True
    timeout_seconds: float | None = None
    run_index: int = 1
    provider_options: dict[str, Any] = field(default_factory=dict)
    method_options: dict[str, Any] = field(default_factory=dict)
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.attempts < 1:
            raise ConfigurationError("attempts must be at least 1")
        if self.generation_retries < 0:
            raise ConfigurationError("generation_retries cannot be negative")
        if self.temperature < 0:
            raise ConfigurationError("temperature cannot be negative")
        if not 0 < self.top_p <= 1:
            raise ConfigurationError("top_p must be in the interval (0, 1]")
        if self.max_tokens < 1:
            raise ConfigurationError("max_tokens must be positive")
        if self.max_steps is not None and self.max_steps < 1:
            raise ConfigurationError("max_steps must be positive")
        if self.timeout_seconds is not None and self.timeout_seconds <= 0:
            raise ConfigurationError("timeout_seconds must be positive")
        if self.mode == RunMode.ADAPTATION and self.source.value != "Initial":
            raise ConfigurationError(
                "Canonical adaptation runs must use Initial as the source"
            )
        if self.mode == RunMode.ADAPTATION and self.target.value == "Initial":
            raise ConfigurationError(
                "Adaptation target must be Stage-1 through Stage-4"
            )
        if self.run_index < 1:
            raise ConfigurationError("run_index must be positive")

    def to_dict(self) -> dict[str, Any]:
        return to_jsonable(self)


@dataclass(frozen=True)
class StrategyContext:
    config: RunConfig
    task_context: dict[str, Any]
    reference_code: str | None = None
    reference_feedback: str | None = None


@runtime_checkable
class ModelProvider(Protocol):
    name: str
    model: str

    def generate(self, request: GenerationRequest) -> GenerationResult: ...

    def close(self) -> None: ...


@dataclass(frozen=True)
class CandidateSubmission:
    """One model request that a V2 strategy submits for sandbox evaluation."""

    request: GenerationRequest
    generation: GenerationResult | None = None
    label: str | None = None
    metadata: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class StrategyStep:
    """A strategy decision containing one or more candidate submissions."""

    submissions: tuple[CandidateSubmission, ...]
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.submissions:
            raise ConfigurationError(
                "A strategy step must submit at least one candidate"
            )

    @classmethod
    def one(
        cls,
        request: GenerationRequest,
        *,
        generation: GenerationResult | None = None,
        label: str | None = None,
        metadata: dict[str, Any] | None = None,
    ) -> StrategyStep:
        """Build a step containing one candidate request."""

        return cls(
            submissions=(
                CandidateSubmission(
                    request=request, generation=generation, label=label
                ),
            ),
            metadata=dict(metadata or {}),
        )

    @classmethod
    def batch(
        cls,
        requests: Sequence[GenerationRequest | CandidateSubmission],
        *,
        metadata: dict[str, Any] | None = None,
    ) -> StrategyStep:
        """Build a step from requests or explicitly labelled submissions."""

        submissions = tuple(
            item if isinstance(item, CandidateSubmission) else CandidateSubmission(item)
            for item in requests
        )
        return cls(submissions=submissions, metadata=dict(metadata or {}))


class StrategyRuntime:
    """Controlled provider access and usage accounting for V2 strategies.

    Candidate calls are made by the evaluation engine. Strategies may use
    :meth:`generate_auxiliary` for algorithm-specific LLM calls; those calls do
    not consume sandbox attempts, but their latency and token usage are audited.
    Prompts and raw provider responses are deliberately excluded from snapshots.
    """

    def __init__(self, provider: ModelProvider) -> None:
        self._provider = provider
        self._candidate_calls: list[dict[str, Any]] = []
        self._auxiliary_calls: list[dict[str, Any]] = []
        self._steps: list[dict[str, Any]] = []

    @property
    def provider(self) -> ModelProvider:
        """Return the configured backend for methods that adapt model weights.

        Most strategies should use the audited generation helpers below. A
        parameter-training strategy may inspect a compatible local provider,
        but it must not use this property to bypass generation accounting.
        """

        return self._provider

    def generate_candidate(
        self,
        request: GenerationRequest,
        *,
        attempt: int,
        retry: int,
        label: str | None = None,
    ) -> GenerationResult:
        """Generate one engine-controlled candidate and record its usage."""

        return self._generate(
            request,
            kind="candidate",
            details={
                "attempt": attempt,
                "retry": retry,
                "label": label,
                "strategy_owned": False,
            },
        )

    def generate_prepared_candidate(
        self,
        request: GenerationRequest,
        *,
        label: str | None = None,
    ) -> GenerationResult:
        """Generate a strategy-prepared candidate for later engine submission.

        This is useful when a method's final inner-loop call is itself the code
        candidate. The returned result should be attached to
        :class:`CandidateSubmission`; the engine then validates it without
        issuing a duplicate provider call.
        """

        return self._generate(
            request,
            kind="candidate",
            details={
                "attempt": None,
                "retry": 0,
                "label": label,
                "strategy_owned": True,
            },
        )

    def generate_auxiliary(
        self,
        request: GenerationRequest,
        *,
        purpose: str,
    ) -> GenerationResult:
        """Generate a non-candidate response without consuming a sandbox attempt."""

        if not purpose.strip():
            raise ConfigurationError(
                "Auxiliary generation requires a non-empty purpose"
            )
        return self._generate(
            request,
            kind="auxiliary",
            details={"purpose": purpose.strip()},
        )

    def record_step(
        self,
        step: StrategyStep,
        *,
        remaining_attempts: int,
        accepted_submissions: int,
    ) -> None:
        """Record a strategy decision without retaining prompts or code."""

        self._steps.append(
            {
                "requested_submissions": len(step.submissions),
                "accepted_submissions": accepted_submissions,
                "remaining_attempts": remaining_attempts,
                "labels": [item.label for item in step.submissions],
                "submission_metadata": [
                    to_jsonable(item.metadata) for item in step.submissions
                ],
                "metadata": to_jsonable(step.metadata),
            }
        )

    @property
    def auxiliary_usage(self) -> dict[str, Any]:
        """Return aggregate auxiliary-call usage for result metadata."""

        return self._usage_summary(self._auxiliary_calls)

    def snapshot(self) -> dict[str, Any]:
        """Return a JSON-safe audit record of all strategy-owned generation."""

        return {
            "candidate_usage": self._usage_summary(self._candidate_calls),
            "auxiliary_usage": self.auxiliary_usage,
            "candidate_calls": to_jsonable(self._candidate_calls),
            "auxiliary_calls": to_jsonable(self._auxiliary_calls),
            "steps": to_jsonable(self._steps),
        }

    def _generate(
        self,
        request: GenerationRequest,
        *,
        kind: str,
        details: dict[str, Any],
    ) -> GenerationResult:
        calls = self._candidate_calls if kind == "candidate" else self._auxiliary_calls
        started = time.perf_counter()
        record = {
            **details,
            "seed": request.seed,
            "temperature": request.temperature,
            "top_p": request.top_p,
            "max_tokens": request.max_tokens,
            "request_metadata": to_jsonable(request.metadata),
        }
        try:
            result = self._provider.generate(request)
        except Exception as exc:
            record.update(
                {
                    "success": False,
                    "latency_seconds": time.perf_counter() - started,
                    "error_type": type(exc).__name__,
                }
            )
            calls.append(record)
            raise
        record.update(
            {
                "success": True,
                "latency_seconds": result.latency_seconds
                if result.latency_seconds is not None
                else time.perf_counter() - started,
                "model": result.model,
                "token_usage": to_jsonable(result.token_usage),
            }
        )
        calls.append(record)
        return result

    @staticmethod
    def _usage_summary(calls: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
        token_usage: dict[str, int] = {}
        for call in calls:
            for key, value in dict(call.get("token_usage") or {}).items():
                token_usage[str(key)] = token_usage.get(str(key), 0) + int(value or 0)
        return {
            "calls": len(calls),
            "successful_calls": sum(bool(call.get("success")) for call in calls),
            "failed_calls": sum(not bool(call.get("success")) for call in calls),
            "latency_seconds": sum(
                float(call.get("latency_seconds", 0.0) or 0.0) for call in calls
            ),
            "token_usage": token_usage,
        }


@runtime_checkable
class EvaluationStrategy(Protocol):
    name: str

    def initialize(self, context: StrategyContext) -> None: ...

    def build_initial_request(self) -> GenerationRequest: ...

    def build_revision_request(
        self, history: Sequence[AttemptRecord]
    ) -> GenerationRequest: ...

    def observe(self, attempt: AttemptRecord) -> None: ...

    def finalize(self, result: EvaluationResult) -> None: ...


@runtime_checkable
class EvaluationStrategyV2(Protocol):
    """Batch-capable additive strategy API with audited auxiliary generation."""

    name: str

    def initialize(
        self, context: StrategyContext, runtime: StrategyRuntime
    ) -> None: ...

    def build_step(
        self, history: Sequence[AttemptRecord], remaining_attempts: int
    ) -> StrategyStep: ...

    def observe(self, attempts: Sequence[AttemptRecord]) -> None: ...

    def snapshot(self) -> Mapping[str, Any]: ...

    def finalize(self, result: EvaluationResult) -> None: ...
