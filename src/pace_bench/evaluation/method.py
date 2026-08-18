"""Vanilla Previous-One + Best method and external-method loading."""

from __future__ import annotations

import importlib.util
import re
import sys
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from pace_bench.core.errors import ConfigurationError
from pace_bench.core.types import (
    AttemptRecord,
    EvaluationResult,
    GenerationRequest,
    RunMode,
)
from pace_bench.evaluation.config import (
    EvaluationStrategy,
    EvaluationStrategyV2,
    StrategyContext,
)
from pace_bench.evaluation.prompts import PromptBuilder
from pace_bench.evaluation.providers import load_object
from pace_bench.core.paths import repository_root


@dataclass(frozen=True)
class PreviousAndBest:
    previous: AttemptRecord
    best: AttemptRecord

    @property
    def same_attempt(self) -> bool:
        return self.previous.attempt == self.best.attempt


def select_previous_and_best(history: Sequence[AttemptRecord]) -> PreviousAndBest:
    """Select the newest attempt and earliest highest-scoring attempt."""

    if not history:
        raise ValueError("Cannot select from an empty attempt history")
    return PreviousAndBest(
        previous=history[-1], best=max(history, key=lambda item: item.score)
    )


class VanillaMethod:
    """The only bundled method: Previous-One + Best iterative refinement."""

    name = "vanilla"

    def __init__(self, prompt_builder: PromptBuilder | None = None) -> None:
        self.prompt_builder = prompt_builder or PromptBuilder()
        self.context: StrategyContext | None = None

    def initialize(self, context: StrategyContext) -> None:
        self.context = context

    def build_initial_request(self) -> GenerationRequest:
        context = self._require_context()
        return self._request(
            self.prompt_builder.initial(context.task_context), attempt=1
        )

    def build_revision_request(
        self, history: Sequence[AttemptRecord]
    ) -> GenerationRequest:
        context = self._require_context()
        selected = select_previous_and_best(history)
        if context.config.mode == RunMode.ADAPTATION:
            reference = history[0]
            prompt = self.prompt_builder.adaptation_revision(
                context.task_context,
                reference_code=context.reference_code or reference.code,
                reference_feedback=context.reference_feedback
                or reference.verification.feedback,
                best_code=selected.best.code,
                best_feedback=selected.best.verification.feedback,
                previous_code=selected.previous.code,
                previous_feedback=selected.previous.verification.feedback,
                best_attempt=selected.best.attempt,
                previous_attempt=selected.previous.attempt,
            )
        else:
            prompt = self.prompt_builder.revision(
                context.task_context,
                best_code=selected.best.code,
                best_feedback=selected.best.verification.feedback,
                previous_code=selected.previous.code,
                previous_feedback=selected.previous.verification.feedback,
                best_attempt=selected.best.attempt,
                previous_attempt=selected.previous.attempt,
            )
        return self._request(prompt, attempt=selected.previous.attempt + 1)

    def observe(self, attempt: AttemptRecord) -> None:
        """Vanilla keeps no hidden state outside serialized attempt history."""

    def finalize(self, result: EvaluationResult) -> None:
        """Vanilla has no auxiliary finalization step."""

    def _request(self, prompt: str, *, attempt: int) -> GenerationRequest:
        context = self._require_context()
        config = context.config
        return GenerationRequest(
            prompt=prompt,
            seed=config.seed + attempt,
            temperature=config.temperature,
            top_p=config.top_p,
            max_tokens=config.max_tokens,
            metadata={"attempt": attempt, "method": self.name},
        )

    def _require_context(self) -> StrategyContext:
        if self.context is None:
            raise RuntimeError("Method must be initialized before building requests")
        return self.context


def load_method(
    name: str, *, options: dict[str, Any] | None = None
) -> EvaluationStrategy | EvaluationStrategyV2:
    """Load vanilla, a repository method, or a dotted-import implementation."""

    if name in {"vanilla", "iterative"}:
        if options:
            raise ConfigurationError(
                "The built-in vanilla method has no method options"
            )
        return VanillaMethod()
    method_class = load_object(name) if ":" in name else _load_local_method_class(name)
    try:
        method = method_class(**dict(options or {}))
    except TypeError as exc:
        raise ConfigurationError(
            f"Cannot initialize method {name!r} with the supplied options: {exc}"
        ) from exc
    if callable(getattr(method, "build_step", None)):
        required = ("initialize", "build_step", "observe", "snapshot", "finalize")
    else:
        required = (
            "initialize",
            "build_initial_request",
            "build_revision_request",
            "observe",
            "finalize",
        )
    missing = [
        attribute
        for attribute in required
        if not callable(getattr(method, attribute, None))
    ]
    if missing:
        raise ConfigurationError(
            f"Method {name!r} is missing required callables: {', '.join(missing)}"
        )
    return method


def _load_local_method_class(name: str) -> Any:
    """Load ``Method`` from a checkout or working-directory method plug-in."""

    if not re.fullmatch(r"[A-Za-z0-9_-]+", name):
        raise ConfigurationError(
            f"Invalid local method name {name!r}; use a simple filename or dotted path"
        )
    candidates = [Path.cwd() / "methods" / f"{name}.py"]
    root = repository_root()
    if root is not None:
        checkout_candidate = root / "methods" / f"{name}.py"
        if checkout_candidate not in candidates:
            candidates.append(checkout_candidate)
    path = next((candidate for candidate in candidates if candidate.is_file()), None)
    if path is None:
        expected = " or ".join(str(candidate) for candidate in candidates)
        raise ConfigurationError(
            f"Unknown method {name!r}. Expected {expected} exporting Method, or use "
            "package.module:MethodClass."
        )
    module_name = f"_pace_bench_local_method_{name.replace('-', '_')}"
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise ConfigurationError(f"Cannot create an import specification for {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    try:
        spec.loader.exec_module(module)
        return getattr(module, "Method")
    except (ImportError, AttributeError) as exc:
        sys.modules.pop(module_name, None)
        raise ConfigurationError(
            f"Cannot load local method {path}; it must export Method ({exc})"
        ) from exc
