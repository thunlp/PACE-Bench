"""Minimal examples of user-owned model and method integrations."""

from __future__ import annotations

from collections.abc import Sequence

from pace_bench.evaluation.prompts import PromptBuilder
from pace_bench.evaluation.config import StrategyContext
from pace_bench.core.types import (
    AttemptRecord,
    EvaluationResult,
    GenerationRequest,
    GenerationResult,
)


class CustomModel:
    """Replace ``generate`` with a call to any model runtime or API."""

    def __init__(self, *, model: str, **_: object) -> None:
        self.model = model

    def generate(self, request: GenerationRequest) -> GenerationResult:
        code = """def build_agent(sandbox):
    beam = sandbox.add_beam(5.0, 5.0, 2.0, 0.2, density=1.0)
    return beam

def agent_action(sandbox, agent_body, step_count):
    pass
"""
        return GenerationResult(text=code, code=code, model=self.model)

    def close(self) -> None:
        return None


class CustomMethod:
    """Example method that revises only the immediately previous candidate."""

    name = "custom-example"

    def __init__(self) -> None:
        self.context: StrategyContext | None = None
        self.prompts = PromptBuilder()

    def initialize(self, context: StrategyContext) -> None:
        self.context = context

    def build_initial_request(self) -> GenerationRequest:
        assert self.context is not None
        return GenerationRequest(self.prompts.initial(self.context.task_context))

    def build_revision_request(
        self, history: Sequence[AttemptRecord]
    ) -> GenerationRequest:
        assert self.context is not None
        previous = history[-1]
        return GenerationRequest(
            self.prompts.revision(
                self.context.task_context,
                best_code=previous.code,
                best_feedback=previous.verification.feedback,
                previous_code=previous.code,
                previous_feedback=previous.verification.feedback,
                best_attempt=previous.attempt,
                previous_attempt=previous.attempt,
            )
        )

    def observe(self, attempt: AttemptRecord) -> None:
        return None

    def finalize(self, result: EvaluationResult) -> None:
        return None
