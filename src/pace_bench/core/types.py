"""Typed, serializable records shared across benchmark components."""

from __future__ import annotations

import re
from collections.abc import Mapping
from dataclasses import dataclass, field, fields, is_dataclass
from enum import Enum
from pathlib import Path
from typing import Any

RESULT_SCHEMA_VERSION = "2.0"


class RunMode(str, Enum):
    """Evaluation protocols supported by the unified engine."""

    SINGLE = "single"
    FROM_SCRATCH = "from-scratch"
    ADAPTATION = "adaptation"
    REFERENCE = "reference"


@dataclass(frozen=True, order=True)
class TaskId:
    """Canonical benchmark task identifier such as ``S_01``."""

    value: str

    def __post_init__(self) -> None:
        normalized = self.value.upper()
        if not re.fullmatch(r"[SKDFCE]_0[1-6]", normalized):
            raise ValueError(f"Invalid benchmark task ID: {self.value!r}")
        object.__setattr__(self, "value", normalized)

    @property
    def category_number(self) -> int:
        return {"S": 1, "K": 2, "D": 3, "F": 4, "C": 5, "E": 6}[self.value[0]]

    def __str__(self) -> str:
        return self.value


@dataclass(frozen=True, order=True)
class EnvironmentId:
    """Canonical environment identifier: Initial or Stage-1 through Stage-4."""

    value: str

    def __post_init__(self) -> None:
        normalized = self.value.strip()
        if normalized.lower() in {"initial", "base"}:
            normalized = "Initial"
        match = re.fullmatch(r"stage[-_ ]?([1-4])", normalized, re.IGNORECASE)
        if match:
            normalized = f"Stage-{match.group(1)}"
        if normalized not in {"Initial", "Stage-1", "Stage-2", "Stage-3", "Stage-4"}:
            raise ValueError(f"Invalid environment ID: {self.value!r}")
        object.__setattr__(self, "value", normalized)

    @property
    def stage_number(self) -> int:
        return 0 if self.value == "Initial" else int(self.value[-1])

    def __str__(self) -> str:
        return self.value


@dataclass(frozen=True, order=True)
class EnvironmentPair:
    """Source/target identity for adaptation evaluation."""

    source: EnvironmentId
    target: EnvironmentId

    def __post_init__(self) -> None:
        if self.source == self.target:
            raise ValueError("Environment pair source and target must differ")

    @property
    def identity(self) -> str:
        return f"{self.source}_to_{self.target}"

    @classmethod
    def parse(cls, value: str) -> EnvironmentPair:
        try:
            source, target = value.split("_to_", 1)
        except ValueError as exc:
            raise ValueError(f"Invalid environment pair: {value!r}") from exc
        return cls(EnvironmentId(source), EnvironmentId(target))

    def __str__(self) -> str:
        return self.identity


@dataclass
class GenerationRequest:
    """A provider request captured for reproducibility."""

    prompt: str
    system_prompt: str | None = None
    seed: int = 0
    temperature: float = 0.7
    top_p: float = 0.95
    max_tokens: int = 65_536
    metadata: dict[str, Any] = field(default_factory=dict)


@dataclass
class GenerationResult:
    """Provider response before sandbox verification."""

    text: str
    code: str | None = None
    token_usage: dict[str, int] = field(default_factory=dict)
    model: str | None = None
    latency_seconds: float | None = None
    raw: dict[str, Any] = field(default_factory=dict)


@dataclass
class VerificationResult:
    """Serializable outcome of one Box2D sandbox execution."""

    success: bool
    score: float
    metrics: dict[str, Any] = field(default_factory=dict)
    feedback: str = ""
    error: str | None = None
    artifact_paths: list[str] = field(default_factory=list)
    duration_seconds: float | None = None


@dataclass
class AttemptRecord:
    """Auditable record for one verified candidate."""

    attempt: int
    code: str
    verification: VerificationResult
    request: GenerationRequest | None = None
    generation: GenerationResult | None = None
    timestamp: float | None = None
    phase: str = "revision"

    @property
    def score(self) -> float:
        return self.verification.score

    @property
    def success(self) -> bool:
        return self.verification.success


@dataclass
class EvaluationResult:
    """Versioned result for one task/environment or adaptation pair."""

    task_id: str
    task_path: str
    mode: str
    provider: str
    model: str
    strategy: str
    attempts: list[AttemptRecord]
    source_environment: str | None = None
    target_environment: str | None = None
    environment_pair: str | None = None
    success: bool = False
    best_score: float = 0.0
    best_attempt: int | None = None
    stop_reason: str = "budget_exhausted"
    started_at: float | None = None
    finished_at: float | None = None
    total_time_seconds: float = 0.0
    config: dict[str, Any] = field(default_factory=dict)
    metadata: dict[str, Any] = field(default_factory=dict)
    schema_version: str = RESULT_SCHEMA_VERSION

    def to_dict(self) -> dict[str, Any]:
        return to_jsonable(self)


def to_jsonable(value: Any) -> Any:
    """Convert nested records to JSON-compatible values."""

    if isinstance(value, TaskId | EnvironmentId):
        return value.value
    if isinstance(value, EnvironmentPair):
        return value.identity
    if is_dataclass(value):
        return {
            item.name: to_jsonable(getattr(value, item.name)) for item in fields(value)
        }
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, Mapping):
        return {str(key): to_jsonable(item) for key, item in value.items()}
    if isinstance(value, list | tuple):
        return [to_jsonable(item) for item in value]
    if value is None or isinstance(value, str | int | float | bool):
        return value
    return str(value)
