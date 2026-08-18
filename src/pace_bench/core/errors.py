"""Project-specific exceptions with actionable failure categories."""


class PaceBenchError(Exception):
    """Base class for expected PACE-Bench failures."""


class ConfigurationError(PaceBenchError):
    """Raised when a run configuration is inconsistent or incomplete."""


class TaskNotFoundError(PaceBenchError):
    """Raised when a task selector cannot be resolved."""


class TaskContractError(PaceBenchError):
    """Raised when a task does not implement the required module contract."""


class ProviderError(PaceBenchError):
    """Raised when a model provider cannot produce a response."""


class InvalidSolverOutputError(ProviderError):
    """Raised when all retries produce structurally unusable solver output."""


class VerificationError(PaceBenchError):
    """Raised when the verification infrastructure itself cannot run."""


class ResultSchemaError(PaceBenchError):
    """Raised when a result cannot be decoded or migrated."""


class AgentRuntimeError(PaceBenchError):
    """Raised when an isolated coding-agent evaluation cannot be orchestrated."""
