"""Black-box sessions for tool-using coding-agent evaluation.

The trusted host owns task discovery and Box2D verification.  An agent receives
only a generated workspace and communicates with this module through a narrow,
token-authenticated HTTP API.  Container isolation is implemented separately in
``agents.container`` so benchmark semantics remain independent of Docker.
"""

from __future__ import annotations

import hmac
import json
import secrets
import stat
import threading
import time
from dataclasses import dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

from pace_bench.evaluation.config import RunConfig, StrategyContext
from pace_bench.evaluation.method import VanillaMethod
from pace_bench.evaluation.prompts import PromptBuilder
from pace_bench.evaluation.results import (
    artifact_directory,
    result_path,
    result_path_candidates,
    save_result,
)
from pace_bench.evaluation.verification.safety import validate_solver_output
from pace_bench.evaluation.verification.verifier import PhysicsVerifier
from pace_bench.tasks.registry import (
    TaskRegistry,
    get_reference_solution,
    get_registry,
    max_steps_for_task,
)
from pace_bench.core.types import (
    AttemptRecord,
    EnvironmentId,
    EvaluationResult,
    RunMode,
)

MAX_SUBMISSION_BYTES = 1_000_000


@dataclass(frozen=True)
class AgentSessionConfig:
    """Configuration for one Initial-to-mutated black-box agent session."""

    task: str
    target: EnvironmentId
    attempts: int = 20
    max_steps: int | None = None
    output: Path = Path("results")
    agent: str = "custom"
    model: str = "unspecified"
    headless: bool = True
    save_gif: bool = False
    seed: int = 0
    run_index: int = 1
    prompt_file: Path | None = None
    overwrite: bool = False

    def __post_init__(self) -> None:
        if self.target.value == "Initial":
            raise ValueError("Agent adaptation target must be Stage-1 through Stage-4")
        if self.attempts < 1:
            raise ValueError("Agent submission budget must be at least 1")
        if self.max_steps is not None and self.max_steps < 1:
            raise ValueError("max_steps must be positive")


class AgentSession:
    """Own attempt accounting, verification, persistence, and public feedback."""

    def __init__(
        self,
        config: AgentSessionConfig,
        *,
        registry: TaskRegistry | None = None,
    ) -> None:
        self.config = config
        self.registry = registry or get_registry()
        self.task = self.registry.resolve(config.task)
        environments = {
            str(item.environment_id): item
            for item in self.registry.environments(self.task)
        }
        self.environment = environments[str(config.target)]
        self.reference_code = get_reference_solution(
            self.task, EnvironmentId("Initial")
        )
        self.task_context = PromptBuilder(self.registry).load_task_context(
            self.task, self.environment
        )
        self.run_config = RunConfig(
            task=self.task.full_name,
            mode=RunMode.ADAPTATION,
            source=EnvironmentId("Initial"),
            target=config.target,
            provider="coding-agent",
            model=config.model,
            strategy=f"agent-{config.agent}",
            attempts=config.attempts,
            max_steps=config.max_steps,
            seed=config.seed,
            headless=config.headless,
            save_gif=config.save_gif,
            output=config.output,
            resume=False,
            run_index=config.run_index,
            metadata={"agent_runtime": "container-black-box"},
        )
        self.environment_pair = f"Initial_to_{config.target}"
        self.result_file = result_path(
            self.run_config,
            self.task,
            environment_identity=self.environment_pair,
        )
        existing_result = next(
            (
                path
                for path in result_path_candidates(
                    self.run_config,
                    self.task,
                    environment_identity=self.environment_pair,
                )
                if path.exists()
            ),
            None,
        )
        if existing_result is not None and not config.overwrite:
            raise ValueError(
                f"Agent result already exists: {existing_result}; use a different "
                "--run-index or pass --overwrite"
            )
        gif_directory = artifact_directory(
            self.run_config,
            self.task,
            environment_identity=self.environment_pair,
        )
        self.verifier = PhysicsVerifier(
            self.task,
            self.environment,
            max_steps=config.max_steps or max_steps_for_task(self.task),
            headless=config.headless,
            save_gif=config.save_gif,
            artifact_directory=gif_directory,
            registry=self.registry,
        )
        self.token = secrets.token_urlsafe(32)
        self.attempts: list[AttemptRecord] = []
        self.started_at = time.time()
        self.stop_reason = "running"
        self.agent_exit_code: int | None = None
        self.runtime_metadata: dict[str, Any] = {}
        self._lock = threading.Lock()
        self._closed = False

        reference = AttemptRecord(
            attempt=0,
            code=self.reference_code,
            verification=self.verifier.verify(self.reference_code, 0),
            timestamp=time.time(),
            phase="reference",
        )
        self.attempts.append(reference)
        if reference.success:
            self.stop_reason = "reference_passes_target"
        self._persist()

    @property
    def submitted(self) -> int:
        return max(0, len(self.attempts) - 1)

    @property
    def remaining(self) -> int:
        return max(0, self.config.attempts - self.submitted)

    @property
    def complete(self) -> bool:
        return self.stop_reason != "running"

    def public_status(self) -> dict[str, Any]:
        """Return session state without task internals or full metric payloads."""

        latest = self.attempts[-1]
        best = max(self.attempts, key=lambda item: item.score)
        return {
            "task": self.task.name,
            "environment_pair": self.environment_pair,
            "attempt": latest.attempt,
            "submitted": self.submitted,
            "remaining": self.remaining,
            "success": any(item.success for item in self.attempts),
            "latest_score": latest.score,
            "best_score": best.score,
            "best_attempt": best.attempt,
            "stop_reason": self.stop_reason,
        }

    def submit(self, code: str) -> tuple[HTTPStatus, dict[str, Any]]:
        """Validate and verify one candidate; unusable output consumes no budget."""

        with self._lock:
            if self.complete:
                return HTTPStatus.CONFLICT, {
                    **self.public_status(),
                    "consumed": False,
                    "error": f"session already stopped: {self.stop_reason}",
                }
            valid, reason = validate_solver_output(code, code)
            if not valid:
                return HTTPStatus.UNPROCESSABLE_ENTITY, {
                    **self.public_status(),
                    "consumed": False,
                    "error": reason,
                }
            if self.remaining <= 0:
                self.stop_reason = "budget_exhausted"
                self._persist()
                return HTTPStatus.CONFLICT, {
                    **self.public_status(),
                    "consumed": False,
                    "error": "submission budget exhausted",
                }

            attempt_number = self.submitted + 1
            verification = self.verifier.verify(code, attempt_number)
            attempt = AttemptRecord(
                attempt=attempt_number,
                code=code,
                verification=verification,
                timestamp=time.time(),
                phase="agent",
            )
            self.attempts.append(attempt)
            if verification.success:
                self.stop_reason = "success"
            elif self.remaining == 0:
                self.stop_reason = "budget_exhausted"
            self._persist()
            return HTTPStatus.OK, {
                "task": self.task.name,
                "environment_pair": self.environment_pair,
                "attempt": attempt_number,
                "consumed": True,
                "success": verification.success,
                "score": verification.score,
                "feedback": verification.feedback,
                "error": verification.error,
                "remaining": self.remaining,
                "stop_reason": self.stop_reason,
            }

    def finalize(
        self, *, agent_exit_code: int | None, reason: str | None = None
    ) -> None:
        """Record orchestration termination without changing verified attempts."""

        with self._lock:
            self.agent_exit_code = agent_exit_code
            if self.stop_reason == "running":
                self.stop_reason = reason or (
                    "agent_exited" if agent_exit_code == 0 else "agent_error"
                )
            self._persist()

    def record_runtime_metadata(self, **values: Any) -> None:
        """Attach serializable container/artifact facts to the persisted result."""

        with self._lock:
            self.runtime_metadata.update(values)
            self._persist()

    def create_workspace(self, workspace: Path) -> Path:
        """Create the only files mounted into the untrusted agent container."""

        workspace.mkdir(parents=True, exist_ok=True)
        (workspace / "TASK.md").write_text(self._task_markdown(), encoding="utf-8")
        (workspace / "initial_solution.py").write_text(
            self.reference_code.rstrip() + "\n", encoding="utf-8"
        )
        (workspace / "solution.py").write_text(
            self.reference_code.rstrip() + "\n", encoding="utf-8"
        )
        prompt = (
            self.config.prompt_file.read_text(encoding="utf-8")
            if self.config.prompt_file
            else self._default_agent_prompt()
        )
        (workspace / "AGENT_PROMPT.md").write_text(
            prompt.rstrip() + "\n", encoding="utf-8"
        )
        submitter = workspace / "pace-submit"
        submitter.write_text(self._submission_client(), encoding="utf-8")
        submitter.chmod(submitter.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP)
        return workspace

    def close(self) -> None:
        if self._closed:
            return
        self.verifier.close()
        self._closed = True

    def _task_markdown(self) -> str:
        context = self.task_context
        reference_feedback = self.attempts[0].verification.feedback
        return f"""# PACE-Bench Agent Task

Task: `{self.task.name}`
Environment pair: `{self.environment_pair}`
Valid revision budget: **{self.config.attempts}**

## Task description

{context.get("task_description", "")}

## Success criteria

{context.get("success_criteria", "")}

## Allowed primitives

{context.get("primitives_api", "")}

## Mutation information

{context.get("prompt_trailer", "No additional mutation information is exposed.")}

## Attempt 0: Initial reference in the target environment

{reference_feedback}
"""

    def _default_agent_prompt(self) -> str:
        reference = self.attempts[0]
        vanilla = VanillaMethod(PromptBuilder(self.registry))
        vanilla.initialize(
            StrategyContext(
                config=self.run_config,
                task_context=self.task_context,
                reference_code=self.reference_code,
                reference_feedback=reference.verification.feedback,
            )
        )
        initial_request = vanilla.build_revision_request([reference])
        return f"""{initial_request.prompt.rstrip()}

# Agent-Mode Execution

You are being evaluated as a tool-using coding agent. The benchmark content above is the same initial request given to the vanilla model baseline. Unlike that baseline, you manage all later reasoning, context, history, and revisions yourself.

The same task context is also available in `TASK.md`, and the Initial code is available in `initial_solution.py` and `solution.py`. Infer the changed physics, edit `solution.py`, and submit candidates with:

```bash
./pace-submit solution.py
```

After every submission, read the command output plus `last_feedback.md`. Continue revising and submitting until the result reports success or the valid-submission budget is exhausted. `./pace-submit --status` shows the remaining budget without consuming it. Structurally unusable output is rejected without consuming a submission; valid Python that fails during construction or simulation is a normal consumed attempt.

You may use your own tools, notes, context, and history-management strategy after this initial request. You are not constrained to the vanilla Previous-One + Best policy for later iterations. Do not try to locate, download, reconstruct, or inspect PACE-Bench task/environment/reference source code. Treat the evaluator as a black box and use only the exposed task description, allowed primitives, and returned feedback.
"""

    def _submission_client(self) -> str:
        endpoint = "http://gateway:8080/evaluator/v1"
        token = json.dumps(self.token)
        return f'''#!/usr/bin/env python3
"""Submit one PACE-Bench candidate to the current black-box session."""
import json
import pathlib
import sys
import urllib.error
import urllib.request

BASE_URL = {endpoint!r}
TOKEN = {token}


def request(method, path, payload=None):
    body = None if payload is None else json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(
        BASE_URL + path,
        data=body,
        method=method,
        headers={{"Authorization": "Bearer " + TOKEN, "Content-Type": "application/json"}},
    )
    try:
        with urllib.request.urlopen(req, timeout=600) as response:
            return response.status, json.load(response)
    except urllib.error.HTTPError as exc:
        try:
            data = json.load(exc)
        except Exception:
            data = {{"error": exc.reason}}
        return exc.code, data


def main():
    if len(sys.argv) == 2 and sys.argv[1] == "--status":
        status, data = request("GET", "/status")
    elif len(sys.argv) == 2:
        path = pathlib.Path(sys.argv[1])
        if not path.is_file():
            print(json.dumps({{"error": f"candidate not found: {{path}}"}}, indent=2))
            return 2
        status, data = request("POST", "/submit", {{"code": path.read_text(encoding="utf-8")}})
    else:
        print("usage: ./pace-submit solution.py | ./pace-submit --status", file=sys.stderr)
        return 2
    pathlib.Path("last_result.json").write_text(json.dumps(data, indent=2) + "\\n", encoding="utf-8")
    if data.get("feedback"):
        pathlib.Path("last_feedback.md").write_text(data["feedback"].rstrip() + "\\n", encoding="utf-8")
    print(json.dumps(data, indent=2))
    return 0 if status < 400 else 1


if __name__ == "__main__":
    raise SystemExit(main())
'''

    def _result(self) -> EvaluationResult:
        best = max(self.attempts, key=lambda item: item.score)
        return EvaluationResult(
            task_id=self.task.name,
            task_path=self.task.full_name,
            mode="agent-adaptation",
            provider="coding-agent",
            model=self.config.model,
            strategy=f"agent-{self.config.agent}",
            attempts=list(self.attempts),
            source_environment="Initial",
            target_environment=str(self.config.target),
            environment_pair=self.environment_pair,
            success=any(item.success for item in self.attempts),
            best_score=best.score,
            best_attempt=best.attempt,
            stop_reason=self.stop_reason,
            started_at=self.started_at,
            finished_at=None if self.stop_reason == "running" else time.time(),
            total_time_seconds=time.time() - self.started_at,
            config=self.run_config.to_dict(),
            metadata={
                "agent": self.config.agent,
                "agent_exit_code": self.agent_exit_code,
                "submission_budget": self.config.attempts,
                "valid_submissions": self.submitted,
                "black_box": True,
                **self.runtime_metadata,
            },
        )

    def _persist(self) -> None:
        save_result(self.result_file, self._result())


class AgentSessionServer:
    """A loopback evaluator API intended to be reached through the gateway."""

    def __init__(self, session: AgentSession) -> None:
        self.session = session
        handler = self._handler_class()
        self.server = ThreadingHTTPServer(("0.0.0.0", 0), handler)
        self.server.daemon_threads = True
        self.thread: threading.Thread | None = None

    @property
    def port(self) -> int:
        return int(self.server.server_address[1])

    def start(self) -> None:
        self.thread = threading.Thread(
            target=self.server.serve_forever,
            name="pace-bench-agent-evaluator",
            daemon=True,
        )
        self.thread.start()

    def close(self) -> None:
        self.server.shutdown()
        self.server.server_close()
        if self.thread:
            self.thread.join(timeout=5)

    def _handler_class(self) -> type[BaseHTTPRequestHandler]:
        session = self.session

        class Handler(BaseHTTPRequestHandler):
            server_version = "PACEBenchAgent/1.0"

            def do_GET(self) -> None:  # noqa: N802
                if not self._authorized():
                    return
                if self.path != "/v1/status":
                    self._json(HTTPStatus.NOT_FOUND, {"error": "not found"})
                    return
                self._json(HTTPStatus.OK, session.public_status())

            def do_POST(self) -> None:  # noqa: N802
                if not self._authorized():
                    return
                if self.path != "/v1/submit":
                    self._json(HTTPStatus.NOT_FOUND, {"error": "not found"})
                    return
                length_text = self.headers.get("Content-Length")
                if not length_text:
                    self._json(HTTPStatus.LENGTH_REQUIRED, {"error": "missing body"})
                    return
                try:
                    length = int(length_text)
                except ValueError:
                    self._json(HTTPStatus.BAD_REQUEST, {"error": "invalid length"})
                    return
                if length > MAX_SUBMISSION_BYTES:
                    self._json(
                        HTTPStatus.REQUEST_ENTITY_TOO_LARGE,
                        {"error": "candidate exceeds 1 MB"},
                    )
                    return
                try:
                    payload = json.loads(self.rfile.read(length))
                except (json.JSONDecodeError, UnicodeDecodeError):
                    self._json(HTTPStatus.BAD_REQUEST, {"error": "invalid JSON"})
                    return
                code = payload.get("code") if isinstance(payload, dict) else None
                if not isinstance(code, str):
                    self._json(
                        HTTPStatus.BAD_REQUEST,
                        {"error": "request must contain string field 'code'"},
                    )
                    return
                status, response = session.submit(code)
                self._json(status, response)

            def log_message(self, _format: str, *_args: object) -> None:
                return

            def _authorized(self) -> bool:
                expected = "Bearer " + session.token
                received = self.headers.get("Authorization", "")
                if hmac.compare_digest(received, expected):
                    return True
                self._json(HTTPStatus.UNAUTHORIZED, {"error": "unauthorized"})
                return False

            def _json(self, status: HTTPStatus, payload: dict[str, Any]) -> None:
                body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
                self.send_response(int(status))
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

        return Handler
