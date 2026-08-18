"""Docker isolation and coding-agent adapters for black-box evaluation."""

from __future__ import annotations

import json
import os
import shlex
import shutil
import subprocess
import tempfile
import threading
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from urllib.parse import urlsplit

from pace_bench.agents.session import AgentSession, AgentSessionServer
from pace_bench.core.errors import AgentRuntimeError

DEFAULT_IMAGE = "pace-bench-agent-runtime:0.2.0"
DEFAULT_CODEX_VERSION = "0.144.4"
DEFAULT_CLAUDE_VERSION = "2.1.211"

_DOCKERFILE = """FROM node:20-bookworm-slim
ARG CODEX_VERSION
ARG CLAUDE_VERSION
RUN apt-get update \\
    && apt-get install -y --no-install-recommends ca-certificates curl git jq python3 ripgrep \\
    && rm -rf /var/lib/apt/lists/* \\
    && npm install --global @openai/codex@${CODEX_VERSION} @anthropic-ai/claude-code@${CLAUDE_VERSION}
RUN useradd --create-home --uid 10001 agent
USER 10001:10001
WORKDIR /workspace
ENV HOME=/home/agent \\
    PYGAME_HIDE_SUPPORT_PROMPT=1 \\
    SDL_AUDIODRIVER=dummy \\
    SDL_VIDEODRIVER=dummy
"""


@dataclass(frozen=True)
class AgentContainerConfig:
    """Docker and adapter settings that do not affect benchmark physics."""

    agent: str
    model: str = "unspecified"
    image: str | None = None
    command: str | None = None
    timeout_seconds: float = 3600
    memory: str = "4g"
    cpus: float = 2.0
    max_turns: int = 200
    rebuild_image: bool = False
    codex_version: str = DEFAULT_CODEX_VERSION
    claude_version: str = DEFAULT_CLAUDE_VERSION
    custom_base_url: str | None = None
    custom_api_key_env: str | None = None

    def __post_init__(self) -> None:
        if self.agent not in {"codex", "claude", "custom"}:
            raise ValueError("agent must be codex, claude, or custom")
        if self.agent == "custom" and not self.command:
            raise ValueError("custom agents require --agent-command")
        if self.timeout_seconds <= 0:
            raise ValueError("agent timeout must be positive")
        if self.cpus <= 0:
            raise ValueError("agent CPU limit must be positive")
        if self.max_turns < 1:
            raise ValueError("max_turns must be positive")
        if bool(self.custom_base_url) != bool(self.custom_api_key_env):
            raise ValueError(
                "custom_base_url and custom_api_key_env must be supplied together"
            )
        if self.custom_base_url:
            upstream = urlsplit(self.custom_base_url)
            local_hosts = {"127.0.0.1", "host.docker.internal", "localhost"}
            if not upstream.hostname or (
                upstream.scheme != "https"
                and not (upstream.scheme == "http" and upstream.hostname in local_hosts)
            ):
                raise ValueError(
                    "custom_base_url must use HTTPS (HTTP is allowed only for a "
                    "local test endpoint)"
                )


@dataclass(frozen=True)
class AgentContainerResult:
    exit_code: int
    timed_out: bool
    log_path: Path
    image: str


def run_agent_container(
    session: AgentSession,
    server: AgentSessionServer,
    workspace: Path,
    config: AgentContainerConfig,
) -> AgentContainerResult:
    """Run one agent with no repository mount and allowlisted provider access."""

    _require_docker()
    secrets = _provider_secrets(config)
    runtime_image = _ensure_runtime_image(config)
    agent_image = config.image or runtime_image
    if config.image and not _image_exists(config.image):
        raise AgentRuntimeError(
            f"Agent image {config.image!r} does not exist; build or pull it first"
        )

    identifier = uuid.uuid4().hex[:12]
    network = f"pace-agent-{identifier}"
    gateway_name = f"pace-gateway-{identifier}"
    agent_name = f"pace-agent-run-{identifier}"
    log_path = workspace / "agent.log"

    with tempfile.TemporaryDirectory(prefix="pace-agent-gateway-") as temporary:
        secret_file = Path(temporary) / "secrets.json"
        secret_file.write_text(json.dumps(secrets), encoding="utf-8")
        secret_file.chmod(0o600)
        gateway_source = Path(__file__).with_name("gateway.py").resolve()
        _docker("network", "create", "--internal", network)
        try:
            gateway_command = [
                "run",
                "-d",
                "--name",
                gateway_name,
                "--network",
                network,
                "--network-alias",
                "gateway",
                "--read-only",
                "--cap-drop",
                "ALL",
                "--security-opt",
                "no-new-privileges",
                "--pids-limit",
                "128",
                "--memory",
                "512m",
                "--add-host",
                "host.docker.internal:host-gateway",
                "--mount",
                f"type=bind,src={gateway_source},dst=/runtime/gateway.py,readonly",
                "--mount",
                f"type=bind,src={secret_file},dst=/run/secrets/providers.json,readonly",
                "--tmpfs",
                "/tmp:rw,noexec,nosuid,size=16m",
                "--entrypoint",
                "python3",
                runtime_image,
                "/runtime/gateway.py",
                "--evaluator-url",
                f"http://host.docker.internal:{server.port}",
                "--secret-file",
                "/run/secrets/providers.json",
            ]
            if config.custom_base_url:
                gateway_command.extend(["--custom-base-url", config.custom_base_url])
            _docker(*gateway_command)
            _docker("network", "connect", "bridge", gateway_name)
            _wait_for_gateway(gateway_name)

            command = _agent_command(config, workspace)
            docker_command = _agent_docker_command(
                agent_name,
                network,
                workspace,
                agent_image,
                command,
                config,
            )
            exit_code, timed_out = _run_and_tee(
                docker_command,
                log_path,
                timeout=config.timeout_seconds,
                container_name=agent_name,
            )
            return AgentContainerResult(
                exit_code=exit_code,
                timed_out=timed_out,
                log_path=log_path,
                image=agent_image,
            )
        finally:
            _docker_quiet("rm", "-f", agent_name)
            _docker_quiet("rm", "-f", gateway_name)
            _docker_quiet("network", "rm", network)


def _require_docker() -> None:
    if shutil.which("docker") is None:
        raise AgentRuntimeError(
            "Docker is required for black-box agent evaluation; install Docker Desktop "
            "or Docker Engine and ensure `docker info` succeeds"
        )
    result = subprocess.run(
        ["docker", "info"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    if result.returncode != 0:
        raise AgentRuntimeError(
            "Cannot reach the Docker daemon; start Docker and retry"
        )


def _ensure_runtime_image(config: AgentContainerConfig) -> str:
    if _image_exists(DEFAULT_IMAGE) and not config.rebuild_image:
        return DEFAULT_IMAGE
    with tempfile.TemporaryDirectory(prefix="pace-agent-image-") as temporary:
        dockerfile = Path(temporary) / "Dockerfile"
        dockerfile.write_text(_DOCKERFILE, encoding="utf-8")
        print(
            f"Building {DEFAULT_IMAGE} with Codex {config.codex_version} and "
            f"Claude Code {config.claude_version}..."
        )
        _docker_stream(
            "build",
            "--tag",
            DEFAULT_IMAGE,
            "--build-arg",
            f"CODEX_VERSION={config.codex_version}",
            "--build-arg",
            f"CLAUDE_VERSION={config.claude_version}",
            temporary,
        )
    return DEFAULT_IMAGE


def _image_exists(image: str) -> bool:
    return (
        subprocess.run(
            ["docker", "image", "inspect", image],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        ).returncode
        == 0
    )


def _provider_secrets(config: AgentContainerConfig) -> dict[str, str]:
    secrets: dict[str, str] = {}
    if config.agent == "codex":
        key = os.environ.get("CODEX_API_KEY") or os.environ.get("OPENAI_API_KEY")
        if not key:
            raise AgentRuntimeError(
                "Codex container evaluation requires CODEX_API_KEY (or "
                "OPENAI_API_KEY) on the trusted host"
            )
        secrets["openai"] = key
    elif config.agent == "claude":
        key = os.environ.get("ANTHROPIC_API_KEY")
        if not key:
            raise AgentRuntimeError(
                "Claude Code container evaluation requires ANTHROPIC_API_KEY on "
                "the trusted host"
            )
        secrets["anthropic"] = key
    if config.custom_api_key_env:
        key = os.environ.get(config.custom_api_key_env)
        if not key:
            raise AgentRuntimeError(
                f"Custom gateway key environment variable "
                f"{config.custom_api_key_env!r} is not set"
            )
        secrets["custom"] = key
    return secrets


def _agent_command(config: AgentContainerConfig, workspace: Path) -> list[str]:
    prompt = (workspace / "AGENT_PROMPT.md").read_text(encoding="utf-8")
    if config.agent == "codex":
        command = [
            "codex",
            "exec",
            "--ephemeral",
            "--ignore-user-config",
            "--ignore-rules",
            "--skip-git-repo-check",
            "--dangerously-bypass-approvals-and-sandbox",
            "--json",
            "-c",
            'model_provider="pace_gateway"',
            "-c",
            'model_providers.pace_gateway.name="PACE-Bench credential gateway"',
            "-c",
            'model_providers.pace_gateway.base_url="http://gateway:8080/openai/v1"',
            "-c",
            'model_providers.pace_gateway.env_key="CODEX_API_KEY"',
            "-c",
            'model_providers.pace_gateway.wire_api="responses"',
            "-c",
            "model_providers.pace_gateway.supports_websockets=false",
            "-c",
            'web_search="disabled"',
        ]
        if config.model != "unspecified":
            command.extend(["--model", config.model])
        command.append(prompt)
        return command
    if config.agent == "claude":
        command = [
            "claude",
            "--print",
            "--output-format",
            "stream-json",
            "--verbose",
            "--max-turns",
            str(config.max_turns),
            "--dangerously-skip-permissions",
            "--disallowedTools",
            "WebSearch",
            "WebFetch",
        ]
        if config.model != "unspecified":
            command.extend(["--model", config.model])
        command.append(prompt)
        return command
    assert config.command is not None
    replacements = {
        "{prompt_file}": "/workspace/AGENT_PROMPT.md",
        "{task_file}": "/workspace/TASK.md",
        "{workspace}": "/workspace",
    }
    command = shlex.split(config.command)
    for token, replacement in replacements.items():
        command = [part.replace(token, replacement) for part in command]
    return command


def _agent_docker_command(
    name: str,
    network: str,
    workspace: Path,
    image: str,
    command: list[str],
    config: AgentContainerConfig,
) -> list[str]:
    environment = {
        "PACE_AGENT_PROMPT_FILE": "/workspace/AGENT_PROMPT.md",
        "PACE_AGENT_TASK_FILE": "/workspace/TASK.md",
        "PACE_AGENT_SUBMIT": "/workspace/pace-submit",
        "PACE_AGENT_API_BASE": "http://gateway:8080/custom",
        "PACE_AGENT_API_KEY": "pace-bench-gateway",
        "CODEX_HOME": "/home/agent",
        "CODEX_API_KEY": "pace-bench-gateway",
        "ANTHROPIC_API_KEY": "pace-bench-gateway",
        "ANTHROPIC_BASE_URL": "http://gateway:8080/anthropic",
        "CLAUDE_CODE_DISABLE_NONESSENTIAL_TRAFFIC": "1",
        "DISABLE_TELEMETRY": "1",
        "DISABLE_ERROR_REPORTING": "1",
        "DISABLE_BUG_COMMAND": "1",
        "DISABLE_AUTOUPDATER": "1",
        "NO_PROXY": "",
        "no_proxy": "",
    }
    docker_command = [
        "docker",
        "run",
        "--name",
        name,
        "--network",
        network,
        "--read-only",
        "--cap-drop",
        "ALL",
        "--security-opt",
        "no-new-privileges",
        "--pids-limit",
        "512",
        "--memory",
        config.memory,
        "--cpus",
        str(config.cpus),
        "--user",
        "10001:10001",
        "--workdir",
        "/workspace",
        "--mount",
        f"type=bind,src={workspace.resolve()},dst=/workspace",
        "--tmpfs",
        "/tmp:rw,nosuid,size=512m",
        "--tmpfs",
        "/home/agent:rw,nosuid,size=512m,uid=10001,gid=10001,mode=0700",
    ]
    for name_, value in environment.items():
        docker_command.extend(["--env", f"{name_}={value}"])
    docker_command.extend(["--entrypoint", command[0], image])
    docker_command.extend(command[1:])
    return docker_command


def _wait_for_gateway(container: str) -> None:
    check = (
        "import urllib.request; "
        "urllib.request.urlopen('http://127.0.0.1:8080/health', timeout=2).read()"
    )
    for _ in range(30):
        result = subprocess.run(
            ["docker", "exec", container, "python3", "-c", check],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        if result.returncode == 0:
            return
        time.sleep(0.2)
    logs = subprocess.run(
        ["docker", "logs", container], capture_output=True, text=True, check=False
    )
    raise AgentRuntimeError(f"Credential gateway did not start: {logs.stderr[-1000:]}")


def _run_and_tee(
    command: list[str],
    log_path: Path,
    *,
    timeout: float,
    container_name: str,
) -> tuple[int, bool]:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    assert process.stdout is not None

    with log_path.open("w", encoding="utf-8") as log:

        def copy_output() -> None:
            for line in process.stdout:
                print(line, end="")
                log.write(line)
                log.flush()

        reader = threading.Thread(
            target=copy_output, name="pace-agent-log", daemon=True
        )
        reader.start()
        try:
            exit_code = process.wait(timeout=timeout)
            timed_out = False
        except subprocess.TimeoutExpired:
            timed_out = True
            _docker_quiet("stop", "--time", "5", container_name)
            try:
                exit_code = process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                process.kill()
                exit_code = process.wait()
        reader.join(timeout=5)
    return exit_code, timed_out


def _docker(*arguments: str) -> str:
    result = subprocess.run(
        ["docker", *arguments],
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        detail = result.stderr.strip() or result.stdout.strip()
        raise AgentRuntimeError(f"docker {' '.join(arguments[:3])} failed: {detail}")
    return result.stdout.strip()


def _docker_stream(*arguments: str) -> None:
    result = subprocess.run(["docker", *arguments], check=False)
    if result.returncode != 0:
        raise AgentRuntimeError(
            f"docker {' '.join(arguments[:3])} failed with exit code "
            f"{result.returncode}"
        )


def _docker_quiet(*arguments: str) -> None:
    subprocess.run(
        ["docker", *arguments],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
