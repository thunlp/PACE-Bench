"""The single public command-line interface for PACE-Bench."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

from pace_bench import __version__
from pace_bench.core.errors import ConfigurationError, PaceBenchError
from pace_bench.evaluation.config import RunConfig
from pace_bench.evaluation.paper import export_paper_artifacts
from pace_bench.evaluation.results import aggregate, load_result, load_results
from pace_bench.evaluation.runner import enumerate_work_items
from pace_bench.evaluation.runner import run_work_items
from pace_bench.evaluation.runner import validate_task_references
from pace_bench.tasks.registry import CATEGORIES, get_registry
from pace_bench.core.types import EnvironmentId, EvaluationResult, RunMode


def build_parser() -> argparse.ArgumentParser:
    """Build the public model, agent, validation, and reporting CLI."""

    parser = argparse.ArgumentParser(
        prog="pace-bench",
        description="Physics Adaptation via Code Evolution in Dynamic Environments",
    )
    parser.add_argument(
        "--version", action="version", version=f"%(prog)s {__version__}"
    )
    commands = parser.add_subparsers(dest="command", required=True)

    list_parser = commands.add_parser("list", help="List tasks and environments")
    list_parser.add_argument("--task", action="append")
    list_parser.add_argument("--json", action="store_true")
    list_parser.set_defaults(handler=_list_command)

    evaluate_parser = commands.add_parser(
        "evaluate",
        help="Run vanilla or an external method on selected task/environment pairs",
    )
    evaluate_parser.add_argument(
        "--task",
        action="append",
        required=True,
        help="Task ID, full task path, category_N, or all; repeat to combine selectors",
    )
    evaluate_parser.add_argument(
        "--env",
        action="append",
        default=None,
        help="Target environment (Stage-1..Stage-4 or all); repeat to select several",
    )
    evaluate_parser.add_argument(
        "--from-scratch",
        action="store_true",
        help="Generate directly instead of evaluating the Initial reference as attempt 0",
    )
    evaluate_parser.add_argument(
        "--method",
        default="vanilla",
        help=(
            "vanilla or an external dotted class path such as "
            "package.module:MyMethod; third-party research methods are not bundled"
        ),
    )
    evaluate_parser.add_argument(
        "--method-option",
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="Method constructor option; repeat for multiple values (JSON is accepted)",
    )
    evaluate_parser.add_argument(
        "--provider",
        default="mock",
        help=(
            "vllm, openai-compatible, local-transformers, mock, "
            "or package.module:Provider"
        ),
    )
    evaluate_parser.add_argument(
        "--model", default="mock", help="Model name or local path"
    )
    evaluate_parser.add_argument(
        "--api-key", help="API key; otherwise use OPENAI_API_KEY or VLLM_API_KEY"
    )
    evaluate_parser.add_argument(
        "--base-url",
        help="OpenAI-compatible endpoint; vLLM defaults to http://127.0.0.1:8000/v1",
    )
    evaluate_parser.add_argument("--device", default="auto")
    evaluate_parser.add_argument(
        "--devices",
        help="Comma-separated devices for local parallel runs, e.g. cuda:0,cuda:1",
    )
    evaluate_parser.add_argument("--dtype", default="auto")
    evaluate_parser.add_argument("--attempts", type=int, default=20)
    evaluate_parser.add_argument(
        "--runs",
        type=int,
        default=2,
        help="Independent complete trajectories per pair (paper default: 2 for Pass@2)",
    )
    evaluate_parser.add_argument("--workers", type=int, default=1)
    evaluate_parser.add_argument("--max-steps", type=int)
    evaluate_parser.add_argument("--generation-retries", type=int, default=2)
    evaluate_parser.add_argument("--timeout-seconds", type=float)
    evaluate_parser.add_argument("--seed", type=int, default=0)
    evaluate_parser.add_argument("--temperature", type=float, default=0.7)
    evaluate_parser.add_argument("--top-p", type=float, default=0.95)
    evaluate_parser.add_argument("--max-tokens", type=int, default=65_536)
    evaluate_parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help=(
            "Result root; defaults to results for adaptation and results_scratch "
            "for from-scratch runs"
        ),
    )
    evaluate_parser.add_argument(
        "--save-gif",
        action="store_true",
        help="Save one rendered GIF for every verified attempt",
    )
    evaluate_parser.add_argument("--display", action="store_true")
    evaluate_parser.add_argument("--no-resume", action="store_true")
    evaluate_parser.add_argument("--dry-run", action="store_true")
    evaluate_parser.set_defaults(handler=_evaluate_command)

    agent_parser = commands.add_parser(
        "agent",
        help="Evaluate Codex, Claude Code, or a custom agent in a black-box container",
    )
    agent_parser.add_argument("--task", required=True, help="One benchmark task ID")
    agent_parser.add_argument(
        "--env", required=True, help="Target environment, Stage-1 through Stage-4"
    )
    agent_parser.add_argument(
        "--agent", choices=("codex", "claude", "custom"), required=True
    )
    agent_parser.add_argument(
        "--model", default="unspecified", help="Agent model name for the run record"
    )
    agent_parser.add_argument("--attempts", type=int, default=20)
    agent_parser.add_argument("--max-steps", type=int)
    agent_parser.add_argument(
        "--runs", type=int, default=1, help="Complete agent trajectories to execute"
    )
    agent_parser.add_argument(
        "--run-index", type=int, default=1, help="Index of the first trajectory"
    )
    agent_parser.add_argument("--overwrite", action="store_true")
    agent_parser.add_argument("--seed", type=int, default=0)
    agent_parser.add_argument("--output", type=Path, default=Path("results"))
    agent_parser.add_argument("--workspace", type=Path)
    agent_parser.add_argument(
        "--prompt-file", type=Path, help="Replace the default agent instruction"
    )
    agent_parser.add_argument(
        "--image", help="Custom agent image; the built-in image is used by default"
    )
    agent_parser.add_argument(
        "--agent-command",
        help="Command inside a custom image; supports {prompt_file}, {task_file}, {workspace}",
    )
    agent_parser.add_argument("--timeout-seconds", type=float, default=3600)
    agent_parser.add_argument("--memory", default="4g")
    agent_parser.add_argument("--cpus", type=float, default=2.0)
    agent_parser.add_argument("--max-turns", type=int, default=200)
    agent_parser.add_argument("--save-gif", action="store_true")
    agent_parser.add_argument("--display", action="store_true")
    agent_parser.add_argument("--rebuild-image", action="store_true")
    agent_parser.add_argument("--codex-version", default="0.144.4")
    agent_parser.add_argument("--claude-version", default="2.1.211")
    agent_parser.add_argument(
        "--custom-base-url",
        help="Optional HTTPS model gateway for a custom agent",
    )
    agent_parser.add_argument(
        "--custom-api-key-env",
        help="Trusted-host environment variable injected at the custom gateway",
    )
    agent_parser.set_defaults(handler=_agent_command)

    validate_parser = commands.add_parser(
        "validate", help="Validate task contracts/references"
    )
    validate_parser.add_argument("--task", action="append")
    validate_parser.add_argument("--contracts-only", action="store_true")
    validate_parser.add_argument("--skip-initial-failures", action="store_true")
    validate_parser.add_argument("--max-steps", type=int)
    validate_parser.set_defaults(handler=_validate_command)

    report_parser = commands.add_parser("report", help="Summarize result JSON files and export paper artifacts")
    report_parser.add_argument("--input", type=Path, required=True)
    report_parser.add_argument("--output", type=Path)
    report_parser.add_argument(
        "--table-dir",
        type=Path,
        help="Directory for LaTeX table fragments; defaults to <result-root>/table",
    )
    report_parser.add_argument(
        "--figure-dir",
        type=Path,
        help="Directory for PDF/PNG figures; defaults to <result-root>/figures",
    )
    report_parser.set_defaults(handler=_report_command)
    return parser


def _list_command(args: argparse.Namespace) -> int:
    registry = get_registry()
    tasks = registry.select(args.task or ["all"])
    if args.json:
        print(
            json.dumps(
                [
                    {
                        "task": task.full_name,
                        "id": task.name,
                        "category": task.category_number,
                        "environments": [
                            str(environment.environment_id)
                            for environment in registry.environments(task)
                        ],
                    }
                    for task in tasks
                ],
                indent=2,
            )
        )
        return 0
    for number, (_, _, display_name) in CATEGORIES.items():
        selected = [task for task in tasks if task.category_number == number]
        if selected:
            print(f"Category {number}: {display_name}")
            for task in selected:
                environments = ", ".join(
                    str(environment.environment_id)
                    for environment in registry.environments(task)
                )
                print(f"  {task.name:<4}  {task.full_name}  [{environments}]")
    demos = [task for task in tasks if not task.benchmark]
    if demos:
        print("Demo")
        for task in demos:
            environments = ", ".join(
                str(environment.environment_id)
                for environment in registry.environments(task)
            )
            print(f"  {task.name:<4}  {task.full_name}  [{environments}]")
    benchmark_count = sum(task.benchmark for task in tasks)
    print(f"{benchmark_count} benchmark task(s), {len(demos)} demo(s)")
    return 0


def _evaluate_command(args: argparse.Namespace) -> int:
    mode = RunMode.FROM_SCRATCH if args.from_scratch else RunMode.ADAPTATION
    environments = args.env or (["Initial"] if args.from_scratch else ["Stage-1"])
    base = _config_from_args(args, mode)
    items = enumerate_work_items(
        base,
        task_selectors=args.task,
        stages=environments,
        runs=args.runs,
    )
    if not items:
        raise ConfigurationError(
            "The selected tasks/environments produced no evaluation pairs"
        )
    outcomes = run_work_items(items, workers=args.workers, dry_run=args.dry_run)
    failures = 0
    for outcome in outcomes:
        if args.dry_run:
            print(outcome.work_item.identity)
        elif outcome.error:
            failures += 1
            print(f"ERROR {outcome.work_item.identity}: {outcome.error}")
        else:
            result = outcome.result
            print(
                f"DONE {outcome.work_item.identity}: success={result.success} "
                f"best_score={result.best_score:.3f} stop={result.stop_reason}"
            )
    completed = [outcome.result for outcome in outcomes if outcome.result is not None]
    if completed and not args.dry_run:
        pass_at_k = aggregate(completed)["pass_at_k"]
        rendered = " ".join(
            f"{name}={values['rate']:.3f} ({values['passed_pairs']}/"
            f"{values['pair_count']} pairs)"
            for name, values in pass_at_k.items()
        )
        if rendered:
            print(rendered)
    print(f"{len(outcomes)} work item(s), {failures} orchestration error(s)")
    return 1 if failures else 0


def _config_from_args(args: argparse.Namespace, mode: RunMode) -> RunConfig:
    options: dict[str, Any] = {}
    if args.provider in {"openai", "openai-compatible", "vllm"}:
        if args.api_key:
            options["api_key"] = args.api_key
        if args.base_url:
            options["base_url"] = args.base_url
        if args.timeout_seconds is not None:
            options["timeout"] = args.timeout_seconds
    elif args.provider in {"local", "local-transformers", "transformers"}:
        if args.devices:
            pool = [
                device.strip() for device in args.devices.split(",") if device.strip()
            ]
            if not pool:
                raise ConfigurationError("--devices must contain at least one device")
            options["device_pool"] = pool
        else:
            options["device"] = args.device
        options["dtype"] = args.dtype
    return RunConfig(
        task=args.task[0],
        mode=mode,
        source=EnvironmentId("Initial"),
        target=EnvironmentId("Initial" if mode == RunMode.FROM_SCRATCH else "Stage-1"),
        provider=args.provider,
        model=args.model,
        strategy=args.method,
        attempts=args.attempts,
        max_steps=args.max_steps,
        generation_retries=args.generation_retries,
        timeout_seconds=args.timeout_seconds,
        seed=args.seed,
        temperature=args.temperature,
        top_p=args.top_p,
        max_tokens=args.max_tokens,
        headless=not args.display,
        save_gif=args.save_gif,
        output=args.output
        or Path("results_scratch" if mode == RunMode.FROM_SCRATCH else "results"),
        resume=not args.no_resume,
        provider_options=options,
        method_options=_key_value_options(args.method_option, "--method-option"),
    )


def _key_value_options(values: list[str], flag: str) -> dict[str, Any]:
    """Parse repeatable CLI KEY=VALUE options, accepting JSON scalar values."""

    options: dict[str, Any] = {}
    for value in values:
        if "=" not in value:
            raise ConfigurationError(f"{flag} expects KEY=VALUE, got {value!r}")
        key, raw = value.split("=", 1)
        key = key.strip()
        if not key:
            raise ConfigurationError(f"{flag} requires a non-empty key")
        if key in options:
            raise ConfigurationError(f"Duplicate {flag} key: {key}")
        try:
            options[key] = json.loads(raw)
        except json.JSONDecodeError:
            options[key] = raw
    return options


def _agent_command(args: argparse.Namespace) -> int:
    """Run one coding agent without exposing the installed benchmark package."""

    if args.runs < 1:
        raise ConfigurationError("--runs must be at least 1")
    results = []
    failed = False
    for offset in range(args.runs):
        result, run_failed = _agent_run_once(
            args,
            run_index=args.run_index + offset,
            seed=args.seed + offset,
        )
        results.append(result)
        failed = failed or run_failed
    pass_at_k = aggregate(results)["pass_at_k"]
    rendered = " ".join(
        f"{name}={values['rate']:.3f} ({values['passed_pairs']}/"
        f"{values['pair_count']} pairs)"
        for name, values in pass_at_k.items()
    )
    if rendered:
        print(rendered)
    return 1 if failed else 0


def _agent_run_once(
    args: argparse.Namespace, *, run_index: int, seed: int
) -> tuple[EvaluationResult, bool]:
    """Execute and persist one isolated coding-agent trajectory."""

    from pace_bench.agents.container import AgentContainerConfig, run_agent_container
    from pace_bench.agents.session import (
        AgentSession,
        AgentSessionConfig,
        AgentSessionServer,
    )

    if args.prompt_file and not args.prompt_file.is_file():
        raise ConfigurationError(f"Prompt file does not exist: {args.prompt_file}")
    session = AgentSession(
        AgentSessionConfig(
            task=args.task,
            target=EnvironmentId(args.env),
            attempts=args.attempts,
            max_steps=args.max_steps,
            output=args.output,
            agent=args.agent,
            model=args.model,
            headless=not args.display,
            save_gif=args.save_gif,
            seed=seed,
            run_index=run_index,
            prompt_file=args.prompt_file,
            overwrite=args.overwrite,
        )
    )
    json_root = next(
        parent for parent in session.result_file.parents if parent.name == "json"
    )
    result_root = json_root.parent
    result_identity = session.result_file.relative_to(result_root / "json").with_suffix(
        ""
    )
    requested_workspace = (
        args.workspace / f"run-{run_index}"
        if args.workspace is not None and args.runs > 1
        else args.workspace
    )
    workspace = requested_workspace or (
        result_root
        / "workspaces"
        / result_identity
        / f"session-{int(session.started_at)}"
    )
    if workspace.exists() and any(workspace.iterdir()):
        session.close()
        raise ConfigurationError(
            f"Agent workspace is not empty: {workspace}; choose --workspace elsewhere"
        )
    session.create_workspace(workspace)
    session.record_runtime_metadata(workspace=str(workspace))
    print(
        f"AGENT {session.task.name}:{session.environment_pair} "
        f"attempt0={session.attempts[0].score:.3f} budget={args.attempts}"
    )
    if session.complete:
        print(
            f"DONE success=True stop={session.stop_reason} result={session.result_file}"
        )
        session.close()
        return load_result(session.result_file), False

    server = AgentSessionServer(session)
    server.start()
    try:
        container_result = run_agent_container(
            session,
            server,
            workspace,
            AgentContainerConfig(
                agent=args.agent,
                model=args.model,
                image=args.image,
                command=args.agent_command,
                timeout_seconds=args.timeout_seconds,
                memory=args.memory,
                cpus=args.cpus,
                max_turns=args.max_turns,
                rebuild_image=args.rebuild_image,
                codex_version=args.codex_version,
                claude_version=args.claude_version,
                custom_base_url=args.custom_base_url,
                custom_api_key_env=args.custom_api_key_env,
            ),
        )
        reason = "agent_timeout" if container_result.timed_out else None
        session.record_runtime_metadata(
            container_image=container_result.image,
            agent_log=str(container_result.log_path),
            timed_out=container_result.timed_out,
        )
        session.finalize(
            agent_exit_code=container_result.exit_code,
            reason=reason,
        )
    except Exception:
        session.finalize(agent_exit_code=None, reason="orchestration_error")
        raise
    finally:
        server.close()
        session.close()

    status = session.public_status()
    print(
        f"DONE success={status['success']} best_score={status['best_score']:.3f} "
        f"submissions={status['submitted']} stop={status['stop_reason']}"
    )
    print(f"Result: {session.result_file}")
    print(f"Workspace and agent log: {workspace}")
    failed = status["stop_reason"] in {"agent_error", "agent_timeout"}
    return load_result(session.result_file), failed


def _validate_command(args: argparse.Namespace) -> int:
    registry = get_registry()
    tasks = registry.select(args.task or ["all"])
    failures = 0
    for task in tasks:
        errors = registry.validate(task, import_modules=True)
        for error in errors:
            print(f"FAIL {task.full_name}: {error}")
        failures += len(errors)
        if errors:
            continue
        print(f"PASS {task.full_name}: contract")
        if args.contracts_only:
            continue
        for check in validate_task_references(
            task,
            registry=registry,
            check_initial_failures=not args.skip_initial_failures,
            max_steps=args.max_steps,
        ):
            label = "PASS" if check.passed else "FAIL"
            print(
                f"{label} {check.task}: {check.reference_environment} reference on "
                f"{check.execution_environment} score={check.score:.3f}"
            )
            if not check.passed:
                failures += 1
                if check.error:
                    print(f"  error: {check.error}")
    print(f"Validation complete: {len(tasks)} task(s), {failures} failure(s)")
    return 1 if failures else 0


def _report_command(args: argparse.Namespace) -> int:
    results, errors = load_results(args.input)
    summary = aggregate(results)
    summary["invalid_result_count"] = len(errors)
    rendered = json.dumps(summary, indent=2, sort_keys=True)
    print(rendered)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    result_root = args.input.parent if args.input.name == "json" else args.input
    table_dir = args.table_dir or result_root / "table"
    figure_dir = args.figure_dir or result_root / "figures"
    artifacts = export_paper_artifacts(summary, table_dir, figure_dir, results=results)
    print(f"Paper tables saved to: {table_dir} ({artifacts['tables']} files)")
    print(f"Paper figures saved to: {figure_dir} ({artifacts['figures']} PDF files + PNG copies)")
    if artifacts.get("figure_error"):
        print(f"warning: {artifacts['figure_error']}")
    for path, error in errors:
        print(f"warning: skipped {path}: {error}")
    return 0 if results else 1


def main(argv: list[str] | None = None) -> int:
    try:
        args = build_parser().parse_args(argv)
        return int(args.handler(args))
    except (PaceBenchError, ValueError) as exc:
        print(f"pace-bench: error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
