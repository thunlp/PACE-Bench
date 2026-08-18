"""Orchestrate static checks, task setup, and preserved Box2D simulation loops."""

from __future__ import annotations

import importlib.util
import random
import sys
import threading
import time
import traceback
from pathlib import Path
from types import SimpleNamespace
from typing import Any

from pace_bench.evaluation.verification.diagnostics import format_feedback
from pace_bench.evaluation.verification.safety import CodeSafetyMixin
from pace_bench.evaluation.verification.safety import ProhibitedOperationError
from pace_bench.evaluation.verification.simulation import SimulationMixin
from pace_bench.evaluation.verification.task_runtime import TaskRuntimeMixin
from pace_bench.tasks.registry import TaskRegistry, get_registry
from pace_bench.tasks.registry import EnvironmentSpec, TaskSpec
from pace_bench.core.types import VerificationResult


# pygame, Box2D task modules, ``random.seed()``, ``sys.path``, and the legacy
# module aliases used by CodeVerifier are process-global. Model generation may
# run concurrently, but the native verification lifecycle must not overlap
# across worker threads.
_NATIVE_VERIFICATION_LOCK = threading.RLock()


class CodeVerifier(CodeSafetyMixin, TaskRuntimeMixin, SimulationMixin):
    """Verify generated code against one registered task environment."""

    def __init__(
        self,
        task_name: str,
        max_steps: int = 10000,
        env_overrides: dict[str, Any] | None = None,
        registry: TaskRegistry | None = None,
    ) -> None:
        self.task_name = task_name
        self.max_steps = max_steps
        self.env_overrides = env_overrides or {}
        self.simulator = None
        self.registry = registry or get_registry()
        task = self.registry.resolve(task_name)
        self._task_dir = str(task.path)
        self._loaded_task_modules: dict[str, Any] = {}
        self.task_module = SimpleNamespace()

        if self._task_dir not in sys.path:
            sys.path.insert(0, self._task_dir)
        for module_name in ("environment", "evaluator", "agent"):
            self._load_task_module(module_name, task.path / f"{module_name}.py")
        if not hasattr(self.task_module, "environment"):
            raise ImportError(f"Environment file not found in {task.path}")
        self.allowed_apis = self._load_allowed_apis(task)

    def _load_task_module(self, module_name: str, path: Path) -> None:
        if not path.is_file():
            return
        spec = importlib.util.spec_from_file_location(module_name, path)
        if spec is None or spec.loader is None:
            raise ImportError(f"Cannot load task module {path}")
        module = importlib.util.module_from_spec(spec)
        setattr(self.task_module, module_name, module)
        sys.modules[module_name] = module
        spec.loader.exec_module(module)
        self._loaded_task_modules[module_name] = module

    def verify_code(
        self,
        code: str,
        headless: bool = True,
        save_gif_path: str | None = None,
        granularity: str = "outcome-based",
    ) -> tuple[bool, float, dict[str, Any], str | None]:
        """Check, execute, and simulate a candidate without raising solution failures."""

        try:
            self._check_prohibited_operations(code)
            code_module = self._execute_code(code)
            physics = self.env_overrides.get("physics_config", {})
            terrain = self.env_overrides.get("terrain_config", {})
            simulation_seed = int(
                physics.get("random_seed", terrain.get("target_rng_seed", 123))
            )
            random.seed(simulation_seed)
            environment = self._init_environment()
            if hasattr(environment, "remove_initial_template"):
                environment.remove_initial_template()
            build_agent = getattr(code_module, "build_agent", None)
            if not build_agent:
                message = "Code missing build_agent function"
                return (
                    False,
                    0.0,
                    {
                        "error_type": "missing_function",
                        "error_stage": "code_parsing",
                        "error_message": message,
                    },
                    message,
                )
            try:
                agent_components = build_agent(environment)
            except Exception as exc:
                message = f"Error building agent: {exc}"
                return (
                    False,
                    0.0,
                    {
                        "error_type": "agent_building_error",
                        "error_stage": "agent_construction",
                        "error_message": str(exc),
                        "error_traceback": traceback.format_exc(),
                    },
                    message,
                )
            if "K_05" in self.task_name and hasattr(
                environment, "enforce_object_at_ground"
            ):
                environment.enforce_object_at_ground()
            evaluator = self._init_evaluator(environment)
            success, score, metrics = self._run_simulation(
                environment,
                agent_components,
                evaluator,
                code_module,
                headless,
                save_gif_path,
                granularity,
            )
            return success, score, metrics, None
        except ProhibitedOperationError as exc:
            message = str(exc)
            return (
                False,
                0.0,
                {
                    "error_type": "prohibited_operation",
                    "error_stage": "static_analysis",
                    "error_message": message,
                },
                message,
            )
        except SyntaxError as exc:
            message = f"Code syntax error: {exc}"
            return (
                False,
                0.0,
                {
                    "error_type": "syntax_error",
                    "error_stage": "code_compilation",
                    "error_message": str(exc),
                    "error_line": getattr(exc, "lineno", None),
                    "error_text": getattr(exc, "text", None),
                },
                message,
            )
        except NameError as exc:
            message = f"Code name error: {exc}"
            return (
                False,
                0.0,
                {
                    "error_type": "name_error",
                    "error_stage": "code_execution",
                    "error_message": str(exc),
                },
                message,
            )
        except Exception as exc:
            message = f"Verification process error: {exc}"
            return (
                False,
                0.0,
                {
                    "error_type": "execution_error",
                    "error_stage": "verification",
                    "error_message": str(exc),
                    "error_traceback": traceback.format_exc(),
                },
                message,
            )

    def cleanup(self) -> None:
        """Release pygame state and temporary legacy task-module aliases."""

        if self.simulator:
            self.simulator.quit()
            self.simulator = None
        while self._task_dir in sys.path:
            sys.path.remove(self._task_dir)
        for name, module in self._loaded_task_modules.items():
            if sys.modules.get(name) is module:
                del sys.modules[name]


class PhysicsVerifier:
    """Typed adapter around the preserved Box2D verifier."""

    def __init__(
        self,
        task: TaskSpec,
        environment: EnvironmentSpec,
        *,
        max_steps: int,
        headless: bool,
        save_gif: bool,
        artifact_directory: Path | None,
        registry: TaskRegistry,
    ) -> None:
        self.task = task
        self.environment = environment
        self.headless = headless
        self.save_gif = save_gif
        self.artifact_directory = artifact_directory
        self.max_steps = max_steps
        self.registry = registry

    def verify(self, code: str, attempt: int) -> VerificationResult:
        artifact_paths: list[str] = []
        gif_path: Path | None = None
        if self.save_gif and self.artifact_directory is not None:
            self.artifact_directory.mkdir(parents=True, exist_ok=True)
            gif_path = self.artifact_directory / f"attempt-{attempt:02d}.gif"
        with _NATIVE_VERIFICATION_LOCK:
            started = time.perf_counter()
            verifier = CodeVerifier(
                self.task.full_name,
                max_steps=self.max_steps,
                env_overrides=self.environment.overrides,
                registry=self.registry,
            )
            try:
                success, score, metrics, error = verifier.verify_code(
                    code,
                    headless=self.headless,
                    save_gif_path=str(gif_path) if gif_path else None,
                    granularity="outcome-based",
                )
            finally:
                verifier.cleanup()
            if gif_path and not gif_path.is_file():
                _save_no_frames_gif(
                    gif_path,
                    attempt=attempt,
                    message=error
                    or str((metrics or {}).get("failure_reason") or ""),
                )
            if gif_path and gif_path.is_file():
                artifact_paths.append(str(gif_path))
            metrics = dict(metrics or {})
            feedback = format_feedback(
                metrics,
                float(score),
                bool(success),
                bool(metrics.get("failed", not success)),
                metrics.get("failure_reason"),
                iteration=attempt,
                error=error,
                task_name=self.task.full_name,
                include_suggestions=False,
            )
            return VerificationResult(
                success=bool(success),
                score=float(score),
                metrics=metrics,
                feedback=feedback,
                error=error,
                artifact_paths=artifact_paths,
                duration_seconds=time.perf_counter() - started,
            )

    def close(self) -> None:
        """Each verification owns and cleans its native verifier eagerly."""


def _save_no_frames_gif(path: Path, *, attempt: int, message: str) -> None:
    """Create an explicit artifact when verification ends before rendering."""

    try:
        from PIL import Image, ImageDraw

        image = Image.new("RGB", (720, 160), "white")
        draw = ImageDraw.Draw(image)
        text = (
            f"PACE-Bench attempt {attempt}\n"
            "No simulation frames were produced.\n"
            f"{message[:180]}"
        )
        draw.multiline_text((20, 20), text, fill="black", spacing=8)
        path.parent.mkdir(parents=True, exist_ok=True)
        image.save(path, format="GIF")
    except (ImportError, OSError, UnicodeError, ValueError):
        # GIF capture is optional and must never alter evaluation semantics.
        return
