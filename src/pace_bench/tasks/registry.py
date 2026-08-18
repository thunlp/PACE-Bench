"""Discovery, validation, and loading for the 36 tasks and basic demo."""

from __future__ import annotations

import ast
import importlib
import importlib.util
import sys
from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass, field
from functools import lru_cache
from pathlib import Path
from types import ModuleType
from typing import Any

from pace_bench.core.errors import TaskContractError, TaskNotFoundError
from pace_bench.core.paths import TASK_CATEGORIES_ROOT, TASK_DEMOS_ROOT, repository_root
from pace_bench.tasks.stage_prompt import uniform_suffix_for_task
from pace_bench.core.types import EnvironmentId, EnvironmentPair, TaskId

CATEGORIES: dict[int, tuple[str, str, str]] = {
    1: ("Category1_Statics_Equilibrium", "S", "Statics / Equilibrium"),
    2: ("Category2_Kinematics_Linkages", "K", "Kinematics / Linkages"),
    3: ("Category3_Dynamics_Energy", "D", "Dynamics / Energy"),
    4: ("Category4_Granular_FluidInteraction", "F", "Granular / Fluid Interaction"),
    5: ("Category5_Cybernetics_Control", "C", "Cybernetics / Control"),
    6: ("Category6_ExoticPhysics", "E", "Exotic Physics"),
}
DEMO_NAMES = ("basic",)

BENCHMARK_REQUIRED_FILES = frozenset(
    {
        "agent.py",
        "environment.py",
        "evaluator.py",
        "feedback.py",
        "prompt.py",
        "renderer.py",
        "stages.py",
    }
)
DEMO_REQUIRED_FILES = BENCHMARK_REQUIRED_FILES - {"stages.py"}


@dataclass(frozen=True)
class TaskSpec:
    name: str
    path: Path
    module_name: str
    benchmark: bool
    category_number: int | None = None
    category_name: str | None = None
    task_id: TaskId | None = None
    legacy_alias: str | None = None

    @property
    def required_files(self) -> frozenset[str]:
        return BENCHMARK_REQUIRED_FILES if self.benchmark else DEMO_REQUIRED_FILES

    @property
    def full_name(self) -> str:
        return (
            f"{self.category_name}/{self.task_id}"
            if self.benchmark
            else f"demo/{self.name}"
        )


@dataclass(frozen=True)
class EnvironmentSpec:
    environment_id: EnvironmentId
    title: str
    terrain_config: dict[str, Any] = field(default_factory=dict)
    physics_config: dict[str, Any] = field(default_factory=dict)
    task_description_suffix: str = ""
    raw: dict[str, Any] = field(default_factory=dict)

    @property
    def overrides(self) -> dict[str, dict[str, Any]]:
        return {
            "terrain_config": dict(self.terrain_config),
            "physics_config": dict(self.physics_config),
        }


def _source_roots() -> tuple[Path, Path]:
    """Use packaged tasks, with a pre-migration checkout fallback."""

    if TASK_CATEGORIES_ROOT.is_dir() and any(
        child.is_dir() and child.name.startswith("Category")
        for child in TASK_CATEGORIES_ROOT.iterdir()
    ):
        return TASK_CATEGORIES_ROOT, TASK_DEMOS_ROOT
    root = repository_root()
    if root and (root / "tasks").is_dir():
        return root / "tasks", root / "tasks" / "demo"
    return TASK_CATEGORIES_ROOT, TASK_DEMOS_ROOT


@contextmanager
def _task_import_path(task_dir: Path) -> Iterator[None]:
    legacy_names = (
        "agent",
        "environment",
        "evaluator",
        "feedback",
        "prompt",
        "renderer",
        "stages",
    )
    previous_modules = {
        name: sys.modules.pop(name) for name in legacy_names if name in sys.modules
    }
    path = str(task_dir)
    sys.path.insert(0, path)
    try:
        yield
    finally:
        while path in sys.path:
            sys.path.remove(path)
        for name in legacy_names:
            module = sys.modules.get(name)
            module_file = getattr(module, "__file__", None)
            if module_file and Path(module_file).resolve().parent == task_dir.resolve():
                sys.modules.pop(name, None)
        sys.modules.update(previous_modules)


class TaskRegistry:
    """Deterministic manifest derived from the packaged task directories."""

    def __init__(self) -> None:
        categories_root, demos_root = _source_roots()
        self.categories_root = categories_root
        self.demos_root = demos_root
        self._benchmark_tasks = self._discover_benchmark_tasks()
        self._demos = self._discover_demos()
        self._lookup = self._build_lookup()

    def _discover_benchmark_tasks(self) -> tuple[TaskSpec, ...]:
        tasks: list[TaskSpec] = []
        packaged = self.categories_root == TASK_CATEGORIES_ROOT
        for category_number, (
            category_dir,
            prefix,
            _display_name,
        ) in CATEGORIES.items():
            category_path = self.categories_root / category_dir
            for number in range(1, 7):
                task_id = TaskId(f"{prefix}_{number:02d}")
                task_path = category_path / str(task_id)
                if not task_path.is_dir():
                    continue
                module_root = "pace_bench.tasks.categories" if packaged else "tasks"
                tasks.append(
                    TaskSpec(
                        name=str(task_id),
                        path=task_path,
                        module_name=f"{module_root}.{category_dir}.{task_id}",
                        benchmark=True,
                        category_number=category_number,
                        category_name=category_dir,
                        task_id=task_id,
                        legacy_alias=f"category_{category_number}_{number:02d}",
                    )
                )
        return tuple(tasks)

    def _discover_demos(self) -> tuple[TaskSpec, ...]:
        demos: list[TaskSpec] = []
        packaged = self.demos_root == TASK_DEMOS_ROOT
        module_root = "pace_bench.tasks.demos" if packaged else "tasks.demo"
        for name in DEMO_NAMES:
            path = self.demos_root / name
            if path.is_dir():
                demos.append(
                    TaskSpec(
                        name=name,
                        path=path,
                        module_name=f"{module_root}.{name}",
                        benchmark=False,
                    )
                )
        return tuple(demos)

    def _build_lookup(self) -> dict[str, TaskSpec]:
        lookup: dict[str, TaskSpec] = {}
        for task in (*self._benchmark_tasks, *self._demos):
            keys = {task.name, task.full_name, task.module_name}
            if task.legacy_alias:
                keys.add(task.legacy_alias)
            for key in keys:
                lookup[key.lower().replace("\\", "/")] = task
        return lookup

    @property
    def benchmark_tasks(self) -> tuple[TaskSpec, ...]:
        return self._benchmark_tasks

    @property
    def demos(self) -> tuple[TaskSpec, ...]:
        return self._demos

    def resolve(self, selector: str) -> TaskSpec:
        key = selector.strip().lower().replace("\\", "/")
        if key in self._lookup:
            return self._lookup[key]
        if "." in key:
            dotted_as_path = key.replace(".", "/", 1)
            if dotted_as_path in self._lookup:
                return self._lookup[dotted_as_path]
        raise TaskNotFoundError(
            f"Unknown task {selector!r}. Use `pace-bench list` to inspect valid IDs."
        )

    def select(
        self, selectors: list[str] | tuple[str, ...] | str
    ) -> tuple[TaskSpec, ...]:
        if isinstance(selectors, str):
            selectors = [selectors]
        selected: dict[str, TaskSpec] = {}
        for raw in selectors:
            for selector in (part.strip() for part in raw.split(",")):
                key = selector.lower()
                if key == "all":
                    for task in self._benchmark_tasks:
                        selected[task.full_name] = task
                    continue
                if key in {"demos", "demo"}:
                    for task in self._demos:
                        selected[task.full_name] = task
                    continue
                category_number = _parse_category_selector(key)
                if category_number is not None:
                    for task in self._benchmark_tasks:
                        if task.category_number == category_number:
                            selected[task.full_name] = task
                    continue
                task = self.resolve(selector)
                selected[task.full_name] = task
        return tuple(sorted(selected.values(), key=lambda item: item.full_name))

    def validate(self, task: TaskSpec, *, import_modules: bool = False) -> list[str]:
        errors: list[str] = []
        for filename in sorted(task.required_files):
            if not (task.path / filename).is_file():
                errors.append(f"missing {filename}")
        agent_path = task.path / "agent.py"
        if agent_path.is_file():
            try:
                tree = ast.parse(
                    agent_path.read_text(encoding="utf-8"), filename=str(agent_path)
                )
                functions = {
                    node.name for node in tree.body if isinstance(node, ast.FunctionDef)
                }
                for function in ("build_agent", "agent_action"):
                    if function not in functions:
                        errors.append(f"agent.py missing {function}()")
            except SyntaxError as exc:
                errors.append(f"agent.py syntax error: {exc}")
        if import_modules and not errors:
            for module_name in (
                "environment",
                "evaluator",
                "prompt",
                "feedback",
                "renderer",
            ):
                try:
                    self.load_module(task, module_name)
                except Exception as exc:  # validation must report every broken module
                    errors.append(f"cannot import {module_name}.py: {exc}")
        return errors

    def require_valid(self, task: TaskSpec) -> None:
        errors = self.validate(task)
        if errors:
            raise TaskContractError(f"{task.full_name}: " + "; ".join(errors))

    def load_module(self, task: TaskSpec, module_name: str) -> ModuleType:
        """Import a task module while isolating its legacy bare imports."""

        qualified = f"{task.module_name}.{module_name}"
        try:
            with _task_import_path(task.path):
                return importlib.import_module(qualified)
        except ModuleNotFoundError as exc:
            file_path = task.path / f"{module_name}.py"
            unique_name = (
                "pace_bench_dynamic_"
                + task.full_name.replace("/", "_")
                + f"_{module_name}"
            )
            spec = importlib.util.spec_from_file_location(unique_name, file_path)
            if spec is None or spec.loader is None:
                raise ImportError(f"Cannot load {file_path}") from exc
            module = importlib.util.module_from_spec(spec)
            with _task_import_path(task.path):
                spec.loader.exec_module(module)
            return module

    def environments(self, task: TaskSpec) -> tuple[EnvironmentSpec, ...]:
        if not task.benchmark:
            return (EnvironmentSpec(EnvironmentId("Initial"), "Initial Task"),)
        stages_module = self.load_stages(task)
        stage_function = next(
            (
                getattr(stages_module, name)
                for name in dir(stages_module)
                if "curriculum_stages" in name.lower()
                and callable(getattr(stages_module, name))
            ),
            None,
        )
        raw_stages = list(stage_function()) if stage_function else []
        environments = [
            EnvironmentSpec(
                environment_id=EnvironmentId("Initial"),
                title="Initial Task",
                terrain_config={"target_rng_seed": 123},
                physics_config={"do_sleep": False}
                if str(task.task_id) == "K_04"
                else {},
                raw={"stage_id": "Initial"},
            )
        ]
        seen = {"Initial"}
        expected_suffix = uniform_suffix_for_task(task.name)
        for raw in raw_stages:
            stage_id = str(raw.get("stage_id", "")).strip()
            if stage_id == "Base":
                continue
            environment_id = EnvironmentId(stage_id)
            if environment_id.value in seen:
                raise TaskContractError(
                    f"{task.full_name}: duplicate stage {environment_id}"
                )
            seen.add(environment_id.value)
            stage_suffix = str(raw.get("task_description_suffix") or "")
            if stage_suffix != expected_suffix:
                raise TaskContractError(
                    f"{task.full_name}/{environment_id}: task_description_suffix "
                    "must use uniform_suffix_for_task()"
                )
            terrain = dict(raw.get("terrain_config") or {})
            terrain.setdefault("target_rng_seed", 123)
            environments.append(
                EnvironmentSpec(
                    environment_id=environment_id,
                    title=str(raw.get("title") or environment_id),
                    terrain_config=terrain,
                    physics_config=dict(raw.get("physics_config") or {}),
                    task_description_suffix=stage_suffix,
                    raw=dict(raw),
                )
            )
        expected = {"Initial", "Stage-1", "Stage-2", "Stage-3", "Stage-4"}
        if seen != expected:
            raise TaskContractError(
                f"{task.full_name}: expected environments {sorted(expected)}, got {sorted(seen)}"
            )
        return tuple(
            sorted(environments, key=lambda env: env.environment_id.stage_number)
        )

    def load_stages(self, task: TaskSpec) -> ModuleType:
        stages_file = task.path / "stages.py"
        unique_name = "pace_bench_stages_" + task.full_name.replace("/", "_")
        module_spec = importlib.util.spec_from_file_location(unique_name, stages_file)
        if module_spec is None or module_spec.loader is None:
            raise ImportError(f"Cannot load stages module {stages_file}")
        module = importlib.util.module_from_spec(module_spec)
        with _task_import_path(task.path):
            module_spec.loader.exec_module(module)
        return module

    def adaptation_pairs(self, task: TaskSpec) -> tuple[EnvironmentPair, ...]:
        initial = EnvironmentId("Initial")
        return tuple(
            EnvironmentPair(initial, environment.environment_id)
            for environment in self.environments(task)
            if environment.environment_id != initial
        )


def _parse_category_selector(selector: str) -> int | None:
    normalized = selector.replace("-", "_").replace(" ", "_")
    if normalized.startswith("category_") and normalized[9:].isdigit():
        number = int(normalized[9:])
        return number if number in CATEGORIES else None
    for number, (directory, _, display_name) in CATEGORIES.items():
        if selector in {directory.lower(), display_name.lower()}:
            return number
    return None


@lru_cache(maxsize=1)
def get_registry() -> TaskRegistry:
    return TaskRegistry()


TASK_MAX_STEPS: dict[str, int] = {
    "S_03": 1800,
    "S_04": 20000,
    "S_05": 20000,
    "S_06": 15000,
    "K_01": 350000,
    "K_02": 20000,
    "K_03": 20000,
    "K_04": 60000,
    "K_05": 60000,
    "K_06": 150000,
    "D_03": 20000,
    "D_04": 15000,
    "D_06": 15000,
    "F_03": 2400,
    "F_05": 10000,
    "C_01": 20000,
    "C_02": 5000,
    "C_04": 250000,
    "C_05": 35000,
    "C_06": 15000,
    "E_01": 2500,
    "E_04": 12000,
    "E_06": 500,
}


def max_steps_for_task(task: TaskSpec) -> int:
    return TASK_MAX_STEPS.get(task.name, 10000)


def get_reference_solution(task: TaskSpec, environment: EnvironmentId | str) -> str:
    """Extract and normalize one environment's entry points from ``agent.py``."""

    environment_id = (
        environment
        if isinstance(environment, EnvironmentId)
        else EnvironmentId(environment)
    )
    agent_path = task.path / "agent.py"
    if environment_id.value == "Stage-3" and (task.path / "triple_scoop.py").is_file():
        agent_path = task.path / "triple_scoop.py"
    content = agent_path.read_text(encoding="utf-8")
    if environment_id.value == "Initial":
        build_function, action_function = "build_agent", "agent_action"
    else:
        number = environment_id.stage_number
        build_function = f"build_agent_stage_{number}"
        action_function = f"agent_action_stage_{number}"

    lines = content.splitlines()

    def is_stage_entry(name: str) -> bool:
        if name in {"build_agent", "agent_action"}:
            return True
        return any(
            name.startswith(prefix) and name[len(prefix) :].isdigit()
            for prefix in ("build_agent_stage_", "agent_action_stage_")
        )

    stage_functions = {
        line.split("(", 1)[0][4:].strip()
        for line in lines
        if line.startswith("def ") and is_stage_entry(line.split("(", 1)[0][4:].strip())
    }
    targets = {build_function, action_function}
    required_callees = {
        name
        for line in lines
        if not line.strip().startswith("def ")
        for name in stage_functions
        if name + "(" in line
    }
    excluded = stage_functions - targets - required_callees

    blocks: list[str] = []
    current: list[str] = []
    for line in lines:
        if line.strip() and not line.startswith((" ", "\t")) and current:
            blocks.append("\n".join(current))
            current = []
        current.append(line)
    if current:
        blocks.append("\n".join(current))

    imports: list[str] = []
    helpers: list[str] = []
    build = ""
    action = ""
    for block in blocks:
        if not block.strip():
            continue
        first = block.splitlines()[0]
        if first.startswith(("import ", "from ")):
            imports.append(block)
        elif first.startswith("def "):
            name = first.split("(", 1)[0][4:].strip()
            if name == build_function:
                build = block.replace(f"def {name}(", "def build_agent(", 1)
            elif name == action_function:
                action = block.replace(f"def {name}(", "def agent_action(", 1)
            elif name not in excluded:
                helpers.append(block)
        elif "__main__" not in first:
            helpers.append(block)
    if not build:
        raise ValueError(f"{agent_path} does not define {build_function}()")
    return "\n\n".join(
        part
        for part in ("\n".join(imports), "\n\n".join(helpers), build, action)
        if part
    )
