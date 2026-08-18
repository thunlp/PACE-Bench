"""Static checks and isolated candidate-module execution."""

from __future__ import annotations

import builtins
import importlib.util
import math
import os
import re


_COMPLETE_FENCE = re.compile(r"```(?:python)?\s*\n?(.*?)```", re.DOTALL | re.IGNORECASE)
_INCOMPLETE_FENCE = re.compile(r"```(?:python)?\s*\n?(.*)", re.DOTALL | re.IGNORECASE)
_ALLOWED_IMPORTS = {"Box2D", "math", "random"}
_PROHIBITED_CALLS = {
    "breakpoint",
    "compile",
    "eval",
    "exec",
    "globals",
    "help",
    "input",
    "locals",
    "open",
    "vars",
    "__import__",
}
_PROHIBITED_NAMES = {
    "__builtins__",
    "__file__",
    "__loader__",
    "__package__",
    "__spec__",
}
_PROHIBITED_ATTRIBUTES = {
    "__bases__",
    "__class__",
    "__closure__",
    "__code__",
    "__dict__",
    "__func__",
    "__getattribute__",
    "__globals__",
    "__loader__",
    "__module__",
    "__mro__",
    "__reduce__",
    "__reduce_ex__",
    "__self__",
    "__subclasses__",
}


class ProhibitedOperationError(Exception):
    """Raised when generated code bypasses documented sandbox primitives."""


def extract_code(raw_text: str | None) -> str:
    """Extract the longest fenced block, or a substantial bare build function."""

    if raw_text is None:
        return ""
    text = raw_text
    for marker in ("<|im_start|>assistant", "<|assistant|>"):
        position = text.rfind(marker)
        if position >= 0:
            text = text[position + len(marker) :]
            break
    text = re.sub(r"<\|im_start\|>.*?<\|im_end\|>", "", text, flags=re.DOTALL)
    text = re.sub(r"<\|im_(?:start|end)\|>\w*", "", text).strip()
    closing_think = text.rfind("</think>")
    if closing_think >= 0:
        text = text[closing_think + len("</think>") :].strip()
    matches = list(_COMPLETE_FENCE.finditer(text))
    if matches:
        return (
            max(matches, key=lambda match: len(match.group(1).strip())).group(1).strip()
        )
    incomplete = _INCOMPLETE_FENCE.search(text)
    if incomplete:
        candidate = re.sub(r"```.*$", "", incomplete.group(1), flags=re.DOTALL).strip()
        if len(candidate) >= 50:
            return candidate
    match = re.search(r"def\s+build_agent\s*\(.*", text, re.DOTALL)
    if match:
        candidate = re.sub(r"<\|.*?\|>.*$", "", text[match.start() :], flags=re.DOTALL)
        if len(candidate.strip()) >= 50:
            return candidate.strip()
    return ""


def validate_solver_output(
    raw_text: str | None, code: str | None = None
) -> tuple[bool, str]:
    """Classify only structurally unusable generations as solver failures."""

    if raw_text is None:
        return False, "provider returned no response"
    candidate = code if code is not None else extract_code(raw_text)
    if len("".join(candidate.split())) < 50:
        return False, "extracted output is unusably short"
    if not re.search(r"\bdef\s+build_agent\s*\(", candidate):
        return False, "output is missing build_agent()"
    return True, ""


class CodeSafetyMixin:
    def _load_allowed_apis(self, task) -> set:
        """Load the APIs exposed by a benchmark task or tutorial demo."""
        import json
        import re

        allowed = {
            "build_agent",
            "agent_action",
            "math",
            "np",
            "numpy",
        }  # Standard allowed symbols

        try:
            data_root = self.registry.categories_root
            api_file = data_root / "primitives_api.json"
            if os.path.exists(api_file):
                with open(api_file) as f:
                    api_data = json.load(f)

                # Get task key (e.g., 'S_01') from task_path (e.g., 'Category1_Statics_Equilibrium/S_01')
                task_key = task.name

                if task_key in api_data:
                    task_apis = api_data[task_key]
                    for api_desc in task_apis.values():
                        # Extract all 'sandbox.method_name' or 'sandbox.attribute' from the documentation string
                        # Support both lowercase and uppercase (for constants)
                        matches = re.findall(r"sandbox\.([a-zA-Z0-9_]+)", api_desc)
                        for m in matches:
                            allowed.add(m)

                # Benchmark tasks use the centralized API manifest. The standalone
                # demo is deliberately smaller and documents its primitives in its
                # own prompt, so derive the same allow-list from that public text.
                if not task.benchmark:
                    prompt_module = self.registry.load_module(task, "prompt")
                    prompt_data = getattr(prompt_module, "TASK_PROMPT", {})
                    primitives = str(prompt_data.get("primitives_api", ""))
                    allowed.update(re.findall(r"sandbox\.([a-zA-Z0-9_]+)", primitives))

        except Exception as e:
            print(f"Warning: Failed to load allowed APIs: {e}")

        return allowed

    @staticmethod
    def _object_has_world_position(obj) -> bool:
        if obj is None:
            return False
        pos = getattr(obj, "position", None)
        return pos is not None and hasattr(pos, "x") and hasattr(pos, "y")

    @classmethod
    def _primary_physics_object(cls, agent_components):
        """Body or body-like object for camera, stuck detection, and evaluator.evaluate (never a raw list/tuple)."""
        if agent_components is None:
            return None
        if isinstance(agent_components, dict):
            for key in ("arm", "sensor", "chassis", "body", "agent"):
                o = agent_components.get(key)
                if cls._object_has_world_position(o):
                    return o
            return None
        if isinstance(agent_components, list | tuple):
            for item in agent_components:
                if cls._object_has_world_position(item):
                    return item
            return None
        return (
            agent_components
            if cls._object_has_world_position(agent_components)
            else None
        )

    @staticmethod
    def _safe_world_xy(obj) -> tuple[float, float] | None:
        if not CodeSafetyMixin._object_has_world_position(obj):
            return None
        try:
            p = obj.position
            return float(p.x), float(p.y)
        except (AttributeError, TypeError, ValueError):
            return None

    @staticmethod
    def _safe_linear_speed(obj) -> float | None:
        if obj is None:
            return None
        try:
            lv = obj.linearVelocity
            return math.sqrt(float(lv.x) ** 2 + float(lv.y) ** 2)
        except (AttributeError, TypeError, ValueError):
            return None

    def _check_prohibited_operations(self, code: str):
        """
        Check for prohibited operations in the agent code using AST-based static analysis.
        1. Enforce task-specific API usage from primitives_api.json.
        2. Prohibit direct assignment to environmental variables.
        3. Allow modification of physical properties (e.g., linearVelocity, friction)
           ONLY for dynamically created tools (e.g., those from sandbox.add_box).
        """
        import ast

        try:
            tree = ast.parse(code)
        except SyntaxError:
            # Let _execute_code handle syntax errors with more context
            return

        self._check_python_runtime_access(tree)

        prohibited_attrs = {
            "linearVelocity",
            "angularVelocity",
            "position",
            "angle",
            "friction",
            "density",
            "restitution",
        }
        agent_tools = set()  # Track variables that represent dynamically created tools

        def _rooted_at_sandbox(expression) -> bool:
            while isinstance(expression, (ast.Attribute, ast.Subscript)):
                expression = expression.value
            return isinstance(expression, ast.Name) and expression.id == "sandbox"

        for node in ast.walk(tree):
            # 0. Track variables created via sandbox creation APIs (e.g., v = sandbox.add_box(...))
            if (
                isinstance(node, ast.Assign)
                and isinstance(node.value, ast.Call)
                and isinstance(node.value.func, ast.Attribute)
                and isinstance(node.value.func.value, ast.Name)
                and node.value.func.value.id == "sandbox"
                and node.value.func.attr.startswith(("add_", "create_"))
            ):
                for target in node.targets:
                    if isinstance(target, ast.Name):
                        agent_tools.add(target.id)

            # 1. Check for attribute access on 'sandbox'
            if isinstance(node, ast.Attribute):
                if isinstance(node.ctx, ast.Store) and _rooted_at_sandbox(node):
                    raise ProhibitedOperationError(
                        "Prohibited operation: candidate code cannot assign to "
                        "sandbox state, environment configuration, or world properties."
                    )

                # Check if it's an access to sandbox (e.g., sandbox.add_beam)
                if isinstance(node.value, ast.Name) and node.value.id == "sandbox":
                    api_name = node.attr
                    # Check if the API is allowed for this task
                    if (
                        api_name.lower() not in self.allowed_apis
                        and api_name not in self.allowed_apis
                    ):
                        raise ProhibitedOperationError(
                            f"Prohibited API usage: 'sandbox.{api_name}' is NOT allowed for this task. "
                            f"You are restricted to the documented Primitives API."
                        )

                # 2. Prohibit direct assignment to physical state variables
                # ALLOWED if the object is an agent-created tool (tracked in agent_tools)
                # PROHIBITED if the object is environment-derived or unknown
                if isinstance(node.ctx, ast.Store) and node.attr in prohibited_attrs:
                    is_tool = False
                    if (
                        isinstance(node.value, ast.Name)
                        and node.value.id in agent_tools
                    ):
                        is_tool = True

                    # Specifically check if they are trying to modify environment via sandbox._terrain_bodies
                    is_env = False
                    if (
                        isinstance(node.value, ast.Attribute)
                        and node.value.attr == "_terrain_bodies"
                    ) or (
                        isinstance(node.value, ast.Subscript)
                        and isinstance(node.value.value, ast.Attribute)
                        and node.value.value.attr == "_terrain_bodies"
                    ):
                        is_env = True

                    if is_env:
                        raise ProhibitedOperationError(
                            "Prohibited operation: Directly modifying physics properties of environment objects (via _terrain_bodies) is STRICTLY PROHIBITED."
                        )

                    if not is_tool:
                        raise ProhibitedOperationError(
                            f"Prohibited operation detected: Direct assignment to '{node.attr}' is ONLY allowed for dynamically created tools. "
                            f"For environmental objects, you must use documented APIs (e.g., ApplyForce, ApplyTorque) "
                            f"instead of direct state manipulation."
                        )

            # 3. Prohibit direct assignment to elements of _terrain_bodies (e.g., sandbox._terrain_bodies['core'] = ...)
            if (
                isinstance(node, ast.Subscript)
                and isinstance(node.ctx, ast.Store)
                and _rooted_at_sandbox(node)
            ):
                raise ProhibitedOperationError(
                    "Prohibited operation: candidate code cannot modify a "
                    "sandbox-owned mapping, environment configuration, or world state."
                )

            if (
                isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr.startswith(
                    (
                        "Create",
                        "Destroy",
                        "Step",
                        "ClearForces",
                        "ShiftOrigin",
                    )
                )
                and _rooted_at_sandbox(node.func.value)
            ):
                raise ProhibitedOperationError(
                    "Prohibited operation: mutate the physics world only through "
                    "the documented task primitives."
                )

            # 4. Handle augmented assignments (e.g., body.linearVelocity += ...)
            if isinstance(node, ast.AugAssign):
                target = node.target
                if (
                    isinstance(target, ast.Attribute)
                    and target.attr in prohibited_attrs
                ):
                    is_tool = False
                    if (
                        isinstance(target.value, ast.Name)
                        and target.value.id in agent_tools
                    ):
                        is_tool = True

                    if not is_tool:
                        raise ProhibitedOperationError(
                            f"Prohibited operation detected: Direct modification of '{target.attr}' via augmented assignment is ONLY allowed for tools."
                        )

    @staticmethod
    def _check_python_runtime_access(tree) -> None:
        """Prevent candidate code from turning feedback into a host-inspection API."""

        import ast

        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                modules = {alias.name.split(".", 1)[0] for alias in node.names}
                denied = modules - _ALLOWED_IMPORTS
                if denied:
                    raise ProhibitedOperationError(
                        "Prohibited import(s): " + ", ".join(sorted(denied))
                    )
            elif isinstance(node, ast.ImportFrom):
                root = (node.module or "").split(".", 1)[0]
                if node.level or root not in _ALLOWED_IMPORTS:
                    raise ProhibitedOperationError(
                        f"Prohibited import: {node.module or '<relative>'}"
                    )
            elif isinstance(node, ast.Call) and isinstance(node.func, ast.Name):
                if node.func.id in _PROHIBITED_CALLS:
                    raise ProhibitedOperationError(
                        f"Prohibited Python runtime call: {node.func.id}()"
                    )
                if node.func.id in {"getattr", "hasattr", "setattr", "delattr"}:
                    if len(node.args) >= 2 and isinstance(node.args[1], ast.Constant):
                        attribute = node.args[1].value
                        if (
                            isinstance(attribute, str)
                            and attribute in _PROHIBITED_ATTRIBUTES
                        ):
                            raise ProhibitedOperationError(
                                f"Prohibited introspection attribute: {attribute}"
                            )
            elif isinstance(node, ast.Name) and node.id in _PROHIBITED_NAMES:
                raise ProhibitedOperationError(
                    f"Prohibited Python runtime name: {node.id}"
                )
            elif (
                isinstance(node, ast.Attribute) and node.attr in _PROHIBITED_ATTRIBUTES
            ):
                raise ProhibitedOperationError(
                    f"Prohibited introspection attribute: {node.attr}"
                )
            elif isinstance(node, ast.Constant) and isinstance(node.value, str):
                if "__" in node.value and node.value != "__name__":
                    raise ProhibitedOperationError(
                        "Prohibited dunder introspection string"
                    )

    def _execute_code(self, code: str):
        """Execute code and return module object"""
        # First perform syntax check
        try:
            compile(code, "<string>", "exec")
        except SyntaxError as e:
            # Include more context in the error message
            error_lines = code.split("\n")
            error_line_num = e.lineno or 0
            context_start = max(0, error_line_num - 3)
            context_end = min(len(error_lines), error_line_num + 3)
            context = "\n".join(
                f"{i + 1:4d}: {line}"
                for i, line in enumerate(
                    error_lines[context_start:context_end], start=context_start
                )
            )
            raise SyntaxError(
                f"Code syntax error: {e}\nCode snippet (lines {context_start + 1}-{context_end}):\n{context}\n\nFull error: {e}"
            ) from e

        # Create temporary module
        spec = importlib.util.spec_from_loader("solver_code", loader=None)
        code_module = importlib.util.module_from_spec(spec)
        safe_builtins = dict(vars(builtins))
        for name in _PROHIBITED_CALLS:
            safe_builtins.pop(name, None)

        def limited_import(
            name: str,
            globals_: dict | None = None,
            locals_: dict | None = None,
            fromlist: tuple | list = (),
            level: int = 0,
        ):
            root = name.split(".", 1)[0]
            if level or root not in _ALLOWED_IMPORTS:
                raise ImportError(f"Import of {name!r} is not allowed")
            return builtins.__import__(name, globals_, locals_, fromlist, level)

        safe_builtins["__import__"] = limited_import
        code_module.__dict__["__builtins__"] = safe_builtins

        # Execute code (in isolated namespace)
        try:
            exec(code, code_module.__dict__)
        except NameError as e:
            # Check if it's a sandbox-related error
            error_msg = str(e)
            if "sandbox" in error_msg.lower():
                raise NameError(
                    f"Code references undefined variable 'sandbox'."
                    f"Please ensure all references to sandbox are inside functions (build_agent or agent_action functions)."
                    f"\nOriginal error: {e}"
                    f"\nCode:\n{code}"
                ) from e
            raise
        except Exception as e:
            raise RuntimeError(f"Error executing code: {e}\nCode:\n{code}") from e

        return code_module
