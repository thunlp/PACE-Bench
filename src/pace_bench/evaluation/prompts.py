"""Task-aware vanilla prompt construction and packaged demonstrations."""

from __future__ import annotations

import inspect
import re
from collections.abc import Callable
from functools import cache
from pathlib import Path
from typing import Any

from pace_bench.tasks.registry import TaskRegistry, get_registry
from pace_bench.tasks.registry import EnvironmentSpec, TaskSpec


@cache
def _read_demonstration(name: str) -> str:
    return (Path(__file__).with_name("prompt_data") / name).read_text(encoding="utf-8")


def initial_demonstration() -> str:
    return _read_demonstration("initial_demonstration.md")


def revision_demonstration() -> str:
    return _read_demonstration("revision_demonstration.md")


def adaptation_setting() -> str:
    return _read_demonstration("adaptation_setting.md")


def adaptation_demonstration() -> str:
    return _read_demonstration("adaptation_demonstration.md")


def with_prompt_trailer(prompt: str, task_prompt: dict[str, Any]) -> str:
    trailer = str(task_prompt.get("prompt_trailer") or "").strip()
    if not trailer:
        return prompt
    return prompt.rstrip() + "\n\n" + trailer + "\n"


def format_initial_prompt(task_prompt: dict[str, Any]) -> str:
    prompt = f"""# Task Description

{task_prompt["task_description"]}

# Success Criteria

{task_prompt["success_criteria"]}

# Available Primitives API

{task_prompt["primitives_api"]}

{initial_demonstration()}

# Your Task

You are designing a physical system in a 2D physics simulation. Before writing code, you MUST reason through the physical design.

## Step 1: Physical Analysis (Required)

1. **Understand the Physics**: What physical principles govern this task? (equilibrium, kinematics, dynamics, energy, fluid interaction, etc.)

2. **Design Strategy**: How will your structure/mechanism achieve the goal? What is the key physical insight?

3. **Parameter Reasoning**: Estimate key parameters (dimensions, masses, forces, speeds) based on physical reasoning.

## Step 2: Write Code

**Code Requirements**:
- All code must be inside functions
- Do not use `sandbox` variable outside functions

**Output Format**:

```python
def build_agent(sandbox):
    # Your implementation
    return chassis

def agent_action(sandbox, agent_body, step_count):
    # Control logic if needed
    pass
```

Begin with your physical analysis, then provide the code.
"""
    return with_prompt_trailer(prompt, task_prompt)


def format_revision_prompt(
    task_prompt: dict[str, Any],
    best_code: str,
    best_feedback: str,
    previous_code: str,
    previous_feedback: str,
    *,
    best_attempt: int | None,
    previous_attempt: int | None,
) -> str:
    show_previous = bool(previous_code) and (
        best_attempt != previous_attempt
        if best_attempt is not None and previous_attempt is not None
        else True
    )
    previous_section = ""
    if show_previous:
        previous_section = f"""
# Previous Attempt

```python
{previous_code}
```

Feedback: {previous_feedback}
"""
    prompt = f"""# Task Description

{task_prompt["task_description"]}

# Success Criteria

{task_prompt["success_criteria"]}

# Available Primitives API

{task_prompt["primitives_api"]}

{revision_demonstration()}

# Best-Scoring Attempt (Reference)

```python
{best_code}
```

Feedback: {best_feedback}
{previous_section}


# Your Task: Diagnose and Fix

Compare these attempts. Learn from what worked best and what changed recently. Provide an improved solution.

**Output Format**:

```python
def build_agent(sandbox):
    # Your improved implementation
    return chassis

def agent_action(sandbox, agent_body, step_count):
    # Control logic if needed
    pass
```

Begin with your analysis, then provide the code.
"""
    return with_prompt_trailer(prompt, task_prompt)


def format_adaptation_revision_prompt(
    task_prompt: dict[str, Any],
    reference_code: str,
    reference_feedback: str,
    best_code: str,
    best_feedback: str,
    previous_code: str,
    previous_feedback: str,
    *,
    best_attempt: int | None,
    previous_attempt: int | None,
) -> str:
    show_previous = bool(previous_code) and (
        best_attempt != previous_attempt
        if best_attempt is not None and previous_attempt is not None
        else True
    )
    previous_section = ""
    if show_previous:
        previous_section = f"""
# Previous Attempt

```python
{previous_code}
```

Feedback: {previous_feedback}
"""
    prompt = f"""{adaptation_setting()}

# Task Description

{task_prompt["task_description"]}

# Success Criteria

{task_prompt["success_criteria"]}

# Available Primitives API

{task_prompt["primitives_api"]}

{adaptation_demonstration()}

# ⚠️ CRITICAL: The Physical Environment Has Changed

The physics environment has been modified. Your previously successful design NO LONGER WORKS.

# Previous Successful Code (worked in the Original Environment)

```python
{reference_code}
```

# Feedback from Running in the NEW Environment (Iteration 1)

{reference_feedback}

You must **infer what changed** from the feedback above and adapt.
# Best-Scoring Attempt So Far (in New Environment)

```python
{best_code}
```

Feedback: {best_feedback}
{previous_section}

# Your Task: Diagnose and Adapt

Compare these attempts. Learn from what worked best and what changed recently. Provide an improved adapted solution.

**Output Format**:

```python
def build_agent(sandbox):
    # Your improved implementation
    return chassis

def agent_action(sandbox, agent_body, step_count):
    # Control logic if needed
    pass
```

Begin with your analysis, then provide the code.
"""
    return with_prompt_trailer(prompt, task_prompt)


class PromptBuilder:
    """Load task context without exposing invisible mutated values."""

    def __init__(self, registry: TaskRegistry | None = None) -> None:
        self.registry = registry or get_registry()

    def load_task_context(
        self,
        task: TaskSpec,
        target: EnvironmentSpec | None = None,
        *,
        include_source_comparison: bool = True,
    ) -> dict[str, Any]:
        prompt_module = self.registry.load_module(task, "prompt")
        context = dict(prompt_module.TASK_PROMPT)
        if target is None or target.environment_id.value == "Initial":
            return context
        stages_module = self.registry.load_stages(task)
        terrain = dict(target.terrain_config)
        physics = dict(target.physics_config)
        raw_stage = dict(target.raw)
        description = str(context.get("task_description") or "")
        criteria = str(context.get("success_criteria") or "")
        for name in dir(stages_module):
            value = getattr(stages_module, name)
            if not callable(value):
                continue
            lowered = name.lower()
            if "update_task_description_for_visible_changes" in lowered:
                description = _call_update_function(
                    value, description, terrain, physics, raw_stage
                )
            elif "update_success_criteria_for_visible_changes" in lowered:
                criteria = _call_update_function(
                    value, criteria, terrain, physics, raw_stage
                )
        context["task_description"] = description
        context["success_criteria"] = criteria
        if target.task_description_suffix:
            context["prompt_trailer"] = target.task_description_suffix
        else:
            context.pop("prompt_trailer", None)
        if not include_source_comparison:
            for key in ("task_description", "success_criteria", "prompt_trailer"):
                if key in context:
                    context[key] = _remove_source_comparisons(str(context[key]))
        return context

    def initial(self, task_context: dict[str, Any]) -> str:
        return format_initial_prompt(task_context)

    def revision(
        self,
        task_context: dict[str, Any],
        *,
        best_code: str,
        best_feedback: str,
        previous_code: str,
        previous_feedback: str,
        best_attempt: int | None,
        previous_attempt: int | None,
    ) -> str:
        return format_revision_prompt(
            task_context,
            best_code,
            best_feedback,
            previous_code,
            previous_feedback,
            best_attempt=best_attempt,
            previous_attempt=previous_attempt,
        )

    def adaptation_revision(
        self,
        task_context: dict[str, Any],
        *,
        reference_code: str,
        reference_feedback: str,
        best_code: str,
        best_feedback: str,
        previous_code: str,
        previous_feedback: str,
        best_attempt: int | None,
        previous_attempt: int | None,
    ) -> str:
        return format_adaptation_revision_prompt(
            task_context,
            reference_code,
            reference_feedback,
            best_code,
            best_feedback,
            previous_code,
            previous_feedback,
            best_attempt=best_attempt,
            previous_attempt=previous_attempt,
        )


def _call_update_function(
    function: Callable[..., str],
    base_text: str,
    terrain: dict[str, Any],
    physics: dict[str, Any],
    stage: dict[str, Any],
) -> str:
    parameters = inspect.signature(function).parameters
    positional = sum(
        parameter.kind
        in (inspect.Parameter.POSITIONAL_ONLY, inspect.Parameter.POSITIONAL_OR_KEYWORD)
        for parameter in parameters.values()
    )
    keyword_arguments = (
        {"stage": stage}
        if "stage" in parameters
        and parameters["stage"].kind == inspect.Parameter.KEYWORD_ONLY
        else {}
    )
    values: list[Any] = [base_text, terrain, {}, physics, {}]
    return function(*values[:positional], **keyword_arguments)


def _remove_source_comparisons(text: str) -> str:
    """Remove transition-only annotations while retaining current target values."""

    def clean_parentheses(
        value: str, start: int = 0, *, stop_at_close: bool = False
    ) -> tuple[str, int]:
        pieces: list[str] = []
        index = start
        while index < len(value):
            character = value[index]
            if character == ")":
                if stop_at_close:
                    return "".join(pieces), index + 1
                pieces.append(character)
                index += 1
                continue
            if character != "(":
                pieces.append(character)
                index += 1
                continue
            inner, index = clean_parentheses(value, index + 1, stop_at_close=True)
            lowered = inner.lower()
            marker = lowered.find("originally")
            if marker < 0:
                pieces.append(f"({inner})")
                continue
            prefix = inner[:marker].rstrip(" ;,")
            if prefix:
                pieces.append(f"({prefix})")
        return "".join(pieces), index

    cleaned, _ = clean_parentheses(text)
    cleaned = re.sub(
        r"(?i)(?:[;,]\s*|\.\s+)originally\b[^\n]*(?:source environment|source env)\.?",
        ".",
        cleaned,
    )
    cleaned = re.sub(
        r"(?i)\b(?:the\s+)?source environment\b|\bsource env\b",
        "current environment",
        cleaned,
    )
    cleaned = re.sub(r"[ \t]+([,.;:)])", r"\1", cleaned)
    cleaned = re.sub(r"\.{2,}", ".", cleaned)
    return cleaned
