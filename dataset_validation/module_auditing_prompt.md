# Module Auditing Prompt

Use this prompt to audit one PACE-Bench task across all five environments. Replace `{{TASK_ID}}`, `{{TASK_DIR}}`, and `{{REPOSITORY_ROOT}}` before giving it to a coding agent.

---

You are performing a strict, evidence-based module audit of PACE-Bench task `{{TASK_ID}}` in `{{TASK_DIR}}`. Work from `{{REPOSITORY_ROOT}}` and fix every confirmed defect rather than stopping after the first few findings.

Inspect the implementation, construct the effective prompt for every environment, exercise the public evaluation path, and validate every reference solution. Do not declare the task clean from static inspection alone.

## Non-negotiable invariants

Preserve:

- task physics, constants, mutation mechanisms, constraints, success criteria, and scoring semantics;
- Box2D timing, maximum-step behavior, seeds, action order, and cleanup;
- `build_agent(sandbox)` and `agent_action(sandbox, agent_body, step_count)`;
- the Constraint / Visible / Invisible distinction and canonical environment identities;
- reference-solution behavior and the feedback interface.

Make the smallest coherent repair. Do not tune difficulty, weaken the evaluator, or make a reference pass through a scoring loophole.

## Variable-exposure rules

Classify each quantity by its role in this task and environment, not solely by its name:

| Class | Meaning | Prompt rule |
| --- | --- | --- |
| Constraint | A hard absolute limit or mandatory success condition | State the current numeric limit. Constraint takes priority over every other class. |
| Visible | Geometry or state directly observable in the scene | State the current numeric value when the prompt exposes measurements. |
| Invisible | A non-constraint environmental constant that is not directly observable | Never reveal its numeric value, original value, change direction, or an invertible comparison. |

For example, a force cap is a Constraint and must be exposed, while an unobservable friction coefficient that is not a constraint remains Invisible. `_CE` stages intentionally expose changes and are not violations.

When a mutation changes a Constraint or Visible value, use the repository's established target-plus-original wording where required, such as `new_value (originally old_value)`. Never use this wording for Invisible non-constraints.

Every stage must use the same `UNIFORM_SUFFIX`. It must contain the union of generic variable names changed anywhere in Stage-1 through Stage-4, without values, directions, severity, or stage-to-variable mappings.

## Required audit

### 1. Establish the module and runtime contract

Read `agent.py`, `environment.py`, `evaluator.py`, `feedback.py`, `prompt.py`, `renderer.py`, and `stages.py` completely. Trace shared prompt, simulator, verification, and diagnostic code only where needed.

Confirm that:

- required modules and Initial/Stage reference entry points import through the public runtime;
- prompt, stage, evaluator, feedback, renderer, and candidate APIs match their callers;
- imports do not initialize displays, GPUs, networks, or models;
- task code does not depend on host paths, stale artifacts, or mutable cross-run state;
- renderer behavior cannot affect physics or scoring;
- failures are surfaced instead of silently swallowed.

### 2. Inventory variables and inspect all effective prompts

Build a compact inventory of every quantity affecting physics, evaluation, feedback, or prompt text. Record its canonical name and aliases, source/default, stage overrides, classification, units, and all consuming modules. Use it to find misspellings, shadow attributes, stale copies, wrong units, and overrides applied after a value is consumed.

Construct the final prompt through the real evaluation path for `Initial` and Stage-1 through Stage-4. For each environment verify:

- every Constraint and success threshold is complete, current, unambiguous, and equal to the evaluator value;
- required Visible geometry matches the simulated world;
- Invisible values, original values, directions, ratios, and invertible derived values are absent;
- changed Constraint/Visible values use the correct target and original values;
- unchanged facts are not presented as mutations;
- primitive names and signatures match the sandbox API;
- instructions reveal neither reference construction details nor a prescribed solution;
- all four stages contain an identical, valid `UNIFORM_SUFFIX`.

For every regex or string replacement used by `update_task_description_for_visible_changes`, execute the updater against the exact base description through the real prompt path. Use `re.subn()` or an equivalent explicit match count and require exactly the intended number of replacements, normally one per field; zero matches and unintended multiple matches are violations. Test Stage-1 through Stage-4 independently and assert that the final prompt contains the target value plus `(originally old_value)`, removes the stale unannotated value, leaves unrelated text unchanged, and never inserts a numeric Invisible non-constraint. Returning without an exception is not evidence that a replacement worked.

Perform a two-way numeric sweep: trace every number in each effective prompt to an allowed Constraint, Visible measurement, budget, or API constant, then confirm that every required Constraint and Visible value appears. Do not delete a value merely because a similarly named variable is Invisible in another task.

### 3. Trace cross-module behavior

For every success condition and mutation, trace:

`stage override -> environment/world -> evaluator metric -> pass/fail and score -> feedback`

Verify comparison direction, strictness, units, coordinate frames, time windows, and aggregation. Ensure stage-dependent limits come from live state rather than duplicated constants, and every feedback metric comes from the evaluator or additive tracking that cannot change simulation behavior.

Feedback must report what happened and the margin to success without revealing hidden constants or suggesting a design. Renderer geometry and labels must match the simulated objects while remaining evaluation-neutral.

Check empty constructions, missing actions, early destruction, NaN/infinity, sleeping bodies, identity mismatches, boundary equality, and apparent success after a hard-constraint violation.

### 4. Audit stages and references

Confirm that:

- mutations update intended existing variables at the correct lifecycle point and do not compound across runs;
- stage identifiers, prompt updates, evaluator behavior, and reference entry points agree;
- all stages are deterministic and maintain the intended Stage-1-to-Stage-4 difficulty progression;
- the Initial reference passes `Initial` and fails all four mutations for genuine verification reasons;
- each stage-specific reference passes its own stage through the public verifier;
- no expected failure is actually an import error, crash, timeout, unavailable primitive, or infrastructure failure.

Inspect a simulation trace when a result is surprising. A passing reference does not excuse a prompt leak or evaluator loophole.

### 5. Repair and validate

List confirmed defects with evidence and affected invariants, then make the minimal synchronized edits. Document intentional behavior and leave it unchanged when the evidence does not support a repair.

Run at least:

```bash
python -m compileall "{{TASK_DIR}}"
pace-bench validate --task "{{TASK_ID}}" --contracts-only
pace-bench validate --task "{{TASK_ID}}"
pace-bench evaluate --task "{{TASK_ID}}" --env all \
  --provider mock --model mock --attempts 1 --dry-run
```

Use headless SDL variables where required, exercise prompt construction, and run a representative verifier call through the installed CLI. Re-run affected checks after every repair. Infrastructure failures are not evidence of correct benchmark behavior.

This is an iterative audit, not a single inspection. After any revision, restart the complete audit from Step 1 against the updated files, including all five prompts, regex replacement counts, cross-module traces, references, and runtime checks. If the new inspection finds any error, fix it and restart again. Stop only when a fresh complete audit performed after the last revision finds zero errors, requires no further edits, and passes every validation command; an inspection that made changes cannot itself be the final clean pass.

## Required final report

Return:

1. verdict: `CLEAN` or `REPAIRED`;
2. variable inventory, five-environment exposure matrix, and regex/replacement match-count matrix;
3. every defect, root cause, and file changed;
4. cross-module traces for success conditions and mutations;
5. actual Initial/stage reference matrix with scores or failure reasons;
6. exact validation commands and results;
7. unresolved ambiguities or risks.

Do not claim `CLEAN` or `REPAIRED` unless executed evidence supports it.
