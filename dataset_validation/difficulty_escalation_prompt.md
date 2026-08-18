# Difficulty Escalation Prompt

Use this prompt after module auditing has established task correctness. Replace `{{TASK_ID}}`, `{{TASK_DIR}}`, `{{TARGET_STAGES}}`, `{{BASELINE_MODEL}}`, `{{TRIAL_COUNT}}`, and `{{REPOSITORY_ROOT}}` before use.

---

You are calibrating PACE-Bench task `{{TASK_ID}}` in `{{TASK_DIR}}`. The target environments are `{{TARGET_STAGES}}`; work from `{{REPOSITORY_ROOT}}`.

`{{BASELINE_MODEL}}` can solve the current mutations. Increase their physical difficulty aggressively while keeping every environment correct, stable, deterministic, and solvable by its stage-specific reference. Because PACE-Bench evaluates adaptation from the Initial solution, a central objective is to maximize the semantic solution distance from the Initial reference to each stage reference and among the stage references themselves. Difficulty must come from the intended adaptation problem, not ambiguity, hidden constraints, simulator instability, evaluator bugs, or unavailable APIs.

## Hard constraints

Preserve:

- Initial physics, prompt, evaluation, and reference success;
- task objective, scoring, simulation timing, maximum steps, seeds, primitives, feedback interface, and environment identities;
- Constraint / Visible / Invisible exposure rules;
- Initial-reference failure on every mutation and stage-reference success on every corresponding stage;
- deterministic, numerically healthy Box2D execution.

Do not add an objective or arbitrary restriction, shorten the time budget, weaken feedback, remove primitives, or alter scoring merely to force failure. Do not use crashes, NaNs, tunneling, impossible geometry, randomness, evaluator loopholes, or reference-only magic values as difficulty.

## Exposure and scope rules

- **Constraint:** expose the current numeric value; this priority overrides invisibility.
- **Visible:** expose directly observable geometry/state according to the task prompt contract.
- **Invisible:** never expose a non-constraint hidden value, its original value, direction, or an invertible comparison.

Changed Constraint/Visible values must be synchronized through the existing stage prompt-update mechanism and use target-plus-original wording where required. Invisible mutations remain numerically hidden.

All stages must share one identical `UNIFORM_SUFFIX` containing the union of generic variable names changed across Stage-1 through Stage-4, without values, directions, severity adjectives, or stage mappings.

Modify only existing physical variables and only `stages.py` plus the corresponding stage-specific functions in `agent.py`. Do not change Initial reference functions, `environment.py`, `evaluator.py`, `feedback.py`, `renderer.py`, or task-wide prompt semantics. If correct exposure cannot be achieved within `stages.py`, stop and request a separate module audit.

## Solution-similarity stopping criterion

Maximize the physical and algorithmic redesign required for every `Initial_to_Stage-N` adaptation:

- Each stage should make important parts of the Initial construction or controller unsuitable, forcing changes in mechanism, topology/geometry, primitive usage, load path, control policy, timing, or another task-relevant design dimension.
- Stage references should differ substantially from the Initial reference and, where physically possible, from one another. Later stages should generally require a larger departure from Initial than earlier stages.
- Prefer mutations that require a new reasoning strategy over mutations solved by uniformly scaling dimensions, forces, gains, or timing constants.
- Compare semantic solutions, not cosmetic code: renamed variables, reordered statements, comments, formatting, duplicated code, or arbitrary constants do not reduce meaningful similarity.
- Treat a single unchanged or lightly retuned solution that passes several stages as evidence that those stages may be under-differentiated. Use cross-stage reference transfer as a diagnostic, while preserving legitimate robustness when different environments genuinely share a solution.

After each escalation, compare the Initial and stage references. If their overall code structure, template, construction, or controller remains very similar and the main differences are numeric constants, continue escalating the environment and revising its reference. Repeat escalation, validation, and comparison until Initial-to-stage and pairwise stage solution similarity is relatively low. Do not stop by merely documenting high similarity.

Reduce similarity only within the task's existing objective, variables, primitives, visibility rules, and stable solvability. Never make code different for its own sake or encode privileged stage constants in solver-visible logic.

## Required process

### 1. Understand the current mechanism

Read all seven task modules and the successful baseline results. Identify:

- the physical mechanism each target stage tests;
- why the current reference and baseline succeed;
- the expected first failure after escalation;
- which Initial-solution mechanisms survive unchanged and which should become invalid;
- existing variables that can be changed, their exposure class, and stable hard ranges;
- how a legitimate stage reference can still solve the escalated environment;
- numerical-stability risks and corresponding checks.

Trace successful baseline candidates when possible. If success came from a prompt leak, generous threshold, scoring loophole, timing artifact, or numerical pathology, repair that correctness defect through module auditing before calibration.

### 2. Design a severe, monotonic escalation

Use a mechanism-level change rather than a small percentage tweak. Push existing parameters toward bounded extremes supported by stable simulation and a clear reference mechanism, and invalidate the successful strategy family rather than one exact program. Among valid alternatives, prefer the mutation that forces the largest semantic redesign from Initial and the clearest differentiation from other stage solutions.

Every stage may mutate one or multiple parameters. The overall challenge must increase monotonically from Stage-1 through Stage-4 (`Stage-1 < Stage-2 < Stage-3 < Stage-4`); do not weaken non-target stages. Later stages may combine conflicting demands when that matches the task's physical design.

Favor hard-to-infer Invisible changes when they fit the existing mutation mechanism, but never hide a Constraint or Visible value to create difficulty.

Check common shortcuts:

- maximum-size, maximum-force, constant-force, constant-torque, or fixed-timing strategies that ignore feedback;
- no-op, out-of-region, evaluator-timing, score-proxy, or hard-coded-renderer-coordinate exploits;
- blind API saturation without physical adaptation;
- NaN/infinity, destroyed tracked bodies, early sleeping, or collision tunneling;
- reference-specific constants that do not implement a legitimate physical solution.

### 3. Implement mutation and reference together

Update each target stage and its reference as one coherent change. The reference must implement the intended physical insight using documented primitives without privileged access to hidden state. Do not preserve the Initial or another stage's architecture merely for code reuse when the escalated physics should demand a different mechanism.

Synchronize stage attribute names and override timing, changed Constraint/Visible prompt values, and the identical `UNIFORM_SUFFIX`. Add only comments needed to explain non-obvious physics, and never leak hidden values into solver-visible text or code.

### 4. Validate correctness

Run:

```bash
python -m compileall "{{TASK_DIR}}"
pace-bench validate --task "{{TASK_ID}}" --contracts-only
pace-bench validate --task "{{TASK_ID}}"
pace-bench evaluate --task "{{TASK_ID}}" --env all \
  --provider mock --model mock --attempts 1 --dry-run
```

Inspect all five effective prompts. Only allowed Constraint/Visible values may appear; Invisible values and directions must not leak through prompt text, suffixes, examples, feedback, or errors.

Record the complete reference matrix:

- Initial reference passes Initial and fails Stage-1 through Stage-4 for genuine verification reasons;
- each stage-specific reference passes its corresponding environment;
- runs do not crash, time out from infrastructure, emit NaN/infinity, or leak pygame/Box2D resources;
- repeated runs under the same seed reproduce outcomes and scores.

Compare Initial with every stage reference and compare stage references pairwise. If similarity remains high—for example, the same structure/template with only constants changed—return to escalation, update the references, and rerun the entire validation. Continue until similarity is relatively low. Cross-stage reference transfer may help detect a near-universal lightly retuned solution, but legitimate robustness is not an automatic failure.

Repair every failed invariant before baseline calibration.

### 5. Calibrate with independent runs

Use the released prompt, feedback, attempt budget, retry policy, seed policy, and target environment. Run `{{TRIAL_COUNT}}` independent completed trials with `{{BASELINE_MODEL}}` for every target stage and keep the full result JSON.

Distinguish valid physics failures and exhausted solver-output retries from provider, credential, scheduling, timeout, or infrastructure failures. Only completed evaluation failures count as difficulty evidence.

If any trial succeeds, analyze its mechanism and repeat with a more fundamental escalation rather than perturbing only its constants. A stage is calibrated only when every required trial fails under the declared budget and its reference passes reliably.

### 6. Final review

Confirm monotonic Stage-1-to-Stage-4 difficulty, unchanged Initial behavior, consistency of non-target stages, correct exposure, diagnostic non-prescriptive feedback, and absence of generated results, credentials, host paths, ports, or GPU assignments in source changes.

## Required final report

Return:

1. expected failure mechanism and physical rationale;
2. old/new parameter values and exposure classifications;
3. source changes and preserved semantics;
4. five-environment prompt matrix and suffix audit;
5. complete reference matrix with scores and failure reasons;
6. every baseline trial with model revision, seed, budget, outcome, and result path;
7. determinism, numerical-health, shortcut, and residual-risk findings.

Do not claim successful escalation unless reference validation and the declared independent baseline trials were executed.
