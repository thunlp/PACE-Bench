# Feedback Design Prompt

Use this prompt after module auditing. Replace `{{TASK_ID}}`, `{{TASK_DIR}}`, `{{RESULT_PATHS}}`, and `{{REPOSITORY_ROOT}}` before use; `{{RESULT_PATHS}}` should cover representative failed runs from every mutated stage.

---

You are improving diagnostic feedback for PACE-Bench task `{{TASK_ID}}` in `{{TASK_DIR}}`. Work from `{{REPOSITORY_ROOT}}` and analyze `{{RESULT_PATHS}}`.

The feedback must explain what happened, when and where failure developed, how close the run was to success, and in what order events occurred. It must not prescribe a design. Preserve physics, pass/fail, score, thresholds, timing, seeds, attempt accounting, public APIs, and reference behavior.

## Feedback principles

Every reported quantity must be:

- **measured:** sourced from the evaluator or additive observational tracking, never inferred from score alone;
- **task-grounded:** tied to an objective, constraint, or physical failure mode;
- **threshold-aware:** compared with the live evaluator target/limit;
- **diagnostic, not prescriptive:** report state and deficit without suggesting geometry, control logic, primitives, or parameter values;
- **phase-aware:** distinguish failure onset and evolution when timing matters;
- **compact:** prioritize high-information signals without cross-section repetition;
- **safe:** never reveal Invisible values, mutation directions, reference code, or privileged state.

For example, “The payload cleared 42% of the required distance before lateral slip began at step 118” is diagnostic; “add a wider wheel” is prescriptive.

Read every limit from current environment/evaluator metrics. Constraint values may be shown; Invisible non-constraint values may not. Never duplicate stage constants in `feedback.py`.

## Phase 1: Forensic analysis

### 1. Trace the interface and metric path

Read all seven task modules and the shared feedback/verifier code that calls them. Trace:

`world state -> tracked metric -> evaluator decision/score -> serialized metrics -> format_task_metrics`

Record expected keys, callers, units, frames, optional values, and serialization requirements. Preserve `get_improvement_suggestions`; it must not become a solution-advice channel.

Create a provenance table:

| Feedback field | Meaning | Measurement source | Live limit source | Unit/frame | Exposure class |
| --- | --- | --- | --- | --- | --- |

Flag stale constants, producer/formatter key mismatches, unit/sign errors, unavailable attributes, non-JSON values, and metrics confused across bodies or phases.

### 2. Reconstruct actual failures

Across all stages, inspect early failures, near misses, plateaus, and different strategies when available. Check system/API misuse, guessed hidden parameters, environment limitations, genuine reasoning versus blind tuning, repeated failure mechanisms, local minima, conflicting properties, and score trajectory across attempts.

Analyze these six dimensions and cite relevant attempts. Mark an inapplicable dimension with a physical reason rather than inventing data:

1. **Temporal chronology:** event order, onset step, duration, recovery, oscillation, and terminal cause.
2. **Spatial margins:** position/orientation error, closest approach, clearance, contact, path deviation, and boundary margin in the correct frame.
3. **Load distribution:** forces, impulses, torque, support/load distribution, saturation, stress concentration, breakage margin, or control-error divergence.
4. **Energy flow:** kinetic/potential/elastic energy, work, dissipation, transfer, loss mechanisms, and efficiency where applicable.
5. **Constraint profile:** all build/runtime constraints with PASS/FAIL, worst/first violation, duration, utilization, and margin.
6. **Numerical health:** NaN/infinity, extreme velocity/acceleration, tunneling, solver divergence, missing bodies, and invalid lifecycle state.

Separate measured observations from hypotheses. A plausible physical explanation is not reportable until the sandbox measures it.

### 3. Prioritize missing diagnostics

Select the top three to five missing or misleading diagnostics. Rank them by ability to distinguish root causes, relevance to pass/fail, coverage across stages/strategies, reliability, context cost, and leakage/prescription risk.

For each diagnostic, specify its sandbox source, update phase, reset state, unit, aggregation, edge cases, formatter wording, and the reasoning step it enables. Reject anything requiring changed physics, evaluation semantics, or candidate APIs.

## Phase 2: Implement additive diagnostics

Prefer formatting metrics already produced. If critical data is missing, add only new evaluator return keys, environment tracking variables, or getters that expose existing simulation state.

Never remove/rename existing keys, modify existing tracking, physics, defaults, thresholds, success/score logic, method signatures, action/evaluation order, RNG consumption, or early-stop behavior. New tracking must reset per run, remain JSON-serializable, and not alter candidate-visible state.

Format with clear sections, worst-first ordering, two or three decimal places, `math.isfinite()` guards, and safe `.get()` handling. Never fabricate a missing/NaN/infinite value or hardcode a limit.

Each feedback report should lead with outcome and decisive evidence, pair measurements with live limits and margins, identify units/frames/objects/phases when ambiguous, distinguish constraints from performance shortfalls, and avoid unsupported causal claims or advice.

Use task-specific emphasis:

| Category | Primary diagnostics |
| --- | --- |
| Statics | failure chronology, spatial margins, load/stress ranking, constraints |
| Kinematics | target margin, collapse timing, joint range, motor utilization |
| Dynamics | energy chain/losses, trajectory margins, event chronology |
| Granular/Fluid | leakage/containment timeline, margins, surge loads, constraints |
| Control | error by zone, speed/stability margins, violation timing |
| Exotic | all applicable dimensions; unusual physics requires extra care |

## Phase 3: Debloat and verify

Run the formatter on real Stage-1 and Stage-4 feedback plus representative failures from every stage. Simplify `feedback.py` without altering other task modules or `get_improvement_suggestions`.

- Report each fact once in its most useful section.
- If three-moment sampling exists, report the first moment fully and later moments as unambiguous deltas.
- Collapse repetitive lists to summaries while expanding failed, distinct, or nearest-to-limit items.
- Remove speculative derived math and evaluator-irrelevant thresholds.
- Keep concise energy/loss summaries, near-limit or failed constraints, nearest spatial margins, and one-line numerical health unless anomalies exist.
- Preserve every applicable diagnostic dimension and hard-constraint violation.

Target at least a 50% reduction when the original output contains that much redundancy, but never delete the only decision-relevant evidence merely to meet the target. Compare actual before/after character or token counts.

Run at least:

```bash
python -m compileall "{{TASK_DIR}}"
pace-bench validate --task "{{TASK_ID}}" --contracts-only
pace-bench validate --task "{{TASK_ID}}"
```

Exercise the full evaluation path for all four stages and inspect saved feedback. Include a near-success, severe failure, hard-constraint violation, missing/partial metrics, NaN/infinity handling, and repeated same-seed run where applicable.

Demonstrate unchanged score, pass/fail, simulation steps, evaluator metrics, and complete reference matrix. New observational fields are allowed, but existing useful result fields and formatter contracts must remain readable.

This is an iterative process. After every feedback revision or debloat pass, rerun the complete forensic inspection on newly generated real outputs and repeat all validation. If the new inspection finds missing, misleading, redundant, prescriptive, leaking, malformed, or runtime-failing feedback, revise it and inspect again. Stop only when a fresh complete inspection performed after the last revision finds no errors, requires no further edits, and preserves all reference and evaluation behavior; the pass that changed code cannot count as the final clean pass.

## Required final report

Return:

1. forensic findings across system/API behavior, reasoning trajectory, failure mechanisms, score trajectory, and six diagnostic dimensions;
2. ranked top three-to-five missing diagnostics and the metric provenance table;
3. exact additive tracking/formatting changes;
4. representative before/after feedback and length counts;
5. exposure, threshold, non-prescription, and compatibility evidence;
6. exact validation commands, reference matrix, and remaining blind spots.

Do not claim improvement from static inspection alone; support it with actual verifier output from mutated environments.
