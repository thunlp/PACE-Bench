"""Task environment, evaluator, renderer, and snapshot setup."""

from __future__ import annotations

import inspect
import re


class TaskRuntimeMixin:
    def _init_environment(self):
        """Initialize environment"""
        # Find environment class
        env_class_name = None
        if hasattr(self.task_module, "environment"):
            for name, obj in self.task_module.environment.__dict__.items():
                if isinstance(obj, type) and "Sandbox" in name:
                    env_class_name = name
                    break

        if not env_class_name:
            raise AttributeError(
                "Unable to find environment class (should contain 'Sandbox')"
            )

        env_class = getattr(self.task_module.environment, env_class_name)
        try:
            return env_class(**self.env_overrides)
        except TypeError:
            # Backward compatible: if environment does not accept overrides, fall back.
            return env_class()

    def _init_evaluator(self, environment):
        """Initialize evaluator by inspecting its __init__ signature"""
        if not hasattr(self.task_module, "evaluator"):
            return None

        eval_class_name = None
        for name, obj in self.task_module.evaluator.__dict__.items():
            if isinstance(obj, type) and "Evaluator" in name:
                eval_class_name = name
                break

        if not eval_class_name:
            return None

        eval_class = getattr(self.task_module.evaluator, eval_class_name)

        # Inspect __init__ signature to determine how to initialize
        try:
            sig = inspect.signature(eval_class.__init__)
            params = list(sig.parameters.keys())
            # Skip 'self' parameter
            params = [p for p in params if p != "self"]

            # Try different initialization patterns based on signature
            if len(params) == 0:
                # No parameters (unlikely but handle it)
                return eval_class()
            elif len(params) == 1:
                # Single parameter - likely just environment
                param_name = params[0]
                if param_name in ["sandbox", "environment"]:
                    return eval_class(environment)
                else:
                    # Try with environment anyway
                    return eval_class(environment)
            elif len(params) == 2:
                # Two parameters - check if first is terrain_bounds
                param1_name = params[0]
                param2_name = params[1]

                if param1_name == "terrain_bounds":
                    # Pattern: (terrain_bounds, environment=None)
                    terrain_bounds = environment.get_terrain_bounds()
                    # Check if second parameter has default value
                    param2 = sig.parameters[param2_name]
                    if param2.default != inspect.Parameter.empty:
                        # Has default, can pass as keyword argument
                        return eval_class(terrain_bounds, environment=environment)
                    else:
                        # No default, must pass as positional
                        return eval_class(terrain_bounds, environment)
                elif param2_name in ["environment", "sandbox"]:
                    # Pattern: (start_x, target_x) or similar, but second is environment
                    # This is less common, try with environment
                    terrain_bounds = environment.get_terrain_bounds()
                    return eval_class(terrain_bounds, environment)
                else:
                    # Two parameters that are not terrain_bounds/environment
                    # Could be numeric parameters (e.g., simple task: start_x, target_x)
                    # Try to get values from environment if possible, otherwise use defaults
                    try:
                        # Try to get terrain bounds first (most common pattern)
                        terrain_bounds = environment.get_terrain_bounds()
                        return eval_class(terrain_bounds, environment)
                    except Exception:
                        # If that fails, try numeric defaults (for simple task pattern)
                        try:
                            return eval_class(3.0, 15.0)
                        except Exception:
                            # Last resort: try with environment only
                            return eval_class(environment)
            else:
                # More than 2 parameters - try default pattern
                terrain_bounds = environment.get_terrain_bounds()
                return eval_class(terrain_bounds, environment)
        except Exception as e:
            # Fallback to old behavior if inspection fails
            print(f"Warning: Failed to inspect evaluator signature: {e}")
            # Try common patterns
            try:
                terrain_bounds = environment.get_terrain_bounds()
                return eval_class(terrain_bounds, environment)
            except Exception:
                return eval_class(environment)

    def _init_renderer(self, environment):
        """Initialize renderer"""
        if not hasattr(self.task_module, "renderer"):
            task = self.registry.resolve(self.task_name)
            try:
                renderer_module = self.registry.load_module(task, "renderer")
            except (ImportError, ModuleNotFoundError):
                return None
            self.task_module.renderer = renderer_module

        renderer_class_name = None
        renderer_candidates = []
        for name, obj in self.task_module.renderer.__dict__.items():
            if (
                isinstance(obj, type)
                and "Renderer" in name
                and name != "Renderer"
                and hasattr(obj, "render")
            ):
                renderer_candidates.append((name, obj))

        if renderer_candidates:
            task_name_lower = self.task_name.lower()
            # Prefer task-specific renderer (name contains task name)
            for name, _obj in renderer_candidates:
                if task_name_lower in name.lower():
                    renderer_class_name = name
                    break
            # If no task-specific found, use first candidate (excluding base Renderer class)
            # This is consistent with main.py logic
            if not renderer_class_name and renderer_candidates:
                for name, _obj in renderer_candidates:
                    if name != "Renderer":
                        renderer_class_name = name
                        break
                # If still none, use first one
                if not renderer_class_name:
                    renderer_class_name = renderer_candidates[0][0]

        if renderer_class_name:
            try:
                renderer_class = getattr(self.task_module.renderer, renderer_class_name)
                renderer = renderer_class(self.simulator)
                # Verify renderer type
                if hasattr(renderer, "render"):
                    pass
                else:
                    print("⚠️  Warning: Renderer does not have render method")
                return renderer
            except Exception as e:
                print(
                    f"⚠️  Warning: Failed to initialize renderer {renderer_class_name}: {e}"
                )
                return None
        else:
            pass
        return None

    def _parse_granularity(self, granularity: str) -> int:
        g = (granularity or "outcome-based").strip().lower()
        if g == "outcome-based":
            return 1
        m = re.fullmatch(r"process_(\d+)", g)
        if not m:
            raise ValueError(f"Unsupported granularity: {granularity}")
        n = int(m.group(1))
        if n <= 0:
            raise ValueError(f"Granularity process_n requires n >= 1, got {n}")
        return n

    @staticmethod
    def _snapshot_delta(snap_a, snap_b, max_steps):
        """
        Task-agnostic state-change metric between two snapshots (0.0-1.0).
        Weights: score change > status flip > structure integrity > step distance.
        """
        total = 0.0
        max_steps = max(1, max_steps)

        # 1. Score change (normalized 0-1) - highest weight
        score_a = float(snap_a.get("score", 0))
        score_b = float(snap_b.get("score", 0))
        total += abs(score_a - score_b) / 100.0 * 2.0

        # 2. Step distance (normalized) — low structural weight
        step_a = float(snap_a.get("step_count", 0))
        step_b = float(snap_b.get("step_count", 0))
        total += abs(step_a - step_b) / max_steps * 0.5

        # 3. Failure status flip
        if bool(snap_a.get("failed")) != bool(snap_b.get("failed")):
            total += 0.25

        # 4. Success status flip
        if bool(snap_a.get("success")) != bool(snap_b.get("success")):
            total += 0.20

        # 5. Joint integrity change (task-agnostic: try common keys)
        ma = snap_a.get("metrics", {}) or {}
        mb = snap_b.get("metrics", {}) or {}
        for jk in ("joint_count", "num_joints"):
            jc_a = ma.get(jk)
            jc_b = mb.get(jk)
            ijc = ma.get("initial_joint_count") or ma.get(jk)
            if jc_a is not None and jc_b is not None and ijc is not None:
                denom = max(1.0, float(ijc))
                total += abs(float(jc_a) - float(jc_b)) / denom * 1.5
                break

        # 6. Structure-broken / pivot-destroyed flag flips (common across tasks)
        for flag in ("structure_broken", "pivot_joint_destroyed", "anchor_broken"):
            if bool(ma.get(flag)) != bool(mb.get(flag)):
                total += 0.20
                break

        # 7. Failure reason change (different failure mode)
        fr_a = (snap_a.get("failure_reason") or "").strip()
        fr_b = (snap_b.get("failure_reason") or "").strip()
        if fr_a and fr_b and fr_a != fr_b:
            total += 0.15

        return min(1.0, total)

    def _extract_granular_snapshots(
        self, all_step_snapshots, actual_termination_step, n_moments, max_steps
    ):
        """
        Adaptive content-aware key-moment selection.

        Only the **terminal frame** (last snapshot) is mandatory — it records
        the final outcome.  The remaining slots are filled greedily: starting
        from the terminal state and scanning backwards, we pick the snapshot
        whose state differs most from the last-selected one.  Runs with little
        content change naturally produce fewer than *n_moments* moments.

        Step-count deduplication guarantees no two moments share the same
        simulation step.
        """
        if n_moments <= 1 or not all_step_snapshots:
            return []

        # ----- deduplicate by step_count (keep first occurrence) -----
        unique: list = []
        seen_steps: set = set()
        for snap in all_step_snapshots:
            step = int(snap.get("step_count", -1))
            if step not in seen_steps:
                unique.append(snap)
                seen_steps.add(step)

        # ----- select key moments (farthest-point sampling) -----
        terminal = unique[-1]
        selected = [terminal]  # mandatory: terminal outcome
        candidates = list(unique[:-1])

        MIN_DELTA = 0.03  # ignore states that are near-identical

        for _slot in range(n_moments - 1):
            if not candidates:
                break
            best = None
            best_min_delta = MIN_DELTA
            for cand in candidates:
                # distance to the closest already-selected moment —
                # pick the candidate that is farthest from ALL selected
                min_d = min(self._snapshot_delta(cand, s, max_steps) for s in selected)
                if min_d > best_min_delta:
                    best_min_delta = min_d
                    best = cand
            if best is None:
                break
            selected.append(best)
            candidates.remove(best)

        # ----- chronological order -----
        selected.sort(key=lambda s: int(s["step_count"]))

        # ----- build output -----
        actual_moments = len(selected)
        granular_snapshots = []
        for moment_idx, snap in enumerate(selected, start=1):
            granular_snapshots.append(
                {
                    "moment_index": moment_idx,
                    "total_moments": actual_moments,
                    "step_count": snap["step_count"],
                    "score": snap["score"],
                    "success": snap["success"],
                    "failed": snap["failed"],
                    "failure_reason": snap.get("failure_reason"),
                    "metrics": dict(snap.get("metrics", {})),
                    "max_steps": max_steps,
                }
            )

        return granular_snapshots
