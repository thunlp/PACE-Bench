"""Cohesive Box2D stepping loops preserved from the benchmark runtime."""

from __future__ import annotations

from contextlib import suppress

from pace_bench.core.simulator import TIME_STEP, Simulator


def _matches_category(task_name: str, category: int) -> bool:
    return f"category_{category}_" in task_name or f"category{category}" in task_name


def _matches_task(task_name: str, prefix: str, category: int, number: int) -> bool:
    return (
        f"{prefix}_{number:02d}" in task_name
        or f"category_{category}_{number:02d}" in task_name
    )


class SimulationMixin:
    def _evaluate_with_penalty(self, evaluator, *args, **kwargs):
        """
        Thin wrapper around evaluator.evaluate() that applies constraint violation penalty.

        When the evaluator returns score == 0 (task failed), the penalty is computed
        based on constraint satisfaction ratio, making the score negative in [-100, 0].
        When score > 0, returns score unchanged.

        This gives the evaluator a richer [-100, 100] score signal instead of binary 0/100.
        """
        done, score, metrics = evaluator.evaluate(*args, **kwargs)
        if hasattr(evaluator, "compute_score_with_penalty"):
            score = evaluator.compute_score_with_penalty(score, metrics)
        return done, score, metrics

    def _run_simulation(
        self,
        environment,
        agent_components,
        evaluator,
        code_module,
        headless,
        save_gif_path=None,
        granularity: str = "outcome-based",
    ):
        """Run simulation loop"""
        # Initialize simulator
        self.simulator = Simulator()
        # If GIF path provided, enable GIF saving
        save_gif = save_gif_path is not None
        can_display = self.simulator.init_display(headless=headless, save_gif=save_gif)

        # Initialize renderer (if need to save GIF or display)
        renderer = None
        if save_gif or can_display:
            renderer = self._init_renderer(environment)

        step_count = 0
        running = True
        camera_offset_x = 0

        # Primary physics object for camera / stuck detection / evaluator (never pass raw list/tuple through)
        agent_body = self._primary_physics_object(agent_components)

        # Get agent_action function
        agent_action_func = getattr(code_module, "agent_action", None)

        # Standard simulation loop
        STABILIZATION_STEPS = 60
        last_position = None
        stuck_counter = 0
        # Unified stuck detection: Use Category1 threshold for all tasks (900 steps, 0.02m)
        # This is more lenient and reduces false positives
        STUCK_THRESHOLD = 300  # Base threshold, will be multiplied by 3
        STUCK_THRESHOLD_MULTIPLIER = 3  # Final threshold = 300 * 3 = 900 steps
        POSITION_EPSILON = 0.01  # Base epsilon, will be multiplied by 2
        POSITION_EPSILON_MULTIPLIER = 2  # Final epsilon = 0.01 * 2 = 0.02m

        # Render initial frame before simulation starts (to ensure at least one frame is collected)
        if save_gif and renderer and hasattr(renderer, "render"):
            try:
                camera_offset_x = 0
                target_x_world = 0
                pos_xy = self._safe_world_xy(agent_body)
                if pos_xy:
                    target_x = pos_xy[0] * self.simulator.ppm
                    camera_offset_x = target_x - self.simulator.screen_width / 2
                elif hasattr(environment, "get_sled_position"):
                    # E-03 Slippery World: camera follows sled
                    sled_pos = environment.get_sled_position()
                    if sled_pos:
                        camera_offset_x = (
                            sled_pos[0] * self.simulator.ppm
                            - self.simulator.screen_width / 2
                        )
                elif hasattr(environment, "get_body_position"):
                    # E-05 Magnet: camera follows body
                    body_pos = environment.get_body_position()
                    if body_pos:
                        camera_offset_x = (
                            body_pos[0] * self.simulator.ppm
                            - self.simulator.screen_width / 2
                        )
                if evaluator and hasattr(evaluator, "target_x"):
                    target_x_world = evaluator.target_x
                renderer.render(
                    environment, agent_body, target_x_world, camera_offset_x
                )
                if can_display:
                    self.simulator.flip()
                self.simulator.collect_frame(0)  # Collect initial frame at step 0
            except Exception:
                pass

        n_moments = self._parse_granularity(granularity)
        # For process_n: continuously sample metrics during simulation.
        # At termination, extract t/3, 2t/3, t moments from actual termination step t.
        all_step_snapshots = []  # List of (step_count, score, metrics) tuples
        granular_snapshots = []

        # Evaluate at step 0 (design constraints only) so build-time constraints are checked before any physics step
        if evaluator and step_count == 0:
            task_lower = self.task_name.lower()
            is_category1 = any(
                x in task_lower
                for x in [
                    "s_01",
                    "s_02",
                    "s_03",
                    "s_04",
                    "s_05",
                    "s_06",
                    "category1",
                    "category_1",
                ]
            )
            is_e03_sled = "e_03" in task_lower
            is_e05_magnet = "e_05" in task_lower
            is_category5_c06 = "c_06" in task_lower or ("category_5_06" in task_lower)
            if is_category1 or is_category5_c06 or is_e05_magnet or is_e03_sled:
                init_done, init_score, init_metrics = self._evaluate_with_penalty(
                    evaluator, None, 0, self.max_steps
                )
            else:
                init_done, init_score, init_metrics = self._evaluate_with_penalty(
                    evaluator, agent_body, 0, self.max_steps
                )
            # Record step 0 snapshot for process_n
            if n_moments > 1:
                all_step_snapshots.append(
                    {
                        "step_count": 0,
                        "score": init_score,
                        "success": bool(init_metrics.get("success", False)),
                        "failed": bool(init_metrics.get("failed", False)),
                        "failure_reason": init_metrics.get("failure_reason"),
                        "metrics": dict(init_metrics or {}),
                    }
                )
            if (
                init_done
                and init_metrics.get("failed")
                and init_metrics.get("failure_reason", "").startswith(
                    "Design constraint"
                )
            ):
                if (
                    save_gif_path
                    and self.simulator
                    and getattr(self.simulator, "frames", None)
                ):
                    self.simulator.save_gif_animation(save_gif_path)
                # Extract granular snapshots: t = 0 (terminated at step 0)
                init_metrics["step_count"] = 0
                if n_moments > 1:
                    granular_snapshots = self._extract_granular_snapshots(
                        all_step_snapshots, 0, n_moments, self.max_steps
                    )
                    init_metrics["granular_snapshots"] = granular_snapshots
                return False, init_score, init_metrics

        while running and step_count < self.max_steps:
            # Handle events
            if not self.simulator.handle_events():
                running = False
                break

            # Agent executes action
            if agent_action_func:
                if isinstance(agent_components, dict):
                    agent_action_func(environment, agent_components, step_count)
                else:
                    agent_action_func(environment, agent_components, step_count)

            # Physics step
            environment.step(TIME_STEP)
            step_count += 1

            # Detect stuck - Unified for all tasks: Use Category1 threshold (900 steps, 0.02m)
            # This is more lenient and reduces false positives while still catching truly stuck cases
            # Use raw task name (underscores preserved) to support both short format (category_3_01)
            # and full path format (Category3_Dynamics_Energy/D_01).
            # Short format: 'category_N_' prefix matches (e.g. 'category_3_' in 'category_3_01')
            # Full path format: 'categoryN' matches (e.g. 'category3' in 'category3_dynamics_energy/d_01')
            task_lower_raw = self.task_name.lower()
            is_category1_task = _matches_category(task_lower_raw, 1)
            is_e03_sled = _matches_task(task_lower_raw, "e", 6, 3)
            is_e05_magnet = _matches_task(task_lower_raw, "e", 6, 5)

            # Try to get position from vehicle (Category1), sled (E-03), body (E-05), or agent_body (other tasks)
            current_pos = None
            if is_category1_task:
                vehicle_pos = (
                    environment.get_vehicle_position()
                    if hasattr(environment, "get_vehicle_position")
                    else None
                )
                if vehicle_pos:
                    current_pos = vehicle_pos
            elif is_e03_sled and hasattr(environment, "get_sled_position"):
                sled_pos = environment.get_sled_position()
                if sled_pos:
                    current_pos = sled_pos
            elif is_e05_magnet and hasattr(environment, "get_body_position"):
                body_pos = environment.get_body_position()
                if body_pos:
                    current_pos = body_pos
            elif agent_body:
                current_pos = self._safe_world_xy(agent_body)

            # Unified stuck detection for all tasks (skip for Category4/F_03: plow can be "stuck" during scoop phase;
            # skip for C_02 Lander: lander sits on ground after landing so position is constant;
            # skip for C_04/C_05/C_06: agent may stay in zones or move slowly to satisfy sequence/timing)
            skip_stuck = (
                _matches_task(task_lower_raw, "e", 6, 3)
                or _matches_task(task_lower_raw, "e", 6, 4)
                or _matches_task(task_lower_raw, "e", 6, 5)
                or _matches_task(task_lower_raw, "e", 6, 6)
                or _matches_task(task_lower_raw, "e", 6, 1)
                or _matches_task(task_lower_raw, "s", 1, 2)
                or _matches_task(task_lower_raw, "s", 1, 6)
                or _matches_task(task_lower_raw, "c", 5, 1)
                or _matches_task(task_lower_raw, "c", 5, 2)
                or _matches_task(task_lower_raw, "c", 5, 3)
                or _matches_task(task_lower_raw, "c", 5, 4)
                or _matches_task(task_lower_raw, "c", 5, 5)
                or _matches_task(task_lower_raw, "c", 5, 6)
                or _matches_task(task_lower_raw, "k", 2, 1)
                or _matches_task(task_lower_raw, "k", 2, 4)
                or _matches_task(task_lower_raw, "k", 2, 5)
                or _matches_task(task_lower_raw, "k", 2, 6)
                or _matches_category(task_lower_raw, 3)
                or _matches_category(task_lower_raw, 4)
            )
            if (
                current_pos
                and not skip_stuck
                and step_count > STABILIZATION_STEPS
                and last_position is not None
            ):
                dx = abs(current_pos[0] - last_position[0])
                dy = abs(current_pos[1] - last_position[1])
                # Use Category1 threshold: 0.02m (more lenient)
                if (
                    dx < POSITION_EPSILON * POSITION_EPSILON_MULTIPLIER
                    and dy < POSITION_EPSILON * POSITION_EPSILON_MULTIPLIER
                ):
                    stuck_counter += 1
                    # Use Category1 threshold: 900 steps (more lenient)
                    if stuck_counter >= STUCK_THRESHOLD * STUCK_THRESHOLD_MULTIPLIER:
                        print(
                            f"Detected stuck at step {step_count} (position change < {POSITION_EPSILON * POSITION_EPSILON_MULTIPLIER}m for {STUCK_THRESHOLD * STUCK_THRESHOLD_MULTIPLIER} steps), stopping simulation"
                        )
                        running = False
                        break
                else:
                    stuck_counter = 0
            if current_pos:
                last_position = current_pos

                # Check if vehicle/agent fell (failure condition, not stuck detection)
                if is_category1_task:
                    # Adaptive falling threshold
                    fall_y = -5.0
                    if current_pos[1] < fall_y:
                        running = False
                        break
                elif agent_body:
                    # Detect anomalies for non-Category1 tasks (e.g. runaway velocity).
                    # Skip speed check for: Category3 (projectile/jumper); Category4 (excavator/plow can move fast).
                    is_projectile_task = _matches_category(task_lower_raw, 3)
                    is_category4_fast = _matches_category(task_lower_raw, 4)
                    if not is_projectile_task and not is_category4_fast:
                        speed = self._safe_linear_speed(agent_body)
                        if speed is not None and speed > 2000:
                            running = False
                            break
                    pos_xy = self._safe_world_xy(agent_body)
                    if pos_xy and pos_xy[1] < -10:
                        running = False
                        break

            # Render
            if (save_gif or can_display) and renderer and hasattr(renderer, "render"):
                # Camera follow: agent_body, or sled for E-03
                pos_xy = self._safe_world_xy(agent_body)
                if pos_xy:
                    target_x = pos_xy[0] * self.simulator.ppm
                    camera_offset_x = target_x - self.simulator.screen_width / 2
                elif is_e03_sled and hasattr(environment, "get_sled_position"):
                    sled_pos = environment.get_sled_position()
                    if sled_pos:
                        camera_offset_x = (
                            sled_pos[0] * self.simulator.ppm
                            - self.simulator.screen_width / 2
                        )
                    else:
                        camera_offset_x = 0
                elif is_e05_magnet and hasattr(environment, "get_body_position"):
                    body_pos = environment.get_body_position()
                    if body_pos:
                        camera_offset_x = (
                            body_pos[0] * self.simulator.ppm
                            - self.simulator.screen_width / 2
                        )
                    else:
                        camera_offset_x = 0
                else:
                    camera_offset_x = 0

                # Get target position (for rendering)
                target_x_world = None
                if evaluator and hasattr(evaluator, "target_x"):
                    target_x_world = evaluator.target_x
                elif evaluator and hasattr(evaluator, "get_task_description"):
                    task_info = evaluator.get_task_description()
                    target_x_world = task_info.get("target_position", 0)

                # Render (even if agent_body is None, render environment)
                try:
                    # Verify renderer type (only for basic task)
                    if "basic" in self.task_name.lower():
                        renderer_type = type(renderer).__name__
                        if renderer_type != "BasicRenderer":
                            print(
                                f"Warning: Using wrong renderer type: {renderer_type}, should be BasicRenderer"
                            )
                    renderer.render(
                        environment, agent_body, target_x_world or 0, camera_offset_x
                    )
                except Exception:
                    pass

                # Refresh display
                if can_display:
                    self.simulator.flip()
                    self.simulator.tick()

                # Collect frames after rendering (only if rendering succeeded)
                if save_gif:
                    self.simulator.collect_frame(step_count)
            elif save_gif:
                # If no renderer but save_gif is enabled, try to collect frame anyway
                # This handles cases where renderer failed to initialize but we still want to save
                self.simulator.collect_frame(step_count)

            # Evaluate
            # For Category1 tasks (S_01-S_06), evaluator doesn't need agent_body (tracks structure/vehicle via environment)
            # For K_03 (gripper), evaluate every 10 steps so success (hold 80 steps) is detected before object may fall
            # For C_03 (seeker), evaluate every 10 steps to detect "target lost" (distance > 6m) promptly
            # For C_01 (cart-pole), evaluate every step so lock-in matches consecutive simulation steps.
            # For C_02 (lander), evaluate every step so landing (and success/failure) is detected immediately
            task_lower = self.task_name.lower()
            # Support both short format (category_N_MM) and full path format (CategoryN_Words/X_MM)
            is_category1 = _matches_category(task_lower, 1)
            is_category2_k03 = "k_03" in task_lower or "category_2_03" in task_lower
            is_category3 = _matches_category(task_lower, 3)
            is_category5_c06 = "c_06" in task_lower or "category_5_06" in task_lower
            is_category5_c03 = "c_03" in task_lower or "category_5_03" in task_lower
            is_category5_c02 = "c_02" in task_lower or "category_5_02" in task_lower
            is_category5_c01 = "c_01" in task_lower or "category_5_01" in task_lower
            is_e05_magnet = "e_05" in task_lower or "category_6_05" in task_lower
            is_e02_thickair = "e_02" in task_lower or "category_6_02" in task_lower
            eval_interval = (
                1
                if (
                    is_category5_c01
                    or is_category5_c02
                    or is_category5_c06
                    or is_e05_magnet
                    or is_category3
                )
                else (10 if (is_category2_k03 or is_category5_c03) else 100)
            )
            # For process_n: sample more frequently to ensure we have data at t/3, 2t/3, t
            # Use finer sampling interval when granularity is process_n
            if n_moments > 1:
                eval_interval = min(
                    eval_interval, max(1, self.max_steps // (n_moments * 10))
                )  # Sample at least 10 x n points
            if step_count % eval_interval == 0 and evaluator:
                # Check for Category1 tasks (case-insensitive) or E-03 (sled, no agent_body)
                # is_category1 already computed above via _tl_is_cat(1)
                if is_category1:
                    # Category1 evaluators don't use agent_body - they get info from environment
                    should_stop, score, metrics = self._evaluate_with_penalty(
                        evaluator, None, step_count, self.max_steps
                    )
                elif is_e05_magnet:
                    should_stop, score, metrics = self._evaluate_with_penalty(
                        evaluator, None, step_count, self.max_steps
                    )
                elif is_e03_sled:
                    # E-03: evaluator uses environment.get_sled_position()
                    should_stop, score, metrics = self._evaluate_with_penalty(
                        evaluator, None, step_count, self.max_steps
                    )
                elif is_category5_c06 or is_category5_c02:
                    should_stop, score, metrics = self._evaluate_with_penalty(
                        evaluator, None, step_count, self.max_steps
                    )
                elif agent_body:
                    should_stop, score, metrics = self._evaluate_with_penalty(
                        evaluator, agent_body, step_count, self.max_steps
                    )
                elif is_category3:
                    # D_04 (Swing) etc.: build_agent returns None, evaluator uses environment (e.g. get_swing_seat_position)
                    should_stop, score, metrics = self._evaluate_with_penalty(
                        evaluator, None, step_count, self.max_steps
                    )
                elif is_e02_thickair:
                    # E-02: build_agent returns None, evaluator tracks craft via environment.get_craft_position()
                    should_stop, score, metrics = self._evaluate_with_penalty(
                        evaluator, None, step_count, self.max_steps
                    )
                else:
                    should_stop, score, metrics = False, 0.0, {}

                # For process_n: record all evaluation snapshots during simulation.
                # At termination, we'll extract t/3, 2t/3, t moments from actual termination step t.
                if n_moments > 1:
                    all_step_snapshots.append(
                        {
                            "step_count": step_count,
                            "score": score,
                            "success": bool(metrics.get("success", False)),
                            "failed": bool(metrics.get("failed", False)),
                            "failure_reason": metrics.get("failure_reason"),
                            "metrics": dict(metrics or {}),
                        }
                    )

                if should_stop and metrics.get("success"):
                    # Render and collect final frame before saving GIF
                    if (
                        (save_gif or can_display)
                        and renderer
                        and hasattr(renderer, "render")
                    ):
                        pos_xy = self._safe_world_xy(agent_body)
                        if pos_xy:
                            target_x = pos_xy[0] * self.simulator.ppm
                            camera_offset_x = target_x - self.simulator.screen_width / 2
                        elif is_e03_sled and hasattr(environment, "get_sled_position"):
                            sled_pos = environment.get_sled_position()
                            camera_offset_x = (
                                (
                                    sled_pos[0] * self.simulator.ppm
                                    - self.simulator.screen_width / 2
                                )
                                if sled_pos
                                else 0
                            )
                        else:
                            camera_offset_x = 0
                        target_x_world = None
                        if evaluator and hasattr(evaluator, "target_x"):
                            target_x_world = evaluator.target_x
                        with suppress(Exception):
                            renderer.render(
                                environment,
                                agent_body,
                                target_x_world or 0,
                                camera_offset_x,
                            )
                    # Collect final frame
                    if save_gif:
                        self.simulator.collect_frame(step_count)
                    # Save GIF before returning (important: save current state)
                    if save_gif_path and self.simulator:
                        self.simulator.save_gif_animation(save_gif_path)
                    # Ensure step_count is in metrics
                    metrics["step_count"] = step_count
                    # Extract granular snapshots: t = actual termination step (step_count)
                    if n_moments > 1:
                        granular_snapshots = self._extract_granular_snapshots(
                            all_step_snapshots, step_count, n_moments, self.max_steps
                        )
                        metrics["granular_snapshots"] = granular_snapshots
                    return True, score, metrics
                elif should_stop and metrics.get("failed"):
                    # Render and collect final frame before saving GIF
                    if (
                        (save_gif or can_display)
                        and renderer
                        and hasattr(renderer, "render")
                    ):
                        pos_xy = self._safe_world_xy(agent_body)
                        if pos_xy:
                            target_x = pos_xy[0] * self.simulator.ppm
                            camera_offset_x = target_x - self.simulator.screen_width / 2
                        elif is_e03_sled and hasattr(environment, "get_sled_position"):
                            sled_pos = environment.get_sled_position()
                            camera_offset_x = (
                                (
                                    sled_pos[0] * self.simulator.ppm
                                    - self.simulator.screen_width / 2
                                )
                                if sled_pos
                                else 0
                            )
                        else:
                            camera_offset_x = 0
                        target_x_world = None
                        if evaluator and hasattr(evaluator, "target_x"):
                            target_x_world = evaluator.target_x
                        with suppress(Exception):
                            renderer.render(
                                environment,
                                agent_body,
                                target_x_world or 0,
                                camera_offset_x,
                            )
                    # Collect final frame
                    if save_gif:
                        self.simulator.collect_frame(step_count)
                    # Save GIF before returning (even if failed)
                    if save_gif_path and self.simulator:
                        self.simulator.save_gif_animation(save_gif_path)
                    # Ensure step_count is in metrics
                    metrics["step_count"] = step_count
                    # Extract granular snapshots: t = actual termination step (step_count)
                    if n_moments > 1:
                        granular_snapshots = self._extract_granular_snapshots(
                            all_step_snapshots, step_count, n_moments, self.max_steps
                        )
                        metrics["granular_snapshots"] = granular_snapshots
                    return False, score, metrics

        # Final evaluation
        # For Category1 (S_01-S_06) and E-03 (sled), evaluator uses environment; agent_body may be None
        if evaluator:
            task_lower = self.task_name.lower()
            is_category1_final = _matches_category(task_lower, 1)
            is_e03_final = "e_03" in task_lower or "category_6_03" in task_lower
            is_e02_final = "e_02" in task_lower or "category_6_02" in task_lower
            is_category3_final = _matches_category(task_lower, 3)
            if (
                is_category1_final
                or is_e03_final
                or is_category5_c06
                or is_category5_c02
            ):
                final_should_stop, final_score, final_metrics = (
                    self._evaluate_with_penalty(
                        evaluator, None, step_count, self.max_steps
                    )
                )
            elif agent_body:
                final_should_stop, final_score, final_metrics = (
                    self._evaluate_with_penalty(
                        evaluator, agent_body, step_count, self.max_steps
                    )
                )
            elif is_category3_final:
                # Category3 tasks where build_agent may return None (e.g. D_04 Swing)
                final_should_stop, final_score, final_metrics = (
                    self._evaluate_with_penalty(
                        evaluator, None, step_count, self.max_steps
                    )
                )
            elif is_e02_final:
                # E-02: build_agent returns None, evaluator tracks craft via environment.get_craft_position()
                final_should_stop, final_score, final_metrics = (
                    self._evaluate_with_penalty(
                        evaluator, None, step_count, self.max_steps
                    )
                )
            else:
                _final_should_stop, final_score, final_metrics = False, 0.0, {}

            # Save GIF
            if save_gif_path and self.simulator:
                with suppress(Exception):
                    self.simulator.save_gif_animation(save_gif_path)

            if final_metrics is None:
                final_metrics = {}
            # Ensure step_count is in final_metrics
            final_metrics["step_count"] = step_count
            # Record final evaluation snapshot
            if n_moments > 1:
                all_step_snapshots.append(
                    {
                        "step_count": step_count,
                        "score": final_score,
                        "success": bool(final_metrics.get("success", False)),
                        "failed": bool(final_metrics.get("failed", False)),
                        "failure_reason": final_metrics.get("failure_reason"),
                        "metrics": dict(final_metrics or {}),
                    }
                )
            # Extract granular snapshots: t = actual termination step (step_count)
            if n_moments > 1:
                granular_snapshots = self._extract_granular_snapshots(
                    all_step_snapshots, step_count, n_moments, self.max_steps
                )
                final_metrics["granular_snapshots"] = granular_snapshots
            elif not final_metrics.get("granular_snapshots"):
                final_metrics["granular_snapshots"] = [
                    {
                        "step_count": step_count,
                        "score": final_score,
                        "success": bool(final_metrics.get("success", False)),
                        "failed": bool(final_metrics.get("failed", False)),
                        "failure_reason": final_metrics.get("failure_reason"),
                        "metrics": dict(final_metrics or {}),
                        "max_steps": self.max_steps,
                    }
                ]
            # Retain constraint metadata for dense failure scores and auditing.
            if evaluator is not None and hasattr(evaluator, "get_constraint_info"):
                try:
                    constraint_info = evaluator.get_constraint_info()
                    if constraint_info and isinstance(constraint_info, dict):
                        final_metrics["constraint_info"] = constraint_info
                except Exception:
                    pass  # Non-fatal; constraint_info is optional

            return final_metrics.get("success", False), final_score, final_metrics

        # Save GIF (even without evaluator)
        if save_gif_path and self.simulator:
            with suppress(Exception):
                self.simulator.save_gif_animation(save_gif_path)

        return False, 0.0, {}
