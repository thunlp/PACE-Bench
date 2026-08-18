import math

from pace_bench.core.simulator import TIME_STEP

from pace_bench.core.primitives import compute_constraint_penalty

class Evaluator:
    def __init__(self, terrain_bounds, environment=None):
        self.terrain_bounds = terrain_bounds
        self.environment = environment
        if not environment:
            raise ValueError("Evaluator requires environment instance")
        env_terrain_cfg = getattr(environment, "_terrain_config", {})
        self.target_reach = float(env_terrain_cfg.get("target_reach", 12.0))
        self.min_tip_height_limit = float(env_terrain_cfg.get("min_tip_height_limit", -15.0))
        self.load_duration = float(env_terrain_cfg.get("load_duration", 10.0))
        self.load_duration_steps = int(self.load_duration / TIME_STEP)
        self.reach_tolerance = float(env_terrain_cfg.get("reach_tolerance", 1.0))
        self.max_tip_x = 0.0
        self.min_tip_y = 1e9
        self.initial_joint_count = -1
        self.structure_broken = False
        self.load_attach_time = float(env_terrain_cfg.get("load_attach_time", 5.0))
        self.load_2_attach_time = float(env_terrain_cfg.get("load_2_attach_time", 15.0))
        self.load_attach_step = int(self.load_attach_time / TIME_STEP)
        self.load_2_attach_step = int(self.load_2_attach_time / TIME_STEP)
        self.load_1_held_steps = 0
        self.load_2_held_steps = 0
        self.last_step_count = 0
        self.MAX_STRUCTURE_MASS = getattr(environment, 'MAX_STRUCTURE_MASS', 15000.0)
        self.BUILD_ZONE_X_MIN = getattr(environment, 'BUILD_ZONE_X_MIN', 0.0)
        self.BUILD_ZONE_X_MAX = getattr(environment, 'BUILD_ZONE_X_MAX', 50.0)
        self.BUILD_ZONE_Y_MIN = getattr(environment, 'BUILD_ZONE_Y_MIN', -20.0)
        self.BUILD_ZONE_Y_MAX = getattr(environment, 'BUILD_ZONE_Y_MAX', 30.0)
        self.max_recorded_torque = 0.0
        self.max_recorded_force = 0.0
        self.torque_limit_recorded = 0.0
        self.internal_torque_limit_recorded = 0.0
        self.force_limit_recorded = 0.0
        self.internal_force_limit_recorded = 0.0
        self.external_force_y = 0.0
        self.design_constraints_checked = False
        self.reach_satisfied_initially = False
        self._peak_anchor_force = 0.0
        self._peak_anchor_torque = 0.0
        self._peak_internal_force = 0.0
        self._peak_internal_torque = 0.0
        self.global_peak_anchor_force = 0.0
        self.global_peak_anchor_torque = 0.0
        self.global_peak_internal_force = 0.0
        self.global_peak_internal_torque = 0.0
        self._joint_stress_ranking = []
    def evaluate(self, agent_body, step_count, max_steps):
        if not self.environment:
            return False, 0.0, {"error": "Environment not available"}
        current_mass = self.environment.get_structure_mass()
        current_tip_x = self.environment.get_max_reach()
        if self.initial_joint_count == -1:
            self.initial_joint_count = len(self.environment._joints)
        self.max_tip_x = max(self.max_tip_x, current_tip_x)
        if step_count < self.load_attach_step:
            if self.max_tip_x >= self.target_reach:
                self.reach_satisfied_initially = True
        total_ext_force_y = 0.0
        for body in self.environment._bodies:
            self.min_tip_y = min(self.min_tip_y, body.position.y)
            spatial_force = self.environment._physics_config.get("spatial_force", None)
            if spatial_force:
                cx, cy = spatial_force.get("center", (0,0))
                mag = spatial_force.get("magnitude", 0.0)
                radius = spatial_force.get("radius", 10.0)
                is_repulsion = spatial_force.get("type", "repulsion") == "repulsion"
                bx, by = body.position
                dx, dy = bx - cx, by - cy
                dist = math.sqrt(dx**2 + dy**2)
                if dist < radius and dist > 0.1:
                    f = mag * (1.0 - dist / radius)
                    if not is_repulsion: f = -f
                    fy = f * (dy / dist)
                    total_ext_force_y += fy
            wind_config = self.environment._physics_config.get("wind", None)
            if wind_config:
                force_vec = wind_config.get("force", (0, 0))
                if wind_config.get("oscillatory", False):
                    freq = wind_config.get("frequency", 1.0)
                    phase = math.sin(self.environment._simulation_time * 2 * math.pi * freq)
                    force_vec = (force_vec[0] * phase, force_vec[1] * phase)
                total_ext_force_y += force_vec[1]
        self.external_force_y = total_ext_force_y / len(self.environment._bodies) if self.environment._bodies else 0.0
        # The environment samples every live joint immediately after each physics
        # step. Reuse that single authoritative observation instead of performing a
        # second fallible Box2D query and silently discarding failures.
        current_forces = (
            self.environment._current_anchor_forces
            + self.environment._current_internal_forces
        )
        current_torques = (
            self.environment._current_anchor_torques
            + self.environment._current_internal_torques
        )
        if current_forces:
            self.max_recorded_force = max(self.max_recorded_force, max(current_forces))
        if current_torques:
            self.max_recorded_torque = max(self.max_recorded_torque, max(current_torques))
        base_anchor_t = self.environment._terrain_config.get("max_anchor_torque", 100000000.0)
        base_anchor_f = self.environment._terrain_config.get("max_anchor_force", 100000000.0)
        strength_map = self.environment._terrain_config.get("anchor_strength_map", None)
        if strength_map and len(strength_map) > 0:
            min_t_mult = min(float(entry[3]) for entry in strength_map if len(entry) >= 4)
            min_f_mult = min(float(entry[2]) for entry in strength_map if len(entry) >= 4)
            self.torque_limit_recorded = base_anchor_t * min_t_mult
            self.force_limit_recorded = base_anchor_f * min_f_mult
        else:
            self.torque_limit_recorded = base_anchor_t
            self.force_limit_recorded = base_anchor_f
        self.internal_torque_limit_recorded = self.environment._terrain_config.get("max_internal_torque", 100000000.0)
        self.internal_force_limit_recorded = self.environment._terrain_config.get("max_internal_force", 100000000.0)
        env_anchor_f = self.environment._current_anchor_forces
        env_anchor_t = self.environment._current_anchor_torques
        env_internal_f = self.environment._current_internal_forces
        env_internal_t = self.environment._current_internal_torques
        if env_anchor_f:
            self._peak_anchor_force = max(self._peak_anchor_force, max(env_anchor_f))
        if env_anchor_t:
            self._peak_anchor_torque = max(self._peak_anchor_torque, max(env_anchor_t))
        if env_internal_f:
            self._peak_internal_force = max(self._peak_internal_force, max(env_internal_f))
        if env_internal_t:
            self._peak_internal_torque = max(self._peak_internal_torque, max(env_internal_t))
        base_anchor_f = self.environment._terrain_config.get("max_anchor_force", 100000000.0)
        base_anchor_t = self.environment._terrain_config.get("max_anchor_torque", 100000000.0)
        base_internal_f = self.environment._terrain_config.get("max_internal_force", 100000000.0)
        base_internal_t = self.environment._terrain_config.get("max_internal_torque", 100000000.0)
        strength_map = self.environment._terrain_config.get("anchor_strength_map", None)
        def _joint_limit(anchor_x, anchor_y, is_wall):
            if is_wall:
                lim_f, lim_t = base_anchor_f, base_anchor_t
                if strength_map:
                    for y_min, y_max, f_mult, t_mult in strength_map:
                        if y_min <= anchor_y <= y_max:
                            lim_f *= f_mult
                            lim_t *= t_mult
                            break
            else:
                lim_f, lim_t = base_internal_f, base_internal_t
            return lim_f, lim_t
        stress_entries = []
        global_peak_anchor_f = self._peak_anchor_force
        global_peak_anchor_t = self._peak_anchor_torque
        global_peak_internal_f = self._peak_internal_force
        global_peak_internal_t = self._peak_internal_torque
        for jid, hist in self.environment._joint_force_history.items():
            is_wall = hist.get("is_wall", False)
            ax = hist.get("anchor_x", 0.0)
            ay = hist.get("anchor_y", 0.0)
            pf = hist.get("peak_force", 0.0)
            pt = hist.get("peak_torque", 0.0)
            lim_f, lim_t = _joint_limit(ax, ay, is_wall)
            f_pct = (pf / lim_f * 100.0) if math.isfinite(lim_f) and lim_f > 0 else 0.0
            t_pct = (pt / lim_t * 100.0) if math.isfinite(lim_t) and lim_t > 0 else 0.0
            max_pct = max(f_pct, t_pct)
            stress_entries.append({
                "is_wall": is_wall, "anchor_x": ax, "anchor_y": ay,
                "peak_force": pf, "peak_torque": pt,
                "limit_force": lim_f, "limit_torque": lim_t,
                "force_pct": f_pct, "torque_pct": t_pct,
                "max_stress_pct": max_pct, "failed": False, "fail_step": -1,
                "status": "survived",
            })
            if is_wall:
                global_peak_anchor_f = max(global_peak_anchor_f, pf)
                global_peak_anchor_t = max(global_peak_anchor_t, pt)
            else:
                global_peak_internal_f = max(global_peak_internal_f, pf)
                global_peak_internal_t = max(global_peak_internal_t, pt)
        for rec in self.environment._joint_failure_records:
            is_wall = rec.get("is_wall", False)
            ax = rec.get("anchor_x", 0.0)
            ay = rec.get("anchor_y", 0.0)
            pf = rec.get("peak_force", 0.0)
            pt = rec.get("peak_torque", 0.0)
            lim_f = rec.get("limit_force", 1.0)
            lim_t = rec.get("limit_torque", 1.0)
            f_pct = (pf / lim_f * 100.0) if math.isfinite(lim_f) and lim_f > 0 else 0.0
            t_pct = (pt / lim_t * 100.0) if math.isfinite(lim_t) and lim_t > 0 else 0.0
            max_pct = max(f_pct, t_pct)
            stress_entries.append({
                "is_wall": is_wall, "anchor_x": ax, "anchor_y": ay,
                "peak_force": pf, "peak_torque": pt,
                "limit_force": lim_f, "limit_torque": lim_t,
                "force_pct": f_pct, "torque_pct": t_pct,
                "max_stress_pct": max_pct, "failed": True,
                "fail_step": rec.get("fail_step", -1),
                "status": "broken",
            })
            if is_wall:
                global_peak_anchor_f = max(global_peak_anchor_f, pf)
                global_peak_anchor_t = max(global_peak_anchor_t, pt)
            else:
                global_peak_internal_f = max(global_peak_internal_f, pf)
                global_peak_internal_t = max(global_peak_internal_t, pt)
        stress_entries.sort(key=lambda e: e["max_stress_pct"], reverse=True)
        self.global_peak_anchor_force = global_peak_anchor_f
        self.global_peak_anchor_torque = global_peak_anchor_t
        self.global_peak_internal_force = global_peak_internal_f
        self.global_peak_internal_torque = global_peak_internal_t
        self._joint_stress_ranking = stress_entries
        if len(self.environment._joints) < self.initial_joint_count:
            self.structure_broken = True
        steps_delta = step_count - self.last_step_count
        if not self.structure_broken:
            if step_count >= self.load_attach_step:
                active_steps = max(0, step_count - max(self.last_step_count, self.load_attach_step))
                self.load_1_held_steps += active_steps
            if step_count >= self.load_2_attach_step:
                active_steps = max(0, step_count - max(self.last_step_count, self.load_2_attach_step))
                self.load_2_held_steps += active_steps
        self.last_step_count = step_count
        failed = False
        failure_reason = None
        if current_mass > self.MAX_STRUCTURE_MASS:
            failed, failure_reason = True, f"Structure mass {current_mass:.2f}kg exceeds maximum {self.MAX_STRUCTURE_MASS}kg"
        if not failed and not self.design_constraints_checked and step_count == 0:
            violations = self._check_design_constraints()
            if violations:
                failed, failure_reason = True, "Design constraint violated: " + "; ".join(violations)
            self.design_constraints_checked = True
        if not failed and self.structure_broken:
            failed, failure_reason = True, "Structure integrity lost (joints or wall anchors broke)"
        if not failed and self.min_tip_y < self.min_tip_height_limit:
            failed, failure_reason = True, f"Structure sagged too much (tip y={self.min_tip_y:.2f}m < {self.min_tip_height_limit}m)"
        if not failed and step_count >= self.load_attach_step:
            if current_tip_x < self.target_reach - self.reach_tolerance:
                failed, failure_reason = True, f"Structure lost reach under load (tip x={current_tip_x:.2f}m < {self.target_reach - self.reach_tolerance}m)"
        is_end = (step_count >= max_steps - 1)
        success = False
        if is_end and not failed:
            if not self.reach_satisfied_initially:
                failed, failure_reason = True, f"Structure never reached target x={self.target_reach}m"
            elif self.load_1_held_steps < self.load_duration_steps:
                failed, failure_reason = True, f"Failed to hold first load for required duration (held {self.load_1_held_steps * TIME_STEP:.2f}s / {self.load_duration}s)"
            elif self.load_2_held_steps < self.load_duration_steps:
                failed, failure_reason = True, f"Failed to hold second load for required duration (held {self.load_2_held_steps * TIME_STEP:.2f}s / {self.load_duration}s)"
            else:
                success = True
        done = failed or (is_end and success) or (is_end and not success)
        score = 100.0 if success else 0.0
        if not done:
            score = min(current_tip_x / self.target_reach, 1.0) * 80.0
        metrics = {
            'tip_x': current_tip_x,
            'max_reach': self.max_tip_x,
            'target_reach': self.target_reach,
            'current_reach': current_tip_x,
            'load_attached': step_count >= self.load_attach_step,
            'load_hold_time': self.load_1_held_steps * TIME_STEP,
            'load2_attached': step_count >= self.load_2_attach_step,
            'load2_hold_time': self.load_2_held_steps * TIME_STEP,
            'anchor_broken': self.structure_broken,
            'min_tip_y': self.min_tip_y,
            'min_tip_height': self.min_tip_height_limit,
            'tip_sagged': self.min_tip_y < self.min_tip_height_limit,
            'external_force_y': self.external_force_y,
            'structure_mass': current_mass,
            'max_structure_mass': self.MAX_STRUCTURE_MASS,
            'peak_joint_torque': self.max_recorded_torque,
            'peak_joint_force': self.max_recorded_force,
            'max_anchor_torque_limit': self.torque_limit_recorded,
            'max_internal_torque_limit': self.internal_torque_limit_recorded,
            'max_anchor_force_limit': self.force_limit_recorded,
            'max_internal_force_limit': self.internal_force_limit_recorded,
            'peak_anchor_force': self._peak_anchor_force,
            'peak_anchor_torque': self._peak_anchor_torque,
            'peak_internal_force': self._peak_internal_force,
            'peak_internal_torque': self._peak_internal_torque,
            'first_failure_step': self.environment._first_failure_step,
            'first_warning_step': self.environment._first_warning_step,
            'joint_failure_records': list(self.environment._joint_failure_records),
            'forbidden_anchor_y': self.environment._forbidden_anchor_y,
            'wall_anchor_positions': list(self.environment._wall_anchor_positions),
            'load_attach_time': self.load_attach_time,
            'load_2_attach_time': self.load_2_attach_time,
            'load_type': self.environment._terrain_config.get("load_type", "static"),
            'anchor_count': len([j for j in self.environment._joints if j.bodyA == self.environment._terrain_bodies["wall"] or j.bodyB == self.environment._terrain_bodies["wall"]]),
            'max_anchor_points': 2,
            'max_anchors_limit': 2,
            'reach_tolerance': self.reach_tolerance,
            'joint_count': len(self.environment._joints),
            'initial_joint_count': self.initial_joint_count,
            'success': success,
            'failed': failed,
            'failure_reason': failure_reason,
            'step_count': step_count,
            'global_peak_anchor_force': self.global_peak_anchor_force,
            'global_peak_anchor_torque': self.global_peak_anchor_torque,
            'global_peak_internal_force': self.global_peak_internal_force,
            'global_peak_internal_torque': self.global_peak_internal_torque,
            'joint_stress_summary': list(self._joint_stress_ranking),
            'base_anchor_force': base_anchor_f,
            'base_anchor_torque': base_anchor_t,
            'base_internal_force': base_internal_f,
            'base_internal_torque': base_internal_t,
            'anchor_strength_map': strength_map,
            'gravity_current': list(self.environment._physics_config.get("gravity", (0, -10))),
            'spatial_force_config': self.environment._physics_config.get("spatial_force", None),
            'wind_config': self.environment._physics_config.get("wind", None),
            'drop_height': self.environment._terrain_config.get("drop_height", 0.0),
            'load_mass': float(self.environment._terrain_config.get("load_mass", 500.0)),
            'obstacle_active': self.environment._obstacle_active,
            'obstacle_rects': list(self.environment._obstacle_rects),
            'load_duration': self.load_duration,
            'time_step': TIME_STEP,
            'reach_satisfied_initially': self.reach_satisfied_initially,
            'build_zone_bounds': {
                'x_min': self.BUILD_ZONE_X_MIN,
                'x_max': self.BUILD_ZONE_X_MAX,
                'y_min': self.BUILD_ZONE_Y_MIN,
                'y_max': self.BUILD_ZONE_Y_MAX,
            },
            'joint_observation_error_count': self.environment._joint_observation_error_count,
            'last_joint_observation_error': self.environment._last_joint_observation_error,
            'joint_warning_fraction': self.environment.JOINT_WARNING_FRACTION,
        }
        return done, score, metrics
    def _check_design_constraints(self):
        violations = []
        if not self.environment: return ["Environment not available"]
        for body in self.environment._bodies:
            x, y = body.position.x, body.position.y
            if not (self.BUILD_ZONE_X_MIN <= x <= self.BUILD_ZONE_X_MAX and
                    self.BUILD_ZONE_Y_MIN <= y <= self.BUILD_ZONE_Y_MAX):
                violations.append(f"Beam at ({x:.2f}, {y:.2f}) outside build zone")
        anchor_count = len([j for j in self.environment._joints if j.bodyA == self.environment._terrain_bodies["wall"] or j.bodyB == self.environment._terrain_bodies["wall"]])
        if anchor_count > 2:
            violations.append(f"Too many wall anchors: {anchor_count} (max 2)")
        return violations
    def compute_score_with_penalty(self, score: float, metrics: dict) -> float:
        if score > 0:
            return score
        constraint_info = self.get_constraint_info()
        penalty = compute_constraint_penalty("S_03", score, metrics, constraint_info)
        return penalty
    def get_constraint_info(self):
        return {
            'target_reach': self.target_reach,
            'min_tip_height_limit': self.min_tip_height_limit,
            'load_duration': self.load_duration,
            'reach_tolerance': self.reach_tolerance,
            'max_structure_mass': self.MAX_STRUCTURE_MASS,
            'build_zone_x_min': self.BUILD_ZONE_X_MIN,
            'build_zone_x_max': self.BUILD_ZONE_X_MAX,
            'build_zone_y_min': self.BUILD_ZONE_Y_MIN,
            'build_zone_y_max': self.BUILD_ZONE_Y_MAX,
            'max_anchor_points': 2,
        }
    def get_task_description(self):
        return {
            'task': 'S-03: The Cantilever',
            'description': 'Design a structure that reaches far out and holds heavy loads',
            'success_criteria': {
                'reach': f'Tip x >= {self.target_reach}m',
                'load': f'Hold all payloads for {self.load_duration:.0f}s duration',
                'integrity': 'No joint or anchor breaks'
            }
        }
