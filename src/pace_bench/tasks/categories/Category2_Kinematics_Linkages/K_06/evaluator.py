import math

from pace_bench.core.simulator import TIME_STEP

from pace_bench.core.primitives import compute_constraint_penalty

class Evaluator:
    def __init__(self, terrain_bounds, environment=None):
        self.terrain_bounds = terrain_bounds
        self.environment = environment
        self.min_removal_ratio = 0.8
        self.min_simulation_time = 8.0
        self.min_simulation_steps = int(self.min_simulation_time / TIME_STEP)
        self._forensic_snapshots = []
        if self.environment and hasattr(self.environment, 'reset_forensic_state'):
            self.environment.reset_forensic_state()
    def evaluate(self, agent_body, step_count, max_steps):
        if not self.environment:
            return (False, 0.0, {"error": "Environment not available"})
        if hasattr(self.environment, 'update_forensic_state'):
            self.environment.update_forensic_state(step_count)
        initial_count = self.environment.get_initial_particle_count()
        remaining_count = self.environment.get_remaining_particle_count()
        if initial_count == 0:
            return (False, 0.0, {"error": "No particles found"})
        removal_ratio = (initial_count - remaining_count) / initial_count
        failed = False
        failure_reason = None
        max_mass = getattr(self.environment, 'MAX_STRUCTURE_MASS', 15.0)
        structure_mass = self.environment.get_structure_mass()
        if structure_mass >= max_mass:
            failed = True
            failure_reason = f"Structure exceeds mass budget: {structure_mass:.2f} kg >= {max_mass:.2f} kg limit."
        if failure_reason is None and self.terrain_bounds and "build_zone" in self.terrain_bounds:
            bz = self.terrain_bounds["build_zone"]
            x_range = bz.get("x", [0.0, 12.0])
            y_range = bz.get("y", [2.0, 10.0])
            x_min, x_max = x_range[0], x_range[1]
            y_min, y_max = y_range[0], y_range[1]
            for body in (self.environment.bodies or []):
                px, py = body.position.x, body.position.y
                if not (math.isfinite(px) and math.isfinite(py)):
                    continue
                if not (x_min <= px <= x_max and y_min <= py <= y_max):
                    failed = True
                    failure_reason = f"Structure left build zone: body at ({px:.2f}, {py:.2f}) outside x=[{x_min}, {x_max}], y=[{y_min}, {y_max}]."
                    break
        wiper = agent_body
        if wiper is None and getattr(self.environment, 'bodies', None):
            wiper = self.environment.bodies[0]
        wiper_x, wiper_y = 0.0, 0.0
        if wiper:
            wiper_x = wiper.position.x
            wiper_y = wiper.position.y
        is_end = (step_count >= max_steps - 1)
        if is_end and removal_ratio < self.min_removal_ratio:
            failed = True
            if failure_reason is None:
                failure_reason = f"Wiper failed: too many particles remaining ({remaining_count}/{initial_count}). Only {removal_ratio*100:.1f}% removed (need {self.min_removal_ratio*100:.0f}%)"
        success = removal_ratio >= self.min_removal_ratio and step_count >= self.min_simulation_steps
        done = failed or success or is_end
        if success and not failed:
            score = 100.0
            progress = 1.0
        elif failed:
            score = 0.0
            progress = 0.0
        else:
            progress = min(removal_ratio / self.min_removal_ratio, 1.0)
            score = progress * 70.0
            if step_count > 0:
                score += (min(step_count, self.min_simulation_steps) / self.min_simulation_steps) * 30.0
        metrics = {
            'wiper_x': wiper_x,
            'wiper_y': wiper_y,
            'initial_particle_count': initial_count,
            'current_particle_count': remaining_count,
            'particles_removed': initial_count - remaining_count,
            'cleaning_percentage': removal_ratio * 100.0,
            'residual_percentage': (1.0 - removal_ratio) * 100.0,
            'max_residual_percent': (1.0 - self.min_removal_ratio) * 100.0,
            'progress': progress * 100.0,
            'success': success and not failed,
            'failed': failed,
            'failure_reason': failure_reason,
            'step_count': step_count,
            'time_step': TIME_STEP,
            'min_simulation_steps_required': self.min_simulation_steps,
            'structure_mass': self.environment.get_structure_mass(),
            'max_structure_mass': getattr(self.environment, 'MAX_STRUCTURE_MASS', 15.0),
        }
        if hasattr(self.environment, 'get_forensic_snapshot'):
            snap = self.environment.get_forensic_snapshot(step_count)
            self._forensic_snapshots.append(snap)
            metrics['forensic_snapshot'] = snap
            metrics['forensic_snapshots'] = list(self._forensic_snapshots)
        env = self.environment
        if hasattr(env, 'get_joint_angle_info'):
            metrics['joint_angle_summary'] = env.get_joint_angle_info()
        if hasattr(env, 'get_motor_energy'):
            metrics['motor_energy_joules'] = env.get_motor_energy()
        if hasattr(env, 'get_body_velocity_warnings'):
            metrics['body_velocity_warnings'] = env.get_body_velocity_warnings()
        if hasattr(env, 'get_peak_body_velocity'):
            metrics['peak_body_velocity'] = env.get_peak_body_velocity()
        if hasattr(env, 'get_particle_positions_on_glass'):
            metrics['particle_positions'] = env.get_particle_positions_on_glass()
        if hasattr(env, 'get_violation_info'):
            metrics['violation_info'] = env.get_violation_info()
        if hasattr(env, 'BUILD_ZONE_X_MIN'):
            metrics['build_zone_x_min'] = env.BUILD_ZONE_X_MIN
            metrics['build_zone_x_max'] = env.BUILD_ZONE_X_MAX
            metrics['build_zone_y_min'] = env.BUILD_ZONE_Y_MIN
            metrics['build_zone_y_max'] = env.BUILD_ZONE_Y_MAX
        if hasattr(env, '_glass_y'):
            metrics['glass_y'] = env._glass_y
        snap = metrics.get('forensic_snapshot') or {}
        metrics['torque_adequacy'] = {
            'torque_cap_nm': snap.get('max_motor_torque_cap'),
            'note': 'no torque cap configured' if snap.get('max_motor_torque_cap') is None else None,
        }
        metrics['constraint_profile'] = self._build_constraint_profile(metrics)
        metrics['temporal_events'] = self._build_temporal_events(metrics)
        metrics['removal_rate_analysis'] = self._compute_removal_rate_analysis(metrics)
        return done, score, metrics
    def _compute_torque_adequacy(self, metrics: dict) -> dict:
        result = {}
        try:
            snap = metrics.get('forensic_snapshot') or {}
            torque_cap = snap.get('max_motor_torque_cap')
            if torque_cap is None:
                result['note'] = 'no torque cap configured'
                return result
            torque_cap = float(torque_cap)
            min_x = snap.get('structure_min_x')
            max_x = snap.get('structure_max_x')
            span_x = snap.get('structure_span_x')
            if span_x is not None and span_x > 0:
                lever_arm = float(span_x) / 2.0
            elif min_x is not None and max_x is not None:
                lever_arm = (float(max_x) - float(min_x)) / 2.0
            else:
                lever_arm = 1.0
            if lever_arm > 0.001:
                tip_force_available = torque_cap / lever_arm
            else:
                tip_force_available = 0.0
            particle_props = metrics.get('particle_properties') or {}
            particle_mass = float(particle_props.get('mass', 0.15))
            particle_friction = float(particle_props.get('friction', 0.35))
            g = 10.0
            force_per_particle = particle_mass * g * particle_friction
            initial_count = metrics.get('initial_particle_count', 45)
            if span_x is not None and snap.get('build_zone_x'):
                glass_width = float(snap['build_zone_x'][1]) - float(snap['build_zone_x'][0])
                if glass_width > 0:
                    sweep_fraction = min(float(span_x) / glass_width, 1.0)
                    particles_in_sweep = max(1, int(initial_count * sweep_fraction))
                else:
                    particles_in_sweep = initial_count
            else:
                particles_in_sweep = initial_count
            total_force_required = force_per_particle * particles_in_sweep
            if tip_force_available > 0:
                adequacy_ratio = tip_force_available / max(total_force_required, 0.001)
            else:
                adequacy_ratio = 0.0
            result = {
                'torque_cap_nm': round(torque_cap, 2),
                'lever_arm_m': round(lever_arm, 3),
                'tip_force_available_n': round(tip_force_available, 3),
                'force_per_particle_n': round(force_per_particle, 3),
                'estimated_particles_in_sweep': particles_in_sweep,
                'total_force_required_n': round(total_force_required, 3),
                'adequacy_ratio': round(adequacy_ratio, 3),
                'deficit_pct': round(max(0, (1.0 - adequacy_ratio) * 100.0), 1),
            }
            if adequacy_ratio < 0.5:
                result['severity'] = 'critical'
            elif adequacy_ratio < 0.8:
                result['severity'] = 'elevated'
            elif adequacy_ratio < 1.0:
                result['severity'] = 'marginal'
            else:
                result['severity'] = 'adequate'
        except Exception:
            result['error'] = 'torque adequacy computation failed'
        return result
    def _build_constraint_profile(self, metrics: dict) -> list:
        profile = []
        mass = metrics.get('structure_mass')
        max_mass = metrics.get('max_structure_mass')
        if mass is not None and max_mass is not None:
            try:
                m, mm = float(mass), float(max_mass)
                margin = mm - m
                profile.append({
                    'constraint': 'Mass budget',
                    'status': 'PASS' if margin > 0 else 'FAIL',
                    'current': f'{m:.3f} kg',
                    'limit': f'{mm:.2f} kg',
                    'margin': f'{margin:+.3f} kg',
                    'utilization_pct': round(m / mm * 100.0, 1) if mm > 0 else 0.0,
                    'phase': 'build-time',
                })
            except (TypeError, ValueError):
                pass
        snap = metrics.get('forensic_snapshot') or {}
        bz_x = snap.get('build_zone_x', [0.0, 12.0])
        bz_y = snap.get('build_zone_y', [2.0, 10.0])
        x_min_m = snap.get('x_min_margin')
        x_max_m = snap.get('x_max_margin')
        y_min_m = snap.get('y_min_margin')
        y_max_m = snap.get('y_max_margin')
        all_margins = [x_min_m, x_max_m, y_min_m, y_max_m]
        bz_ok = all(m is None or m >= 0 for m in all_margins)
        if any(m is not None for m in all_margins):
            min_margin = min([m for m in all_margins if m is not None], default=0.0)
            profile.append({
                'constraint': 'Build zone (all components)',
                'status': 'PASS' if bz_ok else 'FAIL',
                'current': f'x:[{snap.get("structure_min_x","?")}, {snap.get("structure_max_x","?")}], y:[{snap.get("structure_min_y","?")}, {snap.get("structure_max_y","?")}]',
                'limit': f'x:[{bz_x[0]}, {bz_x[1]}], y:[{bz_y[0]}, {bz_y[1]}]',
                'margin': f'tightest: {min_margin:+.3f} m',
                'phase': 'build-time + runtime',
            })
        clean_pct = metrics.get('cleaning_percentage')
        max_res = metrics.get('max_residual_percent')
        if clean_pct is not None and max_res is not None:
            try:
                cp, mr = float(clean_pct), float(max_res)
                required_clean = 100.0 - mr
                clean_margin = cp - required_clean
                profile.append({
                    'constraint': 'Cleaning (≥80% removed)',
                    'status': 'PASS' if clean_margin >= 0 else 'FAIL',
                    'current': f'{cp:.1f}% removed',
                    'limit': f'{required_clean:.0f}% removed',
                    'margin': f'{clean_margin:+.1f}%',
                    'phase': 'runtime (end)',
                })
            except (TypeError, ValueError):
                pass
        steps = metrics.get('step_count')
        min_steps = metrics.get('min_simulation_steps_required')
        if steps is not None and min_steps is not None:
            try:
                s, ms = int(steps), int(min_steps)
                dur_margin = s - ms
                profile.append({
                    'constraint': 'Motion duration (≥8.0s)',
                    'status': 'PASS' if dur_margin >= 0 else 'FAIL',
                    'current': f'{s} steps ({s * TIME_STEP:.1f}s)',
                    'limit': f'{ms} steps ({ms * TIME_STEP:.1f}s)',
                    'margin': f'{dur_margin:+d} steps',
                    'phase': 'runtime (end)',
                })
            except (TypeError, ValueError):
                pass
        torque_cap = snap.get('max_motor_torque_cap')
        torque_req = snap.get('last_torque_requested')
        torque_capped = snap.get('torque_capped', False)
        if torque_cap is not None:
            if torque_capped:
                profile.append({
                    'constraint': 'Motor torque limit',
                    'status': 'CAPPED',
                    'current': f'requested {torque_req:.1f} N·m' if torque_req else '—',
                    'limit': f'{torque_cap:.1f} N·m',
                    'margin': f'over-requested by {torque_req - torque_cap:.1f} N·m' if torque_req else '—',
                    'phase': 'runtime',
                })
            else:
                profile.append({
                    'constraint': 'Motor torque limit',
                    'status': 'PASS',
                    'current': f'requested {torque_req:.1f} N·m' if torque_req else '—',
                    'limit': f'{torque_cap:.1f} N·m',
                    'margin': f'{torque_cap - (torque_req or 0.0):.1f} N·m remaining',
                    'phase': 'runtime',
                })
        ja_summary = metrics.get('joint_angle_summary') or []
        if ja_summary:
            for entry in ja_summary:
                lower = entry.get('lower_limit_rad')
                upper = entry.get('upper_limit_rad')
                angle = entry.get('angle_rad')
                if lower is not None and upper is not None and angle is not None:
                    margin_lower = angle - lower
                    margin_upper = upper - angle
                    tightest = min(margin_lower, margin_upper)
                    status = 'PASS'
                    if tightest < 0:
                        status = 'FAIL'
                    elif tightest < 0.1:
                        status = 'NEAR-LIMIT'
                    profile.append({
                        'constraint': f'Joint #{entry.get("joint_index", "?")} angle limits',
                        'status': status,
                        'current': f'{math.degrees(angle):.1f}° ({angle:.3f} rad)',
                        'limit': f'[{math.degrees(lower):.1f}°, {math.degrees(upper):.1f}°]',
                        'margin': f'{math.degrees(tightest):.1f}° to nearest limit',
                        'phase': 'runtime',
                    })
                    break
        return profile
    def _build_temporal_events(self, metrics: dict) -> list:
        events = []
        snap = metrics.get('forensic_snapshot') or {}
        all_snaps = metrics.get('forensic_snapshots') or []
        viol_info = metrics.get('violation_info') or snap.get('violation_info')
        if viol_info:
            events.append({
                'event': 'build_zone_violation',
                'step': viol_info.get('step', '?'),
                'detail': f'Body at ({viol_info.get("body_x", "?")}, {viol_info.get("body_y", "?")}) outside build zone x:[{viol_info.get("build_zone_x", ["?","?"])[0]}, {viol_info.get("build_zone_x", ["?","?"])[1]}], y:[{viol_info.get("build_zone_y", ["?","?"])[0]}, {viol_info.get("build_zone_y", ["?","?"])[1]}]',
                'severity': 'critical',
            })
        failure_reason = metrics.get('failure_reason') or ''
        if 'mass budget' in failure_reason.lower() or 'exceeds mass' in failure_reason.lower():
            events.append({
                'event': 'mass_budget_exceeded',
                'step': 0,
                'detail': failure_reason,
                'severity': 'critical',
            })
        if snap.get('torque_capped'):
            events.append({
                'event': 'motor_torque_capped',
                'step': snap.get('step', '?'),
                'detail': f'Requested {snap.get("last_torque_requested", "?")} N·m, capped at {snap.get("max_motor_torque_cap", "?")} N·m',
                'severity': 'elevated',
            })
        removal_rate = metrics.get('removal_rate_analysis') or {}
        if removal_rate.get('saturated'):
            events.append({
                'event': 'removal_saturated',
                'step': removal_rate.get('saturation_step', '?'),
                'detail': removal_rate.get('saturation_detail', 'Removal rate dropped significantly'),
                'severity': 'elevated',
            })
        if snap.get('numerical_nan_detected'):
            events.append({
                'event': 'numerical_instability',
                'step': snap.get('step', '?'),
                'detail': 'NaN or extreme velocity detected in simulation',
                'severity': 'critical',
            })
        velocity_warnings = metrics.get('body_velocity_warnings') or []
        for vw in velocity_warnings[:3]:
            events.append({
                'event': f'velocity_{vw.get("issue", "warning")}',
                'step': vw.get('step', '?'),
                'detail': f'Body #{vw.get("body_index", "?")} at ({vw.get("px", "?")}, {vw.get("py", "?")}), speed={vw.get("speed", "?")} m/s',
                'severity': 'critical' if vw.get('issue') == 'extreme_velocity' else 'elevated',
            })
        events.sort(key=lambda e: e.get('step', 0) if isinstance(e.get('step', 0), (int, float)) else 0)
        return events
    def _compute_removal_rate_analysis(self, metrics: dict) -> dict:
        result = {}
        snap = metrics.get('forensic_snapshot') or {}
        traj = snap.get('removal_trajectory') or []
        if not traj or len(traj) < 2:
            result['note'] = 'insufficient removal data'
            return result
        initial_count = snap.get('initial_particle_count', 0)
        if initial_count == 0:
            return result
        rates = []
        for i in range(1, len(traj)):
            s_prev, r_prev = traj[i - 1]
            s_curr, r_curr = traj[i]
            ds = s_curr - s_prev
            if ds > 0:
                dr = r_curr - r_prev
                rate = dr / ds
                rates.append((s_curr, rate, r_curr))
        if not rates:
            return result
        initial_rate = rates[0][1] if rates else 0
        final_rate = rates[-1][1] if rates else 0
        if initial_rate > 0.0001 and final_rate < initial_rate * 0.1:
            sat_step = None
            for s, rate, _ in rates:
                if rate < initial_rate * 0.25:
                    sat_step = s
                    break
            pct_at_sat = rates[-1][2] / initial_count * 100.0 if initial_count > 0 else 0
            result = {
                'saturated': True,
                'initial_rate': round(initial_rate, 6),
                'final_rate': round(final_rate, 6),
                'rate_ratio_final_to_initial': round(final_rate / max(initial_rate, 0.000001), 4),
                'saturation_step': sat_step,
                'particles_removed_at_saturation': rates[-1][2],
                'pct_removed_at_saturation': round(pct_at_sat, 1),
                'max_steps': metrics.get('step_count', 0),
                'saturation_detail': f'Removal rate dropped from {initial_rate:.5f} to {final_rate:.5f} particles/step. Saturation at step ~{sat_step} with {pct_at_sat:.1f}% removed.',
            }
        else:
            result = {
                'saturated': False,
                'initial_rate': round(initial_rate, 6),
                'final_rate': round(final_rate, 6),
            }
        return result
    def compute_score_with_penalty(self, score: float, metrics: dict) -> float:
        if score > 0:
            return score
        constraint_info = self.get_constraint_info()
        penalty = compute_constraint_penalty("K_06", score, metrics, constraint_info)
        return penalty
    def get_constraint_info(self):
        return {
            'min_removal_ratio': self.min_removal_ratio,
            'min_simulation_time': self.min_simulation_time,
            'max_structure_mass': getattr(self.environment, 'MAX_STRUCTURE_MASS', 15.0),
        }
    def get_task_description(self):
        return {
            'task': 'K-06: The Wiper',
            'success_criteria': {
                'removal': f'Remove {self.min_removal_ratio*100:.0f}% of particles',
                'time': f'Wipe for {self.min_simulation_time}s'
            }
        }
