import math

from pace_bench.core.primitives import compute_constraint_penalty

def _build_constraint_dashboard(
    load_caught, beam_angle_deg, max_angle_seen_deg, max_angle_deviation_deg,
    balance_duration, target_balance_time,
    net_torque, max_joint_torque, fragile_joints, pivot_destroyed,
    min_body_y, ground_y_limit,
    peak_angular_velocity, max_angular_velocity_limit,
    step_count, max_steps,

):
    dashboard = []
    dashboard.append({
        'name': 'Load Capture',
        'status': 'PASS' if load_caught else 'FAIL',
        'value': 'caught' if load_caught else 'not caught',
        'limit': 'caught',
        'margin': None,
        'margin_pct': None,
    })
    angle_margin = max_angle_deviation_deg - abs(beam_angle_deg)
    angle_margin_pct = (angle_margin / max_angle_deviation_deg * 100.0) if max_angle_deviation_deg > 0 else 0.0
    angle_max_margin = max_angle_deviation_deg - max_angle_seen_deg
    dashboard.append({
        'name': 'Beam Angle (current)',
        'status': 'PASS' if abs(beam_angle_deg) <= max_angle_deviation_deg else 'FAIL',
        'value': beam_angle_deg,
        'limit': max_angle_deviation_deg,
        'margin': angle_margin,
        'margin_pct': angle_margin_pct,
        'peak_value': max_angle_seen_deg,
        'peak_margin': angle_max_margin,
    })
    bal_margin = balance_duration - target_balance_time
    bal_margin_pct = (balance_duration / target_balance_time * 100.0) if target_balance_time > 0 else 0.0
    dashboard.append({
        'name': 'Balance Duration',
        'status': 'PASS' if balance_duration >= target_balance_time else 'FAIL',
        'value': balance_duration,
        'limit': target_balance_time,
        'margin': bal_margin,
        'margin_pct': bal_margin_pct,
    })
    if fragile_joints and max_joint_torque > 0:
        torque_ratio = abs(net_torque) / max_joint_torque if math.isfinite(net_torque) else float('inf')
        torque_margin = max_joint_torque - abs(net_torque)
        torque_status = 'FAIL' if pivot_destroyed or abs(net_torque) > max_joint_torque else 'PASS'
        if torque_ratio >= 0.8 and not pivot_destroyed and abs(net_torque) <= max_joint_torque:
            torque_status = 'WARN'
        dashboard.append({
            'name': 'Pivot Torque',
            'status': torque_status,
            'value': abs(net_torque),
            'limit': max_joint_torque,
            'margin': torque_margin,
            'margin_pct': (torque_margin / max_joint_torque * 100.0) if max_joint_torque > 0 else 0.0,
            'ratio': torque_ratio,
            'pivot_destroyed': pivot_destroyed,
        })
    if min_body_y is not None and math.isfinite(min_body_y):
        ground_margin = min_body_y - ground_y_limit
        ground_pct = (ground_margin / abs(ground_y_limit) * 100.0) if abs(ground_y_limit) > 1e-9 else 100.0
        dashboard.append({
            'name': 'Ground Clearance',
            'status': 'PASS' if min_body_y >= ground_y_limit else 'FAIL',
            'value': min_body_y,
            'limit': ground_y_limit,
            'margin': ground_margin,
            'margin_pct': ground_pct,
        })
    av_margin = max_angular_velocity_limit - peak_angular_velocity
    av_pct = (peak_angular_velocity / max_angular_velocity_limit * 100.0) if max_angular_velocity_limit > 0 else 100.0
    av_status = 'PASS' if peak_angular_velocity <= max_angular_velocity_limit else 'FAIL'
    if av_pct >= 70.0 and peak_angular_velocity <= max_angular_velocity_limit:
        av_status = 'WARN'
    dashboard.append({
        'name': 'Angular Velocity',
        'status': av_status,
        'value': peak_angular_velocity,
        'limit': max_angular_velocity_limit,
        'margin': av_margin,
        'margin_pct': 100.0 - av_pct,
    })
    step_margin = max_steps - step_count
    step_pct = (step_count / max_steps * 100.0) if max_steps > 0 else 100.0
    step_status = 'PASS' if step_count < max_steps else 'EXHAUSTED'
    if step_pct >= 80.0 and step_count < max_steps:
        step_status = 'WARN'
    dashboard.append({
        'name': 'Step Budget',
        'status': step_status,
        'value': step_count,
        'limit': max_steps,
        'margin': step_margin,
        'margin_pct': 100.0 - step_pct,
    })
    return dashboard

def _build_numerical_health(
    beam_angle_deg,
    net_torque,
    peak_angular_velocity,
    structure_mass,
    pivot_destroy_error_count,
    last_pivot_destroy_error,
):
    flags = []
    if not math.isfinite(beam_angle_deg):
        flags.append({'severity': 'CRITICAL', 'tag': 'beam_angle_nan', 'detail': f'beam_angle_deg = {beam_angle_deg}'})
    if not math.isfinite(net_torque):
        flags.append({'severity': 'CRITICAL', 'tag': 'net_torque_nan', 'detail': f'net_torque = {net_torque}'})
    if not math.isfinite(peak_angular_velocity):
        flags.append({'severity': 'CRITICAL', 'tag': 'angular_velocity_nan', 'detail': f'peak_angular_velocity = {peak_angular_velocity}'})
    if not math.isfinite(structure_mass):
        flags.append({'severity': 'CRITICAL', 'tag': 'structure_mass_nan', 'detail': f'structure_mass = {structure_mass}'})
    if math.isfinite(beam_angle_deg) and abs(beam_angle_deg) > 360.0:
        flags.append({'severity': 'WARNING', 'tag': 'extreme_angle', 'detail': f'beam angle {beam_angle_deg:.1f}° — simulation may be numerically unstable'})
    if math.isfinite(net_torque) and abs(net_torque) > 1e9:
        flags.append({'severity': 'WARNING', 'tag': 'extreme_torque', 'detail': f'net torque {net_torque:.2e} N·m — possible solver divergence'})
    if math.isfinite(peak_angular_velocity) and peak_angular_velocity > 100.0:
        flags.append({'severity': 'WARNING', 'tag': 'extreme_angular_velocity', 'detail': f'peak angular velocity {peak_angular_velocity:.1f} rad/s — possible instability'})
    if pivot_destroy_error_count:
        detail = f'pivot joint destruction failed {pivot_destroy_error_count} time(s)'
        if last_pivot_destroy_error:
            detail += f'; last error: {last_pivot_destroy_error}'
        flags.append({'severity': 'WARNING', 'tag': 'pivot_destroy_error', 'detail': detail})
    return flags

class Evaluator:
    def __init__(self, terrain_bounds, environment=None):
        self.terrain_bounds = terrain_bounds
        self.environment = environment
        self.max_angle_deviation = 10.0 * math.pi / 180.0
        self.balance_time = 15.0
        self.load_caught = False
        self.balance_start_time = None
        self.balance_duration = 0.0
        self.max_angle_seen = 0.0
        if not environment:
            raise ValueError("Evaluator requires environment instance")
        env_class = type(environment)
        try:
            self.MAX_ANGLE_DEVIATION = getattr(environment, 'MAX_ANGLE_DEVIATION', env_class.MAX_ANGLE_DEVIATION)
            self.BALANCE_TIME = getattr(environment, 'BALANCE_TIME', env_class.BALANCE_TIME)
            self.ground_y_limit = getattr(environment, 'GROUND_Y_FAILURE', -5.0)
            self.MAX_ANGULAR_VELOCITY = getattr(environment, 'MAX_ANGULAR_VELOCITY', 2.0)
            self.balance_time = self.BALANCE_TIME
            self.max_angle_deviation = self.MAX_ANGLE_DEVIATION
        except AttributeError as e:
            raise AttributeError(f"Environment class {env_class.__name__} missing required constant: {e}")
        self.design_constraints_checked = False
    def evaluate(self, agent_body, step_count, max_steps):
        if not self.environment:
            return False, 0.0, {"error": "Environment not available"}
        if getattr(self.environment, "_drop_load", False):
            self.load_caught = getattr(self.environment, "_load_caught_by_structure", False)
        elif "load" in self.environment._terrain_bodies:
            self.load_caught = True
        beam_angle = self.environment.get_main_beam_angle()
        angle_deviation = abs(beam_angle)
        self.max_angle_seen = max(self.max_angle_seen, angle_deviation)
        time_step = getattr(self.environment, '_last_time_step', 1.0 / 60.0)
        current_time = step_count * time_step
        if angle_deviation <= self.max_angle_deviation:
            if self.balance_start_time is None:
                self.balance_start_time = current_time
            self.balance_duration = current_time - self.balance_start_time
        else:
            self.balance_start_time = None
            self.balance_duration = 0.0
        catch_ok = self.load_caught
        balance_ok = self.balance_duration >= self.balance_time
        success = catch_ok and balance_ok
        failed = False
        failure_reason = None
        if getattr(self.environment, "_pivot_joint_destroyed", False):
            failed = True
            failure_reason = "Pivot joint snapped (static torque exceeded limit)"
        load_body = self.environment._terrain_bodies.get("load")
        if load_body and load_body.position.y < self.ground_y_limit:
            failed = True
            failure_reason = f"Load fell to the ground (y={load_body.position.y:.2f} < {self.ground_y_limit})"
        for i, body in enumerate(self.environment._bodies):
            if body.position.y < self.ground_y_limit:
                failed = True
                failure_reason = f"Structure body {i} touched ground (y={body.position.y:.2f} < {self.ground_y_limit})"
                break
        if not catch_ok and current_time > 1.0:
            failed = True
            if getattr(self.environment, "_drop_load", False):
                failure_reason = "Failed to catch the load"
            else:
                failure_reason = "Failed to catch load at (3, 5.5)"
        elif catch_ok and angle_deviation > self.max_angle_deviation and current_time > 2.0:
            failed = True
            failure_reason = f"Beam angle {angle_deviation * 180 / math.pi:.1f}° exceeds ±{self.max_angle_deviation * 180 / math.pi:.1f}° limit"
        peak_av = self.environment.get_peak_angular_velocity()
        if math.isfinite(peak_av) and peak_av > self.MAX_ANGULAR_VELOCITY and current_time > 1.0:
            failed = True
            failure_reason = f"Peak angular velocity {peak_av:.2f} rad/s exceeds limit {self.MAX_ANGULAR_VELOCITY} rad/s"
        if success and not failed:
            score = 100.0
        elif failed:
            score = 0.0
        else:
            catch_score = 50.0 if catch_ok else 0.0
            balance_score = min(self.balance_duration / self.balance_time, 1.0) * 50.0
            score = catch_score + balance_score
        structure_mass = 0.0
        com_x = 0.0
        com_y = 0.0
        min_body_y = None
        structure_torque = 0.0
        load_torque = 0.0
        body_torque_contributions = []
        body_details = []
        gx, gy = getattr(self.environment.world, "gravity", (0.0, -10.0)) if self.environment else (0.0, -10.0)
        wind_f = float(getattr(self.environment, "_wind_force_multiplier", 0.0)) if getattr(self.environment, "_wind_active", False) else 0.0
        body_identities = getattr(self.environment, "get_body_identities", lambda: {})()
        for i, body in enumerate(getattr(self.environment, "_bodies", [])):
            m = float(getattr(body, "mass", 0.0))
            structure_mass += m
            rx, ry = float(body.position.x), float(body.position.y)
            com_x += m * rx
            com_y += m * ry
            min_body_y = ry if min_body_y is None else min(min_body_y, ry)
            Fx = m * wind_f + m * gx
            Fy = m * gy
            torque = rx * Fy - ry * Fx
            structure_torque += torque
            label = body_identities.get(i, f"body_{i}")
            body_torque_contributions.append({
                'body_index': i, 'body_label': label,
                'mass': m, 'pos_x': rx, 'pos_y': ry,
                'torque': torque, 'lever_arm_x': rx, 'lever_arm_y': ry,
            })
            body_details.append({
                'body_index': i, 'body_label': label,
                'mass': m, 'pos_x': rx, 'pos_y': ry,
                'velocity_x': float(getattr(body, "linearVelocity", (0.0, 0.0))[0]),
                'velocity_y': float(getattr(body, "linearVelocity", (0.0, 0.0))[1]),
                'angular_velocity': float(getattr(body, "angularVelocity", 0.0)),
                'angle_deg': float(getattr(body, "angle", 0.0)) * 180.0 / math.pi,
            })
        if structure_mass > 1e-9:
            com_x /= structure_mass
            com_y /= structure_mass
        else:
            com_x, com_y = 0.0, 0.0
        load_body = getattr(self.environment, "_terrain_bodies", {}).get("load")
        load_mass = None
        load_pos = None
        load_contribution = None
        if load_body is not None:
            load_mass = float(getattr(load_body, "mass", 0.0))
            rx, ry = float(load_body.position.x), float(load_body.position.y)
            load_pos = (rx, ry)
            if getattr(self.environment, "_load_attached", False) or getattr(self.environment, "_drop_load", False):
                Fx = load_mass * wind_f + load_mass * gx
                Fy = load_mass * gy
                torque = rx * Fy - ry * Fx
                load_torque += torque
                load_contribution = {
                    'body_index': -1, 'body_label': 'load',
                    'mass': load_mass, 'pos_x': rx, 'pos_y': ry,
                    'torque': torque, 'lever_arm_x': rx, 'lever_arm_y': ry,
                }
        net_torque_about_pivot = structure_torque + load_torque
        total_abs_torque = sum(abs(c['torque']) for c in body_torque_contributions)
        if load_contribution is not None:
            total_abs_torque += abs(load_contribution['torque'])
        for c in body_torque_contributions:
            c['torque_pct'] = (abs(c['torque']) / total_abs_torque * 100.0) if total_abs_torque > 1e-9 else 0.0
        if load_contribution is not None:
            load_contribution['torque_pct'] = (abs(load_contribution['torque']) / total_abs_torque * 100.0) if total_abs_torque > 1e-9 else 0.0
            body_torque_contributions.append(load_contribution)
        body_torque_contributions.sort(key=lambda c: abs(c['torque']), reverse=True)
        env_max_joint_torque = float(getattr(self.environment, "_max_joint_torque", 0.0))
        env_fragile_joints = bool(getattr(self.environment, "_fragile_joints", False))
        env_wind_active = bool(getattr(self.environment, "_wind_active", False))
        env_wind_force_multiplier = float(getattr(self.environment, "_wind_force_multiplier", 0.0))
        env_obstacle_rects = getattr(self.environment, "_obstacle_world_rects", [])
        pivot_destroyed = bool(getattr(self.environment, "_pivot_joint_destroyed", False))
        pivot_destroyed_step = getattr(self.environment, "_pivot_destroyed_step", None)
        peak_load_abs_x = float(getattr(self.environment, "_peak_load_abs_x", 0.0))
        min_load_structure_dist = float(getattr(self.environment, "_min_load_structure_dist", float('inf')))
        load_caught_by_structure = bool(getattr(self.environment, "_load_caught_by_structure", False))
        drop_load_active = bool(getattr(self.environment, "_drop_load", False))
        catch_radius = 0.6 if drop_load_active else 0.5
        gravity_x = float(gx) if gx is not None else 0.0
        gravity_y = float(gy) if gy is not None else -10.0
        constraint_dashboard = _build_constraint_dashboard(
            load_caught=self.load_caught,
            beam_angle_deg=beam_angle * 180.0 / math.pi,
            max_angle_seen_deg=self.max_angle_seen * 180.0 / math.pi,
            max_angle_deviation_deg=self.max_angle_deviation * 180.0 / math.pi,
            balance_duration=self.balance_duration,
            target_balance_time=self.balance_time,
            net_torque=net_torque_about_pivot,
            max_joint_torque=env_max_joint_torque,
            fragile_joints=env_fragile_joints,
            pivot_destroyed=pivot_destroyed,
            min_body_y=min_body_y,
            ground_y_limit=self.ground_y_limit,
            peak_angular_velocity=getattr(self.environment, "get_peak_angular_velocity", lambda: 0.0)(),
            max_angular_velocity_limit=self.MAX_ANGULAR_VELOCITY,
            step_count=step_count,
            max_steps=max_steps,
        )
        numerical_health = _build_numerical_health(
            beam_angle_deg=beam_angle * 180.0 / math.pi,
            net_torque=net_torque_about_pivot,
            peak_angular_velocity=getattr(self.environment, "get_peak_angular_velocity", lambda: 0.0)(),
            structure_mass=structure_mass,
            pivot_destroy_error_count=getattr(self.environment, "_pivot_destroy_error_count", 0),
            last_pivot_destroy_error=getattr(self.environment, "_last_pivot_destroy_error", None),
        )
        metrics = {
            'load_caught': self.load_caught,
            'beam_angle_deg': beam_angle * 180 / math.pi,
            'max_angle_seen_deg': self.max_angle_seen * 180 / math.pi,
            'balance_duration': self.balance_duration,
            'target_balance_time': self.balance_time,
            'max_angle_deviation_deg': self.max_angle_deviation * 180 / math.pi,
            'ground_y_limit': self.ground_y_limit,
            'success': success and not failed,
            'failed': failed,
            'failure_reason': failure_reason,
            'step_count': step_count,
            'structure_mass': structure_mass,
            'structure_com_x': com_x,
            'structure_com_y': com_y,
            'min_body_y': min_body_y,
            'net_torque_about_pivot': net_torque_about_pivot,
            'load_mass': load_mass,
            'load_pos': load_pos,
            'structure_torque': structure_torque,
            'load_torque': load_torque,
            'gravity_x': gravity_x,
            'gravity_y': gravity_y,
            'wind_force_multiplier': env_wind_force_multiplier if env_wind_active else 0.0,
            'wind_active': env_wind_active,
            'max_joint_torque': env_max_joint_torque,
            'fragile_joints': env_fragile_joints,
            'pivot_joint_destroyed': pivot_destroyed,
            'pivot_destroyed_step': pivot_destroyed_step,
            'peak_load_abs_x': peak_load_abs_x,
            'min_load_structure_dist': min_load_structure_dist if min_load_structure_dist != float('inf') else None,
            'drop_load_active': drop_load_active,
            'load_caught_by_structure': load_caught_by_structure,
            'catch_radius': catch_radius,
            'obstacle_world_rects': env_obstacle_rects,
            'torque_contributions': body_torque_contributions,
            'body_details': body_details,
            'constraint_dashboard': constraint_dashboard,
            'event_timeline': getattr(self.environment, "get_event_timeline", lambda: [])(),
            'peak_angular_velocity': getattr(self.environment, "get_peak_angular_velocity", lambda: 0.0)(),
            'max_angular_velocity_limit': self.MAX_ANGULAR_VELOCITY,
            'body_count': len(getattr(self.environment, "_bodies", [])),
            'initial_body_count_on_load_attach': getattr(self.environment, "_initial_body_count_on_load_attach", 0),
            'joint_count_current': len(getattr(self.environment, "_joints", [])),
            'numerical_health': numerical_health,
            'pivot_destroy_error_count': getattr(self.environment, "_pivot_destroy_error_count", 0),
            'last_pivot_destroy_error': getattr(self.environment, "_last_pivot_destroy_error", None),
        }
        return success or failed, score, metrics
    def _check_design_constraints(self):
        return []
    def compute_score_with_penalty(self, score: float, metrics: dict) -> float:
        if score > 0:
            return score
        constraint_info = self.get_constraint_info()
        penalty = compute_constraint_penalty("S_04", score, metrics, constraint_info)
        return penalty
    def get_constraint_info(self):
        return {
            'max_angle_deviation': self.max_angle_deviation,
            'balance_time': self.balance_time,
            'max_angular_velocity': self.MAX_ANGULAR_VELOCITY,
            'ground_y_limit': self.ground_y_limit,
        }
    def get_task_description(self):
        angle_deg = self.max_angle_deviation * 180.0 / math.pi
        drop_load = getattr(self.environment, "_drop_load", False)
        catch = "Drop the load" if drop_load else "Connect to load at (3, 5.5)"
        return {
            'task': 'S-04: The Balancer',
            'description': 'Build a structure that balances on a pivot',
            'success_criteria': {
                'catch': catch,
                'balance': f'Keep angle within ±{angle_deg:.1f}° for {self.balance_time}s'
            }
        }
