import math

from pace_bench.core.simulator import TIME_STEP

from pace_bench.core.primitives import compute_constraint_penalty

class Evaluator:
    def __init__(self, terrain_bounds, environment=None):
        self.environment = environment
        self.terrain_bounds = terrain_bounds
        self.max_core_force = float(self.terrain_bounds.get("core_max_force", 150.0))
        self.MAX_STRUCTURE_HEIGHT = float(self.terrain_bounds.get("max_structure_height", 7.5))
        meteor_count = int(self.terrain_bounds.get("meteor_count", 12))
        meteor_spawn_interval = int(self.terrain_bounds.get("meteor_spawn_interval", 30))
        self.min_steps = max(1000, meteor_count * meteor_spawn_interval)
        env_class = type(environment) if environment else None
        self.BUILD_ZONE_X_MIN = getattr(environment, 'BUILD_ZONE_X_MIN', getattr(env_class, 'BUILD_ZONE_X_MIN', 5.0)) if environment else 5.0
        self.BUILD_ZONE_X_MAX = getattr(environment, 'BUILD_ZONE_X_MAX', getattr(env_class, 'BUILD_ZONE_X_MAX', 15.0)) if environment else 15.0
        self.BUILD_ZONE_Y_MIN = getattr(environment, 'BUILD_ZONE_Y_MIN', getattr(env_class, 'BUILD_ZONE_Y_MIN', 0.0)) if environment else 0.0
        self.BUILD_ZONE_Y_MAX = getattr(environment, 'BUILD_ZONE_Y_MAX', getattr(env_class, 'BUILD_ZONE_Y_MAX', 8.0)) if environment else 8.0
    def evaluate(self, agent_body, step_count, max_steps):
        if not self.environment:
            return False, 0.0, {"error": "Environment not available"}
        core_force = self.environment.get_core_max_force()
        structure_mass = self.environment.get_structure_mass()
        max_mass = float(self.terrain_bounds.get("max_structure_mass", 300.0))
        failed = False
        failure_reason = None
        min_body_y = 100.0
        max_body_y = None
        if self.environment._bodies:
            for body in self.environment._bodies:
                min_body_y = min(min_body_y, body.position.y)
                max_body_y = body.position.y if max_body_y is None else max(max_body_y, body.position.y)
        max_joint_force_limit = float(self.terrain_bounds.get("max_joint_force", 1e12))
        max_joint_torque_limit = float(self.terrain_bounds.get("max_joint_torque", 1e12))
        if max_joint_force_limit < 1e11 or max_joint_torque_limit < 1e11:
            force_seen = getattr(self.environment, "_max_reaction_force_seen", 0.0)
            torque_seen = getattr(self.environment, "_max_reaction_torque_seen", 0.0)
            if force_seen > max_joint_force_limit or torque_seen > max_joint_torque_limit:
                reasons = []
                if max_joint_force_limit < 1e11 and force_seen > max_joint_force_limit:
                    reasons.append(f"joint force {force_seen:.1f}N > {max_joint_force_limit:.1f}N limit")
                if max_joint_torque_limit < 1e11 and torque_seen > max_joint_torque_limit:
                    reasons.append(f"joint torque {torque_seen:.1f}Nm > {max_joint_torque_limit:.1f}Nm limit")
                failed, failure_reason = True, "Joint limit exceeded: " + "; ".join(reasons) if reasons else "joint limit breached"
        if not failed and min_body_y < 0.3:
            failed, failure_reason = True, "Shelter collapsed or fell below ground level"
        elif not failed and core_force > self.max_core_force:
            failed, failure_reason = True, f"Core protection failed: force {core_force:.1f}N > {self.max_core_force}N"
        elif not failed and structure_mass > max_mass:
            failed, failure_reason = True, f"Mass budget exceeded: {structure_mass:.1f}kg > {max_mass}kg"
        if not failed:
            for body in self.environment._bodies:
                if body.position.y > self.MAX_STRUCTURE_HEIGHT:
                    failed, failure_reason = True, f"Structure exceeds height limit {self.MAX_STRUCTURE_HEIGHT}m"
                    break
        is_end = (step_count >= max_steps - 1)
        can_eval_success = (step_count >= self.min_steps)
        success = can_eval_success and not failed
        done = failed or (is_end and can_eval_success)
        score = 100.0 if success else 0.0
        if not done and not failed:
            score = (min(step_count, self.min_steps) / self.min_steps) * 80.0
        joint_limit_force = float(self.terrain_bounds.get("max_joint_force", 1e12))
        joint_limit_torque = float(self.terrain_bounds.get("max_joint_torque", 1e12))
        first_breach_step = None
        first_breach_anchor = None
        joints_broken_count = 0
        joint_peak_records = []
        numerical_instability_count = 0
        core_dodge_vs_collapse = None
        env = self.environment
        if env is not None:
            if hasattr(env, 'get_first_joint_breach_step'):
                first_breach_step = env.get_first_joint_breach_step()
            if hasattr(env, 'get_first_joint_breach_anchor'):
                first_breach_anchor = env.get_first_joint_breach_anchor()
            if hasattr(env, 'get_joints_broken_count'):
                joints_broken_count = env.get_joints_broken_count()
            if hasattr(env, 'get_joint_peak_records'):
                joint_peak_records = env.get_joint_peak_records()
            if hasattr(env, 'get_numerical_instability_count'):
                numerical_instability_count = env.get_numerical_instability_count()
            if core_force == 0.0 and joints_broken_count > 0:
                core_dodge_vs_collapse = "collapsed"
            elif core_force == 0.0 and joints_broken_count == 0:
                core_dodge_vs_collapse = "dodged"
        metrics = {
            'core_force': core_force,
            'max_core_force': self.max_core_force,
            'core_x': self.environment.CORE_X if self.environment else 0,
            'core_y': self.environment.CORE_Y if self.environment else 0,
            'meteor_count': self.environment._meteor_count if self.environment else 0,
            'structure_mass': structure_mass,
            'max_mass': max_mass,
            'max_height_limit': self.MAX_STRUCTURE_HEIGHT,
            'min_body_y': min_body_y,
            'success': success,
            'failed': failed,
            'failure_reason': failure_reason,
            'max_joint_force_seen': getattr(self.environment, '_max_reaction_force_seen', None),
            'max_joint_torque_seen': getattr(self.environment, '_max_reaction_torque_seen', None),
            'joint_limit_force': joint_limit_force if joint_limit_force < 1e11 else None,
            'joint_limit_torque': joint_limit_torque if joint_limit_torque < 1e11 else None,
            'joint_breach_step': first_breach_step,
            'joint_breach_anchor': first_breach_anchor,
            'joints_broken_count': joints_broken_count,
            'joint_peak_records': joint_peak_records,
            'joint_peak_records_v2': env.get_joint_peak_records_v2() if hasattr(env, 'get_joint_peak_records_v2') else [],
            'numerical_instability_count': numerical_instability_count,
            'core_dodge_vs_collapse': core_dodge_vs_collapse,
            'joint_failure_events': env.get_joint_breach_events() if env and hasattr(env, 'get_joint_breach_events') else [],
            'total_boulder_ke': env.get_total_boulder_ke() if env and hasattr(env, 'get_total_boulder_ke') else 0.0,
            'max_body_velocity': env.get_max_body_velocity() if env and hasattr(env, 'get_max_body_velocity') else 0.0,
            'core_force_step': env.get_core_force_step() if env and hasattr(env, 'get_core_force_step') else None,
            'collapse_threshold': 0.3,
            'max_body_y': max_body_y,
            'joint_observation_errors': env.get_joint_observation_errors() if hasattr(env, 'get_joint_observation_errors') else {},
        }
        return done, score, metrics
    def compute_score_with_penalty(self, score: float, metrics: dict) -> float:
        if score > 0:
            return score
        constraint_info = self.get_constraint_info()
        penalty = compute_constraint_penalty("S_05", score, metrics, constraint_info)
        return penalty
    def get_constraint_info(self):
        return {
            'max_core_force': self.max_core_force,
            'max_structure_height': self.MAX_STRUCTURE_HEIGHT,
            'max_structure_mass': float(self.terrain_bounds.get("max_structure_mass", 300.0)),
            'build_zone_x_min': self.BUILD_ZONE_X_MIN,
            'build_zone_x_max': self.BUILD_ZONE_X_MAX,
            'build_zone_y_min': self.BUILD_ZONE_Y_MIN,
            'build_zone_y_max': self.BUILD_ZONE_Y_MAX,
            'max_joint_force': float(self.terrain_bounds.get("max_joint_force", 1e12)),
            'max_joint_torque': float(self.terrain_bounds.get("max_joint_torque", 1e12)),
        }
    def get_task_description(self):
        return {
            'task': 'S-05: The Shelter',
            'success_criteria': {
                'protection': f'Core receives < {self.max_core_force}N force',
                'stability': 'Shelter does not collapse',
                'height_limit': f'No beam above y={self.MAX_STRUCTURE_HEIGHT}m',
                'mass_limit': f'Structure mass < {self.terrain_bounds.get("max_structure_mass", 300.0)}kg',
                'core_force_limit': float(self.max_core_force),
            }
        }
