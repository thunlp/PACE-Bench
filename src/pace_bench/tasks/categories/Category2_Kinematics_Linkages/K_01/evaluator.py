import math

from pace_bench.core.simulator import TIME_STEP

from pace_bench.core.primitives import compute_constraint_penalty

class Evaluator:
    def __init__(self, terrain_bounds, environment=None):
        self.terrain_bounds = terrain_bounds
        self.environment = environment
        self.target_distance = float(terrain_bounds.get("target_distance", 15.0))
        self.initial_x = float(terrain_bounds.get("initial_x", 10.0))
        self.min_torso_height = 1.2
        self.min_simulation_time = 15.0
        self.min_simulation_steps = int(self.min_simulation_time / TIME_STEP)
        self.max_x_reached = self.initial_x
        self.min_torso_y = 2.0
        self._first_collapse_step = None
        self._first_bz_violation_step = None
        self._max_joint_angle_abs = 0.0
        self._joint_limit_hit = False
        self._backward_displacement_count = 0
        self._min_walker_body_y = None
        self._last_x = None
        self._physics_snapshot = None
    def evaluate(self, agent_body, step_count, max_steps):
        if not self.environment:
            return (False, 0.0, {"error": "Environment not available"})
        torso = agent_body
        if torso is None:
            return (False, 0.0, {"error": "Walker torso not found: build_agent must return the torso body"})
        current_x = torso.position.x
        current_y = torso.position.y
        self.max_x_reached = max(self.max_x_reached, current_x)
        self.min_torso_y = min(self.min_torso_y, current_y)
        if self._last_x is not None:
            if current_x < self._last_x:
                self._backward_displacement_count += 1
        self._last_x = current_x
        torso_touched_ground = current_y < self.min_torso_height
        if torso_touched_ground and self._first_collapse_step is None:
            self._first_collapse_step = step_count
        joints = self.environment.get_walker_joints() if hasattr(self.environment, 'get_walker_joints') else []
        for jinfo in joints:
            angle = jinfo.get("current_angle", 0.0)
            self._max_joint_angle_abs = max(self._max_joint_angle_abs, abs(angle))
            lo = jinfo.get("lower_limit")
            hi = jinfo.get("upper_limit")
            if lo is not None and hi is not None:
                if angle <= lo or angle >= hi:
                    self._joint_limit_hit = True
        if hasattr(self.environment, 'get_walker_body_positions'):
            body_pos = self.environment.get_walker_body_positions()
            min_y = body_pos.get("min_body_y")
            if min_y is not None:
                if self._min_walker_body_y is None:
                    self._min_walker_body_y = min_y
                else:
                    self._min_walker_body_y = min(self._min_walker_body_y, min_y)
        if self._physics_snapshot is None:
            if hasattr(self.environment, 'get_environment_physics'):
                self._physics_snapshot = self.environment.get_environment_physics()
        failed = False
        failure_reason = None
        if torso_touched_ground:
            failed = True
            failure_reason = f"Walker collapsed: torso touched ground (height {current_y:.2f}m < {self.min_torso_height}m)"
        max_structure_mass = getattr(self.environment, 'MAX_STRUCTURE_MASS', 100.0)
        structure_mass_raw = self.environment.get_structure_mass()
        try:
            structure_mass = float(structure_mass_raw)
        except (TypeError, ValueError):
            structure_mass = None
        if structure_mass is not None and structure_mass > max_structure_mass:
            failed = True
            failure_reason = (
                failure_reason + "; " if failure_reason else ""
            ) + f"Design constraint violated: structure mass {structure_mass:.2f}kg exceeds budget {max_structure_mass:.1f}kg"
        build_zone = self.terrain_bounds.get("build_zone", {})
        x_range = build_zone.get("x", [0.0, 50.0])
        y_range = build_zone.get("y", [2.0, 10.0])
        x_min, x_max = float(x_range[0]), float(x_range[1])
        y_max = float(y_range[1])
        y_min = self.min_torso_height
        bz_violated = current_x < x_min or current_x > x_max or current_y < y_min or current_y > y_max
        if bz_violated and self._first_bz_violation_step is None:
            self._first_bz_violation_step = step_count
        if bz_violated:
            failed = True
            failure_reason = (
                failure_reason + "; " if failure_reason else ""
            ) + f"Build zone violated: torso at ({current_x:.2f}, {current_y:.2f}) is outside allowed region x=[{x_min}, {x_max}], y=[{y_min:.1f}, {y_max}]."
        distance_traveled = current_x - self.initial_x
        progress = min(max(0, distance_traveled) / self.target_distance, 1.0)
        success = distance_traveled >= self.target_distance and step_count >= self.min_simulation_steps
        is_end = (step_count >= max_steps - 1)
        done = failed or success or is_end
        if success and not failed:
            score = 100.0
        elif failed:
            score = 0.0
        else:
            score = progress * 70.0
            if step_count > 0:
                score += (min(step_count, self.min_simulation_steps) / self.min_simulation_steps) * 30.0
        target_x = self.initial_x + self.target_distance
        physics = self._physics_snapshot or {}
        ground_friction = physics.get("ground_friction")
        max_body_friction = physics.get("max_body_friction")
        gravity_y = physics.get("gravity_y")
        linear_damping = physics.get("linear_damping")
        angular_damping = physics.get("angular_damping")
        joint_lo = physics.get("default_joint_lower_limit")
        joint_hi = physics.get("default_joint_upper_limit")
        collapse_margin = current_y - self.min_torso_height
        min_body_y = self._min_walker_body_y if self._min_walker_body_y is not None else current_y
        ground_contact_margin = min_body_y - 1.0
        if structure_mass is not None and max_structure_mass > 0:
            mass_utilization = structure_mass / max_structure_mass
            mass_margin = max_structure_mass - structure_mass
        else:
            mass_utilization = None
            mass_margin = None
        metrics = {
            'walker_x': current_x,
            'walker_y': current_y,
            'distance_traveled': distance_traveled,
            'max_x_reached': self.max_x_reached,
            'min_torso_y': self.min_torso_y,
            'progress': progress * 100.0,
            'success': success and not failed,
            'failed': failed,
            'failure_reason': failure_reason,
            'step_count': step_count,
            'min_simulation_steps_required': self.min_simulation_steps,
            'structure_mass': structure_mass,
            'max_structure_mass': max_structure_mass,
            'target_x': target_x,
            'torso_touched_ground': torso_touched_ground,
            'ground_friction': ground_friction,
            'max_body_friction': max_body_friction,
            'gravity_y': gravity_y,
            'linear_damping': linear_damping,
            'angular_damping': angular_damping,
            'default_joint_lower_limit': joint_lo,
            'default_joint_upper_limit': joint_hi,
            'first_collapse_step': self._first_collapse_step,
            'first_bz_violation_step': self._first_bz_violation_step,
            'backward_displacement_count': self._backward_displacement_count,
            'collapse_margin': collapse_margin,
            'ground_contact_margin': ground_contact_margin,
            'mass_utilization': mass_utilization,
            'mass_margin': mass_margin,
            'max_joint_angle_abs': self._max_joint_angle_abs,
            'joint_limit_hit': self._joint_limit_hit,
            'walker_vx': float(torso.linearVelocity.x),
            'walker_vy': float(torso.linearVelocity.y),
            'torso_angular_velocity': float(torso.angularVelocity),
            'initial_x': self.initial_x,
            'target_distance_val': self.target_distance,
            'min_torso_height': self.min_torso_height,
            'ground_y': 1.0,
            'build_zone_x_min': 0.0,
            'build_zone_x_max': 50.0,
            'build_zone_y_max': 10.0,
            'body_details': self.environment.get_body_details() if hasattr(self.environment, 'get_body_details') else [],
            'wheel_ground_clearances': self.environment.get_wheel_ground_clearances() if hasattr(self.environment, 'get_wheel_ground_clearances') else [],
            'per_joint_details': self.environment.get_joint_details(inv_dt=(1.0 / TIME_STEP if TIME_STEP > 0 else 60.0)) if hasattr(self.environment, 'get_joint_details') else [],
            'num_bodies': len(self.environment._bodies) if hasattr(self.environment, '_bodies') else 0,
            'num_joints': len(self.environment._joints) if hasattr(self.environment, '_joints') else 0,
        }
        return done, score, metrics
    def compute_score_with_penalty(self, score: float, metrics: dict) -> float:
        if score > 0:
            return score
        constraint_info = self.get_constraint_info()
        penalty = compute_constraint_penalty("K_01", score, metrics, constraint_info)
        return penalty
    def get_constraint_info(self):
        return {
            'target_distance': self.target_distance,
            'initial_x': self.initial_x,
            'min_torso_height': self.min_torso_height,
            'min_simulation_time': self.min_simulation_time,
            'max_structure_mass': getattr(self.environment, 'MAX_STRUCTURE_MASS', 100.0),
        }
    def get_task_description(self):
        return {
            'task': 'K-01: The Walker',
            'success_criteria': {
                'distance': f'Travel {self.target_distance}m',
                'height': f'Keep torso y >= {self.min_torso_height}m',
                'time': f'Survive for {self.min_simulation_time}s'
            }
        }
