import math

from pace_bench.core.simulator import TIME_STEP

from pace_bench.core.primitives import compute_constraint_penalty

class Evaluator:
    def __init__(self, terrain_bounds, environment=None):
        self.terrain_bounds = terrain_bounds
        self.environment = environment
        gap_info = terrain_bounds.get("gap", {})
        right_cliff_start = float(gap_info.get("x_end", 25.0))
        self.target_x = right_cliff_start + 5.0
        left_cliff = terrain_bounds.get("left_cliff", {})
        self._cliff_top_y = float(left_cliff.get("y", 10.0))
        gap_start_x = float(gap_info.get("x_start", 10.0))
        gap_width = float(gap_info.get("width", 15.0))
        self.stall_threshold_x = gap_start_x + gap_width / 3.0
        self.max_vertical_acceleration = 2.0 * 9.8
        self.high_angular_velocity_count = 0
        self.MAX_ANGULAR_VELOCITY = 2.0
        self.STABILITY_CHECK_START_STEP = 200
        self.UNSTABLE_THRESHOLD = 5
        self.MAX_AIRBORNE_ROTATION = math.pi
        self.AIRBORNE_THRESHOLD = 0.5
        self._rotation_tracking_initialized = False
        self.vehicle_previous_velocity_y = 0.0
        self._last_eval_step_count = 0
        self.max_vertical_accel_seen = 0.0
        self.max_vertical_accel_step = 0
        self.initial_joint_count = 0
        self.structure_broken = False
        self.evaluation_sample_count = 0
        self.best_vehicle_x = None
        self.best_vehicle_x_step = None
        self.best_vehicle_y_at_progress = None
        self.min_vehicle_y = None
        self.min_vehicle_y_step = None
        self.min_vehicle_x_at_min_y = None
        self.min_structure_y = None
        self.min_structure_y_step = None
        self.min_structure_x_at_min_y = None
        self.min_structure_body_index = None
        self.max_abs_angle = 0.0
        self.max_abs_angle_step = 0
        self.max_high_angular_velocity_count = 0
        self.first_high_angular_velocity_step = None
        self.first_chassis_fail_zone_sample_step = None
        self.first_structure_fail_zone_sample_step = None
        self.max_airborne_rotation_seen = 0.0
        self.max_airborne_rotation_step = 0
        if not environment:
            raise ValueError("Evaluator requires environment instance")
        env_class = type(environment)
        self.MAX_STRUCTURE_MASS = getattr(environment, 'MAX_STRUCTURE_MASS', getattr(env_class, 'MAX_STRUCTURE_MASS', 2000.0))
        self.BUILD_ZONE_X_MIN = getattr(environment, 'BUILD_ZONE_X_MIN', getattr(env_class, 'BUILD_ZONE_X_MIN', 10.0))
        self.BUILD_ZONE_X_MAX = getattr(environment, 'BUILD_ZONE_X_MAX', getattr(env_class, 'BUILD_ZONE_X_MAX', 30.0))
        self.BUILD_ZONE_Y_MIN = getattr(environment, 'BUILD_ZONE_Y_MIN', getattr(env_class, 'BUILD_ZONE_Y_MIN', 5.0))
        self.BUILD_ZONE_Y_MAX = getattr(environment, 'BUILD_ZONE_Y_MAX', getattr(env_class, 'BUILD_ZONE_Y_MAX', 15.0))
        bounds = getattr(environment, 'get_terrain_bounds', lambda: {})()
        self.fail_zone_y = float(bounds.get("fail_zone_y", 0.5))
    def evaluate(self, agent_body, step_count, max_steps):
        if not self.environment:
            return False, 0.0, {"error": "Environment not available"}
        vehicle_pos = self.environment.get_vehicle_position()
        vehicle_vel = self.environment.get_vehicle_velocity()
        if vehicle_pos is None or vehicle_vel is None:
            return False, 0.0, {"error": "Vehicle not found"}
        current_x, current_y = vehicle_pos
        velocity_x, velocity_y = vehicle_vel
        vehicle_chassis = self.environment._terrain_bodies.get("vehicle_chassis")
        angular_velocity = vehicle_chassis.angularVelocity if vehicle_chassis else 0.0
        angle = vehicle_chassis.angle if vehicle_chassis else 0.0
        normalized_angle = (angle + math.pi) % (2 * math.pi) - math.pi
        body_positions = (
            self.environment.get_body_positions_and_angles()
            if hasattr(self.environment, "get_body_positions_and_angles")
            else []
        )
        steps_delta = step_count - self._last_eval_step_count
        actual_time_step = steps_delta * TIME_STEP if steps_delta > 0 else TIME_STEP
        if step_count > 0 and steps_delta > 0:
            vertical_accel = abs(velocity_y - self.vehicle_previous_velocity_y) / actual_time_step
            if vertical_accel > self.max_vertical_accel_seen:
                self.max_vertical_accel_seen = vertical_accel
                self.max_vertical_accel_step = step_count
        self._last_eval_step_count = step_count
        self.vehicle_previous_velocity_y = velocity_y
        self.evaluation_sample_count += 1
        if self.best_vehicle_x is None or current_x > self.best_vehicle_x:
            self.best_vehicle_x = current_x
            self.best_vehicle_x_step = step_count
            self.best_vehicle_y_at_progress = current_y
        if self.min_vehicle_y is None or current_y < self.min_vehicle_y:
            self.min_vehicle_y = current_y
            self.min_vehicle_y_step = step_count
            self.min_vehicle_x_at_min_y = current_x
        finite_structure_positions = [
            (idx, float(position[0]), float(position[1]))
            for idx, position in enumerate(body_positions)
            if len(position) >= 2
            and math.isfinite(float(position[0]))
            and math.isfinite(float(position[1]))
        ]
        sample_min_structure_y = None
        if finite_structure_positions:
            body_idx, structure_x, sample_min_structure_y = min(
                finite_structure_positions, key=lambda item: item[2]
            )
            if (
                self.min_structure_y is None
                or sample_min_structure_y < self.min_structure_y
            ):
                self.min_structure_y = sample_min_structure_y
                self.min_structure_y_step = step_count
                self.min_structure_x_at_min_y = structure_x
                self.min_structure_body_index = body_idx
        abs_angle = abs(normalized_angle)
        if abs_angle > self.max_abs_angle:
            self.max_abs_angle = abs_angle
            self.max_abs_angle_step = step_count
        if (
            current_y <= self.fail_zone_y
            and self.first_chassis_fail_zone_sample_step is None
        ):
            self.first_chassis_fail_zone_sample_step = step_count
        if (
            sample_min_structure_y is not None
            and sample_min_structure_y <= self.fail_zone_y
            and self.first_structure_fail_zone_sample_step is None
        ):
            self.first_structure_fail_zone_sample_step = step_count
        if step_count == 0:
            self.initial_joint_count = len(self.environment._joints)
        if (
            len(self.environment._joints) < self.initial_joint_count
            or self.environment.get_joint_failure_events()
        ):
            self.structure_broken = True
        success = current_x >= self.target_x
        failed = False
        failure_reason = None
        violations = self._check_design_constraints()
        if violations:
            failed = True
            failure_reason = "Design constraint violated: " + "; ".join(violations)
        state_values = (
            current_x,
            current_y,
            velocity_x,
            velocity_y,
            angular_velocity,
            angle,
            self.max_vertical_accel_seen,
        )
        if not failed and not all(math.isfinite(value) for value in state_values):
            failed, failure_reason = True, "Non-finite vehicle state detected"
        if not failed and current_y <= self.fail_zone_y:
            failed, failure_reason = True, "Vehicle fell into water"
        elif not failed and any(
            body.position.y <= self.fail_zone_y for body in self.environment._bodies
        ):
            failed, failure_reason = True, f"Structural component entered fail zone (y <= {self.fail_zone_y} m)"
        elif not failed and self.structure_broken:
            failed, failure_reason = True, "Structure integrity lost (joints broke)"
        elif not failed and self.max_vertical_accel_seen > self.max_vertical_acceleration:
            failed, failure_reason = True, f"Vehicle vertical acceleration {self.max_vertical_accel_seen:.2f} m/s² exceeds 2g limit"
        if not failed and step_count > self.STABILITY_CHECK_START_STEP:
            if abs(angular_velocity) > self.MAX_ANGULAR_VELOCITY:
                if self.first_high_angular_velocity_step is None:
                    self.first_high_angular_velocity_step = step_count
                self.high_angular_velocity_count += 1
                self.max_high_angular_velocity_count = max(
                    self.max_high_angular_velocity_count,
                    self.high_angular_velocity_count,
                )
                if self.high_angular_velocity_count >= self.UNSTABLE_THRESHOLD:
                    failed, failure_reason = True, f"Vehicle unstable (angular velocity {angular_velocity:.2f} rad/s)"
            else:
                self.high_angular_velocity_count = 0
        if not failed and abs(normalized_angle) > math.pi / 2:
            failed, failure_reason = True, f"Vehicle flipped ({math.degrees(abs(normalized_angle)):.1f}°)"
        if not self._rotation_tracking_initialized and self.environment and vehicle_chassis:
            if hasattr(self.environment, 'set_tracked_body'):
                self.environment.set_tracked_body(vehicle_chassis)
                self._rotation_tracking_initialized = True
        airborne_rotation_accumulated = 0.0
        if not failed and self.environment and hasattr(self.environment, 'get_airborne_rotation_status'):
            rotation_status = self.environment.get_airborne_rotation_status()
            airborne_rotation_accumulated = rotation_status['accumulated']
            if airborne_rotation_accumulated > self.max_airborne_rotation_seen:
                self.max_airborne_rotation_seen = airborne_rotation_accumulated
                self.max_airborne_rotation_step = step_count
            if rotation_status['exceeded']:
                failed, failure_reason = True, f"Vehicle rotated {math.degrees(airborne_rotation_accumulated):.1f}° while airborne"
        if success and not failed:
            score = 100.0
        elif failed:
            score = 0.0
        else:
            start_x = 5.0
            max_dist = self.target_x - start_x
            progress = min(max(0, current_x - start_x) / max_dist, 1.0) if max_dist > 0 else 0.0
            score = progress * 80.0
        terrain_bounds = self.environment.get_terrain_bounds()
        right_cliff = terrain_bounds.get("right_cliff", {})
        gap_info = terrain_bounds.get("gap", {})
        build_zone = terrain_bounds.get("build_zone", {})
        terrain_right_cliff_x_start = float(right_cliff.get("x_start", 25.0))
        terrain_right_cliff_x_end = 100.0
        terrain_gap_width = float(gap_info.get("width", 15.0))
        gravity_current = self.environment.get_gravity() if hasattr(self.environment, 'get_gravity') else (0.0, -10.0)
        wind_force_current = self.environment.get_wind_force() if hasattr(self.environment, 'get_wind_force') else (0.0, 0.0)
        body_creation_positions = self.environment.get_body_creation_positions() if hasattr(self.environment, 'get_body_creation_positions') else []
        anchor_positions = self.environment.get_anchor_positions() if hasattr(self.environment, 'get_anchor_positions') else []
        metrics = {
            'vehicle_x': current_x, 'vehicle_y': current_y, 'target_x': self.target_x,
            'velocity_x': velocity_x, 'velocity_y': velocity_y,
            'angular_velocity': angular_velocity, 'angle': angle, 'normalized_angle': normalized_angle,
            'max_vertical_accel': self.max_vertical_accel_seen,
            'max_vertical_accel_seen': self.max_vertical_accel_seen,
            'max_vertical_acceleration_limit': self.max_vertical_acceleration,
            'vehicle_start_x': 5.0,
            'max_airborne_rotation_limit': self.MAX_AIRBORNE_ROTATION,
            'stall_threshold_x': self.stall_threshold_x,
            'fail_zone_y': self.fail_zone_y,
            'success': success and not failed, 'failed': failed, 'failure_reason': failure_reason,
            'step_count': step_count, 'structure_mass': self.environment.get_structure_mass(),
            'max_structure_mass': self.MAX_STRUCTURE_MASS, 'structure_broken': self.structure_broken,
            'joint_count': len(self.environment._joints), 'initial_joint_count': self.initial_joint_count,
            'is_airborne': current_y > (self._cliff_top_y + self.AIRBORNE_THRESHOLD),
            'airborne_rotation_accumulated': airborne_rotation_accumulated,
            'high_angular_velocity_count': self.high_angular_velocity_count,
            'joint_failure_events': self.environment.get_joint_failure_events(),
            'joint_stress_summary': self.environment.get_joint_stress_summary(),
            'joint_max_force_limit': self.environment.get_joint_max_force(),
            'joint_max_torque_limit': self.environment.get_joint_max_torque(),
            'anchor_max_force_limit': self.environment.get_anchor_max_force(),
            'anchor_max_torque_limit': self.environment.get_anchor_max_torque(),
            'current_sim_step': self.environment.get_current_step(),
            'gravity_current': gravity_current,
            'wind_force_current': wind_force_current,
            'terrain_right_cliff_x_start': terrain_right_cliff_x_start,
            'terrain_right_cliff_x_end': terrain_right_cliff_x_end,
            'terrain_gap_width': terrain_gap_width,
            'cliff_top_y': self._cliff_top_y,
            'build_zone_x_min': self.BUILD_ZONE_X_MIN,
            'build_zone_x_max': self.BUILD_ZONE_X_MAX,
            'build_zone_y_min': self.BUILD_ZONE_Y_MIN,
            'build_zone_y_max': self.BUILD_ZONE_Y_MAX,
            'flip_angle_limit_rad': math.pi / 2,
            'max_angular_velocity_limit': self.MAX_ANGULAR_VELOCITY,
            'unstable_threshold_limit': self.UNSTABLE_THRESHOLD,
            'stability_check_start_step': self.STABILITY_CHECK_START_STEP,
            'airborne_threshold_m': self.AIRBORNE_THRESHOLD,
            'body_positions_and_angles': body_positions,
            'body_creation_positions': body_creation_positions,
            'anchor_positions': anchor_positions,
            'evaluation_sample_count': self.evaluation_sample_count,
            'best_vehicle_x': self.best_vehicle_x,
            'best_vehicle_x_step': self.best_vehicle_x_step,
            'best_vehicle_y_at_progress': self.best_vehicle_y_at_progress,
            'min_vehicle_y': self.min_vehicle_y,
            'min_vehicle_y_step': self.min_vehicle_y_step,
            'min_vehicle_x_at_min_y': self.min_vehicle_x_at_min_y,
            'min_structure_y': self.min_structure_y,
            'min_structure_y_step': self.min_structure_y_step,
            'min_structure_x_at_min_y': self.min_structure_x_at_min_y,
            'min_structure_body_index': self.min_structure_body_index,
            'max_abs_angle': self.max_abs_angle,
            'max_abs_angle_step': self.max_abs_angle_step,
            'max_high_angular_velocity_count': self.max_high_angular_velocity_count,
            'first_high_angular_velocity_step': self.first_high_angular_velocity_step,
            'first_chassis_fail_zone_sample_step': self.first_chassis_fail_zone_sample_step,
            'first_structure_fail_zone_sample_step': self.first_structure_fail_zone_sample_step,
            'max_vertical_accel_step': self.max_vertical_accel_step,
            'max_airborne_rotation_seen': self.max_airborne_rotation_seen,
            'max_airborne_rotation_step': self.max_airborne_rotation_step,
        }
        return success or failed, score, metrics
    def _check_design_constraints(self):
        violations = []
        if not self.environment: return ["Environment not available"]
        mass = self.environment.get_structure_mass()
        if not math.isfinite(mass):
            violations.append("Structure mass is not finite")
        elif mass > self.MAX_STRUCTURE_MASS:
            violations.append(f"Mass {mass:.2f}kg exceeds maximum {self.MAX_STRUCTURE_MASS}kg")
        build_zone_x_max = max(self.BUILD_ZONE_X_MAX, self.target_x)
        creation_positions = (
            self.environment.get_body_creation_positions()
            if hasattr(self.environment, 'get_body_creation_positions')
            else [body.position for body in self.environment._bodies]
        )
        for position in creation_positions:
            x, y = position[0], position[1]
            if not (self.BUILD_ZONE_X_MIN <= x <= build_zone_x_max and
                    self.BUILD_ZONE_Y_MIN <= y <= self.BUILD_ZONE_Y_MAX):
                violations.append(f"Beam at ({x:.2f}, {y:.2f}) outside build zone")
        return violations
    def compute_score_with_penalty(self, score: float, metrics: dict) -> float:
        if score > 0:
            return score
        constraint_info = self.get_constraint_info()
        penalty = compute_constraint_penalty("S_01", score, metrics, constraint_info)
        return penalty
    def get_constraint_info(self):
        return {
            'max_structure_mass': self.MAX_STRUCTURE_MASS,
            'build_zone_x_min': self.BUILD_ZONE_X_MIN,
            'build_zone_x_max': self.BUILD_ZONE_X_MAX,
            'build_zone_y_min': self.BUILD_ZONE_Y_MIN,
            'build_zone_y_max': self.BUILD_ZONE_Y_MAX,
            'fail_zone_y': self.fail_zone_y,
            'max_vertical_acceleration': self.max_vertical_acceleration,
            'max_angular_velocity': self.MAX_ANGULAR_VELOCITY,
            'unstable_threshold': self.UNSTABLE_THRESHOLD,
            'max_airborne_rotation': self.MAX_AIRBORNE_ROTATION,
            'joint_max_force_limit': getattr(self.environment, 'get_joint_max_force', lambda: 80.0)(),
            'joint_max_torque_limit': getattr(self.environment, 'get_joint_max_torque', lambda: 300.0)(),
            'anchor_max_force_limit': getattr(self.environment, 'get_anchor_max_force', lambda: 100.0)(),
            'anchor_max_torque_limit': getattr(self.environment, 'get_anchor_max_torque', lambda: 500.0)(),
        }
    def get_task_description(self):
        return {
            'task': 'S-01: The Bridge',
            'description': 'Design a bridge to connect two cliffs and support a testing vehicle',
            'target_position': self.target_x,
            'success_criteria': {
                'primary': f'Vehicle reaches x={self.target_x}m',
                'secondary': 'No structural breaks',
                'tertiary': 'Sampled acceleration <= 2g',
            },
            'evaluation': {'score_range': '0-100', 'success_score': 100, 'failure_score': 0}
        }
