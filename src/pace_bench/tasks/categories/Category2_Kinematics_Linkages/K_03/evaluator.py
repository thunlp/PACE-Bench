import math

from pace_bench.core.simulator import TIME_STEP

from pace_bench.core.primitives import compute_constraint_penalty

class Evaluator:
    def __init__(self, terrain_bounds, environment=None):
        self.terrain_bounds = terrain_bounds
        self.environment = environment
        self.min_object_height = getattr(environment, 'MIN_OBJECT_HEIGHT', 2.0) if environment else 2.0
        self.target_object_y = getattr(environment, 'TARGET_OBJECT_Y', 3.5) if environment else 3.5
        self.target_line_y = getattr(environment, 'TARGET_OBJECT_Y', 3.5) if environment else 3.5
        self.target_y = self.target_line_y
        self.target_x = self.target_object_y
        self.min_simulation_time = getattr(environment, 'MIN_SIMULATION_TIME', 1.34) if environment else 1.34
        self.steps_per_eval = 10
        self.initial_object_y = None
        self.max_object_y_reached = 0.0
        self.min_object_y_seen = float('inf')
        self.object_fell = False
        self.object_grasped = False
        self.steps_with_object_above_target = 0
        self.last_object_y = None
        self.lifting_started = False
        if not environment:
            raise ValueError("Evaluator requires environment instance")
        try:
            self.MAX_STRUCTURE_MASS = environment.MAX_STRUCTURE_MASS
            self.BUILD_ZONE_X_MIN = environment.BUILD_ZONE_X_MIN
            self.BUILD_ZONE_X_MAX = environment.BUILD_ZONE_X_MAX
            self.BUILD_ZONE_Y_MIN = environment.BUILD_ZONE_Y_MIN
            self.BUILD_ZONE_Y_MAX = environment.BUILD_ZONE_Y_MAX
        except AttributeError as e:
            raise AttributeError(f"Environment instance missing required attribute: {e}")
        self.design_constraints_checked = False
    def evaluate(self, agent_body, step_count, max_steps):
        if not self.environment:
            return False, 0.0, {"error": "Environment not available"}
        object_pos = self.environment.get_object_position()
        if object_pos is None:
            return False, 0.0, {"error": "Object not found"}
        current_object_x, current_object_y = object_pos
        if agent_body:
            gripper_x, gripper_y = agent_body.position.x, agent_body.position.y
        else:
            gripper_pos = self.environment.get_gripper_position()
            if gripper_pos is None:
                gripper_x, gripper_y = 5.0, 8.0
            else:
                gripper_x, gripper_y = gripper_pos
        if self.initial_object_y is None:
            self.initial_object_y = current_object_y
            self.last_object_y = current_object_y
            self.min_object_y_seen = current_object_y
        if current_object_y < self.min_object_y_seen:
            self.min_object_y_seen = current_object_y
        if self.lifting_started and current_object_y < self.min_object_height:
            self.object_fell = True
        if current_object_y > self.max_object_y_reached:
            self.max_object_y_reached = current_object_y
        if current_object_y > self.initial_object_y + 0.5:
            self.lifting_started = True
        if current_object_y >= self.target_object_y:
            self.steps_with_object_above_target += getattr(self, 'steps_per_eval', 1)
        distance_to_base = math.sqrt((current_object_x - gripper_x)**2 + (current_object_y - gripper_y)**2)
        if distance_to_base < 1.0:
            self.object_grasped = True
        if not self.object_grasped and hasattr(self.environment, '_bodies'):
            for body in self.environment._bodies:
                dx = current_object_x - body.position.x
                dy = current_object_y - body.position.y
                d = math.sqrt(dx*dx + dy*dy)
                if d < 0.6:
                    self.object_grasped = True
                    break
        if current_object_y > self.initial_object_y + 0.15:
            self.object_grasped = True
        if hasattr(self.environment, 'get_object_contact_count'):
            num_points, num_bodies = self.environment.get_object_contact_count()
            if num_bodies > 0:
                self.object_grasped = True
        self.last_object_y = current_object_y
        reached_target = current_object_y >= self.target_object_y
        maintained_height = not self.object_fell and (not self.lifting_started or self.min_object_y_seen >= self.min_object_height)
        min_steps_hold = max(1, int(self.min_simulation_time / TIME_STEP))
        maintained_grip = self.steps_with_object_above_target >= min_steps_hold
        success = reached_target and maintained_height and maintained_grip and self.object_grasped
        failed = False
        failure_reason = None
        if not self.design_constraints_checked and step_count == 0:
            constraint_violations = self._check_design_constraints()
            if constraint_violations:
                failed = True
                failure_reason = "Design constraint violated: " + "; ".join(constraint_violations)
            self.design_constraints_checked = True
        if self.object_fell:
            failed = True
            failure_reason = f"Object fell (minimum y={self.min_object_y_seen:.2f}m, required >={self.min_object_height}m after lifting)"
        if step_count >= max_steps and not self.lifting_started:
            failed = True
            failure_reason = "Object was not lifted (object y did not increase significantly)"
        height_gained = (current_object_y - self.initial_object_y) if self.initial_object_y is not None else 0.0
        if success and not failed:
            score = 100.0
        elif failed:
            score = 0.0
        else:
            height_score = min(height_gained / 5.0, 1.0) * 50.0
            stability_score = 0.0
            if not self.object_fell and self.lifting_started:
                stability_score = 30.0
            grip_score = min(self.steps_with_object_above_target / min_steps_hold, 1.0) * 20.0
            score = height_score + stability_score + grip_score
        object_contact_points = 0
        gripper_bodies_touching_object = 0
        if hasattr(self.environment, 'get_object_contact_count'):
            object_contact_points, gripper_bodies_touching_object = self.environment.get_object_contact_count()
        slider_data = None
        if hasattr(self.environment, 'get_slider_position'):
            slider_data = self.environment.get_slider_position()
        min_finger_y = None
        if hasattr(self.environment, '_bodies') and self.environment._bodies:
            for body in self.environment._bodies:
                hw = body.fixtures[0].shape.vertexRadius if hasattr(body.fixtures[0].shape, 'vertexRadius') else 0.0
                extents = body.fixtures[0].shape.box if hasattr(body.fixtures[0].shape, 'box') else (hw, hw)
                if extents and len(extents) == 2:
                    bottom_y = body.position.y - extents[1]
                else:
                    bottom_y = body.position.y
                if min_finger_y is None or bottom_y < min_finger_y:
                    min_finger_y = bottom_y
        obj_vel_data = None
        if hasattr(self.environment, 'get_object_velocity'):
            obj_vel_data = self.environment.get_object_velocity()
        obj_info = None
        if hasattr(self.environment, 'get_object_info'):
            obj_info = self.environment.get_object_info()
        platform_top_y_val = None
        if hasattr(self.environment, 'get_platform_top_y'):
            platform_top_y_val = self.environment.get_platform_top_y()
        finger_joint_states = []
        if hasattr(self.environment, 'get_finger_joint_states'):
            finger_joint_states = self.environment.get_finger_joint_states()
        slider_motor_force_val = 0.0
        if hasattr(self.environment, 'get_slider_motor_force'):
            slider_motor_force_val = self.environment.get_slider_motor_force()
        slider_computed_trans = None
        if hasattr(self.environment, 'get_slider_computed_translation'):
            slider_computed_trans = self.environment.get_slider_computed_translation()
        contact_details_res = {'contact_points': 0, 'bodies_touching': 0,
                               'max_normal_impulse': 0.0, 'total_normal_impulse': 0.0}
        if hasattr(self.environment, 'get_object_contact_details'):
            contact_details_res = self.environment.get_object_contact_details()
        gravity_vec = (0.0, -10.0)
        if hasattr(self.environment, 'get_gravity'):
            gravity_vec = self.environment.get_gravity()
        if not hasattr(self, '_event_timeline'):
            self._event_timeline = []
        if not hasattr(self, '_peak_object_speed'):
            self._peak_object_speed = 0.0
        if not hasattr(self, '_was_contacting'):
            self._was_contacting = False
        if not hasattr(self, '_contact_lost_count'):
            self._contact_lost_count = 0
        if not hasattr(self, '_contact_persistence_steps'):
            self._contact_persistence_steps = 0
        if not hasattr(self, '_total_contact_steps'):
            self._total_contact_steps = 0
        if not hasattr(self, '_finger_angle_min_deg'):
            self._finger_angle_min_deg = float('inf')
        if not hasattr(self, '_finger_angle_max_deg'):
            self._finger_angle_max_deg = float('-inf')
        if not hasattr(self, '_grasp_acquired_step'):
            self._grasp_acquired_step = None
        if obj_vel_data and obj_vel_data.get('speed') is not None:
            spd = obj_vel_data['speed']
            if math.isfinite(spd) and spd > self._peak_object_speed:
                self._peak_object_speed = spd
        is_contacting_now = contact_details_res.get('bodies_touching', 0) > 0
        if is_contacting_now:
            self._contact_persistence_steps += 1
            self._total_contact_steps += 1
        if self._was_contacting and not is_contacting_now:
            self._contact_lost_count += 1
            self._event_timeline.append({
                'step': step_count,
                'event': 'contact_lost',
                'object_y': current_object_y,
                'object_x': current_object_x,
            })
        self._was_contacting = is_contacting_now
        for st in finger_joint_states:
            ang = st.get('angle_deg')
            if ang is not None and math.isfinite(ang):
                if ang < self._finger_angle_min_deg:
                    self._finger_angle_min_deg = ang
                if ang > self._finger_angle_max_deg:
                    self._finger_angle_max_deg = ang
        if self.object_grasped and self._grasp_acquired_step is None:
            self._grasp_acquired_step = step_count
            self._event_timeline.append({
                'step': step_count,
                'event': 'grasp_acquired',
                'object_y': current_object_y,
                'object_x': current_object_x,
            })
        if self.object_fell and not any(e.get('event') == 'object_fell' for e in self._event_timeline):
            self._event_timeline.append({
                'step': step_count,
                'event': 'object_fell',
                'object_y': current_object_y,
                'min_object_y_seen': self.min_object_y_seen,
                'min_object_height': self.min_object_height,
            })
        constraint_profile = []
        bx_min = self.BUILD_ZONE_X_MIN
        bx_max = self.BUILD_ZONE_X_MAX
        by_min = self.BUILD_ZONE_Y_MIN
        by_max = self.BUILD_ZONE_Y_MAX
        sm = self.environment.get_structure_mass()
        mm = self.MAX_STRUCTURE_MASS
        if mm > 0:
            mass_pct = (sm / mm) * 100.0
            mass_margin = mm - sm
            constraint_profile.append({
                'name': 'Mass budget',
                'status': 'FAIL' if sm > mm else ('NEAR' if mass_pct > 80.0 else 'PASS'),
                'value': sm, 'limit': mm, 'margin': mass_margin, 'unit': 'kg', 'pct': mass_pct,
            })
        build_zone_failed_from_eval = (failed and failure_reason and
                                       "Design constraint" in str(failure_reason) and
                                       "build zone" in str(failure_reason).lower())
        constraint_profile.append({
            'name': 'Build zone (design-time)',
            'status': 'FAIL' if build_zone_failed_from_eval else 'PASS',
            'value': None,
            'limit': f'x=[{bx_min},{bx_max}] y=[{by_min},{by_max}]',
            'margin': None, 'unit': '',
        })
        obj_y_margin = current_object_y - self.target_object_y
        constraint_profile.append({
            'name': 'Target height',
            'status': 'PASS' if obj_y_margin >= 0 else ('NEAR' if obj_y_margin >= -0.5 else 'FAIL'),
            'value': current_object_y, 'limit': self.target_object_y,
            'margin': obj_y_margin, 'unit': 'm',
        })
        constraint_profile.append({
            'name': 'Min height maintained',
            'status': 'FAIL' if self.object_fell else 'PASS',
            'value': self.min_object_y_seen if math.isfinite(self.min_object_y_seen) else None,
            'limit': self.min_object_height, 'margin': None, 'unit': 'm',
        })
        constraint_profile.append({
            'name': 'Grasp',
            'status': 'PASS' if self.object_grasped else 'FAIL',
            'value': bool(self.object_grasped),
            'limit': 'evaluator grasp proxy true', 'margin': None, 'unit': '',
        })
        held_steps = self.steps_with_object_above_target
        required_steps = max(1, int(self.min_simulation_time / TIME_STEP))
        constraint_profile.append({
            'name': 'Sustain duration',
            'status': 'PASS' if held_steps >= required_steps else ('NEAR' if held_steps >= required_steps * 0.5 else 'FAIL'),
            'value': held_steps, 'limit': required_steps, 'margin': held_steps - required_steps, 'unit': 'steps',
        })
        metrics = {
            'gripper_x': gripper_x,
            'gripper_y': gripper_y,
            'object_x': current_object_x,
            'object_y': current_object_y,
            'target_object_y': self.target_object_y,
            'height_gained': height_gained,
            'max_object_y_reached': self.max_object_y_reached,
            'progress': min((current_object_y - self.initial_object_y) / 5.0, 1.0) * 100 if current_object_y >= self.initial_object_y else 0.0,
            'success': success and not failed,
            'failed': failed,
            'failure_reason': failure_reason,
            'step_count': step_count,
            'structure_mass': self.environment.get_structure_mass(),
            'max_structure_mass': self.MAX_STRUCTURE_MASS,
            'min_object_y_seen': self.min_object_y_seen,
            'object_fell': self.object_fell,
            'object_grasped': self.object_grasped,
            'object_contact_points': object_contact_points,
            'gripper_bodies_touching_object': gripper_bodies_touching_object,
            'steps_with_object_above_target': self.steps_with_object_above_target,
            'min_simulation_steps_required': int(self.min_simulation_time / TIME_STEP),
            'slider_body_y': slider_data['slider_body_y'] if slider_data else None,
            'slider_translation': slider_data['slider_translation'] if slider_data else None,
            'slider_anchor_y': slider_data['slider_anchor_y'] if slider_data else None,
            'slider_lower_limit': slider_data['slider_lower_limit'] if slider_data else None,
            'slider_upper_limit': slider_data['slider_upper_limit'] if slider_data else None,
            'slider_motor_speed': slider_data['slider_motor_speed'] if slider_data else None,
            'slider_max_motor_force': slider_data.get('slider_max_motor_force') if slider_data else None,
            'min_finger_tip_y': min_finger_y,
            'object_velocity': obj_vel_data,
            'object_shape': obj_info.get('shape') if obj_info else None,
            'object_width': obj_info.get('width') if obj_info else None,
            'object_height': obj_info.get('height') if obj_info else None,
            'object_radius': obj_info.get('radius') if obj_info else None,
            'object_friction': obj_info.get('friction') if obj_info else None,
            'score': score,
            'time_step': TIME_STEP,
            'object_mass': obj_info.get('mass') if obj_info else None,
            'platform_top_y': platform_top_y_val,
            'finger_joint_states': finger_joint_states,
            'slider_motor_force': slider_motor_force_val,
            'slider_computed_translation': slider_computed_trans,
            'gravity_x': float(gravity_vec[0]) if hasattr(gravity_vec, '__getitem__') and len(gravity_vec) >= 2 else None,
            'gravity_y': float(gravity_vec[1]) if hasattr(gravity_vec, '__getitem__') and len(gravity_vec) >= 2 else None,
            'contact_max_normal_impulse': contact_details_res.get('max_normal_impulse', 0.0) if contact_details_res else 0.0,
            'contact_total_normal_impulse': contact_details_res.get('total_normal_impulse', 0.0) if contact_details_res else 0.0,
            'event_timeline': list(self._event_timeline),
            'peak_object_speed': self._peak_object_speed,
            'contact_persistence_steps': self._contact_persistence_steps,
            'contact_lost_count': self._contact_lost_count,
            'total_contact_steps': self._total_contact_steps,
            'finger_angle_min_deg': self._finger_angle_min_deg if math.isfinite(self._finger_angle_min_deg) else None,
            'finger_angle_max_deg': self._finger_angle_max_deg if math.isfinite(self._finger_angle_max_deg) else None,
            'constraint_profile': constraint_profile,
            'build_zone_x_min': self.BUILD_ZONE_X_MIN,
            'build_zone_x_max': self.BUILD_ZONE_X_MAX,
            'build_zone_y_min': self.BUILD_ZONE_Y_MIN,
            'build_zone_y_max': self.BUILD_ZONE_Y_MAX,
            'initial_object_y': self.initial_object_y,
            'observation_error_count': getattr(self.environment, '_observation_error_count', 0),
            'last_observation_error': getattr(self.environment, '_last_observation_error', None),
        }
        return success or failed, score, metrics
    def _check_design_constraints(self):
        violations = []
        if not self.environment:
            return ["Environment not available"]
        structure_mass = self.environment.get_structure_mass()
        if structure_mass > self.MAX_STRUCTURE_MASS:
            violations.append(f"Structure mass {structure_mass:.2f}kg exceeds maximum {self.MAX_STRUCTURE_MASS}kg")
        for body in self.environment._bodies:
            x, y = body.position.x, body.position.y
            if not (self.BUILD_ZONE_X_MIN <= x <= self.BUILD_ZONE_X_MAX and
                    self.BUILD_ZONE_Y_MIN <= y <= self.BUILD_ZONE_Y_MAX):
                violations.append(f"Beam at ({x:.2f}, {y:.2f}) is outside build zone x=[{self.BUILD_ZONE_X_MIN}, {self.BUILD_ZONE_X_MAX}], y=[{self.BUILD_ZONE_Y_MIN}, {self.BUILD_ZONE_Y_MAX}]")
        return violations
    def compute_score_with_penalty(self, score: float, metrics: dict) -> float:
        if score > 0:
            return score
        constraint_info = self.get_constraint_info()
        penalty = compute_constraint_penalty("K_03", score, metrics, constraint_info)
        return penalty
    def get_constraint_info(self):
        return {
            'target_object_y': self.target_object_y,
            'min_object_height': self.min_object_height,
            'min_simulation_time': self.min_simulation_time,
            'max_structure_mass': self.MAX_STRUCTURE_MASS,
            'build_zone_x_min': self.BUILD_ZONE_X_MIN,
            'build_zone_x_max': self.BUILD_ZONE_X_MAX,
            'build_zone_y_min': self.BUILD_ZONE_Y_MIN,
            'build_zone_y_max': self.BUILD_ZONE_Y_MAX,
        }
    def get_task_description(self):
        return {
            'task': 'K-03: The Gripper',
            'description': 'Design a gripper mechanism that grasps objects and lifts them using motor rotation',
            'target_position': self.target_object_y,
            'terrain': {
                'ground': self.terrain_bounds.get('ground', {}),
            },
            'success_criteria': {
                'primary': f'Object is lifted to target line (y >= {self.target_object_y}m) and held there',
                'secondary': f'Object never falls below {self.min_object_height}m after being lifted',
                'tertiary': f'Object maintains grip at/above target for required time (~{self.min_simulation_time}s)',
            },
            'evaluation': {
                'score_range': '0-100',
                'success_score': 100,
                'partial_score': 'Based on height reached (max 50), stability (max 30), and sustained grip (max 20)',
                'failure_score': 0
            }
        }
