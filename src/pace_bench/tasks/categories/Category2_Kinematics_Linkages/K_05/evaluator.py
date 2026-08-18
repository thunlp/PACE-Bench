import math

from pace_bench.core.simulator import TIME_STEP

from pace_bench.core.primitives import compute_constraint_penalty

GROUND_Y = 1.0

OBJECT_START_Y = 1.8

TARGET_OBJECT_Y = 9.0

LIFT_HEIGHT_FROM_GROUND = 8.0

MIN_SUSTAIN_S = 3.0

SUSTAIN_VELOCITY_THRESHOLD = -0.4

class Evaluator:
    def __init__(self, terrain_bounds, environment=None):
        self.terrain_bounds = terrain_bounds
        self.environment = environment
        self.start_object_y = OBJECT_START_Y
        self.target_object_y = getattr(environment, 'target_object_y', None) if environment else None
        if self.target_object_y is None:
            self.target_object_y = TARGET_OBJECT_Y
        self.min_simulation_time = getattr(environment, 'min_sustain_s', None) if environment else None
        if self.min_simulation_time is None:
            self.min_simulation_time = MIN_SUSTAIN_S
        self.min_simulation_steps = int(self.min_simulation_time / TIME_STEP)
        self.initial_object_y = None
        self.max_object_y_reached = 0.0
        self.steps_with_object_above_target = 0
        self.last_object_y = None
        self.lifting_started = False
        self.initial_joint_count = 0
        self.structure_broken = False
        self._step_at_max_object_y = None
        self._peak_object_velocity_x = 0.0
        self._peak_object_velocity_y = 0.0
        self._step_at_first_above_target = None
        self._step_at_last_above_target = None
        self._first_above_target_reached = False
        self._max_lifter_y = None
        self._max_lifter_y_at_step = None
        self._vel_y_at_max_height = None
        self._vel_y_on_first_cross = None
        self._diagnostic_error_count = 0
        self._last_diagnostic_error = None
        if not environment:
            raise ValueError("Evaluator requires environment instance")
        try:
            self.MAX_STRUCTURE_MASS = getattr(environment, 'MAX_STRUCTURE_MASS', 60.0)
            self.BUILD_ZONE_X_MIN = getattr(environment, 'BUILD_ZONE_X_MIN', 0.0)
            self.BUILD_ZONE_X_MAX = getattr(environment, 'BUILD_ZONE_X_MAX', 8.0)
            self.BUILD_ZONE_Y_MIN = getattr(environment, 'BUILD_ZONE_Y_MIN', 1.0)
            self.BUILD_ZONE_Y_MAX = getattr(environment, 'BUILD_ZONE_Y_MAX', 12.0)
            self.lifting_threshold_m = getattr(environment, 'LIFTING_THRESHOLD_M', 0.5)
        except Exception as e:
            raise AttributeError(f"Environment missing required constants: {e}")
        self.design_constraints_checked = False
    def evaluate(self, agent_body, step_count, max_steps):
        if not self.environment:
            return False, 0.0, {"error": "Environment not available"}
        object_pos = self.environment.get_object_position()
        if object_pos is None:
            return False, 0.0, {"error": "Object not found"}
        current_object_x, current_object_y = object_pos
        if agent_body:
            lifter_x, lifter_y = agent_body.position.x, agent_body.position.y
        else:
            lifter_pos = self.environment.get_lifter_position()
            if lifter_pos is None:
                lifter_x, lifter_y = 4.0, 2.0
            else:
                lifter_x, lifter_y = lifter_pos
        obj_vel_x = obj_vel_y = 0.0
        if self.environment._object_to_lift:
            obj_vel_x = self.environment._object_to_lift.linearVelocity.x
            obj_vel_y = self.environment._object_to_lift.linearVelocity.y
        if self.initial_object_y is None:
            self.initial_object_y = current_object_y
            self.last_object_y = current_object_y
            self.initial_joint_count = len(self.environment._joints)
            self._vel_y_at_max_height = obj_vel_y
        new_peak = False
        if current_object_y > self.max_object_y_reached:
            self.max_object_y_reached = current_object_y
            self._step_at_max_object_y = step_count
            self._vel_y_at_max_height = obj_vel_y
            new_peak = True
        abs_vx = abs(obj_vel_x)
        abs_vy = abs(obj_vel_y)
        if abs_vx > self._peak_object_velocity_x:
            self._peak_object_velocity_x = abs_vx
        if abs_vy > self._peak_object_velocity_y:
            self._peak_object_velocity_y = abs_vy
        lifter_top_y = self.environment.get_lifter_platform_y()
        if lifter_top_y is not None:
            if (self._max_lifter_y is None) or (lifter_top_y > self._max_lifter_y):
                self._max_lifter_y = lifter_top_y
                self._max_lifter_y_at_step = step_count
        if current_object_y > self.initial_object_y + self.lifting_threshold_m:
            self.lifting_started = True
        if self.max_object_y_reached > self.initial_object_y + self.lifting_threshold_m:
            self.lifting_started = True
        if not self._first_above_target_reached and current_object_y >= self.target_object_y:
            self._first_above_target_reached = True
            self._step_at_first_above_target = step_count
            self._vel_y_on_first_cross = obj_vel_y
        if current_object_y >= self.target_object_y:
            self._step_at_last_above_target = step_count
        if current_object_y >= self.target_object_y and obj_vel_y >= SUSTAIN_VELOCITY_THRESHOLD:
            self.steps_with_object_above_target += 1
        current_joint_count = len(self.environment._joints)
        if current_joint_count < self.initial_joint_count:
            self.structure_broken = True
        self.last_object_y = current_object_y
        reached_target = current_object_y >= self.target_object_y
        maintained_structure = not self.structure_broken
        maintained_height = self.steps_with_object_above_target >= self.min_simulation_steps
        success = reached_target and maintained_structure and maintained_height
        failed = False
        failure_reason = None
        if not self.design_constraints_checked:
            constraint_violations = self._check_design_constraints()
            if constraint_violations:
                failed = True
                failure_reason = "Design constraint violated: " + "; ".join(constraint_violations)
            self.design_constraints_checked = True
        if self.structure_broken:
            failed = True
            failure_reason = "Lifter structure integrity lost (joints broke under load)"
        if step_count >= max_steps and not self.lifting_started:
            failed = True
            failure_reason = "Object was not lifted (object y did not increase significantly)"
        if success and not failed:
            score = 100.0
        elif failed:
            score = 0.0
        else:
            height_gained = current_object_y - self.initial_object_y
            height_score = min(height_gained / (self.target_object_y - self.start_object_y), 1.0) * 50.0
            structure_score = 0.0
            if not self.structure_broken:
                structure_score = 30.0
            if self.max_object_y_reached < self.initial_object_y + self.lifting_threshold_m:
                structure_score = 0.0
            height_maintenance_score = min(self.steps_with_object_above_target / self.min_simulation_steps, 1.0) * 20.0
            score = max(0.0, height_score + structure_score + height_maintenance_score)
        joint_peak_forces = {}
        joint_failure_events_ = []
        try:
            joint_peak_forces = self.environment.get_joint_peak_forces()
        except Exception as exc:
            self._record_diagnostic_error("joint_peak_forces", exc)
        try:
            joint_failure_events_ = self.environment.get_joint_failure_events()
        except Exception as exc:
            self._record_diagnostic_error("joint_failure_events", exc)
        max_body_width = None
        try:
            max_body_width = self.environment.get_max_body_width()
        except Exception as exc:
            self._record_diagnostic_error("max_body_width", exc)
        obj_platform_h_offset = None
        obj_platform_v_offset = None
        lifter_top_y_at_measure = None
        try:
            h_off, v_off, top_y = self.environment.get_platform_object_offset()
            obj_platform_h_offset = h_off
            obj_platform_v_offset = v_off
            lifter_top_y_at_measure = top_y
        except Exception as exc:
            self._record_diagnostic_error("platform_object_offset", exc)
        body_count = len(self.environment._bodies)
        initial_joint_count_val = self.initial_joint_count
        joint_force_summary = []
        if joint_peak_forces:
            max_jf_limit = self.environment.get_max_joint_force_limit()
            for jid, peak_f in joint_peak_forces.items():
                entry = {'joint_id': jid, 'peak_force': float(peak_f)}
                if math.isfinite(max_jf_limit):
                    entry['limit'] = float(max_jf_limit)
                    entry['pct_of_limit'] = float(peak_f) / float(max_jf_limit) * 100.0
                joint_force_summary.append(entry)
            joint_force_summary.sort(key=lambda e: e.get('pct_of_limit', e['peak_force']), reverse=True)
        metrics = {
            'lifter_x': lifter_x,
            'lifter_y': lifter_y,
            'object_x': current_object_x,
            'object_y': current_object_y,
            'object_velocity_x': obj_vel_x,
            'object_velocity_y': obj_vel_y,
            'target_object_y': self.target_object_y,
            'height_gained': height_gained if 'height_gained' in locals() else (current_object_y - self.initial_object_y),
            'max_object_y_reached': self.max_object_y_reached,
            'progress': min((current_object_y - self.initial_object_y) / (self.target_object_y - self.start_object_y), 1.0) * 100 if current_object_y >= self.initial_object_y else 0.0,
            'success': success and not failed,
            'failed': failed,
            'failure_reason': failure_reason,
            'step_count': step_count,
            'max_steps': max_steps,
            'time_step': TIME_STEP,
            'structure_mass': self.environment.get_structure_mass(),
            'max_structure_mass': self.MAX_STRUCTURE_MASS,
            'structure_broken': self.structure_broken,
            'joint_count': current_joint_count,
            'initial_joint_count': initial_joint_count_val,
            'body_count': body_count,
            'steps_with_object_above_target': self.steps_with_object_above_target,
            'min_simulation_steps_required': self.min_simulation_steps,
            'step_at_max_object_y': self._step_at_max_object_y,
            'vel_y_at_max_height': self._vel_y_at_max_height,
            'peak_object_velocity_x': self._peak_object_velocity_x,
            'peak_object_velocity_y': self._peak_object_velocity_y,
            'step_at_first_above_target': self._step_at_first_above_target,
            'step_at_last_above_target': self._step_at_last_above_target,
            'vel_y_on_first_cross': self._vel_y_on_first_cross,
            'max_lifter_y': self._max_lifter_y,
            'max_lifter_y_at_step': self._max_lifter_y_at_step,
            'sustain_velocity_threshold': SUSTAIN_VELOCITY_THRESHOLD,
            'object_mass': self.environment.get_object_config().get("mass"),
            'object_friction': self.environment.get_object_config().get("friction"),
            'object_com_offset': self.environment.get_object_config().get("com_offset"),
            'wind_force_x': self.environment.get_wind_force()[0],
            'wind_force_y': self.environment.get_wind_force()[1],
            'ceiling_gap': self.environment.get_ceiling_gap(),
            'max_joint_force_limit': self.environment.get_max_joint_force_limit(),
            'peak_joint_reaction_force': self.environment.get_peak_joint_reaction_force(),
            'margin_to_target_at_peak': (self.max_object_y_reached - self.target_object_y) if self.max_object_y_reached else None,
            'margin_below_target_at_end': (current_object_y - self.target_object_y) if current_object_y is not None else None,
            'lifting_threshold_m': self.lifting_threshold_m,
            'max_body_width': max_body_width,
            'obj_platform_h_offset': obj_platform_h_offset,
            'obj_platform_v_offset': obj_platform_v_offset,
            'lifter_top_y_at_measure': lifter_top_y_at_measure,
            'joint_force_summary': joint_force_summary,
            'joint_failure_events': joint_failure_events_,
            'initial_object_y': self.initial_object_y,
            'diagnostic_error_count': self._diagnostic_error_count,
            'last_diagnostic_error': self._last_diagnostic_error,
        }
        return success or failed, score, metrics
    def _record_diagnostic_error(self, source, exc):
        self._diagnostic_error_count += 1
        self._last_diagnostic_error = f"{source}: {type(exc).__name__}: {exc}"
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
        penalty = compute_constraint_penalty("K_05", score, metrics, constraint_info)
        return penalty
    def get_constraint_info(self):
        return {
            'target_object_y': self.target_object_y,
            'start_object_y': self.start_object_y,
            'min_simulation_time': self.min_simulation_time,
            'max_structure_mass': self.MAX_STRUCTURE_MASS,
            'build_zone_x_min': self.BUILD_ZONE_X_MIN,
            'build_zone_x_max': self.BUILD_ZONE_X_MAX,
            'build_zone_y_min': self.BUILD_ZONE_Y_MIN,
            'build_zone_y_max': self.BUILD_ZONE_Y_MAX,
            'lifting_threshold_m': self.lifting_threshold_m,
            'sustain_velocity_threshold': SUSTAIN_VELOCITY_THRESHOLD,
        }
    def get_task_description(self):
        lift_height_from_ground = self.target_object_y - GROUND_Y
        return {
            'task': 'K-05: The Lifter',
            'description': 'Design a scissor lift mechanism that lifts objects vertically using motor rotation',
            'target_position': self.target_object_y,
            'ground_y': GROUND_Y,
            'object_start_y': OBJECT_START_Y,
            'lift_height_from_ground': lift_height_from_ground,
            'terrain': {
                'ground': self.terrain_bounds.get('ground', {}),
            },
            'success_criteria': {
                'primary': f'Object is lifted to height of at least {lift_height_from_ground}m from ground (y >= {self.target_object_y}m)',
                'secondary': 'Lifter structure remains intact (no joints break)',
                'tertiary': f'Object maintains height for at least {self.min_simulation_time}s, and not sliding (velocity_y >= {SUSTAIN_VELOCITY_THRESHOLD} m/s)',
            },
            'evaluation': {
                'score_range': '0-100',
                'success_score': 100,
                'partial_score': 'Based on height reached (max 50), structure integrity (max 30), and sustained height (max 20)',
                'failure_score': 0
            }
        }
