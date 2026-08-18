import math

from pace_bench.core.simulator import TIME_STEP

from pace_bench.core.primitives import compute_constraint_penalty

class Evaluator:
    def __init__(self, terrain_bounds, environment=None):
        self.terrain_bounds = terrain_bounds
        self.environment = environment
        self.target_height = float(terrain_bounds.get("target_height", 20.0))
        self.fell_height_threshold = float(terrain_bounds.get("fell_height_threshold", 0.5))
        wall_contact_x = terrain_bounds.get("wall_contact_x", [3.5, 7.5])
        self.wall_contact_x_lo = float(wall_contact_x[0]) if len(wall_contact_x) >= 1 else 3.5
        self.wall_contact_x_hi = float(wall_contact_x[1]) if len(wall_contact_x) >= 2 else 7.5
        self.min_simulation_time = 10.0
        self.min_simulation_steps = int(self.min_simulation_time / TIME_STEP)
        self.initial_y = 1.5
        self.max_y_reached = 1.5
        self.min_height_seen = 1.5
        self._initial_position_captured = False
        self.design_constraints_checked = False
        self._failure_step: int = -1
        self._failure_type: str = ""
        self._peak_joint_force_pct: float = 0.0
        self._peak_joint_torque_pct: float = 0.0
        self._peak_total_ke: float = 0.0
        self._peak_total_pe: float = 0.0
        self._peak_body_speed: float = 0.0
        self._peak_body_ang_vel: float = 0.0
        self._initial_joint_count: int = 0
        self._joints_broken: int = 0
        self._prev_joint_count: int = 0
        self._pad_suction_gap_y: float = -1.0
        self._gravity_evolution: float = 0.0
        self._gravity_initial_y: float = -8.0
        self._wind_force: float = 0.0
        self._vortex_y: float = 100.0
        self._vortex_force_x: float = 0.0
        self._vortex_force_y: float = 0.0
        self._suction_zones: list = []
        self._max_joint_force: float = float("inf")
        self._max_joint_torque: float = float("inf")
        self._gravity_at_failure: float = 0.0
        self._physics_history: list = []
        self._first_body_id: int = 0
        self._joint_failure_events: list = []
        if environment is not None:
            self._sync_environment_config(environment)
    def _sync_environment_config(self, env):
        if hasattr(env, "get_gravity_config"):
            gc = env.get_gravity_config()
            self._gravity_evolution = float(gc.get("gravity_evolution", 0.0))
            self._gravity_initial_y = float(gc.get("initial_gravity_y", -8.0))
        if hasattr(env, "get_wind_config"):
            wc = env.get_wind_config()
            self._wind_force = float(wc.get("wind_force", 0.0))
        if hasattr(env, "get_vortex_config"):
            vc = env.get_vortex_config()
            self._vortex_y = float(vc.get("vortex_y", 100.0))
            self._vortex_force_x = float(vc.get("vortex_force_x", 0.0))
            self._vortex_force_y = float(vc.get("vortex_force_y", 0.0))
        if hasattr(env, "get_suction_zones"):
            zones = env.get_suction_zones()
            self._suction_zones = list(zones) if zones else []
        if hasattr(env, "get_joint_limits"):
            jl = env.get_joint_limits()
            self._max_joint_force = float(jl.get("max_joint_force", float("inf")))
            self._max_joint_torque = float(jl.get("max_joint_torque", float("inf")))
    def evaluate(self, agent_body, step_count, max_steps):
        if not self.environment:
            return (False, 0.0, {"error": "Environment not available"})
        initial_body = agent_body
        if initial_body is None and self.environment._bodies:
            initial_body = self.environment._bodies[0]
        if not self._initial_position_captured and initial_body is not None:
            self.initial_y = float(initial_body.position.y)
            self.max_y_reached = self.initial_y
            self.min_height_seen = self.initial_y
            self._initial_position_captured = True
        if self._max_joint_force == float("inf") and hasattr(self.environment, "get_joint_limits"):
            self._sync_environment_config(self.environment)
        if step_count == 0 and not self.design_constraints_checked:
            self.design_constraints_checked = True
            self._initial_joint_count = len(self.environment._joints)
            self._prev_joint_count = self._initial_joint_count
            self._sync_environment_config(self.environment)
            def _base_metrics() -> dict:
                return {
                    "failure_step": 0,
                    "failure_type": self._failure_type,
                    "joints_initial": self._initial_joint_count,
                    "joints_remaining": self._initial_joint_count,
                    "joints_broken": 0,
                    "peak_joint_force_pct": 0.0,
                    "peak_joint_torque_pct": 0.0,
                    "max_joint_force_limit": self._max_joint_force,
                    "max_joint_torque_limit": self._max_joint_torque,
                    "gravity_initial_y": self._gravity_initial_y,
                    "gravity_evolution": self._gravity_evolution,
                    "gravity_at_failure": None,
                    "wind_force": self._wind_force,
                    "vortex_y": self._vortex_y,
                    "vortex_force_x": self._vortex_force_x,
                    "vortex_force_y": self._vortex_force_y,
                    "suction_zones": list(self._suction_zones),
                    "pad_suction_gap_y": None,
                    "nearest_suction_boundary_y": None,
                    "distance_to_suction_boundary": None,
                    "suction_boundary_type": None,
                    "physics_step": None,
                    "physics_gravity_y": None,
                    "physics_wind_force_x": None,
                    "physics_num_active_pads": None,
                    "physics_peak_joint_force_pct": None,
                    "physics_total_ke": None,
                    "physics_total_pe": None,
                    "physics_max_body_speed": None,
                    "physics_max_body_ang_vel": None,
                    "peak_total_ke": 0.0,
                    "peak_total_pe": 0.0,
                    "peak_body_speed": 0.0,
                    "peak_body_ang_vel": 0.0,
                    "joint_stress_summary": [],
                    "joint_failure_events": [],
                    "nan_flag": False,
                    "inf_flag": False,
                    "extreme_speed_flag": False,
                    "initial_y": self.initial_y,
                }
            total_mass = self.environment.get_structure_mass()
            max_mass = getattr(self.environment, 'MAX_STRUCTURE_MASS', 50.0)
            if total_mass > max_mass:
                self._failure_step = 0
                self._failure_type = "mass_above"
                base = _base_metrics()
                base.update({
                    "failed": True,
                    "failure_reason": f"Design constraint violated: Total mass ({total_mass:.2f}kg) exceeds budget ({max_mass:.0f}kg)",
                    "structure_mass": total_mass,
                    "max_structure_mass": max_mass,
                    "min_structure_mass": getattr(self.environment, 'MIN_STRUCTURE_MASS', 0.0),
                    "wall_contact_x_lo": self.wall_contact_x_lo,
                    "wall_contact_x_hi": self.wall_contact_x_hi,
                    "build_zone_x_min": getattr(self.environment, 'BUILD_ZONE_X_MIN', 0.0),
                    "build_zone_x_max": getattr(self.environment, 'BUILD_ZONE_X_MAX', 5.0),
                    "build_zone_y_min": getattr(self.environment, 'BUILD_ZONE_Y_MIN', 0.0),
                    "build_zone_y_max": getattr(self.environment, 'BUILD_ZONE_Y_MAX', 25.0),
                    "climber_x": 0.0,
                    "climber_y": 0.0,
                    "height_gained": 0.0,
                    "max_height_reached": 0.0,
                    "min_height_seen": 0.0,
                    "target_y": self.target_height,
                    "progress": 0.0,
                    "success": False,
                    "step_count": 0,
                    "min_simulation_steps_required": self.min_simulation_steps,
                    "climber_fell": False,
                })
                return True, 0.0, base
            for body in self.environment._bodies:
                pos = body.position
                if not (self.environment.BUILD_ZONE_X_MIN <= pos.x <= self.environment.BUILD_ZONE_X_MAX and
                        self.environment.BUILD_ZONE_Y_MIN <= pos.y <= self.environment.BUILD_ZONE_Y_MAX):
                    self._failure_step = 0
                    self._failure_type = "build_zone"
                    base = _base_metrics()
                    base.update({
                        "failed": True,
                        "failure_reason": f"Design constraint violated: Component at ({pos.x:.2f}, {pos.y:.2f}) is outside Build Zone",
                        "climber_x": pos.x,
                        "climber_y": pos.y,
                        "height_gained": pos.y - self.initial_y,
                        "max_height_reached": pos.y,
                        "min_height_seen": pos.y,
                        "target_y": self.target_height,
                        "progress": 0.0,
                        "success": False,
                        "step_count": 0,
                        "min_simulation_steps_required": self.min_simulation_steps,
                        "climber_fell": False,
                        "structure_mass": total_mass,
                        "max_structure_mass": max_mass,
                        "min_structure_mass": getattr(self.environment, 'MIN_STRUCTURE_MASS', 0.0),
                        "wall_contact_x_lo": self.wall_contact_x_lo,
                        "wall_contact_x_hi": self.wall_contact_x_hi,
                        "build_zone_x_min": getattr(self.environment, 'BUILD_ZONE_X_MIN', 0.0),
                        "build_zone_x_max": getattr(self.environment, 'BUILD_ZONE_X_MAX', 5.0),
                        "build_zone_y_min": getattr(self.environment, 'BUILD_ZONE_Y_MIN', 0.0),
                        "build_zone_y_max": getattr(self.environment, 'BUILD_ZONE_Y_MAX', 25.0),
                    })
                    return True, 0.0, base
        phys_state = {}
        if hasattr(self.environment, "get_physics_state"):
            phys_state = self.environment.get_physics_state()
        if phys_state:
            pf = float(phys_state.get("peak_joint_force_pct", 0.0))
            pt = float(phys_state.get("peak_joint_torque_pct", 0.0))
            self._peak_joint_force_pct = max(self._peak_joint_force_pct, pf)
            self._peak_joint_torque_pct = max(self._peak_joint_torque_pct, pt)
            ke = float(phys_state.get("total_ke", 0.0))
            pe = float(phys_state.get("total_pe", 0.0))
            self._peak_total_ke = max(self._peak_total_ke, ke)
            self._peak_total_pe = max(self._peak_total_pe, pe)
            bs = float(phys_state.get("max_body_speed", 0.0))
            bav = float(phys_state.get("max_body_ang_vel", 0.0))
            self._peak_body_speed = max(self._peak_body_speed, bs)
            self._peak_body_ang_vel = max(self._peak_body_ang_vel, bav)
            num_joints_now = int(phys_state.get("num_joints_remaining", self._prev_joint_count))
            if num_joints_now < self._prev_joint_count:
                self._joints_broken += (self._prev_joint_count - num_joints_now)
            self._prev_joint_count = num_joints_now
            if hasattr(self.environment, "get_joint_failure_events"):
                env_failures = self.environment.get_joint_failure_events()
                self._joint_failure_events = list(env_failures) if env_failures else []
            self._physics_history.append(phys_state)
        body = agent_body
        if body is None and self.environment._bodies:
            body = self.environment._bodies[0]
        if body is None:
            return (False, 0.0, {"error": "Climber body not found"})
        current_x = body.position.x
        current_y = body.position.y
        self.max_y_reached = max(self.max_y_reached, current_y)
        self.min_height_seen = min(self.min_height_seen, current_y)
        failed = False
        failure_reason = None
        height_progress = 0.0
        if current_y < self.fell_height_threshold:
            if self._failure_step < 0:
                self._failure_step = step_count
                self._failure_type = "fell"
                if phys_state:
                    self._gravity_at_failure = float(phys_state.get("gravity_y", 0.0))
            failed = True
            failure_reason = f"Climber fell: touched the ground (height < {self.fell_height_threshold}m)"
        if not (self.wall_contact_x_lo <= current_x <= self.wall_contact_x_hi):
            if self._failure_step < 0:
                self._failure_step = step_count
                self._failure_type = "lost_contact"
                if phys_state:
                    self._gravity_at_failure = float(phys_state.get("gravity_y", 0.0))
            failed = True
            failure_reason = f"Climber lost wall contact: x={current_x:.2f}m (required x in [{self.wall_contact_x_lo}, {self.wall_contact_x_hi}]m)"
        min_mass = getattr(self.environment, 'MIN_STRUCTURE_MASS', 0.0)
        current_mass = self.environment.get_structure_mass()
        if current_mass < min_mass and step_count == 0:
            pass
        elif current_mass < min_mass and step_count > 0:
            if self._failure_step < 0:
                self._failure_step = step_count
                self._failure_type = "mass_below"
            failed = True
            failure_reason = f"Design constraint violated: Total mass ({current_mass:.2f}kg) is below minimum required ({min_mass:.2f}kg)"
        if self._suction_zones and self._pad_suction_gap_y < 0:
            in_any_zone = any(y_lo <= current_y <= y_hi for (y_lo, y_hi) in self._suction_zones)
            if not in_any_zone:
                if current_y > 1.5 and phys_state and phys_state.get("num_active_pads", 0) > 0:
                    self._pad_suction_gap_y = round(current_y, 2)
        target_y = self.target_height
        is_above_target = (current_y >= target_y)
        success = is_above_target and step_count >= self.min_simulation_steps
        is_end = (step_count >= max_steps - 1)
        done = failed or success or is_end
        if success and not failed:
            score = 100.0
        elif failed:
            score = 0.0
        else:
            height_progress = min(max(0, current_y - self.initial_y) / (target_y - self.initial_y), 1.0)
            time_progress = min(step_count, self.min_simulation_steps) / self.min_simulation_steps
            score = height_progress * 70.0 + time_progress * 30.0
        nearest_suction_boundary_y = None
        distance_to_suction_boundary = None
        suction_boundary_type = None
        if self._suction_zones:
            best_dist = float("inf")
            for (y_lo, y_hi) in self._suction_zones:
                if y_lo <= current_y <= y_hi:
                    dist_to_lo = current_y - y_lo
                    dist_to_hi = y_hi - current_y
                    nearest_edge = min(dist_to_lo, dist_to_hi)
                    if nearest_edge < best_dist:
                        best_dist = nearest_edge
                        nearest_suction_boundary_y = y_lo if dist_to_lo < dist_to_hi else y_hi
                        distance_to_suction_boundary = round(nearest_edge, 2)
                        suction_boundary_type = "lower" if dist_to_lo < dist_to_hi else "upper"
                    break
            if distance_to_suction_boundary is None and current_y > 1.5:
                for (y_lo, y_hi) in self._suction_zones:
                    dist = min(abs(current_y - y_lo), abs(current_y - y_hi))
                    if dist < best_dist:
                        best_dist = dist
                        nearest_suction_boundary_y = y_lo if abs(current_y - y_lo) < abs(current_y - y_hi) else y_hi
                        distance_to_suction_boundary = round(dist, 2)
                        suction_boundary_type = "lower" if abs(current_y - y_lo) < abs(current_y - y_hi) else "upper"
        nan_flag = False
        inf_flag = False
        extreme_speed_flag = False
        try:
            if not math.isfinite(current_x) or not math.isfinite(current_y):
                nan_flag = True
            if math.isinf(current_x) or math.isinf(current_y):
                inf_flag = True
        except (TypeError, ValueError):
            nan_flag = True
        if self._peak_body_speed > 100.0 or self._peak_body_ang_vel > 100.0:
            extreme_speed_flag = True
        metrics = {
            'climber_x': current_x,
            'climber_y': current_y,
            'height_gained': current_y - self.initial_y,
            'max_height_reached': self.max_y_reached,
            'min_height_seen': self.min_height_seen,
            'climber_fell': self.min_height_seen < self.fell_height_threshold,
            'fell_height_threshold': self.fell_height_threshold,
            'target_y': target_y,
            'progress': height_progress * 100.0,
            'success': success and not failed,
            'failed': failed,
            'failure_reason': failure_reason,
            'step_count': step_count,
            'min_simulation_steps_required': self.min_simulation_steps,
            'structure_mass': self.environment.get_structure_mass(),
            'max_structure_mass': getattr(self.environment, 'MAX_STRUCTURE_MASS', 50.0),
            'min_structure_mass': getattr(self.environment, 'MIN_STRUCTURE_MASS', 0.0),
            'wall_contact_x_lo': self.wall_contact_x_lo,
            'wall_contact_x_hi': self.wall_contact_x_hi,
            'build_zone_x_min': getattr(self.environment, 'BUILD_ZONE_X_MIN', 0.0),
            'build_zone_x_max': getattr(self.environment, 'BUILD_ZONE_X_MAX', 5.0),
            'build_zone_y_min': getattr(self.environment, 'BUILD_ZONE_Y_MIN', 0.0),
            'build_zone_y_max': getattr(self.environment, 'BUILD_ZONE_Y_MAX', 25.0),
            'failure_step': self._failure_step,
            'failure_type': self._failure_type if failed else "",
            'joints_initial': self._initial_joint_count,
            'joints_remaining': len(self.environment._joints),
            'joints_broken': self._joints_broken,
            'peak_joint_force_pct': round(self._peak_joint_force_pct, 2),
            'peak_joint_torque_pct': round(self._peak_joint_torque_pct, 2),
            'max_joint_force_limit': self._max_joint_force,
            'max_joint_torque_limit': self._max_joint_torque,
            'gravity_initial_y': self._gravity_initial_y,
            'gravity_evolution': self._gravity_evolution,
            'gravity_at_failure': self._gravity_at_failure if failed else None,
            'wind_force': self._wind_force,
            'vortex_y': self._vortex_y,
            'vortex_force_x': self._vortex_force_x,
            'vortex_force_y': self._vortex_force_y,
            'suction_zones': self._suction_zones,
            'pad_suction_gap_y': self._pad_suction_gap_y if self._pad_suction_gap_y > 0 else None,
            'nearest_suction_boundary_y': nearest_suction_boundary_y,
            'distance_to_suction_boundary': distance_to_suction_boundary,
            'suction_boundary_type': suction_boundary_type,
            'physics_step': phys_state.get("step") if phys_state else None,
            'physics_gravity_y': round(phys_state.get("gravity_y", 0.0), 4) if phys_state else None,
            'physics_wind_force_x': round(phys_state.get("wind_force_x", 0.0), 3) if phys_state else None,
            'physics_num_active_pads': phys_state.get("num_active_pads") if phys_state else None,
            'physics_peak_joint_force_pct': round(phys_state.get("peak_joint_force_pct", 0.0), 2) if phys_state else None,
            'physics_total_ke': round(phys_state.get("total_ke", 0.0), 3) if phys_state else None,
            'physics_total_pe': round(phys_state.get("total_pe", 0.0), 3) if phys_state else None,
            'physics_max_body_speed': round(phys_state.get("max_body_speed", 0.0), 3) if phys_state else None,
            'physics_max_body_ang_vel': round(phys_state.get("max_body_ang_vel", 0.0), 3) if phys_state else None,
            'peak_total_ke': round(self._peak_total_ke, 3),
            'peak_total_pe': round(self._peak_total_pe, 3),
            'peak_body_speed': round(self._peak_body_speed, 3),
            'peak_body_ang_vel': round(self._peak_body_ang_vel, 3),
            'joint_stress_summary': phys_state.get("joint_stress_per_joint", []) if phys_state else [],
            'joint_failure_events': self._joint_failure_events,
            'nan_flag': nan_flag,
            'inf_flag': inf_flag,
            'extreme_speed_flag': extreme_speed_flag,
            'initial_y': self.initial_y,
            'observation_error_count': getattr(self.environment, '_observation_error_count', 0),
            'last_observation_error': getattr(self.environment, '_last_observation_error', None),
        }
        return done, score, metrics
    def compute_score_with_penalty(self, score: float, metrics: dict) -> float:
        if score > 0:
            return score
        constraint_info = self.get_constraint_info()
        penalty = compute_constraint_penalty("K_02", score, metrics, constraint_info)
        return penalty
    def get_constraint_info(self):
        return {
            'target_height': self.target_height,
            'fell_height_threshold': self.fell_height_threshold,
            'wall_contact_x_lo': self.wall_contact_x_lo,
            'wall_contact_x_hi': self.wall_contact_x_hi,
            'min_simulation_time': self.min_simulation_time,
            'max_joint_force': self._max_joint_force,
            'max_joint_torque': self._max_joint_torque,
            'gravity_initial_y': self._gravity_initial_y,
            'gravity_evolution': self._gravity_evolution,
            'build_zone_x_min': getattr(self.environment, 'BUILD_ZONE_X_MIN', 0.0),
            'build_zone_x_max': getattr(self.environment, 'BUILD_ZONE_X_MAX', 5.0),
            'build_zone_y_min': getattr(self.environment, 'BUILD_ZONE_Y_MIN', 0.0),
            'build_zone_y_max': getattr(self.environment, 'BUILD_ZONE_Y_MAX', 25.0),
        }
    def get_task_description(self):
        return {
            'task': 'K-02: The Climber',
            'success_criteria': {
                'height': f'Reach height {self.target_height}m',
                'time': f'Climb for {self.min_simulation_time}s'
            }
        }
