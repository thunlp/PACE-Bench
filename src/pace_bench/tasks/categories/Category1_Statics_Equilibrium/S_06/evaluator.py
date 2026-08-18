import math

from Box2D.b2 import polygonShape

from pace_bench.core.primitives import compute_constraint_penalty

class Evaluator:
    def __init__(self, terrain_bounds, environment=None):
        self.terrain_bounds = terrain_bounds
        self.environment = environment
        self.stability_time = terrain_bounds.get("stability_time", 10.0)
        self.target_overhang = 0.1
        if environment and hasattr(environment, '_terrain_config'):
            self.target_overhang = environment._terrain_config.get("target_overhang", 0.1)
            if "stability_time" not in terrain_bounds:
                self.stability_time = environment._terrain_config.get("stability_time", 10.0)
            self.block_friction = environment._terrain_config.get("block_friction", 0.6)
            self.table_friction = environment._terrain_config.get("table_friction", 0.8)
        self.max_x_position = 0.0
        self.stability_start_time = None
        self.stable_duration = 0.0
        self.last_max_x = 0.0
        self.min_y_position = float('inf')
        self.max_y_position = float('-inf')
        self.structure_mass = 0.0
        self.total_kinetic_energy = 0.0
        self.max_velocity = 0.0
        self.step_at_failure = None
        self.time_at_failure = None
        self.failure_type = None
        self.same_height_overlap_count = 0
        self.y_levels = []
        self.com_to_edge_margin = None
        self.table_edge_x = terrain_bounds.get("edge_x", 0.0)
        self.peak_kinetic_energy = 0.0
        self.ke_spike_detected = False
        self._prev_kinetic_energy = 0.0
        self._failure_events = []
        self._first_movement_tracked = False
        self._com_crossed_edge_tracked = False
        self._first_block_fell_tracked = False
        self._peak_ke_tracked = False
        self._peak_ke_step = None
        self._peak_ke_value = 0.0
        self._peak_ke_event_recorded = False
        self._design_constraint_violations = []
        if not environment:
            raise ValueError("Evaluator requires environment instance")
        env_class = type(environment)
        self.MAX_BLOCK_LENGTH = env_class.MAX_BLOCK_LENGTH
        self.MAX_BLOCK_HEIGHT = env_class.MAX_BLOCK_HEIGHT
        self.MAX_BLOCK_COUNT = env_class.MAX_BLOCK_COUNT
        self.START_ZONE_X_MAX = None
        self.SPAWN_ZONE = terrain_bounds.get("spawn_zone", [-10.0, 0.0])
        self.MAX_TOTAL_MASS = terrain_bounds.get("max_total_mass", 20000.0)
        if self.MAX_TOTAL_MASS == 20000.0 and environment and hasattr(environment, '_terrain_config'):
            self.MAX_TOTAL_MASS = environment._terrain_config.get("max_total_mass", 20000.0)
        self.design_constraints_checked = False
        self.persistently_failed = False
        self.persistent_failure_reason = None
    def evaluate(self, agent_body, step_count, max_steps):
        if not self.environment:
            return False, 0.0, {"error": "Environment not available"}
        table_vel = (0, 0)
        if hasattr(self.environment, '_terrain_bodies') and 'table' in self.environment._terrain_bodies:
            table_vel = self.environment._terrain_bodies['table'].linearVelocity
        current_max_x = self.environment.get_max_x_position()
        self.max_x_position = max(self.max_x_position, current_max_x)
        if self.environment._bodies:
            y_positions = [b.position.y for b in self.environment._bodies]
            self.min_y_position = min(y_positions)
            self.max_y_position = max(y_positions)
            self.structure_mass = self.environment.get_structure_mass()
            self.total_kinetic_energy = 0.0
            self.max_velocity = 0.0
            for body in self.environment._bodies:
                vx, vy = body.linearVelocity
                rel_vx = vx - table_vel[0]
                rel_vy = vy - table_vel[1]
                velocity = math.sqrt(rel_vx*rel_vx + rel_vy*rel_vy)
                self.max_velocity = max(self.max_velocity, velocity)
                kinetic_energy = 0.5 * body.mass * velocity * velocity
                self.total_kinetic_energy += kinetic_energy
        time_step = 1.0 / 60.0
        current_time = step_count * time_step
        if step_count == 0 and self.environment._bodies:
            overlap_count, _ = self.environment.detect_same_height_overlaps()
            self.same_height_overlap_count = overlap_count
            self.y_levels = self.environment.get_block_y_levels()
        elif self.environment._bodies:
            self.y_levels = self.environment.get_block_y_levels()
        self.com_to_edge_margin = self.environment.get_com_to_edge_margin(self.table_edge_x)
        self.peak_kinetic_energy = max(self.peak_kinetic_energy, self.total_kinetic_energy)
        if (self.total_kinetic_energy > 0 and
                self._prev_kinetic_energy > 0 and
                self.total_kinetic_energy > self._prev_kinetic_energy * 10.0):
            self.ke_spike_detected = True
        self._prev_kinetic_energy = self.total_kinetic_energy
        if step_count > 0 and self.environment._bodies:
            if not self._first_movement_tracked and self.max_velocity > 0.01:
                self._first_movement_tracked = True
                self._failure_events.append({
                    "event": "first_movement",
                    "step": step_count,
                    "time": current_time,
                    "max_velocity": self.max_velocity,
                })
            if not self._com_crossed_edge_tracked and self.com_to_edge_margin is not None:
                if self.com_to_edge_margin > 0:
                    self._com_crossed_edge_tracked = True
                    self._failure_events.append({
                        "event": "com_crossed_edge",
                        "step": step_count,
                        "time": current_time,
                        "com_x_margin": self.com_to_edge_margin,
                    })
            if not self._first_block_fell_tracked and self.min_y_position < -5.0:
                self._first_block_fell_tracked = True
                self._failure_events.append({
                    "event": "first_block_fell",
                    "step": step_count,
                    "time": current_time,
                    "min_y": self.min_y_position,
                })
            if self.total_kinetic_energy > self._peak_ke_value:
                self._peak_ke_value = self.total_kinetic_energy
                self._peak_ke_step = step_count
                self._peak_ke_tracked = True
        if step_count == 0 and self.environment._bodies:
            self._failure_events.append({
                "event": "simulation_start",
                "step": 0,
                "time": 0.0,
                "block_count": len(self.environment._bodies),
                "total_mass": self.structure_mass,
            })
        if not self.persistently_failed and step_count > 0:
            will_fail = False
            fail_type = None
            if self.environment._bodies and self.min_y_position < -5.0:
                will_fail = True; fail_type = 'fell_off_table'
            elif self.environment._bodies and self.structure_mass > self.MAX_TOTAL_MASS + 0.01:
                will_fail = True; fail_type = 'mass_overrun'
            if "ceiling_y" in self.terrain_bounds:
                cy = self.terrain_bounds["ceiling_y"]
                if self.max_y_position > cy + 0.01:
                    will_fail = True; fail_type = 'hit_ceiling'
            if will_fail and self.step_at_failure is None:
                self.step_at_failure = step_count
                self.time_at_failure = current_time
                self.failure_type = fail_type
        is_moving = self.max_velocity > 0.01 or self.total_kinetic_energy > 0.01
        if not is_moving:
            if self.stability_start_time is None:
                self.stability_start_time = current_time
            self.stable_duration = current_time - self.stability_start_time
        else:
            self.stability_start_time = None
            self.stable_duration = 0.0
        self.last_max_x = current_max_x
        stability_ok = self.stable_duration >= self.stability_time - 1e-6
        overhang_ok = self.max_x_position >= self.target_overhang - 1e-6
        success = stability_ok and overhang_ok
        if not self.design_constraints_checked and step_count == 0:
            constraint_violations = self._check_design_constraints()
            if constraint_violations:
                self.persistently_failed = True
                self.persistent_failure_reason = "Design constraint violated: " + "; ".join(constraint_violations)
                self.step_at_failure = 0
                self.time_at_failure = 0.0
                self.failure_type = 'spawn_violation'
            self.design_constraints_checked = True
        if self.environment._bodies:
            if self.min_y_position < -5.0:
                self.persistently_failed = True
                self.persistent_failure_reason = "Structure fell off table"
            if self.structure_mass > self.MAX_TOTAL_MASS + 0.01:
                self.persistently_failed = True
                self.persistent_failure_reason = f"Structure exceeds maximum mass: {self.structure_mass:.2f} > {self.MAX_TOTAL_MASS}"
        if "ceiling_y" in self.terrain_bounds:
            cy = self.terrain_bounds["ceiling_y"]
            if self.max_y_position > cy + 0.01:
                self.persistently_failed = True
                self.persistent_failure_reason = f"Structure hit the ceiling at y={cy}m"
        failed = self.persistently_failed
        failure_reason = self.persistent_failure_reason
        if success and not failed:
            score = 100.0
        elif failed:
            score = 0.0
        else:
            stability_score = min(self.stable_duration / self.stability_time, 1.0) * 50.0
            overhang_score = min(self.max_x_position / self.target_overhang, 1.0) * 50.0
            score = stability_score + overhang_score
        if self._peak_ke_tracked and not self._peak_ke_event_recorded and self._peak_ke_step is not None:
            self._failure_events.append({
                "event": "peak_kinetic_energy",
                "step": self._peak_ke_step,
                "time": self._peak_ke_step / 60.0,
                "ke_value": self._peak_ke_value,
            })
            self._peak_ke_event_recorded = True
        metrics = {
            'max_x_position': self.max_x_position,
            'target_overhang': self.target_overhang,
            'stable_duration': self.stable_duration,
            'target_stability_time': self.stability_time,
            'stability_ok': stability_ok,
            'overhang_ok': overhang_ok,
            'success': success and not failed,
            'failed': failed,
            'failure_reason': failure_reason,
            'step_count': step_count,
            'min_y_position': self.min_y_position,
            'max_y_position': self.max_y_position,
            'structure_mass': self.structure_mass,
            'block_count': len(self.environment._bodies),
            'max_block_count_limit': self.MAX_BLOCK_COUNT,
            'max_block_length_limit': self.MAX_BLOCK_LENGTH,
            'max_block_height_limit': self.MAX_BLOCK_HEIGHT,
            'max_total_mass_limit': self.MAX_TOTAL_MASS,
            'total_kinetic_energy': self.total_kinetic_energy,
            'max_velocity': self.max_velocity,
            'peak_kinetic_energy': self.peak_kinetic_energy,
            'ke_spike_detected': self.ke_spike_detected,
            'ceiling_y_limit': self.terrain_bounds.get("ceiling_y", None),
            'table_edge_x': self.table_edge_x,
            'table_friction': self.table_friction,
            'block_friction': self.block_friction,
            'step_at_failure': self.step_at_failure,
            'time_at_failure': self.time_at_failure,
            'failure_type': self.failure_type,
            'y_levels': list(self.y_levels),
            'same_height_overlap_count': self.same_height_overlap_count,
            'com_to_edge_margin': self.com_to_edge_margin,
            'wind_force': list(self.environment.get_wind_force()) if hasattr(self.environment, 'get_wind_force') else [0.0, 0.0],
            'gravity': list(self.environment.get_gravity()) if hasattr(self.environment, 'get_gravity') else [0.0, -10.0],
            'oscillation_active': self.environment.get_oscillation_params()[0] if hasattr(self.environment, 'get_oscillation_params') else False,
            'osc_amplitude': self.environment.get_oscillation_params()[1] if hasattr(self.environment, 'get_oscillation_params') else 0.0,
            'osc_frequency': self.environment.get_oscillation_params()[2] if hasattr(self.environment, 'get_oscillation_params') else 0.0,
            'block_density_default': self.environment.get_block_density_default() if hasattr(self.environment, 'get_block_density_default') else 1.0,
            'spawn_zone': list(self.environment.get_spawn_zone()) if hasattr(self.environment, 'get_spawn_zone') else [-10.0, 0.0],
            'floor_length': self.environment.get_floor_length() if hasattr(self.environment, 'get_floor_length') else 20.0,
            'table_angle_deg': self.environment.get_table_angle() if hasattr(self.environment, 'get_table_angle') else 0.0,
            'failure_event_sequence': list(self._failure_events),
            'design_constraint_violations': list(self._design_constraint_violations),
            'per_block_extents': self.environment.get_per_block_extents() if hasattr(self.environment, 'get_per_block_extents') else [],
            'table_velocity': list(self.environment.get_table_velocity()) if hasattr(self.environment, 'get_table_velocity') else None,
            'time_step': time_step,
        }
        if self.environment._bodies:
            total_mass = sum(b.mass for b in self.environment._bodies)
            if total_mass > 0:
                com_x = sum(b.position.x * b.mass for b in self.environment._bodies) / total_mass
                com_y = sum(b.position.y * b.mass for b in self.environment._bodies) / total_mass
                metrics['center_of_mass_x'] = com_x
                metrics['center_of_mass_y'] = com_y
        return success or failed, score, metrics
    def _check_design_constraints(self):
        violations = []
        violation_details = []
        if len(self.environment._bodies) > self.MAX_BLOCK_COUNT:
            detail = f"Too many blocks: {len(self.environment._bodies)} > {self.MAX_BLOCK_COUNT}"
            violations.append(detail)
            violation_details.append({"constraint": "block_count", "value": len(self.environment._bodies), "limit": self.MAX_BLOCK_COUNT, "detail": detail})
        for body in self.environment._bodies:
            if not (self.SPAWN_ZONE[0] - 0.01 <= body.position.x <= self.SPAWN_ZONE[1] + 0.01):
                detail = f"Block center at x={body.position.x:.2f} is outside spawn zone [{self.SPAWN_ZONE[0]}, {self.SPAWN_ZONE[1]}]"
                violations.append(detail)
                violation_details.append({"constraint": "spawn_zone", "position_x": body.position.x, "spawn_min": self.SPAWN_ZONE[0], "spawn_max": self.SPAWN_ZONE[1], "detail": detail})
            for fixture in body.fixtures:
                shape = fixture.shape
                if isinstance(shape, polygonShape):
                    width = (shape.vertices[1][0] - shape.vertices[0][0])
                    height = (shape.vertices[2][1] - shape.vertices[1][1])
                    if width > self.MAX_BLOCK_LENGTH + 0.01:
                        detail = f"Block width {width:.2f}m exceeds maximum {self.MAX_BLOCK_LENGTH}m"
                        violations.append(detail)
                        violation_details.append({"constraint": "block_width", "value": width, "limit": self.MAX_BLOCK_LENGTH, "detail": detail})
                    if height > self.MAX_BLOCK_HEIGHT + 0.01:
                        detail = f"Block height {height:.2f}m exceeds maximum {self.MAX_BLOCK_HEIGHT}m"
                        violations.append(detail)
                        violation_details.append({"constraint": "block_height", "value": height, "limit": self.MAX_BLOCK_HEIGHT, "detail": detail})
        self._design_constraint_violations = violation_details
        return violations
    def compute_score_with_penalty(self, score: float, metrics: dict) -> float:
        if score > 0:
            return score
        constraint_info = self.get_constraint_info()
        penalty = compute_constraint_penalty("S_06", score, metrics, constraint_info)
        return penalty
    def get_constraint_info(self):
        return {
            'target_overhang': self.target_overhang,
            'stability_time': self.stability_time,
            'max_block_length': self.MAX_BLOCK_LENGTH,
            'max_block_height': self.MAX_BLOCK_HEIGHT,
            'max_block_count': self.MAX_BLOCK_COUNT,
            'spawn_zone': self.SPAWN_ZONE,
            'max_total_mass': self.MAX_TOTAL_MASS,
            'block_friction': self.block_friction,
            'table_friction': self.table_friction,
        }
    def get_task_description(self):
        return {
            'task': 'S-06: The Overhang',
            'description': 'Stack blocks to create maximum overhang under severe physical constraints',
            'target_overhang': self.target_overhang,
            'success_criteria': {
                'stability': f'Structure stays still for {self.stability_time}s',
                'overhang': f'Max x position > {self.target_overhang}m',
                'mass': f'Total mass must be <= {self.MAX_TOTAL_MASS} units'
            }
        }
