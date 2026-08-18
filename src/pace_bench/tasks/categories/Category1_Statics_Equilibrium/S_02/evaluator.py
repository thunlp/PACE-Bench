import math

from pace_bench.core.primitives import compute_constraint_penalty

class Evaluator:
    def __init__(self, terrain_bounds, environment=None):
        self.terrain_bounds = terrain_bounds
        self.environment = environment
        self.TARGET_HEIGHT = 30.0
        self.SURVIVAL_THRESHOLD = 5.0
        self.STABILITY_ZONE = 300.0
        self.initial_height = 0.0
        self.min_height_during_quake = float('inf')
        self.design_constraints_checked = False
        self.quake_start_time = getattr(environment, "_earthquake_start_time", 2.0) if environment else 2.0
        self.quake_start_step = int(self.quake_start_time * 60.0)
    def evaluate(self, agent_body, step_count, max_steps):
        if self.environment is None:
            return True, 0.0, {"error": "Environment not set"}
        bounds = self.environment.get_structure_bounds()
        current_height = bounds.get("top", 0)
        if 10 <= step_count < self.quake_start_step:
            self.initial_height = max(self.initial_height, current_height)
        if step_count >= self.quake_start_step:
            self.min_height_during_quake = min(self.min_height_during_quake, current_height)
        if "foundation" in self.environment._terrain_bodies:
            foundation_x = self.environment._terrain_bodies["foundation"].position.x
        else:
            foundation_x = 0.0
        rel_com_x = 0.0
        total_mass = 0.0
        import Box2D
        for body in self.environment._bodies:
            if body.type == Box2D.b2_dynamicBody:
                total_mass += body.mass
                rel_com_x += body.position.x * body.mass
        if total_mass > 0:
            rel_com_x = (rel_com_x / total_mass) - foundation_x
        failed = False
        reason = None
        if not failed:
            if bounds.get("width", 0) > 24.0:
                failed, reason = True, f"Width {bounds.get('width', 0):.2f}m > 24.0m"
            if not failed:
                for body in self.environment._bodies:
                    if body.type != Box2D.b2_dynamicBody:
                        continue
                    for fixture in body.fixtures:
                        shape = fixture.shape
                        if hasattr(shape, 'vertices') and len(shape.vertices) >= 2:
                            def _vx(v):
                                return v.x if hasattr(v, 'x') else v[0]
                            def _vy(v):
                                return v.y if hasattr(v, 'y') else v[1]
                            xs = [_vx(shape.vertices[i]) for i in range(len(shape.vertices))]
                            ys = [_vy(shape.vertices[i]) for i in range(len(shape.vertices))]
                            beam_w = max(xs) - min(xs)
                            beam_h = max(ys) - min(ys)
                            if beam_w < 0.1 or beam_w > 10.0 or beam_h < 0.1 or beam_h > 10.0:
                                failed, reason = True, f"Beam dimensions {beam_w:.2f}m x {beam_h:.2f}m outside [0.1, 10.0]m"
                                break
                        for vertex in shape.vertices:
                            world_v = body.GetWorldPoint(vertex)
                            if world_v.y < 1.01 and abs(world_v.x - foundation_x) > 4.5:
                                failed, reason = True, f"Foundation contact violation at x={world_v.x:.2f} (Limit: ±4.5m)"
                                break
                        if failed:
                            break
                    if failed:
                        break
        if not failed and step_count >= self.quake_start_step:
            if current_height < self.SURVIVAL_THRESHOLD:
                failed, reason = True, "Collapsed"
            elif abs(rel_com_x) > self.STABILITY_ZONE:
                failed, reason = True, f"Tipped Over (rel_com_x={rel_com_x:.2f}, limit={self.STABILITY_ZONE})"
            elif current_height > 150.0:
                failed, reason = True, "Physical instability (Explosion)"
        is_end = (step_count >= max_steps - 1)
        success = False
        if is_end and not failed:
            if self.initial_height < self.TARGET_HEIGHT:
                failed, reason = True, f"Target height not reached (Max: {self.initial_height:.1f}m, Target: {self.TARGET_HEIGHT}m)"
            elif self.min_height_during_quake < self.SURVIVAL_THRESHOLD:
                failed, reason = True, f"Tower collapsed or fell too low during earthquake ({self.min_height_during_quake:.1f}m < {self.SURVIVAL_THRESHOLD}m)"
            else:
                success = True
        done = failed or is_end
        if success:
            score = 100.0
        elif failed:
            score = 0.0
        else:
            height_score = min(current_height / self.TARGET_HEIGHT, 1.0) * 60.0
            stability_penalty = max(0, abs(rel_com_x) / self.STABILITY_ZONE) * 20.0
            score = max(0.0, height_score - stability_penalty)
        peak_joint_force = getattr(self.environment, "_peak_joint_force", 0.0)
        peak_joint_torque = getattr(self.environment, "_peak_joint_torque", 0.0)
        joint_break_count = getattr(self.environment, "_joint_break_count", 0)
        peak_foundation_disp = getattr(self.environment, "_peak_foundation_displacement", 0.0)
        structure_mass = getattr(self.environment, "_total_structure_mass", 0.0)
        num_bodies = len(getattr(self.environment, "_bodies", []))
        num_joints = len(getattr(self.environment, "_joints", []))
        max_joint_force_limit = getattr(self.environment, "_max_joint_force", float('inf'))
        max_joint_torque_limit = getattr(self.environment, "_max_joint_torque", float('inf'))
        earthquake_freq = getattr(self.environment, "_earthquake_frequency", 2.0)
        earthquake_amp = getattr(self.environment, "_earthquake_amplitude", 0.5)
        max_steps_setting = max_steps
        constraint_profile = {}
        f_width = bounds.get("width", 0)
        constraint_profile['width'] = {
            'label': 'Structure width ≤ 24.0m',
            'passed': f_width <= 24.0,
            'value': float(f_width),
            'limit': 24.0,
            'margin': 24.0 - float(f_width),
            'type': 'runtime',
        }
        beam_dim_ok = True
        beam_dim_worst = None
        for body in self.environment._bodies:
            if body.type != Box2D.b2_dynamicBody:
                continue
            for fixture in body.fixtures:
                shape = fixture.shape
                if hasattr(shape, 'vertices') and len(shape.vertices) >= 2:
                    xs = [(v.x if hasattr(v, 'x') else v[0]) for v in shape.vertices]
                    ys = [(v.y if hasattr(v, 'y') else v[1]) for v in shape.vertices]
                    bw, bh = max(xs) - min(xs), max(ys) - min(ys)
                    if bw < 0.1 or bw > 10.0 or bh < 0.1 or bh > 10.0:
                        beam_dim_ok = False
                        beam_dim_worst = {'width': float(bw), 'height': float(bh)}
                        break
                if not beam_dim_ok:
                    break
            if not beam_dim_ok:
                break
        constraint_profile['beam_dimensions'] = {
            'label': 'Beam dimensions in [0.1, 10.0]m',
            'passed': beam_dim_ok,
            'value': beam_dim_worst,
            'limit': {'min': 0.1, 'max': 10.0},
            'type': 'build_time',
        }
        fc_ok = True
        fc_worst = None
        fc_breach_count = 0
        for body in self.environment._bodies:
            if body.type != Box2D.b2_dynamicBody:
                continue
            for fixture in body.fixtures:
                for vertex in fixture.shape.vertices:
                    world_v = body.GetWorldPoint(vertex)
                    if world_v.y < 1.01:
                        dist = abs(world_v.x - foundation_x)
                        if dist > 4.5:
                            fc_ok = False
                            fc_breach_count += 1
                            if fc_worst is None or dist > fc_worst.get('distance', 0):
                                fc_worst = {
                                    'x': float(world_v.x),
                                    'y': float(world_v.y),
                                    'distance': float(dist),
                                }
        constraint_profile['foundation_contact'] = {
            'label': f'Foundation contact within ±4.5m',
            'passed': fc_ok,
            'value': fc_worst,
            'breach_count': fc_breach_count,
            'limit': {'half_width': 4.5},
            'type': 'runtime' if step_count > 0 else 'build_time',
        }
        if step_count >= self.quake_start_step:
            collapse_passed = current_height >= self.SURVIVAL_THRESHOLD
            constraint_profile['collapse'] = {
                'label': f'Survive quake (height ≥ {self.SURVIVAL_THRESHOLD}m)',
                'passed': collapse_passed,
                'value': float(current_height),
                'limit': float(self.SURVIVAL_THRESHOLD),
                'margin': float(current_height) - float(self.SURVIVAL_THRESHOLD),
                'type': 'runtime',
            }
            tip_passed = abs(rel_com_x) <= self.STABILITY_ZONE
            constraint_profile['tipped_over'] = {
                'label': f'COM within ±{self.STABILITY_ZONE}m',
                'passed': tip_passed,
                'value': float(abs(rel_com_x)),
                'limit': float(self.STABILITY_ZONE),
                'margin': float(self.STABILITY_ZONE) - float(abs(rel_com_x)),
                'type': 'runtime',
            }
            expl_passed = current_height <= 150.0
            constraint_profile['explosion'] = {
                'label': 'No physical instability (height ≤ 150m)',
                'passed': expl_passed,
                'value': float(current_height),
                'limit': 150.0,
                'margin': 150.0 - float(current_height),
                'type': 'runtime',
            }
        if is_end:
            th_passed = self.initial_height >= self.TARGET_HEIGHT
            constraint_profile['target_height'] = {
                'label': f'Target height ≥ {self.TARGET_HEIGHT}m',
                'passed': th_passed,
                'value': float(self.initial_height),
                'limit': float(self.TARGET_HEIGHT),
                'margin': float(self.initial_height) - float(self.TARGET_HEIGHT),
                'type': 'final',
            }
        per_joint_stress = []
        if hasattr(self.environment, 'get_joint_peak_data'):
            per_joint_stress = self.environment.get_joint_peak_data()
        joint_failure_events = []
        if hasattr(self.environment, 'get_joint_failure_events'):
            joint_failure_events = self.environment.get_joint_failure_events()
        per_beam_positions = []
        if hasattr(self.environment, 'get_beam_positions'):
            per_beam_positions = self.environment.get_beam_positions()
        max_body_vel = getattr(self.environment, '_max_body_velocity', 0.0)
        env_params = {}
        if hasattr(self.environment, 'get_environment_params'):
            env_params = self.environment.get_environment_params()
        initial_joint_count = num_joints + joint_break_count
        return done, score, {
            "initial_height": self.initial_height,
            "min_height_during_quake": self.min_height_during_quake if step_count >= self.quake_start_step else None,
            "rel_com_x": rel_com_x,
            "current_height": current_height,
            "success": success,
            "failed": failed,
            "failure_reason": reason,
            "target_height": self.TARGET_HEIGHT,
            "survival_threshold": self.SURVIVAL_THRESHOLD,
            "stability_zone": self.STABILITY_ZONE,
            "max_width_limit": 24.0,
            "instability_height_limit": 150.0,
            "peak_joint_force": peak_joint_force,
            "peak_joint_torque": peak_joint_torque,
            "joint_break_count": joint_break_count,
            "peak_foundation_displacement": peak_foundation_disp,
            "structure_mass": structure_mass,
            "num_bodies": num_bodies,
            "num_joints": num_joints,
            "max_joint_force_limit": max_joint_force_limit if max_joint_force_limit < float('inf') else None,
            "max_joint_torque_limit": max_joint_torque_limit if max_joint_torque_limit < float('inf') else None,
            "earthquake_frequency": earthquake_freq,
            "earthquake_amplitude": earthquake_amp,
            "eval_step": step_count,
            "max_steps_setting": max_steps_setting,
            "reached_final_check": is_end,
            "foundation_contact_limit": 4.5,
            "all_constraint_results": constraint_profile,
            "per_joint_stress_summary": per_joint_stress,
            "joint_failure_events": joint_failure_events,
            "per_beam_positions": per_beam_positions,
            "max_body_velocity": float(max_body_vel) if max_body_vel else 0.0,
            "env_params": env_params,
            "initial_joint_count": initial_joint_count,
            "build_zone_half_width": 4.5,
            "joint_observation_error_count": getattr(self.environment, "_joint_observation_error_count", 0),
            "last_joint_observation_error": getattr(self.environment, "_last_joint_observation_error", None),
        }
    def compute_score_with_penalty(self, score: float, metrics: dict) -> float:
        if score > 0:
            return score
        constraint_info = self.get_constraint_info()
        penalty = compute_constraint_penalty("S_02", score, metrics, constraint_info)
        return penalty
    def get_constraint_info(self):
        return {
            'target_height': self.TARGET_HEIGHT,
            'survival_threshold': self.SURVIVAL_THRESHOLD,
            'stability_zone': self.STABILITY_ZONE,
            'max_width_limit': 24.0,
            'instability_height_limit': 150.0,
            'foundation_contact_limit': 4.5,
        }
    def get_task_description(self):
        return {
            "task": "S-02: The Skyscraper",
            "description": f"Build a tower > {self.TARGET_HEIGHT}m that survives an earthquake",
            "success_criteria": {
                "initial_height": f"> {self.TARGET_HEIGHT}m",
                "survival": f"Remain ≥ {self.SURVIVAL_THRESHOLD}m during quake",
                "stability": f"COM remains within ±{self.STABILITY_ZONE}m of foundation"
            }
        }
