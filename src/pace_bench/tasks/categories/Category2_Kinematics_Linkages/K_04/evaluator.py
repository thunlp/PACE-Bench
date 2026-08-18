import math

from pace_bench.core.simulator import TIME_STEP

from pace_bench.core.primitives import compute_constraint_penalty

class Evaluator:
    def __init__(self, terrain_bounds, environment=None):
        self.terrain_bounds = terrain_bounds
        self.environment = environment
        self.target_distance = float(terrain_bounds.get("target_distance", 10.0))
        ground_info = terrain_bounds.get("ground") or {}
        self.ground_y = float(ground_info.get("y", 1.0))
        self.min_simulation_time = 12.0
        self.min_simulation_steps = int(self.min_simulation_time / TIME_STEP)
        if environment and hasattr(environment, '_object_to_push') and environment._object_to_push is not None:
            self.initial_object_x = environment._object_to_push.position.x
        else:
            self.initial_object_x = 8.0
        self.max_x_reached = None
        self.max_pusher_tilt = 0.0
        self._wheel_contact_heights = []
        self._first_wheel_suspension_step = None
        self._first_wheel_spinning_step = None
        self._suspension_step_count = 0
        self._spinning_step_count = 0
        self._max_wheel_suspension_depth = 0.0
        self._max_wheel_tangential_speed = 0.0
        self._overall_wheel_state = None
        self._first_contact_step = None
        self._first_object_motion_step = None
        self._pusher_object_separation_at_contact = None
        self._peak_object_velocity = 0.0
        self._numerical_instability_flag = False
        if environment:
            self.MAX_STRUCTURE_MASS = getattr(environment, 'MAX_STRUCTURE_MASS', 40.0)
            self.BUILD_ZONE_X_MIN = getattr(environment, 'BUILD_ZONE_X_MIN', 0.0)
            self.BUILD_ZONE_X_MAX = getattr(environment, 'BUILD_ZONE_X_MAX', 15.0)
            self.BUILD_ZONE_Y_MIN = getattr(environment, 'BUILD_ZONE_Y_MIN', 1.5)
            self.BUILD_ZONE_Y_MAX = getattr(environment, 'BUILD_ZONE_Y_MAX', 8.0)
        self.design_constraints_checked = False
        self._motor_ever_active = False
        self._motor_peak_saturation = 0.0
        self._motor_cmd_speeds = []
        self._motor_actual_speeds = []
        self._motor_torques_used = []
        self._wheel_ever_contacted = False
        self._wheel_min_gap = float('inf')
        self._wheel_radii = []
        self._wheel_positions = []
        self._wheel_geo_collected = False
        self._peak_chassis_ke = 0.0
        self._peak_object_ke = 0.0
        self._cumulative_motor_energy_est = 0.0
        self._contact_ever_made = False
        self._contact_peak_normal_force = 0.0
        self._contact_heights = []
        self._temporal_events = []
        self._event_tags_seen = set()
        self._diagnostic_error_count = 0
        self._last_diagnostic_error = None
        self._prev_object_vx = 0.0
        self._peak_body_velocity = 0.0
        self._prev_step_count = -1
        self._diagnostics_collected = False
    def _collect_diagnostics(self, agent_body, step_count):
        if self._diagnostics_collected:
            return
        if not self.environment:
            return
        env = self.environment
        world = env._world
        object_body = getattr(env, '_object_to_push', None)
        ground_y = self.ground_y
        dt = TIME_STEP
        motor_data = []
        for j_idx, joint in enumerate(env._joints):
            try:
                from Box2D import b2RevoluteJoint
                if not isinstance(joint, b2RevoluteJoint):
                    continue
                if not getattr(joint, 'enableMotor', False):
                    continue
                self._motor_ever_active = True
                cmd_speed = float(joint.motorSpeed)
                actual_speed = float(joint.speed)
                torque_used = float(joint.motorTorque)
                torque_limit = float(joint.maxMotorTorque)
                saturation = torque_used / torque_limit if torque_limit > 0 else 0.0
                self._motor_peak_saturation = max(self._motor_peak_saturation, saturation)
                self._motor_cmd_speeds.append(cmd_speed)
                self._motor_actual_speeds.append(actual_speed)
                self._motor_torques_used.append(torque_used)
                motor_data.append({
                    'joint_index': j_idx,
                    'cmd_speed': cmd_speed,
                    'actual_speed': actual_speed,
                    'torque_used': torque_used,
                    'torque_limit': torque_limit,
                    'saturation': saturation,
                })
                motor_power = abs(torque_used * actual_speed)
                self._cumulative_motor_energy_est += motor_power * dt * 100.0
            except Exception as exc:
                self._record_diagnostic_error("motor", exc)
        from Box2D.b2 import circleShape
        wheel_data = []
        if not self._wheel_geo_collected:
            self._wheel_geo_collected = True
            for w_idx, body in enumerate(env._bodies):
                try:
                    for fixture in body.fixtures:
                        if isinstance(fixture.shape, circleShape):
                            radius = fixture.shape.radius
                            wx = float(body.position.x)
                            wy = float(body.position.y)
                            bottom_y = wy - radius
                            gap = bottom_y - ground_y
                            self._wheel_min_gap = min(self._wheel_min_gap, gap)
                            self._wheel_radii.append(radius)
                            self._wheel_positions.append((wx, wy))
                            if gap <= 0.01:
                                self._wheel_ever_contacted = True
                            wheel_data.append({
                                'wheel_index': w_idx,
                                'x': wx, 'y': wy,
                                'radius': radius,
                                'bottom_y': bottom_y,
                                'ground_y': ground_y,
                                'gap_to_ground': gap,
                                'contact': gap <= 0.01,
                            })
                            break
                except Exception as exc:
                    self._record_diagnostic_error("wheel_geometry", exc)
        if object_body and agent_body:
            try:
                chassis_vx = float(agent_body.linearVelocity.x)
                chassis_vy = float(agent_body.linearVelocity.y)
                chassis_mass = float(agent_body.mass)
                chassis_speed_sq = chassis_vx * chassis_vx + chassis_vy * chassis_vy
                chassis_ke = 0.5 * chassis_mass * chassis_speed_sq
                self._peak_chassis_ke = max(self._peak_chassis_ke, chassis_ke)
                obj_vx = float(object_body.linearVelocity.x)
                obj_vy = float(object_body.linearVelocity.y)
                obj_mass = float(object_body.mass)
                obj_ke = 0.5 * obj_mass * (obj_vx * obj_vx + obj_vy * obj_vy)
                self._peak_object_ke = max(self._peak_object_ke, obj_ke)
                for body in env._bodies:
                    v = body.linearVelocity.length
                    if math.isfinite(v):
                        self._peak_body_velocity = max(self._peak_body_velocity, float(v))
            except Exception as exc:
                self._record_diagnostic_error("energy_velocity", exc)
        if object_body:
            try:
                for contact in world.contacts:
                    if not contact.touching:
                        continue
                    fa = contact.fixtureA.body
                    fb = contact.fixtureB.body
                    if fa != object_body and fb != object_body:
                        continue
                    self._contact_ever_made = True
                    manifold = contact.worldManifold
                    for pt in manifold.points:
                        ni = float(pt.normalImpulse)
                        ti = float(pt.tangentImpulse)
                        total_impulse = math.sqrt(ni * ni + ti * ti)
                        self._contact_peak_normal_force = max(
                            self._contact_peak_normal_force, total_impulse)
                        contact_y = float(manifold.points[0].y) if manifold.points else 0.0
                        self._contact_heights.append(contact_y)
            except Exception as exc:
                self._record_diagnostic_error("contact", exc)
        self._track_temporal_events(agent_body, step_count, object_body)
        if step_count == 0:
            self._motor_snapshot_summary = motor_data
            self._wheel_snapshot_summary = wheel_data
        elif step_count >= getattr(self, '_last_snapshot_step', -1) + 500:
            self._motor_snapshot_summary = motor_data
            self._wheel_snapshot_summary = wheel_data
            self._last_snapshot_step = step_count
    def _record_diagnostic_error(self, source, exc):
        self._diagnostic_error_count += 1
        self._last_diagnostic_error = f"{source}: {type(exc).__name__}: {exc}"
    def _compute_force_budget(self):
        budget = {}
        try:
            env = self.environment
            object_body = getattr(env, '_object_to_push', None)
            if not object_body:
                return budget
            obj_mass = float(object_body.mass)
            g = env._world.gravity
            g_mag = math.sqrt(float(g.x) ** 2 + float(g.y) ** 2)
            obj_friction = 0.8
            for fix in object_body.fixtures:
                obj_friction = float(fix.friction)
                break
            F_required = obj_mass * g_mag * obj_friction
            if hasattr(object_body, 'linearDamping'):
                damping = float(object_body.linearDamping)
                F_required += damping * 1.0
            budget['required_push_force'] = F_required
            structure_mass = env.get_structure_mass()
            ground_friction = float(getattr(env, '_ground_friction', 1.2))
            normal_force = structure_mass * g_mag
            F_traction = normal_force * ground_friction
            F_motor_total = 0.0
            try:
                from Box2D import b2RevoluteJoint
            except ImportError:
                try:
                    from Box2D.b2 import revoluteJoint as b2RevoluteJoint
                except ImportError:
                    b2RevoluteJoint = None
            for joint in env._joints:
                if b2RevoluteJoint is not None and isinstance(joint, b2RevoluteJoint) and getattr(joint, 'enableMotor', False):
                    max_torque = float(joint.maxMotorTorque)
                    wheel_radius = 0.3
                    F_motor_total += max_torque / max(wheel_radius, 0.05)
            F_available = min(F_traction, F_motor_total) if F_motor_total > 0 else F_traction
            budget['available_traction'] = F_available
            budget['traction_limit'] = F_traction
            budget['motor_force_limit'] = F_motor_total
            if F_available < F_required and F_required > 0:
                ratio = F_available / F_required * 100.0
                if F_traction < F_motor_total or F_motor_total == 0:
                    budget['bottleneck'] = 'ground_traction'
                else:
                    budget['bottleneck'] = 'motor_torque'
                budget['force_ratio_pct'] = ratio
            else:
                budget['bottleneck'] = 'none'
                budget['force_ratio_pct'] = 100.0 if F_required == 0 else min(F_available / F_required * 100.0, 999.0)
            budget['effective_ground_friction'] = ground_friction
            budget['object_friction'] = obj_friction
            budget['structure_normal_force'] = normal_force
        except Exception as exc:
            self._record_diagnostic_error("temporal_events", exc)
        return budget
    def _track_temporal_events(self, agent_body, step_count, object_body):
        try:
            events = self._temporal_events
            tags = self._event_tags_seen
            env = self.environment
            ground_y = self.ground_y
            fc = self._first_contact_step
            if fc is not None and 'first_contact' not in tags:
                tags.add('first_contact')
                sep = self._pusher_object_separation_at_contact
                events.append({
                    'event': 'first_contact',
                    'step': int(fc),
                    'detail': f'Pusher made contact with object at step {int(fc)}',
                    'separation_at_contact': round(sep, 3) if sep is not None else None,
                })
            fm = self._first_object_motion_step
            if fm is not None and 'first_motion' not in tags:
                tags.add('first_motion')
                delay = int(fm) - int(fc) if fc is not None else None
                detail = f'Object first moved at step {int(fm)}'
                if delay is not None and delay > 0:
                    detail += f' (contact-to-motion delay: {delay} steps)'
                events.append({
                    'event': 'first_object_motion',
                    'step': int(fm),
                    'detail': detail,
                    'contact_to_motion_delay': delay,
                })
            fs = getattr(self, '_first_wheel_suspension_step', None)
            if fs is not None and 'first_suspension' not in tags:
                tags.add('first_suspension')
                depth = self._max_wheel_suspension_depth
                events.append({
                    'event': 'first_wheel_suspension',
                    'step': int(fs),
                    'detail': f'Wheels lost ground contact at step {int(fs)} (suspension depth {depth:.3f}m)',
                    'suspension_depth': round(depth, 3),
                })
            fsp = getattr(self, '_first_wheel_spinning_step', None)
            if fsp is not None and 'first_spinning' not in tags:
                tags.add('first_spinning')
                events.append({
                    'event': 'first_wheel_spinning',
                    'step': int(fsp),
                    'detail': f'Wheels began spinning without traction at step {int(fsp)}',
                })
            if agent_body and object_body:
                px = float(agent_body.position.x) if agent_body else 0.0
                ox = float(object_body.position.x) if object_body else 0.0
                gap = px - ox
                if gap < -0.5 and 'pusher_behind' not in tags:
                    tags.add('pusher_behind')
                    events.append({
                        'event': 'pusher_behind_object',
                        'step': int(step_count),
                        'detail': f'Pusher is {abs(gap):.2f}m behind object (no contact possible)',
                        'gap': round(gap, 3),
                    })
                obj_vx = float(object_body.linearVelocity.x) if object_body else 0.0
                if obj_vx < 0.001 and step_count > 1000 and 'object_stationary' not in tags:
                    tags.add('object_stationary')
                    events.append({
                        'event': 'object_stationary',
                        'step': int(step_count),
                        'detail': f'Object velocity near zero ({obj_vx:.4f} m/s) after {int(step_count)} steps',
                    })
            pv = getattr(self, '_peak_object_velocity', 0.0)
            if pv > 0.1 and 'peak_velocity' not in tags:
                tags.add('peak_velocity')
                events.append({
                    'event': 'peak_object_velocity',
                    'velocity': round(pv, 3),
                    'detail': f'Peak object x-velocity: {pv:.3f} m/s',
                })
        except Exception:
            pass
    def _build_constraint_profile(self, base_metrics):
        profile = []
        try:
            mass = float(base_metrics.get('structure_mass', 0))
            max_mass = float(self.MAX_STRUCTURE_MASS)
            mass_margin = max_mass - mass
            profile.append({
                'constraint': 'Structure mass',
                'status': 'PASS' if mass_margin >= 0 else 'FAIL',
                'current': f'{mass:.2f} kg',
                'limit': f'{max_mass:.2f} kg',
                'margin': f'{mass_margin:+.2f} kg',
                'utilization_pct': round(mass / max_mass * 100.0, 1) if max_mass > 0 else 0.0,
                'phase': 'build-time',
            })
            bz_x_min = float(self.BUILD_ZONE_X_MIN)
            bz_x_max = float(self.BUILD_ZONE_X_MAX)
            bz_y_min = float(self.BUILD_ZONE_Y_MIN)
            bz_y_max = float(self.BUILD_ZONE_Y_MAX)
            bz_violated = False
            for body in self.environment._bodies:
                x, y = float(body.position.x), float(body.position.y)
                if not (bz_x_min <= x <= bz_x_max and bz_y_min <= y <= bz_y_max):
                    bz_violated = True
                    break
            profile.append({
                'constraint': 'Build zone (all components)',
                'status': 'PASS' if not bz_violated else 'FAIL',
                'current': '—',
                'limit': f'x:[{bz_x_min:.1f},{bz_x_max:.1f}], y:[{bz_y_min:.1f},{bz_y_max:.1f}]',
                'margin': '—' if not bz_violated else 'component outside zone',
                'phase': 'build-time',
            })
            oy = float(base_metrics.get('object_y', 0))
            payload_limit = 0.5
            payload_margin = oy - payload_limit
            profile.append({
                'constraint': 'Payload support (y > 0.5m)',
                'status': 'PASS' if payload_margin >= 0 else 'FAIL',
                'current': f'{oy:.2f} m',
                'limit': '0.50 m',
                'margin': f'{payload_margin:+.2f} m',
                'phase': 'runtime',
            })
            tilt = float(base_metrics.get('max_pusher_tilt', 0))
            tilt_limit = math.pi / 6
            tilt_margin = tilt_limit - tilt
            profile.append({
                'constraint': 'Chassis tilt (< π/6 rad)',
                'status': 'PASS' if tilt_margin >= 0 else 'FAIL',
                'current': f'{tilt:.3f} rad ({math.degrees(tilt):.1f}°)',
                'limit': f'{tilt_limit:.3f} rad (30.0°)',
                'margin': f'{tilt_margin:+.3f} rad',
                'phase': 'runtime',
            })
            ox = float(base_metrics.get('object_x', 0))
            tx = float(base_metrics.get('target_object_x', 0))
            dist_margin = ox - tx
            profile.append({
                'constraint': 'Target distance (x ≥ target)',
                'status': 'PASS' if dist_margin >= 0 else 'FAIL',
                'current': f'{ox:.2f} m',
                'limit': f'{tx:.2f} m',
                'margin': f'{dist_margin:+.2f} m',
                'phase': 'runtime',
            })
            steps = int(base_metrics.get('step_count', 0))
            min_steps = int(self.min_simulation_steps)
            time_margin = steps - min_steps
            profile.append({
                'constraint': 'Simulation time (≥ min steps)',
                'status': 'PASS' if time_margin >= 0 else 'FAIL',
                'current': f'{steps} steps',
                'limit': f'{min_steps} steps',
                'margin': f'{time_margin:+d} steps',
                'phase': 'runtime',
            })
            stuck = getattr(self, 'not_pushed_counter', 0)
            stuck_limit = 200
            stuck_margin = stuck_limit - stuck
            profile.append({
                'constraint': 'Effective push (stuck < 200 steps)',
                'status': 'PASS' if stuck <= stuck_limit else 'FAIL',
                'current': f'{stuck} consecutive stuck steps',
                'limit': f'{stuck_limit} steps',
                'margin': f'{stuck_margin:+d} steps',
                'phase': 'runtime',
            })
        except Exception as exc:
            self._record_diagnostic_error("constraint_profile", exc)
        return profile
    def evaluate(self, agent_body, step_count, max_steps):
        if not self.environment:
            return (False, 0.0, {"error": "Environment not available"})
        failed = False
        failure_reason = None
        if not self.design_constraints_checked and step_count == 0:
            violations = self._check_design_constraints()
            if violations:
                failed = True
                failure_reason = "Design constraint violated: " + "; ".join(violations)
            self.design_constraints_checked = True
        object_pos = self.environment.get_object_position()
        if object_pos is None:
            return (False, 0.0, {"error": "Object to push not found"})
        current_object_x, current_object_y = object_pos
        if self.max_x_reached is None:
            self.max_x_reached = current_object_x
        else:
            self.max_x_reached = max(self.max_x_reached, current_object_x)
        pusher = agent_body
        if pusher is None and self.environment._bodies:
            pusher = self.environment._bodies[0]
        pusher_x, pusher_y, pusher_angle = 0.0, 0.0, 0.0
        if pusher:
            pusher_x = pusher.position.x
            pusher_y = pusher.position.y
            pusher_angle = pusher.angle
            while pusher_angle > math.pi: pusher_angle -= 2 * math.pi
            while pusher_angle < -math.pi: pusher_angle += 2 * math.pi
            self.max_pusher_tilt = max(self.max_pusher_tilt, abs(pusher_angle))
            for val in (pusher_x, pusher_y, pusher_angle,
                        pusher.linearVelocity.x, pusher.linearVelocity.y):
                if not math.isfinite(val):
                    self._numerical_instability_flag = True
        object_vx = self.environment._object_to_push.linearVelocity.x if self.environment._object_to_push else 0.0
        object_vy = self.environment._object_to_push.linearVelocity.y if self.environment._object_to_push else 0.0
        for val in (object_vx, object_vy):
            if not math.isfinite(val):
                self._numerical_instability_flag = True
        self._peak_object_velocity = max(self._peak_object_velocity, abs(object_vx))
        if pusher and self._first_contact_step is None:
            object_front_x = current_object_x + 0.5
            if pusher_x + 0.3 >= object_front_x - 0.1:
                self._first_contact_step = step_count
                self._pusher_object_separation_at_contact = pusher_x - current_object_x
        if self._first_object_motion_step is None and abs(object_vx) > 0.05:
            self._first_object_motion_step = step_count
        if not failed:
            if current_object_y < 0.5:
                failed = True
                failure_reason = f"Payload support violated: object center y={current_object_y:.2f}m < 0.5m (ground surface at y=1.0m)"
            elif abs(pusher_angle) > math.pi / 6:
                failed = True
                failure_reason = f"Pusher tipped over: tilt angle {abs(pusher_angle):.3f} rad exceeds limit \u00b1\u03c0/6 (\u00b130\u00b0)"
            else:
                wheel_failure = self._check_wheel_states()
                if wheel_failure:
                    pass
        distance_pushed = current_object_x - self.initial_object_x
        success = distance_pushed >= self.target_distance and step_count >= self.min_simulation_steps
        wheel_state, wheel_detail = self._check_wheel_states()
        if wheel_detail:
            for wd in wheel_detail:
                _, is_susp, susp_depth, tang_speed = wd
                self._max_wheel_tangential_speed = max(
                    self._max_wheel_tangential_speed, tang_speed)
                if is_susp:
                    self._max_wheel_suspension_depth = max(
                        self._max_wheel_suspension_depth, susp_depth)
                    if self._first_wheel_suspension_step is None:
                        self._first_wheel_suspension_step = step_count
                    self._suspension_step_count += 1
            if wheel_state == "wheels suspended":
                if self._overall_wheel_state != "wheels suspended":
                    self._overall_wheel_state = "wheels suspended"
            elif wheel_state == "wheel spinning":
                if self._overall_wheel_state is None:
                    self._overall_wheel_state = "wheel spinning"
        spinning_count_local = 0
        for w in (wheel_detail or []):
            _, _, _, tang_spd = w
            w_body = None
            for body in self.environment._bodies:
                from Box2D.b2 import circleShape
                for fixture in body.fixtures:
                    if isinstance(fixture.shape, circleShape):
                        w_body = body
                        break
            if w_body:
                v_linear = w_body.linearVelocity.length
                if tang_spd > 2.0 and v_linear < 0.5:
                    spinning_count_local += 1
        if spinning_count_local >= 1:
            if self._first_wheel_spinning_step is None:
                self._first_wheel_spinning_step = step_count
            self._spinning_step_count += 1
            if self._overall_wheel_state is None:
                self._overall_wheel_state = "wheel spinning"
        if pusher and self.environment._object_to_push:
            pusher_vx = pusher.linearVelocity.x
            object_vx = self.environment._object_to_push.linearVelocity.x
            if pusher_vx > 0.5 and object_vx < 0.05:
                if not hasattr(self, 'not_pushed_counter'): self.not_pushed_counter = 0
                self.not_pushed_counter += 1
            else:
                self.not_pushed_counter = 0
            if self.not_pushed_counter > 200 and not failed:
                failed = True
                failure_reason = failure_reason or "not pushed effectively"
        self._collect_diagnostics(agent_body, step_count)
        if step_count >= max_steps - 1:
            self._diagnostics_collected = True
        is_end = (step_count >= max_steps - 1)
        done = failed or success or is_end
        progress = min(max(0, distance_pushed) / self.target_distance, 1.0) if self.target_distance > 0 else 0.0
        if success and not failed:
            score = 100.0
        elif failed:
            score = 0.0
        else:
            score = progress * 70.0
            if step_count > 0:
                score += (min(step_count, self.min_simulation_steps) / self.min_simulation_steps) * 30.0
        metrics = {
            'object_x': current_object_x,
            'object_y': current_object_y,
            'pusher_x': pusher_x,
            'pusher_y': pusher_y,
            'pusher_angle': pusher_angle,
            'pusher_velocity_x': pusher.linearVelocity.x if pusher else 0.0,
            'pusher_velocity_y': pusher.linearVelocity.y if pusher else 0.0,
            'object_velocity_x': object_vx,
            'object_velocity_y': object_vy,
            'distance_pushed': distance_pushed,
            'max_distance_pushed': self.max_x_reached - self.initial_object_x,
            'max_pusher_tilt': self.max_pusher_tilt,
            'pusher_tipped': abs(pusher_angle) > math.pi / 6,
            'target_object_x': self.initial_object_x + self.target_distance,
            'progress': progress * 100.0,
            'success': success and not failed,
            'failed': failed,
            'failure_reason': failure_reason or (wheel_state if step_count > 100 else None) or ("not pushed effectively" if getattr(self, 'not_pushed_counter', 0) > 200 else None),
            'step_count': step_count,
            'min_simulation_steps_required': self.min_simulation_steps,
            'structure_mass': self.environment.get_structure_mass(),
            'max_structure_mass': self.MAX_STRUCTURE_MASS,
            'wheel_state': self._overall_wheel_state,
            'first_wheel_suspension_step': self._first_wheel_suspension_step,
            'first_wheel_spinning_step': self._first_wheel_spinning_step,
            'suspension_step_count': self._suspension_step_count,
            'spinning_step_count': self._spinning_step_count,
            'max_wheel_suspension_depth': self._max_wheel_suspension_depth,
            'max_wheel_tangential_speed': self._max_wheel_tangential_speed,
            'first_contact_step': self._first_contact_step,
            'first_object_motion_step': self._first_object_motion_step,
            'pusher_object_separation_at_contact': self._pusher_object_separation_at_contact,
            'peak_object_velocity': self._peak_object_velocity,
            'ground_y': self.ground_y,
            'build_zone_x_min': self.BUILD_ZONE_X_MIN,
            'build_zone_x_max': self.BUILD_ZONE_X_MAX,
            'build_zone_y_min': self.BUILD_ZONE_Y_MIN,
            'build_zone_y_max': self.BUILD_ZONE_Y_MAX,
            'pusher_object_gap': pusher_x - current_object_x if pusher else None,
            'numerical_instability': self._numerical_instability_flag,
            'motor_actuation': {
                'ever_active': self._motor_ever_active,
                'peak_saturation': self._motor_peak_saturation,
                'cmd_speeds': self._motor_cmd_speeds,
                'actual_speeds': self._motor_actual_speeds,
                'torques_used': self._motor_torques_used,
            },
            'wheel_contact_audit': {
                'ever_contacted': self._wheel_ever_contacted,
                'min_gap_to_ground': self._wheel_min_gap if math.isfinite(self._wheel_min_gap) else None,
                'radii': self._wheel_radii,
                'positions': self._wheel_positions,
            },
            'energy_tracking': {
                'peak_chassis_ke': self._peak_chassis_ke,
                'peak_object_ke': self._peak_object_ke,
                'cumulative_motor_energy_est': self._cumulative_motor_energy_est,
            },
            'contact_forces': {
                'ever_made': self._contact_ever_made,
                'peak_normal_impulse': self._contact_peak_normal_force,
                'contact_heights': self._contact_heights,
            },
            'temporal_events': self._temporal_events,
            'constraint_profile': self._build_constraint_profile({
                'object_x': current_object_x,
                'object_y': current_object_y,
                'target_object_x': self.initial_object_x + self.target_distance,
                'structure_mass': self.environment.get_structure_mass(),
                'max_pusher_tilt': self.max_pusher_tilt,
                'step_count': step_count,
                'wheel_state': self._overall_wheel_state,
            }),
            'peak_body_velocity': self._peak_body_velocity,
            'diagnostic_error_count': self._diagnostic_error_count,
            'last_diagnostic_error': self._last_diagnostic_error,
        }
        return done, score, metrics
    def _check_design_constraints(self):
        violations = []
        mass = self.environment.get_structure_mass()
        if mass > self.MAX_STRUCTURE_MASS:
            violations.append(f"Structure mass {mass:.2f}kg exceeds limit {self.MAX_STRUCTURE_MASS}kg")
        for body in self.environment._bodies:
            x, y = body.position.x, body.position.y
            if not (self.BUILD_ZONE_X_MIN <= x <= self.BUILD_ZONE_X_MAX and
                    self.BUILD_ZONE_Y_MIN <= y <= self.BUILD_ZONE_Y_MAX):
                violations.append(f"Component at ({x:.2f}, {y:.2f}) is outside build zone x:[{self.BUILD_ZONE_X_MIN}, {self.BUILD_ZONE_X_MAX}], y:[{self.BUILD_ZONE_Y_MIN}, {self.BUILD_ZONE_Y_MAX}]")
        return violations
    def _check_wheel_states(self):
        from Box2D.b2 import circleShape
        wheels = []
        for body in self.environment._bodies:
            for fixture in body.fixtures:
                if isinstance(fixture.shape, circleShape):
                    wheels.append(body)
                    break
        if not wheels:
            return None, []
        detail = []
        suspension_threshold = self.ground_y + 0.15
        for idx, w in enumerate(wheels):
            radius = w.fixtures[0].shape.radius
            wheel_bottom_y = w.position.y - radius
            suspension_depth = wheel_bottom_y - suspension_threshold
            tang_speed = abs(w.angularVelocity) * radius
            is_susp = wheel_bottom_y > suspension_threshold
            detail.append((idx, is_susp, suspension_depth, tang_speed))
        all_suspended = all(d[1] for d in detail)
        if all_suspended:
            return "wheels suspended", detail
        spinning_count = 0
        for w in wheels:
            v = w.linearVelocity.length
            omega = abs(w.angularVelocity)
            radius = w.fixtures[0].shape.radius
            if omega * radius > 2.0 and v < 0.5:
                spinning_count += 1
        if spinning_count >= len(wheels) / 2:
            return "wheel spinning", detail
        return None, detail
    def compute_score_with_penalty(self, score: float, metrics: dict) -> float:
        if score > 0:
            return score
        constraint_info = self.get_constraint_info()
        penalty = compute_constraint_penalty("K_04", score, metrics, constraint_info)
        return penalty
    def get_constraint_info(self):
        return {
            'target_distance': self.target_distance,
            'ground_y': self.ground_y,
            'min_simulation_time': self.min_simulation_time,
            'max_structure_mass': self.MAX_STRUCTURE_MASS,
            'build_zone_x_min': self.BUILD_ZONE_X_MIN,
            'build_zone_x_max': self.BUILD_ZONE_X_MAX,
            'build_zone_y_min': self.BUILD_ZONE_Y_MIN,
            'build_zone_y_max': self.BUILD_ZONE_Y_MAX,
        }
    def get_task_description(self):
        return {
            'task': 'K-04: The Pusher',
            'success_criteria': {
                'distance': f'Push object {self.target_distance}m',
                'time': f'Push for {self.min_simulation_time}s'
            }
        }
