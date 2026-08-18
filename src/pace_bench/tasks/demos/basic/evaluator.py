import math

from pace_bench.core.primitives import compute_constraint_penalty

class Evaluator:
    def __init__(self, terrain_bounds, environment=None):
        self.terrain_bounds = terrain_bounds
        self.environment = environment
        self.start_x = 5.0
        self.target_x = 30.0
        self.min_distance = 0.0
        self.max_distance = 0.0
        self.high_angular_velocity_count = 0
        self.high_altitude_count = 0
        self.MAX_ANGULAR_VELOCITY = 2.0
        self.MAX_REASONABLE_ALTITUDE = 8.0
        self.STABILITY_CHECK_START_STEP = 200
        self.UNSTABLE_THRESHOLD = 5
        self.MAX_AIRBORNE_ROTATION = math.pi
        self.AIRBORNE_THRESHOLD = 0.5
        self._rotation_tracking_initialized = False
        if not environment:
            raise ValueError("Evaluator requires environment instance to read constraint constants")
        env_class = type(environment)
        try:
            self.MAX_CHASSIS_HEIGHT = env_class.MAX_CHASSIS_HEIGHT
            self.MIN_WHEEL_RADIUS = env_class.MIN_WHEEL_RADIUS
            self.MAX_WHEEL_RADIUS = env_class.MAX_WHEEL_RADIUS
            self.MAX_WHEELS = env_class.MAX_WHEELS
            self.GROUND_TOP = env_class.GROUND_TOP
            self.MAX_CONNECTION_DISTANCE = env_class.MAX_CONNECTION_DISTANCE
            self.MAX_MOTOR_SPEED = env_class.MAX_MOTOR_SPEED
            self.MAX_MOTOR_TORQUE = env_class.MAX_MOTOR_TORQUE
        except AttributeError as e:
            raise AttributeError(f"Environment class {env_class.__name__} missing required constant: {e}")
        self.design_constraints_checked = False
    def evaluate(self, agent_body, step_count, max_steps):
        current_x = agent_body.position.x
        current_y = agent_body.position.y
        distance_traveled = current_x - self.start_x
        if distance_traveled > self.max_distance:
            self.max_distance = distance_traveled
        velocity_x = agent_body.linearVelocity.x
        velocity_y = agent_body.linearVelocity.y
        velocity = math.sqrt(velocity_x**2 + velocity_y**2)
        angular_velocity = agent_body.angularVelocity
        angle = agent_body.angle
        success = current_x >= self.target_x
        failed = False
        failure_reason = None
        if not self.design_constraints_checked and self.environment and step_count == 0:
            constraint_violations = self._check_design_constraints(agent_body)
            if constraint_violations:
                failed = True
                failure_reason = "Design constraint violated: " + "; ".join(constraint_violations)
            self.design_constraints_checked = True
        if current_y < -10:
            failed = True
            failure_reason = "Fell off map"
        if current_x < self.start_x - 5:
            failed = True
            failure_reason = "Moved backward too much"
        if step_count > self.STABILITY_CHECK_START_STEP:
            if abs(angular_velocity) > self.MAX_ANGULAR_VELOCITY:
                self.high_angular_velocity_count += 1
                if self.high_angular_velocity_count >= self.UNSTABLE_THRESHOLD:
                    failed = True
                    failure_reason = f"Vehicle is unstable: excessive rotation (angular velocity {angular_velocity:.2f} rad/s for {self.high_angular_velocity_count * 100} steps)"
            else:
                self.high_angular_velocity_count = 0
        if step_count > self.STABILITY_CHECK_START_STEP:
            if current_y > self.MAX_REASONABLE_ALTITUDE:
                self.high_altitude_count += 1
                if self.high_altitude_count >= self.UNSTABLE_THRESHOLD:
                    failed = True
                    failure_reason = f"Vehicle is flying: excessive altitude (y={current_y:.2f}m for {self.high_altitude_count * 100} steps)"
            else:
                self.high_altitude_count = 0
        is_airborne = current_y > (self.GROUND_TOP + self.AIRBORNE_THRESHOLD)
        if not self._rotation_tracking_initialized and self.environment:
            if hasattr(self.environment, 'set_tracked_body'):
                self.environment.set_tracked_body(agent_body)
                self._rotation_tracking_initialized = True
        airborne_rotation_accumulated = 0.0
        if self.environment and hasattr(self.environment, 'get_airborne_rotation_status'):
            rotation_status = self.environment.get_airborne_rotation_status()
            airborne_rotation_accumulated = rotation_status['accumulated']
            if rotation_status['exceeded']:
                failed = True
                failure_reason = f"Vehicle rotated {airborne_rotation_accumulated * 180 / math.pi:.1f}° while airborne (exceeds 180° limit)"
        if success and not failed:
            score = 100.0
        elif failed:
            score = 0.0
        else:
            progress = min(distance_traveled / (self.target_x - self.start_x), 1.0)
            score = progress * 80.0
        metrics = {
            'distance_traveled': distance_traveled,
            'current_x': current_x,
            'current_y': current_y,
            'target_x': self.target_x,
            'progress': min(distance_traveled / (self.target_x - self.start_x), 1.0) * 100,
            'success': success and not failed,
            'failed': failed,
            'failure_reason': failure_reason,
            'step_count': step_count,
            'max_distance': self.max_distance,
            'velocity': velocity,
            'velocity_x': velocity_x,
            'velocity_y': velocity_y,
            'angular_velocity': angular_velocity,
            'angle': angle,
            'high_angular_velocity_count': self.high_angular_velocity_count,
            'high_altitude_count': self.high_altitude_count,
            'is_airborne': is_airborne,
            'airborne_rotation_accumulated': airborne_rotation_accumulated
        }
        return success or failed, score, metrics
    def _check_design_constraints(self, agent_body):
        violations = []
        if not self.environment:
            raise ValueError("Cannot check design constraints: environment not provided")
        if not hasattr(self.environment, '_bodies') or not hasattr(self.environment, '_joints'):
            raise AttributeError(f"Environment {type(self.environment).__name__} missing required attributes '_bodies' or '_joints'")
        from Box2D.b2 import dynamicBody, circleShape, polygonShape
        bodies = self.environment._bodies
        joints = self.environment._joints
        chassis = agent_body
        wheels = [b for b in bodies if b != chassis and b.type == dynamicBody]
        if len(wheels) > self.MAX_WHEELS:
            violations.append(f"vehicle has {len(wheels)} wheels, but maximum {self.MAX_WHEELS} wheels are allowed")
        if chassis:
            chassis_height = None
            for fixture in chassis.fixtures:
                shape = fixture.shape
                if isinstance(shape, polygonShape):
                    if hasattr(shape, 'box'):
                        box = shape.box
                        if isinstance(box, tuple) and len(box) >= 2:
                            chassis_height = box[1] * 2
                            break
            if chassis_height and chassis_height > self.MAX_CHASSIS_HEIGHT:
                violations.append(f"chassis height {chassis_height:.2f}m exceeds maximum {self.MAX_CHASSIS_HEIGHT}m")
        for wheel in wheels:
            wheel_radius = None
            for fixture in wheel.fixtures:
                shape = fixture.shape
                if isinstance(shape, circleShape):
                    wheel_radius = shape.radius
                    break
            if wheel_radius:
                if wheel_radius < self.MIN_WHEEL_RADIUS:
                    violations.append(f"wheel radius {wheel_radius:.2f}m is below minimum {self.MIN_WHEEL_RADIUS}m")
                elif wheel_radius > self.MAX_WHEEL_RADIUS:
                    violations.append(f"wheel radius {wheel_radius:.2f}m exceeds maximum {self.MAX_WHEEL_RADIUS}m")
                wheel_bottom = wheel.position.y - wheel_radius
                if wheel_bottom > self.GROUND_TOP + 0.2:
                    violations.append(f"wheel at ({wheel.position.x:.2f}, {wheel.position.y:.2f}) does not contact ground initially (bottom y={wheel_bottom:.2f}m, ground top y={self.GROUND_TOP}m)")
        for joint in joints:
            if hasattr(joint, 'anchor') and hasattr(joint, 'bodyA') and hasattr(joint, 'bodyB'):
                anchor = joint.anchor
                body_a = joint.bodyA
                body_b = joint.bodyB
                if body_a and body_b:
                    if isinstance(anchor, tuple) and len(anchor) >= 2:
                        anchor_x, anchor_y = anchor[0], anchor[1]
                    elif hasattr(anchor, 'x') and hasattr(anchor, 'y'):
                        anchor_x, anchor_y = anchor.x, anchor.y
                    else:
                        continue
                    pos_a = body_a.position
                    pos_b = body_b.position
                    distance_a = math.sqrt((anchor_x - pos_a.x)**2 + (anchor_y - pos_a.y)**2)
                    distance_b = math.sqrt((anchor_x - pos_b.x)**2 + (anchor_y - pos_b.y)**2)
                    if distance_a > self.MAX_CONNECTION_DISTANCE:
                        violations.append(f"connection point too far from body_a: {distance_a:.2f}m (max {self.MAX_CONNECTION_DISTANCE}m)")
                    if distance_b > self.MAX_CONNECTION_DISTANCE:
                        violations.append(f"connection point too far from body_b: {distance_b:.2f}m (max {self.MAX_CONNECTION_DISTANCE}m)")
        for joint in joints:
            if hasattr(joint, 'motorEnabled') and joint.motorEnabled:
                if hasattr(joint, 'motorSpeed'):
                    if abs(joint.motorSpeed) > self.MAX_MOTOR_SPEED:
                        violations.append(f"motor speed {joint.motorSpeed:.2f} rad/s exceeds maximum {self.MAX_MOTOR_SPEED} rad/s")
                if hasattr(joint, 'maxMotorTorque'):
                    if joint.maxMotorTorque > self.MAX_MOTOR_TORQUE:
                        violations.append(f"motor torque {joint.maxMotorTorque:.2f} N·m exceeds maximum {self.MAX_MOTOR_TORQUE} N·m")
        return violations
    def compute_score_with_penalty(self, score: float, metrics: dict) -> float:
        if score > 0:
            return score
        constraint_info = self.get_constraint_info()
        penalty = compute_constraint_penalty("basic", score, metrics, constraint_info)
        return penalty
    def get_constraint_info(self):
        return {
            'start_x': self.start_x,
            'target_x': self.target_x,
            'max_angular_velocity': self.MAX_ANGULAR_VELOCITY,
            'max_reasonable_altitude': self.MAX_REASONABLE_ALTITUDE,
            'stability_check_start_step': self.STABILITY_CHECK_START_STEP,
            'unstable_threshold': self.UNSTABLE_THRESHOLD,
            'max_airborne_rotation': self.MAX_AIRBORNE_ROTATION,
            'airborne_threshold': self.AIRBORNE_THRESHOLD,
            'max_chassis_height': self.MAX_CHASSIS_HEIGHT,
            'min_wheel_radius': self.MIN_WHEEL_RADIUS,
            'max_wheel_radius': self.MAX_WHEEL_RADIUS,
            'max_wheels': self.MAX_WHEELS,
            'ground_top': self.GROUND_TOP,
            'max_connection_distance': self.MAX_CONNECTION_DISTANCE,
            'max_motor_speed': self.MAX_MOTOR_SPEED,
            'max_motor_torque': self.MAX_MOTOR_TORQUE,
        }
    def get_task_description(self):
        return {
            'task': 'Design a vehicle that can climb slopes',
            'description': 'Agent needs to design a mechanical structure (vehicle) that can move on terrain and pass obstacles',
            'start_position': self.start_x,
            'target_position': self.target_x,
            'terrain': {
                'ground_length': self.terrain_bounds['end'],
                'obstacles': self.terrain_bounds['obstacles']
            },
            'success_criteria': {
                'primary': f'Agent chassis must reach position x={self.target_x}m',
                'secondary': 'Agent cannot fall off map (y < -10)',
                'tertiary': 'Agent cannot move backward too much (x < start_x - 5)',
                'stability': f'Agent must move stably: angular velocity < {self.MAX_ANGULAR_VELOCITY} rad/s, altitude < {self.MAX_REASONABLE_ALTITUDE}m'
            },
            'evaluation': {
                'score_range': '0-100',
                'success_score': 100,
                'partial_score': 'Based on travel distance, max 80 points',
                'failure_score': 0
            }
        }
