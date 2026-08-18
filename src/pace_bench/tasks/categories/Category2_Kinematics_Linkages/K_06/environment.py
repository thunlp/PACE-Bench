import Box2D

from Box2D.b2 import (world, polygonShape, circleShape, staticBody, dynamicBody, revoluteJoint, weldJoint)

import math

import random

class Sandbox:
    def __init__(self, *, terrain_config=None, physics_config=None):
        terrain_config = terrain_config or {}
        physics_config = physics_config or {}
        self._terrain_config = dict(terrain_config)
        self._physics_config = dict(physics_config)
        gravity = tuple(physics_config.get("gravity", (0, -10)))
        self._default_linear_damping = float(physics_config.get("linear_damping", 0.0))
        self._default_angular_damping = float(physics_config.get("angular_damping", 0.0))
        self._world = world(gravity=gravity, doSleep=True)
        self._bodies = []
        self._joints = []
        self._terrain_bodies = {}
        self._wiper_bodies = {}
        self._wiper_joints = []
        self._wiper_motor_joint = None
        self._particles = []
        self._forensic_min_body_y = float('inf')
        self._forensic_max_body_y = float('-inf')
        self._forensic_min_body_x = float('inf')
        self._forensic_max_body_x = float('-inf')
        self._forensic_body_count = 0
        self._forensic_wiper_y_at_eval = None
        self._forensic_particles_removed_history = []
        self._forensic_last_particle_count = None
        self._forensic_step_count = 0
        self._forensic_nan_detected = False
        self._forensic_torque_requested = None
        _motor_cap = terrain_config.get("max_motor_torque")
        if _motor_cap is not None:
            _motor_cap = float(_motor_cap)
        self._forensic_torque_cap = _motor_cap
        self._forensic_max_body_reach_y = float('-inf')
        self._forensic_min_body_reach_y = float('inf')
        self._forensic_joint_angle_history = []
        self._forensic_motor_energy = 0.0
        self._forensic_peak_body_velocity = 0.0
        self._forensic_body_velocity_warnings = []
        self._forensic_violation_info = None
        self._forensic_last_step = -1
        self.world = self._world
        self.bodies = self._bodies
        self.joints = self._joints
        self._create_terrain(terrain_config)
        self.BUILD_ZONE_X_MIN = 0.0
        self.BUILD_ZONE_X_MAX = 12.0
        self.BUILD_ZONE_Y_MIN = 2.0
        self.BUILD_ZONE_Y_MAX = 10.0
        self.MAX_STRUCTURE_MASS = float(terrain_config.get("max_structure_mass", 15.0))
        self._max_motor_torque_cap = terrain_config.get("max_motor_torque")
        if self._max_motor_torque_cap is not None:
            self._max_motor_torque_cap = float(self._max_motor_torque_cap)
        self._create_particles(terrain_config)
        self._create_initial_wiper_template(terrain_config)
    _CATEGORY_GLASS = 0x0001
    _CATEGORY_WIPER = 0x0002
    _CATEGORY_PARTICLE = 0x0004
    def _create_terrain(self, terrain_config: dict):
        glass_friction = float(terrain_config.get("glass_friction", 0.25))
        glass_length = 12.0
        glass_height = 0.1
        glass_y = 2.0
        fd = Box2D.b2FixtureDef(
            shape=polygonShape(box=(glass_length / 2, glass_height / 2)),
            friction=glass_friction,
        )
        if terrain_config.get("wiper_ignore_glass_collision", True):
            fd.filter.categoryBits = self._CATEGORY_GLASS
            fd.filter.maskBits = self._CATEGORY_GLASS | self._CATEGORY_PARTICLE
        glass = self._world.CreateStaticBody(
            position=(glass_length / 2, glass_y - glass_height / 2),
            fixtures=fd,
        )
        self._terrain_bodies["glass"] = glass
        self._glass_y = glass_y
        self._glass_length = glass_length
        self._glass_friction = glass_friction
    def _create_particles(self, terrain_config: dict):
        particle_config = terrain_config.get("particles", {})
        num_particles = int(particle_config.get("count", 45))
        particle_friction = float(particle_config.get("friction", 0.35))
        particle_mass = float(particle_config.get("mass", 0.15))
        particle_radius = float(particle_config.get("radius", 0.08))
        particle_seed = int(particle_config.get("seed", 42))
        self._particle_friction = particle_friction
        self._particle_mass = particle_mass
        self._particle_radius = particle_radius
        self._particle_count = num_particles
        rng = random.Random(particle_seed)
        glass_start_x = 1.0
        glass_end_x = 11.0
        use_filter = terrain_config.get("wiper_ignore_glass_collision", True)
        for i in range(num_particles):
            x = rng.uniform(glass_start_x, glass_end_x)
            y = self._glass_y + particle_radius
            density = particle_mass / (math.pi * particle_radius * particle_radius)
            pfd = Box2D.b2FixtureDef(
                shape=circleShape(radius=particle_radius),
                density=density,
                friction=particle_friction,
            )
            if use_filter:
                pfd.filter.categoryBits = self._CATEGORY_PARTICLE
                pfd.filter.maskBits = self._CATEGORY_GLASS | self._CATEGORY_WIPER | self._CATEGORY_PARTICLE
            particle = self._world.CreateDynamicBody(
                position=(x, y),
                fixtures=pfd,
            )
            particle.linearDamping = self._default_linear_damping
            particle.angularDamping = self._default_angular_damping
            self._particles.append(particle)
        self._initial_particle_count = len(self._particles)
    def _create_initial_wiper_template(self, terrain_config: dict):
        spawn_x = 6.0
        spawn_y = 4.0
        body_width = 0.3
        body_height = 0.3
        body = self._world.CreateDynamicBody(
            position=(spawn_x, spawn_y),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(body_width/2, body_height/2)),
                density=1.0,
                friction=0.5,
            )
        )
        body.linearDamping = self._default_linear_damping
        body.angularDamping = self._default_angular_damping
        self._wiper_bodies["body_template"] = body
    def remove_initial_template(self):
        if "body_template" in self._wiper_bodies:
            body = self._wiper_bodies.pop("body_template")
            if body and self._world:
                self._world.DestroyBody(body)
    def weld_to_glass(self, body, anchor_point):
        glass = self._terrain_bodies.get("glass")
        if glass is None or body is None:
            return
        ax, ay = float(anchor_point[0]), float(anchor_point[1])
        joint = self._world.CreateWeldJoint(
            bodyA=glass,
            bodyB=body,
            anchor=(ax, ay),
            collideConnected=False
        )
        self._joints.append(joint)
    MIN_BEAM_SIZE = 0.05
    MAX_BEAM_SIZE = 2.0
    MIN_JOINT_LIMIT = -math.pi
    MAX_JOINT_LIMIT = math.pi
    BUILD_ZONE_X_MIN = 0.0
    BUILD_ZONE_X_MAX = 12.0
    BUILD_ZONE_Y_MIN = 2.0
    BUILD_ZONE_Y_MAX = 10.0
    MAX_STRUCTURE_MASS = 15.0
    def add_beam(self, x, y, width, height, angle=0, density=1.0):
        width = max(self.MIN_BEAM_SIZE, min(width, self.MAX_BEAM_SIZE))
        height = max(self.MIN_BEAM_SIZE, min(height, self.MAX_BEAM_SIZE))
        beam_fd = Box2D.b2FixtureDef(
            shape=polygonShape(box=(width/2, height/2)),
            density=density,
            friction=0.5,
        )
        if self._terrain_config.get("wiper_ignore_glass_collision"):
            beam_fd.filter.categoryBits = self._CATEGORY_WIPER
            beam_fd.filter.maskBits = self._CATEGORY_WIPER | self._CATEGORY_PARTICLE
        body = self._world.CreateDynamicBody(
            position=(x, y),
            angle=angle,
            fixtures=beam_fd,
        )
        body.linearDamping = self._default_linear_damping
        body.angularDamping = self._default_angular_damping
        self._bodies.append(body)
        return body
    def add_joint(self, body_a, body_b, anchor_point, type='pivot', lower_limit=None, upper_limit=None):
        if body_a is None:
            raise ValueError("add_joint: body_a cannot be None. You must provide a valid body object (e.g., from add_beam).")
        if body_b is None:
            raise ValueError("add_joint: body_b cannot be None. You must provide a valid body object.")
        anchor_x, anchor_y = anchor_point[0], anchor_point[1]
        if type == 'rigid':
            joint = self._world.CreateWeldJoint(
                bodyA=body_a,
                bodyB=body_b,
                anchor=(anchor_x, anchor_y),
                collideConnected=False
            )
        elif type == 'pivot':
            joint_kwargs = {
                'bodyA': body_a,
                'bodyB': body_b,
                'anchor': (anchor_x, anchor_y),
                'collideConnected': False
            }
            if lower_limit is not None and upper_limit is not None:
                joint_kwargs['lowerAngle'] = max(self.MIN_JOINT_LIMIT, min(lower_limit, self.MAX_JOINT_LIMIT))
                joint_kwargs['upperAngle'] = min(self.MAX_JOINT_LIMIT, max(upper_limit, self.MIN_JOINT_LIMIT))
                joint_kwargs['enableLimit'] = True
            joint = self._world.CreateRevoluteJoint(**joint_kwargs)
        else:
            raise ValueError(f"Unknown joint type: {type}")
        self._joints.append(joint)
        return joint
    def set_motor(self, joint, motor_speed, max_torque=100.0):
        if not isinstance(joint, Box2D.b2RevoluteJoint):
            raise ValueError("set_motor: joint must be a pivot/revolute joint")
        torque = float(max_torque)
        if getattr(self, "_max_motor_torque_cap", None) is not None:
            torque = min(torque, self._max_motor_torque_cap)
        joint.motorEnabled = True
        joint.motorSpeed = float(motor_speed)
        joint.maxMotorTorque = torque
        self._wiper_motor_joint = joint
        self.record_torque_request(float(max_torque))
    def get_structure_mass(self):
        total_mass = 0.0
        for body in self._bodies:
            total_mass += body.mass
        return total_mass
    def set_material_properties(self, body, restitution=0.2, friction=None):
        for fixture in body.fixtures:
            fixture.restitution = float(restitution)
            if friction is not None:
                fixture.friction = float(friction)
    def set_awake(self, body, awake=True):
        if body:
            body.awake = bool(awake)
    def step(self, time_step):
        self._world.Step(time_step, 10, 10)
    def get_terrain_bounds(self):
        return {
            "glass": {"y": self._glass_y, "length": self._glass_length, "friction": self._glass_friction},
            "build_zone": {
                "x": [self.BUILD_ZONE_X_MIN, self.BUILD_ZONE_X_MAX],
                "y": [self.BUILD_ZONE_Y_MIN, self.BUILD_ZONE_Y_MAX]
            }
        }
    def get_wiper_position(self):
        if not self._bodies:
            return None
        if self._bodies:
            body = self._bodies[0]
            return (body.position.x, body.position.y)
        return None
    def get_particle_count(self):
        remaining = 0
        for particle in self._particles:
            if abs(particle.position.y - self._glass_y) < 0.5 and 0.5 <= particle.position.x <= 11.5:
                remaining += 1
        return remaining
    def get_remaining_particle_count(self):
        return self.get_particle_count()
    def get_initial_particle_count(self):
        return self._initial_particle_count
    def get_particle_properties(self):
        return {
            'friction': getattr(self, '_particle_friction', 0.35),
            'mass': getattr(self, '_particle_mass', 0.15),
            'radius': getattr(self, '_particle_radius', 0.08),
            'count': getattr(self, '_particle_count', 45),
        }
    def get_particle_positions_on_glass(self):
        positions = []
        for particle in self._particles:
            px, py = float(particle.position.x), float(particle.position.y)
            if 0.5 <= px <= 11.5 and abs(py - self._glass_y) < 0.5:
                positions.append((round(px, 3), round(py, 3)))
        return positions
    def get_joint_angle_info(self):
        result = []
        for j_idx, joint in enumerate(self._joints):
            try:
                if hasattr(joint, 'angle') and hasattr(joint, 'limits'):
                    angle = float(joint.angle)
                    lower = float(joint.limits[0]) if joint.limits else None
                    upper = float(joint.limits[1]) if joint.limits else None
                    result.append({
                        'joint_index': j_idx,
                        'angle_rad': round(angle, 4),
                        'angle_deg': round(math.degrees(angle), 2),
                        'lower_limit_rad': round(lower, 4) if lower is not None else None,
                        'upper_limit_rad': round(upper, 4) if upper is not None else None,
                    })
            except Exception:
                pass
        return result
    def get_motor_energy(self):
        return self._forensic_motor_energy
    def get_peak_body_velocity(self):
        return self._forensic_peak_body_velocity
    def get_body_velocity_warnings(self):
        return list(self._forensic_body_velocity_warnings)
    def get_violation_info(self):
        return self._forensic_violation_info
    def update_forensic_state(self, step_count: int):
        import math
        current_min_y = float('inf')
        current_max_y = float('-inf')
        current_min_x = float('inf')
        current_max_x = float('-inf')
        for body in self._bodies:
            px, py = body.position.x, body.position.y
            if math.isfinite(px) and math.isfinite(py):
                if py < current_min_y:
                    current_min_y = py
                if py > current_max_y:
                    current_max_y = py
                if px < current_min_x:
                    current_min_x = px
                if px > current_max_x:
                    current_max_x = px
                if py < self._forensic_min_body_reach_y:
                    self._forensic_min_body_reach_y = py
                if py > self._forensic_max_body_reach_y:
                    self._forensic_max_body_reach_y = py
        if current_min_y < float('inf'):
            self._forensic_min_body_y = min(self._forensic_min_body_y, current_min_y)
            self._forensic_max_body_y = max(self._forensic_max_body_y, current_max_y)
            self._forensic_min_body_x = min(self._forensic_min_body_x, current_min_x)
            self._forensic_max_body_x = max(self._forensic_max_body_x, current_max_x)
            self._forensic_body_count = len(self._bodies)
        if self._forensic_violation_info is None and current_min_x < float('inf'):
            for body in self._bodies:
                px, py = body.position.x, body.position.y
                if not (math.isfinite(px) and math.isfinite(py)):
                    continue
                if not (self.BUILD_ZONE_X_MIN <= px <= self.BUILD_ZONE_X_MAX and
                        self.BUILD_ZONE_Y_MIN <= py <= self.BUILD_ZONE_Y_MAX):
                    self._forensic_violation_info = {
                        'step': step_count,
                        'body_x': round(float(px), 3),
                        'body_y': round(float(py), 3),
                        'build_zone_x': [self.BUILD_ZONE_X_MIN, self.BUILD_ZONE_X_MAX],
                        'build_zone_y': [self.BUILD_ZONE_Y_MIN, self.BUILD_ZONE_Y_MAX],
                    }
                    break
        current_particle_count = self.get_remaining_particle_count()
        if self._forensic_last_particle_count is not None:
            removed = self._initial_particle_count - current_particle_count
            self._forensic_particles_removed_history.append((step_count, removed))
        self._forensic_last_particle_count = current_particle_count
        self._forensic_step_count = step_count
        if step_count % 60 == 0 or step_count == self._forensic_last_step + 1:
            for j_idx, joint in enumerate(self._joints):
                try:
                    if hasattr(joint, 'angle'):
                        angle = float(joint.angle)
                        lower = float(joint.limits[0]) if hasattr(joint, 'limits') and joint.limits else None
                        upper = float(joint.limits[1]) if hasattr(joint, 'limits') and joint.limits else None
                        self._forensic_joint_angle_history.append({
                            'step': step_count,
                            'joint_index': j_idx,
                            'angle': round(angle, 4),
                            'lower_limit': round(lower, 4) if lower is not None else None,
                            'upper_limit': round(upper, 4) if upper is not None else None,
                        })
                except Exception:
                    pass
        for joint in self._joints:
            try:
                if hasattr(joint, 'motorEnabled') and joint.motorEnabled:
                    torque = float(joint.motorTorque) if hasattr(joint, 'motorTorque') else 0.0
                    speed = float(joint.speed) if hasattr(joint, 'speed') else 0.0
                    self._forensic_motor_energy += abs(torque * speed) * (1.0 / 60.0)
            except Exception:
                pass
        for b_idx, body in enumerate(self._bodies):
            px, py = body.position.x, body.position.y
            vx, vy = body.linearVelocity.x, body.linearVelocity.y
            if not (math.isfinite(px) and math.isfinite(py) and
                    math.isfinite(vx) and math.isfinite(vy)):
                self._forensic_nan_detected = True
                self._forensic_body_velocity_warnings.append({
                    'step': step_count,
                    'body_index': b_idx,
                    'issue': 'NaN/Inf in position or velocity',
                    'px': str(px), 'py': str(py),
                    'vx': str(vx), 'vy': str(vy),
                })
                continue
            speed = math.sqrt(vx**2 + vy**2)
            if speed > self._forensic_peak_body_velocity:
                self._forensic_peak_body_velocity = speed
            if speed > 1000:
                self._forensic_nan_detected = True
                self._forensic_body_velocity_warnings.append({
                    'step': step_count,
                    'body_index': b_idx,
                    'issue': f'extreme_velocity',
                    'speed': round(speed, 1),
                    'px': round(float(px), 3),
                    'py': round(float(py), 3),
                })
            elif speed > 100 and len(self._forensic_body_velocity_warnings) < 10:
                self._forensic_body_velocity_warnings.append({
                    'step': step_count,
                    'body_index': b_idx,
                    'issue': f'high_velocity',
                    'speed': round(speed, 1),
                    'px': round(float(px), 3),
                    'py': round(float(py), 3),
                })
        self._forensic_last_step = step_count
    def record_torque_request(self, requested_torque: float):
        self._forensic_torque_requested = requested_torque
    def get_forensic_snapshot(self, step_count: int) -> dict:
        import math
        x_min_margin = self._forensic_min_body_x - self.BUILD_ZONE_X_MIN if self._forensic_min_body_x < float('inf') else None
        x_max_margin = self.BUILD_ZONE_X_MAX - self._forensic_max_body_x if self._forensic_max_body_x > 0 else None
        y_min_margin = self._forensic_min_body_y - self.BUILD_ZONE_Y_MIN if self._forensic_min_body_y < float('inf') else None
        y_max_margin = self.BUILD_ZONE_Y_MAX - self._forensic_max_body_y if self._forensic_max_body_y > 0 else None
        span_x = self._forensic_max_body_x - self._forensic_min_body_x if (self._forensic_max_body_x > 0 and self._forensic_min_body_x < float('inf')) else None
        wiper_bottom_y = self._forensic_min_body_y if self._forensic_min_body_y < float('inf') else None
        particle_top_y = self._glass_y + 0.08
        particle_contact_gap = None
        if wiper_bottom_y is not None:
            particle_contact_gap = wiper_bottom_y - particle_top_y
        torque_capped = False
        torque_capped_by = None
        if self._forensic_torque_cap is not None and self._forensic_torque_requested is not None:
            if self._forensic_torque_requested > self._forensic_torque_cap:
                torque_capped = True
                torque_capped_by = self._forensic_torque_cap
        nan_detected = self._forensic_nan_detected
        removal_trajectory = list(self._forensic_particles_removed_history)
        if self._forensic_last_particle_count is not None:
            last_removed = self._initial_particle_count - self._forensic_last_particle_count
            if removal_trajectory and removal_trajectory[-1][0] != step_count:
                removal_trajectory.append((step_count, last_removed))
        current_removed = self._initial_particle_count - self.get_remaining_particle_count()
        joint_angle_summary = []
        for entry in self._forensic_joint_angle_history[-30:]:
            joint_angle_summary.append(entry)
        return {
            'step': step_count,
            'structure_min_x': round(self._forensic_min_body_x, 3) if self._forensic_min_body_x < float('inf') else None,
            'structure_max_x': round(self._forensic_max_body_x, 3) if self._forensic_max_body_x > 0 else None,
            'structure_min_y': round(self._forensic_min_body_y, 3) if self._forensic_min_body_y < float('inf') else None,
            'structure_max_y': round(self._forensic_max_body_y, 3) if self._forensic_max_body_y > 0 else None,
            'structure_span_x': round(span_x, 3) if span_x is not None else None,
            'wiper_bottom_y': round(wiper_bottom_y, 3) if wiper_bottom_y is not None else None,
            'glass_y': self._glass_y,
            'particle_radius': 0.08,
            'particle_top_y': round(particle_top_y, 3),
            'particle_contact_gap': round(particle_contact_gap, 3) if particle_contact_gap is not None else None,
            'x_min_margin': round(x_min_margin, 3) if x_min_margin is not None else None,
            'x_max_margin': round(x_max_margin, 3) if x_max_margin is not None else None,
            'y_min_margin': round(y_min_margin, 3) if y_min_margin is not None else None,
            'y_max_margin': round(y_max_margin, 3) if y_max_margin is not None else None,
            'build_zone_x': [self.BUILD_ZONE_X_MIN, self.BUILD_ZONE_X_MAX],
            'build_zone_y': [self.BUILD_ZONE_Y_MIN, self.BUILD_ZONE_Y_MAX],
            'max_motor_torque_cap': self._forensic_torque_cap,
            'last_torque_requested': self._forensic_torque_requested,
            'torque_capped': torque_capped,
            'torque_capped_by': torque_capped_by,
            'numerical_nan_detected': nan_detected,
            'particles_removed_so_far': current_removed,
            'initial_particle_count': self._initial_particle_count,
            'removal_trajectory': removal_trajectory,
            'max_body_reach_y': round(self._forensic_max_body_reach_y, 3) if self._forensic_max_body_reach_y > 0 else None,
            'min_body_reach_y': round(self._forensic_min_body_reach_y, 3) if self._forensic_min_body_reach_y < float('inf') else None,
            'joint_angle_history': joint_angle_summary,
            'motor_energy_joules': round(self._forensic_motor_energy, 2),
            'peak_body_velocity': round(self._forensic_peak_body_velocity, 2) if self._forensic_peak_body_velocity > 0 else None,
            'body_velocity_warnings': list(self._forensic_body_velocity_warnings),
            'violation_info': self._forensic_violation_info,
        }
    def reset_forensic_state(self):
        self._forensic_min_body_y = float('inf')
        self._forensic_max_body_y = float('-inf')
        self._forensic_min_body_x = float('inf')
        self._forensic_max_body_x = float('-inf')
        self._forensic_body_count = 0
        self._forensic_wiper_y_at_eval = None
        self._forensic_particles_removed_history = []
        self._forensic_last_particle_count = None
        self._forensic_step_count = 0
        self._forensic_nan_detected = False
        self._forensic_torque_requested = None
        self._forensic_max_body_reach_y = float('-inf')
        self._forensic_min_body_reach_y = float('inf')
        self._forensic_joint_angle_history = []
        self._forensic_motor_energy = 0.0
        self._forensic_peak_body_velocity = 0.0
        self._forensic_body_velocity_warnings = []
        self._forensic_violation_info = None
        self._forensic_last_step = -1
        if hasattr(self, '_initial_particle_count'):
            self._forensic_last_particle_count = self._initial_particle_count
