import math

import Box2D

from Box2D.b2 import world, polygonShape

_kinematicBody = getattr(Box2D.b2, "kinematicBody", 1)
FIXED_TIME_STEP = 1.0 / 60.0

class Sandbox:
    BUILD_ZONE_X_MIN = 5.0
    BUILD_ZONE_X_MAX = 15.0
    BUILD_ZONE_Y_MIN = 1.5
    BUILD_ZONE_Y_MAX = 8.0
    MAX_STRUCTURE_MASS = 400.0
    MIN_BEAM_SIZE = 0.1
    MAX_BEAM_SIZE = 4.0
    MIN_BEAMS = 5
    MIN_JOINTS = 6
    SPAN_LEFT_X = 6.0
    SPAN_RIGHT_X = 14.0
    MASS_FREQ_1 = 0.5
    MASS_AMP_1 = 0.2
    MASS_FREQ_2 = 1.0
    MASS_AMP_2 = 0.16
    MASS_PHASE_GRADIENT = 0.4
    BASE_EXCITATION_VERTICAL_AMPLITUDE = 0.06
    BASE_EXCITATION_HORIZONTAL_AMPLITUDE = 0.04
    BASE_EXCITATION_FREQUENCY = 0.45
    JOINT_BREAK_FORCE = 6.0
    JOINT_BREAK_TORQUE = 10.0
    FATIGUE_TAU_SECONDS = 100.0
    WIND_PRESSURE = 0.0
    MAX_STEPS = 12000
    def __init__(self, *, terrain_config=None, physics_config=None):
        terrain_config = terrain_config or {}
        physics_config = physics_config or {}
        self._terrain_config = dict(terrain_config)
        self._physics_config = dict(physics_config)
        gravity = tuple(physics_config.get("gravity", (0, -10)))
        self._linear_damping = float(physics_config.get("linear_damping", 0.0))
        self._angular_damping = float(physics_config.get("angular_damping", 0.0))
        self.MASS_FREQ_1 = physics_config.get("mass_freq_1", self.MASS_FREQ_1)
        self.MASS_AMP_1 = physics_config.get("mass_amp_1", self.MASS_AMP_1)
        self.MASS_FREQ_2 = physics_config.get("mass_freq_2", self.MASS_FREQ_2)
        self.MASS_AMP_2 = physics_config.get("mass_amp_2", self.MASS_AMP_2)
        self.MASS_PHASE_GRADIENT = physics_config.get("mass_phase_gradient", self.MASS_PHASE_GRADIENT)
        self.BASE_EXCITATION_VERTICAL_AMPLITUDE = physics_config.get(
            "base_excitation_vertical_amplitude", self.BASE_EXCITATION_VERTICAL_AMPLITUDE
        )
        self.BASE_EXCITATION_HORIZONTAL_AMPLITUDE = physics_config.get(
            "base_excitation_horizontal_amplitude", self.BASE_EXCITATION_HORIZONTAL_AMPLITUDE
        )
        self.BASE_EXCITATION_FREQUENCY = physics_config.get(
            "base_excitation_frequency", self.BASE_EXCITATION_FREQUENCY
        )
        self.JOINT_BREAK_FORCE = float(physics_config.get("joint_break_force", self.JOINT_BREAK_FORCE))
        self.JOINT_BREAK_TORQUE = float(physics_config.get("joint_break_torque", self.JOINT_BREAK_TORQUE))
        self.FATIGUE_TAU_SECONDS = float(physics_config.get("fatigue_tau_seconds", self.FATIGUE_TAU_SECONDS))
        self.WIND_PRESSURE = float(physics_config.get("wind_pressure", 0.0))
        required_positive = {
            "joint_break_force": self.JOINT_BREAK_FORCE,
            "joint_break_torque": self.JOINT_BREAK_TORQUE,
            "fatigue_tau_seconds": self.FATIGUE_TAU_SECONDS,
        }
        for name, value in required_positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        self._build_zone_x_min = float(terrain_config.get("build_zone_x_min", self.BUILD_ZONE_X_MIN))
        self._build_zone_x_max = float(terrain_config.get("build_zone_x_max", self.BUILD_ZONE_X_MAX))
        self._build_zone_y_min = float(terrain_config.get("build_zone_y_min", self.BUILD_ZONE_Y_MIN))
        self._build_zone_y_max = float(terrain_config.get("build_zone_y_max", self.BUILD_ZONE_Y_MAX))
        self._max_structure_mass = float(terrain_config.get("max_structure_mass", self.MAX_STRUCTURE_MASS))
        self._span_left_x = float(terrain_config.get("span_left_x", self.SPAN_LEFT_X))
        self._span_right_x = float(terrain_config.get("span_right_x", self.SPAN_RIGHT_X))
        self._min_beams = int(terrain_config.get("min_beams", self.MIN_BEAMS))
        self._min_joints = int(terrain_config.get("min_joints", self.MIN_JOINTS))
        self._world = world(gravity=(0, -10), doSleep=True)
        self._world.gravity = gravity
        self._bodies = []
        self._joints = []
        self._terrain_bodies = {}
        self._time = 0.0
        self._joint_peak_forces = {}
        self._joint_peak_torques = {}
        self._joint_types = {}
        self._joint_first_failure_step = {}
        self._joint_anchor_positions = {}
        self._joints_ever_broken = []
        self._peak_body_speed = 0.0
        self._peak_reaction_force_all = 0.0
        self._peak_reaction_torque_all = 0.0
        self._closest_force_event = None
        self._closest_torque_event = None
        self._peak_structure_mass = 0.0
        self._first_mass_violation_step = None
        self._step_count = 0
        self.world = self._world
        self.bodies = self._bodies
        self.joints = self._joints
        self._create_terrain(terrain_config)
    def _create_terrain(self, terrain_config: dict):
        ground_length = 40.0
        ground_height = 1.0
        self._ground_y = ground_height
        self._ground_y_base = ground_height / 2.0
        body_def = Box2D.b2BodyDef()
        body_def.type = _kinematicBody
        body_def.position = (ground_length / 2, self._ground_y_base)
        ground = self._world.CreateBody(body_def)
        ground.CreateFixture(Box2D.b2FixtureDef(
            shape=polygonShape(box=(ground_length / 2, ground_height / 2)),
            friction=0.6,
        ))
        self._terrain_bodies["ground"] = ground
    def _mass_factor_for_phase(self, t, phase):
        return (1.0
                + self.MASS_AMP_1 * math.sin(2.0 * math.pi * self.MASS_FREQ_1 * t + phase)
                + self.MASS_AMP_2 * math.sin(2.0 * math.pi * self.MASS_FREQ_2 * t + 2.0 * phase))
    def _effective_joint_force_limit(self, t):
        return self.JOINT_BREAK_FORCE * math.exp(-t / self.FATIGUE_TAU_SECONDS)
    def _effective_joint_torque_limit(self, t):
        return self.JOINT_BREAK_TORQUE * math.exp(-t / self.FATIGUE_TAU_SECONDS)
    def step(self, time_step):
        del time_step
        time_step = FIXED_TIME_STEP
        t = self._time
        for body in self._bodies:
            base = getattr(body, "_base_density", None)
            phase = getattr(body, "_mass_phase", 0.0)
            if base is not None:
                factor = self._mass_factor_for_phase(t, phase)
                for fixture in body.fixtures:
                    fixture.density = base * factor
                body.ResetMassData()
            if self.WIND_PRESSURE != 0.0:
                area = getattr(body, "_area", 0.1)
                force_x = self.WIND_PRESSURE * area
                body.ApplyForceToCenter((force_x, 0.0), True)
        current_mass = self.get_structure_mass()
        self._peak_structure_mass = max(self._peak_structure_mass, current_mass)
        if (
            current_mass > self._max_structure_mass
            and self._first_mass_violation_step is None
        ):
            self._first_mass_violation_step = self._step_count + 1
        ground = self._terrain_bodies.get("ground")
        if ground is not None:
            omega = 2.0 * math.pi * self.BASE_EXCITATION_FREQUENCY
            vx = self.BASE_EXCITATION_HORIZONTAL_AMPLITUDE * omega * math.cos(omega * t)
            vy = self.BASE_EXCITATION_VERTICAL_AMPLITUDE * omega * math.cos(omega * t)
            ground.linearVelocity = (vx, vy)
        self._time += time_step
        self._world.Step(time_step, 10, 10)
        for body in self._bodies:
            self._peak_body_speed = max(
                self._peak_body_speed,
                math.hypot(body.linearVelocity.x, body.linearVelocity.y),
            )
        force_limit = self._effective_joint_force_limit(self._time)
        torque_limit = self._effective_joint_torque_limit(self._time)
        joints_to_remove = []
        for joint in list(self._joints):
            force = joint.GetReactionForce(1.0 / time_step)
            force_mag = math.hypot(force.x, force.y)
            torque_mag = abs(joint.GetReactionTorque(1.0 / time_step))
            self._joint_peak_forces[joint] = max(
                self._joint_peak_forces.get(joint, 0.0), force_mag
            )
            self._joint_peak_torques[joint] = max(
                self._joint_peak_torques.get(joint, 0.0), torque_mag
            )
            self._peak_reaction_force_all = max(
                self._peak_reaction_force_all, force_mag
            )
            self._peak_reaction_torque_all = max(
                self._peak_reaction_torque_all, torque_mag
            )
            anchor = self._joint_anchor_positions.get(joint, (None, None))
            joint_type = self._joint_types.get(joint, "unknown")
            force_utilization = force_mag / force_limit
            torque_utilization = torque_mag / torque_limit
            if (
                self._closest_force_event is None
                or force_utilization > self._closest_force_event["utilization"]
            ):
                self._closest_force_event = {
                    "utilization": force_utilization,
                    "step": self._step_count + 1,
                    "joint_type": joint_type,
                    "anchor_x": anchor[0],
                    "anchor_y": anchor[1],
                    "load": force_mag,
                    "limit": force_limit,
                }
            if (
                self._closest_torque_event is None
                or torque_utilization > self._closest_torque_event["utilization"]
            ):
                self._closest_torque_event = {
                    "utilization": torque_utilization,
                    "step": self._step_count + 1,
                    "joint_type": joint_type,
                    "anchor_x": anchor[0],
                    "anchor_y": anchor[1],
                    "load": torque_mag,
                    "limit": torque_limit,
                }
            exceeded = (
                force_mag > force_limit
                or torque_mag > torque_limit
            )
            if exceeded and self._joint_first_failure_step.get(joint) is None:
                self._joint_first_failure_step[joint] = self._step_count + 1
            if exceeded:
                joints_to_remove.append(
                    (joint, force_mag, torque_mag, force_limit, torque_limit)
                )
        for (
            joint,
            force_at_break,
            torque_at_break,
            force_limit_at_break,
            torque_limit_at_break,
        ) in joints_to_remove:
            joint_type = self._joint_types.get(joint, "unknown")
            anchor = self._joint_anchor_positions.get(joint, (None, None))
            break_step = self._joint_first_failure_step.get(joint, None)
            self._joints_ever_broken.append({
                "joint_type": joint_type,
                "anchor_x": anchor[0],
                "anchor_y": anchor[1],
                "break_step": break_step,
                "force_at_break": force_at_break,
                "torque_at_break": torque_at_break,
                "peak_force_at_break": self._joint_peak_forces.get(joint, None),
                "peak_torque_at_break": self._joint_peak_torques.get(joint, None),
                "force_limit_at_break": force_limit_at_break,
                "torque_limit_at_break": torque_limit_at_break,
            })
            self._world.DestroyJoint(joint)
            self._joints.remove(joint)
            self._joint_peak_forces.pop(joint, None)
            self._joint_peak_torques.pop(joint, None)
            self._joint_types.pop(joint, None)
            self._joint_first_failure_step.pop(joint, None)
            self._joint_anchor_positions.pop(joint, None)
        self._step_count += 1
    def add_beam(self, x, y, width, height, angle=0, density=1.0):
        values = tuple(float(value) for value in (x, y, width, height, angle, density))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Beam parameters must be finite")
        x, y, width, height, angle, density = values
        if density <= 0.0:
            raise ValueError("Beam density must be positive")
        width = max(self.MIN_BEAM_SIZE, min(width, self.MAX_BEAM_SIZE))
        height = max(self.MIN_BEAM_SIZE, min(height, self.MAX_BEAM_SIZE))
        body = self._world.CreateDynamicBody(
            position=(x, y),
            # The documented primitive uses clockwise-positive angles, while
            # Box2D uses counter-clockwise-positive angles.
            angle=-angle,
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(width / 2, height / 2)),
                density=density,
                friction=0.5,
            ),
        )
        body._base_density = density
        body._mass_phase = (x - self._build_zone_x_min) * self.MASS_PHASE_GRADIENT
        body._area = width * height
        body.linearDamping = self._linear_damping
        body.angularDamping = self._angular_damping
        self._bodies.append(body)
        return body
    def add_joint(self, body_a, body_b, anchor_point, type="rigid", **kwargs):
        if body_a is None:
            raise ValueError("add_joint: body_a cannot be None.")
        if body_a not in self._bodies:
            raise ValueError("add_joint: body_a must be an agent-created beam.")
        anchor_x, anchor_y = float(anchor_point[0]), float(anchor_point[1])
        if not math.isfinite(anchor_x) or not math.isfinite(anchor_y):
            raise ValueError("add_joint: anchor coordinates must be finite.")
        if body_b is None:
            body_b = self._terrain_bodies.get("ground")
            if body_b is None:
                raise ValueError("add_joint: Cannot anchor to ground.")
        elif body_b not in self._bodies:
            raise ValueError("add_joint: body_b must be an agent-created beam or None.")
        if type == "rigid":
            if kwargs:
                raise ValueError("Rigid joints do not accept pivot options.")
            joint = self._world.CreateWeldJoint(
                bodyA=body_a, bodyB=body_b,
                anchor=(anchor_x, anchor_y), collideConnected=False,
            )
        elif type == "pivot":
            allowed_options = {
                "lower_limit",
                "upper_limit",
                "enable_motor",
                "motor_speed",
                "max_motor_torque",
            }
            unknown = set(kwargs) - allowed_options
            if unknown:
                raise ValueError(
                    "Unknown pivot option(s): " + ", ".join(sorted(unknown))
                )
            lower = float(kwargs.get("lower_limit", 0.0))
            upper = float(kwargs.get("upper_limit", 0.0))
            motor_speed = float(kwargs.get("motor_speed", 0.0))
            max_motor_torque = float(kwargs.get("max_motor_torque", 0.0))
            if not all(
                math.isfinite(value)
                for value in (lower, upper, motor_speed, max_motor_torque)
            ):
                raise ValueError("Pivot options must be finite.")
            if (
                ("lower_limit" in kwargs or "upper_limit" in kwargs)
                and lower > upper
            ):
                raise ValueError("Pivot lower_limit cannot exceed upper_limit.")
            joint = self._world.CreateRevoluteJoint(
                bodyA=body_a, bodyB=body_b,
                anchor=(anchor_x, anchor_y), collideConnected=False,
                enableLimit=("lower_limit" in kwargs or "upper_limit" in kwargs),
                lowerAngle=lower,
                upperAngle=upper,
                enableMotor=bool(kwargs.get("enable_motor", False)),
                motorSpeed=motor_speed,
                maxMotorTorque=max_motor_torque,
            )
        else:
            raise ValueError(f"Unknown joint type: {type}")
        self._joints.append(joint)
        self._joint_peak_forces[joint] = 0.0
        self._joint_peak_torques[joint] = 0.0
        self._joint_types[joint] = type
        self._joint_anchor_positions[joint] = (anchor_x, anchor_y)
        self._joint_first_failure_step[joint] = None
        return joint
    def get_structure_mass(self):
        total = 0.0
        for body in self._bodies:
            total += body.mass
        return total
    def get_peak_structure_mass(self):
        return max(self._peak_structure_mass, self.get_structure_mass())
    def get_first_mass_violation_step(self):
        return self._first_mass_violation_step
    def get_max_joint_reaction_force(self):
        return self._peak_reaction_force_all
    def get_max_joint_reaction_torque(self):
        return self._peak_reaction_torque_all
    def get_effective_joint_force_limit(self):
        return self._effective_joint_force_limit(self._time)
    def get_effective_joint_torque_limit(self):
        return self._effective_joint_torque_limit(self._time)
    def get_ground_y_top(self):
        return self._ground_y
    def get_build_zone(self):
        return (self._build_zone_x_min, self._build_zone_x_max, self._build_zone_y_min, self._build_zone_y_max)
    def get_span_bounds(self):
        return (self._span_left_x, self._span_right_x)
    def get_structure_mass_limit(self):
        return self._max_structure_mass
    def get_joints_ever_broken(self):
        return list(self._joints_ever_broken)
    def get_per_joint_peaks(self):
        result = {}
        for joint in self._joints:
            jid = id(joint)
            result[jid] = {
                "force": self._joint_peak_forces.get(joint, 0.0),
                "torque": self._joint_peak_torques.get(joint, 0.0),
                "type": self._joint_types.get(joint, "unknown"),
                "anchor_x": self._joint_anchor_positions.get(joint, (None, None))[0],
                "anchor_y": self._joint_anchor_positions.get(joint, (None, None))[1],
            }
        return result
    def get_peak_body_speed(self):
        return self._peak_body_speed
    def get_closest_joint_margin_events(self):
        return {
            "force": (
                dict(self._closest_force_event)
                if self._closest_force_event is not None
                else None
            ),
            "torque": (
                dict(self._closest_torque_event)
                if self._closest_torque_event is not None
                else None
            ),
        }
    def get_current_fatigue_factor(self):
        return math.exp(-self._time / self.FATIGUE_TAU_SECONDS)
    def get_simulation_time(self):
        return self._time
    def get_structure_counts(self):
        return len(self._bodies), len(self._joints)
    def get_structure_positions(self):
        return tuple(
            (float(body.position.x), float(body.position.y))
            for body in self._bodies
        )
    def get_joint_types(self):
        return tuple(self._joint_types.values())
    def get_min_beams(self):
        return self._min_beams
    def get_min_joints(self):
        return self._min_joints
    def get_terrain_bounds(self):
        return {
            "ground_y": self._ground_y,
            "build_zone": {
                "x": [self._build_zone_x_min, self._build_zone_x_max],
                "y": [self._build_zone_y_min, self._build_zone_y_max],
            },
            "max_structure_mass": self._max_structure_mass,
            "span_left_x": self._span_left_x,
            "span_right_x": self._span_right_x,
            "min_beams": self._min_beams,
            "min_joints": self._min_joints,
        }
