import math

from collections import deque

import Box2D

from Box2D.b2 import (
    world,
    circleShape,
    staticBody,
    dynamicBody,

)

TARGET_SPEED_RAD_S = 3.0

TIME_STEP = 1.0 / 60.0

STALL_SPEED_THRESHOLD = 0.3

MEAN_SPEED_ERROR_THRESHOLD = 0.22

REGULATION_START_STEP = 1000

STALL_STEPS_THRESHOLD = 60

MAX_STEPS = 15000

DEFAULT_WHEEL_MASS_KG = 10.0

DEFAULT_WHEEL_RADIUS_M = 0.5

DEFAULT_WHEEL_ANGULAR_DAMPING = 0.02

BASE_LOAD = 2.0

K_DRAG = 0.55

STEP_LOAD_AT_STEP = 3500

STEP_LOAD_EXTRA = 4.0

DISTURB_PERIOD = 400

DISTURB_TORQUE = -5.0

STICTION_SPEED_BAND = 0.5

STICTION_FACTOR = 1.4

COGGING_AMPLITUDE = 1.4

TORQUE_DEADZONE = 2.0

TORQUE_LIMIT_AT_ZERO = 3.5

TORQUE_LIMIT_SLOPE = 15.5

TORQUE_LIMIT_OMEGA_CAP_RAD_S = 3.0

MEASURE_DELAY_STEPS = 5

TARGET_SCHEDULE = [
    (2500, 3.0),
    (5000, 2.0),
    (7500, 4.0),
    (MAX_STEPS, 3.0),

]

class Sandbox:
    def __init__(self, *, terrain_config=None, physics_config=None):
        terrain_config = terrain_config or {}
        physics_config = physics_config or {}
        self._terrain_config = dict(terrain_config)
        self._physics_config = dict(physics_config)
        gravity = tuple(physics_config.get("gravity", (0, -10)))
        self._default_linear_damping = float(physics_config.get("linear_damping", 0.0))
        self._default_angular_damping = float(
            physics_config.get("angular_damping", DEFAULT_WHEEL_ANGULAR_DAMPING)
        )
        self._world = world(gravity=gravity, doSleep=True)
        self._bodies = []
        self._joints = []
        self._terrain_bodies = {}
        self.world = self._world
        self.bodies = self._bodies
        self.joints = self._joints
        self._target_speed = float(terrain_config.get("target_speed_rad_s", TARGET_SPEED_RAD_S))
        self._stall_threshold = float(terrain_config.get("stall_speed_threshold", STALL_SPEED_THRESHOLD))
        self._mean_speed_error_threshold = float(terrain_config.get("mean_speed_error_threshold", MEAN_SPEED_ERROR_THRESHOLD))
        self._wheel_radius = float(terrain_config.get("wheel_radius", DEFAULT_WHEEL_RADIUS_M))
        self._wheel_mass = float(terrain_config.get("wheel_mass", DEFAULT_WHEEL_MASS_KG))
        self._anchor_x = float(terrain_config.get("anchor_x", 5.0))
        self._anchor_y = float(terrain_config.get("anchor_y", 5.0))
        self._regulation_start_step = int(
            terrain_config.get("regulation_start_step", REGULATION_START_STEP)
        )
        self._stall_steps_threshold = int(
            terrain_config.get("stall_steps_threshold", STALL_STEPS_THRESHOLD)
        )
        self._create_anchor_and_wheel(terrain_config)
        self._motor_torque = 0.0
        self._sim_time = 0.0
        self._step_count = 0
        self._last_commanded_torque = 0.0
        self._last_applied_torque = 0.0
        self._last_load_torque = 0.0
        self._last_max_torque = 0.0
        self._measure_delay_steps = int(physics_config.get("measure_delay_steps", MEASURE_DELAY_STEPS))
        self._torque_deadzone = float(physics_config.get("torque_deadzone", TORQUE_DEADZONE))
        self._torque_limit_at_zero = float(physics_config.get("torque_limit_at_zero", TORQUE_LIMIT_AT_ZERO))
        self._torque_limit_slope = float(physics_config.get("torque_limit_slope", TORQUE_LIMIT_SLOPE))
        self._torque_limit_omega_cap = float(physics_config.get("torque_limit_omega_cap_rad_s", TORQUE_LIMIT_OMEGA_CAP_RAD_S))
        self._cogging_amplitude = float(physics_config.get("cogging_amplitude", COGGING_AMPLITUDE))
        self._base_load = float(physics_config.get("base_load", BASE_LOAD))
        self._k_drag = float(physics_config.get("k_drag", K_DRAG))
        self._step_load_at_step = int(physics_config.get("step_load_at_step", STEP_LOAD_AT_STEP))
        self._step_load_extra = float(physics_config.get("step_load_extra", STEP_LOAD_EXTRA))
        self._disturb_period = int(physics_config.get("disturb_period", DISTURB_PERIOD))
        self._disturb_torque = float(physics_config.get("disturb_torque", DISTURB_TORQUE))
        self._stiction_speed_band = float(physics_config.get("stiction_speed_band", STICTION_SPEED_BAND))
        self._stiction_factor = float(physics_config.get("stiction_factor", STICTION_FACTOR))
        self._target_schedule = [(TARGET_SCHEDULE[0][0], self._target_speed)] + list(
            TARGET_SCHEDULE[1:]
        )
        self._omega_history = deque(
            [self._get_real_omega()] * (self._measure_delay_steps + 1),
            maxlen=self._measure_delay_steps + 1,
        )
    def _get_real_omega(self):
        wheel = self._terrain_bodies.get("wheel")
        if wheel is None:
            return 0.0
        return wheel.angularVelocity
    def _get_target_speed_for_step(self, step_count):
        for threshold, target in self._target_schedule:
            if step_count < threshold:
                return target
        return self._target_schedule[-1][1]
    def _create_anchor_and_wheel(self, terrain_config: dict):
        ax, ay = self._anchor_x, self._anchor_y
        r = self._wheel_radius
        density = self._wheel_mass / (math.pi * r * r)
        anchor = self._world.CreateStaticBody(position=(ax, ay))
        anchor.CreateFixture(shape=circleShape(radius=0.1), density=0, friction=0)
        self._terrain_bodies["anchor"] = anchor
        wheel = self._world.CreateDynamicBody(
            position=(ax, ay),
            fixtures=Box2D.b2FixtureDef(
                shape=circleShape(radius=r),
                density=density,
                friction=0.1,
                restitution=0.0,
            ),
        )
        wheel.linearDamping = self._default_linear_damping
        wheel.angularDamping = self._default_angular_damping
        self._terrain_bodies["wheel"] = wheel
        joint = self._world.CreateRevoluteJoint(
            bodyA=anchor,
            bodyB=wheel,
            anchor=(ax, ay),
            collideConnected=False,
        )
        self._joints.append(joint)
        self._terrain_bodies["wheel"] = wheel
    def get_wheel_angular_velocity(self):
        return self._omega_history[0]
    def _get_wheel_angular_velocity_actual(self):
        return self._get_real_omega()
    def get_target_speed(self):
        return self._get_target_speed_for_step(self._step_count)
    def apply_motor_torque(self, torque):
        omega_real = self._get_real_omega()
        max_torque = self._torque_limit_at_zero + self._torque_limit_slope * min(
            abs(omega_real), self._torque_limit_omega_cap
        )
        self._last_commanded_torque = float(torque)
        t = float(torque)
        if abs(t) < self._torque_deadzone:
            t = 0.0
        self._motor_torque = max(-max_torque, min(max_torque, t))
        self._last_applied_torque = self._motor_torque
        self._last_max_torque = max_torque
    def _get_wheel_angle(self):
        wheel = self._terrain_bodies.get("wheel")
        if wheel is None:
            return 0.0
        return wheel.angle
    def _get_load_torque_magnitude(self):
        omega = self._get_real_omega()
        load = self._base_load + self._k_drag * (omega * omega)
        if self._step_count >= self._step_load_at_step:
            load += self._step_load_extra
        if abs(omega) < self._stiction_speed_band:
            load *= self._stiction_factor
        return load
    def _get_cogging_torque(self):
        angle = self._get_wheel_angle()
        return self._cogging_amplitude * math.sin(angle)
    def step(self, time_step):
        if not math.isclose(float(time_step), TIME_STEP, rel_tol=0.0, abs_tol=1e-12):
            raise ValueError(
                f"C-06 requires time_step={TIME_STEP}, received {time_step}"
            )
        wheel = self._terrain_bodies.get("wheel")
        if wheel is not None:
            motor = self._motor_torque
            load_mag = self._get_load_torque_magnitude()
            omega = self._get_real_omega()
            if abs(omega) < 1e-12:
                if abs(motor) < 1e-12:
                    load_torque = 0.0
                else:
                    load_torque = -math.copysign(load_mag, motor)
            else:
                load_torque = -math.copysign(load_mag, omega)
            cogging = self._get_cogging_torque()
            disturbance = self._disturb_torque if (self._step_count % self._disturb_period == 0 and self._step_count > 0) else 0.0
            self._last_load_torque = abs(load_torque)
            wheel.ApplyTorque(motor + load_torque - cogging + disturbance, True)
            self._motor_torque = 0.0
        self._world.Step(time_step, 10, 10)
        self._sim_time += time_step
        self._omega_history.append(self._get_real_omega())
        self._step_count += 1
    def get_terrain_bounds(self):
        out = {
            "target_speed_rad_s": self._target_speed,
            "target_speed_time_varying": True,
            "stall_speed_threshold": self._stall_threshold,
            "stall_steps_threshold": self._stall_steps_threshold,
            "mean_speed_error_threshold": self._mean_speed_error_threshold,
            "regulation_start_step": self._regulation_start_step,
            "max_steps_hint": MAX_STEPS,
        }
        return out
    def _get_wheel_body(self):
        return self._terrain_bodies.get("wheel")
    def _get_last_commanded_torque(self):
        return self._last_commanded_torque
