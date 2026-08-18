import math

import random

from collections import deque

import Box2D

from Box2D.b2 import (
    world,
    polygonShape,
    circleShape,
    staticBody,
    dynamicBody,

)

MAX_SAFE_VERTICAL_SPEED = 2.0

MAX_LANDING_ANGLE = math.radians(10.0)

TOTAL_FUEL_IMPULSE = 5500.0

MIN_FUEL_REMAINING_AT_LANDING = 450.0

GUST_PROB = 0.05

GUST_AMPLITUDE = 55.0

WIND_AMPLITUDE = 28.0

WIND_PERIOD1 = 3.0

WIND_PERIOD2 = 7.0

THRUST_DELAY_STEPS = 3

PLATFORM_CENTER_BASE = 17.0

PLATFORM_AMPLITUDE = 1.8

PLATFORM_PERIOD = 6.0

PLATFORM_HALF_WIDTH = 2.0

BARRIER_X_LEFT = 10.5

BARRIER_X_RIGHT = 13.5

BARRIER_Y_TOP = 6.0

BARRIER_Y_BOTTOM = 20.0

MAX_EPISODE_STEPS = 5000

DEFAULT_TIME_STEP = 1.0 / 60.0

DEFAULT_TIME_STEP_LABEL = "1/60"

CURRICULUM_STAGE2_MAX_LANDING_ANGLE_RAD = 1.2

LAND_TOLERANCE = 0.02

MAX_THRUST = 600.0

MAX_TORQUE = 120.0

SPAWN_X = 6.0

SPAWN_Y = 12.0

GROUND_Y_TOP = 1.0

GROUND_LENGTH = 30.0

GROUND_SLAB_HEIGHT = 0.5

LANDER_HALF_WIDTH = 0.4

LANDER_HALF_HEIGHT = 0.3

LANDER_MASS = 50.0

GROUND_FRICTION = 0.5

LANDER_FRICTION = 0.3

LANDER_RESTITUTION = 0.15

DEFAULT_LINEAR_DAMPING = 0.0

DEFAULT_ANGULAR_DAMPING = 0.1

class Sandbox:
    def __init__(self, *, terrain_config=None, physics_config=None):
        terrain_config = terrain_config or {}
        physics_config = physics_config or {}
        self._terrain_config = dict(terrain_config)
        self._physics_config = dict(physics_config)
        simulation_seed = int(
            physics_config.get(
                "random_seed", terrain_config.get("target_rng_seed", 123)
            )
        )
        self._rng = random.Random(simulation_seed)
        gravity = tuple(physics_config.get("gravity", (0, -10)))
        self._default_linear_damping = float(
            physics_config.get("linear_damping", DEFAULT_LINEAR_DAMPING)
        )
        self._default_angular_damping = float(
            physics_config.get("angular_damping", DEFAULT_ANGULAR_DAMPING)
        )
        self._world = world(gravity=gravity, doSleep=True)
        self._bodies = []
        self._joints = []
        self._terrain_bodies = {}
        self.world = self._world
        self.bodies = self._bodies
        self.joints = self._joints
        self._max_safe_vertical_speed = float(
            terrain_config.get("max_safe_vertical_speed", MAX_SAFE_VERTICAL_SPEED)
        )
        self._ground_y_top = float(terrain_config.get("ground_y_top", GROUND_Y_TOP))
        self._max_landing_angle = float(terrain_config.get("max_landing_angle", MAX_LANDING_ANGLE))
        self._platform_center_base = float(physics_config.get("platform_center_base", PLATFORM_CENTER_BASE))
        self._platform_amplitude = float(physics_config.get("platform_amplitude", PLATFORM_AMPLITUDE))
        self._platform_period = float(physics_config.get("platform_period", PLATFORM_PERIOD))
        self._platform_half_width = float(physics_config.get("platform_half_width", PLATFORM_HALF_WIDTH))
        self._time_step = float(physics_config.get("time_step", DEFAULT_TIME_STEP))
        self._land_tolerance = float(
            physics_config.get("land_tolerance", LAND_TOLERANCE)
        )
        self._lander_half_width = float(terrain_config.get("lander_half_width", LANDER_HALF_WIDTH))
        self._lander_half_height = float(terrain_config.get("lander_half_height", LANDER_HALF_HEIGHT))
        self._lander_mass = float(terrain_config.get("lander_mass", LANDER_MASS))
        self._spawn_x = float(terrain_config.get("spawn_x", SPAWN_X))
        self._spawn_y = float(terrain_config.get("spawn_y", SPAWN_Y))
        self._thrust_delay_steps = int(physics_config.get("thrust_delay_steps", THRUST_DELAY_STEPS))
        qlen = max(1, self._thrust_delay_steps)
        self._thrust_queue = deque([(0.0, 0.0)] * qlen, maxlen=qlen)
        self._step_count = 0
        self._wind_amplitude = float(physics_config.get("wind_amplitude", WIND_AMPLITUDE))
        self._wind_period1 = float(physics_config.get("wind_period1", WIND_PERIOD1))
        self._wind_period2 = float(physics_config.get("wind_period2", WIND_PERIOD2))
        self._gust_prob = float(physics_config.get("gust_prob", GUST_PROB))
        self._gust_amplitude = float(physics_config.get("gust_amplitude", GUST_AMPLITUDE))
        self._sim_time = 0.0
        self._total_fuel = float(physics_config.get("total_fuel_impulse", TOTAL_FUEL_IMPULSE))
        self._remaining_fuel = self._total_fuel
        self._min_fuel_remaining_at_landing = float(
            physics_config.get("min_fuel_remaining_at_landing", MIN_FUEL_REMAINING_AT_LANDING)
        )
        self._barrier_hit = False
        self._barrier_failure_kind = None
        self._corridor_entered = False
        self._corridor_exited = False
        self._corridor_entry_step = None
        self._corridor_entry_x = None
        self._corridor_entry_y = None
        self._corridor_min_y = float('inf')
        self._corridor_max_y = float('-inf')
        self._corridor_exit_step = None
        self._corridor_exit_x = None
        self._corridor_exit_y = None
        self._corridor_violation_step = None
        self._corridor_violation_kind = None
        self._corridor_violation_x = None
        self._corridor_violation_y = None
        self._touchdown_snapshot = None
        def _barrier_param(key: str, default: float) -> float:
            if key in physics_config and physics_config[key] is not None:
                return float(physics_config[key])
            if key in terrain_config and terrain_config[key] is not None:
                return float(terrain_config[key])
            return float(default)
        self._barrier_x_left = _barrier_param("barrier_x_left", BARRIER_X_LEFT)
        self._barrier_x_right = _barrier_param("barrier_x_right", BARRIER_X_RIGHT)
        self._barrier_y_top = _barrier_param("barrier_y_top", BARRIER_Y_TOP)
        self._barrier_y_bottom = _barrier_param("barrier_y_bottom", BARRIER_Y_BOTTOM)
        self._gravity_mutation = physics_config.get("gravity_mutation")
        self._create_ground(terrain_config)
        self._create_lander(terrain_config)
        self._main_thrust = 0.0
        self._steering_torque = 0.0
        self._max_thrust = float(physics_config.get("max_thrust", MAX_THRUST))
        self._max_torque = float(physics_config.get("max_torque", MAX_TORQUE))
        self._max_episode_steps = int(
            physics_config.get("max_episode_steps", MAX_EPISODE_STEPS)
        )
        self._last_applied_thrust = 0.0
        self._last_applied_torque = 0.0
        self._peak_abs_applied_thrust = 0.0
        self._peak_abs_applied_torque = 0.0
        self._thrust_saturation_steps = 0
        self._torque_saturation_steps = 0
        self._first_thrust_saturation_step = None
        self._first_torque_saturation_step = None
        self._applied_main_impulse = 0.0
        self._motion_peaks = {
            "abs_vx": {"value": 0.0, "step": 0},
            "abs_vy": {"value": 0.0, "step": 0},
            "abs_angular_velocity": {"value": 0.0, "step": 0},
        }
    def _create_ground(self, terrain_config: dict):
        ground_len = float(terrain_config.get("ground_length", GROUND_LENGTH))
        ground_h = float(terrain_config.get("ground_slab_height", GROUND_SLAB_HEIGHT))
        self._ground_slab_height = ground_h
        center_y = self._ground_y_top - ground_h / 2
        ground = self._world.CreateStaticBody(
            position=(ground_len / 2, center_y),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(ground_len / 2, ground_h / 2)),
                friction=GROUND_FRICTION,
                restitution=0.0,
            ),
        )
        self._terrain_bodies["ground"] = ground
        self._ground_length = ground_len
    def _create_lander(self, terrain_config: dict):
        hw = self._lander_half_width
        hh = self._lander_half_height
        area = 4.0 * hw * hh
        density = self._lander_mass / area
        lander = self._world.CreateDynamicBody(
            position=(self._spawn_x, self._spawn_y),
            angle=0.0,
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(hw, hh)),
                density=density,
                friction=LANDER_FRICTION,
                restitution=LANDER_RESTITUTION,
            ),
        )
        lander.linearDamping = self._default_linear_damping
        lander.angularDamping = self._default_angular_damping
        self._terrain_bodies["lander"] = lander
    def get_lander_position(self):
        lander = self._terrain_bodies.get("lander")
        if lander is None:
            return (0.0, 0.0)
        return (lander.position.x, lander.position.y)
    def _get_lander_velocity(self):
        lander = self._terrain_bodies.get("lander")
        if lander is None:
            return (0.0, 0.0)
        return (lander.linearVelocity.x, lander.linearVelocity.y)
    def get_lander_velocity(self):
        return self._get_lander_velocity()
    def get_lander_angle(self):
        lander = self._terrain_bodies.get("lander")
        if lander is None:
            return 0.0
        return lander.angle
    def get_lander_angular_velocity(self):
        lander = self._terrain_bodies.get("lander")
        if lander is None:
            return 0.0
        return lander.angularVelocity
    def apply_thrust(self, main_thrust, steering_torque):
        main_thrust = float(main_thrust)
        steering_torque = float(steering_torque)
        if not math.isfinite(main_thrust) or not math.isfinite(steering_torque):
            raise ValueError("Thrust and steering commands must be finite")
        self._main_thrust = max(0.0, min(self._max_thrust, main_thrust))
        self._steering_torque = max(
            -self._max_torque, min(self._max_torque, steering_torque)
        )
    def get_remaining_fuel(self):
        return max(0.0, self._remaining_fuel)
    def get_thrust_delay_steps(self):
        return self._thrust_delay_steps
    def get_actuation_diagnostics(self):
        return {
            "last_applied_thrust": self._last_applied_thrust,
            "last_applied_torque": self._last_applied_torque,
            "peak_abs_applied_thrust": self._peak_abs_applied_thrust,
            "peak_abs_applied_torque": self._peak_abs_applied_torque,
            "thrust_saturation_steps": self._thrust_saturation_steps,
            "torque_saturation_steps": self._torque_saturation_steps,
            "first_thrust_saturation_step": self._first_thrust_saturation_step,
            "first_torque_saturation_step": self._first_torque_saturation_step,
            "applied_main_impulse": self._applied_main_impulse,
        }
    def get_motion_diagnostics(self):
        return {
            key: dict(value) for key, value in self._motion_peaks.items()
        }
    def get_touchdown_snapshot(self):
        return (
            dict(self._touchdown_snapshot)
            if self._touchdown_snapshot is not None
            else None
        )
    def step(self, time_step):
        if abs(float(time_step) - self._time_step) > 1e-12:
            raise ValueError(
                f"C-02 requires dt={self._time_step}, received {time_step}"
            )
        lander = self._terrain_bodies.get("lander")
        pre_step_state = None
        if lander is not None:
            pre_step_state = {
                "x": float(lander.position.x),
                "y": float(lander.position.y),
                "vx": float(lander.linearVelocity.x),
                "vy": float(lander.linearVelocity.y),
                "angle": float(lander.angle),
                "omega": float(lander.angularVelocity),
            }
            t = self._sim_time
            wind_fx = self._wind_amplitude * (
                math.sin(2.0 * math.pi * t / self._wind_period1) * 0.6
                + math.sin(2.0 * math.pi * t / self._wind_period2) * 0.4
            )
            if self._rng.random() < self._gust_prob:
                wind_fx += (self._rng.random() * 2 - 1) * self._gust_amplitude
            lander.ApplyForceToCenter((wind_fx, 0.0), True)
            if self._thrust_delay_steps == 0:
                thrust_to_use = self._main_thrust
                torque_to_use = self._steering_torque
            else:
                thrust_to_use, torque_to_use = self._thrust_queue.popleft()
                self._thrust_queue.append(
                    (self._main_thrust, self._steering_torque)
                )
            self._main_thrust = 0.0
            self._steering_torque = 0.0
            impulse_cost = abs(thrust_to_use) * time_step
            if self._remaining_fuel <= 0:
                thrust_to_use = 0.0
            elif impulse_cost > self._remaining_fuel:
                scale = self._remaining_fuel / impulse_cost
                thrust_to_use *= scale
                impulse_cost = self._remaining_fuel
            self._remaining_fuel -= impulse_cost
            self._last_applied_thrust = float(thrust_to_use)
            self._last_applied_torque = float(torque_to_use)
            self._applied_main_impulse += float(impulse_cost)
            abs_thrust = abs(float(thrust_to_use))
            abs_torque = abs(float(torque_to_use))
            self._peak_abs_applied_thrust = max(
                self._peak_abs_applied_thrust, abs_thrust
            )
            self._peak_abs_applied_torque = max(
                self._peak_abs_applied_torque, abs_torque
            )
            if abs_thrust >= self._max_thrust - 1e-9:
                self._thrust_saturation_steps += 1
                if self._first_thrust_saturation_step is None:
                    self._first_thrust_saturation_step = self._step_count
            if abs_torque >= self._max_torque - 1e-9:
                self._torque_saturation_steps += 1
                if self._first_torque_saturation_step is None:
                    self._first_torque_saturation_step = self._step_count
            a = float(lander.angle)
            fx = -thrust_to_use * math.sin(a)
            fy = thrust_to_use * math.cos(a)
            mass = float(lander.mass)
            inertia = float(lander.inertia)
            projected_vx = (
                pre_step_state["vx"]
                + (wind_fx + fx) / mass * time_step
            ) / (1.0 + time_step * float(lander.linearDamping))
            projected_vy = (
                pre_step_state["vy"]
                + (float(self._world.gravity.y) + fy / mass) * time_step
            ) / (1.0 + time_step * float(lander.linearDamping))
            projected_omega = (
                pre_step_state["omega"] + torque_to_use / inertia * time_step
            ) / (1.0 + time_step * float(lander.angularDamping))
            pre_step_state.update(
                {
                    "projected_x": pre_step_state["x"] + projected_vx * time_step,
                    "projected_y": pre_step_state["y"] + projected_vy * time_step,
                    "projected_vy": projected_vy,
                    "projected_angle": pre_step_state["angle"]
                    + projected_omega * time_step,
                }
            )
            if thrust_to_use != 0.0:
                lander.ApplyForceToCenter((fx, fy), True)
            if torque_to_use != 0.0:
                lander.ApplyTorque(torque_to_use, True)
        self._world.Step(time_step, 10, 10)
        self._sim_time += time_step
        self._step_count += 1
        if lander is not None:
            observed_motion = {
                "abs_vx": abs(float(lander.linearVelocity.x)),
                "abs_vy": abs(float(lander.linearVelocity.y)),
                "abs_angular_velocity": abs(float(lander.angularVelocity)),
            }
            for key, value in observed_motion.items():
                if math.isfinite(value) and value > self._motion_peaks[key]["value"]:
                    self._motion_peaks[key] = {
                        "value": value,
                        "step": self._step_count,
                    }
            if (
                self._touchdown_snapshot is None
                and self.get_lander_bottom_y()
                <= self._ground_y_top + self._land_tolerance
                and pre_step_state is not None
            ):
                hw, hh = self._lander_half_width, self._lander_half_height
                corners = ((-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh))

                def _transformed(state_x, state_y, state_angle):
                    return [
                        (
                            state_x
                            + math.cos(state_angle) * bx
                            - math.sin(state_angle) * by,
                            state_y
                            + math.sin(state_angle) * bx
                            + math.cos(state_angle) * by,
                        )
                        for bx, by in corners
                    ]

                before = _transformed(
                    pre_step_state["x"],
                    pre_step_state["y"],
                    pre_step_state["angle"],
                )
                projected = _transformed(
                    pre_step_state["projected_x"],
                    pre_step_state["projected_y"],
                    pre_step_state["projected_angle"],
                )
                before_bottom = min(point[1] for point in before)
                projected_bottom = min(point[1] for point in projected)
                threshold = self._ground_y_top + self._land_tolerance
                denominator = before_bottom - projected_bottom
                if denominator > 1e-12:
                    fraction = max(
                        0.0,
                        min(1.0, (before_bottom - threshold) / denominator),
                    )
                else:
                    fraction = 1.0
                x = pre_step_state["x"] + fraction * (
                    pre_step_state["projected_x"] - pre_step_state["x"]
                )
                y = pre_step_state["y"] + fraction * (
                    pre_step_state["projected_y"] - pre_step_state["y"]
                )
                angle = pre_step_state["angle"] + fraction * (
                    pre_step_state["projected_angle"] - pre_step_state["angle"]
                )
                vy = pre_step_state["vy"] + fraction * (
                    pre_step_state["projected_vy"] - pre_step_state["vy"]
                )
                transformed = _transformed(x, y, angle)
                bottom_x = [transformed[0][0], transformed[1][0]]
                self._touchdown_snapshot = {
                    "step": self._step_count,
                    "x": x,
                    "vy": vy,
                    "angle": angle,
                    "x_lo": min(bottom_x),
                    "x_hi": max(bottom_x),
                }
        if self._gravity_mutation:
            at_step = self._gravity_mutation.get("at_step")
            if at_step is not None and self._step_count >= int(at_step):
                gravity_after = tuple(self._gravity_mutation.get("gravity_after", (0, -10)))
                self._world.gravity = gravity_after
                self._gravity_mutation = None
        if lander is not None and not self._barrier_hit:
            x, y = lander.position.x, lander.position.y
            a = lander.angle
            hw, hh = self._lander_half_width, self._lander_half_height
            for bx, by in ((-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)):
                wx = x + math.cos(a) * bx - math.sin(a) * by
                wy = y + math.sin(a) * bx + math.cos(a) * by
                if self._barrier_x_left <= wx <= self._barrier_x_right:
                    if wy < self._barrier_y_top:
                        self._barrier_hit = True
                        self._barrier_failure_kind = "obstacle"
                        break
                    if wy > self._barrier_y_bottom:
                        self._barrier_hit = True
                        self._barrier_failure_kind = "ceiling"
                        break
        if lander is not None:
            lx, ly = lander.position.x, lander.position.y
            in_x_band = self._barrier_x_left <= lx <= self._barrier_x_right
            if in_x_band and not self._corridor_entered:
                self._corridor_entered = True
                self._corridor_entry_step = self._step_count
                self._corridor_entry_x = lx
                self._corridor_entry_y = ly
                self._corridor_min_y = ly
                self._corridor_max_y = ly
            if in_x_band and self._corridor_entered and not self._corridor_exited:
                if ly < self._corridor_min_y:
                    self._corridor_min_y = ly
                if ly > self._corridor_max_y:
                    self._corridor_max_y = ly
            if not in_x_band and self._corridor_entered and not self._corridor_exited:
                self._corridor_exited = True
                self._corridor_exit_step = self._step_count
                self._corridor_exit_x = lx
                self._corridor_exit_y = ly
            if self._barrier_hit and self._corridor_violation_step is None:
                self._corridor_violation_step = self._step_count
                self._corridor_violation_kind = self._barrier_failure_kind
                self._corridor_violation_x = lx
                self._corridor_violation_y = ly
    def get_barrier_hit(self):
        return getattr(self, "_barrier_hit", False)
    def get_barrier_failure_kind(self):
        return getattr(self, "_barrier_failure_kind", None)
    def get_corridor_transit_data(self):
        min_y = self._corridor_min_y
        max_y = self._corridor_max_y
        return {
            "entered": self._corridor_entered,
            "entry_step": self._corridor_entry_step,
            "entry_x": self._corridor_entry_x,
            "entry_y": self._corridor_entry_y,
            "min_y_in_corridor": None if min_y == float('inf') else min_y,
            "max_y_in_corridor": None if max_y == float('-inf') else max_y,
            "exit_step": self._corridor_exit_step,
            "exit_x": self._corridor_exit_x,
            "exit_y": self._corridor_exit_y,
            "violation_step": self._corridor_violation_step,
            "violation_kind": self._corridor_violation_kind,
            "violation_x": self._corridor_violation_x,
            "violation_y": self._corridor_violation_y,
        }
    def get_terrain_bounds(self):
        return {
            "ground_y_top": self._ground_y_top,
            "ground_length": self._ground_length,
            "ground_slab_height": getattr(self, "_ground_slab_height", GROUND_SLAB_HEIGHT),
            "spawn_x": self._spawn_x,
            "spawn_y": self._spawn_y,
            "lander_mass": self._lander_mass,
            "lander_half_width": self._lander_half_width,
            "lander_half_height": self._lander_half_height,
            "max_safe_vertical_speed": self._max_safe_vertical_speed,
            "max_landing_angle": self._max_landing_angle,
            "total_fuel_impulse": self._total_fuel,
            "time_step": self._time_step,
            "thrust_delay_steps": self._thrust_delay_steps,
            "barrier_x_left": self._barrier_x_left,
            "barrier_x_right": self._barrier_x_right,
            "barrier_y_top": self._barrier_y_top,
            "barrier_y_bottom": self._barrier_y_bottom,
            "min_fuel_remaining_at_landing": self._min_fuel_remaining_at_landing,
            "max_episode_steps": self._max_episode_steps,
            "land_tolerance": self._land_tolerance,
            "platform_center_base": self._platform_center_base,
            "platform_amplitude": self._platform_amplitude,
            "platform_period": self._platform_period,
            "platform_half_width": self._platform_half_width,
            "max_thrust": self._max_thrust,
            "max_torque": self._max_torque,
        }
    def get_platform_center_at_time(self, sim_time: float) -> float:
        return self._platform_center_base + self._platform_amplitude * math.sin(
            2.0 * math.pi * sim_time / self._platform_period
        )
    def get_zone_x_bounds_at_step(self, step: int):
        t = step * self._time_step
        center = self.get_platform_center_at_time(t)
        return (center - self._platform_half_width, center + self._platform_half_width)
    def get_lander_body(self):
        return self._terrain_bodies.get("lander")
    def get_ground_y_top(self):
        return self._ground_y_top
    def get_lander_size(self):
        return (self._lander_half_width, self._lander_half_height)
    def get_lander_radius(self):
        return math.sqrt(
            self._lander_half_width ** 2 + self._lander_half_height ** 2
        )
    def get_lander_bottom_y(self):
        lander = self._terrain_bodies.get("lander")
        if lander is None:
            return 0.0
        x, y = lander.position.x, lander.position.y
        a = lander.angle
        hw, hh = self._lander_half_width, self._lander_half_height
        corners = ((-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh))
        min_wy = min(
            y + math.sin(a) * bx + math.cos(a) * by for bx, by in corners
        )
        return min_wy
    def get_lander_bottom_contact_x_span(self):
        lander = self._terrain_bodies.get("lander")
        if lander is None:
            return (0.0, 0.0)
        x, y = lander.position.x, lander.position.y
        a = lander.angle
        hw, hh = self._lander_half_width, self._lander_half_height
        wx_list = []
        for bx, by in ((-hw, -hh), (hw, -hh)):
            wx = x + math.cos(a) * bx - math.sin(a) * by
            wx_list.append(wx)
        return (min(wx_list), max(wx_list))
