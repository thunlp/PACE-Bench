import math

from typing import Dict

import Box2D

from Box2D.b2 import world, polygonShape, staticBody, dynamicBody

class Sandbox:
    SLED_START_X = 8.0
    SLED_START_Y = 2.0
    TARGET_X_MIN = 28.0
    TARGET_X_MAX = 32.0
    TARGET_Y_MIN = 2.2
    TARGET_Y_MAX = 2.8
    DEFAULT_GROUND_FRICTION = 0.02
    DEFAULT_SLED_FRICTION = 0.02
    CHECKPOINT_X_LO = 17.5
    CHECKPOINT_X_HI = 19.0
    CHECKPOINT_Y_LO = 3.8
    CHECKPOINT_Y_HI = 4.5
    CHECKPOINT_B_X_LO = 23.0
    CHECKPOINT_B_X_HI = 24.5
    CHECKPOINT_B_Y_LO = 2.5
    CHECKPOINT_B_Y_HI = 3.2
    THRUST_SCALE_X_LO = 19.5
    THRUST_SCALE_X_HI = 21.0
    THRUST_SCALE_FACTOR = 0.5
    OSCILLATING_FX_X_LO = 21.0
    OSCILLATING_FX_X_HI = 27.0
    OSCILLATING_FX_AMP = 30.0
    OSCILLATING_FX_OMEGA = 0.04
    MOMENTUM_DRAIN_X_LO = 11.0
    MOMENTUM_DRAIN_X_HI = 17.0
    MOMENTUM_DRAIN_FACTOR = 0.85
    REVERSE_THRUST_X_LO = 20.0
    REVERSE_THRUST_X_HI = 25.0
    WIND_ZONE_X_LO = 14.0
    WIND_ZONE_X_HI = 28.0
    WIND_FY_BASE = 20.0
    WIND_FY_AMP = 35.0
    WIND_OMEGA = 0.06
    SPEED_PENALTY_X_LO = 22.0
    SPEED_PENALTY_X_HI = 26.0
    SPEED_PENALTY_THRESHOLD = 4.0
    SPEED_PENALTY_FACTOR = 0.35
    VERT_REVERSE_X_LO = 26.5
    VERT_REVERSE_X_HI = 28.5
    def __init__(self, *, terrain_config=None, physics_config=None):
        terrain_config = terrain_config or {}
        physics_config = physics_config or {}
        self._terrain_config = dict(terrain_config)
        self._physics_config = dict(physics_config)
        gravity = physics_config.get("gravity", (0, -10))
        if callable(gravity):
            gravity = gravity(0.0)
        self._gravity = tuple(gravity)
        self._ground_friction = float(terrain_config.get("ground_friction", self.DEFAULT_GROUND_FRICTION))
        self._sled_friction = float(terrain_config.get("sled_friction", self.DEFAULT_SLED_FRICTION))
        self._linear_damping = float(physics_config.get("linear_damping", 0.0))
        self._angular_damping = float(physics_config.get("angular_damping", 0.0))
        self._momentum_drain_factor = float(physics_config.get("momentum_drain_factor", self.MOMENTUM_DRAIN_FACTOR))
        self._thrust_scale_factor = float(physics_config.get("thrust_scale_factor", self.THRUST_SCALE_FACTOR))
        self._speed_penalty_factor = float(physics_config.get("speed_penalty_factor", self.SPEED_PENALTY_FACTOR))
        self._speed_penalty_threshold = float(physics_config.get("speed_penalty_threshold", self.SPEED_PENALTY_THRESHOLD))
        self._world = world(gravity=self._gravity, doSleep=True)
        self._terrain_bodies = {}
        self._pending_thrust = (0.0, 0.0)
        self._step_count = 0
        self._checkpoint_a_reached = False
        self._checkpoint_b_reached = False
        self._target_reached = False
        self._zone_traversal: Dict[str, dict] = {
            zone_key: {"entered": False, "entry_step": -1,
                       "exited": False, "exit_step": -1,
                       "peak_speed": 0.0, "x_at_entry": 0.0,
                       "y_at_entry": 0.0,
                       "vx_at_entry": 0.0, "vy_at_entry": 0.0,
                       "speed_at_entry": 0.0,
                       "vx_at_exit": 0.0, "vy_at_exit": 0.0,
                       "speed_at_exit": 0.0}
            for zone_key in [
                "momentum_drain",
                "checkpoint_a",
                "thrust_scale",
                "oscillating_wind",
                "speed_penalty",
                "vertical_reverse",
                "checkpoint_b",
                "target_zone",
            ]
        }
        self._peak_systemic_velocity = 0.0
        self._furthest_x = self._sled_start_x if hasattr(self, "_sled_start_x") else self.SLED_START_X
        self._furthest_x_step = 0
        self._closest_objective_distance = {
            "checkpoint_a": {"distance": float("inf"), "step": 0},
            "checkpoint_b": {"distance": float("inf"), "step": 0},
            "target_zone": {"distance": float("inf"), "step": 0},
        }
        self._peak_thrust_magnitude = 0.0
        self._commanded_fx = 0.0
        self._commanded_fy = 0.0
        self._delivered_fx = 0.0
        self._delivered_fy = 0.0
        self._near_running_peak_command_steps = 0
        self._peak_commanded_thrust = 0.0
        self._longest_stuck_duration = 0
        self._longest_stuck_start_step = -1
        self._longest_stuck_x = 0.0
        self._longest_stuck_y = 0.0
        self._consecutive_near_zero_steps = 0
        self._current_stuck_start_step = -1
        self._current_stuck_x = 0.0
        self._current_stuck_y = 0.0
        self.world = self._world
        self.bodies = []
        self.joints = []
        self._sled_start_x = float(terrain_config.get("sled_start_x", self.SLED_START_X))
        self._sled_start_y = float(terrain_config.get("sled_start_y", self.SLED_START_Y))
        self._create_terrain(terrain_config)
        self._create_sled(terrain_config)
        self._furthest_x = self._sled_start_x
        self._update_observations(self._sled_start_x, self._sled_start_y, 0.0, 0.0)

    @staticmethod
    def _distance_to_box(x, y, x_min, x_max, y_min, y_max):
        dx = max(x_min - x, 0.0, x - x_max)
        dy = max(y_min - y, 0.0, y - y_max)
        return math.sqrt(dx * dx + dy * dy)

    def _update_objectives(self, x, y):
        if (
            self.CHECKPOINT_X_LO <= x <= self.CHECKPOINT_X_HI
            and self.CHECKPOINT_Y_LO <= y <= self.CHECKPOINT_Y_HI
        ):
            self._checkpoint_a_reached = True
        if (
            self._checkpoint_a_reached
            and self.CHECKPOINT_B_X_LO <= x <= self.CHECKPOINT_B_X_HI
            and self.CHECKPOINT_B_Y_LO <= y <= self.CHECKPOINT_B_Y_HI
        ):
            self._checkpoint_b_reached = True
        if (
            self._checkpoint_b_reached
            and self.TARGET_X_MIN <= x <= self.TARGET_X_MAX
            and self.TARGET_Y_MIN <= y <= self.TARGET_Y_MAX
        ):
            self._target_reached = True

    def _update_observations(self, x, y, vx, vy):
        if x > self._furthest_x:
            self._furthest_x = float(x)
            self._furthest_x_step = self._step_count
        objective_boxes = {
            "checkpoint_a": (
                self.CHECKPOINT_X_LO,
                self.CHECKPOINT_X_HI,
                self.CHECKPOINT_Y_LO,
                self.CHECKPOINT_Y_HI,
            ),
            "checkpoint_b": (
                self.CHECKPOINT_B_X_LO,
                self.CHECKPOINT_B_X_HI,
                self.CHECKPOINT_B_Y_LO,
                self.CHECKPOINT_B_Y_HI,
            ),
            "target_zone": (
                self.TARGET_X_MIN,
                self.TARGET_X_MAX,
                self.TARGET_Y_MIN,
                self.TARGET_Y_MAX,
            ),
        }
        for key, bounds in objective_boxes.items():
            distance = self._distance_to_box(x, y, *bounds)
            if distance < self._closest_objective_distance[key]["distance"]:
                self._closest_objective_distance[key] = {
                    "distance": float(distance),
                    "step": self._step_count,
                }
        self._track_zone_forensics(float(x), float(y), float(vx), float(vy))
    def _create_terrain(self, terrain_config: dict):
        ground_length = 50.0
        ground_height = 1.0
        ground = self._world.CreateStaticBody(
            position=(ground_length / 2, ground_height / 2),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(ground_length / 2, ground_height / 2)),
                friction=self._ground_friction,
            ),
        )
        self._terrain_bodies["ground"] = ground
        self._ground_y = ground_height
    def _create_sled(self, terrain_config: dict):
        sx, sy = self._sled_start_x, self._sled_start_y
        w, h = 1.0, 0.5
        sled = self._world.CreateDynamicBody(
            position=(sx, sy),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(w / 2, h / 2)),
                density=50.0,
                friction=self._sled_friction,
                restitution=0.1,
            ),
        )
        sled.linearDamping = self._linear_damping
        sled.angularDamping = self._angular_damping
        self._terrain_bodies["sled"] = sled
    def step(self, time_step):
        sled = self._terrain_bodies.get("sled")
        if not sled:
            self._pending_thrust = (0.0, 0.0)
            self._world.Step(time_step, 10, 10)
            self._step_count += 1
            return
        fx, fy = self._pending_thrust
        self._pending_thrust = (0.0, 0.0)
        self._commanded_fx = fx
        self._commanded_fy = fy
        commanded_mag = math.sqrt(fx * fx + fy * fy)
        if commanded_mag > self._peak_commanded_thrust:
            self._peak_commanded_thrust = commanded_mag
        sx, sy = sled.position.x, sled.position.y
        self._update_objectives(sx, sy)
        if self.THRUST_SCALE_X_LO <= sx <= self.THRUST_SCALE_X_HI:
            fx *= self._thrust_scale_factor
            fy *= self._thrust_scale_factor
        if self.REVERSE_THRUST_X_LO <= sx <= self.REVERSE_THRUST_X_HI:
            fx = -fx
        if self.VERT_REVERSE_X_LO <= sx <= self.VERT_REVERSE_X_HI:
            fy = -fy
        if self.WIND_ZONE_X_LO <= sx <= self.WIND_ZONE_X_HI:
            fy += self.WIND_FY_BASE + self.WIND_FY_AMP * math.sin(self._step_count * self.WIND_OMEGA)
        if self.OSCILLATING_FX_X_LO <= sx <= self.OSCILLATING_FX_X_HI:
            fx += self.OSCILLATING_FX_AMP * math.sin(self._step_count * self.OSCILLATING_FX_OMEGA)
        self._delivered_fx = fx
        self._delivered_fy = fy
        delivered_mag = math.sqrt(fx * fx + fy * fy)
        if self._peak_commanded_thrust > 0.01:
            if commanded_mag >= self._peak_commanded_thrust * 0.98:
                self._near_running_peak_command_steps += 1
        sled.ApplyForceToCenter((fx, fy), wake=True)
        self._world.Step(time_step, 10, 10)
        if self.MOMENTUM_DRAIN_X_LO <= sled.position.x <= self.MOMENTUM_DRAIN_X_HI:
            vx, vy = sled.linearVelocity.x, sled.linearVelocity.y
            sled.linearVelocity = (vx * self._momentum_drain_factor, vy * self._momentum_drain_factor)
        if self.SPEED_PENALTY_X_LO <= sled.position.x <= self.SPEED_PENALTY_X_HI:
            vx, vy = sled.linearVelocity.x, sled.linearVelocity.y
            speed = math.sqrt(vx * vx + vy * vy)
            if speed > self._speed_penalty_threshold:
                sled.linearVelocity = (vx * self._speed_penalty_factor, vy * self._speed_penalty_factor)
        self._step_count += 1
        post_x = float(sled.position.x)
        post_y = float(sled.position.y)
        post_vx = sled.linearVelocity.x
        post_vy = sled.linearVelocity.y
        self._update_objectives(post_x, post_y)
        post_speed = math.sqrt(post_vx * post_vx + post_vy * post_vy)
        if post_speed < 0.05:
            if self._consecutive_near_zero_steps == 0:
                self._current_stuck_start_step = self._step_count
                self._current_stuck_x = float(sled.position.x)
                self._current_stuck_y = float(sled.position.y)
            self._consecutive_near_zero_steps += 1
            if self._consecutive_near_zero_steps > self._longest_stuck_duration:
                self._longest_stuck_duration = self._consecutive_near_zero_steps
                self._longest_stuck_start_step = self._current_stuck_start_step
                self._longest_stuck_x = self._current_stuck_x
                self._longest_stuck_y = self._current_stuck_y
        else:
            self._consecutive_near_zero_steps = 0
            self._current_stuck_start_step = -1
        svx = float(sled.linearVelocity.x)
        svy = float(sled.linearVelocity.y)
        self._update_observations(post_x, post_y, svx, svy)
    def _track_zone_forensics(self, sx: float, sy: float, svx: float, svy: float):
        zt = self._zone_traversal
        sc = self._step_count
        speed = math.sqrt(svx * svx + svy * svy)
        def _record_entry(zone_key):
            if not zt[zone_key]["entered"]:
                zt[zone_key]["entered"] = True
                zt[zone_key]["entry_step"] = sc
                zt[zone_key]["x_at_entry"] = sx
                zt[zone_key]["y_at_entry"] = sy
                zt[zone_key]["vx_at_entry"] = svx
                zt[zone_key]["vy_at_entry"] = svy
                zt[zone_key]["speed_at_entry"] = speed
        def _record_exit(zone_key):
            if zt[zone_key]["entered"] and not zt[zone_key]["exited"]:
                zt[zone_key]["exited"] = True
                zt[zone_key]["exit_step"] = sc
                zt[zone_key]["vx_at_exit"] = svx
                zt[zone_key]["vy_at_exit"] = svy
                zt[zone_key]["speed_at_exit"] = speed
        in_md = self.MOMENTUM_DRAIN_X_LO <= sx <= self.MOMENTUM_DRAIN_X_HI
        if in_md:
            _record_entry("momentum_drain")
        else:
            _record_exit("momentum_drain")
        in_ca = (self.CHECKPOINT_X_LO <= sx <= self.CHECKPOINT_X_HI and
                 self.CHECKPOINT_Y_LO <= sy <= self.CHECKPOINT_Y_HI)
        if in_ca:
            _record_entry("checkpoint_a")
        else:
            _record_exit("checkpoint_a")
        in_ts = self.THRUST_SCALE_X_LO <= sx <= self.THRUST_SCALE_X_HI
        if in_ts:
            _record_entry("thrust_scale")
        else:
            _record_exit("thrust_scale")
        in_ow = self.OSCILLATING_FX_X_LO <= sx <= self.OSCILLATING_FX_X_HI
        if in_ow:
            _record_entry("oscillating_wind")
        else:
            _record_exit("oscillating_wind")
        in_sp = self.SPEED_PENALTY_X_LO <= sx <= self.SPEED_PENALTY_X_HI
        if in_sp:
            _record_entry("speed_penalty")
        else:
            _record_exit("speed_penalty")
        in_vr = self.VERT_REVERSE_X_LO <= sx <= self.VERT_REVERSE_X_HI
        if in_vr:
            _record_entry("vertical_reverse")
        else:
            _record_exit("vertical_reverse")
        in_cb = (self.CHECKPOINT_B_X_LO <= sx <= self.CHECKPOINT_B_X_HI and
                 self.CHECKPOINT_B_Y_LO <= sy <= self.CHECKPOINT_B_Y_HI)
        if in_cb:
            _record_entry("checkpoint_b")
        else:
            _record_exit("checkpoint_b")
        in_tz = (self.TARGET_X_MIN <= sx <= self.TARGET_X_MAX and
                 self.TARGET_Y_MIN <= sy <= self.TARGET_Y_MAX)
        if in_tz:
            _record_entry("target_zone")
        else:
            _record_exit("target_zone")
        sled = self._terrain_bodies.get("sled")
        if sled:
            vx, vy = sled.linearVelocity.x, sled.linearVelocity.y
            spd = math.sqrt(vx * vx + vy * vy)
            if spd > self._peak_systemic_velocity:
                self._peak_systemic_velocity = spd
    def get_zone_forensics(self) -> dict:
        zt = self._zone_traversal
        def _zone_dict(key):
            z = zt[key]
            return {
                "entered": z["entered"],
                "entry_step": z["entry_step"],
                "exited": z["exited"],
                "exit_step": z["exit_step"],
                "x_at_entry": z["x_at_entry"],
                "y_at_entry": z["y_at_entry"],
                "vx_at_entry": z["vx_at_entry"],
                "vy_at_entry": z["vy_at_entry"],
                "speed_at_entry": z["speed_at_entry"],
                "vx_at_exit": z["vx_at_exit"],
                "vy_at_exit": z["vy_at_exit"],
                "speed_at_exit": z["speed_at_exit"],
            }
        return {
            "checkpoint_a": _zone_dict("checkpoint_a"),
            "checkpoint_b": _zone_dict("checkpoint_b"),
            "target_zone": _zone_dict("target_zone"),
            "peak_systemic_speed": self._peak_systemic_velocity,
            "peak_thrust_magnitude": self._peak_thrust_magnitude,
            "furthest_x": self._furthest_x,
            "furthest_x_step": self._furthest_x_step,
            "closest_objective_distance": {
                key: dict(value)
                for key, value in self._closest_objective_distance.items()
            },
            "total_steps": self._step_count,
        }
    def apply_thrust(self, fx, fy):
        self._pending_thrust = (float(fx), float(fy))
        thrust_mag = math.sqrt(fx * fx + fy * fy)
        if thrust_mag > self._peak_thrust_magnitude:
            self._peak_thrust_magnitude = thrust_mag
    def get_sled_position(self):
        sled = self._terrain_bodies.get("sled")
        if sled:
            return (sled.position.x, sled.position.y)
        return None
    def get_sled_velocity(self):
        sled = self._terrain_bodies.get("sled")
        if sled:
            return (sled.linearVelocity.x, sled.linearVelocity.y)
        return None
    def get_checkpoint_a_reached(self):
        return getattr(self, "_checkpoint_a_reached", False)
    def get_checkpoint_b_reached(self):
        return getattr(self, "_checkpoint_b_reached", False)
    def get_checkpoint_reached(self):
        return self.get_checkpoint_a_reached() and self.get_checkpoint_b_reached()
    def get_target_reached(self):
        return getattr(self, "_target_reached", False)
    def get_thrust_forensics(self) -> dict:
        return {
            "commanded_fx": self._commanded_fx,
            "commanded_fy": self._commanded_fy,
            "delivered_fx": self._delivered_fx,
            "delivered_fy": self._delivered_fy,
            "commanded_magnitude": math.sqrt(
                self._commanded_fx ** 2 + self._commanded_fy ** 2
            ),
            "delivered_magnitude": math.sqrt(
                self._delivered_fx ** 2 + self._delivered_fy ** 2
            ),
            "peak_commanded_thrust": self._peak_commanded_thrust,
            "near_running_peak_command_steps": self._near_running_peak_command_steps,
            "total_steps": self._step_count,
        }
    def get_stuck_forensics(self) -> dict:
        return {
            "longest_stuck_duration": self._longest_stuck_duration,
            "longest_stuck_start_step": self._longest_stuck_start_step,
            "longest_stuck_x": self._longest_stuck_x,
            "longest_stuck_y": self._longest_stuck_y,
            "still_stuck_at_end": self._consecutive_near_zero_steps > 0,
            "consecutive_stuck_steps_at_end": self._consecutive_near_zero_steps,
        }
    def get_terrain_bounds(self):
        return {
            "ground_y": self._ground_y,
            "sled_start": {"x": self._sled_start_x, "y": self._sled_start_y},
            "target_zone": {
                "x_min": self.TARGET_X_MIN,
                "x_max": self.TARGET_X_MAX,
                "y_min": self.TARGET_Y_MIN,
                "y_max": self.TARGET_Y_MAX,
            },
            "checkpoint_zone": {
                "x_min": self.CHECKPOINT_X_LO,
                "x_max": self.CHECKPOINT_X_HI,
                "y_min": self.CHECKPOINT_Y_LO,
                "y_max": self.CHECKPOINT_Y_HI,
            },
            "checkpoint_b_zone": {
                "x_min": self.CHECKPOINT_B_X_LO,
                "x_max": self.CHECKPOINT_B_X_HI,
                "y_min": self.CHECKPOINT_B_Y_LO,
                "y_max": self.CHECKPOINT_B_Y_HI,
            },
        }
