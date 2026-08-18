import math

import random

import Box2D

from Box2D.b2 import (
    world,
    polygonShape,
    circleShape,
    staticBody,
    dynamicBody,

)

DELAY_MIN_STEPS = 2

DELAY_MAX_STEPS = 6

DELAY_CHANGE_INTERVAL_STEPS = 120

BLIND_ZONE_X_MIN = 12.0

BLIND_ZONE_X_MAX = 15.0

SPEED_BLIND_THRESHOLD = 2.0

EVASIVE_DISTANCE = 4.0

EVASIVE_GAIN = 0.45

COOLDOWN_THRESHOLD = 120.0

COOLDOWN_STEPS = 80

COOLDOWN_MAX_THRUST = 40.0

RENDEZVOUS_ZONE_X_MIN = 10.0

RENDEZVOUS_ZONE_X_MAX = 20.0

RENDEZVOUS_DISTANCE_DEFAULT = 6.0

RENDEZVOUS_REL_SPEED_DEFAULT = 1.8

TRACK_DISTANCE_DEFAULT = 8.5

RENDEZVOUS_HEADING_TOLERANCE_DEG_DEFAULT = 55.0

HEADING_REFERENCE_MIN_TARGET_SPEED = 0.15

MAX_ANGULAR_RATE = 0.12

JUMP_INTERVAL_STEPS = 300

JUMP_MAG = 1.2

MAX_THRUST_MAGNITUDE = 200.0

ACTUATION_DELAY_STEPS = 1

TARGET_POSITION_UPDATE_PERIOD = 5

TARGET_SENSOR_VISUAL_RADIUS = 0.25

OBSTACLES = [
    (7.5, 1.5, 0.3, 0.5),
    (14.0, 1.5, 0.3, 0.5),
    (20.5, 1.5, 0.3, 0.5),

]

MOVING_OBSTACLE = (10.5, 1.5, 0.35, 0.55)

MOVING_AMP = 0.7

MOVING_PERIOD = 2.5

MOVING_OBSTACLE_2 = (17.0, 1.5, 0.3, 0.5)

MOVING_AMP_2 = 0.5

MOVING_PERIOD_2 = 3.5

MOVING_PHASE_2 = 0.8

ICE_PATCH_FRICTION_DEFAULT = 0.08

ICE_ZONES = [
    ((9.0, 1.25, 1.0, 0.12), ICE_PATCH_FRICTION_DEFAULT),
    ((16.5, 1.25, 1.0, 0.12), ICE_PATCH_FRICTION_DEFAULT),

]

WIND_ZONE_X = (14.0, 17.0)

WIND_BASE_X = -3.5

WIND_AMP_X = 2.5

WIND_OMEGA = 1.2

IMPULSE_BUDGET = 18500.0

SLOTS_PHASE1 = [(3700, 3800), (4200, 4300), (4700, 4800)]

SLOTS_PHASE2 = [(6200, 6300), (6700, 6800), (7200, 7300)]

ACTIVATION_ZONE_X_MIN = 13.0

ACTIVATION_ZONE_X_MAX = 17.0

ACTIVATION_REQUIRED_STEPS = 80

CORRIDOR_X_BASE_L = 8.0

CORRIDOR_X_BASE_R = 22.0

CORRIDOR_AMP = 2.0

CORRIDOR_OMEGA = 0.4

CORRIDOR_PINCH_MARGIN = 2.0

CORRIDOR_PINCH_OMEGA = 0.35

CORRIDOR_PINCH_PHASE = 32.0

CORRIDOR_PINCH_THRESHOLD = 0.25

DEFAULT_GROUND_Y_TOP = 1.0

CORRIDOR_VIOLATION_TOLERANCE = 0.02

OBSTACLE_PENETRATION_LIMIT = 0.05

class Sandbox:
    def __init__(self, *, terrain_config=None, physics_config=None):
        terrain_config = terrain_config or {}
        physics_config = physics_config or {}
        self._terrain_config = dict(terrain_config)
        self._physics_config = dict(physics_config)
        gravity = tuple(physics_config.get("gravity", (0, -10)))
        self._default_linear_damping = float(physics_config.get("linear_damping", 0.5))
        self._default_angular_damping = float(physics_config.get("angular_damping", 0.5))
        self._world = world(gravity=gravity, doSleep=True)
        self._bodies = []
        self._joints = []
        self._terrain_bodies = {}
        self._obstacle_bodies = []
        self._obstacle_shapes = []
        self._moving_obstacle = None
        self._moving_obstacle_shape = None
        self._moving_obstacle_2 = None
        self._moving_obstacle_2_shape = None
        self._ice_bodies = []
        self._target_history = []
        self._step_count = 0
        self._current_delay_steps = 4
        self._delay_change_counter = 0
        self._last_reported_target = (0.0, 0.0)
        self._target_jump_counter = 0
        self.world = self._world
        self.bodies = self._bodies
        self.joints = self._joints
        self._target_speed = float(terrain_config.get("target_speed", 1.5))
        self._target_change_interval = float(terrain_config.get("target_change_interval", 1.2))
        self._ground_y_top = float(terrain_config.get("ground_y_top", DEFAULT_GROUND_Y_TOP))
        self._seeker_mass = float(terrain_config.get("seeker_mass", 20.0))
        self._seeker_radius = float(terrain_config.get("seeker_radius", 0.35))
        self._spawn_x = float(terrain_config.get("spawn_x", 11.0))
        self._spawn_y = float(terrain_config.get("spawn_y", 1.35))
        rng_seed = terrain_config.get("target_rng_seed", None)
        self._delay_rng = random.Random(rng_seed if rng_seed is not None else 42)
        self._create_ground(terrain_config)
        self._create_obstacles(terrain_config)
        self._create_moving_obstacle(terrain_config)
        self._create_moving_obstacle_2(terrain_config)
        self._create_ice_zones(terrain_config)
        self._create_seeker(terrain_config)
        self._init_target(terrain_config)
        self._thrust_x = 0.0
        self._thrust_y = 0.0
        self._thrust_next_x = 0.0
        self._thrust_next_y = 0.0
        self._sim_time = 0.0
        self._thrust_impulse_used = 0.0
        self._out_of_fuel = False
        self._impulse_budget = float(terrain_config.get("impulse_budget", IMPULSE_BUDGET))
        self._failure_events = []
        self._peak_seeker_speed = 0.0
        self._peak_acceleration = 0.0
        self._prev_seeker_velocity = (0.0, 0.0)
        self._acceleration_samples = []
        self._max_thrust_magnitude = float(terrain_config.get("max_thrust_magnitude", MAX_THRUST_MAGNITUDE))
        self._cooldown_threshold = float(terrain_config.get("cooldown_threshold", COOLDOWN_THRESHOLD))
        self._cooldown_max_thrust = float(terrain_config.get("cooldown_max_thrust", COOLDOWN_MAX_THRUST))
        self._cooldown_steps = int(terrain_config.get("cooldown_steps", COOLDOWN_STEPS))
        self._blind_zone_x_min = float(terrain_config.get("blind_zone_x_min", BLIND_ZONE_X_MIN))
        self._blind_zone_x_max = float(terrain_config.get("blind_zone_x_max", BLIND_ZONE_X_MAX))
        self._speed_blind_threshold = float(
            terrain_config.get("speed_blind_threshold_mps", SPEED_BLIND_THRESHOLD)
        )
        self._rendezvous_zone_x_min = float(
            terrain_config.get("rendezvous_zone_x_min", RENDEZVOUS_ZONE_X_MIN)
        )
        self._rendezvous_zone_x_max = float(
            terrain_config.get("rendezvous_zone_x_max", RENDEZVOUS_ZONE_X_MAX)
        )
        self._activation_zone_x_min = float(
            terrain_config.get("activation_zone_x_min", ACTIVATION_ZONE_X_MIN)
        )
        self._activation_zone_x_max = float(
            terrain_config.get("activation_zone_x_max", ACTIVATION_ZONE_X_MAX)
        )
        self._activation_required_steps = int(
            terrain_config.get("activation_required_steps", ACTIVATION_REQUIRED_STEPS)
        )
        self._corridor_violation = False
        self._cooldown_remaining = 0
        self._activation_achieved = False
        self._activation_consecutive_steps = 0
        self._activation_max_consecutive_steps = 0
        self._activation_achieved_step = None
        self._seeker_heading = 0.0
        self._thrust_next_mag = 0.0
        self._thrust_next_desired_angle = 0.0
        self._thrust_delayed_mag = 0.0
        self._last_applied_thrust_magnitude = 0.0
        self._obstacle_collision = False
        wz = terrain_config.get("wind_zone_x", WIND_ZONE_X)
        if wz is not None and len(wz) >= 2:
            self._wind_zone_x = (float(wz[0]), float(wz[1]))
        else:
            self._wind_zone_x = (float(WIND_ZONE_X[0]), float(WIND_ZONE_X[1]))
        self._wind_base_x = float(terrain_config.get("wind_base_x", WIND_BASE_X))
        self._wind_amp_x = float(terrain_config.get("wind_amp_x", WIND_AMP_X))
        self._wind_omega = float(terrain_config.get("wind_omega", WIND_OMEGA))
    def _create_ground(self, terrain_config: dict):
        ground_len = 30.0
        ground_h = 1.0
        center_y = self._ground_y_top - ground_h / 2
        ground = self._world.CreateStaticBody(
            position=(ground_len / 2, center_y),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(ground_len / 2, ground_h / 2)),
                friction=float(terrain_config.get("ground_friction", 0.4)),
                restitution=0.0,
            ),
        )
        self._terrain_bodies["ground"] = ground
        self._ground_length = ground_len
    def _create_obstacles(self, terrain_config: dict):
        obstacles_cfg = terrain_config.get("obstacles", OBSTACLES)
        for cx, cy, hw, hh in obstacles_cfg:
            obs = self._world.CreateStaticBody(
                position=(cx, cy),
                fixtures=Box2D.b2FixtureDef(
                    shape=polygonShape(box=(hw, hh)),
                    friction=0.5,
                    restitution=0.1,
                ),
            )
            self._obstacle_bodies.append(obs)
            self._obstacle_shapes.append((hw, hh))
    def _create_moving_obstacle(self, terrain_config: dict):
        mob_cfg = terrain_config.get("moving_obstacle", MOVING_OBSTACLE)
        if mob_cfg is None:
            return
        cx, cy, hw, hh = mob_cfg[:4]
        body_def = Box2D.b2BodyDef()
        body_def.type = getattr(Box2D.b2, "kinematicBody", 1)
        body_def.position = (cx, cy)
        mob = self._world.CreateBody(body_def)
        mob.CreateFixture(Box2D.b2FixtureDef(
            shape=polygonShape(box=(hw, hh)),
            friction=0.5,
            restitution=0.1,
        ))
        self._moving_obstacle = mob
        self._moving_obstacle_shape = (hw, hh)
        self._moving_obstacle_center_x = cx
    def _create_moving_obstacle_2(self, terrain_config: dict):
        mob_cfg = terrain_config.get("moving_obstacle_2", MOVING_OBSTACLE_2)
        if mob_cfg is None:
            return
        cx, cy, hw, hh = mob_cfg[:4]
        body_def = Box2D.b2BodyDef()
        body_def.type = getattr(Box2D.b2, "kinematicBody", 1)
        body_def.position = (cx, cy)
        mob = self._world.CreateBody(body_def)
        mob.CreateFixture(Box2D.b2FixtureDef(
            shape=polygonShape(box=(hw, hh)),
            friction=0.5,
            restitution=0.1,
        ))
        self._moving_obstacle_2 = mob
        self._moving_obstacle_2_shape = (hw, hh)
        self._moving_obstacle_2_center_x = cx
    def _create_ice_zones(self, terrain_config: dict):
        ice_cfg = terrain_config.get("ice_zones", ICE_ZONES)
        if not ice_cfg:
            return
        for (cx, cy, hw, hh), friction in ice_cfg:
            ice = self._world.CreateStaticBody(
                position=(cx, cy),
                fixtures=Box2D.b2FixtureDef(
                    shape=polygonShape(box=(hw, hh)),
                    friction=friction,
                    restitution=0.0,
                ),
            )
            self._ice_bodies.append((ice, friction))
    def _create_seeker(self, terrain_config: dict):
        radius = self._seeker_radius
        density = self._seeker_mass / (math.pi * radius * radius)
        _gfr = float(terrain_config.get("ground_friction", 0.4))
        seeker = self._world.CreateDynamicBody(
            position=(self._spawn_x, self._spawn_y),
            fixtures=Box2D.b2FixtureDef(
                shape=circleShape(radius=radius),
                density=density,
                friction=_gfr,
                restitution=0.1,
            ),
        )
        seeker.linearDamping = self._default_linear_damping
        seeker.angularDamping = self._default_angular_damping
        self._terrain_bodies["seeker"] = seeker
    def _init_target(self, terrain_config: dict):
        self._target_x = float(terrain_config.get("target_start_x", 12.0))
        self._target_y = float(terrain_config.get("target_start_y", 2.0))
        self._target_vx = 0.0
        self._target_vy = 0.0
        self._target_change_time = 0.0
        self._target_rng_seed = terrain_config.get("target_rng_seed", None)
        _seed = self._target_rng_seed if self._target_rng_seed is not None else 42
        self._target_rng = random.Random(_seed)
        max_delay = max(DELAY_MAX_STEPS, 10)
        for _ in range(max_delay + 1):
            self._target_history.append((self._target_x, self._target_y))
        self._last_reported_target = (self._target_x, self._target_y)
    def get_seeker_position(self):
        seeker = self._terrain_bodies.get("seeker")
        if seeker is None:
            return (0.0, 0.0)
        return (seeker.position.x, seeker.position.y)
    def get_seeker_velocity(self):
        seeker = self._terrain_bodies.get("seeker")
        if seeker is None:
            return (0.0, 0.0)
        return (seeker.linearVelocity.x, seeker.linearVelocity.y)
    def get_seeker_heading(self):
        return self._seeker_heading
    def get_target_position(self):
        sx, sy = self.get_seeker_position()
        vx, vy = self.get_seeker_velocity()
        seeker_speed = math.sqrt(vx * vx + vy * vy)
        in_blind = self._blind_zone_x_min <= sx <= self._blind_zone_x_max
        speed_blind = seeker_speed > self._speed_blind_threshold
        if in_blind or speed_blind:
            return self._last_reported_target
        if self._step_count % TARGET_POSITION_UPDATE_PERIOD != 0:
            return self._last_reported_target
        delay = min(len(self._target_history) - 1, max(0, self._current_delay_steps))
        if delay >= 0 and self._target_history:
            idx = len(self._target_history) - 1 - delay
            if idx >= 0:
                out = self._target_history[idx]
                self._last_reported_target = out
                return out
        self._last_reported_target = (self._target_x, self._target_y)
        return self._last_reported_target
    def peek_target_position_sensor(self):
        sx, sy = self.get_seeker_position()
        vx, vy = self.get_seeker_velocity()
        seeker_speed = math.sqrt(vx * vx + vy * vy)
        in_blind = self._blind_zone_x_min <= sx <= self._blind_zone_x_max
        speed_blind = seeker_speed > self._speed_blind_threshold
        if in_blind or speed_blind:
            return self._last_reported_target
        if self._step_count % TARGET_POSITION_UPDATE_PERIOD != 0:
            return self._last_reported_target
        delay = min(len(self._target_history) - 1, max(0, self._current_delay_steps))
        if delay >= 0 and self._target_history:
            idx = len(self._target_history) - 1 - delay
            if idx >= 0:
                return self._target_history[idx]
        return (self._target_x, self._target_y)
    def apply_seeker_force(self, force_x, force_y):
        fx, fy = float(force_x), float(force_y)
        if not math.isfinite(fx) or not math.isfinite(fy):
            raise ValueError("Seeker force components must be finite")
        mag = math.sqrt(fx * fx + fy * fy)
        max_mag = self._cooldown_max_thrust if self._cooldown_remaining > 0 else self._max_thrust_magnitude
        if mag > 1e-9:
            self._thrust_next_mag = min(max_mag, mag)
            self._thrust_next_desired_angle = math.atan2(fy, fx)
        else:
            self._thrust_next_mag = 0.0
            self._thrust_next_desired_angle = self._seeker_heading
    def get_terrain_obstacles(self):
        result = [
            (b.position.x, b.position.y, hw, hh)
            for b, (hw, hh) in zip(self._obstacle_bodies, self._obstacle_shapes)
        ]
        if self._moving_obstacle is not None:
            mx, my = self._moving_obstacle.position.x, self._moving_obstacle.position.y
            hw, hh = self._moving_obstacle_shape
            result.append((mx, my, hw, hh))
        if self._moving_obstacle_2 is not None:
            mx, my = self._moving_obstacle_2.position.x, self._moving_obstacle_2.position.y
            hw, hh = self._moving_obstacle_2_shape
            result.append((mx, my, hw, hh))
        return result
    def get_local_wind(self):
        sx, sy = self.get_seeker_position()
        x_min, x_max = self._wind_zone_x
        if x_min <= sx <= x_max:
            wx = self._wind_base_x + self._wind_amp_x * math.sin(self._wind_omega * self._sim_time)
            wy = 0.0
            return (wx, wy)
        return (0.0, 0.0)
    def step(self, time_step):
        time_step = float(time_step)
        if not math.isfinite(time_step) or time_step <= 0:
            raise ValueError(f"C-03 requires a finite positive time step, received {time_step!r}")
        self._step_count += 1
        self._sim_time += time_step
        self._delay_change_counter += 1
        if self._delay_change_counter >= DELAY_CHANGE_INTERVAL_STEPS:
            self._delay_change_counter = 0
            self._current_delay_steps = self._delay_rng.randint(DELAY_MIN_STEPS, DELAY_MAX_STEPS)
        self._target_history.append((self._target_x, self._target_y))
        if len(self._target_history) > DELAY_MAX_STEPS + 20:
            self._target_history.pop(0)
        dist_to_seeker = self.get_distance_to_target()
        if dist_to_seeker < EVASIVE_DISTANCE and dist_to_seeker > 0.01:
            sx, sy = self.get_seeker_position()
            dx = self._target_x - sx
            dy = self._target_y - sy
            inv_d = 1.0 / dist_to_seeker
            ux, uy = dx * inv_d, dy * inv_d
            self._target_vx += ux * EVASIVE_GAIN * time_step
            self._target_vy += uy * EVASIVE_GAIN * time_step
            tv_mag = math.sqrt(self._target_vx**2 + self._target_vy**2)
            if tv_mag > 2.8:
                s = 2.8 / tv_mag
                self._target_vx *= s
                self._target_vy *= s
        self._target_change_time += time_step
        if self._target_change_time >= self._target_change_interval:
            self._target_change_time = 0.0
            angle = self._target_rng.uniform(0, 2 * math.pi)
            speed = self._target_speed
            self._target_vx = speed * math.cos(angle)
            self._target_vy = speed * math.sin(angle)
        self._target_x += self._target_vx * time_step
        self._target_y += self._target_vy * time_step
        self._target_jump_counter += 1
        if self._target_jump_counter >= JUMP_INTERVAL_STEPS:
            self._target_jump_counter = 0
            dx = self._target_rng.uniform(-JUMP_MAG, JUMP_MAG)
            dy = self._target_rng.uniform(-JUMP_MAG, JUMP_MAG)
            self._target_x += dx
            self._target_y += dy
        target_x_min = 6.0
        target_x_max = self._ground_length - 4.0
        self._target_x = max(target_x_min, min(target_x_max, self._target_x))
        target_y_min = self._ground_y_top + 0.5
        target_y_max = self._ground_y_top + 2.0
        self._target_y = max(target_y_min, min(target_y_max, self._target_y))
        if self._moving_obstacle is not None:
            t = self._sim_time
            omega = 2.0 * math.pi / MOVING_PERIOD
            new_x = self._moving_obstacle_center_x + MOVING_AMP * math.sin(omega * t)
            cy = self._moving_obstacle.position.y
            self._moving_obstacle.position = (new_x, cy)
        if self._moving_obstacle_2 is not None:
            t = self._sim_time + MOVING_PHASE_2
            omega = 2.0 * math.pi / MOVING_PERIOD_2
            new_x = self._moving_obstacle_2_center_x + MOVING_AMP_2 * math.sin(omega * t)
            cy = self._moving_obstacle_2.position.y
            self._moving_obstacle_2.position = (new_x, cy)
        seeker = self._terrain_bodies.get("seeker")
        if seeker is not None:
            cooldown_active = self._cooldown_remaining > 0
            wx, wy = self.get_local_wind()
            if wx != 0 or wy != 0:
                wind_force = (self._seeker_mass * wx, self._seeker_mass * wy)
                seeker.ApplyForceToCenter(wind_force, True)
            applied_mag = self._thrust_delayed_mag
            if cooldown_active:
                applied_mag = min(applied_mag, self._cooldown_max_thrust)
            self._last_applied_thrust_magnitude = applied_mag
            self._thrust_x = applied_mag * math.cos(self._seeker_heading)
            self._thrust_y = applied_mag * math.sin(self._seeker_heading)
            cooldown_triggered = False
            if self._thrust_x != 0.0 or self._thrust_y != 0.0:
                seeker.ApplyForceToCenter((self._thrust_x, self._thrust_y), True)
                thrust_mag = math.sqrt(self._thrust_x ** 2 + self._thrust_y ** 2)
                self._thrust_impulse_used += thrust_mag * time_step
                if self._thrust_impulse_used >= self._impulse_budget:
                    was_out = self._out_of_fuel
                    self._out_of_fuel = True
                    if not was_out:
                        seeker_pos = self.get_seeker_position()
                        self._failure_events.append({
                            "step": self._step_count,
                            "type": "fuel_exhausted",
                            "detail": (f"Impulse budget exhausted: used={self._thrust_impulse_used:.1f} N·s "
                                       f"/ budget={self._impulse_budget:.1f} N·s "
                                       f"at seeker x={seeker_pos[0]:.2f}, y={seeker_pos[1]:.2f}"),
                        })
                if thrust_mag > self._cooldown_threshold:
                    self._cooldown_remaining = self._cooldown_steps
                    cooldown_triggered = True
            if cooldown_active and not cooldown_triggered:
                self._cooldown_remaining -= 1
            delta = self._thrust_next_desired_angle - self._seeker_heading
            while delta > math.pi: delta -= 2.0 * math.pi
            while delta < -math.pi: delta += 2.0 * math.pi
            step = max(-MAX_ANGULAR_RATE, min(MAX_ANGULAR_RATE, delta))
            self._seeker_heading += step
            while self._seeker_heading > math.pi: self._seeker_heading -= 2.0 * math.pi
            while self._seeker_heading < -math.pi: self._seeker_heading += 2.0 * math.pi
            self._thrust_delayed_mag = self._thrust_next_mag
        self._world.Step(time_step, 10, 10)
        if seeker is not None:
            cvx, cvy = seeker.linearVelocity.x, seeker.linearVelocity.y
            cspeed = math.sqrt(cvx * cvx + cvy * cvy)
            if cspeed > self._peak_seeker_speed:
                self._peak_seeker_speed = cspeed
            pvx, pvy = self._prev_seeker_velocity
            ax = (cvx - pvx) / max(time_step, 1e-9)
            ay = (cvy - pvy) / max(time_step, 1e-9)
            atotal = math.sqrt(ax * ax + ay * ay)
            if atotal > self._peak_acceleration:
                self._peak_acceleration = atotal
            self._acceleration_samples.append((self._step_count, ax, ay, atotal))
            if len(self._acceleration_samples) > 200:
                self._acceleration_samples.pop(0)
            self._prev_seeker_velocity = (cvx, cvy)
        _penetration_min = OBSTACLE_PENETRATION_LIMIT
        if seeker is not None and not self._obstacle_collision:
            sx, sy = self.get_seeker_position()
            r = self._seeker_radius
            for cx, cy, hw, hh in self.get_terrain_obstacles():
                closest_x = max(cx - hw, min(sx, cx + hw))
                closest_y = max(cy - hh, min(sy, cy + hh))
                dx, dy = sx - closest_x, sy - closest_y
                d_sq = dx * dx + dy * dy
                if d_sq < r * r:
                    dist = math.sqrt(max(0.0, d_sq))
                    if (r - dist) >= _penetration_min:
                        self._obstacle_collision = True
                        self._failure_events.append({
                            "step": self._step_count,
                            "type": "obstacle_collision",
                            "detail": (f"Collision with obstacle at ({cx:.2f}, {cy:.2f}) "
                                       f"[half-size ({hw:.2f}, {hh:.2f})]; "
                                       f"seeker at ({sx:.2f}, {sy:.2f}), "
                                       f"penetration={(r - dist):.3f} m"),
                        })
                        break
        if seeker is not None and not self._corridor_violation:
            sx, _ = self.get_seeker_position()
            x_lo, x_hi = self._corridor_bounds_at_time(self._sim_time)
            if (
                sx < x_lo - CORRIDOR_VIOLATION_TOLERANCE
                or sx > x_hi + CORRIDOR_VIOLATION_TOLERANCE
            ):
                self._corridor_violation = True
                margin_lo = sx - x_lo
                margin_hi = x_hi - sx
                if sx < x_lo:
                    boundary = "left"
                    overflow = x_lo - sx
                else:
                    boundary = "right"
                    overflow = sx - x_hi
                self._failure_events.append({
                    "step": self._step_count,
                    "type": "corridor_violation",
                    "detail": (f"Corridor violation at sim_time={self._sim_time:.3f} s: "
                               f"seeker x={sx:.3f}, bounds=[{x_lo:.3f}, {x_hi:.3f}], "
                               f"violated {boundary} boundary by {overflow:.3f} m "
                               f"(margin_lo={margin_lo:.3f}, margin_hi={margin_hi:.3f})"),
                    "seeker_x": sx,
                    "bound_lo": x_lo,
                    "bound_hi": x_hi,
                    "boundary": boundary,
                    "overflow": overflow,
                })
        if seeker is not None and not self._activation_achieved:
            sx, _ = self.get_seeker_position()
            if self._activation_zone_x_min <= sx <= self._activation_zone_x_max:
                self._activation_consecutive_steps += 1
                self._activation_max_consecutive_steps = max(
                    self._activation_max_consecutive_steps,
                    self._activation_consecutive_steps,
                )
                if self._activation_consecutive_steps >= self._activation_required_steps:
                    self._activation_achieved = True
                    self._activation_achieved_step = self._step_count
                    self._failure_events.append({
                        "step": self._step_count,
                        "type": "activation_achieved",
                        "detail": (f"Activation completed at step {self._step_count}: "
                                   f"{self._activation_consecutive_steps} consecutive steps "
                                   f"in x=[{self._activation_zone_x_min:.1f}, {self._activation_zone_x_max:.1f}]"),
                    })
            else:
                self._activation_consecutive_steps = 0
    def get_gravity(self):
        return (float(self._world.gravity.x), float(self._world.gravity.y))
    def get_linear_damping(self):
        return float(self._default_linear_damping)
    def get_angular_damping(self):
        return float(self._default_angular_damping)
    def get_seeker_mass(self):
        return float(self._seeker_mass)
    def get_ground_friction(self):
        return float(self._terrain_config.get("ground_friction", 0.4))
    def get_seeker_radius(self):
        return float(self._seeker_radius)
    def get_spawn_position(self):
        return (float(self._spawn_x), float(self._spawn_y))
    def get_failure_events(self):
        return list(self._failure_events)
    def get_peak_seeker_speed(self):
        return float(self._peak_seeker_speed)
    def get_peak_acceleration(self):
        return float(self._peak_acceleration)
    def get_sim_time(self):
        return float(self._sim_time)
    def get_step_count(self):
        return int(self._step_count)
    def get_impulse_budget(self):
        return float(self._impulse_budget)
    def get_impulse_used(self):
        return float(self._thrust_impulse_used)
    def get_distance_to_target(self):
        sx, sy = self.get_seeker_position()
        dx = self._target_x - sx
        dy = self._target_y - sy
        return math.sqrt(dx * dx + dy * dy)
    def get_target_position_true(self):
        return (self._target_x, self._target_y)
    def get_target_velocity_true(self):
        return (self._target_vx, self._target_vy)
    def _corridor_bounds_at_time(self, t):
        s = math.sin(CORRIDOR_OMEGA * t)
        x_lo = CORRIDOR_X_BASE_L + CORRIDOR_AMP * s
        x_hi = CORRIDOR_X_BASE_R - CORRIDOR_AMP * s
        pinch = math.sin(CORRIDOR_PINCH_OMEGA * t - CORRIDOR_PINCH_PHASE)
        if pinch > CORRIDOR_PINCH_THRESHOLD:
            x_lo += CORRIDOR_PINCH_MARGIN
            x_hi -= CORRIDOR_PINCH_MARGIN
        return (x_lo, x_hi)
    def get_corridor_bounds(self):
        return self._corridor_bounds_at_time(self._sim_time)
    def get_rendezvous_slots(self):
        p1 = self._terrain_config.get("slots_phase1", SLOTS_PHASE1)
        p2 = self._terrain_config.get("slots_phase2", SLOTS_PHASE2)
        out = []
        for s in p1:
            out.append((int(s[0]), int(s[1])))
        for s in p2:
            out.append((int(s[0]), int(s[1])))
        return out
    def get_remaining_impulse_budget(self):
        return max(0.0, self._impulse_budget - self._thrust_impulse_used)
    def get_out_of_fuel(self):
        return self._out_of_fuel
    def get_corridor_violation(self):
        return self._corridor_violation
    def get_activation_achieved(self):
        return self._activation_achieved
    def get_activation_progress(self):
        return {
            "current_consecutive_steps": int(self._activation_consecutive_steps),
            "max_consecutive_steps": int(self._activation_max_consecutive_steps),
            "achieved_step": self._activation_achieved_step,
        }
    def get_cooldown_remaining_steps(self):
        return int(self._cooldown_remaining)
    def get_last_applied_thrust_magnitude(self):
        return float(self._last_applied_thrust_magnitude)
    def get_obstacle_collision(self):
        return self._obstacle_collision
    def get_terrain_bounds(self):
        out = {
            "ground_y_top": self._ground_y_top,
            "ground_length": self._ground_length,
            "seeker_radius": self._seeker_radius,
            "rendezvous_distance": self._terrain_config.get(
                "rendezvous_distance", RENDEZVOUS_DISTANCE_DEFAULT
            ),
            "rendezvous_rel_speed": self._terrain_config.get(
                "rendezvous_rel_speed", RENDEZVOUS_REL_SPEED_DEFAULT
            ),
            "track_distance": self._terrain_config.get("track_distance", TRACK_DISTANCE_DEFAULT),
            "rendezvous_heading_tolerance_deg": float(
                self._terrain_config.get(
                    "rendezvous_heading_tolerance_deg", RENDEZVOUS_HEADING_TOLERANCE_DEG_DEFAULT
                )
            ),
            "heading_reference_min_target_speed": float(
                self._terrain_config.get(
                    "heading_reference_min_target_speed", HEADING_REFERENCE_MIN_TARGET_SPEED
                )
            ),
            "rendezvous_zone_x_min": self._rendezvous_zone_x_min,
            "rendezvous_zone_x_max": self._rendezvous_zone_x_max,
            "activation_zone_x_min": self._activation_zone_x_min,
            "activation_zone_x_max": self._activation_zone_x_max,
            "activation_required_steps": self._activation_required_steps,
            "cooldown_steps": self._cooldown_steps,
            "impulse_budget": self._impulse_budget,
            "max_thrust_magnitude": self._max_thrust_magnitude,
            "cooldown_threshold": self._cooldown_threshold,
            "cooldown_max_thrust": self._cooldown_max_thrust,
            "blind_zone_x_min": self._blind_zone_x_min,
            "blind_zone_x_max": self._blind_zone_x_max,
            "speed_blind_threshold_mps": self._speed_blind_threshold,
            "target_sensor_visual_radius": TARGET_SENSOR_VISUAL_RADIUS,
            "corridor_violation_tolerance": CORRIDOR_VIOLATION_TOLERANCE,
            "obstacle_penetration_limit": OBSTACLE_PENETRATION_LIMIT,
        }
        if "slots_phase1" in self._terrain_config:
            out["slots_phase1"] = self._terrain_config["slots_phase1"]
        if "slots_phase2" in self._terrain_config:
            out["slots_phase2"] = self._terrain_config["slots_phase2"]
        return out
    def get_seeker_body(self):
        return self._terrain_bodies.get("seeker")
    def get_ground_y_top(self):
        return self._ground_y_top
