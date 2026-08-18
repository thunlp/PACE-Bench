import math

from collections import deque

import Box2D

from Box2D.b2 import (
    world,
    staticBody,
    dynamicBody,

)

FPS = 60

TIME_STEP = 1.0 / FPS

WORLD_VELOCITY_ITERATIONS = 8

WORLD_POSITION_ITERATIONS = 3

BALANCE_ANGLE_DEG = 45.0

FAILURE_ANGLE_DEG = 90.0

BALANCE_HOLD_STEPS_REQUIRED = 200

BALANCE_LOCK_ANGLE_RAD = math.radians(BALANCE_ANGLE_DEG)

CART_MASS = 10.0

POLE_MASS = 1.0

POLE_LENGTH = 2.0

POLE_WIDTH = 0.2

DEFAULT_POLE_START_ANGLE = 0.0

DEFAULT_SENSOR_DELAY_ANGLE_STEPS = 0

DEFAULT_SENSOR_DELAY_OMEGA_STEPS = 0

TRACK_CENTER_X = 10.0

SAFE_HALF_RANGE = 8.5

CART_RAIL_CENTER_Y = 2.0

MAX_STEPS = 20000

CART_FORCE_LIMIT_NEWTONS = 450.0

DEFAULT_GRAVITY_XY = (0.0, -10.0)

def gravity_from_config(g) -> tuple:
    if isinstance(g, (list, tuple)) and len(g) >= 2:
        return (float(g[0]), float(g[1]))
    return (0.0, -float(g))

class Sandbox:
    def __init__(self, terrain_config=None, physics_config=None, **kwargs):
        if kwargs:
            raise TypeError(f"Sandbox got unexpected keyword arguments: {sorted(kwargs.keys())}")
        self.terrain_config = terrain_config or {}
        self.physics_config = physics_config or {}
        self.world = world(gravity=DEFAULT_GRAVITY_XY, doSleep=True)
        self._terrain_bodies = {}
        self.TRACK_CENTER_X = TRACK_CENTER_X
        self.SAFE_HALF_RANGE = SAFE_HALF_RANGE
        self.MAX_STEPS = MAX_STEPS
        self.cart_rail_center_y = CART_RAIL_CENTER_Y
        self._apply_configs()
        self._create_environment()
        self._step_count = 0
        self._last_applied_force = 0.0
        self._angle_buffer = deque(maxlen=max(1, self._sensor_delay_angle_steps + 1))
        self._omega_buffer = deque(maxlen=max(1, self._sensor_delay_omega_steps + 1))
        self._prime_sensor_delay_buffers()
        self._consecutive_upright_sim_steps = 0
    def _prime_sensor_delay_buffers(self):
        da = self._sensor_delay_angle_steps
        dw = self._sensor_delay_omega_steps
        angle = self.get_true_pole_angle()
        omega = self.get_true_pole_angular_velocity()
        for _ in range(max(0, da) + 1):
            self._angle_buffer.append(angle)
        for _ in range(max(0, dw) + 1):
            self._omega_buffer.append(omega)
    def _apply_configs(self):
        pc = self.physics_config
        if "gravity" in pc:
            self.world.gravity = gravity_from_config(pc["gravity"])
        self.cart_force_limit_newtons = float(pc.get("cart_force_limit_newtons", CART_FORCE_LIMIT_NEWTONS))
        self._initial_angle = pc.get("pole_start_angle", DEFAULT_POLE_START_ANGLE)
        self._cart_mass = pc.get("cart_mass", CART_MASS)
        self._pole_length = pc.get("pole_length", POLE_LENGTH)
        self._pole_mass = pc.get("pole_mass", POLE_MASS)
        self._sensor_delay_angle_steps = pc.get(
            "sensor_delay_angle_steps", DEFAULT_SENSOR_DELAY_ANGLE_STEPS
        )
        self._sensor_delay_omega_steps = pc.get(
            "sensor_delay_omega_steps", DEFAULT_SENSOR_DELAY_OMEGA_STEPS
        )
        self.TRACK_CENTER_X = pc.get("track_center_x", TRACK_CENTER_X)
        self.SAFE_HALF_RANGE = pc.get("safe_half_range", SAFE_HALF_RANGE)
        self.MAX_STEPS = pc.get("max_steps", MAX_STEPS)
        self.cart_rail_center_y = float(pc.get("cart_rail_center_y", CART_RAIL_CENTER_Y))
        self.balance_angle_deg = float(pc.get("balance_angle_deg", BALANCE_ANGLE_DEG))
        self.failure_angle_deg = float(pc.get("failure_angle_deg", FAILURE_ANGLE_DEG))
        self.balance_hold_steps_required = int(
            pc.get("balance_hold_steps_required", BALANCE_HOLD_STEPS_REQUIRED)
        )
        self._balance_lock_angle_rad = math.radians(self.balance_angle_deg)
    def _create_environment(self):
        cart = self.world.CreateDynamicBody(position=(self.TRACK_CENTER_X, self.cart_rail_center_y))
        cart.CreatePolygonFixture(box=(0.5, 0.25), density=self._cart_mass/0.5)
        self._terrain_bodies["cart"] = cart
        ground = self.world.CreateStaticBody(position=(0, 0))
        self.world.CreatePrismaticJoint(
            bodyA=ground,
            bodyB=cart,
            anchor=cart.position,
            axis=(1, 0),
            lowerTranslation=-self.SAFE_HALF_RANGE,
            upperTranslation=self.SAFE_HALF_RANGE,
            enableLimit=True,
        )
        half_pw = POLE_WIDTH / 2.0
        pole_area = POLE_WIDTH * self._pole_length
        cx = self.TRACK_CENTER_X - (self._pole_length / 2) * math.sin(self._initial_angle)
        cy = self.cart_rail_center_y + (self._pole_length / 2) * math.cos(self._initial_angle)
        pole = self.world.CreateDynamicBody(position=(cx, cy), angle=self._initial_angle)
        pole.CreatePolygonFixture(box=(half_pw, self._pole_length / 2), density=self._pole_mass / pole_area)
        self._terrain_bodies["pole"] = pole
        self.world.CreateRevoluteJoint(
            bodyA=cart, bodyB=pole, anchor=(self.TRACK_CENTER_X, self.cart_rail_center_y)
        )
    def step(self, dt):
        if not math.isclose(float(dt), TIME_STEP, rel_tol=0.0, abs_tol=1e-12):
            raise ValueError(
                f"C-01 requires dt={TIME_STEP}, received {dt}"
            )
        self._step_count += 1
        cart = self._terrain_bodies["cart"]
        cart.ApplyForce((self._last_applied_force, 0), cart.position, True)
        self.world.Step(TIME_STEP, WORLD_VELOCITY_ITERATIONS, WORLD_POSITION_ITERATIONS)
        self._angle_buffer.append(self.get_true_pole_angle())
        self._omega_buffer.append(self.get_true_pole_angular_velocity())
        if abs(self.get_true_pole_angle()) <= self._balance_lock_angle_rad:
            self._consecutive_upright_sim_steps += 1
        else:
            self._consecutive_upright_sim_steps = 0
    def get_true_pole_angle(self):
        p = self._terrain_bodies.get("pole")
        return math.atan2(math.sin(p.angle), math.cos(p.angle)) if p else 0.0
    def get_true_pole_angular_velocity(self):
        p = self._terrain_bodies.get("pole")
        return p.angularVelocity if p else 0.0
    def get_pole_angle(self):
        if not self._angle_buffer:
            return self.get_true_pole_angle()
        return self._angle_buffer[0]
    def get_pole_angular_velocity(self):
        if not self._omega_buffer:
            return self._terrain_bodies["pole"].angularVelocity if "pole" in self._terrain_bodies else 0.0
        return self._omega_buffer[0]
    def get_cart_position(self): return self._terrain_bodies["cart"].position.x
    def get_cart_velocity(self): return self._terrain_bodies["cart"].linearVelocity.x
    def apply_cart_force(self, f):
        lim = self.cart_force_limit_newtons
        self._last_applied_force = max(-lim, min(lim, float(f)))
    def get_terrain_bounds(self):
        return {
            "track_center_x": self.TRACK_CENTER_X,
            "safe_half_range": self.SAFE_HALF_RANGE,
            "cart_rail_center_y": self.cart_rail_center_y,
        }
    def get_cart_body(self): return self._terrain_bodies.get("cart")
    def get_pole_body(self): return self._terrain_bodies.get("pole")
    def get_consecutive_upright_sim_steps(self) -> int:
        return int(self._consecutive_upright_sim_steps)
    def get_last_applied_force(self) -> float:
        return float(self._last_applied_force)
    def get_cart_force_limit(self) -> float:
        return float(self.cart_force_limit_newtons)
    def get_pole_mass(self) -> float:
        return float(self._pole_mass)
    def get_cart_mass(self) -> float:
        return float(self._cart_mass)
    def get_pole_length(self) -> float:
        return float(self._pole_length)
    def get_sensor_delay_angle(self) -> int:
        return int(self._sensor_delay_angle_steps)
    def get_sensor_delay_omega(self) -> int:
        return int(self._sensor_delay_omega_steps)
    def get_gravity(self) -> tuple:
        return (float(self.world.gravity[0]), float(self.world.gravity[1]))
