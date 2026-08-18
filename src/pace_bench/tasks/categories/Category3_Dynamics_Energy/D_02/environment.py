import Box2D

from Box2D.b2 import (
    world,
    polygonShape,
    staticBody,
    dynamicBody,
    revoluteJoint,
    weldJoint,

)

class Sandbox:
    def __init__(self, *, terrain_config=None, physics_config=None):
        terrain_config = terrain_config or {}
        physics_config = physics_config or {}
        self._terrain_config = dict(terrain_config)
        self._physics_config = dict(physics_config)
        gravity = tuple(physics_config.get("gravity", (0, -14)))
        self._default_linear_damping = float(physics_config.get("linear_damping", 0.0))
        self._default_angular_damping = float(physics_config.get("angular_damping", 0.0))
        self._wind = tuple(physics_config.get("wind", (0, 0)))
        self._world = world(gravity=gravity, doSleep=True)
        self._bodies = []
        self._joints = []
        self._terrain_bodies = {}
        self.world = self._world
        self.bodies = self._bodies
        self.joints = self._joints
        self._create_terrain(terrain_config)
        self._create_jumper(terrain_config)
        self.BUILD_ZONE_X_MIN = float(terrain_config.get("build_zone_x_min", 1.5))
        self.BUILD_ZONE_X_MAX = float(terrain_config.get("build_zone_x_max", 6.5))
        self.BUILD_ZONE_Y_MIN = float(terrain_config.get("build_zone_y_min", 2.5))
        self.BUILD_ZONE_Y_MAX = float(terrain_config.get("build_zone_y_max", 5.5))
        self.MAX_STRUCTURE_MASS = float(terrain_config.get("max_structure_mass", 180.0))
    def _create_terrain(self, terrain_config: dict):
        platform_height = 1.0
        self._ground_y = platform_height
        left_end_x = float(terrain_config.get("left_platform_end_x", 8.0))
        pit_width = float(terrain_config.get("pit_width", 18.0))
        right_start_x = left_end_x + pit_width
        right_end_x = right_start_x + 15.0
        left_half = left_end_x / 2
        left_platform = self._world.CreateStaticBody(
            position=(left_half, platform_height / 2),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(left_half, platform_height / 2)),
                friction=float(terrain_config.get("left_platform_friction", 0.6)),
                restitution=float(terrain_config.get("left_platform_restitution", 0.0)),
            ),
        )
        self._terrain_bodies["left_platform"] = left_platform
        right_half_w = (right_end_x - right_start_x) / 2
        right_center_x = right_start_x + right_half_w
        right_platform = self._world.CreateStaticBody(
            position=(right_center_x, platform_height / 2),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(right_half_w, platform_height / 2)),
                friction=0.6,
                restitution=0.0,
            ),
        )
        self._terrain_bodies["right_platform"] = right_platform
        self._left_platform_end_x = left_end_x
        self._pit_width = pit_width
        self._right_platform_start_x = right_start_x
        self._pit_bottom_y = 0.0
        CEILING_HALF_H = 0.3
        b1_cx = float(terrain_config.get("slot1_x", 17.0))
        b1_floor = float(terrain_config.get("slot1_floor", 13.2))
        b1_ceil = float(terrain_config.get("slot1_ceil", 14.7))
        b1_half_h = b1_floor / 2.0
        barrier = self._world.CreateStaticBody(
            position=(b1_cx, b1_half_h),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(0.5, b1_half_h)),
                friction=0.6,
                restitution=0.0,
            ),
        )
        self._terrain_bodies["barrier"] = barrier
        ceiling1 = self._world.CreateStaticBody(
            position=(b1_cx, b1_ceil + CEILING_HALF_H),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(0.5, CEILING_HALF_H)),
                friction=0.6,
                restitution=0.0,
            ),
        )
        self._terrain_bodies["ceiling1"] = ceiling1
        self._barrier_x_min, self._barrier_x_max = b1_cx - 0.5, b1_cx + 0.5
        self._barrier_y_max = b1_floor
        self._slot1_floor, self._slot1_ceil = b1_floor, b1_ceil
        b2_cx = float(terrain_config.get("slot2_x", 21.0))
        b2_floor = float(terrain_config.get("slot2_floor", 11.3))
        b2_ceil = float(terrain_config.get("slot2_ceil", 13.3))
        b2_half_h = b2_floor / 2.0
        barrier2 = self._world.CreateStaticBody(
            position=(b2_cx, b2_half_h),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(0.5, b2_half_h)),
                friction=0.6,
                restitution=0.0,
            ),
        )
        self._terrain_bodies["barrier2"] = barrier2
        ceiling2 = self._world.CreateStaticBody(
            position=(b2_cx, b2_ceil + CEILING_HALF_H),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(0.5, CEILING_HALF_H)),
                friction=0.6,
                restitution=0.0,
            ),
        )
        self._terrain_bodies["ceiling2"] = ceiling2
        self._barrier2_x_min, self._barrier2_x_max = b2_cx - 0.5, b2_cx + 0.5
        self._barrier2_y_max = b2_floor
        self._slot2_floor, self._slot2_ceil = b2_floor, b2_ceil
        b3_cx = float(terrain_config.get("slot3_x", 19.0))
        b3_floor = float(terrain_config.get("slot3_floor", 12.4))
        b3_ceil = float(terrain_config.get("slot3_ceil", 14.2))
        b3_half_h = b3_floor / 2.0
        barrier3 = self._world.CreateStaticBody(
            position=(b3_cx, b3_half_h),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(0.5, b3_half_h)),
                friction=0.6,
                restitution=0.0,
            ),
        )
        self._terrain_bodies["barrier3"] = barrier3
        ceiling3 = self._world.CreateStaticBody(
            position=(b3_cx, b3_ceil + CEILING_HALF_H),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(0.5, CEILING_HALF_H)),
                friction=0.6,
                restitution=0.0,
            ),
        )
        self._terrain_bodies["ceiling3"] = ceiling3
        self._barrier3_x_min, self._barrier3_x_max = b3_cx - 0.5, b3_cx + 0.5
        self._barrier3_y_max = b3_floor
        self._slot3_floor, self._slot3_ceil = b3_floor, b3_ceil
    def _create_jumper(self, terrain_config: dict):
        spawn_x = float(terrain_config.get("jumper_spawn_x", 5.0))
        spawn_y = float(terrain_config.get("jumper_spawn_y", 5.0))
        width = float(terrain_config.get("jumper_width", 0.8))
        height = float(terrain_config.get("jumper_height", 0.6))
        density = float(terrain_config.get("jumper_density", 50.0))
        jumper = self._world.CreateDynamicBody(
            position=(spawn_x, spawn_y),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(width / 2, height / 2)),
                density=density,
                friction=0.5,
                restitution=0.1,
            ),
        )
        jumper.linearDamping = self._default_linear_damping
        jumper.angularDamping = self._default_angular_damping
        self._terrain_bodies["jumper"] = jumper
    MIN_BEAM_SIZE = 0.1
    MAX_BEAM_SIZE = 4.0
    def add_beam(self, x, y, width, height, angle=0, density=1.0):
        width = max(self.MIN_BEAM_SIZE, min(width, self.MAX_BEAM_SIZE))
        height = max(self.MIN_BEAM_SIZE, min(height, self.MAX_BEAM_SIZE))
        body = self._world.CreateDynamicBody(
            position=(x, y),
            angle=angle,
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(width / 2, height / 2)),
                density=density,
                friction=0.5,
            ),
        )
        body.linearDamping = self._default_linear_damping
        body.angularDamping = self._default_angular_damping
        self._bodies.append(body)
        return body
    def add_joint(self, body_a, body_b, anchor_point, type="rigid"):
        if body_a is None:
            raise ValueError("add_joint: body_a cannot be None.")
        anchor_x, anchor_y = anchor_point[0], anchor_point[1]
        if body_b is None:
            body_b = self._terrain_bodies.get("left_platform")
            if body_b is None:
                raise ValueError("add_joint: Cannot anchor to platform; left_platform not found.")
        if type == "rigid":
            joint = self._world.CreateWeldJoint(
                bodyA=body_a,
                bodyB=body_b,
                anchor=(anchor_x, anchor_y),
                collideConnected=False,
            )
        elif type == "pivot":
            joint = self._world.CreateRevoluteJoint(
                bodyA=body_a,
                bodyB=body_b,
                anchor=(anchor_x, anchor_y),
                collideConnected=False,
            )
        else:
            raise ValueError(f"Unknown joint type: {type}")
        self._joints.append(joint)
        return joint
    def get_structure_mass(self):
        return sum(b.mass for b in self._bodies)
    def set_material_properties(self, body, restitution=0.2):
        for fixture in body.fixtures:
            fixture.restitution = float(restitution)
    def step(self, time_step):
        if self._wind != (0, 0):
            for body in self._world.bodies:
                if body.type == Box2D.b2_dynamicBody:
                    body.ApplyForceToCenter(
                        (self._wind[0] * body.mass, self._wind[1] * body.mass), True
                    )
        self._world.Step(time_step, 10, 10)
    def get_terrain_bounds(self):
        return {
            "ground_y": self._ground_y,
            "left_platform_end_x": self._left_platform_end_x,
            "pit_width": self._pit_width,
            "right_platform_start_x": self._right_platform_start_x,
            "pit_bottom_y": self._pit_bottom_y,
            "barrier_x_min": getattr(self, "_barrier_x_min", None),
            "barrier_x_max": getattr(self, "_barrier_x_max", None),
            "barrier_y_max": getattr(self, "_barrier_y_max", None),
            "barrier2_x_min": getattr(self, "_barrier2_x_min", None),
            "barrier2_x_max": getattr(self, "_barrier2_x_max", None),
            "barrier2_y_max": getattr(self, "_barrier2_y_max", None),
            "barrier3_x_min": getattr(self, "_barrier3_x_min", None),
            "barrier3_x_max": getattr(self, "_barrier3_x_max", None),
            "barrier3_y_max": getattr(self, "_barrier3_y_max", None),
            "slots": [
                (getattr(self, "_barrier_x_min"), getattr(self, "_barrier_x_max"), getattr(self, "_slot1_floor", None), getattr(self, "_slot1_ceil", None)),
                (getattr(self, "_barrier2_x_min"), getattr(self, "_barrier2_x_max"), getattr(self, "_slot2_floor", None), getattr(self, "_slot2_ceil", None)),
                (getattr(self, "_barrier3_x_min"), getattr(self, "_barrier3_x_max"), getattr(self, "_slot3_floor", None), getattr(self, "_slot3_ceil", None)),
            ],
            "build_zone": {
                "x": [self.BUILD_ZONE_X_MIN, self.BUILD_ZONE_X_MAX],
                "y": [self.BUILD_ZONE_Y_MIN, self.BUILD_ZONE_Y_MAX],
            },
            "jumper_spawn": (
                self._terrain_config.get("jumper_spawn_x", 5.0),
                self._terrain_config.get("jumper_spawn_y", 5.0),
            ),
            "jumper_width": float(self._terrain_config.get("jumper_width", 0.8)),
            "jumper_height": float(self._terrain_config.get("jumper_height", 0.6)),
            "landing_min_y": float(self._terrain_config.get("landing_min_y", 1.0)),
        }
    def get_jumper_position(self):
        j = self._terrain_bodies.get("jumper")
        if j is None:
            return None
        return (j.position.x, j.position.y)
    def get_body_position(self):
        return self.get_jumper_position()
    def get_jumper_velocity(self):
        j = self._terrain_bodies.get("jumper")
        if j is None:
            return None
        return (j.linearVelocity.x, j.linearVelocity.y)
    def get_jumper(self):
        return self._terrain_bodies.get("jumper")
    def set_jumper_velocity(self, vx, vy):
        j = self._terrain_bodies.get("jumper")
        if j is not None:
            j.linearVelocity = (float(vx), float(vy))
    def get_gravity(self):
        g = self._world.gravity
        return (float(g.x), float(g.y))
    def get_wind(self):
        return (float(self._wind[0]), float(self._wind[1]))
    def get_linear_damping(self):
        return float(self._default_linear_damping)
    def get_angular_damping(self):
        return float(self._default_angular_damping)
    def get_jumper_mass(self):
        j = self._terrain_bodies.get("jumper")
        if j is None:
            return 0.0
        return float(j.mass)
