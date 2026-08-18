import math

import Box2D

from Box2D.b2 import world, polygonShape, staticBody, dynamicBody, weldJoint, revoluteJoint

def default_gravity_function(t):
    g_y = 10.0 * math.sin(2.0 * math.pi * t / 5.0)
    return (0.0, g_y)

class Sandbox:
    ARENA_X_MIN = 0.0
    ARENA_X_MAX = 40.0
    ARENA_Y_MIN = 0.0
    ARENA_Y_MAX = 20.0
    BUILD_ZONE_X_MIN = 12.0
    BUILD_ZONE_X_MAX = 28.0
    BUILD_ZONE_Y_MIN = 6.0
    BUILD_ZONE_Y_MAX = 18.0
    MAX_STRUCTURE_MASS = 200.0
    MAX_BEAM_COUNT = 12
    TIME_STEP = 1.0 / 60.0
    OBSTACLE1_X_MIN = 18.0
    OBSTACLE1_X_MAX = 22.0
    OBSTACLE1_Y_CENTER = 10.0
    OBSTACLE1_HALF_W = 2.0
    OBSTACLE1_HALF_H = 0.25
    OBSTACLE2_X_MIN = 14.0
    OBSTACLE2_X_MAX = 26.0
    OBSTACLE2_Y_CENTER = 13.0
    OBSTACLE2_HALF_W = 6.0
    OBSTACLE2_HALF_H = 0.25
    OBSTACLE3_X_MIN = 18.5
    OBSTACLE3_X_MAX = 19.5
    OBSTACLE3_Y_CENTER = 14.0
    OBSTACLE3_HALF_W = 0.5
    OBSTACLE3_HALF_H = 0.25
    FORBIDDEN_X_MIN = 19.0
    FORBIDDEN_X_MAX = 20.0
    FORBIDDEN_Y_MIN = 14.5
    FORBIDDEN_Y_MAX = 15.5
    FORBIDDEN2_X_MIN = 18.0
    FORBIDDEN2_X_MAX = 21.0
    FORBIDDEN2_Y_MIN = 15.9
    FORBIDDEN2_Y_MAX = 16.1
    MIN_BEAM_SIZE = 0.1
    MAX_BEAM_SIZE = 5.0
    MAX_STEPS = 2500
    def __init__(self, *, terrain_config=None, physics_config=None):
        terrain_config = terrain_config or {}
        physics_config = physics_config or {}
        self._terrain_config = dict(terrain_config)
        self._physics_config = dict(physics_config)
        if "arena_y_max" in terrain_config:
            self.ARENA_Y_MAX = float(terrain_config["arena_y_max"])
        if "build_zone_y_max" in terrain_config:
            self.BUILD_ZONE_Y_MAX = float(terrain_config["build_zone_y_max"])
        if "max_structure_mass" in physics_config:
            self.MAX_STRUCTURE_MASS = float(physics_config["max_structure_mass"])
        if "max_beam_count" in physics_config:
            self.MAX_BEAM_COUNT = int(physics_config["max_beam_count"])
        gravity_spec = physics_config.get("gravity", default_gravity_function)
        if callable(gravity_spec):
            self._gravity_function = gravity_spec
            self._world = world(gravity=(0, 0), doSleep=True)
        else:
            g = tuple(gravity_spec)
            self._gravity_function = lambda t: g
            self._world = world(gravity=g, doSleep=True)
        self._time = 0.0
        self._default_linear_damping = float(physics_config.get("linear_damping", 0.0))
        self._default_angular_damping = float(physics_config.get("angular_damping", 0.0))
        self._beam_density_scale = float(physics_config.get("beam_density_scale", 1.0))
        self._joint_force_limit = float(physics_config.get("joint_force_limit", float('inf')))
        self._terrain_friction = float(terrain_config.get("friction", 0.6))
        self._joint_tracking = {
            "joint_force_history": [],
            "joint_failure_events": [],
        }
        self._step_count = 0
        self._ke_history = []
        self._peak_body_velocity = 0.0
        self._peak_reaction_force_ever = 0.0
        self._bodies = []
        self._joints = []
        self._terrain_bodies = {}
        self.world = self._world
        self.bodies = self._bodies
        self.joints = self._joints
        self._create_terrain(terrain_config)
        self._create_demonstrator_bodies(terrain_config)
    def _create_terrain(self, terrain_config: dict):
        w = self.ARENA_X_MAX - self.ARENA_X_MIN
        h_half = 0.5
        friction = self._terrain_friction
        floor = self._world.CreateStaticBody(
            position=(self.ARENA_X_MIN + w / 2, h_half / 2),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(w / 2, h_half / 2)),
                friction=friction,
            ),
        )
        self._terrain_bodies["floor"] = floor
        ceiling_y = self.ARENA_Y_MAX
        ceiling = self._world.CreateStaticBody(
            position=(self.ARENA_X_MIN + w / 2, ceiling_y + h_half / 2),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(w / 2, h_half / 2)),
                friction=friction,
            ),
        )
        self._terrain_bodies["ceiling"] = ceiling
        wall_h = self.ARENA_Y_MAX
        left_wall = self._world.CreateStaticBody(
            position=(h_half / 2, wall_h / 2),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(h_half / 2, wall_h / 2)),
                friction=friction,
            ),
        )
        self._terrain_bodies["left_wall"] = left_wall
        right_x = self.ARENA_X_MAX
        right_wall = self._world.CreateStaticBody(
            position=(right_x - h_half / 2, wall_h / 2),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(h_half / 2, wall_h / 2)),
                friction=friction,
            ),
        )
        self._terrain_bodies["right_wall"] = right_wall
        obs1_cx = (self.OBSTACLE1_X_MIN + self.OBSTACLE1_X_MAX) / 2
        obs1_cy = self.OBSTACLE1_Y_CENTER
        obstacle1 = self._world.CreateStaticBody(
            position=(obs1_cx, obs1_cy),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(self.OBSTACLE1_HALF_W, self.OBSTACLE1_HALF_H)),
                friction=friction,
            ),
        )
        self._terrain_bodies["obstacle_1"] = obstacle1
        obs2_cx = (self.OBSTACLE2_X_MIN + self.OBSTACLE2_X_MAX) / 2
        obs2_cy = self.OBSTACLE2_Y_CENTER
        obstacle2 = self._world.CreateStaticBody(
            position=(obs2_cx, obs2_cy),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(self.OBSTACLE2_HALF_W, self.OBSTACLE2_HALF_H)),
                friction=friction,
            ),
        )
        self._terrain_bodies["obstacle_2"] = obstacle2
        obs3_cx = (self.OBSTACLE3_X_MIN + self.OBSTACLE3_X_MAX) / 2
        obs3_cy = self.OBSTACLE3_Y_CENTER
        obstacle3 = self._world.CreateStaticBody(
            position=(obs3_cx, obs3_cy),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(self.OBSTACLE3_HALF_W, self.OBSTACLE3_HALF_H)),
                friction=friction,
            ),
        )
        self._terrain_bodies["obstacle_3"] = obstacle3
    def _create_demonstrator_bodies(self, terrain_config: dict):
        if terrain_config.get("no_demonstrators", False):
            return
        cx = (self.BUILD_ZONE_X_MIN + self.BUILD_ZONE_X_MAX) / 2
        cy = (self.BUILD_ZONE_Y_MIN + self.BUILD_ZONE_Y_MAX) / 2
        for i, (dx, dy) in enumerate([(-3, 0), (0, 0), (3, 0)]):
            body = self._world.CreateDynamicBody(
                position=(cx + dx, cy + dy),
                fixtures=Box2D.b2FixtureDef(
                    shape=polygonShape(box=(0.5, 0.5)),
                    density=2.0,
                    friction=0.4,
                    restitution=0.2,
                ),
            )
            body.linearDamping = self._default_linear_damping
            body.angularDamping = self._default_angular_damping
            self._terrain_bodies[f"demonstrator_{i}"] = body
    def step(self, time_step):
        if not math.isclose(float(time_step), self.TIME_STEP, rel_tol=0.0, abs_tol=1e-12):
            raise ValueError(
                f"E-01 requires time_step={self.TIME_STEP}, received {time_step}"
            )
        self._time += time_step
        gx, gy = self._gravity_function(self._time)
        self._world.gravity = (float(gx), float(gy))
        self._world.Step(time_step, 10, 10)
        self._step_count += 1
        total_ke = 0.0
        for body in self._bodies:
            vx, vy = body.linearVelocity.x, body.linearVelocity.y
            omega = body.angularVelocity
            v_sq = vx * vx + vy * vy
            total_ke += 0.5 * body.mass * v_sq + 0.5 * body.inertia * omega * omega
            self._peak_body_velocity = max(self._peak_body_velocity, math.sqrt(v_sq))
        for key, body in self._terrain_bodies.items():
            if key.startswith("demonstrator_"):
                vx, vy = body.linearVelocity.x, body.linearVelocity.y
                omega = body.angularVelocity
                v_sq = vx * vx + vy * vy
                total_ke += 0.5 * body.mass * v_sq + 0.5 * body.inertia * omega * omega
                self._peak_body_velocity = max(self._peak_body_velocity, math.sqrt(v_sq))
        self._ke_history.append({
            "step": self._step_count,
            "kinetic_energy": total_ke,
        })
        forces_this_step = []
        for j in self._joints:
            force = j.GetReactionForce(1.0 / time_step).length
            forces_this_step.append(force)
            self._peak_reaction_force_ever = max(
                self._peak_reaction_force_ever, force
            )
        if forces_this_step:
            self._joint_tracking["joint_force_history"].append({
                "step": self._step_count,
                "max_force": max(forces_this_step),
                "mean_force": sum(forces_this_step) / len(forces_this_step),
                "joint_count_at_step": len(self._joints),
            })
        if self._joint_force_limit < float('inf'):
            to_break = []
            for j in self._joints:
                force = j.GetReactionForce(1.0 / time_step).length
                if force > self._joint_force_limit:
                    to_break.append(j)
            for j in to_break:
                anchor = j.anchorA
                self._joint_tracking["joint_failure_events"].append({
                    "step": self._step_count,
                    "anchor_x": float(anchor[0]),
                    "anchor_y": float(anchor[1]),
                    "force_at_break": float(j.GetReactionForce(1.0 / time_step).length),
                })
                self._world.DestroyJoint(j)
                if j in self._joints:
                    self._joints.remove(j)
    def add_beam(self, x, y, width, height, angle=0, density=1.0):
        width = max(self.MIN_BEAM_SIZE, min(width, self.MAX_BEAM_SIZE))
        height = max(self.MIN_BEAM_SIZE, min(height, self.MAX_BEAM_SIZE))
        density = density * self._beam_density_scale
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
            if anchor_y < self.ARENA_Y_MAX * 0.25:
                body_b = self._terrain_bodies.get("floor")
            elif anchor_y > self.ARENA_Y_MAX * 0.75:
                body_b = self._terrain_bodies.get("ceiling")
            elif anchor_x < self.ARENA_X_MAX * 0.25:
                body_b = self._terrain_bodies.get("left_wall")
            else:
                body_b = self._terrain_bodies.get("right_wall")
            if body_b is None:
                body_b = self._terrain_bodies.get("floor")
        if type == "rigid":
            joint = self._world.CreateWeldJoint(
                bodyA=body_a, bodyB=body_b,
                anchor=(anchor_x, anchor_y), collideConnected=False,
            )
        elif type == "pivot":
            joint = self._world.CreateRevoluteJoint(
                bodyA=body_a, bodyB=body_b,
                anchor=(anchor_x, anchor_y), collideConnected=False,
            )
        else:
            raise ValueError(f"Unknown joint type: {type}")
        self._joints.append(joint)
        return joint
    def get_structure_mass(self):
        total = 0.0
        for body in self._bodies:
            total += body.mass
        return total
    def set_material_properties(self, body, restitution=0.2):
        for fixture in body.fixtures:
            fixture.restitution = float(restitution)
    def get_arena_bounds(self):
        return (self.ARENA_X_MIN, self.ARENA_X_MAX, self.ARENA_Y_MIN, self.ARENA_Y_MAX)
    def get_build_zone(self):
        return (self.BUILD_ZONE_X_MIN, self.BUILD_ZONE_X_MAX, self.BUILD_ZONE_Y_MIN, self.BUILD_ZONE_Y_MAX)
    def get_terrain_bounds(self):
        return {
            "arena": {
                "x_min": self.ARENA_X_MIN,
                "x_max": self.ARENA_X_MAX,
                "y_min": self.ARENA_Y_MIN,
                "y_max": self.ARENA_Y_MAX,
            },
            "build_zone": {
                "x": [self.BUILD_ZONE_X_MIN, self.BUILD_ZONE_X_MAX],
                "y": [self.BUILD_ZONE_Y_MIN, self.BUILD_ZONE_Y_MAX],
            },
            "max_beam_count": getattr(self, "MAX_BEAM_COUNT", 99),
            "obstacles": [
                {
                    "x_min": self.OBSTACLE1_X_MIN,
                    "x_max": self.OBSTACLE1_X_MAX,
                    "y_min": self.OBSTACLE1_Y_CENTER - self.OBSTACLE1_HALF_H,
                    "y_max": self.OBSTACLE1_Y_CENTER + self.OBSTACLE1_HALF_H,
                },
                {
                    "x_min": self.OBSTACLE2_X_MIN,
                    "x_max": self.OBSTACLE2_X_MAX,
                    "y_min": self.OBSTACLE2_Y_CENTER - self.OBSTACLE2_HALF_H,
                    "y_max": self.OBSTACLE2_Y_CENTER + self.OBSTACLE2_HALF_H,
                },
                {
                    "x_min": self.OBSTACLE3_X_MIN,
                    "x_max": self.OBSTACLE3_X_MAX,
                    "y_min": self.OBSTACLE3_Y_CENTER - self.OBSTACLE3_HALF_H,
                    "y_max": self.OBSTACLE3_Y_CENTER + self.OBSTACLE3_HALF_H,
                },
            ],
            "forbidden_zones": [
                {
                    "x_min": self.FORBIDDEN_X_MIN,
                    "x_max": self.FORBIDDEN_X_MAX,
                    "y_min": self.FORBIDDEN_Y_MIN,
                    "y_max": self.FORBIDDEN_Y_MAX,
                },
                {
                    "x_min": self.FORBIDDEN2_X_MIN,
                    "x_max": self.FORBIDDEN2_X_MAX,
                    "y_min": self.FORBIDDEN2_Y_MIN,
                    "y_max": self.FORBIDDEN2_Y_MAX,
                },
            ],
        }
    def get_joint_force_tracking(self):
        return dict(self._joint_tracking)
    def get_kinetic_energy_history(self):
        return list(self._ke_history)
    def get_peak_body_velocity(self):
        return float(self._peak_body_velocity)
    def get_peak_reaction_force_ever(self):
        return float(self._peak_reaction_force_ever)
