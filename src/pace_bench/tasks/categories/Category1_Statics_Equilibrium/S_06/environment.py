import Box2D

from Box2D.b2 import (world, polygonShape, staticBody, dynamicBody, kinematicBody)

import math

class DaVinciSandbox:
    MAX_BLOCK_LENGTH = 1.0
    MAX_BLOCK_HEIGHT = 0.2
    MAX_BLOCK_COUNT = 100
    def __init__(self, terrain_config=None, physics_config=None):
        terrain_config = terrain_config or {}
        physics_config = physics_config or {}
        self._terrain_config = dict(terrain_config)
        self._physics_config = dict(physics_config)
        gravity = tuple(physics_config.get("gravity", (0, -10)))
        self._world = world(gravity=gravity, doSleep=True)
        self._bodies = []
        self._joints = []
        self._terrain_bodies = {}
        self.world = self._world
        self.bodies = self._bodies
        self.joints = self._joints
        self._world.linearDamping = physics_config.get("linear_damping", 0.1)
        self._world.angularDamping = physics_config.get("angular_damping", 0.1)
        self._oscillate = terrain_config.get("oscillate", False)
        self._osc_amplitude = terrain_config.get("osc_amplitude", 0.2)
        self._osc_frequency = terrain_config.get("osc_frequency", 5.0)
        self._timer = 0.0
        self._table_angle = terrain_config.get("table_angle", 0.0)
        wf = physics_config.get("wind_force", 0.0)
        if isinstance(wf, (tuple, list)) and len(wf) >= 2:
            self._wind_force = (float(wf[0]), float(wf[1]))
        else:
            self._wind_force = (float(wf), 0.0)
        self._create_terrain(terrain_config)
    def _create_terrain(self, terrain_config: dict):
        floor_length = terrain_config.get("floor_length", 20.0)
        floor_height = 1.0
        table_friction = terrain_config.get("table_friction", 0.8)
        angle_rad = math.radians(self._table_angle)
        pos = (-10.0, -0.5)
        if self._oscillate:
            table = self._world.CreateKinematicBody(
                position=pos,
                angle=angle_rad,
                fixtures=Box2D.b2FixtureDef(
                    shape=polygonShape(box=(floor_length / 2, floor_height / 2)),
                    friction=table_friction,
                ),
            )
        else:
            table = self._world.CreateStaticBody(
                position=pos,
                angle=angle_rad,
                fixtures=Box2D.b2FixtureDef(
                    shape=polygonShape(box=(floor_length / 2, floor_height / 2)),
                    friction=table_friction,
                ),
            )
        self._terrain_bodies["table"] = table
        cy = terrain_config.get("ceiling_y", 100.0)
        ceiling = self._world.CreateStaticBody(
            position=(0, cy + 0.5),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(20.0, 0.5)),
                friction=0.2,
            ),
        )
        self._terrain_bodies["ceiling"] = ceiling
    def add_block(self, x, y, width, height, angle=0, density=None):
        if density is None:
            density = self._terrain_config.get("block_density", 1.0)
        block_friction = self._terrain_config.get("block_friction", 0.6)
        body = self._world.CreateDynamicBody(position=(x, y), angle=angle)
        body.CreatePolygonFixture(box=(width/2, height/2), density=density, friction=block_friction)
        self._bodies.append(body)
        return body
    def get_max_x_position(self):
        if not self._bodies: return 0.0
        max_x = -1e9
        for body in self._bodies:
            for fixture in body.fixtures:
                shape = fixture.shape
                if isinstance(shape, polygonShape):
                    for v in shape.vertices:
                        wv = body.GetWorldPoint(v)
                        max_x = max(max_x, wv.x)
        return max(0.0, max_x)
    def get_structure_mass(self):
        return sum(b.mass for b in self._bodies)
    def step(self, time_step):
        if self._oscillate:
            self._timer += time_step
            vx = self._osc_amplitude * self._osc_frequency * math.cos(self._osc_frequency * self._timer)
            self._terrain_bodies["table"].linearVelocity = (vx, 0)
        if self._wind_force[0] != 0 or self._wind_force[1] != 0:
            for body in self._bodies:
                body.ApplyForceToCenter(self._wind_force, True)
        self._world.Step(time_step, 10, 10)
    def get_body_kinetic_energy(self, body):
        vx, vy = body.linearVelocity
        v_sq = vx * vx + vy * vy
        ang = body.angularVelocity
        ang_sq = ang * ang
        inertia = 0.0
        for fixture in body.fixtures:
            inertia += fixture.body.mass * fixture.shape.I
        return 0.5 * body.mass * v_sq + 0.5 * inertia * ang_sq
    def get_block_y_levels(self):
        if not self._bodies:
            return []
        levels = sorted(set(round(b.position.y, 3) for b in self._bodies))
        return levels
    def detect_same_height_overlaps(self):
        if len(self._bodies) < 2:
            return 0, []
        from collections import defaultdict
        y_groups = defaultdict(list)
        for b in self._bodies:
            y_groups[round(b.position.y, 3)].append(b)
        overlaps = 0
        overlap_details = []
        for y_lvl, bodies in sorted(y_groups.items()):
            if len(bodies) < 2:
                continue
            x_ranges = []
            for b in bodies:
                x_min = float('inf')
                x_max = float('-inf')
                for fixture in b.fixtures:
                    if isinstance(fixture.shape, polygonShape):
                        for v in fixture.shape.vertices:
                            wx = b.GetWorldPoint(v).x
                            x_min = min(x_min, wx)
                            x_max = max(x_max, wx)
                x_ranges.append((x_min, x_max, id(b)))
            for i in range(len(x_ranges)):
                for j in range(i + 1, len(x_ranges)):
                    a_min, a_max = x_ranges[i][0], x_ranges[i][1]
                    b_min, b_max = x_ranges[j][0], x_ranges[j][1]
                    if a_min < b_max and b_min < a_max:
                        overlaps += 1
                        overlap_details.append((y_lvl, (a_min, a_max), (b_min, b_max)))
        return overlaps, overlap_details
    def get_com_to_edge_margin(self, edge_x):
        if not self._bodies:
            return None
        total_mass = sum(b.mass for b in self._bodies)
        if total_mass == 0:
            return None
        com_x = sum(b.position.x * b.mass for b in self._bodies) / total_mass
        return float(com_x) - float(edge_x)
    def get_terrain_bounds(self):
        bounds = {
            "table": {"x": [-20.0, 0.0], "angle": self._table_angle},
            "edge_x": 0.0,
            "max_block_length": self.MAX_BLOCK_LENGTH,
            "max_block_height": self.MAX_BLOCK_HEIGHT,
            "max_block_count": self.MAX_BLOCK_COUNT,
            "spawn_zone": self._terrain_config.get("spawn_zone", [-10.0, 0.0]),
            "ceiling_y": self._terrain_config.get("ceiling_y", 100.0),
            "max_total_mass": self._terrain_config.get("max_total_mass", 20000.0),
            "stability_time": self._terrain_config.get("stability_time", 10.0),
            "target_overhang": self._terrain_config.get("target_overhang", 0.1),
        }
        return bounds
    def get_wind_force(self):
        return (float(self._wind_force[0]), float(self._wind_force[1]))
    def get_gravity(self):
        g = self._world.gravity
        return (float(g[0]), float(g[1]))
    def get_oscillation_params(self):
        return (bool(self._oscillate), float(self._osc_amplitude), float(self._osc_frequency))
    def get_table_friction(self):
        return float(self._terrain_config.get("table_friction", 0.8))
    def get_block_friction(self):
        return float(self._terrain_config.get("block_friction", 0.6))
    def get_block_density_default(self):
        return float(self._terrain_config.get("block_density", 1.0))
    def get_table_angle(self):
        return float(self._table_angle)
    def get_spawn_zone(self):
        return list(self._terrain_config.get("spawn_zone", [-10.0, 0.0]))
    def get_floor_length(self):
        return float(self._terrain_config.get("floor_length", 20.0))
    def get_ceiling_y(self):
        return float(self._terrain_config.get("ceiling_y", 100.0))
    def get_per_block_extents(self):
        extents = []
        for b in self._bodies:
            x_min = float('inf')
            x_max = float('-inf')
            y_min = float('inf')
            y_max = float('-inf')
            for fixture in b.fixtures:
                if isinstance(fixture.shape, polygonShape):
                    for v in fixture.shape.vertices:
                        wv = b.GetWorldPoint(v)
                        x_min = min(x_min, wv.x)
                        x_max = max(x_max, wv.x)
                        y_min = min(y_min, wv.y)
                        y_max = max(y_max, wv.y)
            extents.append({
                "id": id(b),
                "x_min": float(x_min),
                "x_max": float(x_max),
                "y_min": float(y_min),
                "y_max": float(y_max),
                "mass": float(b.mass),
                "vx": float(b.linearVelocity[0]),
                "vy": float(b.linearVelocity[1]),
            })
        return extents
    def get_table_velocity(self):
        if 'table' in self._terrain_bodies:
            v = self._terrain_bodies['table'].linearVelocity
            return (float(v[0]), float(v[1]))
        return (0.0, 0.0)
