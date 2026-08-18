import Box2D

from Box2D.b2 import (world, polygonShape, circleShape, staticBody, dynamicBody, revoluteJoint, weldJoint)

import math

from typing import Dict, List

class DaVinciSandbox:
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
        self._load_bodies = []
        self.MIN_BEAM_SIZE = 0.1
        self.MAX_BEAM_SIZE = 15.0
        self.MAX_STRUCTURE_MASS = float(terrain_config.get("max_structure_mass", 15000.0))
        self._load_1_active = False
        self._load_2_active = False
        self._load_attach_time = float(terrain_config.get("load_attach_time", 5.0))
        self._load_2_attach_time = float(terrain_config.get("load_2_attach_time", 15.0))
        self._forbidden_anchor_y = terrain_config.get("forbidden_anchor_y", None)
        self._obstacle_active = terrain_config.get("obstacle_active", False)
        self._obstacle_rects = terrain_config.get("obstacle_rects", [])
        if not self._obstacle_rects and terrain_config.get("obstacle_rect"):
            self._obstacle_rects = [terrain_config.get("obstacle_rect")]
        self._simulation_time = 0.0
        self._joint_force_history: Dict[int, Dict] = {}
        self._first_failure_step: int = -1
        self._first_warning_step: int = -1
        self._joint_failure_records: List[Dict] = []
        self._wall_anchor_positions: List[float] = []
        self._current_anchor_forces: List[float] = []
        self._current_anchor_torques: List[float] = []
        self._current_internal_forces: List[float] = []
        self._current_internal_torques: List[float] = []
        self._joint_observation_error_count: int = 0
        self._last_joint_observation_error: str | None = None
        self.JOINT_WARNING_FRACTION = 0.9
        self._step_count: int = 0
        self.BUILD_ZONE_X_MIN = 0.0
        self.BUILD_ZONE_X_MAX = 50.0
        self.BUILD_ZONE_Y_MIN = -20.0
        self.BUILD_ZONE_Y_MAX = 30.0
        self._setup_terrain()
    @property
    def world(self):
        return self._world
    def _setup_terrain(self):
        wall = self._world.CreateStaticBody(
            position=(-0.5, 10),
            fixtures=Box2D.b2FixtureDef(shape=polygonShape(box=(0.5, 20.0)), friction=0.8),
        )
        self._terrain_bodies["wall"] = wall
        if self._obstacle_active:
            for rect in self._obstacle_rects:
                x_min, y_min, x_max, y_max = rect
                hw, hh = (x_max - x_min) / 2, (y_max - y_min) / 2
                obs = self._world.CreateStaticBody(
                    position=(x_min + hw, y_min + hh),
                    fixtures=Box2D.b2FixtureDef(shape=polygonShape(box=(hw, hh)), friction=0.5),
                )
                self._terrain_bodies[f"obstacle_{len(self._terrain_bodies)}"] = obs
    def add_beam(self, x, y, width, height, angle=0, density=1.0):
        width = max(self.MIN_BEAM_SIZE, min(width, self.MAX_BEAM_SIZE))
        height = max(self.MIN_BEAM_SIZE, min(height, self.MAX_BEAM_SIZE))
        body = self._world.CreateDynamicBody(position=(x, y), angle=angle)
        body.CreatePolygonFixture(box=(width/2, height/2), density=density, friction=0.5)
        self._bodies.append(body)
        return body
    def add_joint(self, body_a, body_b, anchor, type='rigid'):
        if body_b is None: body_b = self._terrain_bodies["wall"]
        anchor_x, anchor_y = anchor
        is_wall_joint = body_a == self._terrain_bodies["wall"] or body_b == self._terrain_bodies["wall"]
        if is_wall_joint:
            if self._forbidden_anchor_y:
                y_min, y_max = self._forbidden_anchor_y
                if y_min <= anchor_y <= y_max:
                    raise ValueError(f"Anchor y={anchor_y} forbidden")
            self._wall_anchor_positions.append(float(anchor_y))
        if type == 'rigid':
            joint_def = Box2D.b2WeldJointDef()
            joint_def.Initialize(body_a, body_b, anchor)
        else:
            joint_def = Box2D.b2RevoluteJointDef()
            joint_def.Initialize(body_a, body_b, anchor)
        j = self._world.CreateJoint(joint_def)
        self._joints.append(j)
        self._joint_force_history[id(j)] = {
            "is_wall": is_wall_joint,
            "anchor_x": anchor_x,
            "anchor_y": anchor_y,
            "peak_force": 0.0,
            "peak_torque": 0.0,
            "failed": False,
            "fail_step": -1,
        }
        return j
    def get_max_reach(self):
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
        self._simulation_time += time_step
        self._step_count += 1
        wind_config = self._physics_config.get("wind", None)
        if wind_config:
            force_vec = wind_config.get("force", (0, 0))
            oscillatory = wind_config.get("oscillatory", False)
            if oscillatory:
                freq = wind_config.get("frequency", 1.0)
                phase = math.sin(self._simulation_time * 2 * math.pi * freq)
                force_vec = (force_vec[0] * phase, force_vec[1] * phase)
            for body in self._bodies:
                body.ApplyForceToCenter(force_vec, True)
        spatial_force = self._physics_config.get("spatial_force", None)
        if spatial_force:
            cx, cy = spatial_force.get("center", (0,0))
            mag = spatial_force.get("magnitude", 0.0)
            radius = spatial_force.get("radius", 10.0)
            is_repulsion = spatial_force.get("type", "repulsion") == "repulsion"
            for body in self._bodies:
                bx, by = body.position
                dx, dy = bx - cx, by - cy
                dist = math.sqrt(dx**2 + dy**2)
                if dist < radius and dist > 0.1:
                    f = mag * (1.0 - dist / radius)
                    if not is_repulsion:
                        f = -f
                    fx = f * (dx / dist)
                    fy = f * (dy / dist)
                    body.ApplyForceToCenter((fx, fy), True)
        if self._simulation_time >= self._load_attach_time and not self._load_1_active:
            self._handle_load(1)
            self._load_1_active = True
        if self._simulation_time >= self._load_2_attach_time and not self._load_2_active:
            self._handle_load(2)
            self._load_2_active = True
        self._world.Step(time_step, 10, 10)
        joints_to_remove = []
        self._current_anchor_forces.clear()
        self._current_anchor_torques.clear()
        self._current_internal_forces.clear()
        self._current_internal_torques.clear()
        for joint in self._joints:
            try:
                is_wall_joint = joint.bodyA == self._terrain_bodies["wall"] or joint.bodyB == self._terrain_bodies["wall"]
                anchor = joint.anchorA if hasattr(joint, 'anchorA') else joint.GetAnchorA()
                if is_wall_joint:
                    max_f = self._terrain_config.get("max_anchor_force", 100000000.0)
                    max_t = self._terrain_config.get("max_anchor_torque", 100000000.0)
                    strength_map = self._terrain_config.get("anchor_strength_map", None)
                    if strength_map:
                        for y_min, y_max, f_mult, t_mult in strength_map:
                            if y_min <= anchor.y <= y_max:
                                max_f *= f_mult
                                max_t *= t_mult
                                break
                else:
                    max_f = self._terrain_config.get("max_internal_force", 100000000.0)
                    max_t = self._terrain_config.get("max_internal_torque", 100000000.0)
                force = joint.GetReactionForce(1.0/time_step)
                torque = abs(joint.GetReactionTorque(1.0/time_step))
                fm = math.sqrt(force.x**2 + force.y**2)
                if is_wall_joint:
                    self._current_anchor_forces.append(fm)
                    self._current_anchor_torques.append(torque)
                else:
                    self._current_internal_forces.append(fm)
                    self._current_internal_torques.append(torque)
                jid = id(joint)
                hist = self._joint_force_history.get(jid)
                if hist is not None:
                    if fm > hist["peak_force"]:
                        hist["peak_force"] = fm
                    if torque > hist["peak_torque"]:
                        hist["peak_torque"] = torque
                    if self._first_warning_step < 0 and (
                        fm > self.JOINT_WARNING_FRACTION * max_f
                        or torque > self.JOINT_WARNING_FRACTION * max_t
                    ):
                        self._first_warning_step = self._step_count
                if fm > max_f or torque > max_t:
                    joints_to_remove.append(joint)
                    if self._first_failure_step < 0:
                        self._first_failure_step = self._step_count
                    if hist is not None:
                        hist["failed"] = True
                        hist["fail_step"] = self._step_count
                        hist["limit_force"] = max_f
                        hist["limit_torque"] = max_t
                    self._joint_failure_records.append({
                        "joint_id": jid,
                        "is_wall": is_wall_joint,
                        "anchor_x": anchor.x,
                        "anchor_y": anchor.y,
                        "peak_force": fm,
                        "peak_torque": torque,
                        "limit_force": max_f,
                        "limit_torque": max_t,
                        "fail_step": self._step_count,
                    })
            except Exception as exc:
                # Reaction telemetry is diagnostic-only, but losing it silently makes
                # the serialized stress report look complete when it is not.
                self._joint_observation_error_count += 1
                self._last_joint_observation_error = f"{type(exc).__name__}: {exc}"
        for j in joints_to_remove:
            self._world.DestroyJoint(j)
            if j in self._joints:
                self._joints.remove(j)
                self._joint_force_history.pop(id(j), None)
    def _handle_load(self, load_id):
        if not self._bodies: return
        load_type = self._terrain_config.get("load_type", "static")
        mass = float(self._terrain_config.get("load_mass", 500.0))
        if load_id == 1:
            anchor_body = max(self._bodies, key=lambda b: b.position.x)
        else:
            anchor_body = sorted(self._bodies, key=lambda b: b.position.x)[len(self._bodies)//2]
        if load_type == "static":
            load_body = self._world.CreateDynamicBody(position=anchor_body.position)
            load_body.CreatePolygonFixture(box=(0.5, 0.5), density=mass, friction=0.5)
            self._world.CreateJoint(Box2D.b2WeldJointDef(bodyA=anchor_body, bodyB=load_body, anchor=anchor_body.position))
            self._load_bodies.append(load_body)
        elif load_type == "dropped":
            y_pos = self._terrain_config.get("drop_height", 10.0)
            load_body = self._world.CreateDynamicBody(position=(anchor_body.position.x, anchor_body.position.y + y_pos))
            load_body.CreatePolygonFixture(box=(0.5, 0.5), density=mass, friction=0.5)
            self._load_bodies.append(load_body)
    def get_terrain_bounds(self):
        return {
            "wall": {"x": [-1.0, 0.0], "y": [-10, 30]},
            "max_beam_width": self.MAX_BEAM_SIZE,
            "max_beam_height": self.MAX_BEAM_SIZE,
            "obstacle_active": self._obstacle_active,
            "obstacle_rects": self._obstacle_rects,
            "forbidden_anchor_y": self._forbidden_anchor_y,
            "build_zone": {"x": [self.BUILD_ZONE_X_MIN, self.BUILD_ZONE_X_MAX], "y": [self.BUILD_ZONE_Y_MIN, self.BUILD_ZONE_Y_MAX]}
        }
