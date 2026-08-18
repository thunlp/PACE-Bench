import Box2D

from Box2D.b2 import (world, polygonShape, circleShape, staticBody, dynamicBody, revoluteJoint, weldJoint)

import math

class DaVinciSandbox:
    MIN_BEAM_SIZE = 0.1
    MAX_BEAM_WIDTH = 7.0
    MAX_BEAM_HEIGHT = 2.0
    PIVOT_POSITION = (0.0, 5.0)
    LOAD_POSITION = (3.0, 5.5)
    LOAD_MASS = 200.0
    MAX_ANGLE_DEVIATION = 10.0 * math.pi / 180.0
    BALANCE_TIME = 15.0
    GROUND_Y_FAILURE = -5.0
    def __init__(self, *, terrain_config=None, physics_config=None):
        terrain_config = terrain_config or {}
        physics_config = physics_config or {}
        self._terrain_config = dict(terrain_config)
        self._physics_config = dict(physics_config)
        gravity = tuple(physics_config.get("gravity", (0, -10)))
        self._default_linear_damping = float(physics_config.get("linear_damping", 0.0))
        self._default_angular_damping = float(physics_config.get("angular_damping", 0.0))
        self._default_friction = float(physics_config.get("friction", 0.5))
        self._default_restitution = float(physics_config.get("restitution", 0.0))
        self._world = world(gravity=gravity, doSleep=True)
        self._bodies = []
        self._joints = []
        self._terrain_bodies = {}
        self._obstacles = []
        self.MAX_ANGLE_DEVIATION = terrain_config.get("max_angle_deviation_deg", 10.0) * math.pi / 180.0
        self.BALANCE_TIME = float(terrain_config.get("balance_time", 15.0))
        self.GROUND_Y_FAILURE = float(terrain_config.get("ground_y_failure", -5.0))
        self.MAX_ANGULAR_VELOCITY = float(terrain_config.get("max_angular_velocity", 2.0))
        self.world = self._world
        self.bodies = self._bodies
        self.joints = self._joints
        self._obstacle_active = terrain_config.get("obstacle_active", False)
        self._drop_load = terrain_config.get("drop_load", False)
        self._wind_active = terrain_config.get("wind_active", False)
        self._wind_force_multiplier = float(terrain_config.get("wind_force_multiplier", 5.0))
        self._moving_obstacle = terrain_config.get("moving_obstacle", False)
        self._obstacle_amplitude = terrain_config.get("obstacle_amplitude", 2.0)
        self._obstacle_frequency = terrain_config.get("obstacle_frequency", 0.5)
        self._step_timer = 0.0
        self._last_time_step = 1.0 / 60.0
        self._fragile_joints = terrain_config.get("fragile_joints", False)
        self._max_joint_torque = float(terrain_config.get("max_joint_torque", 1000.0))
        self._pivot_joint_destroyed = False
        self._pivot_destroyed_step = None
        self._pivot_destroy_error_count = 0
        self._last_pivot_destroy_error = None
        self._peak_load_abs_x = 0.0
        self._min_load_structure_dist = float('inf')
        self._event_timeline = []
        self._peak_angular_velocity = 0.0
        self._body_identities = {}
        self._body_label_counter = {"beam": 0, "body": 0}
        self._initial_body_count_on_load_attach = 0
        self.PIVOT_X = 0.0
        self.PIVOT_Y = 5.0
        self._create_terrain(terrain_config)
        self.world.CreateStaticBody(
            position=(0, -50),
            shapes=polygonShape(box=(100, 1)),
        )
        self._setup_load(terrain_config)
        self._obstacle_world_rects = []
        for obs in self._obstacles:
            bb = obs.fixtures[0].shape
            hw = getattr(bb, 'halfWidth', 0.5)
            hh = getattr(bb, 'halfHeight', 0.5)
            cx, cy = obs.position.x, obs.position.y
            self._obstacle_world_rects.append({
                'xmin': cx - hw, 'ymin': cy - hh,
                'xmax': cx + hw, 'ymax': cy + hh,
            })
    def _create_terrain(self, terrain_config: dict):
        pivot_shape = terrain_config.get("pivot_shape", "sharp")
        pivot_friction = float(terrain_config.get("pivot_friction", self._default_friction * 1.6))
        if pivot_shape == "rounded":
            pivot = self._world.CreateStaticBody(
                position=(self.PIVOT_X, self.PIVOT_Y),
                fixtures=Box2D.b2FixtureDef(shape=circleShape(radius=0.05), friction=pivot_friction),
            )
        else:
            pivot = self._world.CreateStaticBody(
                position=(self.PIVOT_X, self.PIVOT_Y),
                fixtures=Box2D.b2FixtureDef(shape=polygonShape(vertices=[(0, 0.05), (-0.05, 0), (0.05, 0)]), friction=pivot_friction),
            )
        self._terrain_bodies["pivot"] = pivot
        if self._obstacle_active:
            if "obstacles" in terrain_config:
                rects = terrain_config["obstacles"]
            else:
                rects = [terrain_config.get("obstacle_rect", [-2.5, -0.1, -1.5, 1.5])]
            for i, rect in enumerate(rects):
                xmin, ymin, xmax, ymax = rect
                ymin += self.PIVOT_Y
                ymax += self.PIVOT_Y
                cx, cy = (xmin + xmax) / 2.0, (ymin + ymax) / 2.0
                hw, hh = (xmax - xmin) / 2.0, (ymax - ymin) / 2.0
                obs = self._world.CreateStaticBody(
                    position=(cx, cy),
                    fixtures=Box2D.b2FixtureDef(shape=polygonShape(box=(hw, hh)), friction=0.5),
                )
                self._terrain_bodies[f"obstacle_{i}"] = obs
                self._obstacles.append(obs)
    def _setup_load(self, terrain_config: dict):
        self._load_mass = float(terrain_config.get("load_mass", 200.0))
        self._load_position = (3.0, self.PIVOT_Y + 0.5)
        self._load_body = None
        self._load_attached = False
        self._initial_disturbance_applied = False
        self._initial_disturbance = terrain_config.get("initial_disturbance", None)
        if self._drop_load:
            self._load_position = (3.0, self.PIVOT_Y + 4.0)
            self._load_body = self._world.CreateDynamicBody(
                position=self._load_position,
                fixtures=Box2D.b2FixtureDef(
                    shape=polygonShape(box=(0.5, 0.5)),
                    density=self._load_mass / (1.0 * 1.0),
                    friction=0.8,
                    restitution=0.1
                )
            )
            self._load_attached = True
            self._load_caught_by_structure = False
            self._terrain_bodies["load"] = self._load_body
    def step(self, time_step):
        self._last_time_step = time_step
        if self._load_body is not None:
            lx = abs(float(self._load_body.position.x))
            if lx > self._peak_load_abs_x:
                self._peak_load_abs_x = lx
        if not self._initial_disturbance_applied:
            for body in self._bodies:
                body.angularVelocity = 0
                body.linearVelocity = (0, 0)
                body.angle = 0
            if self._load_body and not self._drop_load:
                self._load_body.angularVelocity = 0
                self._load_body.linearVelocity = (0, 0)
                self._load_body.angle = 0
            self._initial_disturbance_applied = True
        if not self._load_attached and not self._drop_load and self._bodies:
            target_y = self.PIVOT_Y + 0.5
            for body in self._bodies:
                dist = math.sqrt((body.position.x - 3.0)**2 + (body.position.y - target_y)**2)
                if dist < 0.5:
                    self._load_body = self._world.CreateDynamicBody(
                        position=(3.0, target_y),
                        fixtures=Box2D.b2FixtureDef(
                            shape=polygonShape(box=(0.5, 0.5)),
                            density=self._load_mass / (1.0 * 1.0),
                        )
                    )
                    self._world.CreateWeldJoint(
                        bodyA=body,
                        bodyB=self._load_body,
                        anchor=(3.0, self.PIVOT_Y),
                        collideConnected=False
                    )
                    self._load_attached = True
                    self._terrain_bodies["load"] = self._load_body
                    self._initial_body_count_on_load_attach = len(self._bodies)
                    current_sn = int(self._step_timer / time_step) if time_step > 0 else 0
                    self._event_timeline.append({
                        "step": current_sn,
                        "type": "load_attached",
                        "load_pos": (float(self._load_body.position.x), float(self._load_body.position.y)),
                        "key": "load_attached",
                    })
                    break
        if self._wind_active:
            for body in self._bodies:
                body.ApplyForceToCenter((body.mass * self._wind_force_multiplier, 0), wake=True)
            if self._load_body:
                self._load_body.ApplyForceToCenter((self._load_body.mass * self._wind_force_multiplier, 0), wake=True)
        if self._fragile_joints and self._load_attached:
            net_torque = 0.0
            gx, gy = self._world.gravity
            wind_f = self._wind_force_multiplier if self._wind_active else 0.0
            for i, b in enumerate(self._bodies):
                rx, ry = b.position.x - self.PIVOT_X, b.position.y - self.PIVOT_Y
                Fx = b.mass * wind_f + b.mass * gx
                Fy = b.mass * gy
                tq = rx * Fy - ry * Fx
                net_torque += tq
            if self._load_attached and self._load_body:
                b = self._load_body
                rx, ry = b.position.x - self.PIVOT_X, b.position.y - self.PIVOT_Y
                Fx = b.mass * wind_f + b.mass * gx
                Fy = b.mass * gy
                tq = rx * Fy - ry * Fx
                net_torque += tq
            if abs(net_torque) > self._max_joint_torque:
                pivot = self._terrain_bodies.get("pivot")
                for j in list(self._joints):
                    if (isinstance(j, Box2D.b2RevoluteJoint)) and (j.bodyA == pivot or j.bodyB == pivot):
                        try:
                            self._world.DestroyJoint(j)
                            self._joints.remove(j)
                            self._pivot_joint_destroyed = True
                            self._pivot_destroyed_step = self._step_timer / time_step if time_step > 0 else None
                            current_sn = int(self._pivot_destroyed_step) if self._pivot_destroyed_step else (int(self._step_timer / time_step) if time_step > 0 else 0)
                            self._event_timeline.append({
                                "step": current_sn,
                                "type": "pivot_destroyed",
                                "net_torque": float(net_torque),
                                "torque_limit": self._max_joint_torque,
                                "ratio": float(abs(net_torque) / self._max_joint_torque) if self._max_joint_torque > 0 else float('inf'),
                                "key": "pivot_destroyed",
                            })
                        except Exception as exc:
                            self._pivot_destroy_error_count += 1
                            self._last_pivot_destroy_error = f"{type(exc).__name__}: {exc}"
                            current_sn = int(self._step_timer / time_step) if time_step > 0 else 0
                            self._event_timeline.append({
                                "step": current_sn,
                                "type": "pivot_destroy_error",
                                "error": self._last_pivot_destroy_error,
                                "key": f"pivot_destroy_error_{self._pivot_destroy_error_count}",
                            })
        self._world.Step(time_step, 60, 60)
        self._step_timer += time_step
        current_step_num = int(self._step_timer / time_step) if time_step > 0 else 0
        for body in self._bodies:
            av = abs(float(getattr(body, "angularVelocity", 0.0)))
            if math.isfinite(av) and av > self._peak_angular_velocity:
                self._peak_angular_velocity = av
        if self._load_body:
            lav = abs(float(getattr(self._load_body, "angularVelocity", 0.0)))
            if math.isfinite(lav) and lav > self._peak_angular_velocity:
                self._peak_angular_velocity = lav
        for i, body in enumerate(self._bodies):
            if math.isfinite(body.position.y) and body.position.y < self.GROUND_Y_FAILURE:
                label = self._body_identities.get(i, f"body_{i}")
                evt_key = f"ground_body_{i}"
                if not any(e.get("key") == evt_key for e in self._event_timeline):
                    self._event_timeline.append({
                        "step": current_step_num,
                        "type": "ground_contact",
                        "body_index": i,
                        "body_label": label,
                        "y": float(body.position.y),
                        "ground_limit": self.GROUND_Y_FAILURE,
                        "key": evt_key,
                    })
        if self._load_body and math.isfinite(self._load_body.position.y) and self._load_body.position.y < self.GROUND_Y_FAILURE:
            evt_key = "ground_load"
            if not any(e.get("key") == evt_key for e in self._event_timeline):
                self._event_timeline.append({
                    "step": current_step_num,
                    "type": "ground_contact",
                    "body_index": -1,
                    "body_label": "load",
                    "y": float(self._load_body.position.y),
                    "ground_limit": self.GROUND_Y_FAILURE,
                    "key": evt_key,
                })
        if self._load_body and self._bodies and getattr(self, "_drop_load", False):
            lx, ly = self._load_body.position.x, self._load_body.position.y
            for body in self._bodies:
                dx = body.position.x - lx
                dy = body.position.y - ly
                dist = math.sqrt(dx * dx + dy * dy)
                if dist < self._min_load_structure_dist:
                    self._min_load_structure_dist = dist
        if self._drop_load and self._load_body and not getattr(self, "_load_caught_by_structure", False) and self._bodies:
            lx, ly = self._load_body.position.x, self._load_body.position.y
            for body in self._bodies:
                dx = body.position.x - lx
                dy = body.position.y - ly
                if math.sqrt(dx * dx + dy * dy) < 0.6:
                    ax, ay = (lx + body.position.x) / 2.0, (ly + body.position.y) / 2.0
                    self._world.CreateWeldJoint(
                        bodyA=body,
                        bodyB=self._load_body,
                        anchor=(ax, ay),
                        collideConnected=False,
                    )
                    self._load_caught_by_structure = True
                    self._joints.append(self._world.joints[-1])
                    self._initial_body_count_on_load_attach = len(self._bodies)
                    current_sn = int(self._step_timer / time_step) if time_step > 0 else 0
                    self._event_timeline.append({
                        "step": current_sn,
                        "type": "load_caught_drop",
                        "catch_pos": (float(ax), float(ay)),
                        "key": "load_caught_drop",
                    })
                    break
    def add_beam(self, x, y, width, height, angle=0, density=1.0, friction=None, label=None):
        width = max(self.MIN_BEAM_SIZE, min(width, self.MAX_BEAM_WIDTH))
        height = max(self.MIN_BEAM_SIZE, min(height, self.MAX_BEAM_HEIGHT))
        if friction is None: friction = self._default_friction
        body = self._world.CreateDynamicBody(
            position=(x, y), angle=angle,
            fixtures=Box2D.b2FixtureDef(shape=polygonShape(box=(width/2, height/2)), density=density, friction=friction, restitution=self._default_restitution)
        )
        body.linearDamping = self._default_linear_damping
        body.angularDamping = self._default_angular_damping
        self._bodies.append(body)
        idx = len(self._bodies) - 1
        if label:
            self._body_identities[idx] = label
        else:
            beam_num = self._body_label_counter["beam"] + 1
            self._body_label_counter["beam"] = beam_num
            self._body_identities[idx] = f"beam_{beam_num}"
        return body
    def add_joint(self, body_a, body_b, anchor_point, type='rigid'):
        anchor_x, anchor_y = anchor_point[0], anchor_point[1]
        pivot = self._terrain_bodies.get("pivot")
        if body_b is None:
            body_b = pivot
        if pivot and (body_a == pivot or body_b == pivot):
            other = body_b if body_a == pivot else body_a
            if self._terrain_config.get("force_pivot_joint", False) or type == 'pivot':
                joint = self._world.CreateRevoluteJoint(bodyA=other, bodyB=pivot, anchor=(self.PIVOT_X, self.PIVOT_Y), collideConnected=False)
            else:
                joint = self._world.CreateWeldJoint(bodyA=other, bodyB=pivot, anchor=(self.PIVOT_X, self.PIVOT_Y), collideConnected=False)
        elif type == 'rigid':
            joint = self._world.CreateWeldJoint(bodyA=body_a, bodyB=body_b, anchor=(anchor_x, anchor_y), collideConnected=False)
        elif type == 'pivot':
            joint = self._world.CreateRevoluteJoint(bodyA=body_a, bodyB=body_b, anchor=(anchor_x, anchor_y), collideConnected=False)
        else:
            raise ValueError(f"Unknown joint type: {type}")
        self._joints.append(joint)
        return joint
    def get_structure_mass(self):
        return sum(b.mass for b in self._bodies)
    def get_main_beam_angle(self):
        return self._bodies[0].angle if self._bodies else 0.0
    def get_terrain_bounds(self):
        load_pos = (3.0, self.PIVOT_Y + 4.0) if self._drop_load else self.LOAD_POSITION
        return {
            "pivot": (self.PIVOT_X, self.PIVOT_Y),
            "load_position": load_pos,
            "max_angle_deviation": self.MAX_ANGLE_DEVIATION * 180 / math.pi,
            "max_beam_width": self.MAX_BEAM_WIDTH,
            "max_beam_height": self.MAX_BEAM_HEIGHT,
            "balance_time": self.BALANCE_TIME,
            "ground_y_failure": self.GROUND_Y_FAILURE,
        }
    def get_event_timeline(self):
        return list(self._event_timeline)
    def get_peak_angular_velocity(self):
        return self._peak_angular_velocity
    def get_body_identities(self):
        return dict(self._body_identities)
    def get_initial_body_count_on_load_attach(self):
        return self._initial_body_count_on_load_attach
