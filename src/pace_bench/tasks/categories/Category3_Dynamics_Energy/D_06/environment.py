import math

import Box2D

from Box2D.b2 import (
    world,
    polygonShape,
    circleShape,
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
        self._observation_errors: list[str] = []
        gravity = tuple(physics_config.get("gravity", (0, -10)))
        self._physics_gravity = (float(gravity[0]), float(gravity[1]))
        self._default_linear_damping = float(physics_config.get("linear_damping", 0.0))
        self._default_angular_damping = float(physics_config.get("angular_damping", 0.0))
        self._world = world(gravity=self._physics_gravity, doSleep=True)
        self._structure_friction = float(terrain_config.get("structure_friction", 0.5))
        self._ball_restitution = float(terrain_config.get("ball_restitution", 0.06))
        self._wind_on_structure = bool(terrain_config.get("wind_on_structure", False))
        self._structure_wind_scale = float(terrain_config.get("structure_wind_scale", 0.12))
        self._gravity_pulse_amplitude = float(terrain_config.get("gravity_pulse_amplitude", 0.0))
        self._ball_velocity_scale = float(terrain_config.get("ball_velocity_scale", 1.0))
        self._bodies = []
        self._joints = []
        self._terrain_bodies = {}
        self._structure_smashed = False
        self._max_joint_force = float(terrain_config.get("max_joint_force", 880.0))
        self._joint_fatigue_threshold = float(terrain_config.get("joint_fatigue_threshold", 760.0))
        self._joint_force_last = {}
        self._joint_force_history_len = 2
        self._peak_joint_force_seen = 0.0
        self._sim_time = 0.0
        self._step_count = 0
        self._ball2_launched = False
        self._ball3_launched = False
        self._ball4_launched = False
        self._ball5_launched = False
        self._ball6_launched = False
        self._ball7_launched = False
        self._second_ball_launch_time = float(terrain_config.get("second_ball_launch_time", 0.4))
        self._third_ball_launch_time = float(terrain_config.get("third_ball_launch_time", 1.0))
        self._fourth_ball_launch_time = float(terrain_config.get("fourth_ball_launch_time", 1.3))
        self._fifth_ball_launch_time = float(terrain_config.get("fifth_ball_launch_time", 1.8))
        self._sixth_ball_launch_time = float(terrain_config.get("sixth_ball_launch_time", 2.2))
        self._seventh_ball_launch_time = float(terrain_config.get("seventh_ball_launch_time", 2.7))
        self.world = self._world
        self.bodies = self._bodies
        self.joints = self._joints
        self._create_terrain(terrain_config)
        self._create_balls(terrain_config)
        self.BUILD_ZONE_X_MIN = float(terrain_config.get("build_zone_x_min", 7.0))
        self.BUILD_ZONE_X_MAX = float(terrain_config.get("build_zone_x_max", 11.0))
        self.BUILD_ZONE_Y_MIN = float(terrain_config.get("build_zone_y_min", 0.5))
        self.BUILD_ZONE_Y_MAX = float(terrain_config.get("build_zone_y_max", 5.5))
        self.FORBIDDEN_ZONE_X_MIN = float(terrain_config.get("forbidden_zone_x_min", 8.5))
        self.FORBIDDEN_ZONE_X_MAX = float(terrain_config.get("forbidden_zone_x_max", 9.5))
        self.FORBIDDEN_ZONE_2_X_MIN = float(terrain_config.get("forbidden_zone_2_x_min", 7.35))
        self.FORBIDDEN_ZONE_2_X_MAX = float(terrain_config.get("forbidden_zone_2_x_max", 7.75))
        self.FORBIDDEN_ZONE_3_X_MIN = float(terrain_config.get("forbidden_zone_3_x_min", 7.78))
        self.FORBIDDEN_ZONE_3_X_MAX = float(terrain_config.get("forbidden_zone_3_x_max", 8.55))
        self.FORBIDDEN_ZONE_4_X_MIN = float(terrain_config.get("forbidden_zone_4_x_min", 10.0))
        self.FORBIDDEN_ZONE_4_X_MAX = float(terrain_config.get("forbidden_zone_4_x_max", 10.5))
        self.FORBIDDEN_ZONE_5_X_MIN = float(terrain_config.get("forbidden_zone_5_x_min", 7.18))
        self.FORBIDDEN_ZONE_5_X_MAX = float(terrain_config.get("forbidden_zone_5_x_max", 7.34))
        self.MAX_STRUCTURE_MASS = float(terrain_config.get("max_structure_mass", 10.0))
        self.MAX_BEAM_COUNT = int(terrain_config.get("max_beam_count", 9))
        self.SWEEPER_BAND_1_Y_MIN = float(terrain_config.get("sweeper_band_1_y_min", 2.95))
        self.SWEEPER_BAND_1_Y_MAX = float(terrain_config.get("sweeper_band_1_y_max", 3.55))
        self.SWEEPER_BAND_2_Y_MIN = float(terrain_config.get("sweeper_band_2_y_min", 4.15))
        self.SWEEPER_BAND_2_Y_MAX = float(terrain_config.get("sweeper_band_2_y_max", 4.75))
        self.SWEEPER_BAND_3_Y_MIN = float(terrain_config.get("sweeper_band_3_y_min", 1.0))
        self.SWEEPER_BAND_3_Y_MAX = float(terrain_config.get("sweeper_band_3_y_max", 1.5))
        self.SWEEPER_BAND_4_Y_MIN = float(terrain_config.get("sweeper_band_4_y_min", 2.0))
        self.SWEEPER_BAND_4_Y_MAX = float(terrain_config.get("sweeper_band_4_y_max", 2.5))
        self.WIND_AMPLITUDE = float(terrain_config.get("wind_amplitude", 5.0))
        self.GRAVITY_PULSE_PERIOD = float(terrain_config.get("gravity_pulse_period", 1.2))
        self.WIND_PERIOD = float(terrain_config.get("wind_period", 1.8))
        self.MAX_STEPS = int(terrain_config.get("max_steps", 10000))
    def _create_terrain(self, terrain_config: dict):
        ground_len = 35.0
        ground_h = 0.5
        ground = self._world.CreateStaticBody(
            position=(ground_len / 2, ground_h / 2),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(ground_len / 2, ground_h / 2)),
                friction=0.6,
            ),
        )
        self._terrain_bodies["ground"] = ground
        self._ground_y = ground_h
        zone_x_min = float(terrain_config.get("build_zone_x_min", 7.0))
        wall_x = zone_x_min - 0.05
        wall_h = 6.0
        left_wall = self._world.CreateStaticBody(
            position=(wall_x, ground_h + wall_h / 2),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(0.08, wall_h / 2)),
                friction=0.5,
                restitution=0.1,
            ),
        )
        self._terrain_bodies["left_boundary"] = left_wall
        if terrain_config.get("central_obstacle_enabled", False):
            obs_x = float(terrain_config.get("central_obstacle_x", 9.0))
            obs_h = float(terrain_config.get("central_obstacle_height", 4.0))
            central = self._world.CreateStaticBody(
                position=(obs_x, ground_h + obs_h / 2),
                fixtures=Box2D.b2FixtureDef(
                    shape=polygonShape(box=(0.06, obs_h / 2)),
                    friction=0.4,
                    restitution=0.3,
                ),
            )
            self._terrain_bodies["central_obstacle"] = central
        if terrain_config.get("deflector_enabled", True):
            self._create_deflector(terrain_config)
    def _create_deflector(self, terrain_config: dict):
        deflector_x = float(terrain_config.get("deflector_x", 8.36))
        deflector_y_center = float(terrain_config.get("deflector_y_center", 3.5))
        deflector_amplitude = float(terrain_config.get("deflector_amplitude", 0.65))
        deflector_omega = float(terrain_config.get("deflector_omega", 0.008))
        bar_half_w = 0.12
        bar_half_h = 0.08
        self._deflector_x = deflector_x
        self._deflector_y_center = deflector_y_center
        self._deflector_amplitude = deflector_amplitude
        self._deflector_omega = deflector_omega
        ffd = Box2D.b2FixtureDef(
            shape=polygonShape(box=(bar_half_w, bar_half_h)),
            friction=0.5,
            restitution=0.4,
        )
        bar = self._world.CreateDynamicBody(
            position=(deflector_x, deflector_y_center),
            fixtures=ffd,
        )
        try:
            bar.type = getattr(Box2D.b2, "kinematicBody", 1)
        except Exception as exc:
            self._observation_errors.append(
                f"deflector body type assignment unavailable: {type(exc).__name__}: {exc}"
            )
        self._terrain_bodies["deflector"] = bar
    def _create_balls(self, terrain_config: dict):
        ball_radius = float(terrain_config.get("ball_radius", 0.35))
        ball_density = float(terrain_config.get("ball_density", 95.0))
        ball_vx = float(terrain_config.get("ball_velocity_x", -24.0))
        ball_vy = float(terrain_config.get("ball_velocity_y", 0.0))
        linear_damp = float(terrain_config.get("ball_linear_damping", 0.9))
        angular_damp = float(terrain_config.get("ball_angular_damping", 0.5))
        ball1_x = float(terrain_config.get("ball_spawn_x", 22.0))
        ball1_y = float(terrain_config.get("ball_spawn_y", 4.0))
        ball1 = self._world.CreateDynamicBody(
            position=(ball1_x, ball1_y),
            fixtures=Box2D.b2FixtureDef(
                shape=circleShape(radius=ball_radius),
                density=ball_density,
                friction=0.5,
                restitution=self._ball_restitution,
            ),
        )
        ball1.linearVelocity = (ball_vx * self._ball_velocity_scale, ball_vy)
        ball1.linearDamping = linear_damp
        ball1.angularDamping = angular_damp
        self._terrain_bodies["ball"] = ball1
        ball2_x = float(terrain_config.get("ball2_spawn_x", 22.0))
        ball2_y = float(terrain_config.get("ball2_spawn_y", 3.5))
        ball2 = self._world.CreateDynamicBody(
            position=(ball2_x, ball2_y),
            fixtures=Box2D.b2FixtureDef(
                shape=circleShape(radius=ball_radius),
                density=ball_density,
                friction=0.5,
                restitution=self._ball_restitution,
            ),
        )
        ball2.linearVelocity = (0.0, 0.0)
        ball2.linearDamping = linear_damp
        ball2.angularDamping = angular_damp
        self._terrain_bodies["ball2"] = ball2
        ball3_x = float(terrain_config.get("ball3_spawn_x", 22.0))
        ball3_y = float(terrain_config.get("ball3_spawn_y", 4.5))
        ball3 = self._world.CreateDynamicBody(
            position=(ball3_x, ball3_y),
            fixtures=Box2D.b2FixtureDef(
                shape=circleShape(radius=ball_radius),
                density=ball_density,
                friction=0.5,
                restitution=self._ball_restitution,
            ),
        )
        ball3.linearVelocity = (0.0, 0.0)
        ball3.linearDamping = linear_damp
        ball3.angularDamping = angular_damp
        self._terrain_bodies["ball3"] = ball3
        ball4_x = float(terrain_config.get("ball4_spawn_x", 22.0))
        ball4_y = float(terrain_config.get("ball4_spawn_y", 3.8))
        ball4 = self._world.CreateDynamicBody(
            position=(ball4_x, ball4_y),
            fixtures=Box2D.b2FixtureDef(
                shape=circleShape(radius=ball_radius),
                density=ball_density,
                friction=0.5,
                restitution=self._ball_restitution,
            ),
        )
        ball4.linearVelocity = (0.0, 0.0)
        ball4.linearDamping = linear_damp
        ball4.angularDamping = angular_damp
        self._terrain_bodies["ball4"] = ball4
        ball5_x = float(terrain_config.get("ball5_spawn_x", 22.0))
        ball5_y = float(terrain_config.get("ball5_spawn_y", 4.2))
        ball5 = self._world.CreateDynamicBody(
            position=(ball5_x, ball5_y),
            fixtures=Box2D.b2FixtureDef(
                shape=circleShape(radius=ball_radius),
                density=ball_density,
                friction=0.5,
                restitution=self._ball_restitution,
            ),
        )
        ball5.linearVelocity = (0.0, 0.0)
        ball5.linearDamping = linear_damp
        ball5.angularDamping = angular_damp
        self._terrain_bodies["ball5"] = ball5
        ball6_x = float(terrain_config.get("ball6_spawn_x", 22.0))
        ball6_y = float(terrain_config.get("ball6_spawn_y", 3.9))
        ball6 = self._world.CreateDynamicBody(
            position=(ball6_x, ball6_y),
            fixtures=Box2D.b2FixtureDef(
                shape=circleShape(radius=ball_radius),
                density=ball_density,
                friction=0.5,
                restitution=self._ball_restitution,
            ),
        )
        ball6.linearVelocity = (0.0, 0.0)
        ball6.linearDamping = linear_damp
        ball6.angularDamping = angular_damp
        self._terrain_bodies["ball6"] = ball6
        ball7_x = float(terrain_config.get("ball7_spawn_x", 22.0))
        ball7_y = float(terrain_config.get("ball7_spawn_y", 4.1))
        ball7 = self._world.CreateDynamicBody(
            position=(ball7_x, ball7_y),
            fixtures=Box2D.b2FixtureDef(
                shape=circleShape(radius=ball_radius),
                density=ball_density,
                friction=0.5,
                restitution=self._ball_restitution,
            ),
        )
        ball7.linearVelocity = (0.0, 0.0)
        ball7.linearDamping = linear_damp
        ball7.angularDamping = angular_damp
        self._terrain_bodies["ball7"] = ball7
    MIN_BEAM_SIZE = 0.1
    MAX_BEAM_SIZE = 3.0
    def add_beam(self, x, y, width, height, angle=0, density=1.0):
        width = max(self.MIN_BEAM_SIZE, min(width, self.MAX_BEAM_SIZE))
        height = max(self.MIN_BEAM_SIZE, min(height, self.MAX_BEAM_SIZE))
        body = self._world.CreateDynamicBody(
            position=(x, y),
            angle=angle,
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(width / 2, height / 2)),
                density=density,
                friction=self._structure_friction,
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
            body_b = self._terrain_bodies.get("ground")
            if body_b is None:
                raise ValueError("add_joint: Cannot anchor to ground.")
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
        return sum(b.mass for b in self._bodies)
    def set_material_properties(self, body, restitution=0.2):
        for fixture in body.fixtures:
            fixture.restitution = restitution
    def set_damping(self, body, linear=None, angular=None):
        if linear is not None:
            body.linearDamping = linear
        if angular is not None:
            body.angularDamping = angular
    def step(self, time_step):
        self._sim_time += time_step
        self._step_count += 1
        if self._gravity_pulse_amplitude != 0.0:
            period = max(self.GRAVITY_PULSE_PERIOD, 0.05)
            gy = self._physics_gravity[1] + self._gravity_pulse_amplitude * math.sin(
                2 * math.pi * self._sim_time / period
            )
            self._world.gravity = (self._physics_gravity[0], gy)
        else:
            self._world.gravity = self._physics_gravity
        deflector = self._terrain_bodies.get("deflector")
        if deflector is not None and hasattr(self, "_deflector_x"):
            y_def = self._deflector_y_center + self._deflector_amplitude * math.sin(
                self._step_count * self._deflector_omega
            )
            try:
                deflector.SetTransform((self._deflector_x, y_def), 0.0)
            except AttributeError:
                deflector.transform = ((self._deflector_x, y_def), 0.0)
        ball2_vx = float(self._terrain_config.get("ball2_velocity_x", -26.0))
        ball3_vx = float(self._terrain_config.get("ball3_velocity_x", -24.0))
        ball4_vx = float(self._terrain_config.get("ball4_velocity_x", -28.0))
        if not self._ball2_launched and self._sim_time >= self._second_ball_launch_time:
            ball2 = self._terrain_bodies.get("ball2")
            if ball2 is not None:
                ball_vy = float(self._terrain_config.get("ball_velocity_y", 0.0))
                ball2.linearVelocity = (ball2_vx * self._ball_velocity_scale, ball_vy)
                self._ball2_launched = True
        if not self._ball3_launched and self._sim_time >= self._third_ball_launch_time:
            ball3 = self._terrain_bodies.get("ball3")
            if ball3 is not None:
                ball_vy = float(self._terrain_config.get("ball_velocity_y", 0.0))
                ball3.linearVelocity = (ball3_vx * self._ball_velocity_scale, ball_vy)
                self._ball3_launched = True
        ball5_vx = float(self._terrain_config.get("ball5_velocity_x", -25.0))
        if not self._ball4_launched and self._sim_time >= self._fourth_ball_launch_time:
            ball4 = self._terrain_bodies.get("ball4")
            if ball4 is not None:
                ball_vy = float(self._terrain_config.get("ball_velocity_y", 0.0))
                ball4.linearVelocity = (ball4_vx * self._ball_velocity_scale, ball_vy)
                self._ball4_launched = True
        ball6_vx = float(self._terrain_config.get("ball6_velocity_x", -26.0))
        if not self._ball5_launched and self._sim_time >= self._fifth_ball_launch_time:
            ball5 = self._terrain_bodies.get("ball5")
            if ball5 is not None:
                ball_vy = float(self._terrain_config.get("ball_velocity_y", 0.0))
                ball5.linearVelocity = (ball5_vx * self._ball_velocity_scale, ball_vy)
                self._ball5_launched = True
        ball7_vx = float(self._terrain_config.get("ball7_velocity_x", -25.0))
        if not self._ball6_launched and self._sim_time >= self._sixth_ball_launch_time:
            ball6 = self._terrain_bodies.get("ball6")
            if ball6 is not None:
                ball_vy = float(self._terrain_config.get("ball_velocity_y", 0.0))
                ball6.linearVelocity = (ball6_vx * self._ball_velocity_scale, ball_vy)
                self._ball6_launched = True
        if not self._ball7_launched and self._sim_time >= self._seventh_ball_launch_time:
            ball7 = self._terrain_bodies.get("ball7")
            if ball7 is not None:
                ball_vy = float(self._terrain_config.get("ball_velocity_y", 0.0))
                ball7.linearVelocity = (ball7_vx * self._ball_velocity_scale, ball_vy)
                self._ball7_launched = True
        wind_fx = self.WIND_AMPLITUDE * math.sin(2 * math.pi * self._sim_time / self.WIND_PERIOD)
        if self._wind_on_structure:
            for body in self._bodies:
                body.ApplyForceToCenter(
                    (wind_fx * body.mass * self._structure_wind_scale, 0.0), wake=True
                )
        for key in ("ball", "ball2", "ball3", "ball4", "ball5", "ball6", "ball7"):
            b = self._terrain_bodies.get(key)
            if b is not None:
                b.ApplyForceToCenter((wind_fx * b.mass * 0.08, 0.0), wake=True)
        self._world.Step(time_step, 20, 20)
        self._apply_passive_absorber_drag(time_step)
        if not self._structure_smashed and time_step > 0:
            inv_dt = 1.0 / time_step
            to_remove = []
            for joint in list(self._joints):
                try:
                    force = joint.GetReactionForce(inv_dt)
                    mag = math.sqrt(force.x ** 2 + force.y ** 2)
                    if mag > self._peak_joint_force_seen:
                        self._peak_joint_force_seen = mag
                    hist = self._joint_force_last.setdefault(joint, [])
                    hist.append(mag)
                    if len(hist) > self._joint_force_history_len:
                        hist.pop(0)
                    peak_break = mag >= self._max_joint_force
                    fatigue_break = (
                        len(hist) >= 2
                        and hist[-1] > self._joint_fatigue_threshold
                        and hist[-2] > self._joint_fatigue_threshold
                    )
                    if peak_break or fatigue_break:
                        to_remove.append(joint)
                        self._structure_smashed = True
                except Exception as exc:
                    self._observation_errors.append(
                        f"joint reaction force unavailable: {type(exc).__name__}: {exc}"
                    )
            for joint in to_remove:
                self._joint_force_last.pop(joint, None)
                try:
                    self._world.DestroyJoint(joint)
                    if joint in self._joints:
                        self._joints.remove(joint)
                except Exception as exc:
                    self._observation_errors.append(
                        f"joint removal failed: {type(exc).__name__}: {exc}"
                    )
    def _apply_passive_absorber_drag(self, time_step):
        """Dissipate projectile velocity while touching a damped agent absorber.

        ``set_damping`` configures only an agent-created body.  This contact law
        gives that public material control a physically meaningful absorption
        effect without granting reference code access to environment bodies.
        """
        if time_step <= 0:
            return
        ball_bodies = {
            body for key, body in self._terrain_bodies.items()
            if key.startswith("ball") and body is not None
        }
        agent_bodies = set(self._bodies)
        damping_by_ball = {}
        for contact in self._world.contacts:
            if not contact.touching:
                continue
            body_a = contact.fixtureA.body
            body_b = contact.fixtureB.body
            if body_a in ball_bodies and body_b in agent_bodies:
                ball, absorber = body_a, body_b
            elif body_b in ball_bodies and body_a in agent_bodies:
                ball, absorber = body_b, body_a
            else:
                continue
            damping = max(0.0, float(absorber.linearDamping))
            if damping > damping_by_ball.get(ball, 0.0):
                damping_by_ball[ball] = damping
        for ball, damping in damping_by_ball.items():
            if damping <= 0.0:
                continue
            scale = math.exp(-min(damping, 300.0) * time_step)
            ball.linearVelocity = (
                ball.linearVelocity.x * scale,
                ball.linearVelocity.y * scale,
            )
    def get_terrain_bounds(self):
        return {
            "ground_y": self._ground_y,
            "max_joint_force": self._max_joint_force,
            "joint_fatigue_threshold": self._joint_fatigue_threshold,
            "build_zone": {
                "x": [self.BUILD_ZONE_X_MIN, self.BUILD_ZONE_X_MAX],
                "y": [self.BUILD_ZONE_Y_MIN, self.BUILD_ZONE_Y_MAX],
            },
            "forbidden_zone_x": [self.FORBIDDEN_ZONE_X_MIN, self.FORBIDDEN_ZONE_X_MAX],
            "forbidden_zone_2_x": [self.FORBIDDEN_ZONE_2_X_MIN, self.FORBIDDEN_ZONE_2_X_MAX],
            "forbidden_zone_3_x": [self.FORBIDDEN_ZONE_3_X_MIN, self.FORBIDDEN_ZONE_3_X_MAX],
            "forbidden_zone_4_x": [self.FORBIDDEN_ZONE_4_X_MIN, self.FORBIDDEN_ZONE_4_X_MAX],
            "forbidden_zone_5_x": [self.FORBIDDEN_ZONE_5_X_MIN, self.FORBIDDEN_ZONE_5_X_MAX],
            "max_beam_count": self.MAX_BEAM_COUNT,
            "sweeper_band_1_y": [self.SWEEPER_BAND_1_Y_MIN, self.SWEEPER_BAND_1_Y_MAX],
            "sweeper_band_2_y": [self.SWEEPER_BAND_2_Y_MIN, self.SWEEPER_BAND_2_Y_MAX],
            "sweeper_band_3_y": [self.SWEEPER_BAND_3_Y_MIN, self.SWEEPER_BAND_3_Y_MAX],
            "sweeper_band_4_y": [self.SWEEPER_BAND_4_Y_MIN, self.SWEEPER_BAND_4_Y_MAX],
        }
    def get_ball_position(self):
        b = self._terrain_bodies.get("ball")
        return (b.position.x, b.position.y) if b else None
    def get_ball_velocity(self):
        b = self._terrain_bodies.get("ball")
        return (b.linearVelocity.x, b.linearVelocity.y) if b else None
    def get_ball2_position(self):
        b = self._terrain_bodies.get("ball2")
        return (b.position.x, b.position.y) if b else None
    def get_ball2_velocity(self):
        b = self._terrain_bodies.get("ball2")
        return (b.linearVelocity.x, b.linearVelocity.y) if b else None
    def get_ball3_position(self):
        b = self._terrain_bodies.get("ball3")
        return (b.position.x, b.position.y) if b else None
    def get_ball3_velocity(self):
        b = self._terrain_bodies.get("ball3")
        return (b.linearVelocity.x, b.linearVelocity.y) if b else None
    def get_ball4_position(self):
        b = self._terrain_bodies.get("ball4")
        return (b.position.x, b.position.y) if b else None
    def get_ball4_velocity(self):
        b = self._terrain_bodies.get("ball4")
        return (b.linearVelocity.x, b.linearVelocity.y) if b else None
    def get_ball5_position(self):
        b = self._terrain_bodies.get("ball5")
        return (b.position.x, b.position.y) if b else None
    def get_ball5_velocity(self):
        b = self._terrain_bodies.get("ball5")
        return (b.linearVelocity.x, b.linearVelocity.y) if b else None
    def get_ball6_position(self):
        b = self._terrain_bodies.get("ball6")
        return (b.position.x, b.position.y) if b else None
    def get_ball6_velocity(self):
        b = self._terrain_bodies.get("ball6")
        return (b.linearVelocity.x, b.linearVelocity.y) if b else None
    def get_ball7_position(self):
        b = self._terrain_bodies.get("ball7")
        return (b.position.x, b.position.y) if b else None
    def get_ball7_velocity(self):
        b = self._terrain_bodies.get("ball7")
        return (b.linearVelocity.x, b.linearVelocity.y) if b else None
    def get_all_balls_positions(self):
        out = []
        if self._terrain_bodies.get("ball") is not None:
            out.append(self.get_ball_position())
        if self._terrain_bodies.get("ball2") is not None:
            out.append(self.get_ball2_position())
        if self._terrain_bodies.get("ball3") is not None:
            out.append(self.get_ball3_position())
        if self._terrain_bodies.get("ball4") is not None:
            out.append(self.get_ball4_position())
        if self._terrain_bodies.get("ball5") is not None:
            out.append(self.get_ball5_position())
        if self._terrain_bodies.get("ball6") is not None:
            out.append(self.get_ball6_position())
        if self._terrain_bodies.get("ball7") is not None:
            out.append(self.get_ball7_position())
        return out
    def get_all_balls_velocities(self):
        out = []
        if self._terrain_bodies.get("ball") is not None:
            out.append(self.get_ball_velocity())
        if self._terrain_bodies.get("ball2") is not None:
            out.append(self.get_ball2_velocity())
        if self._terrain_bodies.get("ball3") is not None:
            out.append(self.get_ball3_velocity())
        if self._terrain_bodies.get("ball4") is not None:
            out.append(self.get_ball4_velocity())
        if self._terrain_bodies.get("ball5") is not None:
            out.append(self.get_ball5_velocity())
        if self._terrain_bodies.get("ball6") is not None:
            out.append(self.get_ball6_velocity())
        if self._terrain_bodies.get("ball7") is not None:
            out.append(self.get_ball7_velocity())
        return out
    def is_structure_smashed(self):
        return self._structure_smashed
    def get_joint_reaction_forces(self):
        result = []
        inv_dt = 60.0
        limit = max(self._max_joint_force, 1e-9)
        for i, joint in enumerate(self._joints):
            try:
                force = joint.GetReactionForce(inv_dt)
                mag = math.sqrt(force.x * force.x + force.y * force.y)
                anchor = None
                try:
                    anchor = joint.anchor
                except Exception:
                    try:
                        anchor = joint.anchorA
                    except Exception:
                        try:
                            anchor = joint.GetAnchorA()
                        except Exception:
                            pass
                if anchor is not None:
                    ax, ay = float(anchor.x), float(anchor.y)
                else:
                    ax, ay = 0.0, 0.0
                result.append((i, ax, ay, mag, mag / limit))
            except Exception as exc:
                self._observation_errors.append(
                    f"joint reaction force sample {i} unavailable: {type(exc).__name__}: {exc}"
                )
        return result
    def get_peak_joint_force_seen(self):
        peak = self._peak_joint_force_seen
        for hist in self._joint_force_last.values():
            for mag in hist:
                if mag > peak:
                    peak = mag
        return peak
    def get_joint_fatigue_state(self):
        result = []
        inv_dt = 60.0
        fatigue = max(self._joint_fatigue_threshold, 1e-9)
        for i, joint in enumerate(self._joints):
            try:
                force = joint.GetReactionForce(inv_dt)
                mag = math.sqrt(force.x * force.x + force.y * force.y)
                if mag >= self._joint_fatigue_threshold:
                    anchor = None
                    try:
                        anchor = joint.anchor
                    except Exception:
                        try:
                            anchor = joint.anchorA
                        except Exception:
                            try:
                                anchor = joint.GetAnchorA()
                            except Exception:
                                pass
                    if anchor is not None:
                        ax, ay = float(anchor.x), float(anchor.y)
                    else:
                        ax, ay = 0.0, 0.0
                    result.append((i, ax, ay, mag, mag / fatigue))
            except Exception as exc:
                self._observation_errors.append(
                    f"joint fatigue sample {i} unavailable: {type(exc).__name__}: {exc}"
                )
        return result
    def get_all_balls_masses(self):
        out = []
        for key in ("ball", "ball2", "ball3", "ball4", "ball5", "ball6", "ball7"):
            b = self._terrain_bodies.get(key)
            if b is not None:
                out.append(b.mass)
        return out
    def get_environment_config_snapshot(self):
        return {
            "max_joint_force": self._max_joint_force,
            "joint_fatigue_threshold": self._joint_fatigue_threshold,
            "gravity_base": self._physics_gravity,
            "gravity_pulse_amplitude": self._gravity_pulse_amplitude,
            "gravity_pulse_period": self.GRAVITY_PULSE_PERIOD,
            "wind_amplitude": self.WIND_AMPLITUDE,
            "wind_on_structure": self._wind_on_structure,
            "structure_wind_scale": self._structure_wind_scale,
            "ball_restitution": self._ball_restitution,
            "ball_linear_damping": float(self._terrain_config.get("ball_linear_damping", 0.9)),
            "ball_angular_damping": float(self._terrain_config.get("ball_angular_damping", 0.5)),
            "structure_friction": self._structure_friction,
            "structure_linear_damping": self._default_linear_damping,
            "structure_angular_damping": self._default_angular_damping,
            "ball_velocity_scale": self._ball_velocity_scale,
        }
    def get_observation_errors(self):
        return list(self._observation_errors)
