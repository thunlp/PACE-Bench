import Box2D

from Box2D.b2 import (
    world,
    polygonShape,
    circleShape,
    staticBody,
    dynamicBody,
    revoluteJoint,
    weldJoint,
    distanceJointDef,

)

import math

class Sandbox:
    def __init__(self, *, terrain_config=None, physics_config=None):
        terrain_config = terrain_config or {}
        physics_config = physics_config or {}
        self._terrain_config = dict(terrain_config)
        self._physics_config = dict(physics_config)
        gravity = tuple(physics_config.get("gravity", (0, -10)))
        self._default_linear_damping = float(physics_config.get("linear_damping", 0.0))
        self._default_angular_damping = float(physics_config.get("angular_damping", 0.0))
        self._world = world(gravity=gravity, doSleep=True)
        self._bodies = []
        self._joints = []
        self._springs = []
        self._spring_metadata = []
        self._terrain_bodies = {}
        self._observation_error_count = 0
        self._last_observation_error = None
        self.world = self._world
        self.bodies = self._bodies
        self.joints = self._joints
        self.springs = self._springs
        self._create_terrain(terrain_config)
        self._create_projectile(terrain_config)
        self._create_target_zone(terrain_config)
        self.BUILD_ZONE_X_MIN = float(terrain_config.get("build_zone_x_min", 5.0))
        self.BUILD_ZONE_X_MAX = float(terrain_config.get("build_zone_x_max", 15.0))
        self.BUILD_ZONE_Y_MIN = float(terrain_config.get("build_zone_y_min", 1.5))
        self.BUILD_ZONE_Y_MAX = float(terrain_config.get("build_zone_y_max", 8.0))
        self.MAX_STRUCTURE_MASS = float(terrain_config.get("max_structure_mass", 500.0))
    def _create_terrain(self, terrain_config: dict):
        ground_friction = float(terrain_config.get("ground_friction", 0.6))
        ground_length = 60.0
        ground_height = 1.0
        ground = self._world.CreateStaticBody(
            position=(ground_length / 2, ground_height / 2),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(ground_length / 2, ground_height / 2)),
                friction=ground_friction,
            ),
        )
        self._terrain_bodies["ground"] = ground
        self._ground_y = ground_height
    def _create_projectile(self, terrain_config: dict):
        spawn_x = float(terrain_config.get("projectile_spawn_x", 10.0))
        spawn_y = float(terrain_config.get("projectile_spawn_y", 3.0))
        radius = float(terrain_config.get("projectile_radius", 0.25))
        density = float(terrain_config.get("projectile_density", 1.0))
        projectile = self._world.CreateDynamicBody(
            position=(spawn_x, spawn_y),
            fixtures=Box2D.b2FixtureDef(
                shape=circleShape(radius=radius),
                density=density,
                friction=0.3,
                restitution=0.2,
            ),
        )
        projectile.linearDamping = self._default_linear_damping
        projectile.angularDamping = self._default_angular_damping
        self._terrain_bodies["projectile"] = projectile
    def _create_target_zone(self, terrain_config: dict):
        self._target_x_min = float(terrain_config.get("target_x_min", 40.0))
        self._target_x_max = float(terrain_config.get("target_x_max", 45.0))
        self._target_y_min = float(terrain_config.get("target_y_min", 2.0))
        self._target_y_max = float(terrain_config.get("target_y_max", 5.0))
    SIM_BOUNDS_X_MIN = -10.0
    SIM_BOUNDS_X_MAX = 60.0
    SIM_BOUNDS_Y_MIN = -5.0
    MIN_BEAM_SIZE = 0.1
    MAX_BEAM_SIZE = 5.0
    MIN_SPRING_STIFFNESS = 10.0
    MAX_SPRING_STIFFNESS = 3000.0
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
            body_b = self._terrain_bodies.get("ground")
            if body_b is None:
                raise ValueError("add_joint: Cannot anchor to ground; ground body not found.")
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
    def add_spring(
        self,
        body_a,
        body_b,
        anchor_a,
        anchor_b,
        rest_length=None,
        stiffness=500.0,
        damping_ratio=0.5,
    ):
        stiffness = max(
            self.MIN_SPRING_STIFFNESS,
            min(stiffness, self.MAX_SPRING_STIFFNESS),
        )
        ax, ay = anchor_a[0], anchor_a[1]
        bx, by = anchor_b[0], anchor_b[1]
        if rest_length is None:
            rest_length = math.sqrt((bx - ax) ** 2 + (by - ay) ** 2)
            rest_length = max(0.1, rest_length)
        defn = distanceJointDef()
        defn.bodyA = body_a
        defn.bodyB = body_b
        defn.localAnchorA = body_a.GetLocalPoint((ax, ay))
        defn.localAnchorB = body_b.GetLocalPoint((bx, by))
        defn.length = rest_length
        defn.collideConnected = False
        try:
            defn.frequencyHz = min(10.0, math.sqrt(stiffness / 10.0) / (2 * math.pi))
        except Exception as exc:
            self._record_observation_error("add_spring.frequency", exc)
            defn.frequencyHz = 4.0
        defn.dampingRatio = max(0.0, min(1.0, damping_ratio))
        joint = self._world.CreateJoint(defn)
        self._springs.append(joint)
        self._spring_metadata.append({
            'body_a': body_a,
            'body_b': body_b,
            'anchor_a': (ax, ay),
            'anchor_b': (bx, by),
            'rest_length': rest_length,
            'stiffness': stiffness,
            'damping_ratio': damping_ratio,
        })
        return joint
    def get_structure_mass(self):
        total = 0.0
        for body in self._bodies:
            total += body.mass
        return total
    def set_material_properties(self, body, restitution=0.2):
        for fixture in body.fixtures:
            fixture.restitution = float(restitution)
    def step(self, time_step):
        self._world.Step(time_step, 10, 10)
    def get_terrain_bounds(self):
        return {
            "ground_y": self._ground_y,
            "target_zone": {
                "x_min": self._target_x_min,
                "x_max": self._target_x_max,
                "y_min": self._target_y_min,
                "y_max": self._target_y_max,
            },
            "build_zone": {
                "x": [self.BUILD_ZONE_X_MIN, self.BUILD_ZONE_X_MAX],
                "y": [self.BUILD_ZONE_Y_MIN, self.BUILD_ZONE_Y_MAX],
            },
            "projectile_spawn": (
                self._terrain_config.get("projectile_spawn_x", 10.0),
                self._terrain_config.get("projectile_spawn_y", 3.0),
            ),
        }
    def get_projectile_position(self):
        proj = self._terrain_bodies.get("projectile")
        if proj is None:
            return None
        return (proj.position.x, proj.position.y)
    def get_projectile_velocity(self):
        proj = self._terrain_bodies.get("projectile")
        if proj is None:
            return None
        return (proj.linearVelocity.x, proj.linearVelocity.y)
    def get_ground(self):
        return self._terrain_bodies.get("ground")
    def get_projectile(self):
        return self._terrain_bodies.get("projectile")
    def get_spring_states(self):
        states = []
        for i, spring in enumerate(self._springs):
            try:
                metadata = self._spring_metadata[i] if i < len(self._spring_metadata) else {}
                wa_x = float(spring.anchorA.x)
                wa_y = float(spring.anchorA.y)
                wb_x = float(spring.anchorB.x)
                wb_y = float(spring.anchorB.y)
                current_length = math.sqrt(
                    (wb_x - wa_x) ** 2 + (wb_y - wa_y) ** 2
                )
                rest_length = float(spring.length)
                stiffness = float(metadata.get('stiffness', 0.0))
                dl = current_length - rest_length
                pe = 0.5 * stiffness * dl * dl
                eps = 0.001
                ratio = rest_length / max(eps, current_length)
                compressed = current_length < rest_length - eps
                tensioned = current_length > rest_length + eps
                slack = not compressed and not tensioned
                force_est = stiffness * abs(dl)
                states.append({
                    'index': i,
                    'anchor_a': (wa_x, wa_y),
                    'anchor_b': (wb_x, wb_y),
                    'current_length': current_length,
                    'rest_length': rest_length,
                    'compression_ratio': ratio,
                    'stiffness': stiffness,
                    'damping_ratio': float(spring.dampingRatio),
                    'elastic_pe': pe,
                    'force_est': force_est,
                    'is_compressed': compressed,
                    'is_tensioned': tensioned,
                    'is_slack': slack,
                })
            except Exception as exc:
                self._record_observation_error("get_spring_states", exc)
                states.append({
                    'index': i, 'error': f'{type(exc).__name__}: {exc}',
                })
        return states
    def get_joint_topology(self):
        proj = self._terrain_bodies.get('projectile')
        joints_info = []
        for i, joint in enumerate(self._joints):
            try:
                jtype = type(joint).__name__
                body_a = joint.bodyA
                body_b = joint.bodyB
                proj_connected = (body_a is proj) or (body_b is proj)
                anchor = None
                if hasattr(joint, 'anchorA'):
                    anchor = (
                        float(joint.anchorA.x),
                        float(joint.anchorA.y),
                    )
                joints_info.append({
                    'index': i,
                    'type': jtype,
                    'projectile_connected': proj_connected,
                    'anchor': anchor,
                })
            except Exception as exc:
                self._record_observation_error("get_joint_topology", exc)
                joints_info.append({
                    'index': i, 'error': f'{type(exc).__name__}: {exc}',
                })
        return joints_info
    def get_arm_state(self):
        if not self._bodies:
            return None
        try:
            arm = self._bodies[0]
            ke_lin = 0.5 * arm.mass * (
                arm.linearVelocity.x ** 2 + arm.linearVelocity.y ** 2
            )
            ke_ang = 0.5 * arm.inertia * (arm.angularVelocity ** 2)
            pivot = None
            ground = self._terrain_bodies.get('ground')
            if ground is not None:
                for joint in self._joints:
                    if type(joint).__name__ == 'b2RevoluteJoint':
                        if ((joint.bodyA is arm and joint.bodyB is ground) or
                                (joint.bodyB is arm and joint.bodyA is ground)):
                            if hasattr(joint, 'anchorA'):
                                pivot = (
                                    float(joint.anchorA.x),
                                    float(joint.anchorA.y),
                                )
                            break
            return {
                'position': (float(arm.position.x), float(arm.position.y)),
                'angle': float(arm.angle),
                'angular_velocity': float(arm.angularVelocity),
                'linear_velocity': (
                    float(arm.linearVelocity.x),
                    float(arm.linearVelocity.y),
                ),
                'speed': float(math.sqrt(
                    arm.linearVelocity.x ** 2 + arm.linearVelocity.y ** 2
                )),
                'mass': float(arm.mass),
                'inertia': float(arm.inertia),
                'kinetic_energy': ke_lin + ke_ang,
                'awake': bool(arm.awake),
                'pivot': pivot,
            }
        except Exception as exc:
            self._record_observation_error("get_arm_state", exc)
            return None
    def get_contacts_involving(self, body_a, body_b=None):
        results = []
        try:
            for contact in self._world.contacts:
                if not contact.contact.fixtureA or not contact.contact.fixtureB:
                    continue
                ba = contact.contact.fixtureA.body
                bb = contact.contact.fixtureB.body
                match = False
                if body_b is not None:
                    match = (ba is body_a and bb is body_b) or (ba is body_b and bb is body_a)
                else:
                    match = (ba is body_a) or (bb is body_a)
                if match:
                    manifold = contact.contact.manifold
                    num_pts = getattr(manifold, 'pointCount', 0)
                    points = []
                    for pi in range(min(num_pts, 2)):
                        try:
                            pt = manifold.points[pi]
                            points.append({
                                'x': float(pt.localPoint.x),
                                'y': float(pt.localPoint.y),
                            })
                        except Exception as exc:
                            self._record_observation_error("get_contacts_involving.point", exc)
                    results.append({
                        'body_a': id(ba),
                        'body_b': id(bb),
                        'touching': contact.contact.touching,
                        'enabled': contact.contact.enabled,
                        'manifold_points': num_pts,
                    })
        except Exception as exc:
            self._record_observation_error("get_contacts_involving", exc)
        return results
    def _record_observation_error(self, source, exc):
        self._observation_error_count += 1
        self._last_observation_error = f"{source}: {type(exc).__name__}: {exc}"
    def get_physics_config(self):
        return {
            'gravity': (
                float(self._world.gravity.x),
                float(self._world.gravity.y),
            ),
            'linear_damping': float(self._default_linear_damping),
            'angular_damping': float(self._default_angular_damping),
        }
