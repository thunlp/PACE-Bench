import Box2D

from Box2D.b2 import (world, polygonShape, staticBody, dynamicBody, weldJoint)

import math

class S02Sandbox:
    def __init__(self, terrain_config=None, physics_config=None):
        self._terrain_config = terrain_config or {}
        self._physics_config = physics_config or {}
        gravity = self._physics_config.get("gravity", (0, -10.0))
        self._world = world(gravity=gravity, doSleep=True)
        self._bodies = []
        self._joints = []
        self._springs = []
        self._terrain_bodies = {}
        self._simulation_time = 0.0
        self._earthquake_amplitude = self._terrain_config.get("earthquake_amplitude", 0.5)
        self._earthquake_frequency = self._terrain_config.get("earthquake_frequency", 2.0)
        self._earthquake_start_time = self._terrain_config.get("earthquake_start_time", 2.0)
        self._earthquake_amplitude_evolution = self._terrain_config.get("earthquake_amplitude_evolution", 0.0)
        self._wind_force = self._terrain_config.get("wind_force", 100.0)
        self._wind_height_threshold = self._terrain_config.get("wind_height_threshold", 20.0)
        self._wind_shear_factor = self._terrain_config.get("wind_shear_factor", 0.0)
        self._wind_oscillation_frequency = self._terrain_config.get("wind_oscillation_frequency", 0.0)
        self._max_joint_force = self._physics_config.get("max_joint_force", float('inf'))
        self._max_joint_torque = self._physics_config.get("max_joint_torque", float('inf'))
        self._peak_joint_force = 0.0
        self._peak_joint_torque = 0.0
        self._joint_break_count = 0
        self._peak_foundation_displacement = 0.0
        self._foundation_initial_x = self._terrain_config.get("foundation_initial_x", 0.0)
        self._total_structure_mass = 0.0
        self.TARGET_HEIGHT = 30.0
        self._joint_peak_data = {}
        self._joint_failure_events = []
        self._max_body_velocity = 0.0
        self._num_steps = 0
        self._joint_observation_error_count = 0
        self._last_joint_observation_error = None
        self._setup_terrain()
    @property
    def world(self):
        return self._world
    def _setup_terrain(self):
        self._terrain_bodies["ground"] = self._world.CreateStaticBody(
            position=(0, -5), shapes=polygonShape(box=(50, 5)))
        self._terrain_bodies["foundation"] = self._world.CreateKinematicBody(
            position=(0, 0.5), shapes=polygonShape(box=(2.0, 0.5)))
    def add_beam(self, x, y, width, height, angle=0, density=1.0):
        width = max(0.1, min(10.0, float(width)))
        height = max(0.1, min(10.0, float(height)))
        body = self._world.CreateDynamicBody(position=(x, y), angle=angle, linearDamping=0.1, angularDamping=0.1)
        body.CreatePolygonFixture(box=(width/2, height/2), density=density, friction=0.5, restitution=0.1)
        self._bodies.append(body)
        return body
    def add_joint(self, body_a, body_b, anchor, type='rigid'):
        if body_b is None: body_b = self._terrain_bodies["ground"]
        joint_def = Box2D.b2WeldJointDef()
        joint_def.Initialize(body_a, body_b, anchor)
        j = self._world.CreateJoint(joint_def)
        self._joints.append(j)
        return j
    def add_spring(self, body_a, body_b, anchor_a, anchor_b, stiffness, damping):
        joint_def = Box2D.b2DistanceJointDef(bodyA=body_a, bodyB=body_b, anchorA=anchor_a, anchorB=anchor_b,
                                            frequencyHz=stiffness, dampingRatio=damping)
        j = self._world.CreateJoint(joint_def)
        self._springs.append(j)
        return j
    def get_foundation(self):
        return self._terrain_bodies["foundation"]
    def get_terrain_bounds(self):
        return (-50, 50, 0, 100)
    def get_vehicle_position(self): return (0.0, 100.0)
    def get_structure_bounds(self):
        if not self._bodies: return {"top": 0, "width": 0, "center_x": 0}
        min_x, max_x, max_y = float('inf'), float('-inf'), float('-inf')
        for b in self._bodies:
            if b.type != Box2D.b2_dynamicBody: continue
            for f in b.fixtures:
                for v in f.shape.vertices:
                    wv = b.GetWorldPoint(v)
                    min_x, max_x, max_y = min(min_x, wv.x), max(max_x, wv.x), max(max_y, wv.y)
        return {"top": max_y, "width": max_x - min_x, "center_x": (min_x + max_x) / 2.0}
    def step(self, time_step):
        self._simulation_time += time_step
        is_during_earthquake = self._simulation_time >= self._earthquake_start_time
        if is_during_earthquake:
            f = self._terrain_bodies["foundation"]
            p = self._earthquake_frequency * (self._simulation_time - self._earthquake_start_time)
            current_amplitude = self._earthquake_amplitude * (1.0 + self._earthquake_amplitude_evolution * (self._simulation_time - self._earthquake_start_time))
            tx = current_amplitude * math.sin(p)
            f.position = (tx, 0.5)
            f.linearVelocity = (current_amplitude * self._earthquake_frequency * math.cos(p), 0)
            abs_disp = abs(tx - self._foundation_initial_x)
            if abs_disp > self._peak_foundation_displacement:
                self._peak_foundation_displacement = abs_disp
        wind_mod = 1.0
        if self._wind_oscillation_frequency > 0:
            wind_mod = 0.5 + 0.5 * math.sin(self._wind_oscillation_frequency * self._simulation_time)
        for b in self._bodies:
            if b.position.y > self._wind_height_threshold:
                h_factor = 1.0 + self._wind_shear_factor * (b.position.y - self._wind_height_threshold)
                force = self._wind_force * h_factor * wind_mod
                b.ApplyForce((force, 0), b.worldCenter, True)
        self._world.Step(time_step, 10, 10)
        self._num_steps += 1
        if self._max_joint_force < float('inf') or self._max_joint_torque < float('inf'):
            to_destroy = []
            for j in self._joints:
                try:
                    force = j.GetReactionForce(1.0/time_step).length
                    torque = abs(j.GetReactionTorque(1.0/time_step))
                    if force > self._peak_joint_force:
                        self._peak_joint_force = force
                    if torque > self._peak_joint_torque:
                        self._peak_joint_torque = torque
                    jid = id(j)
                    if jid not in self._joint_peak_data:
                        anchor = j.anchorA
                        body_a_y = j.bodyA.position.y if j.bodyA else 0.0
                        body_b_y = j.bodyB.position.y if j.bodyB else 0.0
                        self._joint_peak_data[jid] = {
                            'peak_force': 0.0,
                            'peak_torque': 0.0,
                            'anchor_y': float(anchor.y),
                            'body_a_y': float(body_a_y),
                            'body_b_y': float(body_b_y),
                            'broken': False,
                        }
                    entry = self._joint_peak_data[jid]
                    if force > entry['peak_force']:
                        entry['peak_force'] = float(force)
                    if torque > entry['peak_torque']:
                        entry['peak_torque'] = float(torque)
                    if force > self._max_joint_force or torque > self._max_joint_torque:
                        to_destroy.append(j)
                except Exception as exc:
                    self._joint_observation_error_count += 1
                    self._last_joint_observation_error = f"{type(exc).__name__}: {exc}"
                    continue
            for j in to_destroy:
                if j in self._joints:
                    jid = id(j)
                    if jid in self._joint_peak_data:
                        entry = self._joint_peak_data[jid]
                        entry['broken'] = True
                        self._joint_failure_events.append({
                            'step': self._num_steps,
                            'sim_time': float(self._simulation_time),
                            'anchor_y': entry['anchor_y'],
                            'body_a_y': entry['body_a_y'],
                            'body_b_y': entry['body_b_y'],
                            'force': entry['peak_force'],
                            'torque': entry['peak_torque'],
                            'max_force_limit': (float(self._max_joint_force)
                                                if self._max_joint_force < float('inf') else None),
                            'max_torque_limit': (float(self._max_joint_torque)
                                                 if self._max_joint_torque < float('inf') else None),
                        })
                    self._joint_break_count += 1
                    self._world.DestroyJoint(j)
                    self._joints.remove(j)
        for b in self._bodies:
            if b.type == Box2D.b2_dynamicBody:
                speed = math.sqrt(b.linearVelocity.x ** 2 + b.linearVelocity.y ** 2)
                if speed > self._max_body_velocity:
                    self._max_body_velocity = float(speed)
        total_mass = 0.0
        for b in self._bodies:
            if b.type == Box2D.b2_dynamicBody:
                total_mass += b.mass
        self._total_structure_mass = total_mass
    def get_joint_peak_data(self):
        result = []
        for _jid, data in self._joint_peak_data.items():
            result.append(dict(data))
        result.sort(key=lambda x: x.get('peak_force', 0.0), reverse=True)
        return result
    def get_joint_failure_events(self):
        return list(self._joint_failure_events)
    def get_max_body_velocity(self):
        return self._max_body_velocity
    def get_environment_params(self):
        return {
            'gravity': (float(self._world.gravity.x), float(self._world.gravity.y)),
            'wind_force': float(self._wind_force),
            'wind_height_threshold': float(self._wind_height_threshold),
            'wind_shear_factor': float(self._wind_shear_factor),
            'wind_oscillation_frequency': float(self._wind_oscillation_frequency),
            'earthquake_amplitude': float(self._earthquake_amplitude),
            'earthquake_frequency': float(self._earthquake_frequency),
            'earthquake_start_time': float(self._earthquake_start_time),
            'earthquake_amplitude_evolution': float(self._earthquake_amplitude_evolution),
            'max_joint_force': (float(self._max_joint_force)
                                if self._max_joint_force < float('inf') else None),
            'max_joint_torque': (float(self._max_joint_torque)
                                 if self._max_joint_torque < float('inf') else None),
        }
    def get_beam_positions(self):
        result = []
        for i, b in enumerate(self._bodies):
            if b.type != Box2D.b2_dynamicBody:
                continue
            result.append({
                'index': i,
                'x': float(b.position.x),
                'y': float(b.position.y),
                'vx': float(b.linearVelocity.x),
                'vy': float(b.linearVelocity.y),
                'mass': float(b.mass),
                'angle': float(b.angle),
            })
        return result
