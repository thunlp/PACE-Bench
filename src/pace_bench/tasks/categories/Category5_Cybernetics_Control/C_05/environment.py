import math

import Box2D

DEFAULT_SIMULATION_TIME_STEP = 1.0 / 60.0

from Box2D.b2 import (
    world,
    polygonShape,
    circleShape,
    staticBody,
    dynamicBody,

)

REQUIRED_ORDER = ("A", "B", "C")

TRIGGER_STAY_STEPS = 25

SPEED_CAP_INSIDE = 0.5

REPULSION_MAG = 22.0

REPULSION_STRONG_THRESHOLD = 40.0

REPULSION_RANGE = 1.5

REPULSION_TANGENTIAL_MAG = 0.0

COOLDOWN_STEPS = 55

BARRIER_DELAY_STEPS = 70

BARRIER_X = 4.5

BARRIER_HALFW = 0.08

BARRIER_LO = 0.0

BARRIER_HI = 4.0

WIND_AMP = 0.0

WIND_PERIOD = 200

C_HIGH_HISTORY = 150

C_REQUIRED_MAX_Y = 2.9

RECENT_A_FOR_B = 160

RECENT_B_FOR_C = 400

FORCE_LIMIT_INSIDE = 60.0

ZONE_A = (2.0, 2.0, 0.5, 0.5)

ZONE_B = (4.95, 3.2, 0.7, 0.4)

ZONE_C = (8.0, 2.0, 0.5, 0.5)

SPAWN_X = 0.5

SPAWN_Y = 1.95

AGENT_RADIUS = 0.2

AGENT_MASS = 3.0

MAX_AGENT_FORCE_PER_AXIS = 50.0

GROUND_FRICTION_DEFAULT = 0.5

RAMP_FRICTION_DEFAULT = 0.12

PLATFORM_FRICTION_DEFAULT = 0.45

AGENT_FIXTURE_FRICTION = 0.4

BARRIER_FIXTURE_FRICTION = 0.3

DEFAULT_LINEAR_DAMPING = 0.3

DEFAULT_ANGULAR_DAMPING = 0.3

class Sandbox:
    def __init__(self, *, terrain_config=None, physics_config=None):
        terrain_config = terrain_config or {}
        physics_config = physics_config or {}
        self._terrain_config = dict(terrain_config)
        self._physics_config = dict(physics_config)
        gravity = tuple(physics_config.get("gravity", (0, -10)))
        self._default_linear_damping = float(
            physics_config.get("linear_damping", DEFAULT_LINEAR_DAMPING)
        )
        self._default_angular_damping = float(
            physics_config.get("angular_damping", DEFAULT_ANGULAR_DAMPING)
        )
        self._trigger_stay_steps = int(physics_config.get("trigger_stay_steps", TRIGGER_STAY_STEPS))
        self._speed_cap_inside = float(physics_config.get("speed_cap_inside", SPEED_CAP_INSIDE))
        self._repulsion_mag = float(physics_config.get("repulsion_mag", REPULSION_MAG))
        self._repulsion_range = float(physics_config.get("repulsion_range", REPULSION_RANGE))
        self._cooldown_steps = int(physics_config.get("cooldown_steps", COOLDOWN_STEPS))
        self._barrier_delay_steps = int(physics_config.get("barrier_delay_steps", BARRIER_DELAY_STEPS))
        self._wind_amp = float(physics_config.get("wind_amp", WIND_AMP))
        self._wind_period = int(physics_config.get("wind_period", WIND_PERIOD))
        self._c_high_history = int(physics_config.get("c_high_history", C_HIGH_HISTORY))
        self._c_required_max_y = float(physics_config.get("c_required_max_y", C_REQUIRED_MAX_Y))
        self._recent_a_for_b = int(physics_config.get("recent_a_for_b", RECENT_A_FOR_B))
        self._recent_b_for_c = int(physics_config.get("recent_b_for_c", RECENT_B_FOR_C))
        self._repulsion_tangential_mag = float(
            physics_config.get("repulsion_tangential_mag", REPULSION_TANGENTIAL_MAG)
        )
        self._force_limit_inside = float(physics_config.get("force_limit_inside", FORCE_LIMIT_INSIDE))
        self._max_agent_force = float(
            physics_config.get("max_agent_force_per_axis", MAX_AGENT_FORCE_PER_AXIS)
        )
        self._barrier_x = float(terrain_config.get("barrier_x", BARRIER_X))
        self._world = world(gravity=gravity, doSleep=True)
        self._bodies = []
        self._joints = []
        self._terrain_bodies = {}
        self.world = self._world
        self.bodies = self._bodies
        self.joints = self._joints
        self._zones = {"A": ZONE_A, "B": ZONE_B, "C": ZONE_C}
        self._triggered_order = []
        self._wrong_order = False
        self._trigger_step = {}
        self._zone_contact_steps = {"A": 0, "B": 0, "C": 0}
        self._last_zone = None
        self._agent_radius = float(terrain_config.get("agent_radius", AGENT_RADIUS))
        self._agent_mass = float(terrain_config.get("agent_mass", AGENT_MASS))
        self._spawn_x = float(terrain_config.get("spawn_x", SPAWN_X))
        self._spawn_y = float(terrain_config.get("spawn_y", SPAWN_Y))
        self._ground_friction = float(
            terrain_config.get("ground_friction", GROUND_FRICTION_DEFAULT)
        )
        self._ramp_friction = float(terrain_config.get("ramp_friction", RAMP_FRICTION_DEFAULT))
        self._platform_friction = float(
            terrain_config.get("platform_friction", PLATFORM_FRICTION_DEFAULT)
        )
        self._agent_fixture_friction = float(
            terrain_config.get("agent_fixture_friction", AGENT_FIXTURE_FRICTION)
        )
        self._barrier_fixture_friction = float(
            terrain_config.get("barrier_fixture_friction", BARRIER_FIXTURE_FRICTION)
        )
        self._step_count = 0
        self._barrier_remove_at_step = None
        self._agent_y_history = []
        self._last_step_in_A = -9999
        self._last_step_in_B = -9999
        self._create_ground(terrain_config)
        self._create_barrier()
        self._create_agent(terrain_config)
        self._force_x = 0.0
        self._force_y = 0.0
        self._last_force_x = 0.0
        self._last_force_y = 0.0
        self._dwell_reset_zone_change = 0
        self._dwell_reset_speed = 0
        self._dwell_reset_force = 0
        self._dwell_blocked_temporal = 0
        self._dwell_blocked_altitude = 0
        self._dwell_blocked_cooldown = 0
        self._peak_speed_seen = 0.0
        self._peak_force_applied = 0.0
    def _create_ground(self, terrain_config: dict):
        ground_y = 2.0
        h = 0.25
        ground_segments = self._terrain_bodies.setdefault("ground_segments", [])
        body_flat_left = self._world.CreateStaticBody(
            position=(2.0, ground_y - h),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(2.0, h)),
                friction=self._ground_friction,
                restitution=0.0,
            ),
        )
        ground_segments.append(body_flat_left)
        ramp1_verts = [(-0.75, -1.25), (0.75, 0.25), (0.75, 0.75), (-0.75, -0.75)]
        body_ramp1 = self._world.CreateStaticBody(
            position=(4.75, 2.75),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(vertices=ramp1_verts),
                friction=self._ramp_friction,
                restitution=0.0,
            ),
        )
        ground_segments.append(body_ramp1)
        platform_hh = 0.25
        body_platform = self._world.CreateStaticBody(
            position=(5.5, 3.25),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(0.5, platform_hh)),
                friction=self._platform_friction,
                restitution=0.0,
            ),
        )
        ground_segments.append(body_platform)
        ramp2_verts = [(-0.5, 0.5), (-0.5, 0.75), (0.5, -0.75), (0.5, -1.0)]
        body_ramp2 = self._world.CreateStaticBody(
            position=(6.5, 2.75),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(vertices=ramp2_verts),
                friction=self._ramp_friction,
                restitution=0.0,
            ),
        )
        ground_segments.append(body_ramp2)
        body_flat_right = self._world.CreateStaticBody(
            position=(9.5, ground_y - h),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(2.5, h)),
                friction=self._ground_friction,
                restitution=0.0,
            ),
        )
        ground_segments.append(body_flat_right)
        self._ground_y_top = ground_y
    def _create_barrier(self):
        cx = self._barrier_x
        cy = (BARRIER_LO + BARRIER_HI) / 2
        hh = (BARRIER_HI - BARRIER_LO) / 2
        self._barrier_body = self._world.CreateStaticBody(
            position=(cx, cy),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(BARRIER_HALFW, hh)),
                friction=self._barrier_fixture_friction,
                restitution=0.0,
            ),
        )
        self._terrain_bodies["barrier"] = self._barrier_body
    def _schedule_barrier_removal(self):
        if self._barrier_remove_at_step is None:
            self._barrier_remove_at_step = self._step_count + int(self._barrier_delay_steps)
    def _create_agent(self, terrain_config: dict):
        r = self._agent_radius
        density = self._agent_mass / (math.pi * r * r)
        agent = self._world.CreateDynamicBody(
            position=(self._spawn_x, self._spawn_y),
            fixtures=Box2D.b2FixtureDef(
                shape=circleShape(radius=r),
                density=density,
                friction=self._agent_fixture_friction,
                restitution=0.0,
            ),
        )
        agent.linearDamping = self._default_linear_damping
        agent.angularDamping = self._default_angular_damping
        self._terrain_bodies["agent"] = agent
    def _point_in_zone(self, x, y, zone_name):
        cx, cy, hw, hh = self._zones[zone_name]
        return (cx - hw <= x <= cx + hw) and (cy - hh <= y <= cy + hh)
    def _zone_center(self, zone_name):
        cx, cy, _, _ = self._zones[zone_name]
        return (cx, cy)
    def _update_sequence(self):
        agent = self._terrain_bodies.get("agent")
        if agent is None:
            return
        x, y = agent.position.x, agent.position.y
        next_required = REQUIRED_ORDER[len(self._triggered_order)] if len(self._triggered_order) < 3 else None
        current_zone = None
        for name in REQUIRED_ORDER:
            if self._point_in_zone(x, y, name):
                current_zone = name
                break
        if current_zone == "A":
            self._last_step_in_A = self._step_count
        if current_zone == "B":
            self._last_step_in_B = self._step_count
        if self._last_zone != current_zone:
            if self._last_zone is not None:
                self._dwell_reset_zone_change += 1
            for z in REQUIRED_ORDER:
                self._zone_contact_steps[z] = 0
        self._last_zone = current_zone
        if current_zone is None:
            return
        if current_zone in self._triggered_order:
            return
        if current_zone != next_required:
            self._wrong_order = True
            return
        prev_zone = REQUIRED_ORDER[len(self._triggered_order) - 1] if self._triggered_order else None
        if prev_zone and prev_zone in self._trigger_step:
            steps_since = self._step_count - self._trigger_step[prev_zone]
            if steps_since < int(self._cooldown_steps):
                self._dwell_blocked_cooldown += 1
                return
        vx = agent.linearVelocity.x
        vy = agent.linearVelocity.y
        speed = math.sqrt(vx * vx + vy * vy)
        if speed > self._peak_speed_seen:
            self._peak_speed_seen = speed
        if speed > float(self._speed_cap_inside):
            self._dwell_reset_speed += 1
            self._zone_contact_steps[current_zone] = 0
            return
        applied_f_mag = math.sqrt(self._force_x**2 + self._force_y**2)
        if applied_f_mag > self._peak_force_applied:
            self._peak_force_applied = applied_f_mag
        if applied_f_mag > float(self._force_limit_inside):
            self._dwell_reset_force += 1
            self._zone_contact_steps[current_zone] = 0
            return
        if current_zone == "B":
            if self._step_count - self._last_step_in_A > int(self._recent_a_for_b):
                self._dwell_blocked_temporal += 1
                return
        if current_zone == "C":
            if self._step_count - self._last_step_in_B > int(self._recent_b_for_c):
                self._dwell_blocked_temporal += 1
                return
            ch = int(self._c_high_history)
            if not self._agent_y_history:
                return
            relevant_history = self._agent_y_history[-ch:]
            max_recent_y = max(relevant_history)
            if max_recent_y < float(self._c_required_max_y):
                self._dwell_blocked_altitude += 1
                return
        self._zone_contact_steps[current_zone] += 1
        if self._zone_contact_steps[current_zone] >= int(self._trigger_stay_steps):
            self._triggered_order.append(current_zone)
            self._trigger_step[current_zone] = self._step_count
            self._zone_contact_steps[current_zone] = 0
            if current_zone == "A":
                self._schedule_barrier_removal()
    def _repulsion_force(self, x, y):
        fx, fy = 0.0, 0.0
        rep_mag = float(self._repulsion_mag)
        rep_range = float(self._repulsion_range)
        t_mag = float(self._repulsion_tangential_mag)
        if "A" not in self._triggered_order:
            bx, by = self._zone_center("B")
            dist = math.sqrt((x - bx) ** 2 + (y - by) ** 2)
            if dist < rep_range and dist > 1e-6:
                strength = rep_mag * (1.0 - dist / rep_range)
                ux, uy = (x - bx) / dist, (y - by) / dist
                fx += strength * ux
                fy += strength * uy
                if t_mag != 0:
                    t_strength = t_mag * (1.0 - dist / rep_range)
                    fx += t_strength * (-uy)
                    fy += t_strength * ux
        if "B" not in self._triggered_order:
            cx, cy = self._zone_center("C")
            dist = math.sqrt((x - cx) ** 2 + (y - cy) ** 2)
            if dist < rep_range and dist > 1e-6:
                strength = rep_mag * (1.0 - dist / rep_range)
                ux, uy = (x - cx) / dist, (y - cy) / dist
                fx += strength * ux
                fy += strength * uy
                if t_mag != 0:
                    t_strength = t_mag * (1.0 - dist / rep_range)
                    fx += t_strength * (-uy)
                    fy += t_strength * ux
        return (fx, fy)
    def get_agent_position(self):
        agent = self._terrain_bodies.get("agent")
        if agent is None:
            return (0.0, 0.0)
        return (agent.position.x, agent.position.y)
    def get_agent_velocity(self):
        agent = self._terrain_bodies.get("agent")
        if agent is None:
            return (0.0, 0.0)
        return (agent.linearVelocity.x, agent.linearVelocity.y)
    def get_next_required_switch(self):
        if self._wrong_order:
            return None
        idx = len(self._triggered_order)
        if idx >= len(REQUIRED_ORDER):
            return None
        return REQUIRED_ORDER[idx]
    def get_triggered_switches(self):
        return list(self._triggered_order)
    def get_sequence_correct(self):
        return (
            not self._wrong_order
            and self._triggered_order == list(REQUIRED_ORDER)
        )
    def get_steps_in_current_zone(self):
        next_req = self.get_next_required_switch()
        if next_req is None:
            return 0
        return self._zone_contact_steps.get(next_req, 0)
    def get_steps_required_to_trigger(self):
        return int(self._trigger_stay_steps)
    def get_cooldown_remaining(self):
        if not self._triggered_order:
            return 0
        prev = self._triggered_order[-1]
        if prev not in self._trigger_step:
            return 0
        elapsed = self._step_count - self._trigger_step[prev]
        return max(0, int(self._cooldown_steps) - elapsed)
    def get_barrier_delay_steps(self):
        return int(self._barrier_delay_steps)
    def get_barrier_x(self):
        return float(self._barrier_x)
    def apply_agent_force(self, force_x, force_y):
        max_f = float(self._max_agent_force)
        self._force_x = max(-max_f, min(max_f, float(force_x)))
        self._force_y = max(-max_f, min(max_f, float(force_y)))
        self._last_force_x = self._force_x
        self._last_force_y = self._force_y
    def _wind_force(self):
        phase = 2.0 * math.pi * self._step_count / max(1, int(self._wind_period))
        return (float(self._wind_amp) * math.sin(phase), 0.0)
    def step(self, time_step=None):
        dt = float(DEFAULT_SIMULATION_TIME_STEP if time_step is None else time_step)
        if self._barrier_remove_at_step is not None and self._step_count >= self._barrier_remove_at_step:
            self._barrier_remove_at_step = None
            if "barrier" in self._terrain_bodies:
                body = self._terrain_bodies.pop("barrier", None)
                if body is not None and body.world is not None:
                    self._world.DestroyBody(body)
        self._step_count += 1
        agent = self._terrain_bodies.get("agent")
        if agent is not None:
            self._agent_y_history.append(agent.position.y)
            if len(self._agent_y_history) > int(self._c_high_history):
                self._agent_y_history.pop(0)
        self._update_sequence()
        agent = self._terrain_bodies.get("agent")
        if agent is not None:
            x, y = agent.position.x, agent.position.y
            self._last_force_x = self._force_x
            self._last_force_y = self._force_y
            wx, wy = self._wind_force()
            rx, ry = self._repulsion_force(x, y)
            total_fx = self._force_x + wx + rx
            total_fy = self._force_y + wy + ry
            if total_fx != 0.0 or total_fy != 0.0:
                agent.ApplyForceToCenter((total_fx, total_fy), True)
            self._force_x = 0.0
            self._force_y = 0.0
        self._world.Step(dt, 10, 10)
    def get_terrain_bounds(self):
        return {
            "zones": dict(self._zones),
            "required_order": list(REQUIRED_ORDER),
            "trigger_stay_steps": int(self._trigger_stay_steps),
            "speed_cap_inside": float(self._speed_cap_inside),
            "cooldown_steps": int(self._cooldown_steps),
            "barrier_delay_steps": int(self._barrier_delay_steps),
            "barrier_x": float(self._barrier_x),
            "recent_a_for_b": int(self._recent_a_for_b),
            "recent_b_for_c": int(self._recent_b_for_c),
            "c_high_history": int(self._c_high_history),
            "c_required_max_y": float(self._c_required_max_y),
            "force_limit_inside": float(self._force_limit_inside),
            "repulsion_mag": float(self._repulsion_mag),
            "repulsion_range": float(self._repulsion_range),
            "repulsion_tangential_mag": float(self._repulsion_tangential_mag),
            "max_agent_force_per_axis": float(self._max_agent_force),
            "ground_friction": float(self._ground_friction),
            "ramp_friction": float(self._ramp_friction),
            "platform_friction": float(self._platform_friction),
            "agent_fixture_friction": float(self._agent_fixture_friction),
            "barrier_fixture_friction": float(self._barrier_fixture_friction),
        }
    def get_agent_body(self):
        return self._terrain_bodies.get("agent")
    def get_wrong_order(self):
        return self._wrong_order
    def get_repulsion_at_agent(self):
        agent = self._terrain_bodies.get("agent")
        if agent is None:
            return (0.0, 0.0, 0.0)
        x, y = agent.position.x, agent.position.y
        fx, fy = self._repulsion_force(x, y)
        mag = math.sqrt(fx * fx + fy * fy)
        return (fx, fy, mag)
    def get_barrier_status(self):
        active = self._barrier_remove_at_step is not None and self._step_count < self._barrier_remove_at_step
        if not active and self._barrier_remove_at_step is None:
            remaining = 0
        elif self._barrier_remove_at_step is not None:
            remaining = max(0, int(self._barrier_remove_at_step) - self._step_count)
        else:
            remaining = 0
        return {"active": active, "steps_until_open": remaining, "barrier_total_delay": int(self._barrier_delay_steps)}
    def get_cooldown_total(self):
        return int(self._cooldown_steps)
    def get_temporal_window_status(self):
        A_visited = self._last_step_in_A >= 0
        B_visited = self._last_step_in_B >= 0
        steps_since_A = self._step_count - self._last_step_in_A if A_visited else -1
        steps_since_B = self._step_count - self._last_step_in_B if B_visited else -1
        return {
            "A_visited": A_visited,
            "B_visited": B_visited,
            "steps_since_last_A": steps_since_A,
            "steps_since_last_B": steps_since_B,
            "window_A_to_B": int(self._recent_a_for_b),
            "window_B_to_C": int(self._recent_b_for_c),
        }
    def get_dwell_reset_stats(self):
        return {
            "zone_change": self._dwell_reset_zone_change,
            "speed": self._dwell_reset_speed,
            "force": self._dwell_reset_force,
            "blocked_temporal": self._dwell_blocked_temporal,
            "blocked_altitude": self._dwell_blocked_altitude,
            "blocked_cooldown": self._dwell_blocked_cooldown,
        }
    def get_last_applied_force(self):
        return (
            self._last_force_x,
            self._last_force_y,
            math.hypot(self._last_force_x, self._last_force_y),
        )
    def get_agent_y_history_stats(self):
        ch = int(self._c_high_history)
        relevant = self._agent_y_history[-ch:] if self._agent_y_history else []
        max_y = max(relevant) if relevant else 0.0
        return {
            "max_recent_y": max_y,
            "history_length": len(self._agent_y_history),
            "history_window": ch,
            "required_max_y": float(self._c_required_max_y),
        }
    def get_peak_values(self):
        return {
            "peak_speed": self._peak_speed_seen,
            "peak_force_applied": self._peak_force_applied,
        }
    def get_in_zone_force_limit(self):
        return float(self._force_limit_inside)
    def get_in_zone_speed_cap(self):
        return float(self._speed_cap_inside)
    def get_repulsion_params(self):
        return {
            "magnitude": float(self._repulsion_mag),
            "range": float(self._repulsion_range),
        }
    def get_trigger_stay_steps(self):
        return int(self._trigger_stay_steps)
    def get_agent_mass(self):
        return float(self._agent_mass)
    def get_max_agent_force(self):
        return float(self._max_agent_force)
    def get_c_high_history(self):
        return int(self._c_high_history)
    def get_c_required_max_y(self):
        return float(self._c_required_max_y)
