import math
import random

import Box2D

from Box2D import b2World, b2PolygonShape, b2CircleShape, b2FixtureDef, b2BodyDef, b2_dynamicBody

FPS = 60

TIME_STEP = 1.0 / FPS

VEL_ITERS, POS_ITERS = 10, 10

FORCE_HISTORY_CAP = 100

STATE_HISTORY_CAP = 300

MAX_STEPS = 250000

AGENT_MASS = 5.0

WHISKER_RANGE = 3.0

HOLD_STEPS = 5

LINEAR_DAMPING = 1.0

RESTITUTION = 0.1

MAZE_WALLS = [
    (0.0, 0.0, 20.0, 0.5),
    (0.0, 2.5, 20.0, 0.5),
    (0.0, 0.0, 0.5, 3.0),
    (20.0, 0.0, 0.5, 3.0),
    (5.0, 0.0, 0.2, 1.0),
    (9.0, 1.8, 0.2, 1.2),
    (14.0, 1.8, 0.2, 1.2),

]

WIND_OSCILLATION_AMP = 5.0

WIND_OSCILLATION_OMEGA = 0.05

SHEAR_WIND_REFERENCE_Y = 1.5

BACKWARD_FX_THRESHOLD = -34.0

BACKWARD_SPEED_MAX = 100.0

BACKWARD_STEPS_REQUIRED = HOLD_STEPS

STRUCTURAL_IMPULSE_SCALE_K = 25.0

ACTIVATION_X_MIN = 5.0

ACTIVATION_X_MAX = 10.0

SLIP_FRICTION = 0.5

ONEWAY_X = 10.2

ONEWAY_FORCE_RIGHT = 50.0

LOCK_GATE_X_MIN = 12.0

LOCK_GATE_X_MAX = 16.0

LOCK_GATE_FX = -1200.0

EXIT_X_MIN = 15.0

EXIT_Y_MIN = 0.5

EXIT_Y_MAX = 2.5

class Sandbox:
    def __init__(self, terrain_config: dict = None, physics_config: dict = None):
        if physics_config is None:
            physics_config = {}
        if terrain_config is None:
            terrain_config = {}
        self.physics_config = dict(physics_config)
        self.terrain_config = dict(terrain_config)
        simulation_seed = int(
            physics_config.get(
                "random_seed",
                terrain_config.get("target_rng_seed", 123),
            )
        )
        self._rng = random.Random(simulation_seed)
        g_val = physics_config.get("gravity", -9.8)
        if isinstance(g_val, (list, tuple)):
            g_y = float(g_val[1])
        else:
            g_y = float(g_val)
        self._world = b2World(gravity=(0, g_y))
        self._terrain_bodies = {}
        self._current_step = 0
        self._current_force_back = float(physics_config.get("current_force_back", 0.0))
        self._shear_wind_gradient = float(physics_config.get("shear_wind_gradient", 0.0))
        self._whisker_delay_steps = int(terrain_config.get("whisker_delay_steps", 0))
        self._position_delay_steps = int(terrain_config.get("position_delay_steps", 0))
        self._whisker_blind_front_x_lo = float(terrain_config.get("whisker_blind_front_x_lo", -999.0))
        self._whisker_blind_front_x_hi = float(terrain_config.get("whisker_blind_front_x_hi", -999.0))
        self._control_reversal_x_min = float(physics_config.get("control_reversal_x_min", -999.0))
        self._control_reversal_x_max = float(physics_config.get("control_reversal_x_max", -999.0))
        self._fluid_drag_x_min = float(physics_config.get("fluid_drag_x_min", -999.0))
        self._fluid_drag_x_max = float(physics_config.get("fluid_drag_x_max", -999.0))
        self._fluid_drag_coeff = float(physics_config.get("fluid_drag_coeff", 0.0))
        self._magnetic_floor_y_max = float(physics_config.get("magnetic_floor_y_max", -999.0))
        self._magnetic_floor_force = float(physics_config.get("magnetic_floor_force", 0.0))
        self._control_lag_steps = int(physics_config.get("control_lag_steps", 0))
        self._turbulence_intensity = float(physics_config.get("turbulence_intensity", 0.0))
        self._slip_friction = float(physics_config.get("slip_friction", SLIP_FRICTION))
        self._oneway_x = float(terrain_config.get("oneway_x", ONEWAY_X))
        self._oneway_force_right = float(physics_config.get("oneway_force_right", ONEWAY_FORCE_RIGHT))
        self._lock_gate_fx = float(physics_config.get("lock_gate_fx", LOCK_GATE_FX))
        self._lock_gate_x_min = float(physics_config.get("lock_gate_x_min", LOCK_GATE_X_MIN))
        self._lock_gate_x_max = float(physics_config.get("lock_gate_x_max", LOCK_GATE_X_MAX))
        self._activation_x_min = float(physics_config.get("activation_x_min", ACTIVATION_X_MIN))
        self._activation_x_max = float(physics_config.get("activation_x_max", ACTIVATION_X_MAX))
        self._wind_oscillation_amp = float(physics_config.get("wind_oscillation_amp", WIND_OSCILLATION_AMP))
        self._wind_oscillation_omega = float(physics_config.get("wind_oscillation_omega", WIND_OSCILLATION_OMEGA))
        self._shear_wind_reference_y = float(
            physics_config.get("shear_wind_reference_y", SHEAR_WIND_REFERENCE_Y)
        )
        self._structural_impulse_scale_k = float(
            physics_config.get(
                "structural_impulse_scale_k",
                physics_config.get("collision_velocity_limit", STRUCTURAL_IMPULSE_SCALE_K),
            )
        )
        self._backward_fx_threshold = float(
            physics_config.get("backward_fx_threshold", BACKWARD_FX_THRESHOLD)
        )
        self._backward_speed_max = float(
            physics_config.get("backward_speed_max", BACKWARD_SPEED_MAX)
        )
        self._backward_steps_required = int(
            physics_config.get("backward_steps_required", BACKWARD_STEPS_REQUIRED)
        )
        self._force_history = []
        self._last_requested_force_x = 0.0
        self._last_requested_force_y = 0.0
        self._behavioral_unlock = False
        self._backward_steps = 0
        self._max_backward_steps = 0
        self._is_destroyed = False
        self._destruction_reason = None
        self._destruction_step = None
        self._peak_collision_impulse = 0.0
        self._peak_collision_impulse_step = None
        self._first_activation_entry_step = None
        self._first_unlock_step = None
        self._first_exit_entry_step = None
        self._first_qualified_exit_step = None
        self._exit_hold_completion_step = None
        self._consecutive_exit_steps = 0
        self._max_consecutive_exit_steps = 0
        self._first_all_whiskers_max_step = None
        self._max_reported_x = 2.0
        self._max_reported_x_step = 0
        self._closest_exit_distance = max(0.0, EXIT_X_MIN - 2.0)
        self._closest_exit_distance_step = 0
        self._max_speed = 0.0
        self._max_speed_step = 0
        self._peak_requested_force_magnitude = 0.0
        self._peak_requested_force_step = 0
        self._create_maze(terrain_config)
        self._create_agent(terrain_config)
        p_init = (self._terrain_bodies["agent"].position.x, self._terrain_bodies["agent"].position.y)
        self._position_history = [p_init]
        self._whisker_readings_history = [tuple(self.get_whisker_readings())]
        self._force_x = 0.0
        self._force_y = 0.0
        self._last_turbulence_x = 0.0
        self._last_turbulence_y = 0.0
        self.MAX_STEPS = int(physics_config.get("max_steps", MAX_STEPS))
    def _create_maze(self, terrain_config: dict):
        walls = list(MAZE_WALLS)
        overrides = terrain_config.get("wall_overrides", {})
        for idx_str, val in overrides.items():
            walls[int(idx_str)] = val
        for i, (x, y, w, h) in enumerate(walls):
            body = self._world.CreateStaticBody(
                position=(x + w/2, y + h/2),
                shapes=b2PolygonShape(box=(w/2, h/2)),
            )
            body.fixtures[0].friction = self._slip_friction
            self._terrain_bodies[f"wall_{i}"] = body
    def _create_agent(self, terrain_config: dict):
        self._agent_radius = 0.2
        body_def = b2BodyDef(
            type=b2_dynamicBody,
            position=(2.0, 1.5),
            fixedRotation=True,
            linearDamping=LINEAR_DAMPING,
        )
        agent = self._world.CreateBody(body_def)
        shape = b2CircleShape(radius=self._agent_radius)
        fixture_def = b2FixtureDef(
            shape=shape,
            density=AGENT_MASS / (math.pi * self._agent_radius**2),
            friction=self._slip_friction,
            restitution=RESTITUTION,
        )
        agent.CreateFixture(fixture_def)
        self._terrain_bodies["agent"] = agent
        class MyContactListener(Box2D.b2ContactListener):
            def __init__(self, sandbox):
                super().__init__()
                self.sandbox = sandbox
            def PostSolve(self, contact, impulse):
                for i in range(contact.manifold.pointCount):
                    impulse_value = float(impulse.normalImpulses[i])
                    if impulse_value > self.sandbox._peak_collision_impulse:
                        self.sandbox._peak_collision_impulse = impulse_value
                        self.sandbox._peak_collision_impulse_step = self.sandbox._current_step + 1
                    if impulse_value > self.sandbox._structural_impulse_scale_k * AGENT_MASS:
                        self.sandbox._is_destroyed = True
                        if self.sandbox._destruction_step is None:
                            self.sandbox._destruction_step = self.sandbox._current_step + 1
                        self.sandbox._destruction_reason = f"Structural Failure: Collision impulse {impulse_value:.1f} exceeded limit."
        self._world.contactListener = MyContactListener(self)
    def _raycast(self, p1, p2, ignore_body):
        class RayCastCallback(Box2D.b2RayCastCallback):
            def __init__(self, ignore):
                super().__init__()
                self.ignore = ignore
                self.hit_fraction = 1.0
            def ReportFixture(self, fixture, point, normal, fraction):
                if fixture.body == self.ignore:
                    return -1
                self.hit_fraction = fraction
                return 0
        callback = RayCastCallback(ignore_body)
        self._world.RayCast(callback, p1, p2)
        return callback.hit_fraction
    def get_agent_position(self):
        agent = self._terrain_bodies.get("agent")
        if agent is None:
            return (0.0, 0.0)
        p = (agent.position.x, agent.position.y)
        delay = max(0, self._position_delay_steps)
        if delay > 0:
            if len(self._position_history) > delay:
                return self._position_history[-(delay + 1)]
            return self._position_history[0]
        return p
    def get_agent_velocity(self):
        agent = self._terrain_bodies.get("agent")
        if agent is None:
            return (0.0, 0.0)
        return (agent.linearVelocity.x, agent.linearVelocity.y)
    def get_whisker_readings(self):
        agent = self._terrain_bodies.get("agent")
        if agent is None:
            return [WHISKER_RANGE] * 3
        x, y = agent.position.x, agent.position.y
        if self._whisker_blind_front_x_lo <= x <= self._whisker_blind_front_x_hi:
            return [WHISKER_RANGE] * 3
        delay = max(0, self._whisker_delay_steps)
        if delay > 0:
            if len(self._whisker_readings_history) > delay:
                return list(self._whisker_readings_history[-(delay + 1)])
            return list(self._whisker_readings_history[0])
        r = WHISKER_RANGE
        directions = [(1, 0), (0, 1), (0, -1)]
        out = []
        for dx, dy in directions:
            p2 = (x + dx * r, y + dy * r)
            frac = self._raycast((x, y), p2, agent)
            out.append(frac * r)
        return out
    def apply_agent_force(self, force_x, force_y):
        requested_fx = float(force_x)
        requested_fy = float(force_y)
        self._last_requested_force_x = requested_fx
        self._last_requested_force_y = requested_fy
        requested_magnitude = math.hypot(requested_fx, requested_fy)
        if requested_magnitude > self._peak_requested_force_magnitude:
            self._peak_requested_force_magnitude = requested_magnitude
            self._peak_requested_force_step = self._current_step
        self._force_history.append((requested_fx, requested_fy))
        if len(self._force_history) > FORCE_HISTORY_CAP:
            self._force_history.pop(0)
        delay = max(0, self._control_lag_steps)
        if delay > 0 and len(self._force_history) > delay:
            fx, fy = self._force_history[-(delay + 1)]
        else:
            fx, fy = requested_fx, requested_fy
        self._force_x, self._force_y = fx, fy
    def _update_diagnostics(self):
        agent = self._terrain_bodies.get("agent")
        if agent is None:
            return
        reported_x, reported_y = self.get_agent_position()
        speed = math.hypot(float(agent.linearVelocity.x), float(agent.linearVelocity.y))
        if self._activation_x_min <= reported_x <= self._activation_x_max:
            if self._first_activation_entry_step is None:
                self._first_activation_entry_step = self._current_step
        if self._behavioral_unlock and self._first_unlock_step is None:
            self._first_unlock_step = self._current_step
        if self.has_reached_exit() and self._first_exit_entry_step is None:
            self._first_exit_entry_step = self._current_step
        if self._behavioral_unlock and self.has_reached_exit():
            self._consecutive_exit_steps += 1
            self._max_consecutive_exit_steps = max(
                self._max_consecutive_exit_steps, self._consecutive_exit_steps
            )
            if self._first_qualified_exit_step is None:
                self._first_qualified_exit_step = self._current_step
            if (
                self._exit_hold_completion_step is None
                and self._consecutive_exit_steps >= self._backward_steps_required
            ):
                self._exit_hold_completion_step = self._current_step
        else:
            self._consecutive_exit_steps = 0
        if reported_x > self._max_reported_x:
            self._max_reported_x = reported_x
            self._max_reported_x_step = self._current_step
        exit_dx = max(0.0, EXIT_X_MIN - reported_x)
        exit_dy = 0.0
        if reported_y < EXIT_Y_MIN:
            exit_dy = EXIT_Y_MIN - reported_y
        elif reported_y > EXIT_Y_MAX:
            exit_dy = reported_y - EXIT_Y_MAX
        exit_distance = math.hypot(exit_dx, exit_dy)
        if exit_distance < self._closest_exit_distance:
            self._closest_exit_distance = exit_distance
            self._closest_exit_distance_step = self._current_step
        if speed > self._max_speed:
            self._max_speed = speed
            self._max_speed_step = self._current_step
        whiskers = self.get_whisker_readings()
        if (
            self._first_all_whiskers_max_step is None
            and whiskers
            and all(abs(float(value) - WHISKER_RANGE) <= 1e-6 for value in whiskers)
        ):
            self._first_all_whiskers_max_step = self._current_step
    def step(self, time_step):
        if self._is_destroyed:
            self._world.Step(time_step, VEL_ITERS, POS_ITERS)
            self._current_step += 1
            self._update_diagnostics()
            return
        agent = self._terrain_bodies.get("agent")
        if agent is not None:
            x, y = agent.position.x, agent.position.y
            px, py = self.get_agent_position()
            vx, vy = agent.linearVelocity.x, agent.linearVelocity.y
            speed = math.sqrt(vx * vx + vy * vy)
            self._position_history.append((x, y))
            if len(self._position_history) > STATE_HISTORY_CAP:
                self._position_history.pop(0)
            r = WHISKER_RANGE
            directions = [(1, 0), (0, 1), (0, -1)]
            true_whiskers = []
            for dx, dy in directions:
                p2 = (x + dx * r, y + dy * r)
                frac = self._raycast((x, y), p2, agent)
                true_whiskers.append(frac * r)
            self._whisker_readings_history.append(tuple(true_whiskers))
            if len(self._whisker_readings_history) > STATE_HISTORY_CAP:
                self._whisker_readings_history.pop(0)
            if (
                self._activation_x_min <= px <= self._activation_x_max
                and self._force_x < self._backward_fx_threshold
                and speed < self._backward_speed_max
            ):
                self._backward_steps += 1
                self._max_backward_steps = max(
                    self._max_backward_steps, self._backward_steps
                )
                if self._backward_steps >= self._backward_steps_required:
                    self._behavioral_unlock = True
            else:
                self._backward_steps = 0
            force_x_applied = self._force_x
            force_y_applied = self._force_y
            if self._control_reversal_x_min <= px <= self._control_reversal_x_max:
                force_x_applied *= -1.0
            agent.ApplyForceToCenter((force_x_applied, force_y_applied), True)
            if self._fluid_drag_x_min <= px <= self._fluid_drag_x_max:
                drag_x = -self._fluid_drag_coeff * vx * abs(vx)
                drag_y = -self._fluid_drag_coeff * vy * abs(vy)
                agent.ApplyForceToCenter((drag_x, drag_y), True)
            if y < self._magnetic_floor_y_max:
                agent.ApplyForceToCenter((0.0, self._magnetic_floor_force), True)
            if self._turbulence_intensity > 0:
                tx = (self._rng.random() - 0.5) * self._turbulence_intensity
                ty_turb = (self._rng.random() - 0.5) * self._turbulence_intensity
                agent.ApplyForceToCenter((tx, ty_turb), True)
                self._last_turbulence_x = tx
                self._last_turbulence_y = ty_turb
            else:
                self._last_turbulence_x = 0.0
                self._last_turbulence_y = 0.0
            osc = self._wind_oscillation_amp * math.sin(self._wind_oscillation_omega * self._current_step)
            wind_x = (
                -self._current_force_back
                + self._shear_wind_gradient * (y - self._shear_wind_reference_y)
                + osc
            )
            agent.ApplyForceToCenter((wind_x, 0), True)
            if px > self._oneway_x:
                agent.ApplyForceToCenter((self._oneway_force_right, 0), True)
            if not self._behavioral_unlock and self._lock_gate_x_min <= px <= self._lock_gate_x_max:
                agent.ApplyForceToCenter((self._lock_gate_fx, 0.0), True)
        self._world.Step(time_step, VEL_ITERS, POS_ITERS)
        self._current_step += 1
        self._update_diagnostics()
    def get_metrics(self):
        agent = self._terrain_bodies.get("agent")
        if agent is None:
            return {}
        px, py = self.get_agent_position()
        return {"x": px, "y": py, "unlocked": self._behavioral_unlock, "step": self._current_step}
    def get_agent_body(self): return self._terrain_bodies.get("agent")
    def get_terrain_bounds(self):
        return {
            "x_min": 0.0,
            "x_max": 20.0,
            "y_min": 0.0,
            "y_max": 3.0,
            "exit_x_min": EXIT_X_MIN,
            "exit_y_min": EXIT_Y_MIN,
            "exit_y_max": EXIT_Y_MAX,
        }
    def get_agent_components(self):
        return {
            "agent": self.get_agent_body(),
            "exit_x_min": EXIT_X_MIN,
            "exit_y_min": EXIT_Y_MIN,
            "exit_y_max": EXIT_Y_MAX,
        }
    def has_reached_exit(self):
        if self._is_destroyed:
            return False
        b = self.get_terrain_bounds()
        ex, ey0, ey1 = b["exit_x_min"], b["exit_y_min"], b["exit_y_max"]
        x, y = self.get_agent_position()
        return x >= ex and ey0 <= y <= ey1
    def get_whisker_max_range(self): return WHISKER_RANGE
    def is_destroyed(self): return self._is_destroyed
    def get_force_ledger(self):
        agent = self._terrain_bodies.get("agent")
        if agent is None:
            return {}
        return {
            "requested_force": {
                "fx": self._last_requested_force_x,
                "fy": self._last_requested_force_y,
            },
            "unlock_evaluated_force": {
                "fx": float(self._force_x),
                "fy": float(self._force_y),
            },
            "agent_destroyed": self._is_destroyed,
            "destruction_reason": self._destruction_reason,
        }
    def get_unlock_condition_status(self):
        agent = self._terrain_bodies.get("agent")
        if agent is None:
            return {"error": "No agent"}
        px, py = self.get_agent_position()
        vx, vy = agent.linearVelocity.x, agent.linearVelocity.y
        speed = (vx * vx + vy * vy) ** 0.5
        x_in_act = self._activation_x_min <= px <= self._activation_x_max
        x_margin = 0.0
        if px < self._activation_x_min:
            x_margin = px - self._activation_x_min
        elif px > self._activation_x_max:
            x_margin = px - self._activation_x_max
        fx_ok = self._force_x < self._backward_fx_threshold
        fx_margin = self._force_x - self._backward_fx_threshold
        speed_ok = speed < self._backward_speed_max
        speed_margin = speed - self._backward_speed_max
        all_ok = x_in_act and fx_ok and speed_ok
        return {
            "unlocked": self._behavioral_unlock,
            "consecutive_count": self._backward_steps,
            "required_consecutive": self._backward_steps_required,
            "conditions": [
                {
                    "name": "reported_x_in_activation_zone",
                    "value": px,
                    "zone": [self._activation_x_min, self._activation_x_max],
                    "pass": x_in_act,
                    "margin": x_margin,
                },
                {
                    "name": "commanded_fx_below_threshold",
                    "value": self._force_x,
                    "limit": self._backward_fx_threshold,
                    "pass": fx_ok,
                    "margin": fx_margin,
                },
                {
                    "name": "physical_speed_below_max",
                    "value": speed,
                    "limit": self._backward_speed_max,
                    "pass": speed_ok,
                    "margin": speed_margin,
                },
            ],
            "all_conditions_met": all_ok,
        }
    def get_whisker_health(self):
        if self._terrain_bodies.get("agent") is None:
            return {"error": "No agent"}
        readings = [float(value) for value in self.get_whisker_readings()]
        return {
            "readings": readings,
            "max_range_m": WHISKER_RANGE,
            "all_readings_at_max": all(
                abs(value - WHISKER_RANGE) <= 1e-6 for value in readings
            ),
        }
    def get_wall_clearance_map(self):
        agent = self._terrain_bodies.get("agent")
        if agent is None:
            return {"error": "No agent"}
        x, y = self.get_agent_position()
        floor = self._terrain_bodies.get("wall_0")
        ceiling = self._terrain_bodies.get("wall_1")
        arena_y_min = 0.5
        arena_y_max = 2.5
        if floor is not None and floor.fixtures:
            floor_vertices = getattr(floor.fixtures[0].shape, "vertices", ())
            if floor_vertices:
                arena_y_min = max(
                    float(floor.position.y + vertex[1])
                    for vertex in floor_vertices
                )
        if ceiling is not None and ceiling.fixtures:
            ceiling_vertices = getattr(ceiling.fixtures[0].shape, "vertices", ())
            if ceiling_vertices:
                arena_y_max = min(
                    float(ceiling.position.y + vertex[1])
                    for vertex in ceiling_vertices
                )
        walls = []
        for idx in [4, 5, 6]:
            body = self._terrain_bodies.get(f"wall_{idx}")
            if body is None:
                continue
            wx = float(body.position.x)
            wy = float(body.position.y)
            vertices = [
                vertex
                for fixture in body.fixtures
                for vertex in getattr(fixture.shape, "vertices", ())
            ]
            if not vertices:
                continue
            wall_x_min = min(float(vertex[0] + wx) for vertex in vertices)
            wall_x_max = max(float(vertex[0] + wx) for vertex in vertices)
            wall_y_min = min(float(vertex[1] + wy) for vertex in vertices)
            wall_y_max = max(float(vertex[1] + wy) for vertex in vertices)
            gap_above_y = [wall_y_max, arena_y_max]
            gap_above_exists = gap_above_y[1] > gap_above_y[0]
            gap_above_size = gap_above_y[1] - gap_above_y[0] if gap_above_exists else 0.0
            gap_below_y = [arena_y_min, wall_y_min]
            gap_below_exists = gap_below_y[1] > gap_below_y[0]
            gap_below_size = gap_below_y[1] - gap_below_y[0] if gap_below_exists else 0.0
            agent_behind_wall = x < wall_x_min
            agent_past_wall = x > wall_x_max
            agent_at_wall_x = wall_x_min <= x <= wall_x_max
            agent_above_wall = y > wall_y_max
            agent_below_wall = y < wall_y_min
            clearance_above = max(0.0, wall_y_max + self._agent_radius - y)
            clearance_below = max(0.0, y - (wall_y_min - self._agent_radius))
            dist_to_wall_x = wall_x_min - x if x < wall_x_min else 0.0
            walls.append({
                "wall_index": idx,
                "position": {"x_min": wall_x_min, "x_max": wall_x_max, "y_min": wall_y_min, "y_max": wall_y_max},
                "dimensions": {"width": wall_x_max - wall_x_min, "height": wall_y_max - wall_y_min},
                "agent_relative": {
                    "behind_wall": agent_behind_wall,
                    "at_wall_x": agent_at_wall_x,
                    "past_wall": agent_past_wall,
                    "above_wall": agent_above_wall,
                    "below_wall": agent_below_wall,
                    "distance_to_wall_x": dist_to_wall_x,
                },
                "gaps": {
                    "above": {"exists": gap_above_exists, "y_range": gap_above_y, "size_m": gap_above_size},
                    "below": {"exists": gap_below_exists, "y_range": gap_below_y, "size_m": gap_below_size},
                },
                "clearance_needed_m": {
                    "to_pass_above": clearance_above,
                    "to_pass_below": clearance_below,
                },
            })
        return {
            "agent_y": y,
            "arena_limits": {"y_min": arena_y_min, "y_max": arena_y_max},
            "walls": walls,
        }
    def get_control_lag_info(self):
        return {
            "requested_force": [
                self._last_requested_force_x, self._last_requested_force_y
            ],
            "unlock_evaluated_force": [float(self._force_x), float(self._force_y)],
            "requested_matches_evaluated": (
                abs(self._last_requested_force_x - self._force_x) <= 1e-9
                and abs(self._last_requested_force_y - self._force_y) <= 1e-9
            ),
        }
    def get_exit_dwell_status(self):
        return {
            "consecutive_steps": self._consecutive_exit_steps,
            "max_consecutive_steps": self._max_consecutive_exit_steps,
            "first_qualified_step": self._first_qualified_exit_step,
            "completion_step": self._exit_hold_completion_step,
            "required_steps": self._backward_steps_required,
        }
    def get_wind_params(self):
        return {
            "current_step": self._current_step,
            "parameters_exposed": False,
        }
    def get_diagnostic_timeline(self):
        return {
            "first_activation_entry_step": self._first_activation_entry_step,
            "first_unlock_step": self._first_unlock_step,
            "max_unlock_condition_streak": self._max_backward_steps,
            "first_exit_entry_step": self._first_exit_entry_step,
            "first_all_whiskers_max_step": self._first_all_whiskers_max_step,
            "destruction_step": self._destruction_step,
            "max_reported_x_m": self._max_reported_x,
            "max_reported_x_step": self._max_reported_x_step,
            "closest_exit_distance_m": self._closest_exit_distance,
            "closest_exit_distance_step": self._closest_exit_distance_step,
            "max_speed_mps": self._max_speed,
            "max_speed_step": self._max_speed_step,
            "peak_requested_force_n": self._peak_requested_force_magnitude,
            "peak_requested_force_step": self._peak_requested_force_step,
            "peak_collision_impulse_ns": self._peak_collision_impulse,
            "peak_collision_impulse_step": self._peak_collision_impulse_step,
        }
    def get_structural_impulse_limit(self):
        return self._structural_impulse_scale_k * AGENT_MASS
    def get_destruction_reason(self): return self._destruction_reason
    @property
    def world(self): return self._world
