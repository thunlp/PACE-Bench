import math

import Box2D

from Box2D.b2 import world, polygonShape, staticBody, dynamicBody

TIME_STEP = 1.0 / 60.0

def default_magnets():
    return [
        (12.0, 4.0, -300.0),
        (12.0, 5.0, -300.0),
        (12.0, 6.0, -300.0),
        (12.0, 7.0, -300.0),
        (12.0, 8.0, -280.0),
        (12.0, 8.3, -260.0),
        (11.0, 9.7, -200.0),
        (13.0, 9.7, -200.0),
        (15.0, 9.7, -200.0),
        (17.0, 9.7, -200.0),
        (19.0, 9.7, -200.0),
        (21.0, 9.7, -180.0),
        (15.0, 9.0, -250.0, 230.0, 0.12),
        (20.0, 9.0, -350.0, 330.0, 0.15, 3.14159),
        (19.0, 3.0, 160.0),
        (21.0, 3.5, 130.0),
        (24.0, 5.0, -190.0),
        (24.0, 8.2, -180.0),
        (24.0, 6.6, -180.0, 160.0, 0.165),
        (26.0, 5.5, -130.0),
        (27.0, 9.5, -120.0),
        (29.5, 7.5, 95.0),
    ]

class Sandbox:
    BODY_START_X = 8.0
    BODY_START_Y = 5.0
    TARGET_X_MIN = 28.0
    TARGET_X_MAX = 32.0
    TARGET_Y_MIN = 6.0
    TARGET_Y_MAX = 9.0
    PIT_X_MIN = 16.0
    PIT_X_MAX = 24.0
    PIT_Y_MAX = 5.5
    MAX_STEPS = 10000
    MAGNET_R_MIN = 0.5
    def __init__(self, *, terrain_config=None, physics_config=None):
        terrain_config = terrain_config or {}
        physics_config = physics_config or {}
        self._terrain_config = dict(terrain_config)
        self._physics_config = dict(physics_config)
        gravity = tuple(physics_config.get("gravity", (0, -10)))
        self._linear_damping = float(physics_config.get("linear_damping", 0.28))
        self._angular_damping = float(physics_config.get("angular_damping", 0.15))
        self._world = world(gravity=gravity, doSleep=True)
        self._terrain_bodies = {}
        self._pending_thrust = (0.0, 0.0)
        self._magnets = list(terrain_config.get("magnets", default_magnets()))
        self._step_count = 0
        self._max_thrust_magnitude = float(terrain_config.get("max_thrust", 165.0))
        self._body_start_x = float(terrain_config.get("body_start_x", self.BODY_START_X))
        self._body_start_y = float(terrain_config.get("body_start_y", self.BODY_START_Y))
        self.world = self._world
        self.bodies = []
        self.joints = []
        self._create_terrain(terrain_config)
        self._create_body(terrain_config)
        self._body_mass = 30.0 * 0.8 * 0.4
        self._reset_forensic_state()
    def _create_terrain(self, terrain_config: dict):
        ground_length = 45.0
        ground_height = 1.0
        ground = self._world.CreateStaticBody(
            position=(ground_length / 2, ground_height / 2),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(ground_length / 2, ground_height / 2)),
                friction=0.4,
            ),
        )
        self._terrain_bodies["ground"] = ground
        self._ground_y = ground_height
    def _create_body(self, terrain_config: dict):
        sx, sy = self._body_start_x, self._body_start_y
        w, h = 0.8, 0.4
        body = self._world.CreateDynamicBody(
            position=(sx, sy),
            fixtures=Box2D.b2FixtureDef(
                shape=polygonShape(box=(w / 2, h / 2)),
                density=30.0,
                friction=0.3,
                restitution=0.1,
            ),
        )
        body.linearDamping = self._linear_damping
        body.angularDamping = self._angular_damping
        self._terrain_bodies["body"] = body
    def step(self, time_step):
        if not math.isclose(float(time_step), TIME_STEP, rel_tol=0.0, abs_tol=1e-12):
            raise ValueError(f"E_05 requires a fixed time step of {TIME_STEP}")
        self._step_count += 1
        body = self._terrain_bodies.get("body")
        if body:
            bx, by = body.position.x, body.position.y
            vel = (body.linearVelocity.x, body.linearVelocity.y)
            if not hasattr(self, "_forensic") or self._forensic is None:
                self._reset_forensic_state()
            self._update_forensic(self._step_count, (bx, by), vel)
            net_mag_fx, net_mag_fy = 0.0, 0.0
            for m in self._magnets:
                if len(m) == 3:
                    mx, my, strength = m[0], m[1], m[2]
                else:
                    mx, my, base, amp, omega = m[0], m[1], m[2], m[3], m[4]
                    phase = m[5] if len(m) >= 6 else 0.0
                    strength = base + amp * math.sin(self._step_count * omega + phase)
                dx = mx - bx
                dy = my - by
                r = math.sqrt(dx * dx + dy * dy) + 1e-6
                r = max(r, self.MAGNET_R_MIN)
                scale = strength / (r * r * r)
                fx = scale * dx
                fy = scale * dy
                net_mag_fx += fx
                net_mag_fy += fy
                body.ApplyForceToCenter((fx, fy), wake=True)
            self._forensic["current_net_magnetic_force"] = (net_mag_fx, net_mag_fy)
            mag_total = math.hypot(net_mag_fx, net_mag_fy)
            if mag_total > self._forensic["peak_magnetic_force_magnitude"]:
                self._forensic["peak_magnetic_force_magnitude"] = mag_total
            tx, ty = self._pending_thrust
            thrust_mag = math.sqrt(tx * tx + ty * ty)
            if thrust_mag > self._forensic["peak_requested_thrust"]:
                self._forensic["peak_requested_thrust"] = thrust_mag
            if thrust_mag > self._max_thrust_magnitude:
                self._forensic["thrust_clipped_steps"] += 1
                if self._forensic["first_thrust_clip_step"] is None:
                    self._forensic["first_thrust_clip_step"] = self._step_count
                scale = self._max_thrust_magnitude / thrust_mag
                tx, ty = tx * scale, ty * scale
            applied_magnitude = math.hypot(tx, ty)
            if applied_magnitude > self._forensic["peak_applied_thrust"]:
                self._forensic["peak_applied_thrust"] = applied_magnitude
            self._forensic["current_applied_thrust"] = (tx, ty)
            body.ApplyForceToCenter((tx, ty), wake=True)
        self._pending_thrust = (0.0, 0.0)
        self._forensic["_prev_body_pos"] = (bx, by) if body else None
        self._forensic["_prev_net_magnetic_force"] = self._forensic["current_net_magnetic_force"]
        self._forensic["_prev_applied_thrust"] = self._forensic["current_applied_thrust"]
        self._world.Step(time_step, 10, 10)
    def apply_thrust(self, fx, fy):
        self._pending_thrust = (float(fx), float(fy))
    def get_body_position(self):
        body = self._terrain_bodies.get("body")
        if body:
            return (body.position.x, body.position.y)
        return None
    def get_body_velocity(self):
        body = self._terrain_bodies.get("body")
        if body:
            return (body.linearVelocity.x, body.linearVelocity.y)
        return None
    def get_step_count(self):
        return self._step_count
    def get_terrain_bounds(self):
        return {
            "ground_y": self._ground_y,
            "body_start": {"x": self._body_start_x, "y": self._body_start_y},
            "target_zone": {
                "x_min": self.TARGET_X_MIN,
                "x_max": self.TARGET_X_MAX,
                "y_min": self.TARGET_Y_MIN,
                "y_max": self.TARGET_Y_MAX,
            },
            "pit_zone": {
                "x_min": self.PIT_X_MIN,
                "x_max": self.PIT_X_MAX,
                "y_max": self.PIT_Y_MAX,
            },
        }
    def _reset_forensic_state(self):
        self._forensic = {
            "max_body_x": None,
            "min_body_x": None,
            "max_body_y": None,
            "min_body_y": None,
            "max_speed": 0.0,
            "total_dx": 0.0,
            "total_dy": 0.0,
            "steps_near_ceiling": 0,
            "steps_in_pit_zone": 0,
            "steps_in_ground_zone": 0,
            "steps_stationary": 0,
            "_last_stationary_check_pos": None,
            "_stationary_run": 0,
            "first_ceiling_entry_step": None,
            "first_pit_entry_step": None,
            "first_ground_entry_step": None,
            "max_x_reached": None,
            "vertical_zone_samples": {},
            "current_net_magnetic_force": (0.0, 0.0),
            "current_applied_thrust": (0.0, 0.0),
            "peak_magnetic_force_magnitude": 0.0,
            "cumulative_magnetic_work": 0.0,
            "cumulative_thrust_work": 0.0,
            "body_mass": self._body_mass,
            "temporal_events": [],
            "velocity_reversal_events": [],
            "progress_plateau_end_step": None,
            "progress_plateau_x": None,
            "progress_plateau_duration": 0,
            "_last_progress_x": None,
            "_progress_stall_counter": 0,
            "peak_vertical_accel": 0.0,
            "peak_requested_thrust": 0.0,
            "peak_applied_thrust": 0.0,
            "thrust_clipped_steps": 0,
            "first_thrust_clip_step": None,
            "_prev_body_vel": None,
            "_prev_body_pos": None,
            "_prev_net_magnetic_force": (0.0, 0.0),
            "_prev_applied_thrust": (0.0, 0.0),
        }
    def _update_forensic(self, step_count, body_pos, body_vel):
        f = self._forensic
        x, y = body_pos
        vx, vy = body_vel
        speed = math.sqrt(vx * vx + vy * vy)
        if f["max_body_x"] is None or x > f["max_body_x"]:
            f["max_body_x"] = x
        if f["min_body_x"] is None or x < f["min_body_x"]:
            f["min_body_x"] = x
        if f["max_body_y"] is None or y > f["max_body_y"]:
            f["max_body_y"] = y
        if f["min_body_y"] is None or y < f["min_body_y"]:
            f["min_body_y"] = y
        if speed > f["max_speed"]:
            f["max_speed"] = speed
        sx = self._body_start_x
        sy = self._body_start_y
        f["total_dx"] = x - sx
        f["total_dy"] = y - sy
        if f["max_x_reached"] is None or x > f["max_x_reached"]:
            f["max_x_reached"] = x
        CEILING_THRESHOLD = 9.7
        GROUND_THRESHOLD = 1.0
        zones_hit = set()
        if y > CEILING_THRESHOLD:
            f["steps_near_ceiling"] += 1
            zones_hit.add("ceiling")
            if f["first_ceiling_entry_step"] is None:
                f["first_ceiling_entry_step"] = step_count
                f["temporal_events"].append(
                    {"step": step_count, "event": "ceiling_entry",
                     "body_x": round(x, 3), "body_y": round(y, 3)}
                )
        if (self.PIT_X_MIN <= x <= self.PIT_X_MAX) and (y < self.PIT_Y_MAX):
            f["steps_in_pit_zone"] += 1
            zones_hit.add("pit")
            if f["first_pit_entry_step"] is None:
                f["first_pit_entry_step"] = step_count
                f["temporal_events"].append(
                    {"step": step_count, "event": "pit_entry",
                     "body_x": round(x, 3), "body_y": round(y, 3)}
                )
        if y < GROUND_THRESHOLD:
            f["steps_in_ground_zone"] += 1
            zones_hit.add("ground")
            if f["first_ground_entry_step"] is None:
                f["first_ground_entry_step"] = step_count
                f["temporal_events"].append(
                    {"step": step_count, "event": "ground_entry",
                     "body_x": round(x, 3), "body_y": round(y, 3)}
                )
        if not zones_hit:
            zones_hit.add("corridor")
        for z in zones_hit:
            f["vertical_zone_samples"][z] = f["vertical_zone_samples"].get(z, 0) + 1
        if speed < 0.05:
            f["_stationary_run"] += 1
        else:
            f["_stationary_run"] = 0
        if f["_stationary_run"] >= 60:
            f["steps_stationary"] += 1
        prev_vel = f["_prev_body_vel"]
        if prev_vel is not None:
            pvx, pvy = prev_vel
            if (vx * pvx < 0) and abs(vx) > 0.01 and abs(pvx) > 0.01:
                f["velocity_reversal_events"].append(
                    {"step": step_count, "axis": "x",
                     "from": round(pvx, 3), "to": round(vx, 3),
                     "body_x": round(x, 3), "body_y": round(y, 3)}
                )
            if (vy * pvy < 0) and abs(vy) > 0.01 and abs(pvy) > 0.01:
                f["velocity_reversal_events"].append(
                    {"step": step_count, "axis": "y",
                     "from": round(pvy, 3), "to": round(vy, 3),
                     "body_x": round(x, 3), "body_y": round(y, 3)}
                )
            ay = (vy - pvy) / (1.0 / 60.0)
            if abs(ay) > f["peak_vertical_accel"]:
                f["peak_vertical_accel"] = abs(ay)
        f["_prev_body_vel"] = (vx, vy)
        prev_pos = f["_prev_body_pos"]
        if prev_pos is not None:
            px, py = prev_pos
            dx, dy = x - px, y - py
            pmf = f["_prev_net_magnetic_force"]
            pt = f["_prev_applied_thrust"]
            f["cumulative_magnetic_work"] += pmf[0] * dx + pmf[1] * dy
            f["cumulative_thrust_work"] += pt[0] * dx + pt[1] * dy
        last_px = f["_last_progress_x"]
        f["_last_progress_x"] = x
        if last_px is not None and x <= last_px + 0.001:
            f["_progress_stall_counter"] += 1
        else:
            f["_progress_stall_counter"] = 0
        if f["_progress_stall_counter"] >= 300:
            f["progress_plateau_duration"] = max(
                f["progress_plateau_duration"], f["_progress_stall_counter"]
            )
            if f["progress_plateau_end_step"] is None:
                f["progress_plateau_end_step"] = step_count
                f["progress_plateau_x"] = round(x, 3)
                f["progress_plateau_duration"] = f["_progress_stall_counter"]
                f["temporal_events"].append(
                    {"step": step_count, "event": "progress_plateau_detected",
                     "body_x": round(x, 3), "body_y": round(y, 3),
                     "stall_duration_steps": f["_progress_stall_counter"]}
                )
    def _get_forensic_summary(self):
        return dict(self._forensic)
    def _get_magnetic_force_summary(self):
        f = self._forensic
        net_fx, net_fy = f.get("current_net_magnetic_force", (0.0, 0.0))
        return {
            "net_magnetic_force_x": round(net_fx, 3),
            "net_magnetic_force_y": round(net_fy, 3),
            "net_magnetic_force_magnitude": round(math.sqrt(net_fx * net_fx + net_fy * net_fy), 3),
            "peak_magnetic_force_magnitude": round(f.get("peak_magnetic_force_magnitude", 0.0), 3),
        }
    def _get_energy_summary(self):
        f = self._forensic
        body = self._terrain_bodies.get("body")
        ke = 0.0
        if body:
            vx, vy = body.linearVelocity.x, body.linearVelocity.y
            ke = 0.5 * self._body_mass * (vx * vx + vy * vy)
        cum_mag = f.get("cumulative_magnetic_work", 0.0)
        cum_thrust = f.get("cumulative_thrust_work", 0.0)
        return {
            "kinetic_energy": round(ke, 3),
            "cumulative_magnetic_work": round(cum_mag, 3),
            "cumulative_thrust_work": round(cum_thrust, 3),
            "body_mass": round(self._body_mass, 3),
        }
    def _get_temporal_events(self):
        return list(self._forensic.get("temporal_events", []))
    def _get_velocity_reversals(self):
        return list(self._forensic.get("velocity_reversal_events", []))
    def _get_force_decomposition(self):
        f = self._forensic
        net_mag_fx, net_mag_fy = f.get("current_net_magnetic_force", (0.0, 0.0))
        tx, ty = f.get("current_applied_thrust", (0.0, 0.0))
        gx, gy = self._world.gravity
        grav_fx = gx * self._body_mass
        grav_fy = gy * self._body_mass
        required_hover = self._body_mass * abs(gy)
        return {
            "thrust_applied_x": round(tx, 3),
            "thrust_applied_y": round(ty, 3),
            "thrust_magnitude": round(math.sqrt(tx * tx + ty * ty), 3),
            "net_magnetic_force_x": round(net_mag_fx, 3),
            "net_magnetic_force_y": round(net_mag_fy, 3),
            "gravity_force_x": round(grav_fx, 3),
            "gravity_force_y": round(grav_fy, 3),
            "required_hover_thrust": round(required_hover, 3),
            "peak_requested_thrust": round(f.get("peak_requested_thrust", 0.0), 3),
            "peak_applied_thrust": round(f.get("peak_applied_thrust", 0.0), 3),
            "thrust_clipped_steps": int(f.get("thrust_clipped_steps", 0)),
            "first_thrust_clip_step": f.get("first_thrust_clip_step"),
        }
    def _get_progress_plateau_info(self):
        f = self._forensic
        return {
            "progress_plateau_end_step": f.get("progress_plateau_end_step"),
            "progress_plateau_x": f.get("progress_plateau_x"),
            "progress_plateau_duration": f.get("progress_plateau_duration", 0),
        }
    def _get_peak_vertical_acceleration(self):
        return self._forensic.get("peak_vertical_accel", 0.0)
    def get_physics_params(self):
        gx, gy = self._world.gravity
        return {
            "gravity_x": gx,
            "gravity_y": gy,
            "linear_damping": self._linear_damping,
            "max_thrust": self._max_thrust_magnitude,
            "magnet_count": len(self._magnets),
        }
