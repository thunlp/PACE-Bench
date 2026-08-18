import math

try:
    from .environment import (
        BARRIER_X_LEFT,
        BARRIER_X_RIGHT,
        BARRIER_Y_BOTTOM,
        BARRIER_Y_TOP,
        DEFAULT_TIME_STEP,
        GROUND_LENGTH,
        GROUND_SLAB_HEIGHT,
        GROUND_Y_TOP,
        LAND_TOLERANCE,
        LANDER_HALF_HEIGHT,
        LANDER_HALF_WIDTH,
        MAX_EPISODE_STEPS,
        MAX_LANDING_ANGLE,
        MAX_SAFE_VERTICAL_SPEED,
        MAX_THRUST,
        MAX_TORQUE,
        MIN_FUEL_REMAINING_AT_LANDING,
        PLATFORM_AMPLITUDE,
        PLATFORM_CENTER_BASE,
        PLATFORM_HALF_WIDTH,
        PLATFORM_PERIOD,
        SPAWN_X,
        SPAWN_Y,
        THRUST_DELAY_STEPS,
        TOTAL_FUEL_IMPULSE,
    )

except ImportError:
    from pace_bench.tasks.categories.Category5_Cybernetics_Control.C_02.environment import (
        BARRIER_X_LEFT,
        BARRIER_X_RIGHT,
        BARRIER_Y_BOTTOM,
        BARRIER_Y_TOP,
        DEFAULT_TIME_STEP,
        GROUND_LENGTH,
        GROUND_SLAB_HEIGHT,
        GROUND_Y_TOP,
        LAND_TOLERANCE,
        LANDER_HALF_HEIGHT,
        LANDER_HALF_WIDTH,
        MAX_EPISODE_STEPS,
        MAX_LANDING_ANGLE,
        MAX_SAFE_VERTICAL_SPEED,
        MAX_THRUST,
        MAX_TORQUE,
        MIN_FUEL_REMAINING_AT_LANDING,
        PLATFORM_AMPLITUDE,
        PLATFORM_CENTER_BASE,
        PLATFORM_HALF_WIDTH,
        PLATFORM_PERIOD,
        SPAWN_X,
        SPAWN_Y,
        THRUST_DELAY_STEPS,
        TOTAL_FUEL_IMPULSE,
    )

from pace_bench.core.primitives import compute_constraint_penalty

class Evaluator:
    def __init__(self, terrain_bounds, environment=None):
        self.terrain_bounds = terrain_bounds
        self.environment = environment
        self.max_safe_vertical_speed = float(
            terrain_bounds.get("max_safe_vertical_speed", MAX_SAFE_VERTICAL_SPEED)
        )
        self._landed = False
        self._landing_vy = None
        self._landing_angle = None
        self._landing_x = None
        self._landing_step = None
        self._landing_x_lo = None
        self._landing_x_hi = None
        self._max_landing_angle = float(
            terrain_bounds.get("max_landing_angle", MAX_LANDING_ANGLE)
        )
        self._min_fuel_remaining = float(
            terrain_bounds.get("min_fuel_remaining_at_landing", MIN_FUEL_REMAINING_AT_LANDING)
        )
        self._total_fuel_impulse = float(
            terrain_bounds.get("total_fuel_impulse", TOTAL_FUEL_IMPULSE)
        )
        self._max_thrust = float(terrain_bounds.get("max_thrust", MAX_THRUST))
        self._max_torque = float(terrain_bounds.get("max_torque", MAX_TORQUE))
        self._barrier_x_left = float(terrain_bounds.get("barrier_x_left", BARRIER_X_LEFT))
        self._barrier_x_right = float(terrain_bounds.get("barrier_x_right", BARRIER_X_RIGHT))
        self._barrier_y_top = float(terrain_bounds.get("barrier_y_top", BARRIER_Y_TOP))
        self._barrier_y_bottom = float(terrain_bounds.get("barrier_y_bottom", BARRIER_Y_BOTTOM))
        self._land_tolerance = float(terrain_bounds.get("land_tolerance", LAND_TOLERANCE))
        _ms = terrain_bounds.get("max_episode_steps")
        self._episode_step_limit = int(_ms) if _ms is not None else int(MAX_EPISODE_STEPS)
    def evaluate(self, agent_body, step_count, max_steps):
        if not self.environment:
            hz = (
                min(max_steps, self._episode_step_limit)
                if max_steps > 0
                else self._episode_step_limit
            )
            td = int(self.terrain_bounds.get("thrust_delay_steps", THRUST_DELAY_STEPS))
            return True, 0.0, {
                "error": "Environment not available",
                "failed": True,
                "success": False,
                "landed": False,
                "max_episode_steps": self._episode_step_limit,
                "episode_horizon": hz,
                "thrust_delay_steps": td,
                "corridor_transit": {},
                "constraint_profile": [],
            }
        if max_steps > 0:
            horizon = min(max_steps, self._episode_step_limit)
        else:
            horizon = self._episode_step_limit
        thrust_delay_steps = int(self.terrain_bounds.get("thrust_delay_steps", THRUST_DELAY_STEPS))
        if hasattr(self.environment, "get_thrust_delay_steps"):
            thrust_delay_steps = int(self.environment.get_thrust_delay_steps())
        if getattr(self.environment, "get_barrier_hit", lambda: False)():
            x, y = self.environment.get_lander_position()
            ground_y_top = self.environment.get_ground_y_top()
            bottom_y = self.environment.get_lander_bottom_y()
            kind = (
                self.environment.get_barrier_failure_kind()
                if hasattr(self.environment, "get_barrier_failure_kind")
                else getattr(self.environment, "_barrier_failure_kind", None)
            )
            if kind == "ceiling":
                reason = "Forbidden corridor upper-bound violation"
            elif kind == "obstacle":
                reason = "Forbidden corridor lower-bound violation"
            else:
                bt = float(self.terrain_bounds.get("barrier_y_top", BARRIER_Y_TOP))
                bb = float(self.terrain_bounds.get("barrier_y_bottom", BARRIER_Y_BOTTOM))
                mid = 0.5 * (bt + bb)
                if y < mid:
                    reason = "Forbidden corridor lower-bound violation"
                else:
                    reason = "Forbidden corridor upper-bound violation"
            if hasattr(self.environment, "get_zone_x_bounds_at_step"):
                zone_x_min, zone_x_max = self.environment.get_zone_x_bounds_at_step(step_count)
            else:
                _pc = float(getattr(self.environment, "_platform_center_base", PLATFORM_CENTER_BASE))
                _ph = float(getattr(self.environment, "_platform_half_width", PLATFORM_HALF_WIDTH))
                zone_x_min, zone_x_max = _pc - _ph, _pc + _ph
            rfuel = (
                self.environment.get_remaining_fuel()
                if hasattr(self.environment, "get_remaining_fuel")
                else None
            )
            return True, 0.0, {
                "failed": True,
                "failure_reason": reason,
                "success": False,
                "lander_x": x,
                "lander_y": y,
                "lander_vx": self.environment.get_lander_velocity()[0],
                "lander_vy": self.environment.get_lander_velocity()[1],
                "lander_angle": self.environment.get_lander_angle(),
                "lander_angular_velocity": self.environment.get_lander_angular_velocity()
                if hasattr(self.environment, "get_lander_angular_velocity")
                else 0.0,
                "step_count": step_count,
                "landed": False,
                "landing_vy": None,
                "landing_x": None,
                "landing_x_lo": None,
                "landing_x_hi": None,
                "landing_angle": None,
                "landing_step": None,
                "height_above_ground": bottom_y - ground_y_top,
                "zone_x_min": zone_x_min,
                "zone_x_max": zone_x_max,
                "max_safe_vertical_speed": self.max_safe_vertical_speed,
                "max_landing_angle": self._max_landing_angle,
                "min_fuel_remaining_at_landing": self._min_fuel_remaining,
                "remaining_fuel": rfuel,
                "total_fuel_impulse": self._total_fuel_impulse,
                "max_thrust": self._max_thrust,
                "max_torque": self._max_torque,
                "ground_y_top": ground_y_top,
                "barrier_x_left": self._barrier_x_left,
                "barrier_x_right": self._barrier_x_right,
                "barrier_y_top": self._barrier_y_top,
                "barrier_y_bottom": float(
                    self.terrain_bounds.get("barrier_y_bottom", BARRIER_Y_BOTTOM)
                ),
                "max_episode_steps": self._episode_step_limit,
                "episode_horizon": horizon,
                "thrust_delay_steps": thrust_delay_steps,
                "corridor_transit": self.environment.get_corridor_transit_data()
                if hasattr(self.environment, "get_corridor_transit_data")
                else {},
                "actuation_diagnostics": self.environment.get_actuation_diagnostics()
                if hasattr(self.environment, "get_actuation_diagnostics")
                else {},
                "motion_diagnostics": self.environment.get_motion_diagnostics()
                if hasattr(self.environment, "get_motion_diagnostics")
                else {},
                "constraint_profile": self._compute_full_constraint_profile(
                    landed=False, landing_vy=None, landing_angle=None,
                    landing_x_lo=None, landing_x_hi=None,
                    landing_x=None, remaining_fuel=rfuel,
                    step_count=step_count, horizon=horizon,
                    max_safe_vy=self.max_safe_vertical_speed,
                    max_landing_angle=self._max_landing_angle,
                    min_fuel=self._min_fuel_remaining,
                    barrier_hit=True, barrier_kind=kind,
                    zone_x_min=zone_x_min, zone_x_max=zone_x_max,
                ),
            }
        ground_y_top = self.environment.get_ground_y_top()
        bottom_y = self.environment.get_lander_bottom_y()
        x, y = self.environment.get_lander_position()
        vx, vy = self.environment.get_lander_velocity()
        angle = self.environment.get_lander_angle()
        touchdown_snapshot = (
            self.environment.get_touchdown_snapshot()
            if hasattr(self.environment, "get_touchdown_snapshot")
            else None
        )
        landed_this_step = (
            touchdown_snapshot is not None
            or bottom_y <= ground_y_top + self._land_tolerance
        )
        if landed_this_step and not self._landed:
            self._landed = True
            if touchdown_snapshot is not None:
                self._landing_vy = float(touchdown_snapshot["vy"])
                self._landing_angle = float(touchdown_snapshot["angle"])
                self._landing_x = float(touchdown_snapshot["x"])
                self._landing_step = int(touchdown_snapshot["step"])
                self._landing_x_lo = float(touchdown_snapshot["x_lo"])
                self._landing_x_hi = float(touchdown_snapshot["x_hi"])
            else:
                self._landing_vy = vy
                self._landing_angle = angle
                self._landing_x = x
                self._landing_step = step_count
            if (
                touchdown_snapshot is None
                and hasattr(self.environment, "get_lander_bottom_contact_x_span")
            ):
                self._landing_x_lo, self._landing_x_hi = (
                    self.environment.get_lander_bottom_contact_x_span()
                )
            elif touchdown_snapshot is None:
                self._landing_x_lo = self._landing_x_hi = x
        failed = False
        failure_reason = None
        remaining_fuel = (
            self.environment.get_remaining_fuel()
            if hasattr(self.environment, "get_remaining_fuel")
            else None
        )
        if remaining_fuel is not None and remaining_fuel <= 0 and not self._landed:
            failed = True
            failure_reason = "Fuel exhausted before landing"
        elif self._landed and self._landing_vy is not None:
            zone_x_min, zone_x_max = self.environment.get_zone_x_bounds_at_step(
                self._landing_step
            )
            x_lo = self._landing_x_lo if self._landing_x_lo is not None else self._landing_x
            x_hi = self._landing_x_hi if self._landing_x_hi is not None else self._landing_x
            landing_reasons = []
            if abs(self._landing_vy) > self.max_safe_vertical_speed:
                landing_reasons.append(
                    f"Touchdown vertical speed |vy|={abs(self._landing_vy):.2f} m/s exceeds limit "
                    f"{self.max_safe_vertical_speed:.2f} m/s"
                )
            if x_lo is not None and x_hi is not None and (
                x_lo < zone_x_min or x_hi > zone_x_max
            ):
                landing_reasons.append(
                    f"Hull ground-contact edge x=[{x_lo:.2f}, {x_hi:.2f}] not fully inside landing zone "
                    f"[{zone_x_min:.2f}, {zone_x_max:.2f}] at touchdown"
                )
            if self._landing_angle is not None and abs(self._landing_angle) > self._max_landing_angle:
                limit_deg = math.degrees(self._max_landing_angle)
                angle_deg = math.degrees(abs(self._landing_angle))
                landing_reasons.append(
                    f"Landing angle |θ|={angle_deg:.1f}° exceeds limit {limit_deg:.1f}°"
                )
            if remaining_fuel is not None and remaining_fuel < self._min_fuel_remaining:
                landing_reasons.append(
                    f"Remaining fuel/impulse {remaining_fuel:.3f} below minimum required "
                    f"{self._min_fuel_remaining:.3f} at landing"
                )
            if landing_reasons:
                failed = True
                failure_reason = " | ".join(landing_reasons)
        elif horizon > 0 and step_count >= horizon and not self._landed:
            failed = True
            failure_reason = "Episode step limit reached without a successful landing"
        success = self._landed and not failed
        if success and not failed:
            score = 100.0
        elif failed:
            score = 0.0
        else:
            if horizon > 0:
                progress = step_count / horizon
            elif max_steps > 0:
                progress = step_count / max_steps
            else:
                progress = 0.0
            score = progress * 80.0
        height_above_ground = bottom_y - ground_y_top
        speed = math.sqrt(vx * vx + vy * vy)
        if hasattr(self.environment, "get_zone_x_bounds_at_step"):
            zone_x_min, zone_x_max = self.environment.get_zone_x_bounds_at_step(step_count)
        else:
            _pc = float(getattr(self.environment, "_platform_center_base", PLATFORM_CENTER_BASE))
            _ph = float(getattr(self.environment, "_platform_half_width", PLATFORM_HALF_WIDTH))
            zone_x_min, zone_x_max = _pc - _ph, _pc + _ph
        metrics = {
            "lander_x": x,
            "lander_y": y,
            "lander_vx": vx,
            "lander_vy": vy,
            "lander_angle": angle,
            "lander_angular_velocity": self.environment.get_lander_angular_velocity()
                if self.environment else 0.0,
            "landed": self._landed,
            "landing_vy": self._landing_vy,
            "landing_angle": self._landing_angle,
            "landing_x": self._landing_x,
            "landing_x_lo": self._landing_x_lo,
            "landing_x_hi": self._landing_x_hi,
            "step_count": step_count,
            "success": success,
            "failed": failed,
            "failure_reason": failure_reason,
            "height_above_ground": height_above_ground,
            "speed": speed,
            "ground_y_top": ground_y_top,
            "max_safe_vertical_speed": self.max_safe_vertical_speed,
            "zone_x_min": zone_x_min,
            "zone_x_max": zone_x_max,
            "max_landing_angle": self._max_landing_angle,
            "remaining_fuel": remaining_fuel if remaining_fuel is not None else None,
            "total_fuel_impulse": self._total_fuel_impulse,
            "max_thrust": self._max_thrust,
            "max_torque": self._max_torque,
            "min_fuel_remaining_at_landing": self._min_fuel_remaining,
            "landing_step": self._landing_step,
            "barrier_x_left": self._barrier_x_left,
            "barrier_x_right": self._barrier_x_right,
            "barrier_y_top": self._barrier_y_top,
            "barrier_y_bottom": self._barrier_y_bottom,
            "max_episode_steps": self._episode_step_limit,
            "episode_horizon": horizon,
            "thrust_delay_steps": thrust_delay_steps,
            "corridor_transit": self.environment.get_corridor_transit_data()
            if self.environment and hasattr(self.environment, "get_corridor_transit_data")
            else {},
            "actuation_diagnostics": self.environment.get_actuation_diagnostics()
            if self.environment and hasattr(self.environment, "get_actuation_diagnostics")
            else {},
            "motion_diagnostics": self.environment.get_motion_diagnostics()
            if self.environment and hasattr(self.environment, "get_motion_diagnostics")
            else {},
            "constraint_profile": self._compute_full_constraint_profile(
                landed=self._landed,
                landing_vy=self._landing_vy,
                landing_angle=self._landing_angle,
                landing_x_lo=self._landing_x_lo,
                landing_x_hi=self._landing_x_hi,
                landing_x=self._landing_x,
                remaining_fuel=remaining_fuel,
                step_count=step_count,
                horizon=horizon,
                max_safe_vy=self.max_safe_vertical_speed,
                max_landing_angle=self._max_landing_angle,
                min_fuel=self._min_fuel_remaining,
                barrier_hit=False,
                barrier_kind=None,
                zone_x_min=zone_x_min,
                zone_x_max=zone_x_max,
            ),
        }
        if self._landed and self._landing_step is not None and self.environment:
            landing_t = self._landing_step * float(
                self.terrain_bounds.get("time_step", DEFAULT_TIME_STEP)
            )
            metrics["platform_center_at_landing"] = (
                self.environment.get_platform_center_at_time(landing_t)
            )
            zx_lo, zx_hi = self.environment.get_zone_x_bounds_at_step(
                self._landing_step
            )
            metrics["platform_zone_at_landing"] = [zx_lo, zx_hi]
        done = failed or self._landed
        return done, score, metrics
    def compute_score_with_penalty(self, score: float, metrics: dict) -> float:
        if score > 0:
            return score
        constraint_info = self.get_constraint_info()
        penalty = compute_constraint_penalty("C_02", score, metrics, constraint_info)
        return penalty
    def get_constraint_info(self):
        return {
            'max_safe_vertical_speed': self.max_safe_vertical_speed,
            'max_landing_angle': self._max_landing_angle,
            'min_fuel_remaining': self._min_fuel_remaining,
            'barrier_x_left': self._barrier_x_left,
            'barrier_x_right': self._barrier_x_right,
            'barrier_y_top': self._barrier_y_top,
            'barrier_y_bottom': self._barrier_y_bottom,
            'land_tolerance': self._land_tolerance,
            'episode_step_limit': self._episode_step_limit,
            'thrust_delay_steps': int(self.terrain_bounds.get("thrust_delay_steps", THRUST_DELAY_STEPS)),
        }
    def _compute_full_constraint_profile(
        self,
        landed, landing_vy, landing_angle,
        landing_x_lo, landing_x_hi, landing_x,
        remaining_fuel, step_count, horizon,
        max_safe_vy, max_landing_angle, min_fuel,
        barrier_hit, barrier_kind,
        zone_x_min, zone_x_max,
    ):
        constraints = []
        if landed and landing_vy is not None:
            val = abs(float(landing_vy))
            limit = float(max_safe_vy)
            margin = limit - val
            constraints.append({
                "name": "Touchdown vertical speed",
                "key": "landing_vy",
                "status": "PASS" if margin >= 0 else "FAIL",
                "value": val,
                "limit": limit,
                "margin": margin,
                "pct_of_limit": (val / limit * 100.0) if limit > 0 else 0.0,
                "bound_type": "upper",
            })
        elif not landed:
            constraints.append({
                "name": "Touchdown vertical speed",
                "key": "landing_vy",
                "status": "PENDING",
                "value": None,
                "limit": float(max_safe_vy),
                "margin": None,
                "pct_of_limit": None,
                "bound_type": "upper",
            })
        if landed and landing_x_lo is not None and landing_x_hi is not None:
            lo = float(landing_x_lo)
            hi = float(landing_x_hi)
            zx_lo = float(zone_x_min)
            zx_hi = float(zone_x_max)
            margin_left = lo - zx_lo
            margin_right = zx_hi - hi
            worst_margin = min(margin_left, margin_right)
            constraints.append({
                "name": "Hull ground-contact edge in landing zone",
                "key": "hull_footprint",
                "status": "PASS" if (lo >= zx_lo and hi <= zx_hi) else "FAIL",
                "value": [lo, hi],
                "limit": [zx_lo, zx_hi],
                "margin": worst_margin,
                "pct_of_limit": None,
                "bound_type": "boundary",
            })
        elif not landed:
            constraints.append({
                "name": "Hull ground-contact edge in landing zone",
                "key": "hull_footprint",
                "status": "PENDING",
                "value": None,
                "limit": [float(zone_x_min), float(zone_x_max)],
                "margin": None,
                "pct_of_limit": None,
                "bound_type": "boundary",
            })
        if landed and landing_angle is not None:
            val = abs(float(landing_angle))
            limit = float(max_landing_angle)
            margin = limit - val
            constraints.append({
                "name": "Landing angle",
                "key": "landing_angle",
                "status": "PASS" if margin >= 0 else "FAIL",
                "value": val,
                "limit": limit,
                "margin": margin,
                "pct_of_limit": (val / limit * 100.0) if limit > 0 else 0.0,
                "bound_type": "upper",
            })
        elif not landed:
            constraints.append({
                "name": "Landing angle",
                "key": "landing_angle",
                "status": "PENDING",
                "value": None,
                "limit": float(max_landing_angle),
                "margin": None,
                "pct_of_limit": None,
                "bound_type": "upper",
            })
        if landed and remaining_fuel is not None:
            val = float(remaining_fuel)
            limit = float(min_fuel)
            margin = val - limit
            constraints.append({
                "name": "Fuel remaining at landing",
                "key": "remaining_fuel",
                "status": "PASS" if margin >= 0 else "FAIL",
                "value": val,
                "limit": limit,
                "margin": margin,
                "pct_of_limit": (val / limit * 100.0) if limit > 0 else 0.0,
                "bound_type": "lower",
            })
        elif not landed:
            constraints.append({
                "name": "Fuel remaining at landing",
                "key": "remaining_fuel",
                "status": "PENDING",
                "value": remaining_fuel,
                "limit": float(min_fuel),
                "margin": (float(remaining_fuel) - float(min_fuel)) if remaining_fuel is not None else None,
                "pct_of_limit": None,
                "bound_type": "lower",
            })
        constraints.append({
            "name": "No barrier violation",
            "key": "barrier_hit",
            "status": "FAIL" if barrier_hit else "PASS",
            "value": barrier_kind if barrier_hit else None,
            "limit": "No contact with forbidden zone",
            "margin": None,
            "pct_of_limit": None,
            "bound_type": "boundary",
        })
        fuel_exhausted = (remaining_fuel is not None and remaining_fuel <= 0 and not landed)
        constraints.append({
            "name": "Fuel not exhausted before landing",
            "key": "fuel_exhausted",
            "status": "FAIL" if fuel_exhausted else "PASS",
            "value": remaining_fuel,
            "limit": ">0 N·s until touchdown",
            "margin": float(remaining_fuel) if remaining_fuel is not None else None,
            "pct_of_limit": None,
            "bound_type": "lower",
        })
        if horizon > 0:
            margin = horizon - step_count
            constraints.append({
                "name": "Land within episode steps",
                "key": "episode_steps",
                "status": ("FAIL" if (not landed and step_count >= horizon) else
                           "PASS" if landed else "PENDING"),
                "value": step_count,
                "limit": int(horizon),
                "margin": margin,
                "pct_of_limit": (step_count / horizon * 100.0) if horizon > 0 else 0.0,
                "bound_type": "upper",
            })
        def _sort_key(c):
            status_order = {"FAIL": 0, "PENDING": 1, "PASS": 2}
            margin = c.get("margin")
            if margin is None:
                return (status_order.get(c["status"], 3), 0.0)
            try:
                return (status_order.get(c["status"], 3), -abs(float(margin)))
            except (TypeError, ValueError):
                return (status_order.get(c["status"], 3), 0.0)
        constraints.sort(key=_sort_key)
        return constraints
    def get_task_description(self):
        tb = self.terrain_bounds
        e = self.environment
        def _from_env(attr: str, default):
            if e is not None and hasattr(e, attr):
                return getattr(e, attr)
            return default
        spawn_x = float(tb.get("spawn_x", _from_env("_spawn_x", SPAWN_X)))
        spawn_y = float(tb.get("spawn_y", _from_env("_spawn_y", SPAWN_Y)))
        pc = float(tb.get("platform_center_base", _from_env("_platform_center_base", PLATFORM_CENTER_BASE)))
        pa = float(tb.get("platform_amplitude", _from_env("_platform_amplitude", PLATFORM_AMPLITUDE)))
        pp = float(tb.get("platform_period", _from_env("_platform_period", PLATFORM_PERIOD)))
        phw = float(tb.get("platform_half_width", _from_env("_platform_half_width", PLATFORM_HALF_WIDTH)))
        max_thrust = float(tb.get("max_thrust", _from_env("_max_thrust", MAX_THRUST)))
        max_torque = float(tb.get("max_torque", _from_env("_max_torque", MAX_TORQUE)))
        return {
            "task": "C-02: The Lander (obstacle + moving platform)",
            "description": (
                "Control a delayed-thrust lander through a bounded corridor and "
                "touch down safely inside a moving horizontal landing zone."
            ),
            "spawn_m": {"x": spawn_x, "y": spawn_y},
            "lander": {
                "half_width_m": float(tb.get("lander_half_width", LANDER_HALF_WIDTH)),
                "half_height_m": float(tb.get("lander_half_height", LANDER_HALF_HEIGHT)),
            },
            "ground": {
                "surface_y_m": float(tb.get("ground_y_top", GROUND_Y_TOP)),
                "slab_thickness_m": float(tb.get("ground_slab_height", GROUND_SLAB_HEIGHT)),
                "length_m": float(tb.get("ground_length", GROUND_LENGTH)),
            },
            "simulation": {
                "time_step_s": float(tb.get("time_step", DEFAULT_TIME_STEP)),
                "max_episode_steps": int(tb.get("max_episode_steps", MAX_EPISODE_STEPS)),
                "thrust_delay_steps": int(tb.get("thrust_delay_steps", THRUST_DELAY_STEPS)),
            },
            "no_fly_corridor": {
                "x_left_m": float(tb.get("barrier_x_left", BARRIER_X_LEFT)),
                "x_right_m": float(tb.get("barrier_x_right", BARRIER_X_RIGHT)),
                "y_obstacle_top_m": float(tb.get("barrier_y_top", BARRIER_Y_TOP)),
                "y_ceiling_m": float(tb.get("barrier_y_bottom", BARRIER_Y_BOTTOM)),
            },
            "landing_platform": {
                "center_x_m": pc,
                "half_width_m": phw,
                "amplitude_m": pa,
                "period_s": pp,
            },
            "actuation_limits": {"max_thrust_n": max_thrust, "max_torque_nm": max_torque},
            "fuel_impulse": {
                "total_budget_ns": float(tb.get("total_fuel_impulse", TOTAL_FUEL_IMPULSE)),
                "min_remaining_at_landing_ns": float(
                    tb.get("min_fuel_remaining_at_landing", MIN_FUEL_REMAINING_AT_LANDING)
                ),
            },
            "touchdown": {
                "land_tolerance_m": float(tb.get("land_tolerance", LAND_TOLERANCE)),
                "max_safe_vertical_speed_m_s": self.max_safe_vertical_speed,
                "max_landing_angle_rad": self._max_landing_angle,
            },
            "max_safe_vertical_speed": self.max_safe_vertical_speed,
            "max_landing_angle_rad": self._max_landing_angle,
            "min_fuel_remaining_at_landing": self._min_fuel_remaining,
            "success_criteria": {
                "primary": (
                    "Reach first ground contact within the live speed, angle, "
                    "platform-footprint, fuel, corridor, and episode limits."
                ),
                "failure": (
                    "Any forbidden-corridor entry, exhausted fuel before contact, "
                    "touchdown constraint violation, or episode timeout."
                ),
            },
            "evaluation": {
                "score_range": "0-100",
                "success_score": 100,
                "failure_score": 0,
            },
        }
