import pygame

from pace_bench.core.renderer import Renderer
from Box2D.b2 import dynamicBody, staticBody

# ── Academic palette ──────────────────────────────────────────────
COLOR_BG          = (254, 252, 248)  # near-white background
COLOR_ENV          = ( 34,  80, 129)  # muted teal-gray — environment
COLOR_AGENT          = (234,  98,  85)  # dark slate blue — agent structures
COLOR_OUTLINE          = (109, 188, 208)  # dark blue-gray — unified outlines
COLOR_TARGET          = (237, 141,  73)  # muted red — goal marker
COLOR_BOUNDARY          = (207, 207, 207)  # medium gray — build-zone / zone boundaries
COLOR_ANNOTATION          = ( 34,  80, 129)  # darker gray — text / labels (more readable)


class D03Renderer(Renderer):
    def __init__(self, simulator):
        super().__init__(simulator)
        self._font_body = None
        self._font_label = None

    def _init_fonts(self):
        if self._font_label is not None:
            return
        try:
            self._font_body = pygame.font.SysFont("DejaVu Sans", 28)
            self._font_label = pygame.font.SysFont("DejaVu Sans", 40)
        except Exception:
            self._font_body = pygame.font.Font(None, 28)
            self._font_label = pygame.font.Font(None, 40)

    def render(self, sandbox, agent_body, target_x, camera_offset_x):
        # ── Square aspect ratio (600×600) ──────────────────────────
        if self.simulator.screen_width != 600 or self.simulator.screen_height != 600:
            self.simulator.screen_width = 600
            self.simulator.screen_height = 600
            if self.simulator.can_display:
                self.simulator.screen = pygame.Surface((600, 600))

        self.simulator.ppm = 40.0
        center_x_world = 8.0
        center_y_world = 4.5
        cam_x = center_x_world * self.simulator.ppm - self.simulator.screen_width / 2
        cam_y = self.simulator.screen_height / 2 - center_y_world * self.simulator.ppm
        self.set_camera_offset(cam_x, cam_y)

        # ── Background ─────────────────────────────────────────────
        self.clear(COLOR_BG)

        # ── Resolve environment bodies ─────────────────────────────
        rod_keys = ("gate_rod", "gate_rod_2", "gate_rod_3", "gate_rod_4")
        rod_bodies = [
            sandbox._terrain_bodies.get(k)
            for k in rod_keys
        ] if hasattr(sandbox, '_terrain_bodies') else []
        cabin = sandbox._terrain_bodies.get("vehicle_cabin") \
                if hasattr(sandbox, '_terrain_bodies') else None

        # ── Draw bodies ────────────────────────────────────────────
        for body in sandbox.world.bodies:
            is_environment = False
            if hasattr(sandbox, '_terrain_bodies') and body in sandbox._terrain_bodies.values():
                is_environment = True
            elif body in rod_bodies or body == cabin:
                is_environment = True

            if is_environment:
                self.draw_body(body,
                               dynamic_color=COLOR_ENV,
                               static_color=COLOR_ENV,
                               outline_color=COLOR_OUTLINE,
                               outline_width=1)
            else:
                self.draw_body(body,
                               dynamic_color=COLOR_AGENT,
                               static_color=COLOR_AGENT,
                               outline_color=COLOR_OUTLINE,
                               outline_width=1)

        # ── Target line ────────────────────────────────────────────
        tx = getattr(sandbox, "_target_x_min", 11.75)
        self.draw_line(tx, 0.4, tx, 4.0, COLOR_TARGET, 3)
        # reference line 2 m past target
        self.draw_line(tx + 2, 0.4, tx + 2, 4.0, COLOR_BOUNDARY, 1)

        # ── Zone boundary markers ──────────────────────────────────
        # mud zone
        if hasattr(sandbox, "_mud_zone_x_min"):
            m1, m2 = sandbox._mud_zone_x_min, sandbox._mud_zone_x_max
            self.draw_line(m1, 0.4, m1, 4.0, COLOR_BOUNDARY, 1)
            self.draw_line(m2, 0.4, m2, 4.0, COLOR_BOUNDARY, 1)

        # brake zone
        if hasattr(sandbox, "_brake_zone_x_min"):
            b1, b2 = sandbox._brake_zone_x_min, sandbox._brake_zone_x_max
            self.draw_line(b1, 0.4, b1, 4.0, COLOR_BOUNDARY, 1)
            self.draw_line(b2, 0.4, b2, 4.0, COLOR_BOUNDARY, 1)

        # impulse zone
        if hasattr(sandbox, "_impulse_zone_x_min"):
            i1, i2 = sandbox._impulse_zone_x_min, sandbox._impulse_zone_x_max
            self.draw_line(i1, 0.4, i1, 4.0, COLOR_BOUNDARY, 1)
            self.draw_line(i2, 0.4, i2, 4.0, COLOR_BOUNDARY, 1)

        # impulse2 zone (width=2 as in original)
        if hasattr(sandbox, "_impulse2_zone_x_min"):
            j1 = getattr(sandbox, "_impulse2_zone_x_min", 10.5)
            j2 = getattr(sandbox, "_impulse2_zone_x_max", 11.0)
            self.draw_line(j1, 0.4, j1, 4.0, COLOR_BOUNDARY, 2)
            self.draw_line(j2, 0.4, j2, 4.0, COLOR_BOUNDARY, 2)

        # speed trap (width=2 as in original)
        if hasattr(sandbox, "_speed_trap_x"):
            sx = getattr(sandbox, "_speed_trap_x", 9.0)
            self.draw_line(sx, 0.4, sx, 4.0, COLOR_BOUNDARY, 2)

        # decel zone (width=2 as in original)
        if hasattr(sandbox, "_decel_zone_x_min"):
            d1 = getattr(sandbox, "_decel_zone_x_min", 9.5)
            d2 = getattr(sandbox, "_decel_zone_x_max", 11.0)
            self.draw_line(d1, 0.4, d1, 4.0, COLOR_BOUNDARY, 2)
            self.draw_line(d2, 0.4, d2, 4.0, COLOR_BOUNDARY, 2)

        # ── Build-zone boundary ────────────────────────────────────
        if hasattr(sandbox, 'BUILD_ZONE_X_MIN'):
            x_min = sandbox.BUILD_ZONE_X_MIN
            x_max = sandbox.BUILD_ZONE_X_MAX
            y_min = sandbox.BUILD_ZONE_Y_MIN
            y_max = sandbox.BUILD_ZONE_Y_MAX
            self.draw_line(x_min, y_min, x_max, y_min, COLOR_BOUNDARY, 1)
            self.draw_line(x_max, y_min, x_max, y_max, COLOR_BOUNDARY, 1)
            self.draw_line(x_max, y_max, x_min, y_max, COLOR_BOUNDARY, 1)
            self.draw_line(x_min, y_max, x_min, y_min, COLOR_BOUNDARY, 1)

        # ── Annotations ────────────────────────────────────────────
        if self.simulator.can_display:
            self._init_fonts()
            _sh = self.simulator.screen_height

            # Top-left: task label
            if self._font_label:
                label = self._font_label.render("D-03 | Phase-Locked Gate",
                                                True, COLOR_ANNOTATION)
                self.simulator.screen.blit(label, (18, 14))

            # Bottom-left: scale bar (1 m)
            if self.simulator.ppm > 0:
                scale_px = int(1.0 * self.simulator.ppm)
                bar_x, bar_y = 20, _sh - 28
                pygame.draw.line(self.simulator.screen, COLOR_ANNOTATION,
                                 (bar_x, bar_y), (bar_x + scale_px, bar_y), 4)
                pygame.draw.line(self.simulator.screen, COLOR_ANNOTATION,
                                 (bar_x, bar_y - 5), (bar_x, bar_y + 5), 2)
                pygame.draw.line(self.simulator.screen, COLOR_ANNOTATION,
                                 (bar_x + scale_px, bar_y - 5),
                                 (bar_x + scale_px, bar_y + 5), 2)
                if self._font_body:
                    label_m = self._font_body.render("1 m", True, COLOR_ANNOTATION)
                    self.simulator.screen.blit(label_m, (bar_x + scale_px + 8, bar_y - 14))
