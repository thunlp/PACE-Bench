import pygame

from pace_bench.core.renderer import Renderer
from Box2D.b2 import dynamicBody, staticBody

# ── Academic palette ──────────────────────────────────────────────
COLOR_BG          = (254, 252, 248)  # near-white background
COLOR_ENV          = ( 34,  80, 129)  # muted teal-gray — environment
COLOR_AGENT          = (234,  98,  85)  # dark slate blue — agent structures
COLOR_OUTLINE          = (109, 188, 208)  # dark blue-gray — unified outlines
COLOR_TARGET          = (237, 141,  73)  # muted red — goal marker
COLOR_BOUNDARY          = (207, 207, 207)  # medium gray — build-zone boundary
COLOR_ANNOTATION          = ( 34,  80, 129)  # darker gray — text / labels (more readable)
COLOR_FORBIDDEN          = (234,  98,  85)  # forbidden zone = target red
COLOR_ANCHOR          = (237, 141,  73)  # muted green for anchor zone


class E06Renderer(Renderer):
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

        self.simulator.ppm = 25.0
        center_x_world = 10.0
        center_y_world = 5.0
        cam_x = center_x_world * self.simulator.ppm - self.simulator.screen_width / 2
        cam_y = self.simulator.screen_height / 2 - center_y_world * self.simulator.ppm
        self.set_camera_offset(cam_x, cam_y)

        # ── Background ─────────────────────────────────────────────
        self.clear(COLOR_BG)

        # ── Draw static bodies (environment) ──────────────────────
        for body in sandbox.world.bodies:
            if body.type == staticBody:
                self.draw_body(body,
                               dynamic_color=COLOR_ENV,
                               static_color=COLOR_ENV,
                               outline_color=COLOR_OUTLINE,
                               outline_width=1)

        # ── Draw dynamic bodies ────────────────────────────────────
        for body in sandbox.world.bodies:
            if body.type == dynamicBody:
                if hasattr(sandbox, "bodies") and body in sandbox.bodies:
                    self.draw_body(body,
                                   dynamic_color=COLOR_AGENT,
                                   static_color=COLOR_AGENT,
                                   outline_color=COLOR_OUTLINE,
                                   outline_width=1)
                else:
                    self.draw_body(body,
                                   dynamic_color=COLOR_ENV,
                                   static_color=COLOR_ENV,
                                   outline_color=COLOR_OUTLINE,
                                   outline_width=1)

        # ── Terrain zone boundaries ────────────────────────────────
        bounds = sandbox.get_terrain_bounds()
        bz = bounds.get("build_zone", {})
        if bz:
            x_min = bz["x"][0]
            x_max = bz["x"][1]
            y_min = bz["y"][0]
            y_max = bz["y"][1]
            self.draw_line(x_min, y_min, x_max, y_min, COLOR_BOUNDARY, 1)
            self.draw_line(x_max, y_min, x_max, y_max, COLOR_BOUNDARY, 1)
            self.draw_line(x_max, y_max, x_min, y_max, COLOR_BOUNDARY, 1)
            self.draw_line(x_min, y_max, x_min, y_min, COLOR_BOUNDARY, 1)

        # ── Forbidden zone ─────────────────────────────────────────
        fz = bounds.get("forbidden_zone")
        if bz and fz:
            fz_lo = fz[0]
            fz_hi = fz[1]
            y_min = bz["y"][0]
            y_max = bz["y"][1]
            self.draw_line(fz_lo, y_min, fz_lo, y_max, COLOR_FORBIDDEN, 1)
            self.draw_line(fz_hi, y_min, fz_hi, y_max, COLOR_FORBIDDEN, 1)
            self.draw_line(fz_lo, y_max, fz_hi, y_max, COLOR_FORBIDDEN, 1)

        # ── Allowed anchor zone line ───────────────────────────────
        az = bounds.get("allowed_anchor_zone")
        if az:
            az_lo = az[0]
            az_hi = az[1]
            ground_y = bounds.get("ground_y", 1.0)
            self.draw_line(az_lo, ground_y, az_hi, ground_y, COLOR_ANCHOR, 3)

        # ── Annotations ────────────────────────────────────────────
        if self.simulator.can_display:
            self._init_fonts()
            _sh = self.simulator.screen_height

            # Top-left: task label
            if self._font_label:
                label = self._font_label.render("E-06 | Cantilever Endurance",
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
