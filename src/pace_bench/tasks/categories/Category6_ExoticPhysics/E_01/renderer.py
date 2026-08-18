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


class E01Renderer(Renderer):
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
        center_x_world = 20.0
        center_y_world = 10.0
        cam_x = center_x_world * self.simulator.ppm - self.simulator.screen_width / 2
        cam_y = self.simulator.screen_height / 2 - center_y_world * self.simulator.ppm
        self.set_camera_offset(cam_x, cam_y)

        # ── Background ─────────────────────────────────────────────
        self.clear(COLOR_BG)

        # ── Draw bodies ────────────────────────────────────────────
        demonstrator_bodies = set()
        if hasattr(sandbox, '_terrain_bodies'):
            for key, value in sandbox._terrain_bodies.items():
                if key.startswith('demonstrator_'):
                    demonstrator_bodies.add(value)

        for body in sandbox.world.bodies:
            if body.type == staticBody:
                self.draw_body(body,
                               dynamic_color=COLOR_ENV,
                               static_color=COLOR_ENV,
                               outline_color=COLOR_OUTLINE,
                               outline_width=1)
            else:
                # dynamic body
                if body in demonstrator_bodies:
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

        # ── Arena boundary ─────────────────────────────────────────
        ax_min = getattr(sandbox, 'ARENA_X_MIN', 0)
        ax_max = getattr(sandbox, 'ARENA_X_MAX', 40)
        ay_min = getattr(sandbox, 'ARENA_Y_MIN', 0)
        ay_max = getattr(sandbox, 'ARENA_Y_MAX', 20)
        self.draw_line(ax_min, ay_min, ax_max, ay_min, COLOR_BOUNDARY, 1)
        self.draw_line(ax_max, ay_min, ax_max, ay_max, COLOR_BOUNDARY, 1)
        self.draw_line(ax_max, ay_max, ax_min, ay_max, COLOR_BOUNDARY, 1)
        self.draw_line(ax_min, ay_max, ax_min, ay_min, COLOR_BOUNDARY, 1)

        # ── Annotations ────────────────────────────────────────────
        if self.simulator.can_display:
            self._init_fonts()
            _sh = self.simulator.screen_height

            # Bottom-left: task label (moved from top-left — arena objects block top)
            if self._font_label:
                label = self._font_label.render("E-01 | Gravity Containment",
                                                True, COLOR_ANNOTATION)
                self.simulator.screen.blit(label, (18, _sh - 60))

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
