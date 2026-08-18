import pygame

from pace_bench.core.renderer import Renderer
from Box2D.b2 import dynamicBody, staticBody

# -- Academic palette -------------------------------------------------
COLOR_BG          = (254, 252, 248)  # near-white background
COLOR_ENV          = ( 34,  80, 129)  # muted teal-gray -- environment
COLOR_AGENT          = (234,  98,  85)  # dark slate blue -- agent structures
COLOR_OUTLINE          = (109, 188, 208)  # dark blue-gray -- unified outlines
COLOR_TARGET          = (237, 141,  73)  # muted red -- goal marker
COLOR_BOUNDARY          = (207, 207, 207)  # medium gray -- build-zone boundary
COLOR_ANNOTATION          = ( 34,  80, 129)  # darker gray -- text / labels (more readable)


class E05Renderer(Renderer):
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
        # -- Square aspect ratio (600x600) ----------------------------
        if self.simulator.screen_width != 600 or self.simulator.screen_height != 600:
            self.simulator.screen_width = 600
            self.simulator.screen_height = 600
            if self.simulator.can_display:
                self.simulator.screen = pygame.Surface((600, 600))

        self.simulator.ppm = 20.0
        center_x_world = 20.0
        center_y_world = 10.0
        cam_x = center_x_world * self.simulator.ppm - self.simulator.screen_width / 2
        cam_y = self.simulator.screen_height / 2 - center_y_world * self.simulator.ppm
        self.set_camera_offset(cam_x, cam_y)

        # -- Background -------------------------------------------------
        self.clear(COLOR_BG)

        # -- Draw bodies ------------------------------------------------
        for body in sandbox.world.bodies:
            if body.type == staticBody:
                self.draw_body(body,
                               static_color=COLOR_ENV,
                               outline_color=COLOR_OUTLINE,
                               outline_width=1)
            else:
                # dynamicBody – decide agent vs environment coloring
                is_agent = False
                if hasattr(sandbox, '_terrain_bodies'):
                    if body == sandbox._terrain_bodies.get("body"):
                        is_agent = True
                if hasattr(sandbox, 'bodies') and body in sandbox.bodies:
                    is_agent = True
                if is_agent:
                    self.draw_body(body,
                                   dynamic_color=COLOR_AGENT,
                                   outline_color=COLOR_OUTLINE,
                                   outline_width=1)
                else:
                    self.draw_body(body,
                                   dynamic_color=COLOR_ENV,
                                   outline_color=COLOR_OUTLINE,
                                   outline_width=1)

        # -- Target zone ------------------------------------------------
        bounds = sandbox.get_terrain_bounds()
        tz = bounds.get("target_zone", {})
        tx_min = tz.get("x_min", 28.0)
        tx_max = tz.get("x_max", 32.0)
        ty_min = tz.get("y_min", 6.0)
        ty_max = tz.get("y_max", 9.0)
        self.draw_line(tx_min, ty_min, tx_max, ty_min, COLOR_TARGET, 2)
        self.draw_line(tx_max, ty_min, tx_max, ty_max, COLOR_TARGET, 2)
        self.draw_line(tx_max, ty_max, tx_min, ty_max, COLOR_TARGET, 2)
        self.draw_line(tx_min, ty_max, tx_min, ty_min, COLOR_TARGET, 2)

        # -- Annotations ------------------------------------------------
        if self.simulator.can_display:
            self._init_fonts()
            _sh = self.simulator.screen_height

            # Top-left: task label
            if self._font_label:
                label = self._font_label.render("E-05 | Magnetic Navigation",
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
