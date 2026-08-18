import pygame

from pace_bench.core.renderer import Renderer
from Box2D.b2 import dynamicBody, staticBody

# ── Academic palette ──────────────────────────────────────────────
COLOR_BG          = (254, 252, 248)
COLOR_ENV          = ( 34,  80, 129)
COLOR_AGENT          = (234,  98,  85)
COLOR_OUTLINE          = (109, 188, 208)
COLOR_TARGET          = (237, 141,  73)
COLOR_BOUNDARY          = (207, 207, 207)
COLOR_ANNOTATION          = ( 34,  80, 129)
COLOR_BARRIER          = (234,  98,  85)  # barrier uses target red


class C05Renderer(Renderer):
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

        # 50.0 px/m → 12 m visible across 600 px
        self.simulator.ppm = 50.0
        center_x_world = 6.0
        center_y_world = 6.0
        cam_x = center_x_world * self.simulator.ppm - self.simulator.screen_width / 2
        cam_y = self.simulator.screen_height / 2 - center_y_world * self.simulator.ppm
        self.set_camera_offset(cam_x, cam_y)

        # ── Background ─────────────────────────────────────────────
        self.clear(COLOR_BG)

        # ── Retrieve named bodies ──────────────────────────────────
        barrier_body = None
        agent = None
        if hasattr(sandbox, '_terrain_bodies'):
            barrier_body = sandbox._terrain_bodies.get('barrier')
            agent = sandbox._terrain_bodies.get('agent')

        # ── Draw bodies ────────────────────────────────────────────
        for body in sandbox.world.bodies:
            if body.type == staticBody:
                is_barrier = (body is barrier_body)
                color = COLOR_BARRIER if is_barrier else COLOR_ENV
                self.draw_body(body,
                               dynamic_color=color,
                               static_color=color,
                               outline_color=COLOR_OUTLINE,
                               outline_width=1)
            elif body == agent:
                self.draw_body(body,
                               dynamic_color=COLOR_AGENT,
                               static_color=COLOR_AGENT,
                               outline_color=COLOR_OUTLINE,
                               outline_width=1)
            elif body.type == dynamicBody:
                self.draw_body(body,
                               dynamic_color=COLOR_AGENT,
                               static_color=COLOR_AGENT,
                               outline_color=COLOR_OUTLINE,
                               outline_width=1)

        # ── Zone rectangles ────────────────────────────────────────
        if hasattr(sandbox, "_zones"):
            for _name, (cx, cy, hw, hh) in sandbox._zones.items():
                x1, y1 = cx - hw, cy - hh
                x2, y2 = cx + hw, cy + hh
                self.draw_line(x1, y1, x2, y1, COLOR_BOUNDARY, 1)
                self.draw_line(x2, y1, x2, y2, COLOR_BOUNDARY, 1)
                self.draw_line(x2, y2, x1, y2, COLOR_BOUNDARY, 1)
                self.draw_line(x1, y2, x1, y1, COLOR_BOUNDARY, 1)

        # ── Annotations ────────────────────────────────────────────
        if self.simulator.can_display:
            self._init_fonts()
            _sh = self.simulator.screen_height

            # Top-left: task label
            if self._font_label:
                label = self._font_label.render("C-05 | Sequential Gates",
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
