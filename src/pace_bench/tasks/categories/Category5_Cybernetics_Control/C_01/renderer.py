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
COLOR_ANNOTATION          = ( 34,  80, 129)  # darker gray — text / labels


class C01Renderer(Renderer):
    def __init__(self, simulator):
        super().__init__(simulator)
        self._font_body = None
        self._font_label = None
        self.simulator.ppm = 30.0
        self.simulator.screen_width = 600
        self.simulator.screen_height = 600
        if self.simulator.can_display:
            self.simulator.screen = pygame.Surface((600, 600))

    def _init_fonts(self):
        if self._font_label is not None:
            return
        try:
            self._font_body = pygame.font.SysFont("DejaVu Sans", 28)
            self._font_label = pygame.font.SysFont("DejaVu Sans", 40)
        except Exception:
            self._font_body = pygame.font.Font(None, 28)
            self._font_label = pygame.font.Font(None, 40)

    def render(self, sandbox, agent_body, _target_x, camera_offset_x):
        # ── Square aspect ratio (600×600) ──────────────────────────
        if self.simulator.screen_width != 600 or self.simulator.screen_height != 600:
            self.simulator.screen_width = 600
            self.simulator.screen_height = 600
            if self.simulator.can_display:
                self.simulator.screen = pygame.Surface((600, 600))

        # ── Terrain bounds (track layout) ──────────────────────────
        if hasattr(sandbox, "get_terrain_bounds") and callable(getattr(sandbox, "get_terrain_bounds")):
            bounds = sandbox.get_terrain_bounds()
        else:
            bounds = {}
        track_center_x = float(
            bounds.get("track_center_x", getattr(sandbox, "TRACK_CENTER_X", 10.0))
        )
        track_y = float(
            bounds.get(
                "cart_rail_center_y",
                getattr(sandbox, "cart_rail_center_y", getattr(sandbox, "CART_RAIL_CENTER_Y", 2.0)),
            )
        )

        # ── Dynamic camera following cart ──────────────────────────
        ppm = self.simulator.ppm
        w = float(self.simulator.screen_width)
        h = float(self.simulator.screen_height)
        pan_x = float(camera_offset_x or 0.0)
        fixed_offset_x = track_center_x * ppm - w / 2.0 + pan_x
        fixed_offset_y = h / 2.0 - (track_y + 1.0) * ppm
        self.set_camera_offset(fixed_offset_x, fixed_offset_y)

        # ── Background ─────────────────────────────────────────────
        self.clear(COLOR_BG)

        # ── Track line ─────────────────────────────────────────────
        safe_half_range = float(
            bounds.get("safe_half_range", getattr(sandbox, "SAFE_HALF_RANGE", 8.5))
        )
        track_x_start = track_center_x - safe_half_range
        track_x_end = track_center_x + safe_half_range
        self.draw_line(track_x_start, track_y, track_x_end, track_y, COLOR_BOUNDARY, width=4)
        self.draw_line(track_x_start, track_y - 0.2, track_x_start, track_y + 0.2, COLOR_BOUNDARY, width=4)
        self.draw_line(track_x_end, track_y - 0.2, track_x_end, track_y + 0.2, COLOR_BOUNDARY, width=4)

        # ── Draw bodies ────────────────────────────────────────────
        cart = sandbox.get_cart_body() if callable(getattr(sandbox, "get_cart_body", None)) else None
        pole = sandbox.get_pole_body() if callable(getattr(sandbox, "get_pole_body", None)) else None
        for body in sandbox.world.bodies:
            if body == cart or body == pole:
                # Cart and pole: agent color
                self.draw_body(body,
                               dynamic_color=COLOR_AGENT,
                               static_color=COLOR_AGENT,
                               outline_color=COLOR_OUTLINE,
                               outline_width=1)
            elif body.type == staticBody:
                # Static terrain: environment color
                self.draw_body(body,
                               dynamic_color=COLOR_ENV,
                               static_color=COLOR_ENV,
                               outline_color=COLOR_OUTLINE,
                               outline_width=1)
            elif body.type == dynamicBody:
                # Other dynamic bodies: agent color
                self.draw_body(body,
                               dynamic_color=COLOR_AGENT,
                               static_color=COLOR_AGENT,
                               outline_color=COLOR_OUTLINE,
                               outline_width=1)

        # ── Annotations ────────────────────────────────────────────
        if self.simulator.can_display:
            self._init_fonts()
            _sh = self.simulator.screen_height

            # Top-left: task label
            if self._font_label:
                label = self._font_label.render("C-01 | Cart-Pole",
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
