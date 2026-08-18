import pygame

from pace_bench.core.renderer import Renderer
from Box2D.b2 import staticBody

# ── Academic palette ──────────────────────────────────────────────
COLOR_BG          = (254, 252, 248)  # near-white background
COLOR_ENV          = ( 34,  80, 129)  # muted teal-gray — environment
COLOR_AGENT          = (234,  98,  85)  # dark slate blue — agent structures
COLOR_OUTLINE          = (109, 188, 208)  # dark blue-gray — unified outlines
COLOR_TARGET          = (237, 141,  73)  # muted red — goal marker
COLOR_BOUNDARY          = (207, 207, 207)  # medium gray — build-zone boundary
COLOR_ANNOTATION          = ( 34,  80, 129)  # darker gray — text / labels
COLOR_CHECKPOINT          = ( 34,  80, 129)  # muted blue — checkpoint zones


class E03Renderer(Renderer):
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

        self.simulator.ppm = 20.0
        center_x_world = 20.0
        center_y_world = 10.0
        cam_x = center_x_world * self.simulator.ppm - self.simulator.screen_width / 2
        cam_y = self.simulator.screen_height / 2 - center_y_world * self.simulator.ppm
        self.set_camera_offset(cam_x, cam_y)

        # ── Background ─────────────────────────────────────────────
        self.clear(COLOR_BG)

        # ── Identify sled body ─────────────────────────────────────
        sled_body = None
        if hasattr(sandbox, '_terrain_bodies'):
            sled_body = sandbox._terrain_bodies.get("sled")

        # ── Collect agent body ids for fast lookup ─────────────────
        agent_body_ids = set()
        if hasattr(sandbox, 'bodies'):
            agent_body_ids = set(id(b) for b in sandbox.bodies)

        # ── Draw bodies ────────────────────────────────────────────
        for body in sandbox.world.bodies:
            if body.type == staticBody:
                self.draw_body(body,
                               dynamic_color=COLOR_ENV,
                               static_color=COLOR_ENV,
                               outline_color=COLOR_OUTLINE,
                               outline_width=1)
            else:
                is_highlight = (body == sled_body or id(body) in agent_body_ids)
                color = COLOR_AGENT if is_highlight else COLOR_ENV
                self.draw_body(body,
                               dynamic_color=color,
                               static_color=color,
                               outline_color=COLOR_OUTLINE,
                               outline_width=1)

        # ── Checkpoint zones ───────────────────────────────────────
        if hasattr(sandbox, 'get_terrain_bounds'):
            bounds = sandbox.get_terrain_bounds()
            for zone_key in ("checkpoint_zone", "checkpoint_b_zone"):
                cz = bounds.get(zone_key)
                if cz:
                    cx_min, cx_max = cz["x_min"], cz["x_max"]
                    cy_min, cy_max = cz["y_min"], cz["y_max"]
                    self.draw_line(cx_min, cy_min, cx_max, cy_min,
                                   COLOR_CHECKPOINT, 1)
                    self.draw_line(cx_max, cy_min, cx_max, cy_max,
                                   COLOR_CHECKPOINT, 1)
                    self.draw_line(cx_max, cy_max, cx_min, cy_max,
                                   COLOR_CHECKPOINT, 1)
                    self.draw_line(cx_min, cy_max, cx_min, cy_min,
                                   COLOR_CHECKPOINT, 1)

            # ── Target zone ────────────────────────────────────────
            tz = bounds.get("target_zone", {})
            if all(key in tz for key in ("x_min", "x_max", "y_min", "y_max")):
                tx_min = tz["x_min"]
                tx_max = tz["x_max"]
                ty_min = tz["y_min"]
                ty_max = tz["y_max"]
                self.draw_line(tx_min, ty_min, tx_max, ty_min,
                               COLOR_TARGET, 2)
                self.draw_line(tx_max, ty_min, tx_max, ty_max,
                               COLOR_TARGET, 2)
                self.draw_line(tx_max, ty_max, tx_min, ty_max,
                               COLOR_TARGET, 2)
                self.draw_line(tx_min, ty_max, tx_min, ty_min,
                               COLOR_TARGET, 2)

        # ── Annotations ────────────────────────────────────────────
        if self.simulator.can_display:
            self._init_fonts()
            _sh = self.simulator.screen_height

            # Top-left: task label
            if self._font_label:
                label = self._font_label.render("E-03 | Slippery World",
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
                    self.simulator.screen.blit(label_m,
                                               (bar_x + scale_px + 8, bar_y - 14))
