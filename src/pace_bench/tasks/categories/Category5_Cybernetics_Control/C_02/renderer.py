import pygame

try:
    from .environment import GROUND_SLAB_HEIGHT
except ImportError:
    from pace_bench.tasks.categories.Category5_Cybernetics_Control.C_02.environment import (
        GROUND_SLAB_HEIGHT,
    )
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
COLOR_BARRIER          = (234,  98,  85)  # barrier obstacles use target red
COLOR_PLATFORM          = ( 34,  80, 129)  # landing platform blue-gray


class C02Renderer(Renderer):
    def __init__(self, simulator):
        super().__init__(simulator)
        self.simulator.screen_width = 600
        self.simulator.screen_height = 600
        self.simulator.ppm = float(self.simulator.screen_height) / 22.0
        if self.simulator.can_display:
            self.simulator.screen = pygame.Surface((600, 600))
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
        # ── Camera ────────────────────────────────────────────────
        self.set_camera_offset(float(camera_offset_x), 12)
        _ = target_x

        # ── Background ──────────────────────────────────────────
        self.clear(COLOR_BG)

        # ── Identify lander body ────────────────────────────────
        lander = (
            sandbox._terrain_bodies.get("lander")
            if hasattr(sandbox, "_terrain_bodies")
            else None
        )

        # ── Draw bodies ─────────────────────────────────────────
        for body in sandbox.world.bodies:
            if body.type == staticBody:
                self.draw_body(body,
                               dynamic_color=COLOR_ENV,
                               static_color=COLOR_ENV,
                               outline_color=COLOR_OUTLINE,
                               outline_width=1)
            elif body == lander:
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

        # ── Barrier obstacle ────────────────────────────────────
        if hasattr(sandbox, "_barrier_x_left") and hasattr(sandbox, "_barrier_y_top"):
            bx_left = sandbox._barrier_x_left
            bx_right = sandbox._barrier_x_right
            by_top = sandbox._barrier_y_top
            ground_y = getattr(sandbox, "_ground_y_top", 1.0)
            obs_width = bx_right - bx_left
            obs_height = by_top - ground_y
            if obs_height > 0:
                self.draw_rect(
                    bx_left, ground_y + obs_height / 2.0, obs_width, obs_height,
                    COLOR_BARRIER, outline_color=COLOR_OUTLINE, outline_width=1
                )
            by_bottom = getattr(sandbox, "_barrier_y_bottom", 20.0)
            if by_bottom < 1e6:
                self.draw_line(bx_left, by_bottom, bx_right, by_bottom,
                               COLOR_BARRIER, width=2)

        # ── Landing platform ────────────────────────────────────
        if hasattr(sandbox, "_sim_time") and hasattr(sandbox, "get_platform_center_at_time"):
            t = sandbox._sim_time
            tx = sandbox.get_platform_center_at_time(t)
            ty = sandbox._ground_y_top
            hw = sandbox._platform_half_width
            slab_h = float(
                getattr(sandbox, "_ground_slab_height", GROUND_SLAB_HEIGHT)
            )
            self.draw_rect(
                tx - hw,
                ty - slab_h,
                2.0 * hw,
                slab_h,
                COLOR_PLATFORM,
                outline_color=COLOR_OUTLINE,
                outline_width=1,
            )
            # Top surface line on the platform
            self.draw_line(tx - hw, ty + 0.02, tx + hw, ty + 0.02,
                           COLOR_BOUNDARY, width=2)

        # ── Annotations ─────────────────────────────────────────
        if self.simulator.can_display:
            self._init_fonts()
            _sh = self.simulator.screen_height

            # Top-left: task label
            if self._font_label:
                label = self._font_label.render("C-02 | Lunar Lander",
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
