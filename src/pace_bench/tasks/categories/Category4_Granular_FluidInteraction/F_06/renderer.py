import pygame

from pace_bench.core.renderer import Renderer
from Box2D.b2 import staticBody

# ── Academic palette ──────────────────────────────────────────────
COLOR_BG          = (254, 252, 248)  # near-white background
COLOR_ENV          = ( 34,  80, 129)  # muted teal-gray — environment
COLOR_AGENT          = (234,  98,  85)  # dark slate blue — agent structures
COLOR_OUTLINE          = (109, 188, 208)  # dark blue-gray — unified outlines
COLOR_TARGET          = (237, 141,  73)  # muted red — goal marker
COLOR_BOUNDARY          = (207, 207, 207)  # medium gray — zone boundaries
COLOR_ANNOTATION          = ( 34,  80, 129)  # darker gray — text / labels
COLOR_FLUID          = ( 34,  80, 129)  # clean blue — fluid particles
COLOR_PIT          = (234,  98,  85)  # muted red — pit zones


class F06Renderer(Renderer):
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

        self.simulator.ppm = 30.0
        center_x_world = 13.0
        center_y_world = 7.0
        cam_x = center_x_world * self.simulator.ppm - self.simulator.screen_width / 2
        cam_y = self.simulator.screen_height / 2 - center_y_world * self.simulator.ppm
        self.set_camera_offset(cam_x, cam_y)

        # ── Background ─────────────────────────────────────────────
        self.clear(COLOR_BG)

        # ── Static environment bodies ──────────────────────────────
        for body in sandbox.world.bodies:
            if body.type == staticBody:
                self.draw_body(body,
                               dynamic_color=COLOR_ENV,
                               static_color=COLOR_ENV,
                               outline_color=COLOR_OUTLINE,
                               outline_width=1)

        # ── Fluid particles ────────────────────────────────────────
        if hasattr(sandbox, "_fluid_particles"):
            default_radius = getattr(sandbox, "_PARTICLE_RADIUS", 0.10)
            for p in sandbox._fluid_particles:
                if p is not None and p.active:
                    px, py = p.position.x, p.position.y
                    r = default_radius
                    for f in p.fixtures:
                        if hasattr(f.shape, "radius"):
                            r = f.shape.radius
                            break
                    self.draw_circle(px, py, r, COLOR_FLUID,
                                     outline_color=COLOR_OUTLINE,
                                     outline_width=1)

        # ── Agent bodies ───────────────────────────────────────────
        for body in sandbox._bodies:
            if body.active:
                self.draw_body(body,
                               dynamic_color=COLOR_AGENT,
                               static_color=COLOR_AGENT,
                               outline_color=COLOR_OUTLINE,
                               outline_width=1)

        # ── Build-zone boundary ────────────────────────────────────
        if hasattr(sandbox, "BUILD_ZONE_X_MIN"):
            x_min = sandbox.BUILD_ZONE_X_MIN
            x_max = sandbox.BUILD_ZONE_X_MAX
            y_min = sandbox.BUILD_ZONE_Y_MIN
            y_max = sandbox.BUILD_ZONE_Y_MAX
            self.draw_line(x_min, y_min, x_max, y_min, COLOR_BOUNDARY, 1)
            self.draw_line(x_max, y_min, x_max, y_max, COLOR_BOUNDARY, 1)
            self.draw_line(x_max, y_max, x_min, y_max, COLOR_BOUNDARY, 1)
            self.draw_line(x_min, y_max, x_min, y_min, COLOR_BOUNDARY, 1)

        # ── PIT zones ──────────────────────────────────────────────
        for pit_attr in ["PIT", "PIT2", "PIT3"]:
            if hasattr(sandbox, f"{pit_attr}_X_MIN"):
                px1 = getattr(sandbox, f"{pit_attr}_X_MIN")
                px2 = getattr(sandbox, f"{pit_attr}_X_MAX")
                py1 = getattr(sandbox, f"{pit_attr}_Y_MIN")
                py2 = getattr(sandbox, f"{pit_attr}_Y_MAX")
                self.draw_line(px1, py1, px2, py1, COLOR_PIT, 2)
                self.draw_line(px2, py1, px2, py2, COLOR_PIT, 2)
                self.draw_line(px2, py2, px1, py2, COLOR_PIT, 2)
                self.draw_line(px1, py2, px1, py1, COLOR_PIT, 2)

        # ── Headwind threshold line ────────────────────────────────
        if hasattr(sandbox, "HEADWIND_Y_THRESHOLD"):
            y_thresh = sandbox.HEADWIND_Y_THRESHOLD
            self.draw_line(0, y_thresh, 26, y_thresh, COLOR_BOUNDARY, 1)

        # ── Gravwell rectangle ─────────────────────────────────────
        if hasattr(sandbox, "GRAVWELL_X_MIN"):
            gx1, gx2 = sandbox.GRAVWELL_X_MIN, sandbox.GRAVWELL_X_MAX
            gy1, gy2 = sandbox.GRAVWELL_Y_MIN, sandbox.GRAVWELL_Y_MAX
            self.draw_line(gx1, gy1, gx2, gy1, COLOR_BOUNDARY, 1)
            self.draw_line(gx2, gy1, gx2, gy2, COLOR_BOUNDARY, 1)
            self.draw_line(gx2, gy2, gx1, gy2, COLOR_BOUNDARY, 1)
            self.draw_line(gx1, gy2, gx1, gy1, COLOR_BOUNDARY, 1)

        # ── Annotations ────────────────────────────────────────────
        if self.simulator.can_display:
            self._init_fonts()
            _sh = self.simulator.screen_height

            # Top-left: task label
            if self._font_label:
                label = self._font_label.render("F-06 | Pipeline",
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
