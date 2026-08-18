import pygame

from pace_bench.core.renderer import Renderer
from Box2D.b2 import dynamicBody, staticBody

# ── Academic palette ──────────────────────────────────────────────
COLOR_BG          = (254, 252, 248)  # near-white background
COLOR_ENV          = ( 34,  80, 129)  # muted teal-gray — environment
COLOR_AGENT          = (234,  98,  85)  # dark slate blue — agent structures
COLOR_OUTLINE          = (109, 188, 208)  # dark blue-gray — unified outlines
COLOR_TARGET          = (237, 141,  73)  # muted red — goal marker
COLOR_BOUNDARY          = (207, 207, 207)  # medium gray — boundary lines
COLOR_ANNOTATION          = ( 34,  80, 129)  # darker gray — text / labels
COLOR_SMALL          = ( 34,  80, 129)  # light blue particles
COLOR_MEDIUM          = (247, 207, 100)  # muted green particles
COLOR_LARGE          = (237, 141,  73)  # warm brown particles


class F04Renderer(Renderer):
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
        center_y_world = 5.0
        cam_x = center_x_world * self.simulator.ppm - self.simulator.screen_width / 2
        cam_y = self.simulator.screen_height / 2 - center_y_world * self.simulator.ppm
        self.set_camera_offset(cam_x, cam_y)

        # ── Floor length for zone separator lines ──────────────────
        floor_w = float(getattr(sandbox, "FLOOR_LENGTH", 16.0))

        # ── Background ─────────────────────────────────────────────
        self.clear(COLOR_BG)

        # ── Identify agent bodies ──────────────────────────────────
        agent_bodies = set(getattr(sandbox, "_bodies", []) or [])
        particle_bodies = set(
            (getattr(sandbox, "_particles_small", []) or [])
            + (getattr(sandbox, "_particles_medium", []) or [])
            + (getattr(sandbox, "_particles_large", []) or [])
        )

        # ── Draw environment bodies (including kinematic sweepers) ─
        for body in sandbox.world.bodies:
            if body in agent_bodies or body in particle_bodies:
                continue
            self.draw_body(
                body,
                dynamic_color=COLOR_ENV,
                static_color=COLOR_ENV,
                outline_color=COLOR_OUTLINE,
                outline_width=1,
            )

        # ── Draw particles: small ──────────────────────────────────
        if hasattr(sandbox, "_particles_small"):
            for p in sandbox._particles_small:
                if p is not None and p.active:
                    px, py = p.position.x, p.position.y
                    r = 0.06
                    for f in p.fixtures:
                        if hasattr(f.shape, "radius"):
                            r = f.shape.radius
                            break
                    self.draw_circle(px, py, r, COLOR_SMALL,
                                     outline_color=COLOR_OUTLINE, outline_width=1)

        # ── Draw particles: medium ─────────────────────────────────
        if hasattr(sandbox, "_particles_medium"):
            for p in sandbox._particles_medium:
                if p is not None and p.active:
                    px, py = p.position.x, p.position.y
                    r = 0.10
                    for f in p.fixtures:
                        if hasattr(f.shape, "radius"):
                            r = f.shape.radius
                            break
                    self.draw_circle(px, py, r, COLOR_MEDIUM,
                                     outline_color=COLOR_OUTLINE, outline_width=1)

        # ── Draw particles: large ──────────────────────────────────
        if hasattr(sandbox, "_particles_large"):
            for p in sandbox._particles_large:
                if p is not None and p.active:
                    px, py = p.position.x, p.position.y
                    r = 0.14
                    for f in p.fixtures:
                        if hasattr(f.shape, "radius"):
                            r = f.shape.radius
                            break
                    self.draw_circle(px, py, r, COLOR_LARGE,
                                     outline_color=COLOR_OUTLINE, outline_width=1)

        # ── Draw agent bodies ──────────────────────────────────────
        for body in sandbox._bodies:
            if body.active:
                self.draw_body(
                    body,
                    dynamic_color=COLOR_AGENT,
                    static_color=COLOR_AGENT,
                    outline_color=COLOR_OUTLINE,
                    outline_width=1,
                )

        # ── Zone separators ────────────────────────────────────────
        if hasattr(sandbox, "SMALL_ZONE_Y_MAX"):
            self.draw_line(0, sandbox.SMALL_ZONE_Y_MAX,
                           floor_w, sandbox.SMALL_ZONE_Y_MAX,
                           COLOR_BOUNDARY, 1)
        if hasattr(sandbox, "MEDIUM_ZONE_Y_MIN"):
            y_mid = sandbox.MEDIUM_ZONE_Y_MIN
            if not hasattr(sandbox, "SMALL_ZONE_Y_MAX") or abs(y_mid - sandbox.SMALL_ZONE_Y_MAX) > 1e-6:
                self.draw_line(0, y_mid, floor_w, y_mid, COLOR_BOUNDARY, 1)
        if hasattr(sandbox, "LARGE_ZONE_Y_MIN"):
            self.draw_line(0, sandbox.LARGE_ZONE_Y_MIN,
                           floor_w, sandbox.LARGE_ZONE_Y_MIN,
                           COLOR_BOUNDARY, 1)

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

        # ── Annotations ────────────────────────────────────────────
        if self.simulator.can_display:
            self._init_fonts()
            _sh = self.simulator.screen_height

            # Top-left: task label
            if self._font_label:
                label = self._font_label.render("F-04 | Three-Way Filter",
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
