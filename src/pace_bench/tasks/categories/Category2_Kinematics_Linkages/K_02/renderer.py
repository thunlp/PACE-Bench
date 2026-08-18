import pygame
import math

from pace_bench.core.renderer import Renderer
from Box2D.b2 import dynamicBody, staticBody, revoluteJoint

# ── Academic palette ──────────────────────────────────────────────
COLOR_BG          = (254, 252, 248)
COLOR_ENV          = ( 34,  80, 129)
COLOR_AGENT          = (234,  98,  85)
COLOR_OUTLINE          = (109, 188, 208)
COLOR_TARGET          = (237, 141,  73)
COLOR_BOUNDARY          = (207, 207, 207)
COLOR_ANNOTATION          = ( 34,  80, 129)
COLOR_JOINT          = (247, 207, 100)
COLOR_TEMPLATE          = (251, 236, 165)
COLOR_BARRIER          = (234,  98,  85)
COLOR_PLATFORM          = ( 34,  80, 129)


class K02Renderer(Renderer):
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

    def render(self, sandbox, agent_body, target_y, camera_offset_x):
        # ── Square aspect ratio (600×600) ──────────────────────────
        if self.simulator.screen_width != 600 or self.simulator.screen_height != 600:
            self.simulator.screen_width = 600
            self.simulator.screen_height = 600
            if self.simulator.can_display:
                self.simulator.screen = pygame.Surface((600, 600))

        self.simulator.ppm = 20.0
        center_x_world = 6.0
        center_y_world = 12.0
        cam_x = center_x_world * self.simulator.ppm - self.simulator.screen_width / 2
        cam_y = self.simulator.screen_height / 2 - center_y_world * self.simulator.ppm
        self.set_camera_offset(cam_x, cam_y)

        self.clear(COLOR_BG)

        # ── Static environment ──────────────────────────────────
        for body in sandbox.world.bodies:
            if body.type == staticBody:
                self.draw_body(body,
                               dynamic_color=COLOR_ENV,
                               static_color=COLOR_ENV,
                               outline_color=COLOR_OUTLINE,
                               outline_width=1)

        # ── Dynamic bodies ──────────────────────────────────────
        for body in sandbox.world.bodies:
            if body.type == dynamicBody:
                is_template = False
                if hasattr(sandbox, '_climber_bodies'):
                    for key, value in sandbox._climber_bodies.items():
                        if 'template' in key and body == value:
                            is_template = True
                            break
                if is_template:
                    self.draw_body(body,
                                   dynamic_color=COLOR_TEMPLATE,
                                   static_color=COLOR_TEMPLATE,
                                   outline_color=COLOR_TEMPLATE,
                                   outline_width=1)
                else:
                    self.draw_body(body,
                                   dynamic_color=COLOR_AGENT,
                                   static_color=COLOR_AGENT,
                                   outline_color=COLOR_OUTLINE,
                                   outline_width=1)

        # ── Joint dots ──────────────────────────────────────────
        if hasattr(sandbox, '_joints'):
            seen = set()
            for joint in sandbox._joints:
                if not isinstance(joint, revoluteJoint) or not joint.bodyA or not joint.bodyB:
                    continue
                try:
                    if hasattr(joint, 'localAnchorA'):
                        local_anchor = joint.localAnchorA
                    elif hasattr(joint, 'anchorA'):
                        local_anchor = joint.anchorA
                    else:
                        continue
                    world_anchor = joint.bodyA.GetWorldPoint(local_anchor)
                    key = (round(world_anchor.x, 3), round(world_anchor.y, 3))
                    if key in seen:
                        continue
                    seen.add(key)
                    anchor_screen = self.world_to_screen(world_anchor.x, world_anchor.y)
                    pygame.draw.circle(self.simulator.screen, COLOR_JOINT, anchor_screen, 4)
                except Exception:
                    pass

        # ── Target line ─────────────────────────────────────────
        if target_y and target_y > 0:
            wall_x = 5.0
            if hasattr(sandbox, '_wall_x'):
                wall_x = sandbox._wall_x
            self.draw_line(wall_x, target_y, wall_x + 3.0, target_y, COLOR_TARGET, 4)

        # ── Barrier obstacle ────────────────────────────────────
        if hasattr(sandbox, "_barrier_x_left") and hasattr(sandbox, "_barrier_y_top"):
            bx_left = sandbox._barrier_x_left
            bx_right = sandbox._barrier_x_right
            by_top = sandbox._barrier_y_top
            ground_y = getattr(sandbox, "_ground_y_top", 1.0)
            obs_width = bx_right - bx_left
            obs_height = by_top - ground_y
            if obs_height > 0:
                self.draw_rect(bx_left, ground_y + obs_height / 2.0, obs_width, obs_height,
                               COLOR_BARRIER, outline_color=COLOR_OUTLINE, outline_width=1)
            by_bottom = getattr(sandbox, "_barrier_y_bottom", 20.0)
            if by_bottom < 1e6:
                self.draw_line(bx_left, by_bottom, bx_right, by_bottom, COLOR_BARRIER, width=2)

        # ── Landing platform ────────────────────────────────────
        if hasattr(sandbox, "_sim_time") and hasattr(sandbox, "get_platform_center_at_time"):
            t = sandbox._sim_time
            tx = sandbox.get_platform_center_at_time(t)
            ty = sandbox._ground_y_top
            hw = sandbox._platform_half_width
            slab_h = float(getattr(sandbox, "_ground_slab_height", 1.0))
            self.draw_rect(tx - hw, ty - slab_h, 2.0 * hw, slab_h,
                           COLOR_PLATFORM, outline_color=COLOR_OUTLINE, outline_width=1)
            self.draw_line(tx - hw, ty + 0.02, tx + hw, ty + 0.02, COLOR_BOUNDARY, width=2)

        # ── Build-zone boundary ─────────────────────────────────
        if hasattr(sandbox, 'BUILD_ZONE_X_MIN'):
            x_min = sandbox.BUILD_ZONE_X_MIN
            x_max = sandbox.BUILD_ZONE_X_MAX
            y_min = sandbox.BUILD_ZONE_Y_MIN
            y_max = sandbox.BUILD_ZONE_Y_MAX
            self.draw_line(x_min, y_min, x_max, y_min, COLOR_BOUNDARY, 1)
            self.draw_line(x_max, y_min, x_max, y_max, COLOR_BOUNDARY, 1)
            self.draw_line(x_max, y_max, x_min, y_max, COLOR_BOUNDARY, 1)
            self.draw_line(x_min, y_max, x_min, y_min, COLOR_BOUNDARY, 1)

        # ── Annotations ─────────────────────────────────────────
        if self.simulator.can_display:
            self._init_fonts()
            _sh = self.simulator.screen_height

            if self._font_label:
                label = self._font_label.render("K-02 | Climber",
                                                True, COLOR_ANNOTATION)
                self.simulator.screen.blit(label, (18, 14))

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
