import math
import pygame

from pace_bench.core.renderer import Renderer
from Box2D.b2 import dynamicBody, staticBody, revoluteJoint, polygonShape, circleShape

# ── Academic palette ──────────────────────────────────────────────
COLOR_BG          = (254, 252, 248)
COLOR_ENV          = ( 34,  80, 129)
COLOR_AGENT          = (234,  98,  85)
COLOR_OUTLINE          = (109, 188, 208)
COLOR_TARGET          = (237, 141,  73)
COLOR_BOUNDARY          = (207, 207, 207)
COLOR_ANNOTATION          = ( 34,  80, 129)
COLOR_JOINT          = (247, 207, 100)
COLOR_GLASS          = (207, 207, 207)
COLOR_BIN_PARTICLE          = (109, 188, 208)


class K06Renderer(Renderer):
    GLASS_Y = 2.0
    GLASS_X_MIN, GLASS_X_MAX = 0.5, 11.5
    BIN_LEFT_X = 0.3
    BIN_RIGHT_X = 11.7
    BIN_Y_OFFSET = 0.3
    PARTICLE_RADIUS_PX = 12
    SCALE_X = 50.0
    SCALE_Y = 50.0

    def __init__(self, simulator):
        super().__init__(simulator)
        self._font_body = None
        self._font_label = None
        if simulator.can_display:
            if simulator.screen_width != 600 or simulator.screen_height != 600:
                simulator.screen_width = 600
                simulator.screen_height = 600
                simulator.screen = pygame.Surface((600, 600))

    def _init_fonts(self):
        if self._font_label is not None:
            return
        try:
            self._font_body = pygame.font.SysFont("DejaVu Sans", 28)
            self._font_label = pygame.font.SysFont("DejaVu Sans", 40)
        except Exception:
            self._font_body = pygame.font.Font(None, 28)
            self._font_label = pygame.font.Font(None, 40)

    def _on_glass(self, sandbox, particle):
        glass_y = getattr(sandbox, '_glass_y', 2.0)
        x, y = particle.position.x, particle.position.y
        return (self.GLASS_X_MIN <= x <= self.GLASS_X_MAX and
                abs(y - glass_y) < 0.5)

    def _to_screen(self, px, py):
        w = self.simulator.screen_width
        h = self.simulator.screen_height
        sx, sy = self.SCALE_X, self.SCALE_Y
        cx, cy = 6.0, self.GLASS_Y
        screen_x = int((px - cx) * sx + w / 2)
        screen_y = int(h / 2 - (py - cy) * sy)
        return (screen_x, screen_y)

    def _draw_body(self, body, color, outline_color=None):
        if outline_color is None:
            outline_color = COLOR_OUTLINE
        screen = self.simulator.screen
        for fixture in body.fixtures:
            shape = fixture.shape
            if isinstance(shape, polygonShape):
                verts = [body.GetWorldPoint(v) for v in shape.vertices]
                screen_verts = [self._to_screen(v.x, v.y) for v in verts]
                if len(screen_verts) >= 3:
                    pygame.draw.polygon(screen, color, screen_verts)
                    pygame.draw.polygon(screen, outline_color, screen_verts, 1)
            elif isinstance(shape, circleShape):
                pos = body.worldCenter
                sp = self._to_screen(pos.x, pos.y)
                r = int(shape.radius * self.SCALE_X)
                pygame.draw.circle(screen, color, sp, r)
                pygame.draw.circle(screen, outline_color, sp, r, 1)

    def _draw_joint(self, joint, is_motor=False):
        screen = self.simulator.screen
        color = COLOR_TARGET if is_motor else COLOR_JOINT
        radius = 10 if is_motor else 6
        try:
            anchor = joint.anchorA
            sp = self._to_screen(anchor.x, anchor.y)
            pygame.draw.circle(screen, color, sp, radius)
            pygame.draw.circle(screen, COLOR_OUTLINE, sp, radius, 1)
        except Exception:
            pass

    def render(self, sandbox, agent_body, target_x, camera_offset_x):
        if not self.simulator.screen:
            return

        # ── Square aspect ratio reassign (defensive) ────────────────
        if self.simulator.screen_width != 600 or self.simulator.screen_height != 600:
            self.simulator.screen_width = 600
            self.simulator.screen_height = 600
            if self.simulator.can_display:
                self.simulator.screen = pygame.Surface((600, 600))

        self.clear(COLOR_BG)

        # ── Glass pane ──────────────────────────────────────────────
        gx0, gx1 = self.GLASS_X_MIN, self.GLASS_X_MAX
        gy = self.GLASS_Y
        glass_half_height = 0.7
        pts_glass = [
            self._to_screen(gx0, gy - glass_half_height),
            self._to_screen(gx1, gy - glass_half_height),
            self._to_screen(gx1, gy + glass_half_height),
            self._to_screen(gx0, gy + glass_half_height),
        ]
        pygame.draw.polygon(self.simulator.screen, COLOR_GLASS, pts_glass)
        pygame.draw.polygon(self.simulator.screen, COLOR_BOUNDARY, pts_glass, 2)

        # ── Particles ───────────────────────────────────────────────
        if hasattr(sandbox, '_particles'):
            left_bin_count = 0
            right_bin_count = 0
            for particle in sandbox._particles:
                px, py = particle.position.x, particle.position.y
                if self._on_glass(sandbox, particle):
                    pos = self._to_screen(px, py)
                    pygame.draw.circle(self.simulator.screen, COLOR_ENV, pos, self.PARTICLE_RADIUS_PX)
                    pygame.draw.circle(self.simulator.screen, COLOR_OUTLINE, pos, self.PARTICLE_RADIUS_PX, 1)
                else:
                    r_small = 6
                    if px < self.GLASS_X_MIN:
                        left_bin_count += 1
                        by = self.GLASS_Y + self.BIN_Y_OFFSET * (left_bin_count % 3 - 1)
                        pos = self._to_screen(self.BIN_LEFT_X, by)
                    else:
                        right_bin_count += 1
                        by = self.GLASS_Y + self.BIN_Y_OFFSET * (right_bin_count % 3 - 1)
                        pos = self._to_screen(self.BIN_RIGHT_X, by)
                    pygame.draw.circle(self.simulator.screen, COLOR_BIN_PARTICLE, pos, r_small)
                    pygame.draw.circle(self.simulator.screen, COLOR_BOUNDARY, pos, r_small, 1)

        # ── Agent bodies (wiper mechanism) ──────────────────────────
        if hasattr(sandbox, '_bodies'):
            for body in sandbox._bodies:
                self._draw_body(body, COLOR_AGENT)

        # ── Joints ──────────────────────────────────────────────────
        motor_joint = getattr(sandbox, '_wiper_motor_joint', None)
        if hasattr(sandbox, '_joints'):
            for joint in sandbox._joints:
                is_motor = (joint is motor_joint) or (
                    isinstance(joint, revoluteJoint) and getattr(joint, 'motorEnabled', False)
                )
                self._draw_joint(joint, is_motor=is_motor)
        if hasattr(sandbox, '_wiper_joints'):
            for joint in sandbox._wiper_joints:
                is_motor = (joint is motor_joint) or (
                    isinstance(joint, revoluteJoint) and getattr(joint, 'motorEnabled', False)
                )
                self._draw_joint(joint, is_motor=is_motor)

        # ── Annotations ─────────────────────────────────────────────
        if self.simulator.can_display:
            self._init_fonts()
            _sh = self.simulator.screen_height

            if self._font_label:
                label = self._font_label.render("K-06 | Wiper",
                                                True, COLOR_ANNOTATION)
                self.simulator.screen.blit(label, (18, 14))

            if self.SCALE_X > 0:
                scale_px = int(1.0 * self.SCALE_X)
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
