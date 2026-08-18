"""
Common renderer module
Provides rendering functionality for Box2D objects
"""

import pygame
from Box2D.b2 import circleShape, dynamicBody, polygonShape


class Renderer:
    """
    Common renderer class
    Responsible for rendering objects in Box2D world
    """

    def __init__(self, simulator, camera_offset_x=0, camera_offset_y=0):
        self.simulator = simulator
        self.camera_offset_x = camera_offset_x
        self.camera_offset_y = camera_offset_y

    def set_camera_offset(self, x, y=0):
        """Set camera offset"""
        self.camera_offset_x = x
        self.camera_offset_y = y

    def world_to_screen(self, world_x, world_y):
        """
        Convert world coordinates to screen coordinates
        Box2D: y up, origin at bottom-left
        Pygame: y down, origin at top-left
        """
        screen_x = world_x * self.simulator.ppm - self.camera_offset_x
        screen_y = (
            self.simulator.screen_height
            - (world_y * self.simulator.ppm)
            - self.camera_offset_y
        )
        return (int(screen_x), int(screen_y))

    def clear(self, color=(254, 252, 248)):
        """Clear screen"""
        if self.simulator.can_display:
            self.simulator.screen.fill(color)

    def draw_body(
        self,
        body,
        dynamic_color=(234, 98, 85),
        static_color=(34, 80, 129),
        outline_color=(34, 80, 129),
        outline_width=1,
    ):
        """
        Draw a Box2D rigid body
        Args:
            body: Box2D body object
            dynamic_color: Dynamic object color
            static_color: Static object color
            outline_color: Outline color
            outline_width: Outline width
        """
        if not self.simulator.can_display:
            return

        for fixture in body.fixtures:
            shape = fixture.shape

            if isinstance(shape, circleShape):
                # Draw circle (wheels, etc.)
                pos = self.world_to_screen(body.position.x, body.position.y)
                radius = int(shape.radius * self.simulator.ppm)

                color = dynamic_color if body.type == dynamicBody else static_color
                pygame.draw.circle(self.simulator.screen, color, pos, radius)
                pygame.draw.circle(
                    self.simulator.screen, outline_color, pos, radius, outline_width
                )

                # Draw rotation indicator line
                p = body.GetWorldPoint((0, shape.radius))
                p_screen = self.world_to_screen(p.x, p.y)
                pygame.draw.line(self.simulator.screen, outline_color, pos, p_screen, 1)

            elif isinstance(shape, polygonShape):
                # Draw polygon (beams, terrain, etc.)
                # Use body.transform to convert vertices to world coordinates
                vertices_world = [body.transform * v for v in shape.vertices]
                # Convert to screen coordinates
                vertices = [self.world_to_screen(v.x, v.y) for v in vertices_world]

                color = dynamic_color if body.type == dynamicBody else static_color
                # Ensure at least 3 vertices to draw polygon
                if len(vertices) >= 3:
                    pygame.draw.polygon(self.simulator.screen, color, vertices)
                    pygame.draw.polygon(
                        self.simulator.screen, outline_color, vertices, outline_width
                    )

    def draw_circle(
        self, world_x, world_y, radius, color, outline_color=None, outline_width=1
    ):
        """Draw a circle (world coordinates)"""
        if not self.simulator.can_display:
            return
        pos = self.world_to_screen(world_x, world_y)
        radius_px = int(radius * self.simulator.ppm)
        pygame.draw.circle(self.simulator.screen, color, pos, radius_px)
        if outline_color:
            pygame.draw.circle(
                self.simulator.screen, outline_color, pos, radius_px, outline_width
            )

    def draw_rect(
        self,
        world_x,
        world_y,
        world_width,
        world_height,
        color,
        outline_color=None,
        outline_width=1,
    ):
        """Draw a rectangle (world coordinates)"""
        if not self.simulator.can_display:
            return
        screen_x = int(world_x * self.simulator.ppm - self.camera_offset_x)
        screen_y = int(
            self.simulator.screen_height
            - (world_y + world_height / 2) * self.simulator.ppm
        )
        screen_width = int(world_width * self.simulator.ppm)
        screen_height = int(world_height * self.simulator.ppm)
        rect = pygame.Rect(screen_x, screen_y, screen_width, screen_height)
        pygame.draw.rect(self.simulator.screen, color, rect)
        if outline_color:
            pygame.draw.rect(self.simulator.screen, outline_color, rect, outline_width)

    def draw_line(self, world_x1, world_y1, world_x2, world_y2, color, width=1):
        """Draw a line (world coordinates)"""
        if not self.simulator.can_display:
            return
        pos1 = self.world_to_screen(world_x1, world_y1)
        pos2 = self.world_to_screen(world_x2, world_y2)
        pygame.draw.line(self.simulator.screen, color, pos1, pos2, width)
