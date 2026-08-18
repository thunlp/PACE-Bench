import sys

import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../..'))

from pace_bench.core.renderer import Renderer

from Box2D.b2 import dynamicBody, staticBody

class BasicRenderer(Renderer):
    def render(self, sandbox, agent_body, target_x, camera_offset_x):
        self.set_camera_offset(camera_offset_x)
        self.clear((30, 30, 30))
        for body in sandbox.world.bodies:
            if body.type == staticBody:
                if abs(body.position.y) < 1.0:
                    self.draw_body(body,
                                 dynamic_color=(100, 150, 240),
                                 static_color=(150, 100, 50),
                                 outline_color=(200, 150, 100),
                                 outline_width=2)
                else:
                    self.draw_body(body,
                                 dynamic_color=(100, 150, 240),
                                 static_color=(255, 140, 0),
                                 outline_color=(255, 200, 0),
                                 outline_width=4)
        for body in sandbox.world.bodies:
            if body.type == dynamicBody:
                is_wheel = False
                for fixture in body.fixtures:
                    from Box2D.b2 import circleShape
                    if isinstance(fixture.shape, circleShape):
                        is_wheel = True
                        break
                if is_wheel:
                    self.draw_body(body,
                                 dynamic_color=(100, 200, 100),
                                 static_color=(150, 100, 50),
                                 outline_color=(50, 150, 50),
                                 outline_width=2)
                else:
                    self.draw_body(body,
                                 dynamic_color=(80, 130, 255),
                                 static_color=(150, 100, 50),
                                 outline_color=(200, 200, 255),
                                 outline_width=3)
        target_screen_x = int((target_x * self.simulator.ppm) - camera_offset_x)
        if 0 <= target_screen_x <= self.simulator.screen_width:
            self.draw_line(target_x, 0, target_x, 15, (255, 0, 0), 3)
