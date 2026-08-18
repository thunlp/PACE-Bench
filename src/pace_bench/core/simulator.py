"""
Common simulator module
Provides physics simulation, rendering, GIF generation and other common functionality
"""

import os

# CRITICAL: Force headless BEFORE importing pygame
if not os.environ.get("DISPLAY"):
    os.environ["SDL_VIDEODRIVER"] = "dummy"
os.environ["SDL_AUDIODRIVER"] = "dummy"
os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"

import warnings

pygame = None


def _load_pygame():
    """Import pygame only when display or rendering functionality is requested."""

    global pygame
    if pygame is None:
        import pygame as pygame_module

        pygame = pygame_module
    return pygame


# Try importing PIL/Pillow for GIF generation
try:
    from PIL import Image

    HAS_PIL = True
except ImportError:
    HAS_PIL = False

# Suppress warnings
os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"
warnings.filterwarnings("ignore", category=UserWarning)


def _count_gif_frames(gif_path):
    """Count actual readable frames in a GIF (PIL collapses identical frames)."""
    try:
        from PIL import Image

        img = Image.open(gif_path)
        n = 0
        try:
            while True:
                img.seek(n)
                n += 1
        except EOFError:
            pass
        img.close()
        return n
    except Exception:
        return -1


# --- Global configuration ---
PPM = 40.0  # Pixels Per Meter
TARGET_FPS = 60
TIME_STEP = 1.0 / TARGET_FPS
SCREEN_WIDTH, SCREEN_HEIGHT = 800, 600


class Simulator:
    """
    Common simulator class
    Responsible for physics stepping, rendering, GIF generation, etc.
    """

    _global_skip_gif = False

    def __init__(self, screen_width=SCREEN_WIDTH, screen_height=SCREEN_HEIGHT, ppm=PPM):
        self.screen_width = screen_width
        self.screen_height = screen_height
        self.ppm = ppm
        self.screen = None
        self.clock = None
        self.can_display = False
        self.frames = []
        self.save_gif = False

    def init_display(self, headless=False, save_gif=False):
        """
        Initialize display system
        Args:
            headless: If True, don't display window
            save_gif: If True, save as GIF
        """
        if Simulator._global_skip_gif:
            save_gif = False

        self.save_gif = save_gif
        _load_pygame()
        pygame.init()
        # ... rest of method unchanged

        if not headless or save_gif:
            try:
                if headless or not os.environ.get("DISPLAY"):
                    os.environ["SDL_VIDEODRIVER"] = "dummy"
                    self.screen = pygame.Surface(
                        (self.screen_width, self.screen_height)
                    )
                    self.can_display = True
                    if save_gif:
                        print(
                            "Headless mode: Creating offscreen surface for GIF saving"
                        )
                    else:
                        print("Headless mode: Creating offscreen surface")
                else:
                    self.screen = pygame.display.set_mode(
                        (self.screen_width, self.screen_height)
                    )
                    pygame.display.set_caption("DaVinciBench Simulation")
                    self.clock = pygame.time.Clock()
                    self.can_display = True
                    print("Graphics interface started")
            except Exception as e:
                print(f"Cannot create graphics window: {e}")
                if not headless:
                    print("Trying to create offscreen surface...")
                    try:
                        os.environ["SDL_VIDEODRIVER"] = "dummy"
                        self.screen = pygame.Surface(
                            (self.screen_width, self.screen_height)
                        )
                        self.can_display = True
                        print("Successfully created offscreen surface")
                    except Exception as e2:
                        print(f"Cannot create offscreen surface: {e2}")
                        print(
                            "Switching to pure headless mode (physics simulation only)"
                        )
                        self.can_display = False
                else:
                    self.can_display = False

        if save_gif and self.can_display:
            if HAS_PIL:
                print("Will save as GIF animation...")
            else:
                print(
                    "Warning: Pillow not installed, cannot generate GIF. Please run: pip install Pillow"
                )
                self.save_gif = False

        return self.can_display

    def collect_frame(self, step_count, frame_interval=20):
        """
        Collect frames for GIF generation
        Args:
            step_count: Current step count
            frame_interval: Collect every N frames (0 means always collect)
        """
        if (
            self.save_gif
            and HAS_PIL
            and self.can_display
            and (step_count == 0 or step_count % frame_interval == 0)
        ):
            try:
                img_str = pygame.image.tostring(self.screen, "RGB")
                img = Image.frombytes(
                    "RGB", (self.screen_width, self.screen_height), img_str
                )
                self.frames.append(img)
            except Exception as e:
                print(f"Warning: Failed to collect frame at step {step_count}: {e}")

    def save_gif_animation(self, gif_path, duration=167):
        """
        Save GIF animation
        Args:
            gif_path: GIF file path
            duration: Delay per frame (milliseconds), must be 1-65535 for GIF format
        """
        if Simulator._global_skip_gif:
            return False

        duration = max(1, min(int(duration), 65535))
        if not self.save_gif:
            print("⚠️  GIF saving disabled (save_gif=False)")
            return False

        if not HAS_PIL:
            print(
                "⚠️  Cannot save GIF: Pillow not installed. Please run: pip install Pillow"
            )
            return False

        if not self.frames:
            print("⚠️  Cannot save GIF: No frames collected (frames list is empty)")
            print("   This may happen if:")
            print("   - Simulation ended before any frames were collected")
            print(
                f"   - Rendering was disabled or failed (can_display={self.can_display}, save_gif={self.save_gif}, HAS_PIL={HAS_PIL})"
            )
            print("   - Agent body was None and renderer couldn't render environment")
            print("   - Screen was not initialized or rendering never occurred")
            return False

        try:
            os.makedirs(os.path.dirname(gif_path), exist_ok=True)
            print(f"Generating GIF animation: {gif_path}...")
            # Limit frames to avoid huge files and PIL uint16 duration overflow
            max_frames = 200
            if len(self.frames) <= max_frames:
                images = self.frames
            else:
                step = len(self.frames) // max_frames
                images = [self.frames[i] for i in range(0, len(self.frames), step)][
                    :max_frames
                ]
            images[0].save(
                gif_path,
                save_all=True,
                append_images=images[1:] if len(images) > 1 else [],
                duration=duration,
                loop=0,
            )
            print(f"✅ GIF animation saved: {gif_path}")
            actual_frames = _count_gif_frames(gif_path)
            print(f"   Total {len(images)} frames collected → {actual_frames} stored\n")
            return True
        except Exception as e:
            if isinstance(e, OSError) and e.errno == 122:
                print(
                    "⚠️  Disk quota exceeded! Disabling GIF saving for all subsequent runs."
                )
                Simulator._global_skip_gif = True
            print(f"❌ Error saving GIF: {e}")
            import traceback

            traceback.print_exc()
            return False

    def quit(self):
        """Clean up resources"""
        if pygame is not None and pygame.get_init():
            pygame.quit()
        self.screen = None
        self.clock = None
        self.frames.clear()
        self.can_display = False

    def handle_events(self):
        """Handle pygame events"""
        if self.can_display and self.clock is not None:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return False
        return True

    def tick(self):
        """Clock tick (for controlling frame rate)"""
        if self.clock is not None:
            self.clock.tick(TARGET_FPS)

    def flip(self):
        """Refresh display"""
        if self.clock is not None:
            pygame.display.flip()
