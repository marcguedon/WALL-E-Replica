import board
import displayio
import os
import time
import adafruit_display_shapes.rect as rect
from fourwire import FourWire
from adafruit_st7789 import ST7789

PACKAGE_DIR = os.path.dirname(os.path.abspath(__file__))
WORKSPACE_DIR = os.path.abspath(os.path.join(PACKAGE_DIR, "../../.."))
IMAGE_DIR = os.path.join(WORKSPACE_DIR, "images")
WALL_E_IMAGE = os.path.join(IMAGE_DIR, "wall-e.bmp")

DC_PIN = board.D24
RST_PIN = board.D25
SCREEN_WIDTH = 240
SCREEN_HEIGHT = 240

BLACK = 0x000000


class Screen:
    def __init__(self):
        if not os.path.exists(IMAGE_DIR):
            raise FileNotFoundError(f"Image folder not found: {IMAGE_DIR}")

        spi = board.SPI()

        if not spi.try_lock():
            raise RuntimeError("Failed to acquire SPI lock")

        spi.configure(phase=1, polarity=1)

        display_bus = FourWire(
            spi,
            command=DC_PIN,
            chip_select=None,
            reset=RST_PIN,
        )

        self.display = ST7789(
            display_bus,
            width=SCREEN_WIDTH,
            height=SCREEN_HEIGHT,
            rowstart=80,
        )
        self.display.rotation = 180

        self.splash = displayio.Group()
        self.display.root_group = self.splash

        self.hidding_rect = None

        self.init()

    def init(self):
        self.display_image()
        self.update_battery_charge(100)

    def clear(self):
        for _ in range(len(self.splash)):
            self.splash.pop()

        self.display.root_group = self.splash

    def update_battery_charge(self, battery_pct: int):
        if battery_pct > 100 or battery_pct < 0:
            raise Exception(f"Invalid battery charge value: {battery_pct}")

        if self.hidding_rect in self.splash:
            self.splash.remove(self.hidding_rect)
            self.display.root_group = self.splash

        height = 1

        if battery_pct < 100:
            if battery_pct <= 20:
                height = 142

            else:
                height = int(
                    ((battery_pct // 11 * 11) - 100) * (145 - 0) / (10 - 100) + 0
                )

        self.hidding_rect = rect.Rect(100, 59, width=130, height=height, fill=0x00FF00)

        self.splash.append(self.hidding_rect)
        self.display.root_group = self.splash

    def display_image(self):
        bitmap = displayio.OnDiskBitmap(WALL_E_IMAGE)
        tile_grid = displayio.TileGrid(bitmap, pixel_shader=bitmap.pixel_shader)

        self.splash.append(tile_grid)
        self.display.root_group = self.splash


def main(args=None):
    screen = Screen()

    while True:
        for i in range(0, 101, 10):
            screen.update_battery_charge(i)
            print(i)
            time.sleep(2)


if __name__ == "__main__":
    main()
