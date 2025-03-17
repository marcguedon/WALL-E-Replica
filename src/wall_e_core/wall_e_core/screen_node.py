import board
import math
import rclpy
import displayio
import terminalio
import os
import adafruit_display_shapes.circle as circle
import adafruit_display_shapes.rect as rect
from adafruit_display_text import label
from fourwire import FourWire
from adafruit_st7789 import ST7789
from rclpy.node import Node
from std_msgs.msg import Int8

PACKAGE_DIR = os.path.dirname(os.path.abspath(__file__))
WORKSPACE_DIR = os.path.abspath(os.path.join(PACKAGE_DIR, "../../../../../.."))
IMAGE_DIR = os.path.join(WORKSPACE_DIR, "images")

DC_PIN = board.D24
RST_PIN = board.D25
SPI_BAUDRATE = 40000000
SCREEN_WIDTH = 240
SCREEN_HEIGHT = 240

YELLOW = 0xFFFF00
BLACK = 0x000000


class Screen:
    def __init__(self, logger=None):
        self.logger = logger

        if not os.path.exists(IMAGE_DIR):
            raise FileNotFoundError(f"Image folder not found: {IMAGE_DIR}")

        spi = board.SPI()

        if not spi.try_lock():
            raise RuntimeError("Failed to acquire SPI lock")

        spi.configure(baudrate=SPI_BAUDRATE, phase=1, polarity=1)

        display_bus = FourWire(
            spi,
            command=DC_PIN,
            chip_select=None,
            reset=RST_PIN,
        )

        self.display = ST7789(
            display_bus, width=SCREEN_WIDTH, height=SCREEN_HEIGHT, rowstart=80
        )
        self.display.rotation = 180

        self.splash = displayio.Group()

        self.init()

    def init(self):
        self.display_image()

    def clear(self):        
        bitmap = displayio.Bitmap(SCREEN_WIDTH, SCREEN_HEIGHT, 1)
        palette = displayio.Palette(1)
        palette[0] = BLACK

        tile_grid = displayio.TileGrid(bitmap, pixel_shader=palette)
        self.splash = displayio.Group()
        self.splash.append(tile_grid)
        self.display.root_group = self.splash

    def update_battery_charge(self, battery_pct: int):
        self.clear()
        self.display_image()

        if battery_pct < 20:
            hidding_rect = rect.Rect(100, 59, width=120, height=138, fill=BLACK)

        else:
            height = int(((battery_pct // 10 * 10) - 100) * (140 - 0) / (10 - 100) + 0)
            hidding_rect = rect.Rect(100, 59, width=130, height=height, fill=BLACK)

        self.splash.append(hidding_rect)

    def display_image(self):
        bitmap = displayio.OnDiskBitmap(os.path.join(IMAGE_DIR, "wall-e.bmp"))
        tile_grid = displayio.TileGrid(bitmap, pixel_shader=bitmap.pixel_shader)

        self.splash.append(tile_grid)
        self.display.root_group = self.splash


class ScreenNode(Node):
    def __init__(self):
        super().__init__("screen_node")

        self.screen = Screen(logger=self.get_logger())

        self.battery_charge_subscription = self.create_subscription(
            Int8, "battery_charge_topic", self.battery_charge_callback, 10
        )

    def battery_charge_callback(self, msg):
        battery_pct = msg.data

        self.screen.update_battery_charge(battery_pct)
        self.get_logger().debug(
            f"battery_charge_callback calles\nbattery_pct: {battery_pct}"
        )

    def cleanup(self):
        self.screen.clear()
        super().destroy_node()
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ScreenNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested, stopping node...")
    except:
        pass
    finally:
        node.cleanup()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
