import board
import microcontroller
import rclpy
import displayio
import os
import adafruit_display_shapes.rect as rect
from fourwire import FourWire
from adafruit_st7789 import ST7789
from rclpy.node import Node
from std_msgs.msg import Int8

PACKAGE_DIR = os.path.dirname(os.path.abspath(__file__))
WORKSPACE_DIR = os.path.abspath(os.path.join(PACKAGE_DIR, "../../../../../.."))
IMAGE_DIR = os.path.join(WORKSPACE_DIR, "images")
WALL_E_IMAGE = os.path.join(IMAGE_DIR, "wall-e.bmp")

SCREEN_WIDTH = 240
SCREEN_HEIGHT = 240

BLACK = 0x000000


class Screen:
    def __init__(
        self,
        dc_pin: int,
        rst_pin: int,
        cs_pin: int,
        blk_pin: int,
        baudrate: int,
        logger=None,
    ):
        self.logger = logger

        if not os.path.exists(IMAGE_DIR):
            raise FileNotFoundError(f"Image folder not found: {IMAGE_DIR}")

        spi = board.SPI()

        if not spi.try_lock():
            raise RuntimeError("Failed to acquire SPI lock")

        spi.configure(baudrate=baudrate, phase=1, polarity=1)

        # TODO Modifier la gestion des broches
        display_bus = FourWire(
            spi,
            command=microcontroller.Pin(dc_pin),
            chip_select=microcontroller.Pin(cs_pin),
            reset=microcontroller.Pin(rst_pin),
        )

        self.display = ST7789(
            display_bus,
            width=SCREEN_WIDTH,
            height=SCREEN_HEIGHT,
            rowstart=80,
            backlight_pin=microcontroller.pin(blk_pin),
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

        self.hidding_rect = rect.Rect(100, 59, width=130, height=height, fill=BLACK)

        self.splash.append(self.hidding_rect)
        self.display.root_group = self.splash

    def display_image(self):
        bitmap = displayio.OnDiskBitmap(WALL_E_IMAGE)
        tile_grid = displayio.TileGrid(bitmap, pixel_shader=bitmap.pixel_shader)

        self.splash.append(tile_grid)
        self.display.root_group = self.splash


DEFAULT_DC_PIN = 24
DEFAULT_RST_PIN = 25
DEFAULT_CS_PIN = 8
DEFAULT_BLK_PIN = 27
DEFAULT_BAUDRATE = None


class ScreenNode(Node):
    def __init__(self):
        super().__init__("screen_node")

        self.declare_parameter("dc_pin", DEFAULT_DC_PIN)
        self.declare_parameter("rst_pin", DEFAULT_RST_PIN)
        self.declare_parameter("cs_pin", DEFAULT_CS_PIN)
        self.declare_parameter("blk_pin", DEFAULT_BLK_PIN)
        self.declare_parameter("baudrate", DEFAULT_BAUDRATE)

        dc_pin = self.get_parameter("dc_pin").get_parameter_value().integer_value
        rst_pin = self.get_parameter("rst_pin").get_parameter_value().integer_value
        cs_pin = self.get_parameter("cs_pin").get_parameter_value().integer_value
        cs_pin = None if cs_pin == -1 else int(cs_pin)
        blk_pin = self.get_parameter("blk_pin").get_parameter_value().integer_value
        blk_pin = None if blk_pin == -1 else int(blk_pin)
        baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        baudrate = None if baudrate == -1 else int(baudrate)

        self.screen = Screen(
            dc_pin, rst_pin, cs_pin, blk_pin, baudrate, logger=self.get_logger()
        )

        self.battery_charge_subscription = self.create_subscription(
            Int8, "battery_charge_topic", self.battery_charge_callback, 10
        )

    def battery_charge_callback(self, msg):
        battery_pct = msg.data

        self.screen.update_battery_charge(battery_pct)
        self.get_logger().debug(
            f"battery_charge_callback called\nbattery_pct: {battery_pct}"
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
