import board
import math
import os
import rclpy
import displayio
import terminalio
import adafruit_display_shapes.circle as circle
import adafruit_display_shapes.rect as rect
from adafruit_display_text import label
from adafruit_bitmap_font import bitmap_font
from fourwire import FourWire
from adafruit_st7789 import ST7789
from rclpy.node import Node
from std_msgs.msg import Int8

ROOT_PATH = os.path.dirname(os.path.abspath(__file__))
FONT_PATH = os.path.join(ROOT_PATH, "web_server/fonts/arial.ttf")

RST_PIN = 25
DC_PIN = 24
BLK_PIN = 27
SPI_BAUDRATE = 40000000

SCREEN_WIDTH = 240
SCREEN_HEIGHT = 240

YELLOW = (255, 255, 0)
BLACK = (0, 0, 0)


class Screen:
    def __init__(self):
        spi = board.SPI()

        if not spi.try_lock():
            raise RuntimeError("Failed to acquire SPI lock")

        spi.configure(baudrate=SPI_BAUDRATE)

        display_bus = FourWire(
            spi, command=board.D24, chip_select=board.D27, reset=board.D25
        )

        self.display = ST7789(
            display_bus, width=SCREEN_WIDTH, height=SCREEN_HEIGHT, rowstart=80
        )

        self.splash = displayio.Group()
        self.display.root_group = self.splash

        # Clear screen
        # self.clear()
        self.init()

    def draw_text(self):
        text = "SOLAR CHARGE LEVEL"
        # font = bitmap_font.load_font(FONT_PATH)
        text_area = label.Label(terminalio.FONT, text=text, color=0xFFFF00)
        text_width = text_area.bounding_box[2]
        text_x = (SCREEN_WIDTH - text_width) // 2
        text_y = 15
        text_group = displayio.Group(scale=2, x=text_x, y=text_y)
        text_group.append(text_area)
        self.splash.append(text_group)

    def draw_sun(self):
        sun_group = displayio.Group()
        sun_group.append(circle.Circle(34, 65, 15, fill=0xFFFF00))
        sun_group.append(circle.Circle(40, 71, 9, fill=0x000000))
        sun_group.append(rect.Rect(47, 51, width=4, height=10, fill=0xFFFF00))
        sun_group.append(rect.Rect(47, 99, width=4, height=10, fill=0xFFFF00))
        sun_group.append(rect.Rect(20, 78, width=10, height=4, fill=0xFFFF00))
        sun_group.append(rect.Rect(68, 78, width=10, height=4, fill=0xFFFF00))

        self.splash.append(sun_group)

        positions_angles = [
            (24, 66, 30),
            (33, 57, 60),
            (56, 57, 120),
            (65, 66, 150),
            (65, 91, 210),
            (56, 100, 240),
            (33, 100, 300),
            (24, 91, 330),
        ]

        for x, y, angle_deg in positions_angles:
            self.create_rotated_rectangle(x, y, 10, 4, math.radians(angle_deg))

    def draw_battery_charge(self):
        battery_group = displayio.Group()
        battery_group.append(rect.Rect(100, 61, width=120, height=8, fill=0xFFFF00))
        battery_group.append(rect.Rect(100, 77, width=120, height=8, fill=0xFFFF00))
        battery_group.append(rect.Rect(100, 93, width=120, height=8, fill=0xFFFF00))
        battery_group.append(rect.Rect(100, 109, width=120, height=8, fill=0xFFFF00))
        battery_group.append(rect.Rect(100, 125, width=120, height=8, fill=0xFFFF00))
        battery_group.append(rect.Rect(100, 141, width=120, height=8, fill=0xFFFF00))
        battery_group.append(rect.Rect(100, 157, width=120, height=8, fill=0xFFFF00))
        battery_group.append(rect.Rect(100, 173, width=120, height=8, fill=0xFFFF00))
        battery_group.append(rect.Rect(100, 189, width=120, height=8, fill=0xFFFF00))
        battery_group.append(rect.Rect(100, 205, width=120, height=25, fill=0xFFFF00))

        self.splash.append(battery_group)

    def init(self):
        self.draw_text()
        self.draw_sun()
        self.draw_battery_charge()

        self.update_battery_charge(0)

        # print("Initialized screen")

    # def clear(self):
    #     self.display.clear()

    def update_battery_charge(self, battery_pct: int):
        self.draw_battery_charge()

        if battery_pct < 20:
            hidding_rect = rect.Rect(100, 59, width=120, height=138, fill=0x000000)

        else:
            height = int(((battery_pct // 10 * 10) - 100) * (140 - 0) / (10 - 100) + 0)
            hidding_rect = rect.Rect(100, 59, width=120, height=height, fill=0x000000)

        self.splash.append(hidding_rect)

    def create_rotated_rectangle(self, x, y, width, height, angle):
        rectangle = rect.Rect(x, y, width, height, fill=0xFFFF00)
        rectangle.rotation = angle

        self.splash.append(rectangle)


class ScreenNode(Node):
    def __init__(self):
        super().__init__("screen_node")

        self.screen = Screen()

        self.battery_charge_subscription = self.create_subscription(
            Int8, "battery_charge_topic", self.battery_charge_callback, 10
        )

    def battery_charge_callback(self, msg):
        battery_pct = msg.data

        self.screen.update_battery_charge(battery_pct)


def main(args=None):
    rclpy.init(args=args)
    screen_node = ScreenNode()
    rclpy.spin(screen_node)
    screen_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
