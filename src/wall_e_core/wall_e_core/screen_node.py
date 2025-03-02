import Adafruit_GPIO.SPI as SPI
import ST7789 as TFT
import math
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from PIL import Image, ImageDraw, ImageFont

ROOT_PATH = os.path.dirname(os.path.abspath(__file__))
FONT_PATH = os.path.join(ROOT_PATH, "web_server/fonts/arial.ttf")

RST_PIN = 25
DC_PIN = 24
BLK_PIN = 27
SPI_PORT = 0
SPI_DEVICE = 0
SPI_MODE = 0b11
SPI_SPEED_HZ = 40000000

SCREEN_WIDTH = 240
SCREEN_HEIGHT = 240


class Screen:
    def __init__(
        self,
        spi_port: int,
        spi_device: int,
        spi_speed_hz: int,
        spi_mode: int,
        rst_pin: int,
        dc_pin: int,
        blk_pin: int,
    ):
        self.disp = TFT.ST7789(
            spi=SPI.SpiDev(spi_port, spi_device, max_speed_hz=spi_speed_hz),
            mode=spi_mode,
            rst=rst_pin,
            dc=dc_pin,
            led=blk_pin,
        )
        self.image = Image.new("RGB", (SCREEN_WIDTH, SCREEN_HEIGHT), color=(0, 0, 0))
        self.draw = ImageDraw.Draw(self.image)

        self.disp.begin()

        # Clear screen
        self.clear()
        self.init()

    def init(self):
        text = "SOLAR CHARGE LEVEL"
        font = ImageFont.truetype(FONT_PATH, 18)
        text_bbox = self.draw.textbbox((0, 0), text, font=font)
        text_width = text_bbox[2] - text_bbox[0]
        text_x = (SCREEN_WIDTH - text_width) // 2
        text_y = 15
        self.draw.text((text_x, text_y), text, font=font, fill=(255, 255, 0))

        # Draw yellow sun
        self.draw.ellipse((34, 65, 64, 95), fill=(255, 255, 0))
        self.draw.ellipse((40, 71, 58, 89), fill=(0, 0, 0))
        self.draw.rectangle((47, 51, 51, 61), fill=(255, 255, 0))
        self.draw.rectangle((47, 99, 51, 109), fill=(255, 255, 0))
        self.draw.rectangle((20, 78, 30, 82), fill=(255, 255, 0))
        self.draw.rectangle((68, 78, 78, 82), fill=(255, 255, 0))

        self.__createRotatedRectangle(24, 66, 30)
        self.__createRotatedRectangle(33, 57, 60)
        self.__createRotatedRectangle(56, 57, 120)
        self.__createRotatedRectangle(65, 66, 150)
        self.__createRotatedRectangle(65, 91, 210)
        self.__createRotatedRectangle(56, 100, 240)
        self.__createRotatedRectangle(33, 100, 300)
        self.__createRotatedRectangle(24, 91, 330)

        # Draw yellow battery charge
        self.draw.rectangle((100, 61, 220, 69), fill=(255, 255, 0))
        self.draw.rectangle((100, 77, 220, 85), fill=(255, 255, 0))
        self.draw.rectangle((100, 93, 220, 101), fill=(255, 255, 0))
        self.draw.rectangle((100, 109, 220, 117), fill=(255, 255, 0))
        self.draw.rectangle((100, 125, 220, 133), fill=(255, 255, 0))
        self.draw.rectangle((100, 141, 220, 149), fill=(255, 255, 0))
        self.draw.rectangle((100, 157, 220, 165), fill=(255, 255, 0))
        self.draw.rectangle((100, 173, 220, 181), fill=(255, 255, 0))
        self.draw.rectangle((100, 189, 220, 197), fill=(255, 255, 0))
        self.draw.rectangle((100, 205, 220, 230), fill=(255, 255, 0))

        self.update(0)

        print("Initialized screen")

    def clear(self):
        self.disp.clear()
        self.disp.display()

    def update(self, battery_pct: int):
        # Draw yellow battery charge
        self.draw.rectangle((100, 61, 220, 69), fill=(255, 255, 0))
        self.draw.rectangle((100, 77, 220, 85), fill=(255, 255, 0))
        self.draw.rectangle((100, 93, 220, 101), fill=(255, 255, 0))
        self.draw.rectangle((100, 109, 220, 117), fill=(255, 255, 0))
        self.draw.rectangle((100, 125, 220, 133), fill=(255, 255, 0))
        self.draw.rectangle((100, 141, 220, 149), fill=(255, 255, 0))
        self.draw.rectangle((100, 157, 220, 165), fill=(255, 255, 0))
        self.draw.rectangle((100, 173, 220, 181), fill=(255, 255, 0))
        self.draw.rectangle((100, 189, 220, 197), fill=(255, 255, 0))
        self.draw.rectangle((100, 205, 220, 230), fill=(255, 255, 0))

        # Show battery charges
        if battery_pct < 20:
            self.draw.rectangle((100, 59, 220, 197), fill=(0, 0, 0))

        else:
            height = ((battery_pct // 10 * 10) - 100) * (140 - 0) / (10 - 100) + 0
            self.draw.rectangle((100, 59, 220, 59 + height), fill=(0, 0, 0))

        self.disp.display(self.image)

    # Rotated rectangle creation (used to draw yellow sun)
    def __createRotatedRectangle(self, x: int, y: int, angle_deg: int):
        width = 10
        height = 4

        angle_rad = angle_deg * 3.14159 / 180.0
        center_x = x + width / 2
        center_y = y + height / 2

        # Calculation of rotated rectangle angles position
        x1 = (
            center_x
            + (x - center_x) * math.cos(angle_rad)
            - (y - center_y) * math.sin(angle_rad)
        )
        y1 = (
            center_y
            + (x - center_x) * math.sin(angle_rad)
            + (y - center_y) * math.cos(angle_rad)
        )
        x2 = (
            center_x
            + ((x + width) - center_x) * math.cos(angle_rad)
            - (y - center_y) * math.sin(angle_rad)
        )
        y2 = (
            center_y
            + ((x + width) - center_x) * math.sin(angle_rad)
            + (y - center_y) * math.cos(angle_rad)
        )
        x3 = (
            center_x
            + ((x + width) - center_x) * math.cos(angle_rad)
            - ((y + height) - center_y) * math.sin(angle_rad)
        )
        y3 = (
            center_y
            + ((x + width) - center_x) * math.sin(angle_rad)
            + ((y + height) - center_y) * math.cos(angle_rad)
        )
        x4 = (
            center_x
            + (x - center_x) * math.cos(angle_rad)
            - ((y + height) - center_y) * math.sin(angle_rad)
        )
        y4 = (
            center_y
            + (x - center_x) * math.sin(angle_rad)
            + ((y + height) - center_y) * math.cos(angle_rad)
        )

        self.draw.polygon([(x1, y1), (x2, y2), (x3, y3), (x4, y4)], fill=(255, 255, 0))


class ScreenNode(Node):
    def __init__(self):
        super().__init__("screen_node")

        self.screen = Screen(
            spi_port=SPI_PORT,
            spi_device=SPI_DEVICE,
            spi_speed=SPI_SPEED_HZ,
            spi_mode=SPI_MODE,
            rst_pin=RST_PIN,
            dc_pin=DC_PIN,
            blk_pin=BLK_PIN,
        )

        self.battery_charge_subscription = self.create_subscription(
            Int8, "battery_charge_topic", self.battery_charge_callback, 10
        )

    def battery_charge_callback(self, msg):
        battery_pct = msg.data

        self.screen.update(battery_pct)


def main(args=None):
    rclpy.init(args=args)
    screen_node = ScreenNode()
    rclpy.spin(screen_node)
    screen_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
