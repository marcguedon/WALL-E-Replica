import Adafruit_GPIO.SPI as SPI
import ST7789 as TFT
import math
from PIL import Image, ImageDraw, ImageFont

# Raspberry Pi configuration
RST_PIN = 25
DC_PIN  = 24
BLK_PIN = 27
SPI_PORT = 0
SPI_DEVICE = 0
SPI_MODE = 0b11
SPI_SPEED_HZ = 40000000
SCREEN_WIDTH = 240
SCREEN_HEIGHT = 240

# Display initialization
disp = TFT.ST7789(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE, max_speed_hz=SPI_SPEED_HZ), mode=SPI_MODE, rst=RST_PIN, dc=DC_PIN, led=BLK_PIN) 
image = Image.new('RGB', (SCREEN_WIDTH, SCREEN_HEIGHT), color=(0, 0, 0))
draw = ImageDraw.Draw(image)

disp.begin()

# Screen initialization
def initScreen():
    # Clear screen
    clearScreen()

    # Add text
    text = "SOLAR CHARGE LEVEL"
    font = ImageFont.truetype("Tests/screen/arial.ttf", 18)
    text_width, text_height = draw.textsize(text, font=font)
    text_x = (SCREEN_WIDTH - text_width) // 2
    text_y = 15
    draw.text((text_x, text_y), text, font=font, fill=(255, 255, 0))

    # Draw yellow sun
    draw.ellipse((34, 65, 64, 95), fill=(255, 255, 0))
    draw.ellipse((40, 71, 58, 89), fill=(0, 0, 0))
    draw.rectangle((47, 51, 51, 61), fill=(255, 255, 0))
    draw.rectangle((47, 99, 51, 109), fill=(255, 255, 0))
    draw.rectangle((20, 78, 30, 82), fill=(255, 255, 0))
    draw.rectangle((68, 78, 78, 82), fill=(255, 255, 0))

    createRotatedRectangle(24, 66, 30)
    createRotatedRectangle(33, 57, 60)
    createRotatedRectangle(56, 57, 120)
    createRotatedRectangle(65, 66, 150)
    createRotatedRectangle(65, 91, 210)
    createRotatedRectangle(56, 100, 240)
    createRotatedRectangle(33, 100, 300)
    createRotatedRectangle(24, 91, 330)

    # Draw yellow battery/brightness charge
    draw.rectangle((100, 61, 220, 69), fill=(255, 255, 0))
    draw.rectangle((100, 77, 220, 85), fill=(255, 255, 0))
    draw.rectangle((100, 93, 220, 101), fill=(255, 255, 0))
    draw.rectangle((100, 109, 220, 117), fill=(255, 255, 0))
    draw.rectangle((100, 125, 220, 133), fill=(255, 255, 0))
    draw.rectangle((100, 141, 220, 149), fill=(255, 255, 0))
    draw.rectangle((100, 157, 220, 165), fill=(255, 255, 0))
    draw.rectangle((100, 173, 220, 181), fill=(255, 255, 0))
    draw.rectangle((100, 189, 220, 197), fill=(255, 255, 0))
    draw.rectangle((100, 205, 220, 230), fill=(255, 255, 0))

    updateScreen(100)

    print('Initialized screen')

# Clear screen
def clearScreen():
    disp.clear()
    disp.display()

# Battery/brightness update
def updateScreen(percent):
    # Draw yellow battery/brightness charge
    draw.rectangle((100, 61, 220, 69), fill=(255, 255, 0))
    draw.rectangle((100, 77, 220, 85), fill=(255, 255, 0))
    draw.rectangle((100, 93, 220, 101), fill=(255, 255, 0))
    draw.rectangle((100, 109, 220, 117), fill=(255, 255, 0))
    draw.rectangle((100, 125, 220, 133), fill=(255, 255, 0))
    draw.rectangle((100, 141, 220, 149), fill=(255, 255, 0))
    draw.rectangle((100, 157, 220, 165), fill=(255, 255, 0))
    draw.rectangle((100, 173, 220, 181), fill=(255, 255, 0))
    draw.rectangle((100, 189, 220, 197), fill=(255, 255, 0))
    draw.rectangle((100, 205, 220, 230), fill=(255, 255, 0))

    # Show battery/brightness charges
    if percent < 20:
        draw.rectangle((100, 59, 220, 197), fill=(0, 0, 0))

    else:
        height = ((percent // 10 * 10) - 100) * (140 - 0) / (10 - 100) + 0
        draw.rectangle((100, 59, 220, 59 + height), fill=(0, 0, 0))

    disp.display(image)

# Rotated rectangle creation (used to draw yellow sun)
def createRotatedRectangle(x, y, angle):
    width = 10
    height = 4

    angle = angle * 3.14159 / 180.0
    center_x = x + width / 2
    center_y = y + height / 2

    # Calculation of rotated rectangle angles position
    x1 = center_x + (x - center_x) * math.cos(angle) - (y - center_y) * math.sin(angle)
    y1 = center_y + (x - center_x) * math.sin(angle) + (y - center_y) * math.cos(angle)
    x2 = center_x + ((x + width) - center_x) * math.cos(angle) - (y - center_y) * math.sin(angle)
    y2 = center_y + ((x + width) - center_x) * math.sin(angle) + (y - center_y) * math.cos(angle)
    x3 = center_x + ((x + width) - center_x) * math.cos(angle) - ((y + height) - center_y) * math.sin(angle)
    y3 = center_y + ((x + width) - center_x) * math.sin(angle) + ((y + height) - center_y) * math.cos(angle)
    x4 = center_x + (x - center_x) * math.cos(angle) - ((y + height) - center_y) * math.sin(angle)
    y4 = center_y + (x - center_x) * math.sin(angle) + ((y + height) - center_y) * math.cos(angle)

    draw.polygon([(x1, y1), (x2, y2), (x3, y3), (x4, y4)], fill=(255, 255, 0))

# ~~~~~~~~~~~~~~~ #
#      TESTS      #
# ~~~~~~~~~~~~~~~ #

import time

clearScreen()

initScreen()
updateScreen(64)

time.sleep(5)
clearScreen()