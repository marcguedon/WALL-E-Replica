import RPi.GPIO as GPIO
import time

GPIO.cleanup()

GPIO_LEFT_TRIG_PIN = 11
BCM_LEFT_TRIG_PIN = 17

GPIO_LEFT_ECHO_PIN = 12
BCM_LEFT_ECHO_PIN = 18

GPIO_RIGHT_TRIG_PIN = 15
BCM_RIGHT_TRIG_PIN = 22

GPIO_RIGHT_ECHO_PIN = 16
BCM_RIGHT_ECHO_PIN = 23

GPIO.setmode(GPIO.BCM)

GPIO.setup(BCM_LEFT_TRIG_PIN, GPIO.OUT)
GPIO.setup(BCM_LEFT_ECHO_PIN, GPIO.IN)

GPIO.setup(BCM_RIGHT_TRIG_PIN, GPIO.OUT)
GPIO.setup(BCM_RIGHT_ECHO_PIN, GPIO.IN)

GPIO.output(BCM_LEFT_TRIG_PIN, False)
GPIO.output(BCM_RIGHT_TRIG_PIN, False)

# Get distance from wall
def getDistance(sensor):
    startTime = 0
    stopTime = 0

    initialTime = time.time()

    if(sensor == 'left'):
        GPIO.output(BCM_LEFT_TRIG_PIN, True)
        time.sleep(0.00001)
        GPIO.output(BCM_LEFT_TRIG_PIN, False)

        while GPIO.input(BCM_LEFT_ECHO_PIN) == 0 and startTime - initialTime < 0.05:
            startTime = time.time()

        while GPIO.input(BCM_LEFT_ECHO_PIN) == 1 and stopTime - startTime < 0.05:
            stopTime = time.time()

    if(sensor == 'right'):
        GPIO.output(BCM_RIGHT_TRIG_PIN, True)
        time.sleep(0.00001)
        GPIO.output(BCM_RIGHT_TRIG_PIN, False)

        while GPIO.input(BCM_RIGHT_ECHO_PIN) == 0 and startTime - initialTime < 0.05:
            startTime = time.time()

        while GPIO.input(BCM_RIGHT_ECHO_PIN) == 1 and stopTime - startTime < 0.05:
            stopTime = time.time()

    distance = round((stopTime - startTime) * 34300 / 2, 2)

    return distance