import RPi.GPIO as GPIO

GPIO.cleanup()

GPIO_LED_LIGHT_PIN = 29
BCM_LED_LIGHT_PIN = 5

GPIO_CAMERA_LIGHT_PIN = 7
BCM_CAMERA_LIGHT_PIN = 4

global ledLight
global cameraLight

# Light initilization
def initLights():
    global ledLight
    global cameraLight

    GPIO.setmode(GPIO.BCM)

    GPIO.setup(BCM_LED_LIGHT_PIN, GPIO.OUT)
    GPIO.setup(BCM_CAMERA_LIGHT_PIN, GPIO.OUT)

    ledLight = GPIO.PWM(BCM_LED_LIGHT_PIN, 50)
    cameraLight = GPIO.PWM(BCM_CAMERA_LIGHT_PIN, 50)

    ledLight.start(0)
    cameraLight.start(0)
    ledLight.ChangeDutyCycle(0)
    cameraLight.ChangeDutyCycle(100)

    print('Initialized light')

# Change light value
def setLight(light, value):
    if(light == 'ledLight'):
        ledLight.ChangeDutyCycle(value)

    if(light == 'cameraLight'):
        cameraLight.ChangeDutyCycle(100 - value)
    

# Switch on/off light
def switchLightOnOff(light, isOn):
    if(light == 'ledLight'):
        if isOn:
            ledLight.ChangeDutyCycle(0)
        else:
            ledLight.ChangeDutyCycle(100)
    
    if(light == 'cameraLight'):
        if isOn:
            cameraLight.ChangeDutyCycle(0)
        else:
            cameraLight.ChangeDutyCycle(100)

# ~~~~~~~~~~~~~~~ #
#      TESTS      #
# ~~~~~~~~~~~~~~~ #

import time

initLights()

print('SWITCH LED LIGHT ON')
switchLightOnOff('ledLight', False)
time.sleep(2)

print('SWITCH LED LIGHT OFF')
switchLightOnOff('ledLight', True)
time.sleep(2)

print('SWITCH CAMERA LIGHT ON')
switchLightOnOff('cameraLight', False)
time.sleep(2)

print('SWITCH CAMERA LIGHT OFF')
switchLightOnOff('cameraLight', True)
time.sleep(2)

print('LED LIGHT PWM')
for value in range (0, 100, 1):
    setLight('ledLight', value)
    print(value)
    time.sleep(0.03)
setLight('ledLight', 0)

print('LED LIGHT PWM')
for value in range (0, 100, 1):
    setLight('cameraLight', value)
    print(value)
    time.sleep(0.03)
setLight('cameraLight', 0)

initLights()