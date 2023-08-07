import RPi.GPIO as GPIO

GPIO_LED_LIGHT_PIN = 29
BCM_LED_LIGHT_PIN = 5

GPIO_CAMERA_LIGHT_PIN = 7
BCM_CAMERA_LIGHT_PIN = 4

GPIO.setmode(GPIO.BCM)

GPIO.setup(BCM_LED_LIGHT_PIN, GPIO.OUT)
GPIO.setup(BCM_CAMERA_LIGHT_PIN, GPIO.OUT)

ledLight = GPIO.PWM(BCM_LED_LIGHT_PIN, 50)
cameraLight = GPIO.PWM(BCM_CAMERA_LIGHT_PIN, 50)

# Light initilization
def initLights():
    ledLight.start(0)
    cameraLight.start(0)
    ledLight.ChangeDutyCycle(0)
    cameraLight.ChangeDutyCycle(0)

    print('Initialized light')

# Change light value
def setLight(light, value):
    if(light == 'ledLight'):
        ledLight.ChangeDutyCycle(value)

    if(light == 'cameraLight'):
        cameraLight.ChangeDutyCycle(value)
    

# Switch on/off light
def switchLightOnOff(light):
    if(light == 'ledLight'):
        if GPIO.input(BCM_LED_LIGHT_PIN):
            ledLight.ChangeDutyCycle(0)
        else:
            ledLight.ChangeDutyCycle(100)
    
    if(light == 'cameraLight'):
        if GPIO.input(BCM_CAMERA_LIGHT_PIN):
            cameraLight.ChangeDutyCycle(0)
        else:
            cameraLight.ChangeDutyCycle(100)