from flask import Flask, render_template, jsonify, request, Response
import os
import yaml
import threading
import time

# Importation of homemade libraries
import sys
ROOT_PATH = os.path.dirname(os.path.abspath(__file__))
sys.path.append(ROOT_PATH + '/../libraries')
# sys.path.append('libraries')
print(sys.path)
import camera
import sounds
import light
import motors
import servomotors
import ultrasonic_sensor
import battery
import screen

app = Flask(__name__, static_folder='static')

motor = motors.MotorDriver()

@app.route('/getBatteryPercent')
def getBatteryPercent():
    batteryPercent = battery.getBatteryPercent()

    print('Pourcentage batterie : ' + str(batteryPercent))

    return str(batteryPercent)

@app.route('/getLightState')
def getLightState():
    filePath = os.path.join(os.path.dirname(__file__), 'config.yaml')

    # Reading of YAML file
    with open(filePath, 'r') as file:
        data = yaml.safe_load(file)

    print('Lumière allumée : ' + str(data['light_on']))

    return jsonify({'light_on': data['light_on']})

# Operating mode getter (auto or manual)
@app.route('/getOperatingMode')
def getOperatingMode():
    filePath = os.path.join(os.path.dirname(__file__), 'config.yaml')

    # Reading of YAML file
    with open(filePath, 'r') as file:
        data = yaml.safe_load(file)

    print('Mode auto : ' + str(data['auto_mode']))

    return {'auto_mode': data['auto_mode']}

@app.route('/switchLight')
def switchLight():
    filePath = os.path.join(os.path.dirname(__file__), 'config.yaml')
    with open(filePath, 'r') as file:
        data = yaml.safe_load(file)

    data['light_on'] = not data['light_on']
    light.switchLightOnOff('cameraLight', data['light_on'])

    # Writing in YAML file
    with open(filePath, 'w') as file:
        yaml.dump(data, file)

    print('Lumière allumée : ' + str(data['light_on']))

    return jsonify({'light_on': data['light_on']})

# Operating mode switch (auto or manual)
@app.route('/switchOperatingMode')
def switchOperatingMode():
    filePath = os.path.join(os.path.dirname(__file__), 'config.yaml')
    with open(filePath, 'r') as file:
        data = yaml.safe_load(file)

    data['auto_mode'] = not data['auto_mode']
    if data['auto_mode'] == True:
        data['light_on'] = False

    # Writing in YAML file
    with open(filePath, 'w') as file:
        yaml.dump(data, file)

    print('Mode auto : ' + str(data['auto_mode']))

    return jsonify({'auto_mode': data['auto_mode']})

@app.route('/showCameraWithoutAIOverlay')
def showCameraWithoutAIOverlay():
    return Response(camera.cameraCaptureWithoutAIOverlay(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/showCameraWithAIOverlay')
def showCameraWithAIOverlay():
    return Response(camera.cameraCaptureWithAIOverlay(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/moveHead')
def head():
    xDirection = str(request.args.get('xDirection'))
    yDirection = str(request.args.get('yDirection'))

    if xDirection == 'left':
        print('Turn head left')
        servomotors.moveHead('left')

    if xDirection == 'right':
        print('Turn head right')
        servomotors.moveHead('right')

    if xDirection == 'none':
        print('')      


    if yDirection == 'up':
        print('Head up')
        servomotors.moveHead('up')

    if yDirection == 'down':
        print('Head down')
        servomotors.moveHead('down')

    if yDirection == 'none':
        print('')

    return 'OK'

# Move robot
@app.route('/move')
def move():
    xDirection = float(request.args.get('xDirection'))
    yDirection = float(request.args.get('yDirection'))

    motor.move(xDirection, yDirection)

    return 'OK'

@app.route('/moveArm')
def moveArm():
    arm = str(request.args.get('arm'))
    angle = int(request.args.get('angle'))

    servomotors.moveArm(arm, angle)

    return 'OK'

# Make sound
@app.route('/makeSound')
def makeSound():
    soundNum = str(request.args.get('sound'))
    duration = float(request.args.get('duration'))

    soundPath = ROOT_PATH + '/static/sounds/son' + soundNum + '.mp3'

    soundThread = threading.Thread(target=playSoundWithLight, args=(soundPath, duration))
    soundThread.start()

    return 'OK'

# Get ultrasonic sensors value
@app.route('/getSensorsValue')
def getSensorsValue():
    leftSensorValue = ultrasonic_sensor.getDistance('left')
    rightSensorValue = ultrasonic_sensor.getDistance('right')

    values = [leftSensorValue, rightSensorValue]

    return jsonify(values)

# Home
@app.route('/')
def index():
    return render_template('index.html')

# Server initialization
def initWebServer():
    light.initLights()
    servomotors.initServomotors()
    screen.initScreen()
    
    batteryPercent = battery.getBatteryPercent()
    screen.updateScreen(batteryPercent)

    filePath = os.path.join(os.path.dirname(__file__), 'config.yaml')
    with open(filePath, 'r') as file:
        data = yaml.safe_load(file)

    data['auto_mode'] = False
    data['light_on'] = False

    # Writing in YAML file
    with open(filePath, 'w') as file:
        yaml.dump(data, file)

    print('Initialized server')

def updateScreen():
    while True:
        batteryPercent = battery.getBatteryPercent()
        screen.updateScreen(batteryPercent)

        print('Screen updated')

        time.sleep(60)

def playSoundWithLight(soundPath, duration):
    sounds.playSound(soundPath)

    light.switchLightOnOff('ledLight', True)
    time.sleep(duration)
    light.switchLightOnOff('ledLight', False)

if __name__ == '__main__':
    initWebServer()
    print('Le serveur Flask a démarré !')

    ScreenThread = threading.Thread(target=updateScreen)
    ScreenThread.start()
    
    app.run(host='0.0.0.0', port=5000, debug=True)