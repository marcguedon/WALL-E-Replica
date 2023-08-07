from flask import Flask, render_template, jsonify, request, Response
import os
import yaml

# Importation of homemade libraries
import sys
sys.path.append('libraries')
import camera
import sounds
import light
import motors
import servomotors
# import screen
import ultrasonic_sensor
# import battery
# import controller

app = Flask(__name__, static_folder='static')

motor = motors.MotorDriver()

# Battery percent getter
@app.route('/getBatteryPercent')
def getBatteryPercent():
    # batteryPercent = battery.getBatteryPercent()
    batteryPercent = 50

    print('Pourcentage batterie : ' + str(batteryPercent))

    return str(batteryPercent)

# Light state getter
@app.route('/getLightState')
def getLightState():
    filePath = os.path.join(os.path.dirname(__file__), 'config.yaml')

    # Reading of YAML file
    with open(filePath, 'r') as file:
        data = yaml.safe_load(file)

    print('Lumière allumée : ' + str(data['light_on']))

    return jsonify({'light_on': data['light_on']})

# Operating mode getter
@app.route('/getOperatingMode')
def getOperatingMode():
    filePath = os.path.join(os.path.dirname(__file__), 'config.yaml')

    # Reading of YAML file
    with open(filePath, 'r') as file:
        data = yaml.safe_load(file)

    print('Mode auto : ' + str(data['auto_mode']))

    return {'auto_mode': data['auto_mode']}

# Light state switch
@app.route('/switchLight')
def switchLight():
    filePath = os.path.join(os.path.dirname(__file__), 'config.yaml')
    with open(filePath, 'r') as file:
        data = yaml.safe_load(file)

    data['light_on'] = not data['light_on']
    light.switchLightOnOff('cameraLight')

    # Writing in YAML file
    with open(filePath, 'w') as file:
        yaml.dump(data, file)

    print('Lumière allumée : ' + str(data['light_on']))

    return jsonify({'light_on': data['light_on']})

# Operating mode switch
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

# Show camera without AI overlay
@app.route('/showCameraWithoutAIOverlay')
def showCameraWithoutAIOverlay():
    return Response(camera.cameraCaptureWithoutAIOverlay(), mimetype='multipart/x-mixed-replace; boundary=frame')

# Show camera with AI overlay
@app.route('/showCameraWithAIOverlay')
def showCameraWithAIOverlay():
    return Response(camera.cameraCaptureWithAIOverlay(), mimetype='multipart/x-mixed-replace; boundary=frame')

# Move head
@app.route('/head')
def head():
    xDirection = request.args.get('xDirection')
    yDirection = request.args.get('yDirection')

    if xDirection == 'left':
        print('Tourner tête à gauche')
        servomotors.moveHead('left')

    if xDirection == 'right':
        print('Tourner tête à droite')
        servomotors.moveHead('right')

    if xDirection == 'none':
        print('')
        
        
    if yDirection == 'up':
        print('Monter tête')
        servomotors.moveHead('up')

    if yDirection == 'down':
        print('Descendre tête')
        servomotors.moveHead('down')

    if yDirection == 'none':
        print('')

    return 'OK'

# Move
@app.route('/move')
def move():
    xDirection = float(request.args.get('xDirection'))
    yDirection = float(request.args.get('yDirection'))

    motor.move(xDirection, yDirection)

    return 'OK'

# Move arm
@app.route('/moveArm')
def moveArm():
    arm = request.args.get('arm')
    angle = request.args.get('angle')

    servomotors.moveArm(arm, angle)

    return 'OK'

# Make sound
@app.route('/makeSound')
def makeSound():
    soundNum = request.args.get('sound')

    print('Son ' + str(soundNum))
    sounds.playSound(soundNum)

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
    # screen.initScreen()
    
    # batteryPercent = battery.getBatteryPercent()
    # screen.updateScreen(batteryPercent)

    filePath = os.path.join(os.path.dirname(__file__), 'config.yaml')
    with open(filePath, 'r') as file:
        data = yaml.safe_load(file)

    data['auto_mode'] = False
    data['light_on'] = False

    # Writing in YAML file
    with open(filePath, 'w') as file:
        yaml.dump(data, file)

    print('Initialized server')


if __name__ == '__main__':
    initWebServer()
    print('Le serveur Flask a démarré !')
    
    # app.run(port=8888)
    app.run(host='0.0.0.0', port=5000, debug=True)