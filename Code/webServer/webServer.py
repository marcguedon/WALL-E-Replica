from flask import Flask, render_template, jsonify, request, Response
import os
import yaml

#Importation of homemade libraries
import sys
sys.path.append('libraries')
import camera
import sounds

import random #A RETIRER

app = Flask(__name__, static_folder='static')

#Route Récupération pourcentage batterie
@app.route('/getBatteryPercent')
def getBatteryPercent():
    batteryPercent = random.randint(0, 100) #FAIRE CODE RECUPERATION POURCENTAGE BATTERIE

    print('Pourcentage batterie : ' + str(batteryPercent))

    return str(batteryPercent)

#Route Récupération pourcentage batterie
@app.route('/getLightState')
def getLightState():
    filePath = os.path.join(os.path.dirname(__file__), 'config.yaml')

    #Lecture du fichier YAML
    with open(filePath, 'r') as file:
        data = yaml.safe_load(file)

    print('Lumière allumée : ' + str(data['light_on']))

    return jsonify({'light_on': data['light_on']})

#Route lecture Mode dans fichier config
@app.route('/getOperatingMode')
def getOperatingMode():
    filePath = os.path.join(os.path.dirname(__file__), 'config.yaml')

    #Lecture du fichier YAML
    with open(filePath, 'r') as file:
        data = yaml.safe_load(file)

    print('Mode auto : ' + str(data['auto_mode']))

    return {'auto_mode': data['auto_mode']}

#Route écriture Light On dans fichier config
@app.route('/switchLight')
def switchLight():
    filePath = os.path.join(os.path.dirname(__file__), 'config.yaml')
    with open(filePath, 'r') as file:
        data = yaml.safe_load(file)

    data['light_on'] = not data['light_on']

    #Ecriture dans le fichier YAML
    with open(filePath, 'w') as file:
        yaml.dump(data, file)

    print('Lumière allumée : ' + str(data['light_on']))

    return jsonify({'light_on': data['light_on']})

#Route écriture Mode Manuel dans fichier config
@app.route('/switchOperatingMode')
def switchOperatingMode():
    filePath = os.path.join(os.path.dirname(__file__), 'config.yaml')
    with open(filePath, 'r') as file:
        data = yaml.safe_load(file)

    data['auto_mode'] = not data['auto_mode']
    if data['auto_mode'] == True:
        data['light_on'] = False

    #Ecriture dans le fichier YAML
    with open(filePath, 'w') as file:
        yaml.dump(data, file)

    print('Mode auto : ' + str(data['auto_mode']))

    return jsonify({'auto_mode': data['auto_mode']})

#Route affichage caméra sans overlay
@app.route('/showCameraWithoutAIOverlay')
def showCameraWithoutAIOverlay():
    return Response(camera.cameraCaptureWithoutAIOverlay(), mimetype='multipart/x-mixed-replace; boundary=frame')

#Route affichage caméra avec overlay
@app.route('/showCameraWithAIOverlay')
def showCameraWithAIOverlay():
    return Response(camera.cameraCaptureWithAIOverlay(), mimetype='multipart/x-mixed-replace; boundary=frame')

#Route Avancer
@app.route('/move')
def move():
    direction = request.args.get('direction')

    match direction:
        case 'forward':
            print('Avancer')
        case 'left':
            print('Tourner à gauche')
        case 'right':
            print('Tourner à droite')
        case 'backward':
            print('Reculer')

    return 'OK'

#Route Lancer son
@app.route('/makeSound')
def makeSound():
    soundNum = request.args.get('sound')
    sounds.playSound(soundNum)

    print('Son ' + str(soundNum))

    return 'OK'

#Route d'accueil
@app.route('/')
def index():
    return render_template('index.html')

def initWebServer():
    filePath = os.path.join(os.path.dirname(__file__), 'config.yaml')
    with open(filePath, 'r') as file:
        data = yaml.safe_load(file)

    data['auto_mode'] = False
    data['light_on'] = False

    #Ecriture dans le fichier YAML
    with open(filePath, 'w') as file:
        yaml.dump(data, file)

    print('Initialisation serveur')

if __name__ == '__main__':
    initWebServer()
    print('Le serveur Flask a démarré !')
    
    app.run(port=8888)
    #app.run(host='0.0.0.0', port=5000, debug=True)