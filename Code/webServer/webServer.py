from flask import Flask, render_template, jsonify, request, Response
import os
import yaml

import random #A RETIRER

app = Flask(__name__, static_folder='static')

#Route Récupération pourcentage batterie
@app.route('/getBatteryPercent')
def getBatteryPercent():
    batteryPercent = random.randint(0, 100) #FAIRE CODE RECUPERATION POURCENTAGE BATTERIE

    return str(batteryPercent)

#Route écriture Light On dans fichier config
@app.route('/setLightOn')
def setLightOn():
    filePath = os.path.join(os.path.dirname(__file__), 'config.yaml')
    with open(filePath, 'r') as file:
        data = yaml.safe_load(file)

    data['light_on'] = True

    #Ecriture dans le fichier YAML
    with open(filePath, 'w') as file:
        yaml.dump(data, file)

    return "OK"

#Route écriture Light Off dans fichier config
@app.route('/setLightOff')
def setLightOff():
    filePath = os.path.join(os.path.dirname(__file__), 'config.yaml')
    with open(filePath, 'r') as file:
        data = yaml.safe_load(file)

    data['light_on'] = False

    #Ecriture dans le fichier YAML
    with open(filePath, 'w') as file:
        yaml.dump(data, file)

    return "OK"

#Route Récupération pourcentage batterie
@app.route('/getLightState')
def getLightState():
    filePath = os.path.join(os.path.dirname(__file__), 'config.yaml')

    #Lecture du fichier YAML
    with open(filePath, 'r') as file:
        data = yaml.safe_load(file)

    return jsonify({'light_on': data['light_on']})

#Route affichage caméra sans overlay
@app.route('/showCameraWithoutAIOverlay')
def showCameraWithoutAIOverlay():
    return Response(cameraCaptureWithoutAIOverlay(), mimetype='multipart/x-mixed-replace; boundary=frame')

#Route affichage caméra avec overlay
@app.route('/showCameraWithAIOverlay')
def showCameraWithAIOverlay():
    return Response(cameraCaptureWithAIOverlay(), mimetype='multipart/x-mixed-replace; boundary=frame')

#Route écriture Mode Auto dans fichier config
@app.route('/setAutoMode')
def setAutoMode():
    filePath = os.path.join(os.path.dirname(__file__), 'config.yaml')
    with open(filePath, 'r') as file:
        data = yaml.safe_load(file)

    data['auto_mode'] = True

    #Ecriture dans le fichier YAML
    with open(filePath, 'w') as file:
        yaml.dump(data, file)

    return "OK"

#Route écriture Mode Manuel dans fichier config
@app.route('/setManualMode')
def setManualMode():
    filePath = os.path.join(os.path.dirname(__file__), 'config.yaml')
    with open(filePath, 'r') as file:
        data = yaml.safe_load(file)

    data['auto_mode'] = False
    data['light_on'] = False

    #Ecriture dans le fichier YAML
    with open(filePath, 'w') as file:
        yaml.dump(data, file)

    return "OK"

#Route lecture Mode dans fichier config
@app.route('/getOperatingMode')
def getOperatingMode():
    filePath = os.path.join(os.path.dirname(__file__), 'config.yaml')

    #Lecture du fichier YAML
    with open(filePath, 'r') as file:
        data = yaml.safe_load(file)

    return jsonify({'auto_mode': data['auto_mode']})

#Route d'accueil
@app.route('/')
def index():
    return render_template('index.html')

#Route Avancer
@app.route('/moveForward')
def moveForward():
    print("Avancer")
    #FAIRE CODE AVANCER

    return "OK"

#Route Gauche
@app.route('/moveLeft')
def moveLeft():
    print("Tourner à gauche")
    #FAIRE CODE TOURNER A GAUCHE

    return "OK"
    
#Route Droite
@app.route('/moveRight')
def moveRight():
    print("Tourner à droite")
    #FAIRE CODE TOURNER A DROITE

    return "OK"
 
#Route Reculer
@app.route('/moveBackward')
def moveBackward():
    print("Reculer")
    #FAIRE CODE RECULER

    return "OK"

#Route Lancer son
@app.route('/makeSound')
def makeSound():
    soundNum = request.args.get("sound")
    print("Son " + str(soundNum))
    #FAIRE CODE LANCER SON SUR HAUT-PARLEURS
    playSound(soundNum)

    return "OK"

if __name__ == '__main__':
    setManualMode()
    print("Le serveur Flask a démarré !")
    
    app.run(port=8888)
    #app.run(host='0.0.0.0', port=5000, debug=True)