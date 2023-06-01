from flask import Flask, render_template, jsonify, request
import os
import yaml

app = Flask(__name__, static_folder='static')


#Route écriture Mode Auto dans fichier config
@app.route('/setAutoMode')
def setAutoMode():
    filePath = os.path.join(os.path.dirname(__file__), 'config.yaml')
    data = {
        'auto_mode': True
    }

    #Ecriture dans le fichier YAML
    with open(filePath, 'w') as file:
        yaml.dump(data, file)

    return "OK"

#Route écriture Mode Manuel dans fichier config
@app.route('/setManualMode')
def setManualMode():
    filePath = os.path.join(os.path.dirname(__file__), 'config.yaml')
    data = {
        'auto_mode': False
    }

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

    return "OK"

#Route Gauche
@app.route('/moveLeft')
def moveLeft():
    print("Tourner à gauche")

    return "OK"
    
#Route Droite
@app.route('/moveRight')
def moveRight():
    print("Tourner à droite")

    return "OK"
 
#Route Reculer
@app.route('/moveBackward')
def moveBackward():
    print("Reculer")

    return "OK"

#Route Lancer son
@app.route('/makeSound')
def makeSound():
    soundNum = request.args.get("sound")
    print("Son " + soundNum)

    return "OK"
    
if __name__ == '__main__':
    setManualMode()
    print("Le serveur Flask a démarré !")
    app.run(port=8888)