import cv2
import mediapipe as mp

camera = None

#Fonction récupération vidéo sans overlay IA
def cameraCaptureWithoutAIOverlay():
    global camera

    if camera is None:
        camera = cv2.VideoCapture(0)
        print("Lancement capture caméra")
    
    while True:
        success, frame = camera.read()
        if not success:
            break
        else:
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

#Fonction récupération vidéo avec overlay IA
def cameraCaptureWithAIOverlay():
    global camera

    if camera is None:
        camera = cv2.VideoCapture(0)
        print("Lancement capture caméra")

    while True:
        success, frame = camera.read()
        if not success:
            break
        else:
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')