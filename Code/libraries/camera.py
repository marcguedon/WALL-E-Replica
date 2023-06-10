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
        ret, frame = camera.read()
        if not ret:
            break
        
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

#Fonction récupération vidéo avec overlay IA
def cameraCaptureWithAIOverlay():
    global camera

    if camera is None:
        camera = cv2.VideoCapture(0)
        print("Lancement capture caméra")

    mp_hands = mp.solutions.hands.Hands()

    while True:
        ret, frame = camera.read()
        if not ret:
            break

        # Convertissez l'image en niveaux de gris pour la détection de la main
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Détectez les mains dans l'image
        results = mp_hands.process(image)

        # Dessinez des annotations sur l'image
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Dessinez des annotations sur l'image en fonction des coordonnées des landmarks
                for landmark in hand_landmarks.landmark:
                    # Access landmark coordinates and draw annotations
                    x = min(int(landmark.x * frame.shape[1]), frame.shape[1] - 1)
                    y = min(int(landmark.y * frame.shape[0]), frame.shape[0] - 1)
                    cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)

                connections = mp.solutions.holistic.HAND_CONNECTIONS
                for connection in connections:
                    x0 = min(int(hand_landmarks.landmark[connection[0]].x * frame.shape[1]), frame.shape[1] - 1)
                    y0 = min(int(hand_landmarks.landmark[connection[0]].y * frame.shape[0]), frame.shape[0] - 1)
                    x1 = min(int(hand_landmarks.landmark[connection[1]].x * frame.shape[1]), frame.shape[1] - 1)
                    y1 = min(int(hand_landmarks.landmark[connection[1]].y * frame.shape[0]), frame.shape[0] - 1)
                    cv2.line(frame, (x0, y0), (x1, y1), (0, 255, 0), 2)

        # Convertissez l'image en format JPG pour l'affichage dans le navigateur
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()

        yield (b'--frame\r\n'b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')