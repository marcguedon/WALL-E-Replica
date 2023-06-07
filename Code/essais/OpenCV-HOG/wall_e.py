import cv2
import numpy as np

#Initialisation de HOG (Histograms of Oriented Gradients)
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

cv2.startWindowThread()

#Acces à la webcam
cam = cv2.VideoCapture(0)

title = "WALL-E"

cv2.namedWindow(title, cv2.WINDOW_AUTOSIZE)
#cv2.namedWindow(webcam, cv2.WND_PROP_FULLSCREEN)

while True:
    #On récupère une image (pas une vidéo)
    ret, image = cam.read()

    #On réduit la taille et on passe en niveaux de gris pour augmenter la vitesse de détection
    reducedImage = cv2.resize(image, (640, 480))
    transformedImage = cv2.cvtColor(reducedImage, cv2.COLOR_RGB2GRAY)

    #Détection des personnes + acquisition des box
    boxes, weights = hog.detectMultiScale(transformedImage, winStride=(8,8) )
    boxes = np.array([[x, y, x + w, y + h] for (x, y, w, h) in boxes])

    for (xA, yA, xB, yB) in boxes:
        #Affichages des box sur image d'origine
        cv2.rectangle(image, (xA, yA), (xB, yB), (0, 255, 0), 2)
    
    #Affichage de l'image résultante
    cv2.imshow(title, image)

    #Arrêt quand appuie sur 'q'
    if cv2.waitKey(25) & 0xFF == ord('q'):
        break

#On arrête la webcam et on ferme la fenêtre
cam.release()
cv2.destroyAllWindows()
cv2.waitKey(1)