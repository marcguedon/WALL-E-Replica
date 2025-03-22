import cv2

cap = cv2.VideoCapture(0)  # Assure-toi que l'index est 0

if not cap.isOpened():
    print("Erreur : Impossible d'ouvrir la caméra.")
else:
    ret, frame = cap.read()
    if ret:
        cv2.imshow("Capture", frame)
        cv2.waitKey(0)
    else:
        print("Erreur : Capture impossible.")

cap.release()
cv2.destroyAllWindows()
