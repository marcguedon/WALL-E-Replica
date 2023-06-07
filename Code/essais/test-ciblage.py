import cv2
import mediapipe as mp

def horizontalLine(img, point):
  cv2.line(img, (point[0],0), (point[0],point[1]*2), (0,255,0), 1)

def verticalLine(img, point):
  cv2.line(img, (0,point[1]), (point[0]*2,point[1]), (0,255,0), 1)

def isCentered(point, target, tolerance):
  return ( (point[0] > target[0] - tolerance and point[0] < target[0] + tolerance) and (point[1] > target[1] - tolerance and point[1] < target[1] + tolerance) )

def goToTarget(point, target):
  if point[0] < target[0]:
    if point[1] < target[1]:
      print("AIM TOP LEFT !")
    else:
      print("AIM BOTTOM LEFT")
  else:
    if point[1] < target[1]:
      print("AIM TOP RIGHT !")
    else:
      print("AIM BOTTOM RIGHT")


mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_pose = mp.solutions.pose

# For webcam input:
cap = cv2.VideoCapture(0)

with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
  while cap.isOpened():
    success, image = cap.read()
    if not success:
      print("Missing Frame ...")
      continue

    # To improve performance, optionally mark the image as not writeable to
    # pass by reference.
    image.flags.writeable = False
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    results = pose.process(image)
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    height, width = image.shape[0], image.shape[1]
    center = (int(width/2), int(height/2))

    # Draw centerlines
    horizontalLine(image, center)
    verticalLine(image, center)

    if results.pose_landmarks:
      landmarkList = []
      for data_point in results.pose_landmarks.landmark:
        landmarkList.append((int(data_point.x * width), int(data_point.y * height)))

      #Draw line to target
      cv2.line(image, (center[0],center[1]), (landmarkList[0][0], landmarkList[0][1]), (0,0,255), 2)

      if (isCentered(landmarkList[0], center, 20)):
        print("FIRE !")
      else:
        goToTarget(landmarkList[0], center)

    # Flip the image horizontally for a selfie-view display.
    cv2.imshow('Turret View', image)
    if cv2.waitKey(5) & 0xFF == 27:
      break
cap.release()