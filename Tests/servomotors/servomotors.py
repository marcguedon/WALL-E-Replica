from PCA9685 import PCA9685

pwm = PCA9685(0x41, debug=True)
pwm.setPWMFreq(50)

headRotation = 0
neckTop = 1
neckBottom = 2
rightEye = 3
leftEye = 4
leftArm = 5
rightArm = 6

currentHeadRotationAngle = 90
currentNeckTopAngle = 10
currentNeckBottomAngle = 30

# Convert angle to pulse (more easier to use)
def setServoAngle(channel, angle):
    pulse = (angle - 0) * (2000 - 1000) / (180 - 0) + 1000
    pwm.setServoPulse(channel, pulse)

# Servomotors position initialization
def initServomotors():
    setServoAngle(headRotation, 90)
    setServoAngle(leftArm, 140)
    setServoAngle(rightArm, 40)
    setServoAngle(neckTop, 0)
    setServoAngle(neckBottom, 30)
    setServoAngle(rightEye, 110)
    setServoAngle(leftEye, 70)

    print('Initialized servomotors')

# Arms movement
def moveArm(arm, angle):
    if arm == 'left':
        # max angle (up) : 40 / min angle (down) : 140
        setServoAngle(leftArm, angle)

    if arm == 'right':
        # max angle (up) : 140 / min angle (down) : 40
        setServoAngle(rightArm, angle)

# Head movement
def moveHead(direction):
    global currentHeadRotationAngle
    global currentNeckTopAngle
    global currentNeckBottomAngle

    if direction == 'left':
        if currentHeadRotationAngle + 1 <= 150:
            currentHeadRotationAngle = currentHeadRotationAngle + 1
            setServoAngle(headRotation, currentHeadRotationAngle)

    if direction == 'right':
        if currentHeadRotationAngle - 1 >= 30:
            currentHeadRotationAngle = currentHeadRotationAngle - 1
            setServoAngle(headRotation, currentHeadRotationAngle)

    if direction == 'up':
        if currentNeckTopAngle - 1 >= 0:
            currentNeckTopAngle = currentNeckTopAngle - 1
            setServoAngle(neckTop, currentNeckTopAngle)

        if currentNeckBottomAngle + 1 <= 180:
            currentNeckBottomAngle = currentNeckBottomAngle + 1
            setServoAngle(neckBottom, currentNeckBottomAngle)

    if direction == 'down':
        if currentNeckTopAngle + 1 <= 180:
            currentNeckTopAngle = currentNeckTopAngle + 1
            setServoAngle(neckTop, currentNeckTopAngle)

        if currentNeckBottomAngle - 1 >= 30:
            currentNeckBottomAngle = currentNeckBottomAngle - 1
            setServoAngle(neckBottom, currentNeckBottomAngle)

# Eyes movement
def moveEye(eye, angle):
    if eye == 'left':
        setServoAngle(leftEye, angle)

    if eye == 'right':
        # MAX (levé) : 140 / MIN (baissé) : 80
        setServoAngle(rightEye, angle)

# ~~~~~~~~~~~~~~~ #
#      TESTS      #
# ~~~~~~~~~~~~~~~ #

import time

initServomotors()

print('Right arm')
for angle in range(40, 141, 10):
    print(angle)
    setServoAngle(rightArm, angle)
    time.sleep(1)

print('Left arm')
for angle in range(140, 39, -10):
    print(angle)
    setServoAngle(leftArm, angle)
    time.sleep(1)

print('Head rotation')
for angle in range(30, 151, 10):
    print(angle)
    setServoAngle(headRotation, angle)
    time.sleep(1)

print('Right eye')
for angle in range(80, 141, 10):
    print(angle)
    setServoAngle(rightEye, angle)
    time.sleep(1)

print('Left eye')
for angle in range(100, 39, -10):
    print(angle)
    setServoAngle(leftEye, angle)
    time.sleep(1)

print('Top neck')
for angle in range(0, 181, 10):
    print(angle)
    setServoAngle(neckTop, angle)
    time.sleep(1)

print('Bottom neck')
for angle in range(30, 181, 10):
    print(angle)
    setServoAngle(neckBottom, angle)
    time.sleep(1)

initServomotors()