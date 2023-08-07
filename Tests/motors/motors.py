from PCA9685 import PCA9685
from math import sqrt

pwm = PCA9685(0x40, debug=False)
pwm.setPWMFreq(50)

Dir = [
    'forward',
    'backward',
]

class MotorDriver():
    # Initialization
    def __init__(self):
        self.PWMA = 0
        self.AIN1 = 1
        self.AIN2 = 2
        self.PWMB = 5
        self.BIN1 = 3
        self.BIN2 = 4

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
    # The 2 next functions are used for each motor #
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #

    # Movements function (1 motor)
    def motorRun(self, motor, index, speed):
        if speed > 100:
            return
        if(motor == 0):
            pwm.setDutycycle(self.PWMA, speed)
            if(index == Dir[0]):
                pwm.setLevel(self.AIN1, 0)
                pwm.setLevel(self.AIN2, 1)
            else:
                pwm.setLevel(self.AIN1, 1)
                pwm.setLevel(self.AIN2, 0)
        else:
            pwm.setDutycycle(self.PWMB, speed)
            if(index == Dir[0]):
                pwm.setLevel(self.BIN1, 0)
                pwm.setLevel(self.BIN2, 1)
            else:
                pwm.setLevel(self.BIN1, 1)
                pwm.setLevel(self.BIN2, 0)

    # Stop function (1 motor)
    def motorStop(self, motor):
        if (motor == 0):
            pwm.setDutycycle(self.PWMA, 0)
        else:
            pwm.setDutycycle(self.PWMB, 0)

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
    # The next functions are used for the complete robot #
    # It uses the first two functions                    #
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #

    def stopMotors(self):
        self.motorStop(0)
        self.motorStop(1)

    def runMotor(self, motor, value):
        maxSpeed = 100
        if value == 0:
            self.motorStop(motor)
        else:
            direction = 'backward' if value < 0 else 'forward'
            absValue = abs(value)
            if absValue > 1:
                absValue = 1
            self.motorRun(motor, direction, absValue * maxSpeed)

    def runMotors(self, rightMotor, leftMotor):
        self.runMotor(0, rightMotor)
        self.runMotor(1, leftMotor)
        return
    
    def runMotorDirection(self, mainMotor, secondaryMotor, left):
        if left:
            self.runMotors(mainMotor, secondaryMotor)
        else:
            self.runMotors(secondaryMotor, mainMotor)

    def move(self, xDirection: float, yDirection: float):
        if xDirection == 0 and yDirection == 0:
            self.stopMotors()
            return
        
        magnitude = sqrt(xDirection * xDirection + yDirection * yDirection)

        if abs(yDirection) < abs(xDirection) / 2:
            self.runMotorDirection(magnitude, -magnitude, xDirection < 0)
        else:
            signedMagnitude = magnitude
            if yDirection < 0:
                signedMagnitude = -magnitude
            self.runMotorDirection(signedMagnitude, yDirection, xDirection < 0)

# ~~~~~~~~~~~~~~~ #
#      TESTS      #
# ~~~~~~~~~~~~~~~ #

import time

motor = MotorDriver()

# TODO
