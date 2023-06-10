from PCA9685 import PCA9685
import time

Dir = [
    'forward',
    'backward',
]

pwm = PCA9685(0x40, debug=False)
pwm.setPWMFreq(50)

class MotorDriver():
    #Initialisation
    def __init__(self):
        self.PWMA = 0
        self.AIN1 = 1
        self.AIN2 = 2
        self.PWMB = 5
        self.BIN1 = 3
        self.BIN2 = 4

    #Fonction mouvements
    def MotorRun(self, motor, index, speed):
        if speed > 100:
            return
        if(motor == 0):
            pwm.setDutycycle(self.PWMA, speed)
            if(index == Dir[0]):
                print ("1")
                pwm.setLevel(self.AIN1, 0)
                pwm.setLevel(self.AIN2, 1)
            elif(index == Dir[1]):
                print ("2")
                pwm.setLevel(self.AIN1, 1)
                pwm.setLevel(self.AIN2, 1)
            elif(index == Dir[2]):
                print ("3")
                pwm.setLevel(self.AIN1, 0)
                pwm.setLevel(self.AIN2, 0)
            else:
                print ("4")
                pwm.setLevel(self.AIN1, 1)
                pwm.setLevel(self.AIN2, 0)
        else:
            pwm.setDutycycle(self.PWMB, speed)
            if(index == Dir[0]):
                print ("5")
                pwm.setLevel(self.BIN1, 0)
                pwm.setLevel(self.BIN2, 1)
            elif(index == Dir[1]):
                print ("6")
                pwm.setLevel(self.BIN1, 1)
                pwm.setLevel(self.BIN2, 1)
            elif(index == Dir[2]):
                print ("7")
                pwm.setLevel(self.BIN1, 0)
                pwm.setLevel(self.BIN2, 0)
            else:
                print ("8")
                pwm.setLevel(self.BIN1, 1)
                pwm.setLevel(self.BIN2, 0)

    #Fonction stop
    def MotorStop(self, motor):
        if (motor == 0):
            pwm.setDutycycle(self.PWMA, 0)
        else:
            pwm.setDutycycle(self.PWMB, 0)

    def forward(self, speed):
        self.MotorRun(0, 'forward', speed)
        self.MotorRun(1, 'forward', speed)

    def backward(self, speed):
        self.MotorRun(0, 'backward', speed)
        self.MotorRun(1, 'backward', speed)

    def left(self, speed):
        self.MotorRun(0, 'backward', speed)
        self.MotorRun(1, 'forward', speed)

    def right(self, speed):
        self.MotorRun(0, 'forward', speed)
        self.MotorRun(1, 'backward', speed)

    def stop(self):
        self.MotorStop(0)
        self.MotorStop(1)

# Code de test des moteurs
print("this is a motor driver test code")
Motor = MotorDriver()

while(1):
    print("forward 2s")
    Motor.forward(100)
    time.sleep(2)

    print("backward 2s")
    Motor.backward(100)
    time.sleep(2)

    print("left 2s")
    Motor.left(100)
    time.sleep(2)

    print("right 2s")
    Motor.right(100)
    time.sleep(2)

    print("stop 2s")
    Motor.stop()
    time.sleep(2)