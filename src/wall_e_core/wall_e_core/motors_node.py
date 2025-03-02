import rclpy
from rclpy.node import Node
from PCA9685 import PCA9685
from numpy import sqrt, min, abs
from wall_e_msg_srv.srv import Move


class Motor:
    def __init__(self, driver, pwm: int, in1: int, in2: int):
        self.driver = driver
        self.pwm = pwm
        self.in1 = in1
        self.in2 = in2

    def run(self, speed: float):
        if speed == 0:
            self.driver.setDutycycle(self.pwm, 0)
            return

        abs_speed = min([abs(speed), 1])
        speed_pct = abs_speed * 100
        forward = speed > 0

        self.driver.setDutycycle(self.pwm, speed_pct)
        self.driver.setLevel(self.in1, not forward)
        self.driver.setLevel(self.in2, forward)


DEFAULT_DRIVER_ADR = 0x40
DEFAULT_A_PWM = 0
DEFAULT_A_IN1 = 1
DEFAULT_A_IN2 = 2
DEFAULT_B_PWM = 5
DEFAULT_B_IN1 = 3
DEFAULT_B_IN2 = 4


class MotorsNode(Node):
    def __init__(self):
        super().__init__("motors_node")

        self.driver = PCA9685(DEFAULT_DRIVER_ADR, debug=False)
        self.driver.setPWMFreq(50)

        self.left_motor = Motor(
            self.driver, DEFAULT_A_PWM, DEFAULT_A_IN1, DEFAULT_A_IN2
        )
        self.right_motor = Motor(
            self.driver, DEFAULT_B_PWM, DEFAULT_B_IN1, DEFAULT_B_IN2
        )

        self.move_srv = self.create_service(Move, "move", self.move_callback)

    def stop_motors(self):
        self.left_motor.run(0)
        self.right_motor.run(0)

    def run_motors(self, right_motor_speed: float, left_motor_speed: float):
        self.right_motor.run(right_motor_speed)
        self.left_motor.run(left_motor_speed)

    def move(self, x_direction: float, y_direction: float):
        if x_direction == 0 and y_direction == 0:
            self.stop_motors()
            return

        magnitude = sqrt(x_direction * x_direction + y_direction * y_direction)

        if abs(y_direction) < abs(x_direction) / 2:
            if x_direction < 0:
                self.run_motors(-magnitude, magnitude)
            else:
                self.run_motors(magnitude, -magnitude)

        else:
            signed_magnitude = -magnitude if y_direction < 0 else magnitude
            if x_direction < 0:
                self.run_motors(signed_magnitude, y_direction)
            else:
                self.run_motors(y_direction, signed_magnitude)

    def move_callback(self, request, response):
        self.move(request.x_direction, request.y_direction)

        response.success = True

        return response


def main(args=None):
    rclpy.init(args=args)
    node = MotorsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
