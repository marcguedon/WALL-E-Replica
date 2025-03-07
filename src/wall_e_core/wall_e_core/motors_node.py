import board
import rclpy
from numpy import sqrt
from rclpy.node import Node
from adafruit_pca9685 import PCA9685
from wall_e_msg_srv.srv import Move


class Motor:
    def __init__(self, pca, pwm: int, in1: int, in2: int, logger=None):
        self.pca = pca
        self.pwm = pwm
        self.in1 = in1
        self.in2 = in2
        self.logger = logger

    def run(self, speed: float):
        if speed == 0:
            self.pca.channels[self.pwm].duty_cycle = 0x0000
            return

        abs_speed = min(abs(speed), 1)
        speed_pct = int(abs_speed * 4095)  # PWM range for PCA9685 is 0 to 4095
        forward = speed > 0 

        if self.logger:
            self.logger.info(f"Motor running at speed: {speed_pct}%")

        self.pca.channels[self.pwm].duty_cycle = speed_pct

        # Control direction using IN1 and IN2
        if forward:
            self.pca.channels[self.in1].duty_cycle = 0x0000
            self.pca.channels[self.in2].duty_cycle = 0xFFFF
        else:
            self.pca.channels[self.in1].duty_cycle = 0xFFFF
            self.pca.channels[self.in2].duty_cycle = 0x0000


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
        i2c = board.I2C()
        self.pca = PCA9685(i2c, address=DEFAULT_DRIVER_ADR)
        self.pca.frequency = 60

        self.left_motor = Motor(
            self.pca, DEFAULT_A_PWM, DEFAULT_A_IN1, DEFAULT_A_IN2, self.get_logger()
        )
        self.right_motor = Motor(
            self.pca, DEFAULT_B_PWM, DEFAULT_B_IN1, DEFAULT_B_IN2, self.get_logger()
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
        # self.get_logger().info(f"move_callback service is called")
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
