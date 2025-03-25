import board
import rclpy
from numpy import sqrt
from rclpy.node import Node
from adafruit_pca9685 import PCA9685
from wall_e_msg_srv.srv import Move


class Motor:
    def __init__(self, driver, pwm: int, in1: int, in2: int, logger=None):
        self.logger = logger

        self.driver = driver
        self.pwm = pwm
        self.in1 = in1
        self.in2 = in2

    def run(self, speed: float):
        if speed == 0:
            self.driver.channels[self.pwm].duty_cycle = 0x0000
            return

        abs_speed = min(abs(speed), 1)
        speed_pct = int(abs_speed * 65535)  # PWM range for PCA9685 is 0 to 4095
        forward = speed > 0

        self.driver.channels[self.pwm].duty_cycle = speed_pct

        # Control direction using IN1 and IN2
        if forward:
            self.driver.channels[self.in1].duty_cycle = 0x0000
            self.driver.channels[self.in2].duty_cycle = 0xFFFF
        else:
            self.driver.channels[self.in1].duty_cycle = 0xFFFF
            self.driver.channels[self.in2].duty_cycle = 0x0000


DEFAULT_DRIVER_ADR = 0x40
DEFAULT_A_PWM_CHANNEL = 0
DEFAULT_A_IN1_CHANNEL = 1
DEFAULT_A_IN2_CHANNEL = 2
DEFAULT_B_PWM_CHANNEL = 5
DEFAULT_B_IN1_CHANNEL = 3
DEFAULT_B_IN2_CHANNEL = 4


class MotorsNode(Node):
    def __init__(self):
        super().__init__("motors_node")

        self.declare_parameter("driver_adress", DEFAULT_DRIVER_ADR)
        self.declare_parameter("a_pwm_channel", DEFAULT_A_PWM_CHANNEL)
        self.declare_parameter("a_in1_channel", DEFAULT_A_IN1_CHANNEL)
        self.declare_parameter("a_in2_channel", DEFAULT_A_IN2_CHANNEL)
        self.declare_parameter("b_pwm_channel", DEFAULT_B_PWM_CHANNEL)
        self.declare_parameter("b_in1_channel", DEFAULT_B_IN1_CHANNEL)
        self.declare_parameter("b_in2_channel", DEFAULT_B_IN2_CHANNEL)

        driver_adress = (
            self.get_parameter("driver_adress").get_parameter_value().integer_value
        )
        a_pwm_channel = (
            self.get_parameter("a_pwm_channel").get_parameter_value().integer_value
        )
        a_in1_channel = (
            self.get_parameter("a_in1_channel").get_parameter_value().integer_value
        )
        a_in2_channel = (
            self.get_parameter("a_in2_channel").get_parameter_value().integer_value
        )
        b_pwm_channel = (
            self.get_parameter("b_pwm_channel").get_parameter_value().integer_value
        )
        b_in1_channel = (
            self.get_parameter("b_in1_channel").get_parameter_value().integer_value
        )
        b_in2_channel = (
            self.get_parameter("b_in2_channel").get_parameter_value().integer_value
        )

        i2c = board.I2C()
        self.driver = PCA9685(i2c, address=driver_adress)
        self.driver.frequency = 1000

        self.left_motor = Motor(
            self.driver,
            a_pwm_channel,
            a_in1_channel,
            a_in2_channel,
            self.get_logger(),
        )
        self.right_motor = Motor(
            self.driver,
            b_pwm_channel,
            b_in1_channel,
            b_in2_channel,
            self.get_logger(),
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
        x_direction = request.x_direction
        y_direction = request.y_direction

        self.get_logger().debug(
            f"move_callback service called\nx_direction: {x_direction}, y_direction:{y_direction}"
        )

        self.move(x_direction, y_direction)
        response.success = True

        return response

    def cleanup(self):
        self.driver.deinit()
        super().destroy_node()
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested, stopping node...")
    except:
        pass
    finally:
        node.cleanup()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
