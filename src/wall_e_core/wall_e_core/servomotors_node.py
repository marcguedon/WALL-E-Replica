import board
import rclpy
from rclpy.node import Node
from adafruit_pca9685 import PCA9685
from wall_e_msg_srv.srv import MoveArm
from wall_e_msg_srv.srv import MoveEye
from wall_e_msg_srv.srv import MoveHead


class Servomotor:
    def __init__(self, driver, channel: int, logger=None):
        self.logger = logger

        self.driver = driver
        self.channel = channel

    def set_angle(self, angle_deg: int):
        pulse_length = int((int(angle_deg) - 0) * (2000 - 1000) / (180 - 0) + 1000)
        duty_cycle = int(pulse_length * 65535 / 20000)

        self.driver.channels[self.channel].duty_cycle = duty_cycle


DEFAULT_DRIVER_ADR = 0x41
DEFAULT_HEAD_ROTATION_CHANNEL = 0
DEFAULT_NECK_TOP_CHANNEL = 1
DEFAULT_NECK_BOTTOM_CHANNEL = 2
DEFAULT_LEFT_EYE_CHANNEL = 3
DEFAULT_RIGHT_EYE_CHANNEL = 4
DEFAULT_LEFT_ARM_CHANNEL = 5
DEFAULT_RIGHT_ARM_CHANNEL = 6

DEFAULT_HEAD_ROTATION_INIT_ANGLE = 90
DEFAULT_NECK_TOP_INIT_ANGLE = 0
DEFAULT_NECK_BOTTOM_INIT_ANGLE = 30
DEFAULT_LEFT_EYE_INIT_ANGLE = 70
DEFAULT_RIGHT_EYE_INIT_ANGLE = 110
DEFAULT_LEFT_ARM_INIT_ANGLE = 140
DEFAULT_RIGHT_ARM_INIT_ANGLE = 40


# TODO: Rework the operation of the various servomotors
class ServomotorsNode(Node):
    def __init__(self):
        super().__init__("servomotors_node")

        self.declare_parameter("driver_adress", DEFAULT_DRIVER_ADR)
        self.declare_parameter("head_rotation_channel", DEFAULT_HEAD_ROTATION_CHANNEL)
        self.declare_parameter("neck_top_channel", DEFAULT_NECK_TOP_CHANNEL)
        self.declare_parameter("neck_bot_channel", DEFAULT_NECK_BOTTOM_CHANNEL)
        self.declare_parameter("left_eye_channel", DEFAULT_LEFT_EYE_CHANNEL)
        self.declare_parameter("right_eye_channel", DEFAULT_RIGHT_EYE_CHANNEL)
        self.declare_parameter("left_arm_channel", DEFAULT_LEFT_ARM_CHANNEL)
        self.declare_parameter("right_arm_channel", DEFAULT_RIGHT_ARM_CHANNEL)
        self.declare_parameter(
            "head_rotation_init_angle", DEFAULT_HEAD_ROTATION_INIT_ANGLE
        )
        self.declare_parameter("neck_top_init_angle", DEFAULT_NECK_TOP_INIT_ANGLE)
        self.declare_parameter("neck_bot_init_angle", DEFAULT_NECK_BOTTOM_INIT_ANGLE)
        self.declare_parameter("left_eye_init_angle", DEFAULT_LEFT_EYE_INIT_ANGLE)
        self.declare_parameter("right_eye_init_angle", DEFAULT_RIGHT_EYE_INIT_ANGLE)
        self.declare_parameter("left_arm_init_angle", DEFAULT_LEFT_ARM_INIT_ANGLE)
        self.declare_parameter("right_arm_init_angle", DEFAULT_RIGHT_ARM_INIT_ANGLE)

        driver_adress = (
            self.get_parameter("driver_adress").get_parameter_value().integer_value
        )
        head_rotation_channel = (
            self.get_parameter("head_rotation_channel")
            .get_parameter_value()
            .integer_value
        )
        neck_top_channel = (
            self.get_parameter("neck_top_channel").get_parameter_value().integer_value
        )
        neck_bot_channel = (
            self.get_parameter("neck_bot_channel").get_parameter_value().integer_value
        )
        left_eye_channel = (
            self.get_parameter("left_eye_channel").get_parameter_value().integer_value
        )
        right_eye_channel = (
            self.get_parameter("right_eye_channel").get_parameter_value().integer_value
        )
        left_arm_channel = (
            self.get_parameter("left_arm_channel").get_parameter_value().integer_value
        )
        right_arm_channel = (
            self.get_parameter("right_arm_channel").get_parameter_value().integer_value
        )
        self.head_rotation_init_angle = (
            self.get_parameter("head_rotation_init_angle")
            .get_parameter_value()
            .integer_value
        )
        self.neck_top_init_angle = (
            self.get_parameter("neck_top_init_angle")
            .get_parameter_value()
            .integer_value
        )
        self.neck_bot_init_angle = (
            self.get_parameter("neck_bot_init_angle")
            .get_parameter_value()
            .integer_value
        )
        self.left_eye_init_angle = (
            self.get_parameter("left_eye_init_angle")
            .get_parameter_value()
            .integer_value
        )
        self.right_eye_init_angle = (
            self.get_parameter("right_eye_init_angle")
            .get_parameter_value()
            .integer_value
        )
        self.left_arm_init_angle = (
            self.get_parameter("left_arm_init_angle")
            .get_parameter_value()
            .integer_value
        )
        self.right_arm_init_angle = (
            self.get_parameter("right_arm_init_angle")
            .get_parameter_value()
            .integer_value
        )

        i2c = board.I2C()
        self.driver = PCA9685(i2c, address=driver_adress)
        self.driver.frequency = 60

        self.head_rotation = Servomotor(
            self.driver, head_rotation_channel, self.get_logger()
        )
        self.neck_top = Servomotor(self.driver, neck_top_channel, self.get_logger())
        self.neck_bottom = Servomotor(self.driver, neck_bot_channel, self.get_logger())
        self.left_eye = Servomotor(self.driver, left_eye_channel, self.get_logger())
        self.right_eye = Servomotor(self.driver, right_eye_channel, self.get_logger())
        self.left_arm = Servomotor(self.driver, left_arm_channel, self.get_logger())
        self.right_arm = Servomotor(self.driver, right_arm_channel, self.get_logger())

        self.init()

        self.current_head_rotation_angle = self.head_rotation_init_angle
        self.current_neck_top_angle = self.neck_top_init_angle
        self.current_neck_bottom_angle = self.neck_bot_init_angle

        self.step = 4

        self.move_arm_srv = self.create_service(
            MoveArm, "move_arm", self.move_arm_callback
        )
        self.move_head_srv = self.create_service(
            MoveHead, "move_head", self.move_head_callback
        )
        self.move_eye_srv = self.create_service(
            MoveEye, "move_eye", self.move_eye_callback
        )

    def init(self):
        self.head_rotation.set_angle(self.head_rotation_init_angle)
        self.neck_top.set_angle(self.neck_top_init_angle)
        self.neck_bottom.set_angle(self.neck_bot_init_angle)
        self.left_eye.set_angle(self.left_eye_init_angle)
        self.right_eye.set_angle(self.right_eye_init_angle)
        self.left_arm.set_angle(self.left_arm_init_angle)
        self.right_arm.set_angle(self.right_arm_init_angle)

    def move_arm(self, arm: str, angle: int):
        if arm == "left":
            # Max angle (up) : 40 / min angle (down) : 140
            self.left_arm.set_angle(angle)

        if arm == "right":
            # Max angle (up) : 140 / min angle (down) : 40
            self.right_arm.set_angle(angle)

    def move_head(self, x_direction: str, y_direction: str):
        if x_direction == "left":
            if self.current_head_rotation_angle + self.step <= 150:
                self.current_head_rotation_angle += self.step
                self.head_rotation.set_angle(self.current_head_rotation_angle)
            else:
                self.current_head_rotation_angle = 150

        if x_direction == "right":
            if self.current_head_rotation_angle - self.step >= 30:
                self.current_head_rotation_angle -= self.step
                self.head_rotation.set_angle(self.current_head_rotation_angle)
            else:
                self.current_head_rotation_angle = 30

        if y_direction == "up":
            if self.current_neck_top_angle - self.step >= 0:
                self.current_neck_top_angle -= self.step
                self.neck_top.set_angle(self.current_neck_top_angle)
            else:
                self.current_neck_top_angle = 0

            if self.current_neck_bottom_angle + self.step <= 180:
                self.current_neck_bottom_angle += self.step
                self.neck_bottom.set_angle(self.current_neck_bottom_angle)
            else:
                self.current_neck_bottom_angle = 180

        if y_direction == "down":
            if self.current_neck_top_angle + self.step <= 180:
                self.current_neck_top_angle += self.step
                self.neck_top.set_angle(self.current_neck_top_angle)
            else:
                self.current_neck_top_angle = 180

            if self.current_neck_bottom_angle - self.step >= 30:
                self.current_neck_bottom_angle -= self.step
                self.neck_bottom.set_angle(self.current_neck_bottom_angle)
            else:
                self.current_neck_bottom_angle = 30

    def move_eye(self, eye: str, angle: int):
        if eye == "left":
            # max angle (up) = 40 / min angle (down) = 140
            self.left_eye.set_angle(angle)

        if eye == "right":
            # max angle (up) = 140 / min angle (down) = 40
            self.right_eye.set_angle(angle)

    def move_arm_callback(self, request, response):
        arm_id = request.arm_id
        angle = request.angle

        self.get_logger().debug(
            f"move_arm_callback service called\narm_id: {arm_id}, angle: {angle}"
        )

        self.move_arm(arm_id, angle)
        response.success = True

        return response

    def move_head_callback(self, request, response):
        x_direction = request.x_direction
        y_direction = request.y_direction

        self.get_logger().debug(
            f"move_head_callback service called\nx_direction: {x_direction}, y_direction: {y_direction}"
        )

        self.move_head(x_direction, y_direction)
        response.success = True

        return response

    def move_eye_callback(self, request, response):
        eye_id = request.eye_id
        angle = request.angle

        self.get_logger().debug(
            f"move_eye_callback service called\neye_id: {eye_id}, angle: {angle}"
        )

        self.move_eye(eye_id, angle)
        response.success = True

        return response

    def cleanup(self):
        self.driver.deinit()
        super().destroy_node()
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServomotorsNode()

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
