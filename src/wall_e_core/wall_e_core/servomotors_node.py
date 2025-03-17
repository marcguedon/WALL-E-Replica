import board
import rclpy
from rclpy.node import Node
from adafruit_pca9685 import PCA9685
from wall_e_msg_srv.srv import MoveArm
from wall_e_msg_srv.srv import MoveEye
from wall_e_msg_srv.srv import MoveHead


class Servomotor:
    def __init__(self, pca, channel: int, logger=None):
        self.pca = pca
        self.channel = channel
        self.logger = logger

    def set_angle(self, angle_deg: int):
        pulse_length = int((int(angle_deg) - 0) * (2000 - 1000) / (180 - 0) + 1000)
        duty_cycle = int(pulse_length * 65535 / 20000)

        self.pca.channels[self.channel].duty_cycle = duty_cycle


DEFAULT_DRIVER_ADR = 0x41
DEFAULT_HEAD_ROTATION_CHANNEL = 0
DEFAULT_HEAD_ROTATION_ANGLE = 90
DEFAULT_NECK_TOP_CHANNEL = 1
DEFAULT_NECK_TOP_ANGLE = 0
DEFAULT_NECK_BOTTOM_CHANNEL = 2
DEFAULT_NECK_BOTTOM_ANGLE = 30
DEFAULT_RIGHT_EYE_CHANNEL = 3
DEFAULT_RIGHT_EYE_ANGLE = 110
DEFAULT_LEFT_EYE_CHANNEL = 4
DEFAULT_LEFT_EYE_ANGLE = 70
DEFAULT_LEFT_ARM_CHANNEL = 5
DEFAULT_LEFT_ARM_ANGLE = 140
DEFAULT_RIGHT_ARM_CHANNEL = 6
DEFAULT_RIGHT_ARM_ANGLE = 40


class ServomotorsNode(Node):
    def __init__(self):
        super().__init__("servomotors_node")
        i2c = board.I2C()
        self.pca = PCA9685(i2c, address=DEFAULT_DRIVER_ADR)
        self.pca.frequency = 60

        self.head_rotation = Servomotor(
            self.pca, DEFAULT_HEAD_ROTATION_CHANNEL, self.get_logger()
        )
        self.neck_top = Servomotor(
            self.pca, DEFAULT_NECK_TOP_CHANNEL, self.get_logger()
        )
        self.neck_bottom = Servomotor(
            self.pca, DEFAULT_NECK_BOTTOM_CHANNEL, self.get_logger()
        )
        self.right_eye = Servomotor(
            self.pca, DEFAULT_RIGHT_EYE_CHANNEL, self.get_logger()
        )
        self.left_eye = Servomotor(
            self.pca, DEFAULT_LEFT_EYE_CHANNEL, self.get_logger()
        )
        self.left_arm = Servomotor(
            self.pca, DEFAULT_LEFT_ARM_CHANNEL, self.get_logger()
        )
        self.right_arm = Servomotor(
            self.pca, DEFAULT_RIGHT_ARM_CHANNEL, self.get_logger()
        )

        self.init()

        self.current_head_rotation_angle = 90
        self.current_neck_top_angle = 10
        self.current_neck_bottom_angle = 30

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
        self.head_rotation.set_angle(DEFAULT_HEAD_ROTATION_ANGLE)
        self.neck_top.set_angle(DEFAULT_NECK_TOP_ANGLE)
        self.neck_bottom.set_angle(DEFAULT_NECK_BOTTOM_ANGLE)
        self.right_eye.set_angle(DEFAULT_RIGHT_EYE_ANGLE)
        self.left_eye.set_angle(DEFAULT_LEFT_EYE_ANGLE)
        self.left_arm.set_angle(DEFAULT_LEFT_ARM_ANGLE)
        self.right_arm.set_angle(DEFAULT_RIGHT_ARM_ANGLE)

    def move_arm(self, arm: str, angle: int):
        if arm == "left":
            # Max angle (up) : 40 / min angle (down) : 140
            self.left_arm.set_angle(angle)

        if arm == "right":
            # Max angle (up) : 140 / min angle (down) : 40
            self.right_arm.set_angle(angle)

    def move_head(self, direction: str):
        if direction == "left":
            if self.current_head_rotation_angle + self.step <= 150:
                self.current_head_rotation_angle += self.step
                self.head_rotation.set_angle(self.current_head_rotation_angle)
            else:
                self.current_head_rotation_angle = 150

        if direction == "right":
            if self.current_head_rotation_angle - self.step >= 30:
                self.current_head_rotation_angle -= self.step
                self.head_rotation.set_angle(self.current_head_rotation_angle)
            else:
                self.current_head_rotation_angle = 30

        if direction == "up":
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

        if direction == "down":
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
        direction = request.direction

        self.get_logger().debug(
            f"move_head_callback service called\ndirection: {direction}"
        )

        self.move_head(direction)
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
        self.pca.deinit()
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
