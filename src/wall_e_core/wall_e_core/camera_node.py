import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class Camera:
    def __init__(
        self,
        camera_index: int,
        width: int,
        height: int,
        logger=None,
    ):
        self.logger = logger

        self.camera = cv2.VideoCapture(camera_index)

        if not self.camera.isOpened():
            raise Exception("Error while initializing camera")

        self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def get_camera_capture(self):
        ret, frame = self.camera.read()

        if not ret:
            raise Exception("Error while capturing image")

        return frame

    def release(self):
        self.camera.release()


DEFAULT_FRAMERATE = 30
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

DEFAULT_CAMERA_INDEX = 0


class CameraNode(Node):
    def __init__(self, framerate: int = DEFAULT_FRAMERATE):
        super().__init__("camera_node")

        self.declare_parameter("camera_index", DEFAULT_CAMERA_INDEX)
        self.declare_parameter("framerate", DEFAULT_FRAMERATE)
        self.declare_parameter("frame_width", FRAME_WIDTH)
        self.declare_parameter("frame_height", FRAME_HEIGHT)

        camera_index = (
            self.get_parameter("camera_index").get_parameter_value().integer_value
        )
        framerate = self.get_parameter("framerate").get_parameter_value().integer_value
        frame_width = (
            self.get_parameter("frame_width").get_parameter_value().integer_value
        )
        frame_height = (
            self.get_parameter("frame_height").get_parameter_value().integer_value
        )

        self.camera = Camera(
            camera_index, frame_width, frame_height, logger=self.get_logger()
        )

        self.frame_publisher = self.create_publisher(Image, "camera_frame_topic", 10)

        self.timer = self.create_timer(1.0 / framerate, self.publish_frame_callback)

    def publish_frame_callback(self):
        try:
            frame = self.camera.get_camera_capture()
        except Exception as e:
            self.get_logger().error(f"{e}")

        if frame is None or frame.size == 0:
            self.get_logger().error("Frame is none or has no size")
            return

        ret, jpeg_frame = cv2.imencode(".jpg", frame)

        if not ret:
            self.get_logger().error("Failed to encode frame to JPEG")
            return

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = "jpeg"
        msg.is_bigendian = 0
        msg.step = frame.shape[1] * 3
        msg.data = jpeg_frame.tobytes()

        self.frame_publisher.publish(msg)
        self.get_logger().debug(f"Message published from publish_frame_callback")

    def cleanup(self):
        self.camera.release()
        super().destroy_node()
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()

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
