import cv2
import mediapipe as mp
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from wall_e_msg_srv.srv import SwitchAI

DEFAULT_CAMERA_INDEX = 0
DEFAULT_FRAME_WIDTH = 640
DEFAULT_FRAME_HEIGHT = 480


class Camera:
    def __init__(self, camera_index: int = DEFAULT_CAMERA_INDEX, logger=None):
        """Captures pictures to display it with or without AI overlay

        Args:
            camera_index (int, optional): Index of the camera. Defaults to DEFAULT_CAMERA_INDEX.
        """
        self.logger = logger

        self.display_ai_overlay = False
        self.mp_hands = mp.solutions.hands.Hands(
            min_detection_confidence=0.8, min_tracking_confidence=0.8
        )
        # self.mp_poses = mp.solutions.pose.Pose(
        #     min_detection_confidence=0.8, min_tracking_confidence=0.8
        # )
        self.mp_drawing = mp.solutions.drawing_utils

        self.camera = cv2.VideoCapture(camera_index)

        if not self.camera.isOpened():
            raise Exception("Error while initializing camera")

        self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, DEFAULT_FRAME_WIDTH)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, DEFAULT_FRAME_HEIGHT)

    def get_camera_capture(self):
        ret, frame = self.camera.read()

        if not ret:
            raise Exception("Error while capturing image")

        if self.display_ai_overlay:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            hands_results = self.mp_hands.process(frame)
            # poses_results = self.mp_poses.process(frame)

            if hands_results.multi_hand_landmarks:
                for hand_landmarks in hands_results.multi_hand_landmarks:
                    self.mp_drawing.draw_landmarks(
                        frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS
                    )

            # if poses_results.pose_landmarks:
            #     self.mp_drawing.draw_landmarks(
            #         frame, poses_results.pose_landmarks, self.mp_poses.POSE_CONNECTIONS
            #     )

        return frame

    def switch_ai(self, display_ai: bool):
        """Switches on/off the AI overlay of the camera

        Args:
            display_ai (bool): _description_ # TODO
        """
        # self.display_ai_overlay = display_ai
        pass

    def release_camera(self):
        """Releases the camera"""
        self.mp_hands.close()

        self.process.terminate()
        self.process.wait()


DEFAULT_FRAMERATE = 30


class CameraNode(Node):
    def __init__(self, framerate: int = DEFAULT_FRAMERATE):
        """_summary_

        Args:
            framerate (int, optional): Numbers of captured pictures per second. Defaults to DEFAULT_FRAMERATE.
        """
        super().__init__("camera_node")

        self.camera = Camera(logger=self.get_logger())
        self.framerate = framerate

        self.frame_publisher = self.create_publisher(Image, "camera_frame_topic", 10)
        self.switch_ai_srv = self.create_service(
            SwitchAI, "switch_ai", self.switch_ai_callback
        )

        self.timer = self.create_timer(
            1.0 / self.framerate, self.publish_frame_callback
        )
        self.get_logger().info(f"Start camera capture")

    def publish_frame_callback(self):
        """Capture et publie une image"""
        frame = self.camera.get_camera_capture()

        if frame is None or frame.size == 0:
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

    def switch_ai_callback(self, request, response):
        """Callback function to switch on/off the AI overlay

        Args:
            request (_type_): _description_
            response (_type_): _description_
        """
        self.camera.switch_ai(request.ai_on)
        response.success = True

        return response

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
