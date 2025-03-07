import cv2
import mediapipe as mp
import rclpy
import ffmpeg
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from wall_e_msg_srv.srv import SwitchAI

DEFAULT_CAMERA_INDEX = 0


class Camera:
    def __init__(self, camera_index: int = DEFAULT_CAMERA_INDEX, logger=None):
        """Captures pictures to display it with or without AI overlay

        Args:
            camera_index (int, optional): Index of the camera. Defaults to DEFAULT_CAMERA_INDEX.
        """
        self.logger = logger

        self.display_ai_overlay = False
        self.mp_hands = mp.solutions.hands.Hands()

        self.process = (
            ffmpeg.input("/tmp/video_stream.h264")
            .output("pipe:1", format="rawvideo", pix_fmt="bgr24")
            .run_async(pipe_stdout=True, pipe_stderr=True)
        )

    def get_camera_capture(self):
        in_bytes = self.process.stdout.read(640 * 480 * 3)

        if len(in_bytes) < 640 * 480 * 3:
            return

        data = np.frombuffer(in_bytes, np.uint8).reshape([480, 640, 3])

        if self.logger:
            self.logger.info(f"Frame shape: {data.shape}%")

        if self.display_ai_overlay:
            image = cv2.cvtColor(data, cv2.COLOR_BGR2RGB)

            results = self.mp_hands.process(image)

            # Displaying landmarks for each detected hand
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    for landmark in hand_landmarks.landmark:
                        x = min(int(landmark.x * data.shape[1]), data.shape[1] - 1)
                        y = min(int(landmark.y * data.shape[0]), data.shape[0] - 1)
                        cv2.circle(data, (x, y), 5, (0, 255, 0), -1)

                    connections = mp.solutions.holistic.HAND_CONNECTIONS
                    for connection in connections:
                        x0 = min(
                            int(
                                hand_landmarks.landmark[connection[0]].x * data.shape[1]
                            ),
                            data.shape[1] - 1,
                        )
                        y0 = min(
                            int(
                                hand_landmarks.landmark[connection[0]].y * data.shape[0]
                            ),
                            data.shape[0] - 1,
                        )
                        x1 = min(
                            int(
                                hand_landmarks.landmark[connection[1]].x * data.shape[1]
                            ),
                            data.shape[1] - 1,
                        )
                        y1 = min(
                            int(
                                hand_landmarks.landmark[connection[1]].y * data.shape[0]
                            ),
                            data.shape[0] - 1,
                        )
                        cv2.line(data, (x0, y0), (x1, y1), (0, 255, 0), 2)

        return data

    def switch_ai(self, display_ai: bool):
        """Switches on/off the AI overlay of the camera

        Args:
            display_ai (bool): _description_ # TODO
        """
        self.display_ai_overlay = display_ai

    def release_camera(self):
        """Releases the camera"""
        self.camera.release()
        self.mp_hands.close()


DEFAULT_FRAMERATE = 30


class CameraNode(Node):
    def __init__(self, framerate: int = DEFAULT_FRAMERATE):
        """_summary_

        Args:
            framerate (int, optional): Numbers of captured pictures per second. Defaults to DEFAULT_FRAMERATE.
        """
        super().__init__("camera_node")

        self.camera = Camera(self.get_logger())
        self.framerate = framerate
        self.bridge = CvBridge()

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

    def destroy_node(self):
        """Libère la caméra lors de l'arrêt du node"""
        self.camera.release_camera()
        super().destroy_node()

        self.get_logger().info(f"Stop camera capture")

    def switch_ai_callback(self, request, response):
        """Callback function to switch on/off the AI overlay

        Args:
            request (_type_): _description_
            response (_type_): _description_
        """
        self.camera.switch_ai(request.ai_on)
        response.success = True

        return response


def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
