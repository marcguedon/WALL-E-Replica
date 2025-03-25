import rclpy
import cv2
import numpy as np
from ultralytics import YOLO
from rclpy.node import Node
from wall_e_msg_srv.srv import SwitchAutomaticMode
from wall_e_msg_srv.srv import GetAutomaticMode
from sensor_msgs.msg import Image
from wall_e_msg_srv.srv import Move
from wall_e_msg_srv.srv import SetLightIntensity

TARGETED_DISTANCE_M = 1.0
BRIGHTNESS_THRESHOLD = 50


class AutomaticModeNode(Node):
    def __init__(self):
        super().__init__("automatic_mode_node")

        self.declare_parameter("targeted_distance", TARGETED_DISTANCE_M)
        self.declare_parameter("brightness_threshold", BRIGHTNESS_THRESHOLD)

        self.targeted_distance = (
            self.get_parameter("targeted_distance").get_parameter_value().double_value
        )
        self.brightness_threshold = (
            self.get_parameter("brightness_threshold")
            .get_parameter_value()
            .integer_value
        )

        self.is_active = False

        self.switch_automatic_mode_srv = self.create_service(
            SwitchAutomaticMode,
            "switch_automatic_mode",
            self.switch_automatic_mode_callback,
        )

        self.get_automatic_mode_srv = self.create_service(
            GetAutomaticMode,
            "get_automatic_mode",
            self.get_automatic_mode_callback,
        )

        self.model = YOLO("yolov8n.pt")
        self.body_class_id = 0  # 'person' class ID

        self.frame_subscriber = self.create_subscription(
            Image, "camera_frame_topic", self.subscribe_frame_callback, 10
        )
        self.move_client = self.create_client(Move, "move")
        self.light_client = self.create_client(SetLightIntensity, "set_light_intensity")

        while not self.move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "move_client service not available yet, waiting again..."
            )

        while not self.light_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "light_client service not available yet, waiting again..."
            )

    def switch_automatic_mode_callback(self, request, response):
        automatic_mode_on = request.automatic_mode_on

        self.get_logger().debug(
            f"switch_automatic_mode_callback service called\nautomatic_mode_on: {automatic_mode_on}"
        )

        self.is_active = automatic_mode_on
        response.success = True

        self.get_logger().info(f"Turn automatic mode {self.is_active} !")

        return response

    def get_automatic_mode_callback(self, request, response):
        self.get_logger().debug(f"get_automatic_mode_callback service called")

        response.is_active = self.is_active

        return response

    def subscribe_frame_callback(self, msg):
        if self.is_active:
            try:
                frame = self.get_frame_from_message(msg)

                self.process_brightness(frame, self.brightness_threshold)

                width, height, _ = frame.shape
                left_border = int(width / 3 * 1)
                right_border = int(width / 3 * 2)

                results = self.model(frame, verbose=False)
                boxes = self.get_boxes_from_results(results, self.body_class_id)

                if len(boxes) > 0:
                    box = self.filter_boxes_by_distance(boxes, self.targeted_distance)

                    self.process_box(left_border, right_border, box)

                self.get_logger().debug(f"subscribe_frame_callback called")
            except Exception as e:
                self.get_logger().error(f"{e}")

    def get_frame_from_message(self, msg):
        np_array = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_array, cv2.IMREAD_COLOR)

        return frame

    def process_box(self, left_border, right_border, box):
        x_min, y_min, x_max, y_max, confidence = box
        x_mid = (x_min + x_max) / 2

        if x_mid < left_border:
            direction = -0.5
            self.get_logger().info("Go left")

        elif x_mid > right_border:
            direction = 0.5
            self.get_logger().info("Go right")

        else:
            direction = 0.0
            self.get_logger().info("Go straight")

        req = Move.Request()
        req.x_direction = 0.0
        req.y_direction = direction

        self.move_client.call_async(req)

    def process_brightness(self, frame, brightness_threshold: int):
        brightness = self.get_brightness(frame)

        if brightness < brightness_threshold:
            intensity = 100
            self.get_logger().info("Brightness too low, turn light on")
        else:
            intensity = 0
            self.get_logger().info("Brightness high enough, light off")

        req = SetLightIntensity.Request()
        req.light_id = "camera_light"
        req.intensity_pct = intensity

        self.light_client.call_async(req)

    def get_brightness(self, frame):
        brightness = np.dot(frame[..., :3], [0.299, 0.587, 0.114]).mean()

        return brightness

    def get_boxes_from_results(self, results, class_id):
        boxes = []

        for result in results:
            for box in result.boxes:
                xmin, ymin, xmax, ymax = box.xyxy[0].cpu().numpy()
                box_class_id = int(box.cls[0])
                confidence = box.conf[0]

                if box_class_id == class_id:
                    boxes.append((xmin, ymin, xmax, ymax, confidence))

        return boxes

    def estimate_body_distance(self, box):
        # Assuming a simple linear relationship between box height and distance
        xmin, ymin, xmax, ymax, _ = box
        box_height = ymax - ymin

        return 1 / box_height

    def filter_boxes_by_distance(self, boxes, target_distance_m: float):
        closest_box = None
        min_distance_diff = float("inf")

        for box in boxes:
            distance = self.estimate_body_distance(box)
            # print(f"distance: {distance}")
            distance_diff = abs(distance - target_distance_m)

            if distance_diff < min_distance_diff:
                min_distance_diff = distance_diff
                closest_box = box

        return closest_box

    def cleanup(self):
        super().destroy_node()
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AutomaticModeNode()

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
