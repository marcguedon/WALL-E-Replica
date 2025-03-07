import RPi.GPIO as GPIO
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

GPIO.setwarnings(False)

BCM_LEFT_TRIG_PIN = 17
BCM_LEFT_ECHO_PIN = 18

BCM_RIGHT_TRIG_PIN = 22
BCM_RIGHT_ECHO_PIN = 23


class UltrasonicSensor:
    def __init__(self, bcm_trig_pin: int, bcm_echo_pin: int):
        self.bcm_trig_pin = bcm_trig_pin
        self.bcm_echo_pin = bcm_echo_pin

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.bcm_echo_pin, GPIO.IN)
        GPIO.setup(self.bcm_trig_pin, GPIO.OUT)
        GPIO.output(self.bcm_trig_pin, False)

    def get_distance(self):
        startTime = 0
        stopTime = 0

        initialTime = time.time()

        GPIO.output(self.bcm_trig_pin, True)
        time.sleep(1e-5)  # 10 microseconds
        GPIO.output(self.bcm_trig_pin, False)

        while GPIO.input(self.bcm_echo_pin) == 0 and startTime - initialTime < 0.05:
            startTime = time.time()

        while GPIO.input(self.bcm_echo_pin) == 1 and stopTime - startTime < 0.05:
            stopTime = time.time()

        distance_cm = round((stopTime - startTime) * 34300 / 2, 2)

        return distance_cm


DEFAULT_RATE = 20


class UltrasonicSensorsPublisher(Node):
    def __init__(self, rate: float = DEFAULT_RATE):
        """Constructor function

        Args:
            rate (float, optional): Numbers of posts per second. Defaults to DEFAULT_RATE.
        """
        super().__init__("ultrasonic_sensors_node")

        self.sensors = [
            UltrasonicSensor(BCM_LEFT_TRIG_PIN, BCM_LEFT_ECHO_PIN),  # left sensor
            UltrasonicSensor(BCM_RIGHT_TRIG_PIN, BCM_RIGHT_ECHO_PIN),  # right sensor
            # UltrasonicSensor(None, None),  # front sensor
        ]
        self.rate = rate

        self.publisher = self.create_publisher(Float32MultiArray, "distances_topic", 10)
        self.timer = self.create_timer(1.0 / self.rate, self.publish_distances)

    def publish_distances(self):
        msg = Float32MultiArray()
        msg.data = [sensor.get_distance() for sensor in self.sensors]
        self.publisher.publish(msg)
        # self.get_logger().info(f"Message published : {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicSensorsPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
