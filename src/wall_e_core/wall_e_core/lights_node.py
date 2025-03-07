import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from wall_e_msg_srv.srv import SetIntensity

GPIO.setwarnings(False)


class Light:
    def __init__(self, bcm_pin: int, is_default_on: bool):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(bcm_pin, GPIO.OUT)

        self.pwm = GPIO.PWM(bcm_pin, 50)
        self.is_default_on = is_default_on

        self.pwm.start(0)
        # Turn off the light at start
        self.pwm.ChangeDutyCycle(100 if is_default_on else 0)

    def set_intensity(self, intensity: int):
        self.pwm.ChangeDutyCycle(intensity)


BCM_CAMERA_LIGHT_PIN = 4
BCM_LED_LIGHT_PIN = 5


class LightsService(Node):
    def __init__(self):
        super().__init__("lights_node")

        self.lights = {
            "camera_light": Light(BCM_CAMERA_LIGHT_PIN, False),
            "speaker_light": Light(BCM_LED_LIGHT_PIN, False),
        }

        self.set_intensity_srv = self.create_service(
            SetIntensity, "set_intensity", self.set_intensity_callback
        )

    def set_intensity_callback(self, request, response):
        light_id = request.light_id
        intensity_pct = request.intensity_pct

        self.lights[light_id].set_intensity(intensity_pct)
        response.success = True

        return response


def main(args=None):
    rclpy.init(args=args)
    node = LightsService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
