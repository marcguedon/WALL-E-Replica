import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from wall_e_msg_srv.srv import SetLightIntensity


class Light:
    def __init__(self, bcm_pin: int, is_default_on: bool = False, logger=None):
        self.logger = logger

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(bcm_pin, GPIO.OUT)

        self.pwm = GPIO.PWM(bcm_pin, 50)
        # self.is_default_on = is_default_on

        self.pwm.start(0)
        # self.pwm.ChangeDutyCycle(100 if is_default_on else 0)

    def set_intensity(self, intensity: int):
        self.pwm.ChangeDutyCycle(intensity)


BCM_CAMERA_LIGHT_PIN = 4
BCM_LED_LIGHT_PIN = 5


class LightsNode(Node):
    def __init__(self):
        super().__init__("lights_node")
        
        self.declare_parameter("bcm_camera_light_pin", BCM_CAMERA_LIGHT_PIN)
        self.declare_parameter("bcm_led_light_pin", BCM_LED_LIGHT_PIN)

        camera_light_pin = self.get_parameter("bcm_camera_light_pin").get_parameter_value().integer_value
        led_light_pin = self.get_parameter("bcm_led_light_pin").get_parameter_value().integer_value

        self.lights = {
            "camera_light": Light(
                camera_light_pin, is_default_on=False, logger=self.get_logger()
            ),
            "led_light": Light(
                led_light_pin, is_default_on=False, logger=self.get_logger()
            ),
        }

        self.set_light_intensity_srv = self.create_service(
            SetLightIntensity, "set_light_intensity", self.set_light_intensity_callback
        )

    def set_light_intensity_callback(self, request, response):
        light_id = request.light_id
        intensity_pct = request.intensity_pct

        self.get_logger().debug(
            f"set_intensity_callback service called\nlight_id: {light_id}, intensity_pct:{intensity_pct}"
        )

        self.lights[light_id].set_intensity(intensity_pct)
        response.success = True

        return response

    def cleanup(self):
        super().destroy_node()
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LightsNode()

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
