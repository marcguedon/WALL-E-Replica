import adafruit_ads1x15.ads1115 as ADS
import rclpy
import board
import busio
from rclpy.node import Node
from std_msgs.msg import Int8
from adafruit_ads1x15.analog_in import AnalogIn


class Battery:
    def __init__(
        self,
        driver_adress: int,
        in_min: int,
        in_max: int,
        out_min: int,
        out_max: int,
        logger=None,
    ):
        self.logger = logger

        i2c = busio.I2C(board.SCL, board.SDA)
        self.adc = ADS.ADS1115(i2c, address=driver_adress)
        self.channel = AnalogIn(self.adc, ADS.P0)
        self.in_min = in_min
        self.in_max = in_max
        self.out_min = out_min
        self.out_max = out_max

    def get_charge_percentage(self):
        adc_value = self.channel.value
        pct_battery_charge = int(
            (adc_value - self.in_min)
            * (self.out_max - self.out_min)
            / (self.in_max - self.in_min)
            + self.out_min
        )

        return pct_battery_charge


DEFAULT_RATE = 1
DEFAULT_ADR = 0x48

DEFAULT_IN_MIN = 14460  # -> 9V
DEFAULT_IN_MAX = 20195  # -> 12.6V
DEFAULT_OUT_MIN = 0
DEFAULT_OUT_MAX = 100


class BatteryNode(Node):
    def __init__(self):
        super().__init__("battery_node")

        self.declare_parameter("driver_adress", DEFAULT_ADR)
        self.declare_parameter("rate", DEFAULT_RATE)
        self.declare_parameter("in_min", DEFAULT_IN_MIN)
        self.declare_parameter("in_max", DEFAULT_IN_MAX)
        self.declare_parameter("out_min", DEFAULT_OUT_MIN)
        self.declare_parameter("out_max", DEFAULT_OUT_MAX)

        driver_adress = (
            self.get_parameter("driver_adress").get_parameter_value().integer_value
        )
        rate = self.get_parameter("rate").get_parameter_value().integer_value
        in_min = self.get_parameter("in_min").get_parameter_value().integer_value
        in_max = self.get_parameter("in_max").get_parameter_value().integer_value
        out_min = self.get_parameter("out_min").get_parameter_value().integer_value
        out_max = self.get_parameter("out_max").get_parameter_value().integer_value

        self.battery = Battery(
            driver_adress, in_min, in_max, out_min, out_max, logger=self.get_logger()
        )

        self.battery_charge_publisher = self.create_publisher(
            Int8, "battery_charge_topic", 10
        )
        self.timer = self.create_timer(1.0 / rate, self.publish_battery_charge)

    def publish_battery_charge(self):
        msg = Int8()
        msg.data = self.battery.get_charge_percentage()

        self.battery_charge_publisher.publish(msg)
        self.get_logger().debug(
            f"publish_battery_charge published\nmsg.data: {msg.data}"
        )

    def cleanup(self):
        super().destroy_node()
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()

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
