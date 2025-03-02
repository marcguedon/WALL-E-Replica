# import ADS1x15
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8

DEFAULT_IN_MIN = 14460  # 9V -> 14460
DEFAULT_IN_MAX = 20195  # 12.6V -> 20195
DEFAULT_OUT_MIN = 0
DEFAULT_OUT_MAX = 100


class Battery:
    def __init__(
        self,
        driverAdr: int,
        in_min: int = DEFAULT_IN_MIN,
        in_max: int = DEFAULT_IN_MAX,
        out_min: int = DEFAULT_OUT_MIN,
        out_max: int = DEFAULT_OUT_MAX,
    ):
        """Constructor function

        Args:
            driverAdr (int): Adress of the ADC module
            in_min (int, optional): _description_. Defaults to DEFAULT_IN_MIN. # TODO
            in_max (int, optional): _description_. Defaults to DEFAULT_IN_MAX. # TODO
            out_min (int, optional): _description_. Defaults to DEFAULT_OUT_MIN. # TODO
            out_max (int, optional): _description_. Defaults to DEFAULT_OUT_MAX. # TODO
        """
        self.driver = ADS1x15.ADS1115(1, driverAdr)
        self.driver.setGain(self.driver.PGA_4_096V)
        self.in_min = in_min
        self.in_max = in_max
        self.out_min = out_min
        self.out_max = out_max

    def get_charge_percentage(self):
        """Returns the battery charge percentage as an int

        Returns:
            int: Battery percentage
        """
        return (
            int(
                (self.driver.readADC(0) - self.in_min)
                * (self.out_max - self.out_min)
                / (self.in_max - self.in_min)
            )
            + self.out_min
        )


DEFAULT_RATE = 1


class BatteryNode(Node):
    def __init__(
        self,
        rate: float = DEFAULT_RATE,
    ):
        """Constructor function

        Args:
            rate (int, optional): Numbers of posts per second. Defaults to DEFAULT_RATE.
        """
        super().__init__("battery_node")

        self.battery = Battery(0x48)
        self.rate = rate

        self.battery_charge_publisher = self.create_publisher(
            Int8, "battery_charge_topic", 10
        )
        self.timer = self.create_timer(1.0 / self.rate, self.publish_battery_charge)

    def publish_battery_charge(self):
        """Callback function that is called every X secondes, and sends the battery percentage to the topic"""
        msg = Int8()
        msg.data = self.battery.get_charge_percentage()
        self.battery_charge_publisher.publish(msg)
        # self.get_logger().info(f"Message published : {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    battery_node = BatteryNode()
    rclpy.spin(battery_node)
    battery_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
