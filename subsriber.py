import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class ScienceSubscriber(Node):
    def __init__(self):
        super().__init__("science_subscriber_py")

        self.subscription = self.create_subscription(
            Float32MultiArray, "science_sensor_data", self.callback, 10
        )

    def callback(self, msg: Float32MultiArray):
        data = msg.data

        if len(data) < 14:
            self.get_logger().warn("Received incomplete data array")
            return

        red = bool(data[0])
        purple = bool(data[1])
        pink = bool(data[2])

        N = data[3]
        P = data[4]
        K = data[5]

        ph = data[6]
        co2 = data[7]
        temp = data[8]
        press = data[9]
        alt = data[10]
        lat = data[11]
        lon = data[12]
        dist = data[13]

        self.get_logger().info(
            f"\nColors -> Red:{red}, Purple:{purple}, Pink:{pink}"
            f"\nNPK -> N:{N:.1f}, P:{P:.1f}, K:{K:.1f}"
            f"\npH:{ph:.2f}, CO2:{co2:.1f} ppm"
            f"\nTemp:{temp:.2f} °C, Pressure:{press:.2f} hPa, Alt:{alt:.2f} m"
            f"\nLat:{lat:.6f}, Lon:{lon:.6f}, Distance:{dist:.2f} cm"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ScienceSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
