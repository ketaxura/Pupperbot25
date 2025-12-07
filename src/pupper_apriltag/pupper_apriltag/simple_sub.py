import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class SimpleImageSub(Node):
    def __init__(self):
        super().__init__("simple_image_sub")

        self.sub = self.create_subscription(
            Image,
            "/oak/rgb/image_raw",
            self.callback,
            10
        )

        self.last_print_time = 0.0
        self.print_period = 5.0   # seconds

        self.get_logger().info("Simple image subscriber started.")

    def callback(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9  # convert to seconds

        if now - self.last_print_time >= self.print_period:
            self.last_print_time = now
            self.get_logger().info(
                f"Received frame: {msg.width}x{msg.height}  stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec}"
            )


def main():
    rclpy.init()
    node = SimpleImageSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
