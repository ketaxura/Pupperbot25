import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/oak/rgb/image_raw',   # <-- or image_rect if you prefer
            self.callback,
            10
        )

        self.get_logger().info("Camera viewer node started.")

    def callback(self, msg):
        try:
            # Convert using the encoding advertised in the message
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            # If the image is RGB, convert to BGR for OpenCV display
            if msg.encoding == "rgb8":
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

            cv2.imshow("OAK RGB Feed", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraViewer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
