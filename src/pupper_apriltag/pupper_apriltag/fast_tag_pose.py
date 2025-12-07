import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from apriltag_msgs.msg import AprilTagDetectionArray
import numpy as np


class FastTagPose(Node):
    def __init__(self):
        super().__init__("fast_tag_pose")

        # Depth image subscriber
        self.create_subscription(
            Image,
            "/oak/stereo/image_raw",
            self.depth_callback,
            10
        )

        # Stereo camera intrinsics
        self.create_subscription(
            CameraInfo,
            "/oak/stereo/camera_info",
            self.caminfo_callback,
            10
        )

        # Apriltag detections
        self.create_subscription(
            AprilTagDetectionArray,
            "/detections",
            self.tag_callback,
            10
        )

        self.depth_image = None
        self.fx = None
        self.cx = None
        self.cy = None

    def caminfo_callback(self, msg: CameraInfo):
        """Grab fx, cx, cy from CameraInfo."""
        self.fx = msg.k[0]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

        # self.get_logger().info(
        #     # f"Loaded intrinsics: fx={self.fx:.1f}, cx={self.cx:.1f}, cy={self.cy:.1f}"
        # )

    def depth_callback(self, msg):
        """Store depth image."""
        if msg.encoding == "16UC1":
            dtype = np.uint16
        else:
            dtype = np.float32

        self.depth_image = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width)

    def tag_callback(self, msg):
        if self.depth_image is None or self.fx is None:
            return

        for det in msg.detections:
            tag_x = det.centre.x
            tag_y = det.centre.y

            # depth (mm or meters depending on encoding)
            depth = float(self.depth_image[int(tag_y), int(tag_x)])
            if depth == 0:
                continue

            depth_m = depth / 1000.0

            # ---------------------------------------------------
            # PURE HEADING ANGLE (NO TAG ORIENTATION)
            # ---------------------------------------------------
            dx = tag_x - self.cx
            heading_rad = np.arctan(dx / self.fx)
            heading_deg = np.degrees(heading_rad)

            self.get_logger().info(
                f"TAG {det.id} | Dist: {depth_m:.2f} m | Heading: {heading_deg:.2f} deg"
            )


def main(args=None):
    rclpy.init(args=args)
    node = FastTagPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
