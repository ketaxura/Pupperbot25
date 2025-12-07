import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import Vector3
import numpy as np


class FastTagPose(Node):
    def __init__(self):
        super().__init__("fast_tag_pose")

        # Publishers
        self.tag_pose_pub = self.create_publisher(
            Vector3,
            "/tag_pose_fast",
            10
        )

        # Subscribers
        self.create_subscription(Image, "/oak/stereo/image_raw",
                                 self.depth_callback, 10)

        self.create_subscription(CameraInfo, "/oak/stereo/camera_info",
                                 self.caminfo_callback, 10)

        self.create_subscription(AprilTagDetectionArray, "/detections",
                                 self.tag_callback, 10)

        self.depth_image = None
        self.fx = None
        self.cx = None
        self.cy = None

    def caminfo_callback(self, msg: CameraInfo):
        self.fx = msg.k[0]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def depth_callback(self, msg):
        if msg.encoding == "16UC1":
            dtype = np.uint16
        else:
            dtype = np.float32

        self.depth_image = np.frombuffer(msg.data, dtype=dtype)\
                               .reshape(msg.height, msg.width)

    def tag_callback(self, msg):
        if self.depth_image is None or self.fx is None:
            return

        for det in msg.detections:
            u = int(det.centre.x)
            v = int(det.centre.y)

            depth_val = float(self.depth_image[v, u])
            if depth_val == 0:
                continue

            depth_m = depth_val / 1000.0

            # Heading (horizontal angle)
            dx = det.centre.x - self.cx
            heading_rad = np.arctan(dx / self.fx)
            heading_deg = np.degrees(heading_rad)

            # ---------- PUBLISH ----------
            msg_out = Vector3()
            msg_out.x = depth_m       # meters
            msg_out.y = heading_deg   # degrees
            msg_out.z = float(det.id) # tag id encoded in z

            self.tag_pose_pub.publish(msg_out)

            # Optional debug print
            # self.get_logger().info(f"TAG {det.id}: dist={depth_m:.2f} m  head={heading_deg:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = FastTagPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
