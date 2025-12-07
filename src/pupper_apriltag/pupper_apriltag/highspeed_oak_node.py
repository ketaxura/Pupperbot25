import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from builtin_interfaces.msg import Time

import depthai as dai
import time


class OakHighSpeed(Node):
    def __init__(self):
        super().__init__("oak_highspeed_node")

        # -----------------------------
        # Build DepthAI pipeline
        # -----------------------------
        pipeline = dai.Pipeline()

        cam = pipeline.createColorCamera()
        cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720_P)
        cam.setFps(60)                       # TRUE 60 FPS
        cam.setVideoSize(1280, 720)
        cam.setInterleaved(False)            # planar BGR
        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        xout = pipeline.createXLinkOut()
        xout.setStreamName("video")
        cam.video.link(xout.input)

        # Start device
        self.device = dai.Device(pipeline)
        self.q_video = self.device.getOutputQueue("video", maxSize=2, blocking=False)

        # ROS2 publisher
        self.pub = self.create_publisher(Image, "/oak/rgb/image_raw", 10)

        # Timer to poll queue as fast as possible
        self.timer = self.create_timer(0.0, self.loop)

        self.get_logger().info("OAK-D High-Speed Node running at 60 FPS.")

    def loop(self):
        frame = self.q_video.tryGet()
        if frame is None:
            return

        # DepthAI gives BGR already (as a contiguous byte array)
        img_data = frame.getData()   # bytes

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "oak_rgb_frame"

        msg.height = frame.getHeight()
        msg.width = frame.getWidth()
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = frame.getWidth() * 3
        msg.data = img_data

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OakHighSpeed()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
