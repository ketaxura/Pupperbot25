from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package="apriltag_ros",
            executable="apriltag_node",
            name="apriltag_detector",
            output="screen",
            parameters=[{
                "tag_family": "tag36h11",
                "tag_size": 0.149,
                "publish_tf": True,
                "publish_tag_detections_image": True,
                "image_transport": "raw",
            }],
            remappings=[
                ("image_rect", "/oak/rgb/image_raw"),
                ("image", "/oak/rgb/image_raw"),
                ("camera_info", "/oak/rgb/camera_info"),
            ],
        ),
    ])
