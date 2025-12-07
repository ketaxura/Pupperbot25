from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    apriltag_params = {
        "tag_family": "tag36h11",
        "tag_size": 0.162,
        "publish_tf": True,
        "publish_tag_detections_image": True,
    }

    return LaunchDescription([

        # Keepalive subscriber to correctly trigger ISP RGB
        Node(
            package="image_tools",
            executable="showimage",
            name="rgb_keepalive",
            remappings=[("image", "/oak/rgb/image_rect")],
            output="screen"
        ),

        # AprilTag detector
        Node(
            package="apriltag_ros",
            executable="apriltag_node",
            name="apriltag_detector",
            output="screen",
            parameters=[apriltag_params, {"image_transport": "raw"}],
            remappings=[
                ('image_rect', '/oak/rgb/image_rect'),
                ('image', '/oak/rgb/image_rect'),
                ('camera_info', '/oak/rgb/camera_info'),
            ],
        ),
    ])
