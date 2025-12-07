from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
import os

def generate_launch_description():

    show_view = LaunchConfiguration("show_view")

    depthai_launch = os.path.join(
        "/opt/ros/humble/share/depthai_ros_driver/launch/rgbd_pcl.launch.py"
    )

    return LaunchDescription([

        # ========================
        #  Argument
        # ========================
        DeclareLaunchArgument(
            "show_view",
            default_value="false",
            description="Show image windows (image_view + camera_viewer)"
        ),

        # ========================
        #  OAK-D driver
        # ========================
        IncludeLaunchDescription(   
            PythonLaunchDescriptionSource(depthai_launch)
        ),


        # ========================
        #  AprilTag detector
        # ========================
        Node(
            package="apriltag_ros",
            executable="apriltag_node",
            parameters=[{
                "tag_family": "tag36h11",
                "tag_size": 0.149,
                "publish_tf": True,
                "publish_tag_detections_image": True,
                "image_transport": "raw",
            }],
            remappings=[
                ("image", "/oak/rgb/image_raw"),
                ("image_rect", "/oak/rgb/image_raw"),
                ("camera_info", "/oak/rgb/camera_info"),
            ]
        ),

        # ========================
        #  Simple subscriber (python)
        # ========================
        Node(
            package="pupper_apriltag",
            executable="simple_sub",
            name="simple_sub_node",
            output="screen",
        ),
        
        
        # ========================
        #  Fast Tag Pose Node
        # ========================
        Node(
            package="pupper_apriltag",
            executable="fast_tag_pose",
            name="fast_tag_pose_node",
            output="screen",
        ),

        
        
        
        

        # ========================
        #  NEW C++ OPENCV VISUALIZER
        # ========================
        Node(
            condition=IfCondition(show_view),
            package="pupper_apriltag_cpp",
            executable="camera_viewer_cpp",
            name="camera_viewer_cpp_node",
            output="screen",
            remappings=[
                ("/oak/rgb/image_raw", "/oak/rgb/image_raw"),
                ("/apriltag_detections", "/apriltag_detections")
            ]
        ),
    ])
