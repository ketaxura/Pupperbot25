How to install and make the april tag and oak d camera code work:

sudo apt update
sudo apt install ros-humble-depthai-ros-driver
sudo apt install ros-humble-apriltag-ros
sudo apt install ros-humble-image-transport
sudo apt install ros-humble-cv-bridge
sudo apt install ros-humble-rviz2

Then unpack the src folder and colcon build and source

Then run

ros2 launch pupper_apriltag complete.launch.py show_view:=true

The launch argument show_view, turns the camera visualization on and off. In the visualization it shows the bounding box.

When you run this launch file, 

Topics such as:
