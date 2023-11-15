from launch import LaunchDescription 
from launch_ros.actions import Node

def generate_launch_description():
        return LaunchDescription([
        Node(
            package="manual_control",
            executable="odom_node",
            name="odom_node"
        ),
        Node(
            package="manual_control",
            executable="arduino_node",
            name="arduino_node",
        ),
    ])