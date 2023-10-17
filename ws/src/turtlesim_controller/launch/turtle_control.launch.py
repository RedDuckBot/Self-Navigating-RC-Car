from launch import LaunchDescription 
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="turtlesim",
            executable="turtlesim_node",
            name="sim_node"
        ),
        Node(
            package="joy",
            executable="joy_node",
            name="joy_controller_node",
        ),
        Node(
            package="turtlesim_controller",
            executable="turtlesim_control_node",
            name="turtle_control_node",
        ),
    ])