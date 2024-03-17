from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python import get_package_share_directory 

def generate_launch_description():
    ld = LaunchDescription()


    lidar_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("sllidar_ros2"),
            "launch/sllidar_a1_launch.py")
        ),
        launch_arguments={
            "serial_port": "/dev/rplidar",
            "frame_id": "laser_frame"
        }.items()
    )
    drive_node = Node(
        package="mac_drive",
        executable="drive"
    )

    odom_node = Node(
        package="mac_drive",
        executable="odom"
    )

    ld.add_action(drive_node)
    ld.add_action(odom_node)
    ld.add_action(lidar_launch_file)

    return ld
