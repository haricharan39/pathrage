"""
Single entry point: sim + vision pipeline + navigation, all together.

Usage:
    ros2 launch ugv_navigation full_stack.launch.py
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    description_dir = get_package_share_directory("ugv_description")
    vision_dir = get_package_share_directory("ugv_vision")
    navigation_dir = get_package_share_directory("ugv_navigation")

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_dir, "launch", "sim_bringup.launch.py")
        )
    )
    vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vision_dir, "launch", "vision.launch.py")
        )
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_dir, "launch", "navigation.launch.py")
        )
    )

    return LaunchDescription([sim, vision, navigation])
