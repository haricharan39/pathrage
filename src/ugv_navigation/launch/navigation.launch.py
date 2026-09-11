"""
Brings up Nav2 (planner_server, controller_server, costmaps) configured
with this project's params.

Usage:
    ros2 launch ugv_navigation navigation.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    pkg_share = get_package_share_directory("ugv_navigation")
    params_file = os.path.join(pkg_share, "config", "nav2_params.yaml")

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={"params_file": params_file}.items(),
    )

    return LaunchDescription([nav2])
