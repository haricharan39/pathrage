"""
Brings up the full vision pipeline: Tier 1 + Tier 2 perception, costmap
fusion, localization, and the EKF fusion node.

Usage:
    ros2 launch ugv_vision vision.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("ugv_vision")
    ekf_params = os.path.join(pkg_share, "config", "ekf_params.yaml")

    return LaunchDescription([
        Node(package="ugv_vision", executable="depth_obstacle_node", output="screen"),
        Node(package="ugv_vision", executable="segmentation_node", output="screen"),
        Node(package="ugv_vision", executable="costmap_fusion_node", output="screen"),
        Node(package="ugv_vision", executable="localization_node", output="screen"),

        # TODO: launch the external SLAM backend (ORB-SLAM3 or VINS-Fusion)
        # here too, once it's built -- via ExecuteProcess or its own
        # provided launch file, depending on how it's packaged.

        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[ekf_params],
        ),
    ])
