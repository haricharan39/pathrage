"""
Brings up the ugv_vision nodes: Tier 1 + Tier 2 perception, fusion and the EKF.
The ORB-SLAM3 wrapper and its glue node (localization_node) are in orbslam.launch.py.

Usage:
    ros2 launch ugv_vision vision.launch.py
    ros2 launch ugv_vision vision.launch.py use_sim_time:=false

Note: in Phase 0 the perception and fusion nodes are interface-only (they log a
warning and publish nothing); they are implemented in Phases 3, 6 and 7. The EKF has
publish_tf disabled until Phase 2.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("ugv_vision")
    ekf_params = os.path.join(pkg_share, "config", "ekf_params.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")
    sim_param = {"use_sim_time": use_sim_time}

    def vision_node(executable):
        return Node(package="ugv_vision", executable=executable,
                    output="screen", parameters=[sim_param])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="Use the Gazebo clock"),

        vision_node("depth_obstacle_node"),
        vision_node("segmentation_node"),
        vision_node("costmap_fusion_node"),

        # ORB-SLAM3 + localization_node: orbslam.launch.py (full_stack.launch.py orbslam:=true).
        # The EKF reads /localization/slam_pose from there; without it, it only has wheel + gyro.

        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[ekf_params, sim_param],
        ),
    ])
