"""
Brings up the ugv_vision perception nodes: Tier 1, Tier 2 and costmap fusion.
The ORB-SLAM3 wrapper and its glue node (localization_node) are in orbslam.launch.py; the EKF,
the noisy-odometry test node and the localization status node are in localization.launch.py.

Usage:
    ros2 launch ugv_vision vision.launch.py
    ros2 launch ugv_vision vision.launch.py use_sim_time:=false

Note: the perception and fusion nodes are interface-only until Phases 3, 6 and 7 (they log a
warning and publish nothing).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
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
    ])
