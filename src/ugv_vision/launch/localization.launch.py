"""Fused localization (Phase 2): EKF, optional noisy wheel odometry, health status.

Starts:
    ekf_filter_node            robot_localization EKF. Publishes /odometry/filtered and OWNS the
                               transform odom -> base_footprint (the URDF's publish_odom_tf is false).
                               Inputs: wheel odometry (vx, vyaw), /localization/slam_pose (x, y, yaw),
                               /camera/imu (angular velocity). See config/ekf_params.yaml.
    odom_noise_node            only with odom_noise:=true: /odom -> /odom_noisy with seeded errors.
                               The EKF then reads /odom_noisy instead of /odom.
    localization_status_node   /localization/status: "fused" or "degraded" (ORB-SLAM3 lost).

Because the EKF owns odom -> base_footprint, something must run this file whenever the robot is
simulated, or the TF tree has no odometry link. full_stack.launch.py does it by default
(localization:=true).

Usage:
    ros2 launch ugv_vision localization.launch.py
    ros2 launch ugv_vision localization.launch.py odom_noise:=true noise_seed:=7
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory("ugv_vision"), "config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    odom_noise = LaunchConfiguration("odom_noise")
    sim_param = {"use_sim_time": use_sim_time}

    # Wheel odometry the EKF listens to: the noisy copy when the noise node runs, else the clean one.
    odom_topic = PythonExpression(
        ["'/odom_noisy' if '", odom_noise, "'.lower() == 'true' else '/odom'"])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use the Gazebo clock"),
        DeclareLaunchArgument("odom_noise", default_value="false",
                              description="Inject seeded errors into the wheel odometry (honest fusion test)"),
        DeclareLaunchArgument("noise_seed", default_value="42",
                              description="Seed of the injected systematic errors"),

        Node(
            package="ugv_vision", executable="odom_noise_node", name="odom_noise_node",
            output="screen",
            parameters=[os.path.join(cfg, "odom_noise.yaml"), sim_param,
                        {"seed": ParameterValue(LaunchConfiguration("noise_seed"), value_type=int)}],
            condition=IfCondition(odom_noise),
        ),
        Node(
            package="robot_localization", executable="ekf_node", name="ekf_filter_node",
            output="screen",
            parameters=[os.path.join(cfg, "ekf_params.yaml"), sim_param,
                        {"odom0": ParameterValue(odom_topic, value_type=str)}],
        ),
        Node(
            package="ugv_vision", executable="localization_status_node", name="localization_status_node",
            output="screen", parameters=[sim_param],
        ),
    ])
