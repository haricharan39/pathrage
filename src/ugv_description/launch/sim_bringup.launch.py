"""
Spawns the UGV model into the off-road Gazebo world and starts
robot_state_publisher, so the camera/IMU/wheel-odometry topics used by
ugv_vision and ugv_navigation are available.

Usage:
    ros2 launch ugv_description sim_bringup.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("ugv_description")
    world_path = os.path.join(pkg_share, "worlds", "outdoor_offroad.world")
    xacro_path = os.path.join(pkg_share, "urdf", "ugv.urdf.xacro")

    # TODO: swap this for the correct gazebo_ros launch include once the
    # target Gazebo version (Classic vs Fortress/Ignition) is finalized.
    gazebo = ExecuteProcess(
        cmd=["gazebo", "--verbose", world_path, "-s", "libgazebo_ros_factory.so"],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": f"$(xacro {xacro_path})"}],
        # NOTE: in practice, run xacro at launch-build time (via
        # Command(['xacro ', xacro_path]) with launch.substitutions) rather
        # than the raw string above -- left as a TODO for the first real pass.
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "ugv"],
        output="screen",
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
    ])
