"""gazebo.launch.py - Spawn the VisionNav UGV in Gazebo Classic 11 (ROS 2 Humble).

Starts gzserver/gzclient, robot_state_publisher (sim time) and spawns the robot.
Wheel control (/cmd_vel), /odom, /joint_states and the stereo camera topics are
provided by the Gazebo plugins declared in the Xacro.

Usage:
  ros2 launch ugv_description gazebo.launch.py
  ros2 launch ugv_description gazebo.launch.py gui:=false x:=2.0 yaw:=1.57
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('ugv_description')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    xacro_file = os.path.join(pkg_share, 'urdf', 'visionnav.urdf.xacro')
    default_world = os.path.join(pkg_share, 'worlds', 'outdoor_offroad.world')  # your world (ground_plane + sun + rocks)

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    camera_baseline = LaunchConfiguration('camera_baseline')

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', xacro_file,
            ' camera_baseline:=', camera_baseline,
        ]),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': world,
            'gui': gui,
            'verbose': 'false',
        }.items(),
    )

    # Publishes all fixed TFs and the wheel TFs (from /joint_states published by Gazebo).
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # Spawning base_footprint a few cm above ground lets the wheels settle without penetration.
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_visionnav',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'visionnav',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world,
                              description='Gazebo world file'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Start gzclient'),
        DeclareLaunchArgument('camera_baseline', default_value='0.12',
                              description='Stereo baseline in metres'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.05'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        gazebo,
        robot_state_publisher,
        spawn_robot,
    ])
