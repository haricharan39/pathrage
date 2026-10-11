"""gazebo.launch.py - Spawn the VisionNav UGV in Gazebo Classic 11 (ROS 2 Humble).

Starts gzserver/gzclient, robot_state_publisher (sim time) and spawns the robot.
Wheel control (/cmd_vel), /odom, /joint_states and the stereo camera topics are
provided by the Gazebo plugins declared in the Xacro.

Usage:
  ros2 launch ugv_description gazebo.launch.py
  ros2 launch ugv_description gazebo.launch.py gui:=false x:=2.0 yaw:=1.57
  ros2 launch ugv_description gazebo.launch.py world:=offroad_forest      # a name, or a full path
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, EnvironmentVariable, FindExecutable,
                                  LaunchConfiguration)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('ugv_description')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    xacro_file = os.path.join(pkg_share, 'urdf', 'visionnav.urdf.xacro')
    default_world = os.path.join(pkg_share, 'worlds', 'outdoor_offroad.world')  # your world (ground_plane + sun + rocks)

    # Lets worlds use model://terrain_textured (and any model you add under models/).
    # Existing GAZEBO_MODEL_PATH entries are kept.
    models_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        [os.path.join(pkg_share, 'models'), ':',
         EnvironmentVariable('GAZEBO_MODEL_PATH', default_value='')])

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

    def _include_gazebo(context):
        """Accepts a full path or just a name ('offroad_forest' / 'offroad_forest.world')."""
        w = world.perform(context)
        if os.sep not in w:
            cand = os.path.join(pkg_share, 'worlds', w if w.endswith('.world') else w + '.world')
            if os.path.isfile(cand):
                w = cand
        if not os.path.isfile(w):
            raise RuntimeError(
                f"World file not found: {w}. Use a full path or one of: "
                + ', '.join(sorted(f for f in os.listdir(os.path.join(pkg_share, 'worlds'))
                                   if f.endswith('.world'))))
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': w, 'gui': gui, 'verbose': 'false'}.items())]

    gazebo = OpaqueFunction(function=_include_gazebo)

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
                              description='World name (e.g. offroad_forest) or full path to a .world file'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Start gzclient'),
        DeclareLaunchArgument('camera_baseline', default_value='0.12',
                              description='Stereo baseline in metres'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.05'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        models_path,
        gazebo,
        robot_state_publisher,
        spawn_robot,
    ])
