"""
Single entry point for the whole stack.

Always starts:   Gazebo + robot (ugv_description)
Starts by default: stereo depth (stereo_image_proc) and RTAB-Map mapping
Optional layers: vision:=true (perception nodes + EKF), nav:=true (Nav2)

Usage:
    ros2 launch ugv_navigation full_stack.launch.py
    ros2 launch ugv_navigation full_stack.launch.py gui:=true
    ros2 launch ugv_navigation full_stack.launch.py slam:=false vision:=true

Phase 0 state: `vision` is off by default because the perception nodes are
interface-only until Phases 3/6/7, and `nav` is off by default because the Nav2
parameter file is a stub until Phase 4. Turning `nav` on now will fail.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, LogInfo, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(package, *path, **launch_arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), *path)),
        launch_arguments={k: str(v) for k, v in launch_arguments.items()}.items(),
    )


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    world = LaunchConfiguration("world")
    delay = LaunchConfiguration("startup_delay")

    description_dir = get_package_share_directory("ugv_description")
    default_world = os.path.join(description_dir, "worlds", "outdoor_offroad.world")

    # Robot + Gazebo (gazebo.launch.py also runs robot_state_publisher and spawns the UGV).
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_dir, "launch", "gazebo.launch.py")),
        launch_arguments={"gui": gui, "world": world}.items(),
    )

    # Stereo depth: rectified images, /disparity, /points2.
    stereo_depth = _include(
        "stereo_image_proc", "launch", "stereo_image_proc.launch.py",
        left_namespace="/stereo_camera/left",
        right_namespace="/stereo_camera/right",
        approximate_sync="true",
        use_sim_time="true",
    )

    # RTAB-Map: /map and map -> odom.
    mapping = _include("ugv_vision", "launch", "slam.launch.py")

    # Wait for Gazebo and the camera topics before starting the perception chain.
    perception_chain = TimerAction(
        period=delay,
        actions=[
            GroupAction([stereo_depth], condition=IfCondition(LaunchConfiguration("slam"))),
            GroupAction([mapping], condition=IfCondition(LaunchConfiguration("slam"))),
        ],
    )

    vision = TimerAction(
        period=delay,
        actions=[GroupAction(
            [_include("ugv_vision", "launch", "vision.launch.py", use_sim_time="true")],
            condition=IfCondition(LaunchConfiguration("vision")))],
    )

    navigation = TimerAction(
        period=delay,
        actions=[GroupAction(
            [LogInfo(msg="nav:=true: Nav2 parameters are a stub until Phase 4; this will fail."),
             _include("ugv_navigation", "launch", "navigation.launch.py")],
            condition=IfCondition(LaunchConfiguration("nav")))],
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false",
                              description="Show the Gazebo window"),
        DeclareLaunchArgument("world", default_value=default_world,
                              description="Gazebo world file"),
        DeclareLaunchArgument("slam", default_value="true",
                              description="Start stereo_image_proc and RTAB-Map"),
        DeclareLaunchArgument("vision", default_value="false",
                              description="Start perception nodes and the EKF"),
        DeclareLaunchArgument("nav", default_value="false",
                              description="Start Nav2 (needs Phase 4 parameters)"),
        DeclareLaunchArgument("startup_delay", default_value="8.0",
                              description="Seconds to wait for Gazebo before the rest starts"),
        sim,
        perception_chain,
        vision,
        navigation,
    ])
