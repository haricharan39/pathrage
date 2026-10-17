"""
Single entry point for the whole stack.

Always starts:   Gazebo + robot (ugv_description)
Starts by default: stereo depth (stereo_image_proc), RTAB-Map mapping and the localization layer
                 (EKF + status node; the EKF owns the odom -> base_footprint transform since Phase 2)
Optional layers: vision:=true (perception nodes), orbslam:=true (ORB-SLAM3 pose),
                 odom_noise:=true (seeded errors in the wheel odometry, honest fusion test),
                 nav:=true (Nav2),
                 viewer:=true (browser 3D viewer at http://127.0.0.1:8080, replaces RViz2 for 3D)

Usage:
    ros2 launch ugv_navigation full_stack.launch.py
    ros2 launch ugv_navigation full_stack.launch.py gui:=true
    ros2 launch ugv_navigation full_stack.launch.py slam:=false vision:=true
    ros2 launch ugv_navigation full_stack.launch.py viewer:=true
    ros2 launch ugv_navigation full_stack.launch.py orbslam:=true viewer:=true world:=offroad_sparse
    ros2 launch ugv_navigation full_stack.launch.py orbslam:=true odom_noise:=true noise_seed:=7

orbslam:=true needs the ORB-SLAM3 workspace sourced first (source ~/orbslam_ws/install/setup.bash,
see README 4.4) and a textured world (offroad_sparse or offroad_forest).

State: `vision` is off by default because the perception nodes are interface-only until
Phases 3/6/7, and `nav` is off by default because the Nav2 parameter file is a stub until
Phase 4. Turning `nav` on now will fail. `localization:=false` leaves the TF tree without an
odom -> base_footprint transform (the URDF no longer publishes it), so only use it if you run
the EKF yourself.

Spawn pose / viewer options: x:=2.0 y:=-1.0 yaw:=1.57 voxel_size:=0.08 open_browser:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, LogInfo, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitution import Substitution


def _include(package, *path, **launch_arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), *path)),
        launch_arguments={k: (v if isinstance(v, Substitution) else str(v))
                          for k, v in launch_arguments.items()}.items(),
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
        launch_arguments={"gui": gui, "world": world, "x": LaunchConfiguration("x"),
                          "y": LaunchConfiguration("y"), "yaw": LaunchConfiguration("yaw")}.items(),
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

    viewer = TimerAction(
        period=delay,
        actions=[GroupAction(
            [_include("ugv_viewer", "launch", "viewer.launch.py",
                      voxel_size=LaunchConfiguration("voxel_size"),
                      open_browser=LaunchConfiguration("open_browser"),
                      host=LaunchConfiguration("host"))],
            condition=IfCondition(LaunchConfiguration("viewer")))],
    )

    orbslam = TimerAction(
        period=delay,
        actions=[GroupAction(
            [_include("ugv_vision", "launch", "orbslam.launch.py", use_sim_time="true")],
            condition=IfCondition(LaunchConfiguration("orbslam")))],
    )

    # EKF + status (+ noisy odometry): owns odom -> base_footprint, so it starts with the mapping.
    localization = TimerAction(
        period=delay,
        actions=[GroupAction(
            [_include("ugv_vision", "launch", "localization.launch.py", use_sim_time="true",
                      odom_noise=LaunchConfiguration("odom_noise"),
                      noise_seed=LaunchConfiguration("noise_seed"))],
            condition=IfCondition(LaunchConfiguration("localization")))],
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
                              description="Start the perception nodes (interface-only until Phases 3/6/7)"),
        DeclareLaunchArgument("localization", default_value="true",
                              description="Start the EKF (owns odom -> base_footprint) and the status node"),
        DeclareLaunchArgument("odom_noise", default_value="false",
                              description="Inject seeded errors into the wheel odometry (EKF reads /odom_noisy)"),
        DeclareLaunchArgument("noise_seed", default_value="42",
                              description="Seed of the injected systematic odometry errors"),
        DeclareLaunchArgument("orbslam", default_value="false",
                              description="Start ORB-SLAM3 + localization_node (needs the slam stereo chain)"),
        DeclareLaunchArgument("viewer", default_value="false",
                              description="Start the browser 3D viewer (ugv_viewer)"),
        DeclareLaunchArgument("x", default_value="0.0", description="Robot spawn x (m)"),
        DeclareLaunchArgument("y", default_value="0.0", description="Robot spawn y (m)"),
        DeclareLaunchArgument("yaw", default_value="0.0", description="Robot spawn heading (rad)"),
        DeclareLaunchArgument("voxel_size", default_value="0.05",
                              description="Viewer map resolution in m (0.08-0.10 if the browser lags)"),
        DeclareLaunchArgument("open_browser", default_value="false", description="Open the viewer page"),
        DeclareLaunchArgument("host", default_value="127.0.0.1",
                              description="Viewer bind address (0.0.0.0 for other machines)"),
        DeclareLaunchArgument("nav", default_value="false",
                              description="Start Nav2 (needs Phase 4 parameters)"),
        DeclareLaunchArgument("startup_delay", default_value="8.0",
                              description="Seconds to wait for Gazebo before the rest starts"),
        sim,
        perception_chain,
        localization,
        vision,
        orbslam,
        viewer,
        navigation,
    ])
