"""ORB-SLAM3 (stereo) wrapper + localization_node.

The wrapper is built outside this repo (scripts/setup_orbslam3.sh) into ~/orbslam_ws, so that
workspace must be sourced in the shell that runs this launch file:

    source /opt/ros/humble/setup.bash
    source ~/orbslam_ws/install/setup.bash
    source ~/pathrage/install/setup.bash
    ros2 launch ugv_vision orbslam.launch.py

Starts:
    orbslam3 stereo        reads  /stereo_camera/{left,right}/image_rect
                           writes /orb_slam3/pose, /orb_slam3/tracking_ok, /orb_slam3/tracking_state
    localization_node      writes /localization/slam_pose (frame odom)

Needs the simulation (stereo_image_proc running). Normally started by
`ros2 launch ugv_navigation full_stack.launch.py orbslam:=true`.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("ugv_vision")
    home = os.path.expanduser("~")
    orb = os.environ.get("ORB_SLAM3_DIR", os.path.join(home, "ORB_SLAM3"))
    lib_path = ":".join(p for p in (
        os.environ.get("LD_LIBRARY_PATH", ""),
        os.path.join(orb, "lib"),
        os.path.join(orb, "Thirdparty", "DBoW2", "lib"),
        os.path.join(orb, "Thirdparty", "g2o", "lib")) if p)

    use_sim_time = LaunchConfiguration("use_sim_time")
    sim_param = {"use_sim_time": use_sim_time}

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("vocabulary",
                              default_value=os.path.join(orb, "Vocabulary", "ORBvoc.txt"),
                              description="ORB-SLAM3 vocabulary file"),
        DeclareLaunchArgument("settings",
                              default_value=os.path.join(pkg_share, "config", "orbslam3_stereo_pinhole.yaml"),
                              description="ORB-SLAM3 settings yaml"),
        DeclareLaunchArgument("left_topic", default_value="/stereo_camera/left/image_rect"),
        DeclareLaunchArgument("right_topic", default_value="/stereo_camera/right/image_rect"),

        Node(
            package="orbslam3", executable="stereo", name="orbslam3_stereo", output="screen",
            # argv: vocabulary, settings, rectify (false: images are already rectified),
            # visualization (false: no Pangolin window)
            arguments=[LaunchConfiguration("vocabulary"), LaunchConfiguration("settings"), "false", "false"],
            remappings=[("camera/left", LaunchConfiguration("left_topic")),
                        ("camera/right", LaunchConfiguration("right_topic"))],
            parameters=[sim_param],
            additional_env={"LD_LIBRARY_PATH": lib_path},
        ),
        Node(
            package="ugv_vision", executable="localization_node", name="localization_node",
            output="screen", parameters=[sim_param],
        ),
    ])
