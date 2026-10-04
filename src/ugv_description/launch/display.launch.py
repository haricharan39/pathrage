"""display.launch.py - View the VisionNav UGV in RViz2 (no Gazebo).

Starts robot_state_publisher, joint_state_publisher_gui (optional) and RViz2.
Usage:
  ros2 launch ugv_description display.launch.py
  ros2 launch ugv_description display.launch.py camera_baseline:=0.20 gui:=false
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

RVIZ_CONFIG = """\
Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Enabled: true
      Cell Size: 0.5
      Plane Cell Count: 20
    - Class: rviz_default_plugins/RobotModel
      Name: RobotModel
      Enabled: true
      Description Topic:
        Depth: 5
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /robot_description
    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: true
      Show Names: true
      Marker Scale: 0.2
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: base_footprint
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/MoveCamera
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 2.5
      Focal Point:
        X: 0.0
        Y: 0.0
        Z: 0.2
      Pitch: 0.5
      Yaw: 0.8
      Name: Current View
      Target Frame: base_footprint
Window Geometry:
  Height: 800
  Width: 1200
"""


def generate_launch_description():
    pkg_share = get_package_share_directory('ugv_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'visionnav.urdf.xacro')

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    camera_baseline = LaunchConfiguration('camera_baseline')

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', xacro_file,
            ' camera_baseline:=', camera_baseline,
        ]),
        value_type=str,
    )

    # Minimal RViz config written to a temp file (keeps the package to the 3 required files)
    rviz_cfg = tempfile.NamedTemporaryFile(
        mode='w', suffix='.rviz', prefix='visionnav_', delete=False)
    rviz_cfg.write(RVIZ_CONFIG)
    rviz_cfg.close()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use /clock (set true when running alongside Gazebo)'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Start joint_state_publisher_gui to move the wheel joints'),
        DeclareLaunchArgument(
            'camera_baseline', default_value='0.12',
            description='Stereo baseline in metres'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(gui),
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_cfg.name],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
