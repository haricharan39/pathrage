"""Start the browser-based 3D viewer.

Usage:
    ros2 launch ugv_viewer viewer.launch.py
    ros2 launch ugv_viewer viewer.launch.py open_browser:=true fixed_frame:=map
    ros2 launch ugv_viewer viewer.launch.py host:=0.0.0.0 port:=8090      # view from another PC

Then open http://127.0.0.1:8080
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _p(name, kind):
    return ParameterValue(LC(name), value_type=kind)


def generate_launch_description():
    args = [
        ("fixed_frame", "odom", "Frame the map is accumulated in (odom always exists; map needs RTAB-Map)"),
        ("host", "127.0.0.1", "Bind address (0.0.0.0 to allow other machines)"),
        ("port", "8080", "HTTP/WebSocket port"),
        ("open_browser", "false", "Open the page in the default browser"),
        ("voxel_size", "0.05", "Map resolution in metres (raise to 0.08-0.10 if the browser lags)"),
        ("max_points", "1500000", "Maximum number of map voxels"),
        ("use_sim_time", "true", "Use the Gazebo clock"),
    ]
    decl = [DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in args]
    node = Node(
        package="ugv_viewer", executable="viewer_node", name="viewer_node", output="screen",
        parameters=[{
            "use_sim_time": _p("use_sim_time", bool),
            "fixed_frame": _p("fixed_frame", str),
            "host": _p("host", str),
            "port": _p("port", int),
            "open_browser": _p("open_browser", bool),
            "voxel_size": _p("voxel_size", float),
            "max_points": _p("max_points", int),
        }],
    )
    return LaunchDescription(decl + [node])
