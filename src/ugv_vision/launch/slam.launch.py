from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    p = {
        'frame_id': 'base_footprint',
        'odom_frame_id': 'odom',
        'use_sim_time': True,
        'subscribe_stereo': True,
        'subscribe_depth': False,
        'approx_sync': True,
        'Reg/Force3DoF': 'true',
        'Grid/Sensor': '1',
        'Grid/3D': 'false',
        'Grid/CellSize': '0.05',
        'Grid/RangeMax': '5.0',
        'Grid/MaxObstacleHeight': '1.0',
        'Grid/RayTracing': 'true',
    }
    remaps = [
        ('left/image_rect',  '/stereo_camera/left/image_rect'),
        ('right/image_rect', '/stereo_camera/right/image_rect'),
        ('left/camera_info',  '/stereo_camera/left/camera_info'),
        ('right/camera_info', '/stereo_camera/right/camera_info'),
        ('odom', '/odom'),
    ]
    return LaunchDescription([
        Node(package='rtabmap_slam', executable='rtabmap', name='rtabmap',
             output='screen', arguments=['-d'],
             parameters=[p], remappings=remaps),
    ])
