# PathRage / VisionNav - interface contract

Single source of truth for every topic, frame and transform owner. Code, launch files and
configs must match this file. Change it first, then the code.

Conventions: ROS 2 Humble, Gazebo Classic 11, REP-103 frames (x forward, y left, z up).
Simulation clock everywhere (`use_sim_time: true`).

Status column: **Done** = produced by working code today, **Phase N** = interface fixed,
implementation arrives in that phase of the execution plan.

## 1. Topics

### Robot and sensors

| Topic | Type | Publisher | Subscribers | Status |
|---|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | teleop / Nav2 | Gazebo diff-drive plugin | Done |
| `/odom` | `nav_msgs/Odometry` | Gazebo diff-drive plugin (`odom` -> `base_footprint`) | RTAB-Map, EKF (velocities only) | Done |
| `/joint_states` | `sensor_msgs/JointState` | Gazebo joint-state plugin | `robot_state_publisher` | Done |
| `/camera/imu` | `sensor_msgs/Imu` | URDF IMU sensor, frame `stereo_camera_link`, ~200 Hz | EKF (angular velocity only) | Done |
| `/ground_truth/odom` | `nav_msgs/Odometry` | URDF p3d plugin, `world` -> `base_link`, 50 Hz | evaluation scripts ONLY | Done (verify in Gate 0) |
| `/stereo_camera/left/image_raw`, `/stereo_camera/right/image_raw` | `sensor_msgs/Image` | Gazebo cameras, 640x480, 30 Hz | `stereo_image_proc` | Done |
| `/stereo_camera/left/camera_info`, `/stereo_camera/right/camera_info` | `sensor_msgs/CameraInfo` | Gazebo cameras | `stereo_image_proc`, RTAB-Map | Done |

### Stereo depth (`stereo_image_proc`)

| Topic | Type | Subscribers | Status |
|---|---|---|---|
| `/stereo_camera/left/image_rect`, `/stereo_camera/right/image_rect` | `sensor_msgs/Image` (mono) | RTAB-Map, ORB-SLAM3 | Done |
| `/stereo_camera/left/image_rect_color` | `sensor_msgs/Image` (rgb8) | `segmentation_node` | Done |
| `/disparity` | `stereo_msgs/DisparityImage` | - | Done |
| `/points2` | `sensor_msgs/PointCloud2` | `depth_obstacle_node` | Done |

### Mapping and localization

| Topic | Type | Publisher | Subscribers | Status |
|---|---|---|---|---|
| `/map`, `/mapData`, `/mapGraph`, `/info` | RTAB-Map outputs | RTAB-Map | Nav2 global costmap | Done |
| `/orb_slam3/pose` (exact topic and type confirmed in Phase 1) | wrapper-defined | ORB-SLAM3 ROS 2 wrapper | `localization_node` | Phase 1 |
| `/localization/slam_pose` | `geometry_msgs/PoseWithCovarianceStamped` | `localization_node` | EKF | Phase 1 |

### Perception (`ugv_vision`)

| Topic | Type | Publisher | Subscribers | Status |
|---|---|---|---|---|
| `/perception/tier1_obstacles` | `sensor_msgs/PointCloud2`, frame `base_footprint` | `depth_obstacle_node` | `costmap_fusion_node` | Phase 3 |
| `/perception/tier2_terrain` | `sensor_msgs/Image` mono8 (class ids) | `segmentation_node` | `costmap_fusion_node` | Phase 6 |
| `/perception/tier2_confidence` | `sensor_msgs/Image` mono8 (0-255 = softmax max) | `segmentation_node` | `costmap_fusion_node` | Phase 6 |
| `/perception/obstacle_points` | `sensor_msgs/PointCloud2`, frame `base_footprint` | `costmap_fusion_node` | Nav2 local costmap `ObstacleLayer` | Phase 7 |

Until Phase 7, the Nav2 `ObstacleLayer` reads `/perception/tier1_obstacles` directly
(Phase 4). Class ids: 0 unknown, 1 traversable, 2 caution, 3 hazard (finalized in Phase 6).

## 2. Frames and transforms (one publisher per transform)

```
map -> odom -> base_footprint -> base_link
                                   |- front_left / front_right / rear_left / rear_right wheel links
                                   '- stereo_camera_link  (also the IMU frame)
                                        |- left_camera_frame  -> left_camera_optical_frame
                                        '- right_camera_frame -> right_camera_optical_frame
```

| Transform | Owner | Notes |
|---|---|---|
| `map` -> `odom` | RTAB-Map | |
| `odom` -> `base_footprint` | **Phase 0-1:** Gazebo diff-drive plugin. **From Phase 2:** the EKF (`ekf_filter_node`) | Phase 2 switches both together: URDF `publish_odom_tf` -> false and `ekf_params.yaml` `publish_tf` -> true. Never both. |
| `base_footprint` -> `base_link` -> body/wheel/camera links | `robot_state_publisher` | from the URDF and `/joint_states` |

The ground-truth frame `world` is Gazebo's world frame. It is not in the TF tree and is
used only by evaluation scripts.

`base_footprint` is the base frame used by every node and config (the EKF's
`base_link_frame` is `base_footprint`).

## 3. Numbers other components depend on

| Item | Value | Where it matters |
|---|---|---|
| Stereo baseline / focal length | 0.12 m / 381.36 px (80 deg HFOV, 640x480) | ORB-SLAM3 yaml, depth accuracy |
| Trusted stereo range | about 0.8-6 m (error ~0.27 m at 5 m, ~0.70 m at 8 m for 0.5 px disparity error) | Tier 1 `min_range`/`max_range`, RTAB-Map `Grid/RangeMax` |
| Robot footprint | 0.60 x 0.40 m | Nav2 footprint and inflation |
| Wheel radius / track / wheel base | 0.10 / 0.50 / 0.40 m | odometry, controller limits |

## 4. Launch entry points

| Command | Starts |
|---|---|
| `ros2 launch ugv_navigation full_stack.launch.py` | Gazebo + robot, stereo depth, RTAB-Map |
| `... full_stack.launch.py vision:=true` | + perception nodes and EKF (`publish_tf` off until Phase 2) |
| `... full_stack.launch.py nav:=true` | + Nav2 (needs the Phase 4 parameter file) |
| `ros2 launch ugv_description gazebo.launch.py` | Gazebo + robot only |
| `ros2 launch ugv_description display.launch.py` | URDF in RViz with joint sliders (no Gazebo) |
