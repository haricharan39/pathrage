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
| `/orb_slam3/pose` | `geometry_msgs/PoseStamped`, frame `orb_map`, stamp = image pair | ORB-SLAM3 wrapper (patched) | `localization_node` | Phase 1 (code written, not run) |
| `/orb_slam3/tracking_ok` | `std_msgs/Bool` | ORB-SLAM3 wrapper, every frame | viewer, `tools/eval_localization.py` | Phase 1 (code written, not run) |
| `/orb_slam3/tracking_state` | `std_msgs/Int32` (0 no images, 1 not initialised, 2 OK, 3 recently lost, 4 lost) | ORB-SLAM3 wrapper, every frame | `localization_node` | Phase 1 (code written, not run) |
| `/localization/slam_pose` | `geometry_msgs/PoseWithCovarianceStamped`, frame `odom` | `localization_node` | EKF, viewer | Phase 1 (code written, not run) |

`/orb_slam3/pose` is the LEFT OPTICAL camera pose (z forward, x right, y down) in `orb_map`, whose
origin is the first frame (the inverse of ORB-SLAM3's Tcw), published only while tracking is OK.
`localization_node` converts it: `T_orb_base = T_orb_cam * inv(T_base_cam)` with `base_footprint <-
left_camera_optical_frame` from TF, expresses the motion since the first accepted frame in `odom`
(composed with the `odom` -> `base_footprint` pose at that moment), and publishes
`/localization/slam_pose` with the same stamp. Covariance (diagonal): x, y 0.05 m, z 0.10 m,
roll/pitch 0.05 rad, yaw 0.03 rad; multiplied by 10 for 30 messages after a pose jump (more than
0.3 m / 0.3 rad plus 2 m/s / 3 rad/s times the elapsed time) or a new ORB-SLAM3 map. When ORB-SLAM3
starts a new map (state returns to 0 or 1 after it had been OK) its first pose is re-anchored to the
last published pose. Nothing is published while tracking is lost. Metric scale comes from stereo.
Whether the EKF should fuse x, y, yaw absolutely or reject jumps is decided in Phase 2.

### Perception (`ugv_vision`)

| Topic | Type | Publisher | Subscribers | Status |
|---|---|---|---|---|
| `/perception/tier1_obstacles` | `sensor_msgs/PointCloud2`, frame `base_footprint` | `depth_obstacle_node` | `costmap_fusion_node` | Phase 3 |
| `/perception/tier2_terrain` | `sensor_msgs/Image` mono8 (class ids) | `segmentation_node` | `costmap_fusion_node` | Phase 6 |
| `/perception/tier2_confidence` | `sensor_msgs/Image` mono8 (0-255 = softmax max) | `segmentation_node` | `costmap_fusion_node` | Phase 6 |
| `/perception/obstacle_points` | `sensor_msgs/PointCloud2`, frame `base_footprint` | `costmap_fusion_node` | Nav2 local costmap `ObstacleLayer` | Phase 7 |

Until Phase 7, the Nav2 `ObstacleLayer` reads `/perception/tier1_obstacles` directly
(Phase 4). Class ids: 0 unknown, 1 traversable, 2 caution, 3 hazard (finalized in Phase 6).

### 3D viewer (`ugv_viewer`, development tool, nothing depends on it)

| Topic / TF | Direction | Use |
|---|---|---|
| `/points2` | subscribe | accumulated into the voxel map (trusted range 0.8-6 m) |
| `/perception/tier1_obstacles` | subscribe | drawn as the red Tier-1 overlay (Phase 3) |
| `/localization/slam_pose` | subscribe | cyan ORB-SLAM3 trajectory (Phase 1) |
| `/ground_truth/odom` | subscribe | green ground-truth trajectory; display only, rigidly aligned to the fixed frame at the first sample (re-aligned by "Clear paths") |
| `/orb_slam3/tracking_ok` | subscribe | tracking OK / LOST indicator; "position error" = distance between the ORB-SLAM3 and ground-truth points |
| `/plan`, `/goal_pose` | subscribe | plan line and goal marker (Phase 4) |
| `/goal_pose` | **publish, opt-in** | only on double-click in the page with "send goal" ticked; `frame_id` = viewer fixed frame |
| TF `<fixed_frame> <- left_camera_optical_frame, base_footprint` | read | `fixed_frame` defaults to `odom` |

The viewer publishes no TF and writes nothing to the perception topics. Its obstacle colouring is a
height-based preview (`obstacle_thresh` 0.15 m, `drop_thresh` -0.20 m, same defaults as
`depth_obstacle_node`), not the Tier-1 result.

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
| Stereo baseline / focal length | 0.12 m / 381.347 px (= 320 / tan(1.3963 / 2) from the URDF hfov 1.3963 rad, 640x480; cx 320.5, cy 240.5; the 381.36 seen earlier is 80 deg exactly). Confirm against `camera_info` K[0] (P[3] = -45.76) | ORB-SLAM3 yaml (`orbslam3_stereo_pinhole.yaml` is the only one), depth accuracy |
| Trusted stereo range | about 0.8-6 m (error ~0.27 m at 5 m, ~0.70 m at 8 m for 0.5 px disparity error) | Tier 1 `min_range`/`max_range`, RTAB-Map `Grid/RangeMax` |
| Measured rates (Phase 0, 2026-10-11, sim + RTAB-Map running) | `/camera/imu` ~197 Hz; `/points2` ~8-9 Hz (camera is 30 Hz, `stereo_image_proc` on CPU is the bottleneck); `map`->`odom` ~10 Hz; `odom`->`base_footprint` ~50 Hz | Tier 1 can run no faster than `/points2` |
| `/tf` publishers (verified) | `skid_steer_drive` (`odom`->`base_footprint`), `robot_state_publisher`, `rtabmap` (`map`->`odom`) | one owner per transform |
| Robot footprint | 0.60 x 0.40 m | Nav2 footprint and inflation |
| Wheel radius / track / wheel base | 0.10 / 0.50 / 0.40 m | odometry, controller limits |

## 4. Launch entry points

| Command | Starts |
|---|---|
| `ros2 launch ugv_navigation full_stack.launch.py` | Gazebo + robot, stereo depth, RTAB-Map |
| `... full_stack.launch.py vision:=true` | + perception nodes and EKF (`publish_tf` off until Phase 2) |
| `... full_stack.launch.py orbslam:=true` | + ORB-SLAM3 wrapper and `localization_node` (needs `~/orbslam_ws` sourced and a textured world, see README 5.0b) |
| `ros2 launch ugv_vision orbslam.launch.py` | ORB-SLAM3 + `localization_node` only |
| `... full_stack.launch.py nav:=true` | + Nav2 (needs the Phase 4 parameter file) |
| `... full_stack.launch.py viewer:=true` | + browser 3D viewer at http://127.0.0.1:8080 |
| `... full_stack.launch.py world:=offroad_forest` | any world in `worlds/` (name or full path), see docs/WORLDS.md |
| `ros2 launch ugv_viewer viewer.launch.py` | 3D viewer only |
| `ros2 launch ugv_description gazebo.launch.py` | Gazebo + robot only |
| `ros2 launch ugv_description display.launch.py` | URDF in RViz with joint sliders (no Gazebo) |
