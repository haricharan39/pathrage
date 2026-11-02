# ugv_description - Phase 0 (robot + world + verification)

4-wheel skid-steer UGV with one stereo camera (640x480, 30 FPS, 0.12 m baseline) and an IMU on the
camera bracket. ROS 2 Humble, Gazebo Classic 11, REP-103 frames.

## Contents
| Path | What |
|---|---|
| `urdf/visionnav.urdf.xacro` | Robot: chassis, 4 wheels, stereo camera, IMU, skid-steer drive, joint states, ground truth |
| `worlds/outdoor_offroad.world` | Phase 0 default: textured ground, 3 rocks, goal marker at (10, 0) |
| `worlds/offroad_sparse.world`, `offroad_forest.world` | Denser worlds for later phases |
| `models/terrain_textured/` | 120 m textured ground (gives stereo + ORB-SLAM3 features) |
| `launch/display.launch.py` | Robot in RViz2 only (no Gazebo) |
| `launch/gazebo.launch.py` | Gazebo + spawn robot + robot_state_publisher |
| `scripts/verify_phase0.py` | Gate 0 automatic check |

## Build
```bash
cd ~/pathrage_ws            # your workspace root
cp -r <this>/ugv_description src/
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select ugv_description --symlink-install
source install/setup.bash
```

## Run
```bash
# 1. Model only (check wheel/camera placement, TF)
ros2 launch ugv_description display.launch.py

# 2. Simulation
ros2 launch ugv_description gazebo.launch.py
#    options: gui:=false  world:=offroad_sparse  x:=2.0 y:=0.0 yaw:=1.57

# 3. Drive (second terminal)
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 4. Gate 0 check (third terminal, sim running)
ros2 run ugv_description verify_phase0.py            # add --no-drive to skip the driving test
```

## Topics (Gate 0)
`/cmd_vel` (in) | `/odom` | `/joint_states` | `/camera/imu` (200 Hz) | `/ground_truth/odom` (evaluation only)
`/stereo_camera/{left,right}/image_raw`, `/stereo_camera/{left,right}/camera_info`

## TF tree
```
base_footprint
  base_link
    front_left_wheel_link  front_right_wheel_link  rear_left_wheel_link  rear_right_wheel_link
    stereo_camera_link
      left_camera_frame  -> left_camera_optical_frame
      right_camera_frame -> right_camera_optical_frame
```
`odom -> base_footprint` comes from the Gazebo plugin in Phase 0/1 (`publish_odom_tf:=true`, the default).
From Phase 2 launch with `publish_odom_tf:=false` so the EKF owns it. Never both.

## Tunable xacro arguments
`chassis_*`, `wheel_*`, `track_width`, `wheel_base`, `camera_baseline`, `camera_x`, `camera_z`,
`camera_width/height/hfov/fps`, `publish_odom_tf`. Example:
`ros2 launch ugv_description gazebo.launch.py camera_baseline:=0.20`
(then run `verify_phase0.py --baseline 0.20`).

## Troubleshooting
- Robot sinks / jitters: check `wheel_mass`, `chassis_mass`, `kp/kd` in the xacro wheel macro.
- Black or grey ground: `GAZEBO_MODEL_PATH` not set - always start via `gazebo.launch.py`.
- Gazebo hangs on first start: it downloads `sun`; wait ~1 minute once, or run `gazebo` alone first.
- `verify_phase0.py` says no /clock: Gazebo is paused or not running.
