# PATHRAGE – VisionNav (SIH 2026, PS 26126)

**Autonomous Navigation in GPS-Denied Outdoor Environments**

## Step 1: UGV model (URDF/Xacro), teleoperation in Gazebo, visualization in RViz

The first step of the project builds the simulation foundation that every later stage (stereo depth, visual SLAM, semantic segmentation, Nav2) runs on:

1. Create a realistic URDF/Xacro model of the UGV.
2. Spawn it in Gazebo Classic and drive it manually (teleoperation).
3. Visualize the robot, TF tree and stereo camera streams in RViz2.

**Stack:** Ubuntu 22.04 · ROS 2 Humble · Gazebo Classic 11 · RViz2 · URDF/Xacro

---

## 1. The robot

A four-wheel-drive skid-steer UGV with one forward-facing stereo camera and no other sensors.

| Item | Value (all configurable in the Xacro) |
|---|---|
| Chassis | 0.60 × 0.40 × 0.15 m box, 12 kg |
| Ground clearance | 0.06 m |
| Wheels | 4 × cylinders, radius 0.10 m, width 0.06 m, 1.2 kg each |
| Track width / wheel base | 0.50 m / 0.40 m |
| Drive | 4 actuated continuous joints, skid-steer via `libgazebo_ros_diff_drive` (2 wheel pairs) |
| Electronics enclosure | Compact box on the chassis (part of `base_link`) |
| Sensor bracket | Rigid post plus camera bar |
| Stereo camera | 2 × Gazebo camera sensors, 0.12 m baseline, 640×480, 80° HFOV, 30 Hz |

Wheel joints: `front_left_wheel_joint`, `front_right_wheel_joint`, `rear_left_wheel_joint`, `rear_right_wheel_joint`.

### TF tree

```
odom                                  (from the diff-drive plugin)
└── base_footprint
    └── base_link
        ├── front_left_wheel_link
        ├── front_right_wheel_link
        ├── rear_left_wheel_link
        ├── rear_right_wheel_link
        └── stereo_camera_link
            ├── left_camera_frame
            │   └── left_camera_optical_frame
            └── right_camera_frame
                └── right_camera_optical_frame
```

Frames follow REP-103 (x forward, y left, z up). Optical frames use z forward, x right, y down. Each transform has a single publisher: `robot_state_publisher` publishes the fixed and wheel transforms, and the Gazebo plugin publishes `odom → base_footprint`.

### Topics

| Topic | Type | Purpose |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Drive command (input) |
| `/odom` | `nav_msgs/Odometry` | Wheel odometry |
| `/joint_states` | `sensor_msgs/JointState` | Wheel joint states |
| `/stereo_camera/left/image_raw` | `sensor_msgs/Image` | Left image |
| `/stereo_camera/left/camera_info` | `sensor_msgs/CameraInfo` | Left calibration |
| `/stereo_camera/right/image_raw` | `sensor_msgs/Image` | Right image |
| `/stereo_camera/right/camera_info` | `sensor_msgs/CameraInfo` | Right calibration (includes baseline term) |

---

## 2. Files for this step

All files live in `src/ugv_description/`:

```
src/ugv_description/
├── urdf/visionnav.urdf.xacro      # robot model, Gazebo plugins, stereo camera
├── launch/display.launch.py       # RViz2 only (robot_state_publisher + joint GUI + RViz2)
├── launch/gazebo.launch.py        # Gazebo + robot_state_publisher + spawn
├── worlds/outdoor_offroad.world   # existing world, used as the default
├── CMakeLists.txt                 # already installs urdf/ launch/ worlds/
└── package.xml                    # needs joint_state_publisher_gui and rviz2 exec_depends
```

---

## 3. Setup

```bash
sudo apt update
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-xacro \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui \
  ros-humble-rviz2 ros-humble-teleop-twist-keyboard \
  ros-humble-rqt-image-view ros-humble-tf2-tools \
  liburdfdom-tools graphviz
```

Build:

```bash
source /opt/ros/humble/setup.bash
cd ~/pathrage
colcon build --packages-select ugv_description --symlink-install
source install/setup.bash
```

---

## 4. Validate the model

```bash
F=src/ugv_description/urdf/visionnav.urdf.xacro
xacro $F > /tmp/visionnav.urdf
check_urdf /tmp/visionnav.urdf                        # root should be base_footprint
gz sdf -p /tmp/visionnav.urdf > /tmp/visionnav.sdf    # SDF conversion for Gazebo
grep -c 'type="continuous"' /tmp/visionnav.urdf       # expect 4
grep -c 'type="camera"' /tmp/visionnav.urdf           # expect 2
```

---

## 5. Run

Every terminal needs:

```bash
source /opt/ros/humble/setup.bash
source ~/pathrage/install/setup.bash
```

### a) Model only in RViz2 (no Gazebo)

```bash
ros2 launch ugv_description display.launch.py
```

Opens RViz2 with the robot model and TF, plus a joint GUI to rotate the wheels.

### b) Gazebo simulation

**Terminal A**

```bash
ros2 launch ugv_description gazebo.launch.py
# options: gui:=false   x:=2.0 y:=1.0 yaw:=1.57   camera_baseline:=0.20
```

### c) Teleoperate

**Terminal B**

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Keys: `i` forward · `,` back · `j` / `l` rotate left / right · `k` stop.


### d) See the live robot in RViz2

With Gazebo running, open RViz2 in another terminal:

```bash
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
```

In RViz2:
- Set **Fixed Frame** to `odom` (the robot then moves as you drive).
- **Add → RobotModel**, with Description Topic `/robot_description`.
- **Add → TF**.
- **Add → Image**, with Topic `/stereo_camera/left/image_raw` (and another for `right`).

### e) Camera and TF checks

```bash
ros2 topic hz /stereo_camera/left/image_raw
ros2 run rqt_image_view rqt_image_view
cd /tmp && ros2 run tf2_tools view_frames
```

### f) Complete test commands (Gazebo running, from a second terminal)

**TF**

```bash
cd /tmp && ros2 run tf2_tools view_frames                          # writes frames_*.pdf
ros2 run tf2_ros tf2_echo base_footprint base_link                 # z = 0.10
ros2 run tf2_ros tf2_echo base_footprint left_camera_optical_frame
ros2 run tf2_ros tf2_echo left_camera_frame right_camera_frame     # y = -0.12 (baseline)
ros2 topic info /tf -v | grep -E "Node name|Publisher count"
ros2 topic info /joint_states -v | grep -E "Node name|Publisher count"
```

**Stereo camera**

```bash
ros2 topic list | grep stereo_camera
ros2 topic hz /stereo_camera/left/image_raw                        # about 30 Hz
ros2 topic hz /stereo_camera/right/image_raw
ros2 topic echo /stereo_camera/left/image_raw --once --field header.frame_id
ros2 topic echo /stereo_camera/right/camera_info --once
ros2 run rqt_image_view rqt_image_view
```

**Wheel control**

```bash
ros2 topic info /cmd_vel
ros2 topic echo /joint_states --once
ros2 topic echo /odom --once
ros2 node list
```

**Motion checks (read odometry and wheel velocities after each move)**

```bash
timeout 3 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.4}}"
ros2 topic echo /odom --once --field pose.pose.position            # x up by about 1.2 m
timeout 3 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.4}}"
ros2 topic echo /odom --once --field pose.pose.position            # back near 0
timeout 3 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 1.0}}"
ros2 topic echo /joint_states --once --field velocity              # left/right equal and opposite
timeout 3 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.5}}"
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{}"            # stop
```

**Save outputs to /tmp**

```bash
ros2 topic list > /tmp/topics.txt
ros2 topic echo /odom --once > /tmp/odom.txt
ros2 topic echo /joint_states --once > /tmp/joint_states.txt
ros2 topic echo /stereo_camera/right/camera_info --once > /tmp/right_camera_info.txt
```

**Reset / cleanup**

```bash
ros2 service call /reset_simulation std_srvs/srv/Empty
pkill -f gzserver; pkill -f gzclient
```

---

## 6. Logs


```bash
ros2 launch ugv_description gazebo.launch.py 2>&1 | tee /tmp/gazebo_run.log
grep -iE "error|warn|fail" /tmp/gazebo_run.log
ls ~/.ros/log/latest
```

---
