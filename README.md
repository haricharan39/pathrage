# PATHRAGE – VisionNav

**Autonomous Navigation in GPS-Denied Outdoor Environments** (Smart India Hackathon 2026)

An autonomous unmanned ground vehicle (UGV) that navigates using a **single stereo camera**: no GPS, no LiDAR and no IMU. The system is developed and tested in simulation with ROS 2 Humble and Gazebo Classic 11.

**Environment:** Ubuntu 22.04 · ROS 2 Humble · Gazebo Classic 11 · RViz2

---

## Contents

1. [Approach](#1-approach)
2. [Progress](#2-progress)
3. [Repository layout](#3-repository-layout)
4. [Installation](#4-installation)
5. [Run the simulation](#5-run-the-simulation)

---

## 1. Approach

Without GPS the robot has to work out where it is and where it can drive from what it sees. Using the two images of one stereo camera, VisionNav:

1. computes depth (the distance to surfaces),
2. tracks the robot's motion (visual localization),
3. builds a map of the surroundings, and
4. plans and follows a path through that map (next step).

```
Gazebo stereo cameras ──► stereo_image_proc ──► rectified images, /disparity, /points2
 (left/right images                                   │
  + camera_info)                                      ▼
wheel /odom ───────────────────────────────────► RTAB-Map ──► /map  +  map → odom
(odom → base_footprint TF)                            │
                                                      ▼
                                     RViz (Fixed Frame: map)  ...next: Nav2
```

### The robot

A four-wheel-drive skid-steer UGV with four actuated wheel joints and exactly one stereo camera assembly (no other sensors).

| Item | Value (configurable in `visionnav.urdf.xacro`) |
|---|---|
| Chassis | 0.60 × 0.40 × 0.15 m, 12 kg, ground clearance 0.06 m |
| Wheels | radius 0.10 m, width 0.06 m, track 0.50 m, wheel base 0.40 m |
| Drive | Gazebo diff-drive plugin with two wheel pairs, command topic `/cmd_vel` |
| Stereo camera | 640×480, 80° horizontal field of view, 30 Hz, baseline 0.12 m (`camera_baseline`) |

### TF tree

```
map → odom → base_footprint → base_link
                                ├── front_left / front_right / rear_left / rear_right wheel links
                                └── stereo_camera_link
                                      ├── left_camera_frame  → left_camera_optical_frame
                                      └── right_camera_frame → right_camera_optical_frame
```

Each transform has one publisher: `robot_state_publisher` (fixed and wheel frames), the Gazebo drive plugin (`odom → base_footprint`) and RTAB-Map (`map → odom`).

### Main topics

| Topic | Purpose |
|---|---|
| `/cmd_vel` | drive command (input) |
| `/odom`, `/joint_states` | wheel odometry and wheel joint states |
| `/stereo_camera/{left,right}/image_raw`, `.../camera_info` | raw stereo images |
| `/stereo_camera/{left,right}/image_rect` | rectified images |
| `/disparity`, `/points2` | stereo depth |
| `/map`, `/mapData`, `/mapGraph`, `/info` | RTAB-Map outputs |

---

## 2. Progress

| Step | What it does | Status |
|---|---|---|
| 1 | UGV model (URDF/Xacro), spawn in Gazebo, teleoperation, RViz view | Done |
| 2a | Stereo depth: rectified images, disparity, point cloud (`stereo_image_proc`) | Done |
| 2b | Mapping: RTAB-Map builds the occupancy grid `/map` from stereo and wheel odometry | Done |
| 2c | Visual localization with ORB-SLAM3 (stereo) | Built; running it on the simulated feed and publishing its pose to ROS are next |
| 3 | Fuse wheel odometry and visual pose (EKF); connect the `ugv_vision` perception nodes | Pending |
| 4 | Autonomous navigation with Nav2 on the stereo map | Pending |

---

## 3. Repository layout

```
pathrage/
├── src/
│   ├── ugv_description/          
│   │   ├── urdf/visionnav.urdf.xacro
│   │   ├── launch/display.launch.py        
│   │   ├── launch/gazebo.launch.py         
│   │   └── worlds/outdoor_offroad.world
│   ├── ugv_vision/              
│   │   ├── launch/slam.launch.py           
│   │   └── config/orbslam3_stereo.yaml     
│   └── ugv_navigation/           
├── scripts/setup_orbslam3.sh     
└── README.md
```
---

## 4. Installation

### 4.1 Clone

```bash
cd ~
git clone git@github.com:haricharan39/pathrage.git
cd ~/pathrage
```

### 4.2 ROS 2 dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs ros-humble-xacro \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui \
  ros-humble-rviz2 ros-humble-teleop-twist-keyboard \
  ros-humble-rqt-image-view ros-humble-tf2-tools \
  ros-humble-stereo-image-proc ros-humble-rtabmap-ros \
  liburdfdom-tools graphviz
```

### 4.3 Build the ROS 2 packages

```bash
source /opt/ros/humble/setup.bash
cd ~/pathrage
colcon build --packages-select ugv_description ugv_vision --symlink-install
source install/setup.bash
```

### 4.4 ORB-SLAM3 and its ROS 2 wrapper

ORB-SLAM3 and its wrapper are third-party projects and are not stored in this repository. The script builds them outside the repo, in `~/Pangolin`, `~/ORB_SLAM3` and `~/orbslam_ws`:

```bash
cd ~/pathrage
bash scripts/setup_orbslam3.sh 2>&1 | tee /tmp/setup_orbslam3.log
source ~/.bashrc
```

It installs the build dependencies, builds Pangolin v0.8 and ORB-SLAM3 (branch `c++14_comp`), adds the library paths to `~/.bashrc`, clones the wrapper `zang09/ORB_SLAM3_ROS2`, applies `third_party/orbslam3_ros2_humble.patch` and builds the wrapper with one compile job at a time.

---

## 5. Run the simulation

Every terminal needs:

```bash
source /opt/ros/humble/setup.bash
source ~/pathrage/install/setup.bash
```

Start the parts in this order.

**Terminal A: Gazebo**
```bash
ros2 launch ugv_description gazebo.launch.py gui:=false     # remove gui:=false to show the Gazebo window
```
**Terminal B: stereo depth**
```bash
ros2 launch stereo_image_proc stereo_image_proc.launch.py \
  left_namespace:=/stereo_camera/left right_namespace:=/stereo_camera/right \
  approximate_sync:=true use_sim_time:=true
```

**Terminal C: mapping (RTAB-Map)**
```bash
ros2 launch ~/pathrage/src/ugv_vision/launch/slam.launch.py
```

**Terminal D: drive the robot**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard     # i forward, , back, j/l turn, k stop
```
Drive forward slowly for a few metres and turn gradually. RTAB-Map only adds map nodes after the robot has moved, and it needs textured surroundings (rocks, objects) in view.

**Terminal E: RViz**
```bash
rviz2 --ros-args -p use_sim_time:=true
```

---


## Licenses and third-party software

ORB-SLAM3 is licensed under GPLv3 and is built from its own repository; it is not copied into this repository. RTAB-Map, `stereo_image_proc`, Gazebo and ROS 2 packages are installed from their upstream packages under their own licenses.