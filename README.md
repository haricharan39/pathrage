# UGV Visual Navigation (SIH PS 26126)

Vision-only autonomous navigation for an outdoor UGV -- no GPS, no LiDAR.
See /docs for the full architecture writeup and problem statement details.

## Workspace layout
- `src/ugv_description` -- robot model + Gazebo sim world (swap for real
  hardware drivers later; nothing else changes)
- `src/ugv_vision` -- Tier 1 (depth) + Tier 2 (segmentation) perception,
  stereo-inertial localization + EKF fusion
- `src/ugv_navigation` -- Nav2 integration (custom costmap layer, A*/D* Lite
  + TEB config), and the full_stack launch file
- `training/` -- segmentation model training + ONNX export (not a ROS2
  package, run standalone on the dev GPU)
- `bags/` -- recorded rosbags for offline SLAM/perception tuning
- `docs/` -- architecture diagrams, workflow doc, PPT drafts

## Architecture

```
Stereo camera + IMU
        |
   +----+----+
   v         v
Visual SLAM   Perception AI
(pose +      (Tier 1 depth +
global 3D    Tier 2 segmentation)
 map)             |
   |              v
   |         Local costmap
   v              |
Global 3D map -----+
        |
        v
Hierarchical Planner
(Global: A*/D* Lite | Local: TEB)
        |
        v
    Controller
        |
        v
       UGV
```

## Build
```bash
cd ugv_visual_nav_ws
colcon build --symlink-install
source install/setup.bash
```

## Run the full stack in simulation
```bash
ros2 launch ugv_navigation full_stack.launch.py
```

## Status
Scaffolding only -- see TODO comments in each node file for what's
actually implemented vs. stubbed. Build order (see /docs for the full
plan): sim environment -> Nav2 against Tier-1-only costmap -> add
localization -> add Tier 2 segmentation -> stress test.
