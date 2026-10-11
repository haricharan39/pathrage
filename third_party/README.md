# third_party

`orbslam3_ros2_humble.patch` is the set of changes your working ORB-SLAM3 ROS 2
wrapper (`zang09/ORB_SLAM3_ROS2`) needs to build on Humble. `scripts/setup_orbslam3.sh`
applies it automatically if the file exists.

## Regenerate the patch from your working wrapper build

```bash
mkdir -p ~/pathrage/third_party
cd ~/orbslam_ws/src/orbslam3_ros2
git log -1 --format=%h        # the patch is made against 00c5433 (WRAPPER_COMMIT in scripts/setup_orbslam3.sh)
git status --short
# CMakeModules/FindORB_SLAM3.cmake is excluded: the setup script edits it itself.
git diff -- . ':!CMakeModules/FindORB_SLAM3.cmake' > ~/pathrage/third_party/orbslam3_ros2_humble.patch
wc -l ~/pathrage/third_party/orbslam3_ros2_humble.patch
cd ~/pathrage && git add third_party && git commit -m "Add ORB-SLAM3 wrapper patch"
```

If the patch is empty, the wrapper built without source changes and no patch is needed.

## What the patch contains

Builds for Humble (CMake paths, OpenCV, only the `stereo` target) and, since Phase 1, the pose output
in `src/stereo/stereo-slam-node.{hpp,cpp}`: after `TrackStereo` it publishes

| Topic | Type | Content |
|---|---|---|
| `/orb_slam3/pose` | `geometry_msgs/PoseStamped` | Twc (inverse of ORB-SLAM3's Tcw): left optical camera in `orb_map`, same stamp as the image pair, only while tracking is OK |
| `/orb_slam3/tracking_ok` | `std_msgs/Bool` | true only when the tracking state is OK, every frame |
| `/orb_slam3/tracking_state` | `std_msgs/Int32` | raw ORB-SLAM3 state (0 no images, 1 not initialised, 2 OK, 3 recently lost, 4 lost), every frame |

It has not been compiled here (no ORB-SLAM3/ROS in the authoring environment); it applies cleanly to
upstream `00c5433`. Build with one compile job (the setup script does).
