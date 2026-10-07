# third_party

`orbslam3_ros2_humble.patch` is the set of changes your working ORB-SLAM3 ROS 2
wrapper (`zang09/ORB_SLAM3_ROS2`) needs to build on Humble. `scripts/setup_orbslam3.sh`
applies it automatically if the file exists.

## Regenerate the patch from your working wrapper build

```bash
mkdir -p ~/pathrage/third_party
cd ~/orbslam_ws/src/orbslam3_ros2
git status --short
# CMakeModules/FindORB_SLAM3.cmake is excluded: the setup script edits it itself.
git diff -- . ':!CMakeModules/FindORB_SLAM3.cmake' > ~/pathrage/third_party/orbslam3_ros2_humble.patch
wc -l ~/pathrage/third_party/orbslam3_ros2_humble.patch
cd ~/pathrage && git add third_party && git commit -m "Add ORB-SLAM3 wrapper patch"
```

If the patch is empty, the wrapper built without source changes and no patch is needed.
