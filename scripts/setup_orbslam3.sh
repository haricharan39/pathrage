#!/usr/bin/env bash
# Builds Pangolin, ORB-SLAM3 and its ROS 2 wrapper OUTSIDE this repository:
#   ~/Pangolin   ~/ORB_SLAM3   ~/orbslam_ws
# Usage: bash scripts/setup_orbslam3.sh
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PATCH="$REPO_DIR/third_party/orbslam3_ros2_humble.patch"

if [ ! -f "$PATCH" ]; then
  echo "WARNING: wrapper patch not found: $PATCH" >&2
  echo "         The wrapper will be built UNPATCHED. See third_party/README.md." >&2
  PATCH=""
fi

echo "==> Installing build dependencies"
sudo apt update
sudo apt install -y build-essential cmake git pkg-config \
  libeigen3-dev libopencv-dev libglew-dev libssl-dev \
  libboost-serialization-dev libepoxy-dev libegl1-mesa-dev \
  libwayland-dev libxkbcommon-dev \
  ros-humble-vision-opencv ros-humble-cv-bridge ros-humble-message-filters

echo "==> Pangolin v0.8"
if [ ! -d "$HOME/Pangolin" ]; then
  git clone --recursive https://github.com/stevenlovegrove/Pangolin.git "$HOME/Pangolin"
fi
cd "$HOME/Pangolin"
git checkout v0.8
./scripts/install_prerequisites.sh recommended
cmake -B build -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF -DBUILD_PANGOLIN_PYTHON=OFF
cmake --build build -j2
sudo cmake --install build
sudo ldconfig

echo "==> ORB-SLAM3 (branch c++14_comp)"
if [ ! -d "$HOME/ORB_SLAM3" ]; then
  git clone -b c++14_comp https://github.com/UZ-SLAMLab/ORB_SLAM3.git "$HOME/ORB_SLAM3"
fi
cd "$HOME/ORB_SLAM3"
chmod +x build.sh
./build.sh

echo "==> Library path"
if ! grep -q "ORB_SLAM3/lib" "$HOME/.bashrc"; then
  echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/ORB_SLAM3/lib:$HOME/ORB_SLAM3/Thirdparty/DBoW2/lib:$HOME/ORB_SLAM3/Thirdparty/g2o/lib' >> "$HOME/.bashrc"
fi

echo "==> ROS 2 wrapper"
mkdir -p "$HOME/orbslam_ws/src"
cd "$HOME/orbslam_ws/src"
if [ ! -d orbslam3_ros2 ]; then
  git clone https://github.com/zang09/ORB_SLAM3_ROS2.git orbslam3_ros2
fi
cd orbslam3_ros2
if [ -z "$PATCH" ]; then
  echo "no patch to apply"
elif git apply --check "$PATCH" 2>/dev/null; then
  git apply "$PATCH"
  echo "patch applied"
else
  echo "patch not applied (already applied, or the wrapper changed upstream)"
fi
sed -i "s#^set(ORB_SLAM3_ROOT_DIR .*#set(ORB_SLAM3_ROOT_DIR \"$HOME/ORB_SLAM3\")#" CMakeModules/FindORB_SLAM3.cmake

echo "==> Building the wrapper (clean environment, one job at a time)"
cd "$HOME/orbslam_ws"
rm -rf build install log
env -u PYTHONPATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH bash -c '
  source /opt/ros/humble/setup.bash
  MAKEFLAGS=-j1 colcon build --symlink-install --executor sequential
'

echo
echo "Done. Verify with:"
echo "  source ~/.bashrc && source /opt/ros/humble/setup.bash && source ~/orbslam_ws/install/setup.bash"
echo "  ros2 pkg executables orbslam3     # expect: orbslam3 stereo"