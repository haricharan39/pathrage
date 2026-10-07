#!/usr/bin/env bash
# Runs ORB-SLAM3 (stereo) on the simulated camera. Start the sim first:
#   ros2 launch ugv_navigation full_stack.launch.py
set -eo pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VOCAB="$HOME/ORB_SLAM3/Vocabulary/ORBvoc.txt"
YAML="${ORB_YAML:-$REPO_DIR/src/ugv_vision/config/orbslam3_stereo_pinhole.yaml}"
LEFT="${LEFT_TOPIC:-/stereo_camera/left/image_rect}"
RIGHT="${RIGHT_TOPIC:-/stereo_camera/right/image_rect}"

[ -f "$VOCAB" ] || { echo "Vocabulary missing: $VOCAB"; exit 1; }
[ -f "$YAML" ]  || { echo "Config missing: $YAML"; exit 1; }
[ -f "$HOME/orbslam_ws/install/setup.bash" ] || { echo "Wrapper not built"; exit 1; }

export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:$HOME/ORB_SLAM3/lib:$HOME/ORB_SLAM3/Thirdparty/DBoW2/lib:$HOME/ORB_SLAM3/Thirdparty/g2o/lib"
source /opt/ros/humble/setup.bash
source "$HOME/orbslam_ws/install/setup.bash"
[ -f "$REPO_DIR/install/setup.bash" ] && source "$REPO_DIR/install/setup.bash"

if ! timeout 8 ros2 topic echo --once "$LEFT" --no-arr >/dev/null 2>&1; then
  echo "No messages on $LEFT. Is the simulation running?"; exit 1
fi

echo "Starting ORB-SLAM3 stereo on $LEFT / $RIGHT"
exec ros2 run orbslam3 stereo "$VOCAB" "$YAML" false \
  --ros-args -r camera/left:="$LEFT" -r camera/right:="$RIGHT"
