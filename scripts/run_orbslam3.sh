#!/usr/bin/env bash
# Runs ORB-SLAM3 (stereo) on the simulated camera, WITHOUT localization_node (use this for
# debugging the wrapper alone; the normal way is full_stack.launch.py orbslam:=true).
# Start the sim first:
#   ros2 launch ugv_navigation full_stack.launch.py
# Publishes /orb_slam3/pose, /orb_slam3/tracking_ok, /orb_slam3/tracking_state. The pose only
# appears once ORB-SLAM3 has initialised (textured scene + some motion).
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
ros2 run orbslam3 stereo "$VOCAB" "$YAML" false false \
  --ros-args -p use_sim_time:=true -r camera/left:="$LEFT" -r camera/right:="$RIGHT" &
SLAM_PID=$!
trap 'kill "$SLAM_PID" 2>/dev/null || true' EXIT INT TERM

# The pose topic only exists with the patched wrapper (third_party/orbslam3_ros2_humble.patch).
for _ in $(seq 1 20); do
  kill -0 "$SLAM_PID" 2>/dev/null || { echo "ORB-SLAM3 exited early (see messages above)"; exit 1; }
  if ros2 topic list 2>/dev/null | grep -qx "/orb_slam3/pose"; then
    echo "OK: /orb_slam3/pose exists (it carries data once ORB-SLAM3 is initialised)"
    wait "$SLAM_PID"; exit $?
  fi
  sleep 1
done
echo "ERROR: /orb_slam3/pose was not created. The wrapper is probably unpatched:"
echo "       re-run scripts/setup_orbslam3.sh (it applies third_party/orbslam3_ros2_humble.patch)."
exit 1
