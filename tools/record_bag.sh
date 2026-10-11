#!/usr/bin/env bash
# Record the topics needed for localization evaluation into bags/<name>.
#   bash tools/record_bag.sh r1_run1
# Add topics later (e.g. /odom_noisy, /odometry/filtered) by passing them as extra args.
set -eo pipefail
NAME="${1:?usage: record_bag.sh <name> [extra topics...]}"; shift || true
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
exec ros2 bag record -o "$REPO_DIR/bags/$NAME" --use-sim-time \
  /ground_truth/odom /odom /camera/imu /tf /tf_static \
  /localization/slam_pose /orb_slam3/pose /orb_slam3/tracking_ok "$@"
