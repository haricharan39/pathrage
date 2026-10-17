#!/usr/bin/env bash
# Record the topics needed for localization evaluation into bags/<name>.
#   bash tools/record_bag.sh r1_run1
# Phase 2 topics (/odom_noisy, /odometry/filtered, /localization/status) are included; /odom_noisy
# only has data when the stack runs with odom_noise:=true. More topics can be passed as extra args.
set -eo pipefail
NAME="${1:?usage: record_bag.sh <name> [extra topics...]}"; shift || true
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
exec ros2 bag record -o "$REPO_DIR/bags/$NAME" --use-sim-time \
  /ground_truth/odom /odom /camera/imu /tf /tf_static \
  /localization/slam_pose /orb_slam3/pose /orb_slam3/tracking_ok \
  /odom_noisy /odometry/filtered /localization/status "$@"
