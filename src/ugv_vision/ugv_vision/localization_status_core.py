"""ROS-free health logic for the fused localization (Phase 2, step 2.5).

The EKF keeps running on wheel odometry + gyro when ORB-SLAM3 stops delivering poses. This module
decides whether the visual pose is currently healthy ("fused") or not ("degraded") so that other
nodes (Nav2 speed limit in Phase 4, the viewer, the evaluation scripts) can react.

    slam_ok  = a /localization/slam_pose arrived within `timeout_s` AND ORB-SLAM3 does not
               currently report tracking lost
    mode     = "fused" if slam_ok else "degraded"
    reason   = "ok" | "no_pose_yet" | "timeout" | "tracking_lost"

Times are seconds on the node's clock (the simulation clock). Unit-tested in
test/test_localization_status_core.py.
"""
import json


class LocalizationHealth:
    def __init__(self, timeout_s=1.0):
        if timeout_s <= 0:
            raise ValueError("timeout_s must be > 0")
        self.timeout_s = float(timeout_s)
        self._last_pose = None      # time the last slam pose arrived
        self._tracking_ok = None    # None until ORB-SLAM3 ever reports

    def on_slam_pose(self, now):
        self._last_pose = float(now)

    def on_tracking_ok(self, ok):
        self._tracking_ok = bool(ok)

    def status(self, now):
        age = None if self._last_pose is None else max(0.0, float(now) - self._last_pose)
        if age is None:
            reason = "no_pose_yet"
        elif age > self.timeout_s:
            reason = "timeout"
        elif self._tracking_ok is False:
            reason = "tracking_lost"
        else:
            reason = "ok"
        slam_ok = reason == "ok"
        return {"mode": "fused" if slam_ok else "degraded", "slam_ok": slam_ok,
                "age_s": None if age is None else round(age, 3), "reason": reason}

    def status_json(self, now):
        return json.dumps(self.status(now))
