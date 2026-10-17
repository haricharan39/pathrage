"""Fused-localization health: /localization/status (Phase 2, step 2.5).

Publishes {"mode": "fused"|"degraded", "slam_ok": bool, "age_s": float|null, "reason": str} as a
std_msgs/String (JSON) at `publish_hz`. "degraded" means the EKF is running on wheel odometry + gyro
only, because ORB-SLAM3 has been silent for more than `slam_pose_timeout` seconds, reports tracking
lost, or has not produced a pose yet. The logic is in localization_status_core.py (ROS-free).

Interface (docs/INTERFACES.md):
    Subscribes:  /localization/slam_pose   geometry_msgs/PoseWithCovarianceStamped
                 /orb_slam3/tracking_ok    std_msgs/Bool
    Publishes:   /localization/status      std_msgs/String (JSON)
"""
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from std_msgs.msg import Bool, String

from . import localization_status_core as core


class LocalizationStatusNode(Node):
    def __init__(self):
        super().__init__("localization_status_node")
        d = self.declare_parameter
        d("slam_pose_topic", "/localization/slam_pose")
        d("tracking_ok_topic", "/orb_slam3/tracking_ok")
        d("status_topic", "/localization/status")
        d("slam_pose_timeout", 1.0)
        d("publish_hz", 5.0)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.health = core.LocalizationHealth(float(g("slam_pose_timeout")))
        self._last_mode = None
        self.pub = self.create_publisher(String, g("status_topic"), 10)
        self.create_subscription(PoseWithCovarianceStamped, g("slam_pose_topic"), self._on_pose, 10)
        self.create_subscription(Bool, g("tracking_ok_topic"), self._on_ok, 10)
        self.create_timer(1.0 / max(0.5, float(g("publish_hz"))), self._tick)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_pose(self, msg):
        self.health.on_slam_pose(self._now())

    def _on_ok(self, msg):
        self.health.on_tracking_ok(msg.data)

    def _tick(self):
        now = self._now()
        if now == 0.0:
            return  # simulation clock not running yet
        st = self.health.status(now)
        self.pub.publish(String(data=self.health.status_json(now)))
        if st["mode"] != self._last_mode:
            if self._last_mode is not None or st["mode"] == "degraded":
                self.get_logger().info(f"localization mode: {st['mode']} ({st['reason']})")
            self._last_mode = st["mode"]


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
