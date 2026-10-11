"""Localization glue node: ORB-SLAM3 pose -> /localization/slam_pose.

ORB-SLAM3 runs as its own external ROS 2 node (built by scripts/setup_orbslam3.sh, started by
launch/orbslam.launch.py). This node converts its camera pose into what robot_localization's
EKF needs: the base_footprint pose in the odom frame, with a covariance. The maths is in
slam_pose_core.py (ROS-free, unit-tested).

Interface (docs/INTERFACES.md):
    Subscribes:
        /orb_slam3/pose            geometry_msgs/PoseStamped   left optical camera in 'orb_map'
        /orb_slam3/tracking_state  std_msgs/Int32              raw ORB-SLAM3 tracking state
        TF                         base_footprint <- left_camera_optical_frame (static),
                                   odom <- base_footprint (once, at initialisation)
    Publishes:
        /localization/slam_pose    geometry_msgs/PoseWithCovarianceStamped, frame 'odom',
                                   same stamp as the image pair. Nothing while tracking is lost.

The pose is relative to the first accepted frame, composed with the odom pose at that moment,
so it is directly comparable with /odom and /ground_truth/odom (after a rigid alignment).
"""
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Int32
from tf2_ros import Buffer, TransformListener

from . import slam_pose_core as core


class LocalizationNode(Node):
    def __init__(self):
        super().__init__("localization_node")
        d = self.declare_parameter
        d("slam_pose_topic", "/orb_slam3/pose")
        d("tracking_state_topic", "/orb_slam3/tracking_state")
        d("output_topic", "/localization/slam_pose")
        d("orb_frame", "orb_map")
        d("odom_frame", "odom")
        d("base_frame", "base_footprint")
        d("camera_frame", "left_camera_optical_frame")
        d("sigma_xy", 0.05)
        d("sigma_yaw", 0.03)
        d("inflate_factor", 10.0)
        d("inflate_hold", 30)
        d("max_jump_m", 0.3)
        d("max_speed", 2.0)
        d("max_jump_rad", 0.3)
        d("max_yaw_rate", 3.0)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.orb_frame, self.odom_frame = g("orb_frame"), g("odom_frame")
        self.base_frame, self.camera_frame = g("base_frame"), g("camera_frame")
        self.cfg = dict(sigma_xy=float(g("sigma_xy")), sigma_yaw=float(g("sigma_yaw")),
                        inflate_factor=float(g("inflate_factor")), inflate_hold=int(g("inflate_hold")),
                        max_jump_m=float(g("max_jump_m")), max_speed=float(g("max_speed")),
                        max_jump_rad=float(g("max_jump_rad")), max_yaw_rate=float(g("max_yaw_rate")))
        self.conv = None            # created once the camera extrinsics are known
        self._pending_states = []   # tracking states that arrived before the converter existed
        self._n_in = self._n_out = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, g("output_topic"), 10)
        self.create_subscription(Int32, g("tracking_state_topic"), self._on_state, 10)
        self.create_subscription(PoseStamped, g("slam_pose_topic"), self._on_pose, 10)
        self.create_timer(5.0, self._report)
        self.get_logger().info(
            f"localization_node: {g('slam_pose_topic')} ({self.orb_frame}) -> {g('output_topic')} "
            f"({self.odom_frame}), camera {self.camera_frame} -> base {self.base_frame}")

    # ------------------------------------------------------------------ TF
    def _lookup_matrix(self, target, source, stamp):
        """4x4 matrix taking points from `source` into `target`; None if not available."""
        for t in (stamp, Time()):
            try:
                tf = self.tf_buffer.lookup_transform(target, source, t, timeout=Duration(seconds=0.05))
            except Exception:
                continue
            tr, q = tf.transform.translation, tf.transform.rotation
            return core.pose_to_matrix((tr.x, tr.y, tr.z), (q.x, q.y, q.z, q.w))
        return None

    def _ensure_converter(self):
        if self.conv is not None:
            return True
        t_base_cam = self._lookup_matrix(self.base_frame, self.camera_frame, Time())
        if t_base_cam is None:
            self.get_logger().warn(f"waiting for TF {self.base_frame} <- {self.camera_frame}",
                                   throttle_duration_sec=5.0)
            return False
        self.conv = core.SlamPoseConverter(t_base_cam, **self.cfg)
        for s in self._pending_states:
            self.conv.on_tracking_state(s)
        self._pending_states.clear()
        return True

    # ------------------------------------------------------------------ callbacks
    def _on_state(self, msg):
        if self.conv is None:
            self._pending_states.append(msg.data)
            del self._pending_states[:-50]
        else:
            self.conv.on_tracking_state(msg.data)

    def _on_pose(self, msg):
        self._n_in += 1
        if msg.header.frame_id != self.orb_frame:
            self.get_logger().warn(
                f"pose in frame '{msg.header.frame_id}', expected '{self.orb_frame}'; dropped",
                throttle_duration_sec=5.0)
            return
        if not self._ensure_converter():
            return
        p, q = msg.pose.position, msg.pose.orientation
        t_orb_cam = core.pose_to_matrix((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
        stamp = Time.from_msg(msg.header.stamp)

        t_odom_base = None
        if not self.conv.initialised:
            t_odom_base = self._lookup_matrix(self.odom_frame, self.base_frame, stamp)
            if t_odom_base is None:
                self.get_logger().warn(f"waiting for TF {self.odom_frame} <- {self.base_frame}",
                                       throttle_duration_sec=5.0)
                return
        res = self.conv.process(t_orb_cam, stamp.nanoseconds * 1e-9, t_odom_base)
        if res is None:
            return
        out, cov, info = res
        if info["initialised"]:
            self.get_logger().info("ORB-SLAM3 initialised; pose anchored to the odom pose")
        if info["reanchored"]:
            self.get_logger().warn("ORB-SLAM3 started a new map; pose re-anchored, covariance inflated")
        if info["jump"]:
            self.get_logger().warn("ORB-SLAM3 pose jumped; covariance inflated", throttle_duration_sec=2.0)

        pos, quat = core.matrix_to_pose(out)
        m = PoseWithCovarianceStamped()
        m.header.stamp = msg.header.stamp
        m.header.frame_id = self.odom_frame
        m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = pos
        (m.pose.pose.orientation.x, m.pose.pose.orientation.y,
         m.pose.pose.orientation.z, m.pose.pose.orientation.w) = quat
        m.pose.covariance = np.asarray(cov).reshape(-1).tolist()
        self.pub.publish(m)
        self._n_out += 1

    def _report(self):
        if self._n_in == 0:
            self.get_logger().info("no ORB-SLAM3 pose received (not started, not initialised or lost)",
                                   throttle_duration_sec=30.0)
        else:
            c = self.conv
            self.get_logger().info(f"slam poses in/out: {self._n_in}/{self._n_out}"
                                   + (f", jumps {c.jumps}, new maps {c.reanchors}" if c else ""))


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
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
