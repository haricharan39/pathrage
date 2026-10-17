"""Honest-test wheel odometry: /odom -> /odom_noisy (Phase 2, step 2.1).

Republishes Gazebo's wheel odometry with seeded, realistic errors (speed-scale error 3-5 %, yaw-rate
bias, Gaussian noise) so that fusing it with ORB-SLAM3 and the gyro in the EKF actually proves
something. The clean /odom is left untouched and stays the reference. The maths is in
odom_noise_core.py (ROS-free, unit-tested).

Interface (docs/INTERFACES.md):
    Subscribes:  /odom         nav_msgs/Odometry   clean wheel odometry (Gazebo diff-drive plugin)
    Publishes:   /odom_noisy   nav_msgs/Odometry   frame 'odom', child 'base_footprint', same stamp.
                               pose = the noisy twist dead-reckoned from the first /odom pose (this is the
                               "noisy wheel odometry only" trajectory of the Phase 2 results table);
                               twist = the noisy forward speed and yaw rate, with covariances that
                               include the injected systematic errors.

Started by localization.launch.py when odom_noise:=true; the EKF then reads /odom_noisy instead of
/odom. Error levels are in config/odom_noise.yaml.
"""
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from . import odom_noise_core as core


class OdomNoiseNode(Node):
    def __init__(self):
        super().__init__("odom_noise_node")
        d = self.declare_parameter
        c = core.NoiseConfig()
        d("input_topic", "/odom")
        d("output_topic", "/odom_noisy")
        d("seed", c.seed)
        d("scale_err_min", c.scale_err_min)
        d("scale_err_max", c.scale_err_max)
        d("yaw_bias", c.yaw_bias)
        d("sigma_v", c.sigma_v)
        d("sigma_w", c.sigma_w)
        d("max_dt", c.max_dt)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        cfg = core.NoiseConfig(seed=int(g("seed")), scale_err_min=float(g("scale_err_min")),
                               scale_err_max=float(g("scale_err_max")), yaw_bias=float(g("yaw_bias")),
                               sigma_v=float(g("sigma_v")), sigma_w=float(g("sigma_w")),
                               max_dt=float(g("max_dt")))
        self.model = core.OdomNoiseModel(cfg)   # raises ValueError on an invalid config
        self._started = False
        self._n = 0
        # Reliable output (default QoS): robot_localization subscribes reliably, and a reliable
        # subscriber cannot connect to a best-effort publisher. The input may be either kind.
        self.pub = self.create_publisher(Odometry, g("output_topic"), 10)
        self.create_subscription(Odometry, g("input_topic"), self._on_odom, qos_profile_sensor_data)
        self.create_timer(10.0, self._report)
        self.get_logger().info(
            f"odom_noise_node: {g('input_topic')} -> {g('output_topic')}  seed {cfg.seed}  "
            f"speed scale error {self.model.scale_v:+.1%}, yaw-rate scale error {self.model.scale_w:+.1%}, "
            f"yaw bias {cfg.yaw_bias:+.4f} rad/s, noise sigma {cfg.sigma_v} m/s / {cfg.sigma_w} rad/s")

    def _on_odom(self, msg):
        if not self._started:
            p, o = msg.pose.pose.position, msg.pose.pose.orientation
            self.model.reset((p.x, p.y, core.yaw_from_quat(o.x, o.y, o.z, o.w)))
            self._started = True
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        s = self.model.update(t, msg.twist.twist.linear.x, msg.twist.twist.angular.z)

        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose.pose.position.x, out.pose.pose.position.y = s.x, s.y
        out.pose.pose.position.z = msg.pose.pose.position.z
        (out.pose.pose.orientation.x, out.pose.pose.orientation.y,
         out.pose.pose.orientation.z, out.pose.pose.orientation.w) = core.quat_from_yaw(s.yaw)
        # order: x y z roll pitch yaw; z/roll/pitch are not estimated (planar robot)
        pc = [0.0] * 36
        for i, v in zip((0, 7, 14, 21, 28, 35), (s.var_x, s.var_y, 1e-6, 1e-6, 1e-6, s.var_yaw)):
            pc[i] = v
        out.pose.covariance = pc
        out.twist.twist.linear.x = s.vx
        out.twist.twist.angular.z = s.wz
        # order: vx vy vz wx wy wz; vy is not fused (skid-steer lateral slip is not modelled)
        tc = [0.0] * 36
        for i, v in zip((0, 7, 14, 21, 28, 35), (s.var_vx, 1e-2, 1e-2, 1e-2, 1e-2, s.var_wz)):
            tc[i] = v
        out.twist.covariance = tc
        self.pub.publish(out)
        self._n += 1

    def _report(self):
        if self._n == 0:
            self.get_logger().info("no /odom received yet", throttle_duration_sec=30.0)
        else:
            self.get_logger().info(f"{self._n} noisy odometry messages published")


def main(args=None):
    rclpy.init(args=args)
    node = OdomNoiseNode()
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
