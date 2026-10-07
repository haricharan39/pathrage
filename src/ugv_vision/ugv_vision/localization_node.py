"""
Localization glue node.

The SLAM computation happens in ORB-SLAM3 (stereo mode first), run as its own
external ROS 2 node (built by scripts/setup_orbslam3.sh, see README). This node's
job is narrow: re-publish the SLAM pose in a consistent frame with a covariance, so
robot_localization's EKF can fuse it with wheel odometry and the IMU gyro.

Interface (see docs/INTERFACES.md):
    Subscribes:
        <slam_pose_topic>              (default /orb_slam3/pose; the exact topic and
                                        message type are confirmed in Phase 1)
    Publishes:
        /localization/slam_pose        geometry_msgs/PoseWithCovarianceStamped

Status: passthrough only. Frame conversion (ORB-SLAM3 camera convention -> map/odom)
and covariance are implemented in Phase 1.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class LocalizationNode(Node):
    def __init__(self):
        super().__init__("localization_node")

        self.declare_parameter("slam_backend", "orb_slam3")
        self.declare_parameter("slam_pose_topic", "/orb_slam3/pose")

        topic = self.get_parameter("slam_pose_topic").value
        # TODO (Phase 1): set the subscription type to what the wrapper really
        # publishes (it may be PoseStamped or Odometry, not PoseWithCovarianceStamped).
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped, topic, self.slam_pose_callback, 10
        )
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, "/localization/slam_pose", 10
        )

        self.get_logger().info(
            f"localization_node started, backend={self.get_parameter('slam_backend').value}, "
            f"listening on {topic}"
        )

    def slam_pose_callback(self, msg: PoseWithCovarianceStamped):
        # TODO (Phase 1): re-frame/re-stamp, and fill in the covariance if the backend
        # does not provide one (robot_localization needs it), then republish.
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
