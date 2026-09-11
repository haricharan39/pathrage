"""
Localization glue node.

The actual SLAM computation happens in ORB-SLAM3 (stereo-inertial mode) or
VINS-Fusion, run as their own external ROS2 node/process (see /docs for
build+launch instructions -- neither ships as a rosdep package, both are
built from source alongside this workspace).

This node's job is narrower: re-publish the SLAM pose in a consistent
frame/topic, and act as the place where EKF fusion (robot_localization) is
configured to combine it with wheel odometry. In practice most of the real
fusion logic lives in config/ekf_params.yaml + the robot_localization node
launched alongside this one -- this file just handles the SLAM-specific
pre-processing robot_localization can't do out of the box.

Subscribes:
    /orb_slam3/pose  or  /vins_estimator/odometry   (whichever SLAM backend is used)
Publishes:
    /localization/slam_pose   (geometry_msgs/PoseWithCovarianceStamped)
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class LocalizationNode(Node):
    def __init__(self):
        super().__init__("localization_node")

        self.declare_parameter("slam_backend", "orb_slam3")  # or "vins_fusion"
        self.declare_parameter("slam_pose_topic", "/orb_slam3/pose")

        topic = self.get_parameter("slam_pose_topic").value
        # NOTE: message type here depends on the SLAM backend's actual
        # output type -- adjust the subscription type to match once a
        # backend is chosen and built.
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped, topic, self.slam_pose_callback, 10
        )
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, "/localization/slam_pose", 10
        )

        self.get_logger().info(
            f"localization_node started, backend={self.get_parameter('slam_backend').value}"
        )

    def slam_pose_callback(self, msg: PoseWithCovarianceStamped):
        # TODO: re-frame/re-stamp as needed, fill in covariance if the SLAM
        # backend doesn't provide one (robot_localization needs it), then
        # republish for the EKF node to consume alongside wheel odometry.
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
