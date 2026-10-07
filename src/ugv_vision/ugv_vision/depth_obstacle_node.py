"""
Tier 1 perception: geometric, depth-based obstacle detection.

No machine learning here on purpose -- this is the deterministic safety floor
described in the project's architecture. Any stereo point whose height above the
ground plane exceeds `ground_height_thresh` (or that drops far below it, i.e. a
ditch) is marked as an obstacle, independent of whatever Tier 2 decides.

Interface (see docs/INTERFACES.md):
    Subscribes:
        /points2                    sensor_msgs/PointCloud2   (stereo_image_proc, optical frame)
    Publishes:
        /perception/tier1_obstacles sensor_msgs/PointCloud2   (obstacle points, frame base_footprint)

Why PointCloud2 and not a depth image: stereo_image_proc already produces /points2,
and Nav2's built-in ObstacleLayer consumes PointCloud2 directly, so no custom C++
costmap plugin is needed.

Status: interface only. The detection logic is implemented in Phase 3.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class DepthObstacleNode(Node):
    def __init__(self):
        super().__init__("depth_obstacle_node")

        # Stereo depth error grows with range^2 (f=381 px, baseline=0.12 m): about
        # 0.27 m at 5 m and 0.70 m at 8 m for a 0.5 px disparity error. Keep the
        # trusted range short.
        self.declare_parameter("ground_height_thresh", 0.15)  # m above ground -> obstacle
        self.declare_parameter("drop_thresh", -0.20)          # m below ground -> ditch
        self.declare_parameter("min_range", 0.8)              # m
        self.declare_parameter("max_range", 6.0)              # m
        self.declare_parameter("target_frame", "base_footprint")

        self.sub = self.create_subscription(
            PointCloud2, "/points2", self.cloud_callback, 5
        )
        self.pub = self.create_publisher(PointCloud2, "/perception/tier1_obstacles", 5)

        self._warned = False
        self.get_logger().info("depth_obstacle_node started (Tier 1, no ML)")

    def cloud_callback(self, msg: PointCloud2):
        # TODO (Phase 3):
        #  1. Transform the cloud to `target_frame` with tf2.
        #  2. Keep points with min_range <= range <= max_range.
        #  3. Height above ground > ground_height_thresh -> obstacle;
        #     height < drop_thresh -> ditch. Start with a known camera height and
        #     pitch, upgrade to a RANSAC ground-plane fit for slopes.
        #  4. Voxel-downsample and publish the obstacle points on self.pub.
        if not self._warned:
            self.get_logger().warn(
                "Tier 1 detection is not implemented yet (Phase 3); publishing nothing."
            )
            self._warned = True


def main(args=None):
    rclpy.init(args=args)
    node = DepthObstacleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
