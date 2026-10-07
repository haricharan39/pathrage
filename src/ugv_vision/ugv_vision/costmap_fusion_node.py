"""
Merges Tier 1 (depth-geometric) and Tier 2 (learned segmentation) outputs into the
single obstacle point cloud that Nav2's ObstacleLayer reads.

Fusion rule:
    - Tier 1 is a hard safety floor: anything it flags stays an obstacle no matter
      what Tier 2 says. Tier 2 can only ADD cost, never remove it.
    - Tier 2 adds semantic hazards (e.g. mud that a height threshold misses), but
      only where its confidence is above `confidence_threshold`. Where it is unsure,
      this node adds nothing and the system behaves as Tier 1 only.

Interface (see docs/INTERFACES.md):
    Subscribes:
        /perception/tier1_obstacles   sensor_msgs/PointCloud2
        /perception/tier2_terrain     sensor_msgs/Image (mono8 class ids)
        /perception/tier2_confidence  sensor_msgs/Image (mono8)
    Publishes:
        /perception/obstacle_points   sensor_msgs/PointCloud2  (frame base_footprint)

Status: interface only. Implemented in Phase 7.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from message_filters import Subscriber, ApproximateTimeSynchronizer


class CostmapFusionNode(Node):
    def __init__(self):
        super().__init__("costmap_fusion_node")

        # mode: "fused" | "tier1_only" | "tier2_only" (ablation switch, Phase 7)
        self.declare_parameter("mode", "fused")
        self.declare_parameter("confidence_threshold", 0.6)

        self.tier1_sub = Subscriber(self, PointCloud2, "/perception/tier1_obstacles")
        self.tier2_sub = Subscriber(self, Image, "/perception/tier2_terrain")
        self.conf_sub = Subscriber(self, Image, "/perception/tier2_confidence")
        self.sync = ApproximateTimeSynchronizer(
            [self.tier1_sub, self.tier2_sub, self.conf_sub], queue_size=10, slop=0.1
        )
        self.sync.registerCallback(self.fusion_callback)

        self.pub = self.create_publisher(PointCloud2, "/perception/obstacle_points", 5)

        self._warned = False
        self.get_logger().info("costmap_fusion_node started")

    def fusion_callback(self, tier1: PointCloud2, terrain: Image, confidence: Image):
        # TODO (Phase 7):
        #  1. Start from the Tier 1 obstacle points (never remove any).
        #  2. Where confidence >= threshold and the class is hazard/caution,
        #     back-project those pixels to the ground plane and append them as points.
        #  3. Honour `mode` for the tier1_only / tier2_only / fused ablations.
        #  4. Publish on self.pub.
        if not self._warned:
            self.get_logger().warn(
                "Fusion is not implemented yet (Phase 7); publishing nothing."
            )
            self._warned = True


def main(args=None):
    rclpy.init(args=args)
    node = CostmapFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
