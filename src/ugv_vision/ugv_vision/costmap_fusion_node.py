"""
Merges Tier 1 (depth-geometric) and Tier 2 (learned segmentation) outputs
into the single local costmap layer that ugv_navigation's vision_costmap_layer
plugin reads.

Fusion rule (see architecture doc for rationale):
    - Tier 1 is a hard safety floor: anything it flags as an obstacle stays
      an obstacle no matter what Tier 2 says.
    - Tier 2 adds semantic nuance on top (e.g. flagging mud that Tier 1's
      pure height threshold would miss).

Subscribes:
    /perception/tier1_obstacles
    /perception/tier2_terrain
Publishes:
    /perception/local_costmap   (nav_msgs/OccupancyGrid)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from message_filters import Subscriber, ApproximateTimeSynchronizer


class CostmapFusionNode(Node):
    def __init__(self):
        super().__init__("costmap_fusion_node")

        self.tier1_sub = Subscriber(self, Image, "/perception/tier1_obstacles")
        self.tier2_sub = Subscriber(self, Image, "/perception/tier2_terrain")
        self.sync = ApproximateTimeSynchronizer(
            [self.tier1_sub, self.tier2_sub], queue_size=10, slop=0.1
        )
        self.sync.registerCallback(self.fusion_callback)

        self.pub = self.create_publisher(OccupancyGrid, "/perception/local_costmap", 10)

        self.get_logger().info("costmap_fusion_node started")

    def fusion_callback(self, tier1_msg: Image, tier2_msg: Image):
        # TODO:
        #  1. Convert both masks to numpy
        #  2. OR the two obstacle masks together, with Tier 1 always winning
        #     (i.e. Tier 2 can only ADD obstacle cells, never remove ones
        #     Tier 1 flagged)
        #  3. Project the merged mask from image space to a top-down
        #     OccupancyGrid using the ground-plane/camera calibration
        #  4. Publish
        pass


def main(args=None):
    rclpy.init(args=args)
    node = CostmapFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
