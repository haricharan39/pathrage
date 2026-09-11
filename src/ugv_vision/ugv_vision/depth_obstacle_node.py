"""
Tier 1 perception: geometric, depth-based obstacle detection.

No machine learning here on purpose -- this is the deterministic safety
floor described in the project's architecture doc. Any depth pixel whose
height (relative to the assumed ground plane) exceeds `ground_height_thresh`
gets marked as an obstacle, independent of whatever Tier 2 (segmentation)
decides.

Subscribes:
    /camera/depth/image_raw   (sensor_msgs/Image)
Publishes:
    /perception/tier1_obstacles   (sensor_msgs/Image or nav_msgs/OccupancyGrid --
                                    TODO: pick one representation and keep it
                                    consistent with costmap_fusion_node's input)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class DepthObstacleNode(Node):
    def __init__(self):
        super().__init__("depth_obstacle_node")

        self.declare_parameter("ground_height_thresh", 0.15)  # meters
        self.declare_parameter("max_range", 8.0)  # meters

        self.sub = self.create_subscription(
            Image, "/camera/depth/image_raw", self.depth_callback, 10
        )
        self.pub = self.create_publisher(Image, "/perception/tier1_obstacles", 10)

        self.get_logger().info("depth_obstacle_node started (Tier 1, no ML)")

    def depth_callback(self, msg: Image):
        # TODO:
        #  1. Convert Image -> numpy depth array (cv_bridge)
        #  2. Back-project each pixel to a 3D point using camera intrinsics
        #  3. Compute height above the assumed ground plane
        #  4. Threshold against `ground_height_thresh` -> binary obstacle mask
        #  5. Publish the mask (or better: project to a 2D top-down grid here
        #     directly, since that's what costmap_fusion_node ultimately needs)
        pass


def main(args=None):
    rclpy.init(args=args)
    node = DepthObstacleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
