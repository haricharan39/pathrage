"""
Custom Nav2 costmap layer plugin that reads the fused vision-based local
costmap (published by ugv_vision's costmap_fusion_node on
/perception/local_costmap) instead of a lidar-based costmap source.

TODO: implement as a proper nav2_costmap_2d layer plugin (C++ is the
normal path for real Nav2 plugins -- this Python stub is a placeholder to
prototype the data flow before porting to a pluginlib-registered C++ layer).
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid


class VisionCostmapLayer(Node):
    def __init__(self):
        super().__init__("vision_costmap_layer")
        self.sub = self.create_subscription(
            OccupancyGrid, "/perception/local_costmap", self.costmap_callback, 10
        )
        self.get_logger().info("vision_costmap_layer started")

    def costmap_callback(self, msg: OccupancyGrid):
        # TODO: merge into Nav2's local costmap master grid at the
        # appropriate layer priority (above static, below inflation).
        pass


def main(args=None):
    rclpy.init(args=args)
    node = VisionCostmapLayer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
