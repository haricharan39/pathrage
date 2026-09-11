"""
Tier 2 perception: learned terrain segmentation.

Classifies each pixel of the RGB frame into safe-terrain vs. hazard classes
(grass/dirt vs. mud/ditch/dense-brush) -- the semantic judgment call that
Tier 1's pure geometry can't make (e.g. flat mud and flat solid ground sit
at the same height).

Model: Fast-SCNN or BiSeNetV2, trained offline on RUGD/RELLIS-3D (see
/training), exported to ONNX and run here via onnxruntime for real-time
inference.

Subscribes:
    /camera/rgb/image_raw      (sensor_msgs/Image)
Publishes:
    /perception/tier2_terrain  (sensor_msgs/Image  -- per-pixel class mask)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class SegmentationNode(Node):
    def __init__(self):
        super().__init__("segmentation_node")

        self.declare_parameter("model_path", "models/terrain_seg.onnx")
        self.declare_parameter("input_size", [512, 512])

        # TODO: load the ONNX model here via onnxruntime.InferenceSession
        # self.session = onnxruntime.InferenceSession(
        #     self.get_parameter("model_path").value
        # )

        self.sub = self.create_subscription(
            Image, "/camera/rgb/image_raw", self.image_callback, 10
        )
        self.pub = self.create_publisher(Image, "/perception/tier2_terrain", 10)

        self.get_logger().info("segmentation_node started (Tier 2, learned)")

    def image_callback(self, msg: Image):
        # TODO:
        #  1. Convert Image -> numpy array (cv_bridge), resize/normalize
        #     to match the model's expected input
        #  2. Run ONNX inference -> per-pixel class logits
        #  3. Argmax -> class mask (safe / hazard / unknown)
        #  4. Publish the class mask for costmap_fusion_node to merge with
        #     Tier 1's output
        pass


def main(args=None):
    rclpy.init(args=args)
    node = SegmentationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
