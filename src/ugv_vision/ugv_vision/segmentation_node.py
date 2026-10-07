"""
Tier 2 perception: learned terrain segmentation.

Classifies each pixel of the RGB frame into traversable / caution / hazard /
unknown -- the semantic judgment that Tier 1's pure geometry cannot make (flat mud
and flat solid ground sit at the same height).

Model: Fast-SCNN, trained offline on RUGD / RELLIS-3D (see /training), exported to
ONNX and run here with onnxruntime.

Interface (see docs/INTERFACES.md):
    Subscribes:
        /stereo_camera/left/image_rect_color  sensor_msgs/Image (rgb8, rectified left)
    Publishes:
        /perception/tier2_terrain             sensor_msgs/Image (mono8 class ids)
        /perception/tier2_confidence          sensor_msgs/Image (mono8, 0-255 = softmax max)

The confidence image is what lets the fusion node fall back to Tier 1 when the model
is unsure.

Status: interface only. Implemented in Phase 6.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class SegmentationNode(Node):
    def __init__(self):
        super().__init__("segmentation_node")

        self.declare_parameter("model_path", "models/terrain_seg.onnx")
        self.declare_parameter("input_size", [512, 512])
        self.declare_parameter("confidence_threshold", 0.6)

        # TODO (Phase 6): load the ONNX model with onnxruntime.InferenceSession.

        self.sub = self.create_subscription(
            Image, "/stereo_camera/left/image_rect_color", self.image_callback, 5
        )
        self.pub_terrain = self.create_publisher(Image, "/perception/tier2_terrain", 5)
        self.pub_conf = self.create_publisher(Image, "/perception/tier2_confidence", 5)

        self._warned = False
        self.get_logger().info("segmentation_node started (Tier 2, learned)")

    def image_callback(self, msg: Image):
        # TODO (Phase 6):
        #  1. Image -> numpy (cv_bridge), resize/normalize to the model input.
        #  2. ONNX inference -> per-pixel class logits.
        #  3. Softmax -> class mask (argmax) and confidence (max prob).
        #  4. Publish both images with the incoming header.
        if not self._warned:
            self.get_logger().warn(
                "Tier 2 segmentation is not implemented yet (Phase 6); publishing nothing."
            )
            self._warned = True


def main(args=None):
    rclpy.init(args=args)
    node = SegmentationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
