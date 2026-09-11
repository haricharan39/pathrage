"""
Exports a trained segmentation checkpoint to ONNX for real-time inference
in ugv_vision/segmentation_node.py.

Run:
    python export_onnx.py --checkpoint terrain_seg.pth --output terrain_seg.onnx
"""
import argparse


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--output", default="terrain_seg.onnx")
    parser.add_argument("--input-size", type=int, nargs=2, default=[512, 512])
    args = parser.parse_args()

    # TODO:
    #  1. Load the trained PyTorch model from args.checkpoint
    #  2. torch.onnx.export(...) with a dummy input of args.input_size
    #  3. Copy the resulting .onnx file into
    #     src/ugv_vision/models/terrain_seg.onnx
    raise NotImplementedError("Fill in the export logic.")


if __name__ == "__main__":
    main()
