"""
Trains the Tier 2 terrain segmentation model (Fast-SCNN or BiSeNetV2) on
RUGD/RELLIS-3D, fine-tuned on site-specific footage where available.

Run on the Victus GPU:
    python train_segmentation.py --dataset datasets/rugd --epochs 50

TODO: implement the actual training loop (dataset loader, model definition,
loss, optimizer). This is currently a structural stub only.
"""
import argparse


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--epochs", type=int, default=50)
    parser.add_argument("--batch-size", type=int, default=16)
    parser.add_argument("--output", default="terrain_seg.pth")
    args = parser.parse_args()

    # TODO:
    #  1. Load RUGD/RELLIS-3D dataset (or site-specific footage) from args.dataset
    #  2. Define Fast-SCNN or BiSeNetV2 model (torchvision / segmentation-models-pytorch)
    #  3. Train with standard segmentation loss (cross-entropy / dice)
    #  4. Save checkpoint to args.output
    raise NotImplementedError("Fill in the training loop.")


if __name__ == "__main__":
    main()
