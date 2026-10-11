"""ROS-free helpers for measuring stereo depth accuracy from a PointCloud2 buffer."""
import numpy as np


def cloud_xyz(data, point_step, offsets, width, height):
    """Return (height, width, 3) float32 array from raw PointCloud2 bytes.

    offsets: dict with byte offsets for 'x', 'y', 'z' (all float32).
    """
    raw = np.frombuffer(data, dtype=np.uint8).reshape(height * width, point_step)
    out = np.empty((height * width, 3), dtype=np.float32)
    for i, k in enumerate("xyz"):
        o = offsets[k]
        out[:, i] = raw[:, o:o + 4].copy().view(np.float32).reshape(-1)
    return out.reshape(height, width, 3)


def patch_stats(xyz, frac=0.1, zmin=0.1, zmax=30.0):
    """Median/std/count of optical-frame z (forward range) in a central patch.

    frac: patch half-size as a fraction of the image (0.1 -> central 20% x 20%).
    """
    h, w, _ = xyz.shape
    r0, r1 = int(h * (0.5 - frac)), int(h * (0.5 + frac))
    c0, c1 = int(w * (0.5 - frac)), int(w * (0.5 + frac))
    z = xyz[r0:r1, c0:c1, 2].ravel()
    z = z[np.isfinite(z) & (z > zmin) & (z < zmax)]
    if z.size == 0:
        return None
    return {"median": float(np.median(z)), "std": float(np.std(z)), "n": int(z.size)}
