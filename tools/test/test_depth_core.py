import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import depth_core as dc  # noqa: E402


def _buf(h, w, z, step=16):
    pts = np.zeros((h * w, step // 4), dtype=np.float32)  # x y z pad
    pts[:, 2] = z
    return pts.tobytes()


def test_cloud_roundtrip_and_median():
    h, w = 48, 64
    xyz = dc.cloud_xyz(_buf(h, w, 5.0), 16, {"x": 0, "y": 4, "z": 8}, w, h)
    assert xyz.shape == (h, w, 3)
    s = dc.patch_stats(xyz)
    assert abs(s["median"] - 5.0) < 1e-6 and s["std"] < 1e-6


def test_nan_and_range_filtered():
    xyz = np.full((40, 40, 3), np.nan, dtype=np.float32)
    assert dc.patch_stats(xyz) is None
    xyz[:, :, 2] = 100.0
    assert dc.patch_stats(xyz) is None
    xyz[18:22, 18:22, 2] = 3.0
    assert abs(dc.patch_stats(xyz, frac=0.1)["median"] - 3.0) < 1e-6
