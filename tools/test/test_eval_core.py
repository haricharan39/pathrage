import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import eval_core as ec  # noqa: E402


def circle(n=400, radius=5.0, hz=10.0):
    th = np.linspace(0, 2 * math.pi, n)
    t = np.arange(n) / hz
    return np.stack([t, radius * np.cos(th), radius * np.sin(th)], axis=1)


def rigid(traj, yaw, dx, dy):
    c, s = math.cos(yaw), math.sin(yaw)
    out = traj.copy()
    out[:, 1] = c * traj[:, 1] - s * traj[:, 2] + dx
    out[:, 2] = s * traj[:, 1] + c * traj[:, 2] + dy
    return out


def test_rigid_offset_gives_zero_ate():
    gt = circle()
    est = rigid(gt, 0.7, 3.0, -2.0)
    res = ec.summarize(est, gt)
    assert res["ate_rmse_m"] < 1e-9
    assert abs(res["distance_m"] - 2 * math.pi * 5.0) < 0.01


def test_known_noise_level():
    rng = np.random.default_rng(1)
    gt = circle(2000)
    est = gt.copy()
    est[:, 1:] += rng.normal(0, 0.05, size=(len(gt), 2))
    res = ec.summarize(est, gt)
    assert 0.05 * math.sqrt(2) * 0.9 < res["ate_rmse_m"] < 0.05 * math.sqrt(2) * 1.1


def test_scale_drift_detected_per_segment():
    gt = np.stack([np.arange(0, 20.01, 0.1) / 0.5, np.arange(0, 20.01, 0.1), np.zeros(201)], axis=1)
    est = gt.copy()
    est[:, 1] *= 1.05                                  # 5 % too long
    drift = ec.drift_per_segment(est[:, 1:], gt[:, 1:], 10.0)
    assert len(drift) == 2 and all(abs(d[2] - 0.5) < 0.02 for d in drift)


def test_time_alignment_interpolates_and_drops_outside():
    gt = np.array([[0.0, 0.0, 0.0], [1.0, 1.0, 0.0], [2.0, 2.0, 0.0]])
    est = np.array([[-1.0, 9, 9], [0.5, 0.5, 0.0], [1.5, 1.5, 0.0], [5.0, 9, 9]])
    t, e, g = ec.associate(est, gt, max_dt=1.0)
    assert list(t) == [0.5, 1.5] and np.allclose(g[:, 0], [0.5, 1.5])


def test_sparse_ground_truth_is_not_matched():
    gt = np.array([[0.0, 0.0, 0.0], [10.0, 10.0, 0.0]])
    est = np.array([[5.0, 5.0, 0.0]])
    assert len(ec.associate(est, gt, max_dt=0.05)[0]) == 0


def test_unusable_input_raises():
    for est, gt in ((np.zeros((0, 3)), circle()), (circle()[:1], circle())):
        try:
            ec.summarize(est, gt)
            assert False
        except ValueError:
            pass


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
