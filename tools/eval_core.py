"""ROS-free maths for localization evaluation: time alignment, rigid SE(2) alignment, ATE, drift.

Trajectories are numpy arrays with columns [t, x, y] (seconds, metres), time-sorted.
Used by eval_localization.py; unit-tested in test/test_eval_core.py.
"""
import numpy as np


def path_length(xy):
    xy = np.asarray(xy, float)
    if len(xy) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum())


def associate(est, gt, max_dt=0.05):
    """Match each estimate to the ground-truth sample interpolated at its time.

    Returns (t, est_xy, gt_xy) for estimates whose time lies inside the ground-truth time span
    and within `max_dt` of a ground-truth sample. Ground truth is linearly interpolated.
    """
    est, gt = np.asarray(est, float), np.asarray(gt, float)
    if len(est) == 0 or len(gt) < 2:
        return np.zeros(0), np.zeros((0, 2)), np.zeros((0, 2))
    t = est[:, 0]
    inside = (t >= gt[0, 0]) & (t <= gt[-1, 0])
    idx = np.clip(np.searchsorted(gt[:, 0], t), 1, len(gt) - 1)
    near = np.minimum(np.abs(gt[idx, 0] - t), np.abs(gt[idx - 1, 0] - t)) <= max_dt
    keep = inside & near
    t = t[keep]
    g = np.stack([np.interp(t, gt[:, 0], gt[:, 1]), np.interp(t, gt[:, 0], gt[:, 2])], axis=1)
    return t, est[keep, 1:3], g


def align_se2(est_xy, gt_xy):
    """Least-squares rigid transform (rotation + translation, no scale) est -> gt (Umeyama/Horn).

    Returns (R 2x2, t 2,). Needs >= 2 distinct points.
    """
    est_xy, gt_xy = np.asarray(est_xy, float), np.asarray(gt_xy, float)
    if len(est_xy) < 2:
        raise ValueError("need at least 2 points to align")
    mu_e, mu_g = est_xy.mean(axis=0), gt_xy.mean(axis=0)
    h = (est_xy - mu_e).T @ (gt_xy - mu_g)
    u, _, vt = np.linalg.svd(h)
    d = np.sign(np.linalg.det(vt.T @ u.T))
    r = vt.T @ np.diag([1.0, d]) @ u.T
    return r, mu_g - r @ mu_e


def ate_rmse(est_xy, gt_xy, align=True):
    """Absolute trajectory error RMSE (m) after optional rigid SE(2) alignment, and the errors."""
    est_xy, gt_xy = np.asarray(est_xy, float), np.asarray(gt_xy, float)
    if align:
        r, t = align_se2(est_xy, gt_xy)
        est_xy = est_xy @ r.T + t
    err = np.linalg.norm(est_xy - gt_xy, axis=1)
    return float(np.sqrt(np.mean(err ** 2))), err


def drift_per_segment(est_xy, gt_xy, seg_m=10.0):
    """Relative error per `seg_m` of ground-truth path: [(start_m, end_m, error_m), ...].

    The error of a segment is the difference between the estimated and the true displacement
    over it, so it does not depend on any global alignment.
    """
    est_xy, gt_xy = np.asarray(est_xy, float), np.asarray(gt_xy, float)
    if len(gt_xy) < 2:
        return []
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(gt_xy, axis=0), axis=1))])
    out, start = [], 0.0
    while start + seg_m <= s[-1] + 1e-9:
        i0, i1 = np.searchsorted(s, start), np.searchsorted(s, start + seg_m)
        i1 = min(i1, len(s) - 1)
        d_est = est_xy[i1] - est_xy[i0]
        d_gt = gt_xy[i1] - gt_xy[i0]
        out.append((float(s[i0]), float(s[i1]), float(np.linalg.norm(d_est - d_gt))))
        start += seg_m
    return out


def summarize(est, gt, max_dt=0.05, seg_m=10.0):
    """Everything eval_localization.py prints, as a dict. Raises ValueError if unusable."""
    t, e, g = associate(est, gt, max_dt)
    if len(t) < 2:
        raise ValueError("fewer than 2 estimates overlap the ground truth in time")
    dist = path_length(g)
    if dist < 1e-6:
        raise ValueError("ground truth did not move")
    rmse, err = ate_rmse(e, g)
    drift = drift_per_segment(e, g, seg_m)
    return {
        "n_pairs": int(len(t)),
        "duration_s": float(t[-1] - t[0]),
        "distance_m": dist,
        "ate_rmse_m": rmse,
        "ate_max_m": float(err.max()),
        "ate_pct_of_distance": 100.0 * rmse / dist,
        "drift_segments": drift,
        "drift_mean_per_segment_m": float(np.mean([d[2] for d in drift])) if drift else None,
    }
