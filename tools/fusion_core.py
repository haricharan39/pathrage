"""ROS-free maths for the Phase 2 results table: noisy wheel odometry vs ORB-SLAM3 vs fused.

Trajectories are numpy arrays with columns [t, x, y] (see eval_core.py). Used by eval_fusion.py;
unit-tested in test/test_fusion_core.py.
"""
import json

import numpy as np

import eval_core

SOURCES = ("noisy", "slam", "fused")
LABELS = {"noisy": "wheel odometry only (noisy)", "slam": "ORB-SLAM3 only", "fused": "EKF fused"}


def evaluate_sources(gt, sources, max_dt=0.05, seg_m=10.0):
    """ATE summary per source: {name: eval_core.summarize(...) dict, or None if unusable}."""
    out = {}
    for name, traj in sources.items():
        try:
            out[name] = eval_core.summarize(traj, gt, max_dt=max_dt, seg_m=seg_m)
        except ValueError:
            out[name] = None
    return out


def fused_not_worse(summary, tolerance_m=0.01):
    """Exit test: fused ATE <= the better of the two single sources (within `tolerance_m`).

    Returns (passed, fused_ate, best_single_ate, best_single_name); passed is None if the fused
    result or both single sources are missing.
    """
    fused = summary.get("fused")
    singles = {n: summary.get(n) for n in ("noisy", "slam") if summary.get(n) is not None}
    if fused is None or not singles:
        return None, None, None, None
    best = min(singles, key=lambda n: singles[n]["ate_rmse_m"])
    best_ate = singles[best]["ate_rmse_m"]
    return fused["ate_rmse_m"] <= best_ate + tolerance_m, fused["ate_rmse_m"], best_ate, best


def drift_mean(summary_entry):
    """Mean alignment-free drift per segment (m), or None."""
    if not summary_entry:
        return None
    return summary_entry.get("drift_mean_per_segment_m")


def table_rows(per_route, tolerance_m=0.01):
    """Flatten {route: {source: summary|None}} into rows for the CSV / markdown table."""
    rows = []
    for route, summary in per_route.items():
        ok, _, _, _ = fused_not_worse(summary, tolerance_m)
        for src in SOURCES:
            s = summary.get(src)
            rows.append({
                "route": route, "source": src,
                "distance_m": None if s is None else s["distance_m"],
                "ate_rmse_m": None if s is None else s["ate_rmse_m"],
                "ate_max_m": None if s is None else s["ate_max_m"],
                "ate_pct": None if s is None else s["ate_pct_of_distance"],
                "drift_per_10m": drift_mean(s),
                "fused_not_worse": ok if src == "fused" else None,
            })
    return rows


def _f(v, fmt):
    return "-" if v is None else format(v, fmt)


def markdown_table(rows):
    lines = ["| Route | Source | Distance (m) | ATE RMSE (m) | ATE max (m) | ATE (% of dist.) | Drift per 10 m (m) |",
             "|---|---|---|---|---|---|---|"]
    for r in rows:
        lines.append(f"| {r['route']} | {LABELS[r['source']]} | {_f(r['distance_m'], '.1f')} | "
                     f"{_f(r['ate_rmse_m'], '.3f')} | {_f(r['ate_max_m'], '.3f')} | "
                     f"{_f(r['ate_pct'], '.2f')} | {_f(r['drift_per_10m'], '.3f')} |")
    return "\n".join(lines)


def parse_status_rows(rows):
    """[(t, json_string), ...] -> [(t, mode, slam_ok)], skipping malformed JSON."""
    out = []
    for t, text in rows:
        try:
            d = json.loads(text)
            out.append((float(t), str(d["mode"]), bool(d["slam_ok"])))
        except (ValueError, KeyError, TypeError):
            continue
    return out


def degraded_summary(status):
    """From [(t, mode, slam_ok)] -> dict with seconds spent degraded and the first flip time."""
    if len(status) < 2:
        return {"degraded_s": 0.0, "first_degraded_t": None, "n": len(status)}
    deg, first = 0.0, None
    for (t0, m0, _), (t1, _, _) in zip(status[:-1], status[1:]):
        if m0 == "degraded":
            deg += max(0.0, t1 - t0)
            if first is None:
                first = t0
    return {"degraded_s": deg, "first_degraded_t": first, "n": len(status)}


def rate_after(traj, t_from):
    """Messages per second of a [t, ...] array from `t_from` to its last sample (0 if < 2)."""
    traj = np.asarray(traj, float)
    if len(traj) == 0:
        return 0.0
    sel = traj[traj[:, 0] >= t_from]
    if len(sel) < 2 or sel[-1, 0] <= sel[0, 0]:
        return 0.0
    return float((len(sel) - 1) / (sel[-1, 0] - sel[0, 0]))
