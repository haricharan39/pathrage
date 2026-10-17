import json
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import fusion_core as fc  # noqa: E402


def line(n=400, length=20.0, hz=20.0, bend=0.0):
    t = np.arange(n) / hz
    x = np.linspace(0, length, n)
    y = bend * (x / length) ** 2
    return np.stack([t, x, y], axis=1)


def test_summary_and_exit_test_pass_when_fused_is_best():
    gt = line()
    noisy = line(bend=1.5)
    slam = line(bend=0.20)
    fused = line(bend=0.05)
    s = fc.evaluate_sources(gt, {"noisy": noisy, "slam": slam, "fused": fused})
    ok, fused_ate, best_ate, best = fc.fused_not_worse(s, tolerance_m=0.0)
    assert ok and best == "slam" and fused_ate < best_ate
    assert s["noisy"]["ate_rmse_m"] > s["slam"]["ate_rmse_m"] > s["fused"]["ate_rmse_m"]


def test_exit_test_fails_when_fused_is_worse_than_the_best_single_source():
    gt = line()
    s = fc.evaluate_sources(gt, {"noisy": line(bend=1.5), "slam": line(bend=0.05), "fused": line(bend=0.4)})
    ok, *_ = fc.fused_not_worse(s, tolerance_m=0.0)
    assert ok is False


def test_tolerance_lets_millimetre_differences_pass():
    gt = line()
    s = fc.evaluate_sources(gt, {"noisy": line(bend=1.5), "slam": line(bend=0.100), "fused": line(bend=0.104)})
    assert fc.fused_not_worse(s, tolerance_m=0.0)[0] is False
    assert fc.fused_not_worse(s, tolerance_m=0.01)[0] is True


def test_missing_sources_are_reported_not_crashing():
    gt = line()
    s = fc.evaluate_sources(gt, {"noisy": line(bend=1.0), "slam": np.zeros((0, 3)), "fused": line()})
    assert s["slam"] is None and s["noisy"] is not None
    assert fc.fused_not_worse(s)[0] is True            # compared with the one source that exists
    assert fc.fused_not_worse({"fused": None, "slam": None, "noisy": None})[0] is None


def test_table_has_a_row_per_route_and_source_and_dashes_for_missing():
    gt = line()
    per_route = {"R1": fc.evaluate_sources(gt, {"noisy": line(bend=1), "slam": line(bend=.1), "fused": line(bend=.05)}),
                 "R2": fc.evaluate_sources(gt, {"noisy": line(bend=1), "slam": np.zeros((0, 3)), "fused": line()})}
    rows = fc.table_rows(per_route)
    assert len(rows) == 6
    md = fc.markdown_table(rows)
    assert md.count("\n") == 2 + 6 - 1
    assert "| R2 | ORB-SLAM3 only | - | - | - | - | - |" in md


def test_status_parsing_and_degraded_time():
    raw = [(0.0, json.dumps({"mode": "fused", "slam_ok": True})),
           (1.0, json.dumps({"mode": "fused", "slam_ok": True})),
           (2.0, json.dumps({"mode": "degraded", "slam_ok": False})),
           (3.0, "not json"),
           (4.0, json.dumps({"mode": "degraded", "slam_ok": False})),
           (5.0, json.dumps({"mode": "fused", "slam_ok": True}))]
    st = fc.parse_status_rows(raw)
    assert len(st) == 5
    d = fc.degraded_summary(st)
    assert d["degraded_s"] == pytest.approx(3.0) and d["first_degraded_t"] == pytest.approx(2.0)
    assert fc.degraded_summary([])["degraded_s"] == 0.0


def test_rate_after():
    t = np.arange(0, 10, 0.1)
    traj = np.stack([t, t, t], axis=1)
    assert fc.rate_after(traj, 5.0) == pytest.approx(10.0)
    assert fc.rate_after(traj, 100.0) == 0.0
    assert fc.rate_after(np.zeros((0, 3)), 0.0) == 0.0
