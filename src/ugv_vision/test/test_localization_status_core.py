import json
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from ugv_vision import localization_status_core as ls  # noqa: E402


def test_degraded_until_the_first_slam_pose():
    h = ls.LocalizationHealth(1.0)
    s = h.status(5.0)
    assert s == {"mode": "degraded", "slam_ok": False, "age_s": None, "reason": "no_pose_yet"}


def test_fused_while_poses_keep_arriving():
    h = ls.LocalizationHealth(1.0)
    h.on_tracking_ok(True)
    for t in (0.0, 0.1, 0.2):
        h.on_slam_pose(t)
    s = h.status(0.25)
    assert s["mode"] == "fused" and s["slam_ok"] and s["reason"] == "ok"
    assert s["age_s"] == pytest.approx(0.05)


def test_timeout_flips_to_degraded_and_recovers():
    h = ls.LocalizationHealth(1.0)
    h.on_tracking_ok(True)
    h.on_slam_pose(10.0)
    assert h.status(10.9)["mode"] == "fused"
    s = h.status(11.5)
    assert s["mode"] == "degraded" and s["reason"] == "timeout" and not s["slam_ok"]
    h.on_slam_pose(12.0)
    assert h.status(12.1)["mode"] == "fused"


def test_tracking_lost_is_degraded_even_with_a_recent_pose():
    h = ls.LocalizationHealth(1.0)
    h.on_slam_pose(3.0)
    h.on_tracking_ok(False)
    s = h.status(3.1)
    assert s["mode"] == "degraded" and s["reason"] == "tracking_lost"
    h.on_tracking_ok(True)
    assert h.status(3.1)["mode"] == "fused"


def test_pose_without_any_tracking_message_counts_as_ok():
    h = ls.LocalizationHealth(1.0)
    h.on_slam_pose(1.0)
    assert h.status(1.2)["mode"] == "fused"


def test_age_never_negative_when_clock_jumps_back():
    h = ls.LocalizationHealth(1.0)
    h.on_slam_pose(10.0)
    assert h.status(9.0)["age_s"] == 0.0


def test_json_has_the_documented_keys():
    h = ls.LocalizationHealth(1.0)
    h.on_slam_pose(0.0)
    d = json.loads(h.status_json(0.5))
    assert set(d) == {"mode", "slam_ok", "age_s", "reason"}


def test_invalid_timeout_rejected():
    with pytest.raises(ValueError):
        ls.LocalizationHealth(0.0)
