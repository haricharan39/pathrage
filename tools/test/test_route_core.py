import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import route_core as rc  # noqa: E402


def test_r1_straight_20m():
    x, y, yaw = rc.integrate(rc.build_route("R1"))
    assert abs(x - 20.0) < 0.01 and abs(y) < 1e-6 and abs(yaw) < 1e-9


def test_r2_square_returns_to_start():
    x, y, yaw = rc.integrate(rc.build_route("R2"))
    assert math.hypot(x, y) < 0.05
    assert abs(math.remainder(yaw, 2 * math.pi)) < 0.01


def test_r3_figure8_returns_to_start():
    x, y, yaw = rc.integrate(rc.build_route("R3"))
    assert math.hypot(x, y) < 0.05
    assert abs(math.remainder(yaw, 2 * math.pi)) < 0.01


def test_distance_and_errors():
    assert abs(rc.total_distance(rc.build_route("R1")) - 20.0) < 1e-9
    for bad in ("R9",):
        try:
            rc.build_route(bad)
            assert False
        except ValueError:
            pass
