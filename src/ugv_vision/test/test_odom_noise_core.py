import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from ugv_vision import odom_noise_core as on  # noqa: E402


def drive(model, vx, wz, seconds, hz=50.0, t0=0.0):
    """Feed a constant clean twist; returns the last NoisyOdom."""
    out = None
    n = int(seconds * hz)
    for i in range(n + 1):
        out = model.update(t0 + i / hz, vx, wz)
    return out


def test_same_seed_gives_same_systematic_errors():
    a, b = on.OdomNoiseModel(on.NoiseConfig(seed=7)), on.OdomNoiseModel(on.NoiseConfig(seed=7))
    assert (a.scale_v, a.scale_w) == (b.scale_v, b.scale_w)
    assert a.update(0.0, 0.3, 0.0).vx == b.update(0.0, 0.3, 0.0).vx


def test_different_seed_changes_errors():
    errs = {(m.scale_v, m.scale_w) for m in (on.OdomNoiseModel(on.NoiseConfig(seed=s)) for s in range(5))}
    assert len(errs) > 1


def test_scale_error_is_inside_the_requested_range():
    for seed in range(50):
        m = on.OdomNoiseModel(on.NoiseConfig(seed=seed))
        for s in (m.scale_v, m.scale_w):
            assert 0.03 <= abs(s) <= 0.05


def test_noise_disabled_reproduces_the_clean_path():
    cfg = on.NoiseConfig(scale_err_min=0.0, scale_err_max=0.0, yaw_bias=0.0, sigma_v=0.0, sigma_w=0.0)
    m = on.OdomNoiseModel(cfg)
    out = drive(m, 0.4, 0.0, 10.0)
    assert out.x == pytest.approx(4.0, abs=1e-6)
    assert out.y == pytest.approx(0.0, abs=1e-9)
    assert out.yaw == pytest.approx(0.0, abs=1e-12)


def test_straight_distance_error_matches_scale_error():
    cfg = on.NoiseConfig(seed=3, yaw_bias=0.0, sigma_v=0.0, sigma_w=0.0)
    m = on.OdomNoiseModel(cfg)
    out = drive(m, 0.4, 0.0, 50.0)                    # 20 m clean
    assert out.x == pytest.approx(20.0 * (1.0 + m.scale_v), rel=1e-6)
    assert 0.03 * 20.0 <= abs(out.x - 20.0) <= 0.05 * 20.0 + 1e-9


def test_yaw_bias_accumulates_while_moving():
    cfg = on.NoiseConfig(scale_err_min=0.0, scale_err_max=0.0, yaw_bias=0.01, sigma_v=0.0, sigma_w=0.0)
    out = drive(on.OdomNoiseModel(cfg), 0.3, 0.0, 20.0)
    assert out.yaw == pytest.approx(0.2, abs=1e-6)


def test_noise_statistics_are_gaussian_with_the_configured_sigma():
    cfg = on.NoiseConfig(seed=11, scale_err_min=0.0, scale_err_max=0.0, yaw_bias=0.0,
                         sigma_v=0.02, sigma_w=0.01)
    m = on.OdomNoiseModel(cfg)
    vx = np.array([m.update(i * 0.02, 0.3, 0.1).vx for i in range(20000)])
    wz = np.array([m.update(400 + i * 0.02, 0.3, 0.1).wz for i in range(20000)])
    assert vx.mean() == pytest.approx(0.3, abs=0.001) and vx.std() == pytest.approx(0.02, rel=0.05)
    assert wz.mean() == pytest.approx(0.1, abs=0.001) and wz.std() == pytest.approx(0.01, rel=0.05)


def test_stationary_robot_does_not_creep():
    m = on.OdomNoiseModel(on.NoiseConfig(seed=5))
    out = drive(m, 0.0, 0.0, 30.0)
    assert (out.x, out.y, out.yaw, out.vx, out.wz) == (0.0, 0.0, 0.0, 0.0, 0.0)


def test_start_pose_is_the_integration_origin():
    m = on.OdomNoiseModel(on.NoiseConfig(scale_err_min=0.0, scale_err_max=0.0, yaw_bias=0.0,
                                         sigma_v=0.0, sigma_w=0.0))
    m.reset((2.0, -1.0, math.pi / 2))
    out = drive(m, 1.0, 0.0, 3.0)
    assert out.x == pytest.approx(2.0, abs=1e-9)
    assert out.y == pytest.approx(2.0, abs=1e-6)


def test_yaw_is_wrapped():
    cfg = on.NoiseConfig(scale_err_min=0.0, scale_err_max=0.0, yaw_bias=0.0, sigma_v=0.0, sigma_w=0.0)
    out = drive(on.OdomNoiseModel(cfg), 0.0, 1.0, 10.0)   # 10 rad of spin
    assert -math.pi <= out.yaw <= math.pi
    assert out.yaw == pytest.approx(on.wrap_angle(10.0), abs=1e-6)


def test_time_gap_is_clipped_not_integrated_blindly():
    cfg = on.NoiseConfig(scale_err_min=0.0, scale_err_max=0.0, yaw_bias=0.0, sigma_v=0.0, sigma_w=0.0)
    m = on.OdomNoiseModel(cfg)
    m.update(0.0, 1.0, 0.0)
    out = m.update(100.0, 1.0, 0.0)                    # a 100 s gap
    assert out.x == pytest.approx(cfg.max_dt * 1.0)
    out = m.update(99.0, 1.0, 0.0)                     # time going backwards
    assert out.x == pytest.approx(cfg.max_dt * 1.0)


def test_variances_cover_the_injected_systematic_error():
    cfg = on.NoiseConfig(seed=1)
    m = on.OdomNoiseModel(cfg)
    out = m.update(0.0, 0.4, 0.2)
    # the variance is computed from what the EKF can see: the measured (noisy) speeds
    worst_v = cfg.scale_err_max * abs(out.vx)
    worst_w = abs(cfg.yaw_bias) + cfg.scale_err_max * abs(out.wz)
    assert out.var_vx == pytest.approx(cfg.sigma_v ** 2 + worst_v ** 2)
    assert out.var_wz == pytest.approx(cfg.sigma_w ** 2 + worst_w ** 2)
    # and it is at least as large as the real systematic error on the clean speed
    assert math.sqrt(out.var_vx) >= abs(m.scale_v) * 0.4 * 0.9
    assert all(v > 0 for v in (out.var_x, out.var_y, out.var_yaw, out.var_vx, out.var_wz))


def test_pose_variance_grows_with_distance_and_time():
    m = on.OdomNoiseModel(on.NoiseConfig(seed=1))
    early = drive(m, 0.3, 0.0, 5.0)
    late = drive(m, 0.3, 0.0, 60.0, t0=5.02)
    assert late.var_x > early.var_x and late.var_yaw > early.var_yaw


def test_actual_error_stays_inside_the_pose_covariance_bound():
    # the reported pose uncertainty should not be smaller than the error the model really injects
    for seed in range(10):
        m = on.OdomNoiseModel(on.NoiseConfig(seed=seed))
        out = drive(m, 0.3, 0.0, 60.0)
        err = math.hypot(out.x - 0.3 * 60.0, out.y)
        assert err <= 3.0 * math.sqrt(out.var_x) + 1e-9


def test_invalid_config_is_rejected():
    with pytest.raises(ValueError):
        on.OdomNoiseModel(on.NoiseConfig(scale_err_min=0.06, scale_err_max=0.05))
    with pytest.raises(ValueError):
        on.OdomNoiseModel(on.NoiseConfig(sigma_v=-1.0))


def test_quaternion_helpers_roundtrip():
    for a in (0.0, 0.7, -2.5, 3.0):
        assert on.yaw_from_quat(*on.quat_from_yaw(a)) == pytest.approx(a, abs=1e-12)
