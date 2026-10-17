"""ROS-free noise model for the wheel odometry (Phase 2, step 2.1).

Gazebo's wheel odometry is almost perfect, so fusing it with ORB-SLAM3 would prove nothing.
This module turns the clean twist into a *realistically wrong* one and integrates it into a
dead-reckoned pose, so the EKF test is honest. `odom_noise_node.py` wraps it for ROS.

Injected errors (all seeded, so every run of a route sees the same systematic errors):

    scale error   the measured speed is wrong by a fixed factor 3-5 % (sign drawn once from the
                  seed), independently for the forward speed and the yaw rate (wheel slip)
    yaw bias      a constant offset on the yaw rate (rad/s), applied while the robot moves
    white noise   Gaussian noise on every sample, forward speed and yaw rate

Errors are applied only while the robot is commanded to move: encoders read zero at rest, so a
stationary robot does not creep.

Covariances: the EKF treats every message as an independent measurement, so it cannot know about
the slowly varying systematic errors. They are therefore counted as noise: the reported variance
is the per-sample noise variance plus the largest systematic error squared. The filter then trusts
the wheel odometry no more than it deserves. The pose covariance grows with distance and time and
is informational only (the EKF fuses velocities from this topic).

Unit-tested in test/test_odom_noise_core.py.
"""
import math
from dataclasses import dataclass

import numpy as np


@dataclass
class NoiseConfig:
    seed: int = 42
    scale_err_min: float = 0.03     # fraction, magnitude range of the speed-scale error
    scale_err_max: float = 0.05
    yaw_bias: float = 0.003         # rad/s added to the yaw rate while moving
    sigma_v: float = 0.02           # m/s, white noise per sample on the forward speed
    sigma_w: float = 0.01           # rad/s, white noise per sample on the yaw rate
    moving_thresh_v: float = 0.005  # m/s
    moving_thresh_w: float = 0.005  # rad/s
    max_dt: float = 0.5             # s, integration steps are clipped to this

    def validate(self):
        if not 0.0 <= self.scale_err_min <= self.scale_err_max:
            raise ValueError("need 0 <= scale_err_min <= scale_err_max")
        if self.sigma_v < 0 or self.sigma_w < 0:
            raise ValueError("sigma_v and sigma_w must be >= 0")
        if self.max_dt <= 0:
            raise ValueError("max_dt must be > 0")


@dataclass
class NoisyOdom:
    """One output sample: dead-reckoned pose, noisy body twist and the variances to publish."""
    x: float
    y: float
    yaw: float
    vx: float
    wz: float
    var_x: float
    var_y: float
    var_yaw: float
    var_vx: float
    var_wz: float


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_from_yaw(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class OdomNoiseModel:
    """Stateful: feed it the clean (stamp, vx, wz) of every /odom message in order."""

    def __init__(self, cfg=None):
        self.cfg = cfg or NoiseConfig()
        self.cfg.validate()
        rng = np.random.default_rng(self.cfg.seed)
        c = self.cfg

        def draw():
            return float(rng.choice([-1.0, 1.0]) * rng.uniform(c.scale_err_min, c.scale_err_max))

        # Drawn once, in this fixed order, so a seed always gives the same systematic errors.
        self.scale_v = draw()
        self.scale_w = draw()
        self._rng = rng
        self.reset()

    def reset(self, pose=(0.0, 0.0, 0.0)):
        self.x, self.y, self.yaw = float(pose[0]), float(pose[1]), float(pose[2])
        self._t_prev = None
        self._dist = 0.0        # path length integrated so far (m)
        self._t_moving = 0.0    # seconds spent moving
        self._turned = 0.0      # |yaw| turned so far (rad)

    @property
    def scale_bound(self):
        return self.cfg.scale_err_max

    def _variances(self, vx, wz):
        c = self.cfg
        var_vx = c.sigma_v ** 2 + (c.scale_err_max * abs(vx)) ** 2
        var_wz = c.sigma_w ** 2 + (abs(c.yaw_bias) + c.scale_err_max * abs(wz)) ** 2
        s_xy = c.scale_err_max * self._dist + 0.01
        # heading error: bias over the time moving, scale error over the angle turned, and the
        # random walk of the white noise (sigma_w * sqrt(dt * t) ~ sigma_w * 0.14 * sqrt(t) at 50 Hz)
        s_yaw = (abs(c.yaw_bias) * self._t_moving + c.scale_err_max * self._turned
                 + c.sigma_w * 0.14 * math.sqrt(max(self._t_moving, 0.0)) + 0.005)
        return var_vx, var_wz, s_xy ** 2, s_yaw ** 2

    def update(self, t, vx, wz):
        """Process the clean twist measured at time `t` (s); returns a NoisyOdom."""
        c = self.cfg
        moving = abs(vx) > c.moving_thresh_v or abs(wz) > c.moving_thresh_w
        if moving:
            vx_n = vx * (1.0 + self.scale_v) + self._rng.normal(0.0, c.sigma_v)
            wz_n = wz * (1.0 + self.scale_w) + c.yaw_bias + self._rng.normal(0.0, c.sigma_w)
        else:
            vx_n = wz_n = 0.0

        if self._t_prev is not None:
            dt = min(max(t - self._t_prev, 0.0), c.max_dt)
            if dt > 0.0:
                yaw_mid = self.yaw + 0.5 * wz_n * dt
                self.x += vx_n * math.cos(yaw_mid) * dt
                self.y += vx_n * math.sin(yaw_mid) * dt
                self.yaw = wrap_angle(self.yaw + wz_n * dt)
                self._dist += abs(vx_n) * dt
                self._turned += abs(wz_n) * dt
                if moving:
                    self._t_moving += dt
        self._t_prev = t

        var_vx, var_wz, var_xy, var_yaw = self._variances(vx_n, wz_n)
        return NoisyOdom(self.x, self.y, self.yaw, vx_n, wz_n,
                         var_xy, var_xy, var_yaw, var_vx, var_wz)
