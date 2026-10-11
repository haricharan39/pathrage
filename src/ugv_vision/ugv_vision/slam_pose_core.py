"""ROS-free maths for turning an ORB-SLAM3 camera pose into a base pose in the odom frame.

Used by localization_node.py; unit-tested in test/test_slam_pose_core.py without ROS.

Frames
    orb_map   ORB-SLAM3 world frame. Origin = first frame, camera convention
              (z forward, x right, y down). The wrapper publishes T_orb_cam, the pose of the
              LEFT OPTICAL camera in orb_map (the inverse of ORB-SLAM3's Tcw).
    base      base_footprint (REP-103: x forward, y left, z up).
    odom      the odom frame the EKF and Nav2 work in.

Conversion, for every accepted pose
    T_orb_base = T_orb_cam * inv(T_base_cam)          T_base_cam = static TF extrinsics
    T_rel      = inv(T_orb_base_0) * T_orb_base       motion since the first accepted frame
    T_odom_base = T_odom_base_0 * T_rel               T_odom_base_0 = odom pose at that frame
Scale is metric already (stereo), so no scale is estimated.

Failure handling
    * Poses are only accepted while tracking is OK (the caller drops the rest).
    * A jump larger than what the robot can physically move since the last accepted pose
      inflates the covariance for `inflate_hold` messages, so the EKF trusts it less.
    * If ORB-SLAM3 started a NEW map (tracking state went back to 0 / 1 after it had been
      running) its origin is arbitrary. The first pose of the new map is re-anchored to the
      last published pose, so the output stays continuous, with an inflated covariance.
"""
import math

import numpy as np

# ORB-SLAM3 Tracking::eTrackingState
STATE_NO_IMAGES_YET = 0
STATE_NOT_INITIALIZED = 1
STATE_OK = 2
STATE_RECENTLY_LOST = 3
STATE_LOST = 4


# ---------------------------------------------------------------------------------------
# SE(3) helpers (4x4 matrices)
# ---------------------------------------------------------------------------------------
def quat_to_matrix(x, y, z, w):
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([[1 - (yy + zz), xy - wz, xz + wy],
                     [xy + wz, 1 - (xx + zz), yz - wx],
                     [xz - wy, yz + wx, 1 - (xx + yy)]])


def matrix_to_quat(r):
    """Rotation matrix -> (x, y, z, w), w >= 0."""
    t = r[0, 0] + r[1, 1] + r[2, 2]
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w, x, y, z = 0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        w, x, y, z = (r[2, 1] - r[1, 2]) / s, 0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        w, x, y, z = (r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        w, x, y, z = (r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    q /= np.linalg.norm(q)
    if q[3] < 0:
        q = -q
    return tuple(float(v) for v in q)


def pose_to_matrix(position, quat_xyzw):
    m = np.eye(4)
    m[:3, :3] = quat_to_matrix(*quat_xyzw)
    m[:3, 3] = position
    return m


def matrix_to_pose(m):
    return tuple(float(v) for v in m[:3, 3]), matrix_to_quat(m[:3, :3])


def invert(m):
    out = np.eye(4)
    out[:3, :3] = m[:3, :3].T
    out[:3, 3] = -m[:3, :3].T @ m[:3, 3]
    return out


def yaw_of(m):
    """Heading (rotation about z) of a pose matrix, for a z-up frame."""
    return math.atan2(m[1, 0], m[0, 0])


def wrap_angle(a):
    return math.remainder(a, 2.0 * math.pi)


# ---------------------------------------------------------------------------------------
# Converter
# ---------------------------------------------------------------------------------------
class SlamPoseConverter:
    """Stateful ORB-SLAM3 -> odom pose converter. Pure Python + numpy, no ROS."""

    def __init__(self, t_base_cam, sigma_xy=0.05, sigma_z=0.10, sigma_rp=0.05, sigma_yaw=0.03,
                 inflate_factor=10.0, inflate_hold=30, max_jump_m=0.3, max_speed=2.0,
                 max_jump_rad=0.3, max_yaw_rate=3.0, max_dt=5.0):
        self.t_cam_base = invert(np.asarray(t_base_cam, dtype=float))
        self.sigmas = (sigma_xy, sigma_xy, sigma_z, sigma_rp, sigma_rp, sigma_yaw)
        self.inflate_factor = float(inflate_factor)
        self.inflate_hold = int(inflate_hold)
        self.max_jump_m, self.max_speed = float(max_jump_m), float(max_speed)
        self.max_jump_rad, self.max_yaw_rate = float(max_jump_rad), float(max_yaw_rate)
        self.max_dt = float(max_dt)
        self.reset()

    def reset(self):
        self.t_orb_base0 = None     # first accepted base pose in orb_map
        self.t_odom_base0 = None    # odom -> base at that moment
        self.last_out = None        # last published T_odom_base
        self.last_stamp = None
        self.state = STATE_NO_IMAGES_YET
        self._seen_ok = False
        self._new_map = False
        self._inflate_left = 0
        self.jumps = 0
        self.reanchors = 0

    @property
    def initialised(self):
        return self.t_orb_base0 is not None

    @property
    def inflated(self):
        return self._inflate_left > 0

    # ------------------------------------------------------------------ state input
    def on_tracking_state(self, state):
        """Feed the raw ORB-SLAM3 tracking state (every frame)."""
        self.state = int(state)
        if self.state == STATE_OK:
            self._seen_ok = True
        elif self.state in (STATE_NO_IMAGES_YET, STATE_NOT_INITIALIZED) and self._seen_ok:
            # ORB-SLAM3 abandoned the old map and is building a new one with a new origin.
            self._new_map = True
            self._seen_ok = False

    # ------------------------------------------------------------------ pose input
    def process(self, t_orb_cam, stamp_s, t_odom_base_now=None):
        """Convert one OK pose. Returns (T_odom_base 4x4, covariance 6x6, info) or None.

        `t_odom_base_now` (odom -> base_footprint at `stamp_s`) is only needed while the
        converter is not initialised yet; when it is missing then, the frame is skipped.
        """
        t_orb_cam = np.asarray(t_orb_cam, dtype=float)
        if not np.isfinite(t_orb_cam).all():
            return None
        t_orb_base = t_orb_cam @ self.t_cam_base
        info = {"initialised": False, "jump": False, "reanchored": False}

        if not self.initialised:
            if t_odom_base_now is None:
                return None
            self._anchor(t_orb_base, np.asarray(t_odom_base_now, dtype=float))
            info["initialised"] = True
        elif self._new_map:
            # Keep the output continuous: the first pose of the new map is mapped onto the
            # last published pose.
            self._anchor(t_orb_base, self.last_out)
            self._new_map = False
            self.reanchors += 1
            self._inflate_left = self.inflate_hold
            info["reanchored"] = True

        out = self.t_odom_base0 @ (invert(self.t_orb_base0) @ t_orb_base)

        if self.last_out is not None and not info["initialised"] and not info["reanchored"]:
            dt = 0.0 if self.last_stamp is None else min(max(stamp_s - self.last_stamp, 0.0), self.max_dt)
            dist = float(np.linalg.norm(out[:3, 3] - self.last_out[:3, 3]))
            dyaw = abs(wrap_angle(yaw_of(out) - yaw_of(self.last_out)))
            if dist > self.max_jump_m + self.max_speed * dt or dyaw > self.max_jump_rad + self.max_yaw_rate * dt:
                info["jump"] = True
                self.jumps += 1
                self._inflate_left = self.inflate_hold

        cov = self._covariance()
        if self._inflate_left > 0:
            self._inflate_left -= 1
        self.last_out = out
        self.last_stamp = stamp_s
        return out, cov, info

    # ------------------------------------------------------------------ internals
    def _anchor(self, t_orb_base, t_odom_base):
        self.t_orb_base0 = t_orb_base
        self.t_odom_base0 = t_odom_base

    def _covariance(self):
        k = self.inflate_factor if self._inflate_left > 0 else 1.0
        cov = np.zeros((6, 6))
        for i, s in enumerate(self.sigmas):
            cov[i, i] = (s * k) ** 2
        return cov
