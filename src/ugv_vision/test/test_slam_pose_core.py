import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from ugv_vision import slam_pose_core as sp  # noqa: E402


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    m = np.eye(4)
    m[:2, :2] = [[c, -s], [s, c]]
    return m


def trans(x, y, z=0.0):
    m = np.eye(4)
    m[:3, 3] = [x, y, z]
    return m


# base_footprint <- left_camera_optical_frame for the URDF (camera 0.27 forward, 0.30 up,
# optical rotation rpy = (-pi/2, 0, -pi/2) = columns [0,-1,0], [0,0,-1], [1,0,0]).
R_OPT = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
T_BASE_CAM = np.eye(4)
T_BASE_CAM[:3, :3] = R_OPT
T_BASE_CAM[:3, 3] = [0.27, 0.06, 0.30]


def cam_pose_in_orb(base_in_start):
    """What ORB-SLAM3 would report: camera pose in a world whose origin is the first camera.

    `base_in_start` is the base pose relative to where the robot started (ROS axes).
    """
    t_start_cam0 = T_BASE_CAM            # the first camera frame, expressed at the start pose
    t_start_cam = base_in_start @ T_BASE_CAM
    return np.linalg.inv(t_start_cam0) @ t_start_cam


def test_quaternion_roundtrip():
    for a in (0.0, 0.3, 2.0, -3.0):
        r = rot_z(a)[:3, :3]
        assert np.allclose(sp.quat_to_matrix(*sp.matrix_to_quat(r)), r, atol=1e-9)
    r = R_OPT
    assert np.allclose(sp.quat_to_matrix(*sp.matrix_to_quat(r)), r, atol=1e-9)


def test_straight_line_in_odom_with_offset_start():
    odom0 = trans(5.0, -2.0) @ rot_z(0.5)     # robot did not start at the odom origin
    conv = sp.SlamPoseConverter(T_BASE_CAM)
    out = None
    for i in range(0, 101, 10):
        base = trans(i * 0.1, 0.0)             # drives 10 m straight along its own x
        out = conv.process(cam_pose_in_orb(base), i * 0.1, odom0)
        odom0 = None if conv.initialised else odom0
    T, cov, info = out
    expect = trans(5.0, -2.0) @ rot_z(0.5) @ trans(10.0, 0.0)
    assert np.allclose(T, expect, atol=1e-9)
    assert not info["jump"]
    assert abs(cov[0, 0] - 0.05 ** 2) < 1e-12 and abs(cov[5, 5] - 0.03 ** 2) < 1e-12


def test_turn_in_place_gives_yaw_only():
    conv = sp.SlamPoseConverter(T_BASE_CAM)
    conv.process(cam_pose_in_orb(np.eye(4)), 0.0, np.eye(4))
    T, _, _ = conv.process(cam_pose_in_orb(rot_z(0.4)), 0.1)
    assert abs(sp.yaw_of(T) - 0.4) < 1e-9
    assert np.allclose(T[:3, 3], 0.0, atol=1e-9)


def test_waits_for_odom_before_initialising():
    conv = sp.SlamPoseConverter(T_BASE_CAM)
    assert conv.process(cam_pose_in_orb(np.eye(4)), 0.0, None) is None
    assert not conv.initialised
    assert conv.process(cam_pose_in_orb(np.eye(4)), 0.1, np.eye(4)) is not None


def test_rejects_non_finite():
    conv = sp.SlamPoseConverter(T_BASE_CAM)
    bad = cam_pose_in_orb(np.eye(4))
    bad[0, 3] = float("nan")
    assert conv.process(bad, 0.0, np.eye(4)) is None


def test_jump_inflates_then_recovers():
    conv = sp.SlamPoseConverter(T_BASE_CAM, inflate_hold=3)
    conv.process(cam_pose_in_orb(np.eye(4)), 0.0, np.eye(4))
    _, cov, info = conv.process(cam_pose_in_orb(trans(0.01, 0.0)), 0.033)
    assert not info["jump"] and cov[0, 0] < 0.01
    _, cov, info = conv.process(cam_pose_in_orb(trans(4.0, 0.0)), 0.066)   # 4 m in 33 ms
    assert info["jump"] and cov[0, 0] > 0.04
    for k in range(5):
        _, cov, info = conv.process(cam_pose_in_orb(trans(4.0, 0.0)), 0.1 + 0.033 * k)
    assert cov[0, 0] < 0.01
    assert conv.jumps == 1


def test_new_map_is_reanchored_continuously():
    conv = sp.SlamPoseConverter(T_BASE_CAM, inflate_hold=2)
    conv.on_tracking_state(sp.STATE_OK)
    conv.process(cam_pose_in_orb(np.eye(4)), 0.0, np.eye(4))
    T1, _, _ = conv.process(cam_pose_in_orb(trans(3.0, 0.0)), 1.0)
    # tracking lost, ORB-SLAM3 starts a new map (state goes back to NO_IMAGES_YET / 1)
    conv.on_tracking_state(sp.STATE_LOST)
    conv.on_tracking_state(sp.STATE_NO_IMAGES_YET)
    conv.on_tracking_state(sp.STATE_NOT_INITIALIZED)
    conv.on_tracking_state(sp.STATE_OK)
    # new map: origin is wherever the robot is now, so the first pose is ~identity
    T2, cov, info = conv.process(cam_pose_in_orb(np.eye(4)), 5.0)
    assert info["reanchored"] and not info["jump"] and conv.reanchors == 1
    assert np.allclose(T2, T1, atol=1e-9)
    assert cov[0, 0] > 0.04
    # and motion in the new map is added on top
    T3, _, _ = conv.process(cam_pose_in_orb(trans(1.0, 0.0)), 6.0)
    assert np.allclose(T3[:3, 3], T1[:3, 3] + [1.0, 0.0, 0.0], atol=1e-9)


def test_relocalisation_in_same_map_is_not_reanchored():
    conv = sp.SlamPoseConverter(T_BASE_CAM)
    conv.on_tracking_state(sp.STATE_OK)
    conv.process(cam_pose_in_orb(np.eye(4)), 0.0, np.eye(4))
    conv.on_tracking_state(sp.STATE_RECENTLY_LOST)
    conv.on_tracking_state(sp.STATE_OK)
    T, _, info = conv.process(cam_pose_in_orb(trans(0.5, 0.0)), 1.0)
    assert not info["reanchored"] and np.allclose(T[:3, 3], [0.5, 0, 0], atol=1e-9)


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
