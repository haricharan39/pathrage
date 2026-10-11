import struct

import numpy as np

from ugv_viewer import viewer_core as vc

# PointCloud2 layout produced by stereo_image_proc: x,y,z float32, rgb packed in a float32 slot
FIELDS = [("x", 0, 7), ("y", 4, 7), ("z", 8, 7), ("rgb", 12, 7)]


def make_cloud(h, w, fill=None):
    rec = np.zeros(h * w, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")])
    ys, xs = np.mgrid[0:h, 0:w]
    rec["x"] = (xs.ravel() - w / 2) * 0.01
    rec["y"] = (ys.ravel() - h / 2) * 0.01
    rec["z"] = 2.0
    rec["rgb"] = (10 << 16) | (200 << 8) | 30  # r=10 g=200 b=30
    if fill is not None:
        fill(rec)
    return rec.tobytes()


def test_parse_drops_nan_and_decodes_rgb():
    def fill(rec):
        rec["z"][5] = np.nan
        rec["x"][7] = np.inf
    data = make_cloud(4, 6, fill)
    xyz, rgb = vc.parse_cloud(data, FIELDS, 16, 6, 4)
    assert xyz.shape == (22, 3) and rgb.shape == (22, 3)
    assert np.all(np.isfinite(xyz))
    assert tuple(rgb[0]) == (10, 200, 30)


def test_parse_stride_on_organised_cloud():
    data = make_cloud(8, 10)
    xyz, _ = vc.parse_cloud(data, FIELDS, 16, 10, 8, stride=2)
    assert xyz.shape[0] == 4 * 5


def test_parse_without_rgb_field():
    rec = np.zeros(3, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4")])
    rec["z"] = 1
    xyz, rgb = vc.parse_cloud(rec.tobytes(), FIELDS[:3], 12, 3, 1)
    assert rgb is None and xyz.shape == (3, 3)


def test_transform_yaw90():
    q = (0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4))
    out = vc.transform_points(np.array([[1.0, 0.0, 0.0]], np.float32), (1, 2, 3), q)
    assert np.allclose(out, [[1.0, 3.0, 3.0]], atol=1e-5)  # (1,0,0) -> (0,1,0), then + (1,2,3)


def test_optical_to_body_rotation():
    # optical (z fwd, x right, y down) -> body (x fwd, y left, z up): rpy = (-pi/2, 0, -pi/2)
    # quaternion for that rotation
    q = (-0.5, 0.5, -0.5, 0.5)
    fwd = vc.transform_points(np.array([[0, 0, 1.0]], np.float32), (0, 0, 0), q)
    right = vc.transform_points(np.array([[1.0, 0, 0]], np.float32), (0, 0, 0), q)
    down = vc.transform_points(np.array([[0, 1.0, 0]], np.float32), (0, 0, 0), q)
    assert np.allclose(fwd, [[1, 0, 0]], atol=1e-6)     # optical z -> body x
    assert np.allclose(right, [[0, -1, 0]], atol=1e-6)  # optical x -> body -y
    assert np.allclose(down, [[0, 0, -1]], atol=1e-6)   # optical y -> body -z


def test_voxel_dedup_and_obstacle_confirmation():
    m = vc.VoxelMap(voxel_size=0.1, obstacle_min_hits=2)
    pts = np.array([[1.01, 0.01, 0.30], [1.03, 0.04, 0.31], [2.0, 0.0, 0.0]], np.float32)
    cls = np.array([vc.CLS_OBSTACLE, vc.CLS_OBSTACLE, vc.CLS_GROUND], np.uint8)
    idx, n_new = m.integrate(pts, None, cls)
    assert n_new == 2 and len(m) == 2           # first two share a voxel
    assert (m.cls[:2] == vc.CLS_GROUND).all()   # not yet confirmed
    idx, n_new = m.integrate(pts, None, cls)    # second sighting
    assert n_new == 0
    assert (m.cls[:2] == vc.CLS_OBSTACLE).sum() == 1
    assert idx.size == 1                        # one class change reported as a delta


def test_voxel_cap():
    m = vc.VoxelMap(voxel_size=0.1, max_points=5)
    pts = np.stack([np.arange(20, dtype=np.float32), np.zeros(20, np.float32),
                    np.zeros(20, np.float32)], axis=1)
    _, n_new = m.integrate(pts, None, np.zeros(20, np.uint8))
    assert n_new == 5 and len(m) == 5
    _, n_new = m.integrate(pts + 100, None, np.zeros(20, np.uint8))
    assert n_new == 0 and len(m) == 5


def test_negative_coordinates():
    m = vc.VoxelMap(voxel_size=0.1)
    pts = np.array([[-3.0, -4.0, -0.2], [-3.0, -4.0, -0.2]], np.float32)
    _, n_new = m.integrate(pts, None, np.zeros(2, np.uint8))
    assert n_new == 1


def test_pack_roundtrip_map_delta_odd_n():
    n = 7  # odd, forces padding
    idx = np.arange(n, dtype=np.uint32) + 100
    pos = np.random.rand(n, 3).astype(np.float32)
    rgb = (np.random.rand(n, 3) * 255).astype(np.uint8)
    cls = (np.arange(n) % 3).astype(np.uint8)
    frame = vc.pack_map_delta(idx, pos, rgb, cls)
    assert len(frame) % 4 == 0
    t, i2, p2, r2, c2 = vc.unpack(frame)
    assert t == vc.MSG_MAP_DELTA
    assert (i2 == idx).all() and np.allclose(p2, pos) and (r2 == rgb).all() and (c2 == cls).all()


def test_pack_roundtrip_live_and_tier1():
    pos = np.random.rand(5, 3).astype(np.float32)
    rgb = np.full((5, 3), 7, np.uint8)
    cls = np.ones(5, np.uint8)
    t, p, r, c = vc.unpack(vc.pack_live(pos, rgb, cls))
    assert t == vc.MSG_LIVE and np.allclose(p, pos) and (r == rgb).all() and (c == cls).all()
    t, p = vc.unpack(vc.pack_tier1(pos))
    assert t == vc.MSG_TIER1 and np.allclose(p, pos)


def test_snapshot_rebuilds_map():
    m = vc.VoxelMap(voxel_size=0.1)
    pts = (np.random.rand(500, 3) * 5).astype(np.float32)
    m.integrate(pts, None, np.zeros(500, np.uint8))
    got = 0
    for fr in m.snapshot_chunks(chunk=128):
        t, idx, pos, rgb, cls = vc.unpack(fr)
        got += idx.size
    assert got == len(m)


def test_classify_height():
    h = np.array([0.0, 0.2, -0.3, 0.1], np.float32)
    c = vc.classify_height(h, 0.15, -0.2)
    assert list(c) == [vc.CLS_GROUND, vc.CLS_OBSTACLE, vc.CLS_DROP, vc.CLS_GROUND]


def test_ground_truth_alignment():
    # robot at (2, 1) heading 90 deg in the fixed frame, but (10, 5) heading 0 in world
    s2 = np.sqrt(0.5)
    t_fixed_base = vc.pose_to_matrix([2.0, 1.0, 0.0], [0.0, 0.0, s2, s2])
    t_world_base = vc.pose_to_matrix([10.0, 5.0, 0.0], [0.0, 0.0, 0.0, 1.0])
    m = vc.align_transform(t_fixed_base, t_world_base)
    assert np.allclose(vc.apply_matrix(m, [10.0, 5.0, 0.0]), [[2.0, 1.0, 0.0]], atol=1e-9)
    # one metre ahead in world x is one metre ahead along the robot's heading (+y in fixed)
    assert np.allclose(vc.apply_matrix(m, [11.0, 5.0, 0.0]), [[2.0, 2.0, 0.0]], atol=1e-9)
    assert np.allclose(vc.invert_matrix(m) @ m, np.eye(4), atol=1e-9)
