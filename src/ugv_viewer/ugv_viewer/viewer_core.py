"""ROS-free core of the 3D viewer: point-cloud parsing, rigid transform, voxel map, wire format.

Kept free of rclpy so it can be unit-tested anywhere (see test/test_viewer_core.py).

Wire format (binary WebSocket frames, little endian, all arrays 4-byte aligned):
    byte 0      message type
    bytes 1..3  zero padding
    bytes 4..7  uint32 n (number of records)
  MSG_MAP_DELTA (1):   idx[n] uint32 | pos[n*3] float32 | rgb[n*3] uint8 + pad to 4n | cls[n] uint8 + pad
  MSG_LIVE      (2):   pos[n*3] float32 | rgb[n*3] uint8 (padded)  | cls[n] uint8 (padded)   (replace)
  MSG_TIER1     (3):   pos[n*3] float32                                                     (replace)
Everything else (robot pose, trajectories, plan, goal, stats) is JSON text. Trajectory messages
carry `which`: "driven" (TF base pose), "slam" (/localization/slam_pose) or "gt" (/ground_truth/odom).
"""
import struct
import threading

import numpy as np

MSG_MAP_DELTA = 1
MSG_LIVE = 2
MSG_TIER1 = 3

CLS_GROUND = 0
CLS_OBSTACLE = 1
CLS_DROP = 2

_DTYPES = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
           5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}


def _pad4(n):
    return (-n) % 4


# ----------------------------------------------------------------------------------------
# PointCloud2 parsing
# ----------------------------------------------------------------------------------------
def parse_cloud(data, fields, point_step, width, height, is_bigendian=False, stride=1):
    """Return (xyz float32 [N,3], rgb uint8 [N,3] or None) with non-finite points removed.

    `fields` is a list of (name, offset, datatype) tuples. For organised clouds (height > 1)
    `stride` keeps every stride-th row and column before any other work, which is the
    cheapest way to cut a 640x480 stereo cloud down to something we can process at 5 Hz.
    """
    names = {n: (o, d) for n, o, d in fields}
    for req in ("x", "y", "z"):
        if req not in names:
            raise ValueError(f"PointCloud2 has no '{req}' field")
    endian = ">" if is_bigendian else "<"

    buf = np.frombuffer(data, dtype=np.uint8)
    n_pts = width * height
    if buf.size < n_pts * point_step:
        raise ValueError("PointCloud2 data shorter than width*height*point_step")
    buf = buf[: n_pts * point_step].reshape(height, width, point_step)
    if stride > 1 and height > 1:
        buf = buf[::stride, ::stride, :]
    buf = np.ascontiguousarray(buf).reshape(-1, point_step)

    def col(name):
        off, dt = names[name]
        dtype = np.dtype(_DTYPES[dt]).newbyteorder(endian)
        size = dtype.itemsize
        return buf[:, off:off + size].copy().view(dtype).reshape(-1)

    xyz = np.stack([col("x"), col("y"), col("z")], axis=1).astype(np.float32)
    ok = np.isfinite(xyz).all(axis=1)

    rgb = None
    if "rgb" in names or "rgba" in names:
        key = "rgb" if "rgb" in names else "rgba"
        off, dt = names[key]
        raw = buf[:, off:off + 4].copy().view(np.dtype("<u4") if not is_bigendian
                                              else np.dtype(">u4")).reshape(-1)
        # PCL packs as 0x00RRGGBB (little-endian bytes B, G, R, A)
        rgb = np.stack([(raw >> 16) & 255, (raw >> 8) & 255, raw & 255],
                       axis=1).astype(np.uint8)
    xyz = xyz[ok]
    if rgb is not None:
        rgb = rgb[ok]
    return xyz, rgb


def quat_to_matrix(qx, qy, qz, qw):
    n = qx * qx + qy * qy + qz * qz + qw * qw
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = qx * qx * s, qy * qy * s, qz * qz * s
    xy, xz, yz = qx * qy * s, qx * qz * s, qy * qz * s
    wx, wy, wz = qw * qx * s, qw * qy * s, qw * qz * s
    return np.array([[1 - (yy + zz), xy - wz, xz + wy],
                     [xy + wz, 1 - (xx + zz), yz - wx],
                     [xz - wy, yz + wx, 1 - (xx + yy)]])


def transform_points(xyz, translation, quat_xyzw):
    """p' = R p + t for an [N,3] array."""
    r = quat_to_matrix(*quat_xyzw)
    return (xyz.astype(np.float64) @ r.T + np.asarray(translation, dtype=np.float64)).astype(np.float32)


def pose_to_matrix(translation, quat_xyzw):
    """4x4 matrix of a pose (position + quaternion)."""
    m = np.eye(4)
    m[:3, :3] = quat_to_matrix(*quat_xyzw)
    m[:3, 3] = translation
    return m


def invert_matrix(m):
    out = np.eye(4)
    out[:3, :3] = m[:3, :3].T
    out[:3, 3] = -m[:3, :3].T @ m[:3, 3]
    return out


def align_transform(t_fixed_base, t_world_base):
    """Matrix taking points from the ground-truth `world` frame into the viewer's fixed frame.

    Both arguments are poses of the SAME instant: the robot base in the fixed frame (from TF)
    and the robot base in `world` (from /ground_truth/odom). Display only: the ground truth is
    never used by the robot.
    """
    return t_fixed_base @ invert_matrix(t_world_base)


def apply_matrix(m, xyz):
    """p' = M p for an [N,3] array (or a single point)."""
    pts = np.atleast_2d(np.asarray(xyz, dtype=np.float64))
    return pts @ m[:3, :3].T + m[:3, 3]


# ----------------------------------------------------------------------------------------
# Voxel map
# ----------------------------------------------------------------------------------------
class VoxelMap:
    """Accumulating voxel map. One point per occupied voxel (the voxel centre of the first hit).

    Every voxel has a stable index, so the browser can keep one big GPU buffer and the server
    only ever sends new voxels and class changes (deltas).
    """

    _OFF = 1 << 20          # per-axis offset so packed keys are non-negative
    _MASK = (1 << 21) - 1

    def __init__(self, voxel_size=0.05, max_points=1_500_000, obstacle_min_hits=2):
        self.voxel = float(voxel_size)
        self.max_points = int(max_points)
        self.min_hits = int(obstacle_min_hits)
        self.lock = threading.Lock()
        self._clear_locked()
        self.full_warned = False

    def _clear_locked(self):
        self.index = {}
        self.pos = np.zeros((0, 3), np.float32)
        self.rgb = np.zeros((0, 3), np.uint8)
        self.cls = np.zeros(0, np.uint8)
        self.hits = np.zeros(0, np.uint8)
        self.n = 0
        self.full_warned = False
        self._cap = 0

    def clear(self):
        with self.lock:
            self._clear_locked()

    def __len__(self):
        return self.n

    def _grow(self, need):
        if need <= self._cap:
            return
        cap = max(need, int(self._cap * 1.6) + 4096)
        cap = min(max(cap, need), self.max_points)
        if cap < need:
            cap = need
        for name, shape, dt in (("pos", (cap, 3), np.float32), ("rgb", (cap, 3), np.uint8),
                                ("cls", (cap,), np.uint8), ("hits", (cap,), np.uint8)):
            new = np.zeros(shape, dt)
            old = getattr(self, name)
            new[: old.shape[0]] = old
            setattr(self, name, new)
        self._cap = cap

    def _keys(self, xyz):
        ijk = np.floor(xyz / self.voxel).astype(np.int64) + self._OFF
        ijk = np.clip(ijk, 0, self._MASK)
        return (ijk[:, 0] << 42) | (ijk[:, 1] << 21) | ijk[:, 2]

    def integrate(self, xyz, rgb, cls):
        """Insert points. Returns (changed_idx uint32[], new_count).

        `cls` is the per-point class from this observation. A voxel becomes an obstacle/drop
        only after `obstacle_min_hits` observations, and obstacle status is sticky (cleared
        with the Clear button); this suppresses single-frame stereo noise.
        """
        if xyz.shape[0] == 0:
            return np.zeros(0, np.uint32), 0
        keys = self._keys(xyz)
        uniq, first = np.unique(keys, return_index=True)
        xyz, cls = xyz[first], cls[first]
        rgb = rgb[first] if rgb is not None else np.full((xyz.shape[0], 3), 160, np.uint8)
        centres = ((np.floor(xyz / self.voxel) + 0.5) * self.voxel).astype(np.float32)
        # keep the observed height rather than the voxel centre for z so thin ground reads right
        centres[:, 2] = xyz[:, 2]

        changed = []
        with self.lock:
            lookup = self.index.get
            idxs = np.fromiter((lookup(k, -1) for k in uniq.tolist()), dtype=np.int64,
                               count=uniq.size)
            exist = idxs >= 0

            # existing voxels: count obstacle hits, promote class
            if exist.any():
                ei = idxs[exist]
                ecls = cls[exist]
                is_bad = ecls != CLS_GROUND
                bump = ei[is_bad]
                self.hits[bump] = np.minimum(self.hits[bump].astype(np.int16) + 1, 255).astype(np.uint8)
                promote = bump[(self.hits[bump] >= self.min_hits) & (self.cls[bump] == CLS_GROUND)]
                if promote.size:
                    # class of the promoting observation
                    sel = {int(i): int(c) for i, c in zip(ei[is_bad], ecls[is_bad])}
                    self.cls[promote] = np.array([sel[int(i)] for i in promote], np.uint8)
                    changed.append(promote)

            # new voxels
            new = ~exist
            room = self.max_points - self.n
            n_new = int(new.sum())
            if n_new and room <= 0:
                if not self.full_warned:
                    self.full_warned = True
                n_new = 0
            elif n_new > room:
                sel_new = np.flatnonzero(new)[:room]
                new = np.zeros_like(new)
                new[sel_new] = True
                n_new = room
            if n_new:
                start = self.n
                self._grow(start + n_new)
                sl = slice(start, start + n_new)
                self.pos[sl] = centres[new]
                self.rgb[sl] = rgb[new]
                # a brand-new voxel seen as obstacle on its first hit starts as ground until
                # confirmed, unless min_hits <= 1
                first_cls = cls[new]
                if self.min_hits > 1:
                    self.hits[sl] = (first_cls != CLS_GROUND).astype(np.uint8)
                    self.cls[sl] = CLS_GROUND
                else:
                    self.hits[sl] = (first_cls != CLS_GROUND).astype(np.uint8)
                    self.cls[sl] = first_cls
                new_keys = uniq[new].tolist()
                self.index.update(zip(new_keys, range(start, start + n_new)))
                self.n = start + n_new
                changed.append(np.arange(start, start + n_new, dtype=np.int64))
        out = np.concatenate(changed).astype(np.uint32) if changed else np.zeros(0, np.uint32)
        return out, n_new

    def gather(self, idx):
        with self.lock:
            return self.pos[idx].copy(), self.rgb[idx].copy(), self.cls[idx].copy()

    def snapshot_chunks(self, chunk=60000):
        """Yield MAP_DELTA frames that rebuild the whole map."""
        with self.lock:
            n = self.n
            pos, rgb, cls = self.pos[:n].copy(), self.rgb[:n].copy(), self.cls[:n].copy()
        for s in range(0, n, chunk):
            e = min(n, s + chunk)
            yield pack_map_delta(np.arange(s, e, dtype=np.uint32), pos[s:e], rgb[s:e], cls[s:e])


# ----------------------------------------------------------------------------------------
# Classification (viewer-side preview, NOT the Tier-1 result)
# ----------------------------------------------------------------------------------------
def classify_height(height, obstacle_thresh=0.15, drop_thresh=-0.20):
    cls = np.zeros(height.shape[0], np.uint8)
    cls[height > obstacle_thresh] = CLS_OBSTACLE
    cls[height < drop_thresh] = CLS_DROP
    return cls


# ----------------------------------------------------------------------------------------
# Packing
# ----------------------------------------------------------------------------------------
def _header(msg_type, n):
    return struct.pack("<BBBBI", msg_type, 0, 0, 0, n)


def pack_map_delta(idx, pos, rgb, cls):
    n = idx.shape[0]
    rgb_b = np.ascontiguousarray(rgb, np.uint8).tobytes()
    cls_b = np.ascontiguousarray(cls, np.uint8).tobytes()
    return b"".join([
        _header(MSG_MAP_DELTA, n),
        np.ascontiguousarray(idx, "<u4").tobytes(),
        np.ascontiguousarray(pos, "<f4").tobytes(),
        rgb_b, b"\0" * _pad4(len(rgb_b)),
        cls_b, b"\0" * _pad4(len(cls_b)),
    ])


def pack_live(pos, rgb, cls):
    n = pos.shape[0]
    rgb_b = np.ascontiguousarray(rgb, np.uint8).tobytes()
    cls_b = np.ascontiguousarray(cls, np.uint8).tobytes()
    return b"".join([
        _header(MSG_LIVE, n),
        np.ascontiguousarray(pos, "<f4").tobytes(),
        rgb_b, b"\0" * _pad4(len(rgb_b)),
        cls_b, b"\0" * _pad4(len(cls_b)),
    ])


def pack_tier1(pos):
    return _header(MSG_TIER1, pos.shape[0]) + np.ascontiguousarray(pos, "<f4").tobytes()


def unpack(frame):
    """Inverse of the pack_* functions (used by tests)."""
    t, _, _, _, n = struct.unpack_from("<BBBBI", frame, 0)
    off = 8
    if t == MSG_MAP_DELTA:
        idx = np.frombuffer(frame, "<u4", n, off); off += 4 * n
        pos = np.frombuffer(frame, "<f4", 3 * n, off).reshape(n, 3); off += 12 * n
        rgb = np.frombuffer(frame, np.uint8, 3 * n, off).reshape(n, 3); off += 3 * n + _pad4(3 * n)
        cls = np.frombuffer(frame, np.uint8, n, off)
        return t, idx, pos, rgb, cls
    if t == MSG_LIVE:
        pos = np.frombuffer(frame, "<f4", 3 * n, off).reshape(n, 3); off += 12 * n
        rgb = np.frombuffer(frame, np.uint8, 3 * n, off).reshape(n, 3); off += 3 * n + _pad4(3 * n)
        cls = np.frombuffer(frame, np.uint8, n, off)
        return t, pos, rgb, cls
    if t == MSG_TIER1:
        return t, np.frombuffer(frame, "<f4", 3 * n, off).reshape(n, 3)
    raise ValueError(f"unknown message type {t}")
