"""Viewer demo with synthetic data - no ROS, no Gazebo needed.

    ros2 run ugv_viewer viewer_demo            # or: python3 -m ugv_viewer.demo
    open http://127.0.0.1:8080

A fake robot drives a loop through a field of rocks and a slope-free ground, "sees" them with a
stereo-like field of view (80 deg, 0.8-6 m) and feeds the same map/obstacle pipeline as the real
node. Use it to check the browser, GPU and controls on their own: if the demo is smooth but the
real run is not, the problem is in the ROS side (TF, topics, Gazebo CPU load), not the page.
"""
import argparse
import json
import math
import pathlib
import time

import numpy as np

from . import viewer_core as vc
from .server import ViewerServer


def _web_dir():
    here = pathlib.Path(__file__).resolve().parents[1] / "web"
    if (here / "index.html").exists():
        return here
    from ament_index_python.packages import get_package_share_directory
    return pathlib.Path(get_package_share_directory("ugv_viewer")) / "web"


def build_world(rng, n_rocks=14):
    rocks = []
    for _ in range(n_rocks):
        ang = rng.uniform(0, 2 * math.pi)
        rad = rng.uniform(4.0, 9.0)
        rocks.append((rad * math.cos(ang), rad * math.sin(ang), rng.uniform(0.25, 0.5), rng.uniform(0.2, 0.5)))
    return rocks  # x, y, height, radius


def observe(rng, pose, rocks, fov=math.radians(80), rmin=0.8, rmax=6.0, spacing=0.035):
    """Points a stereo camera at `pose` (x, y, yaw) would see: ground + rock surfaces."""
    x, y, yaw = pose
    # ground: polar sampling inside the field of view (more points near the robot, like stereo)
    r = rmin + (rmax - rmin) * rng.random(9000) ** 1.5
    a = rng.uniform(-fov / 2, fov / 2, r.size)
    gx, gy = x + r * np.cos(yaw + a), y + r * np.sin(yaw + a)
    gz = rng.normal(0, 0.008, r.size)
    pts = [np.stack([gx, gy, gz], 1)]
    cols = [np.clip(rng.normal([115, 95, 60], 14, (r.size, 3)), 0, 255)]
    for rx, ry, h, rad in rocks:
        d = math.hypot(rx - x, ry - y)
        bearing = (math.atan2(ry - y, rx - x) - yaw + math.pi) % (2 * math.pi) - math.pi
        if not (rmin < d < rmax and abs(bearing) < fov / 2):
            continue
        n = 700
        zz = rng.uniform(0, h, n)
        face = math.atan2(y - ry, x - rx) + rng.uniform(-1.3, 1.3, n)   # side facing the robot
        px, py = rx + rad * np.cos(face), ry + rad * np.sin(face)
        pts.append(np.stack([px, py, zz], 1))
        cols.append(np.clip(rng.normal([105, 100, 96], 10, (n, 3)), 0, 255))
    return np.concatenate(pts).astype(np.float32), np.concatenate(cols).astype(np.uint8)


def main(argv=None):
    ap = argparse.ArgumentParser(description="3D viewer demo with synthetic data")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--voxel", type=float, default=0.05)
    ap.add_argument("--hz", type=float, default=5.0)
    a = ap.parse_args(argv)

    rng = np.random.default_rng(3)
    rocks = build_world(rng)
    vmap = vc.VoxelMap(a.voxel, 1_500_000, 2)
    traj = []
    cmds = []

    def snapshot():
        return [json.dumps({"type": "hello", "max_points": vmap.max_points, "voxel": vmap.voxel,
                            "fixed_frame": "demo"}),
                *vmap.snapshot_chunks(), json.dumps({"type": "traj_set", "pts": list(traj)})]

    srv = ViewerServer(_web_dir(), a.host, a.port, snapshot, cmds.append)
    srv.start()
    print(f"demo viewer at http://{a.host}:{a.port}  (Ctrl+C to stop)")

    t0 = time.monotonic()
    proc_ms, rate = 0.0, 0.0
    try:
        while True:
            t = time.monotonic() - t0
            # drive a circle of radius 5 m around the origin, heading along the tangent
            w = 0.18
            pose = (5.0 * math.cos(w * t), 5.0 * math.sin(w * t), w * t + math.pi / 2)
            for c in cmds[:]:
                cmds.remove(c)
                if c.get("cmd") == "clear_map":
                    vmap.clear()
                    srv.broadcast_reliable(json.dumps({"type": "cleared", "what": "map"}))
                elif c.get("cmd") == "clear_traj":
                    traj.clear()
                    srv.broadcast_reliable(json.dumps({"type": "cleared", "what": "traj"}))

            p0 = time.perf_counter()
            world, rgb = observe(rng, pose, rocks)
            cls = vc.classify_height(world[:, 2])
            idx, _ = vmap.integrate(world, rgb, cls)
            if idx.size:
                srv.broadcast_reliable(vc.pack_map_delta(idx, *vmap.gather(idx)))
            srv.broadcast_latest("live", vc.pack_live(world[:12000], rgb[:12000], cls[:12000]))
            proc_ms = 0.8 * proc_ms + 0.2 * (time.perf_counter() - p0) * 1000

            q = (0.0, 0.0, math.sin(pose[2] / 2), math.cos(pose[2] / 2))
            if not traj or math.dist(traj[-1][:2], pose[:2]) > 0.05:
                traj.append([round(pose[0], 3), round(pose[1], 3), 0.0])
                srv.broadcast_reliable(json.dumps({"type": "traj_add", "pts": [traj[-1]]}))
            srv.broadcast_latest("state", json.dumps({
                "type": "state", "pose": [pose[0], pose[1], 0.0, *q],
                "stats": {"points": len(vmap), "max_points": vmap.max_points,
                          "proc_ms": round(proc_ms, 1), "rate": a.hz, "clients": srv.client_count,
                          "tf_ok": True, "fixed_frame": "demo (synthetic)"}}))
            time.sleep(1.0 / a.hz)
    except KeyboardInterrupt:
        pass
    finally:
        srv.stop()


if __name__ == "__main__":
    main()
