"""3D viewer node: stereo cloud -> accumulated voxel map + obstacles -> browser (WebGL).

Replaces RViz2 for 3D viewing. Open http://127.0.0.1:8080 after launching.

Interface (see docs/INTERFACES.md, section "3D viewer"):
    Subscribes:
        /points2                      sensor_msgs/PointCloud2   stereo_image_proc cloud (optical frame)
        /perception/tier1_obstacles   sensor_msgs/PointCloud2   Tier-1 obstacles (Phase 3), drawn as red overlay
        /plan                         nav_msgs/Path             Nav2 global plan (Phase 4), optional
        /goal_pose                    geometry_msgs/PoseStamped current goal, optional
        TF                            <fixed_frame> <- cloud frame / base_footprint
    Publishes:
        /goal_pose                    geometry_msgs/PoseStamped only when you double-click the ground

The obstacle colouring of the map is a *preview* computed here from height above the robot's
ground plane (same thresholds as depth_obstacle_node). It is not the Tier-1 result.
"""
import json
import math
import pathlib
import queue
import time
import webbrowser

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener

from . import viewer_core as vc
from .server import ViewerServer


def _fields(msg):
    return [(f.name, f.offset, f.datatype) for f in msg.fields]


class ViewerNode(Node):
    def __init__(self):
        super().__init__("viewer_node")
        p = self.declare_parameter
        p("fixed_frame", "odom")            # 'odom' is always available; 'map' once RTAB-Map runs
        p("base_frame", "base_footprint")
        p("cloud_topic", "/points2")
        p("tier1_topic", "/perception/tier1_obstacles")
        p("plan_topic", "/plan")
        p("goal_topic", "/goal_pose")
        p("voxel_size", 0.05)
        p("max_points", 1_500_000)
        p("min_range", 0.8)                 # trusted stereo range, docs/INTERFACES.md section 3
        p("max_range", 6.0)
        p("min_height", -1.0)               # m relative to the robot's ground
        p("max_height", 2.5)
        p("pixel_stride", 2)                # keep every n-th row/column of the organised cloud
        p("process_hz", 5.0)
        p("publish_hz", 10.0)
        p("obstacle_thresh", 0.15)
        p("drop_thresh", -0.20)
        p("obstacle_min_hits", 2)
        p("live_max_points", 15000)
        p("traj_min_step", 0.05)
        p("host", "127.0.0.1")              # use 0.0.0.0 to view from another machine
        p("port", 8080)
        p("open_browser", False)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.fixed = g("fixed_frame")
        self.base = g("base_frame")
        self.min_range, self.max_range = float(g("min_range")), float(g("max_range"))
        self.min_h, self.max_h = float(g("min_height")), float(g("max_height"))
        self.stride = int(g("pixel_stride"))
        self.obs_t, self.drop_t = float(g("obstacle_thresh")), float(g("drop_thresh"))
        self.live_max = int(g("live_max_points"))
        self.traj_step = float(g("traj_min_step"))
        self.proc_period = 1.0 / max(0.1, float(g("process_hz")))

        self.map = vc.VoxelMap(g("voxel_size"), g("max_points"), g("obstacle_min_hits"))
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.traj = []              # list of [x, y, z]
        self._last_plan = None      # cached JSON strings for new clients
        self._last_goal = None
        self._cmds = queue.SimpleQueue()
        self._last_proc = 0.0
        self._last_tier1 = 0.0
        self._proc_ms = 0.0
        self._proc_count = 0
        self._rate_t0 = time.monotonic()
        self._rate = 0.0
        self._tf_ok = False
        self._last_warn = 0.0
        self._map_full_reported = False

        web_dir = pathlib.Path(get_package_share_directory("ugv_viewer")) / "web"
        self.server = ViewerServer(web_dir, g("host"), g("port"),
                                   snapshot_provider=self._snapshot,
                                   command_handler=self._cmds.put,
                                   logger=lambda m: self.get_logger().warn(m))
        self.server.start()
        url = f"http://{g('host') if g('host') != '0.0.0.0' else '127.0.0.1'}:{g('port')}"
        self.get_logger().info(f"3D viewer running at {url}  (fixed frame '{self.fixed}')")
        if g("open_browser"):
            try:
                webbrowser.open(url)
            except Exception as exc:
                self.get_logger().warn(f"could not open a browser: {exc}")

        self.create_subscription(PointCloud2, g("cloud_topic"), self._on_cloud, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, g("tier1_topic"), self._on_tier1, 5)
        self.create_subscription(Path, g("plan_topic"), self._on_plan, 5)
        self.create_subscription(PoseStamped, g("goal_topic"), self._on_goal, 5)
        self.goal_pub = self.create_publisher(PoseStamped, g("goal_topic"), 5)
        self.create_timer(1.0 / max(1.0, float(g("publish_hz"))), self._on_timer)

    # ------------------------------------------------------------------ TF helpers
    def _lookup(self, source, stamp):
        """(translation, quat_xyzw) taking points from `source` into the fixed frame."""
        for t in (stamp, Time()):
            try:
                tf = self.tf_buffer.lookup_transform(self.fixed, source, t,
                                                     timeout=Duration(seconds=0.05))
                tr, q = tf.transform.translation, tf.transform.rotation
                self._tf_ok = True
                return (tr.x, tr.y, tr.z), (q.x, q.y, q.z, q.w)
            except Exception as exc:
                err = exc
        self._tf_ok = False
        now = time.monotonic()
        if now - self._last_warn > 5.0:
            self._last_warn = now
            self.get_logger().warn(f"waiting for TF {self.fixed} <- {source}: {err}")
        return None

    # ------------------------------------------------------------------ callbacks
    def _on_cloud(self, msg):
        now = time.monotonic()
        if now - self._last_proc < self.proc_period:
            return
        self._last_proc = now
        t0 = time.perf_counter()
        try:
            xyz, rgb = vc.parse_cloud(bytes(msg.data), _fields(msg), msg.point_step, msg.width,
                                      msg.height, msg.is_bigendian, self.stride)
        except ValueError as exc:
            self.get_logger().warn(f"bad cloud: {exc}", throttle_duration_sec=5.0)
            return
        if xyz.shape[0] == 0:
            return
        rng = np.linalg.norm(xyz, axis=1)
        keep = (rng >= self.min_range) & (rng <= self.max_range)
        xyz = xyz[keep]
        rgb = rgb[keep] if rgb is not None else None
        if xyz.shape[0] == 0:
            return

        stamp = Time.from_msg(msg.header.stamp)
        tf = self._lookup(msg.header.frame_id, stamp)
        if tf is None:
            return
        world = vc.transform_points(xyz, *tf)

        # ground reference = the robot's own z in the fixed frame (flat-ground assumption, same
        # as the first version of Tier 1)
        base = self._lookup(self.base, stamp)
        ground_z = base[0][2] if base else 0.0
        height = world[:, 2] - ground_z
        ok = (height >= self.min_h) & (height <= self.max_h)
        world, height = world[ok], height[ok]
        rgb = rgb[ok] if rgb is not None else None
        if world.shape[0] == 0:
            return
        cls = vc.classify_height(height, self.obs_t, self.drop_t)

        idx, _ = self.map.integrate(world, rgb, cls)
        if idx.size:
            pos, col, c = self.map.gather(idx)
            self.server.broadcast_reliable(vc.pack_map_delta(idx, pos, col, c))
        if len(self.map) >= self.map.max_points and not self._map_full_reported:
            self._map_full_reported = True
            self.get_logger().warn(
                f"map is full ({self.map.max_points} voxels); new areas are no longer added. "
                "Raise max_points or voxel_size, or press Clear map in the viewer.")

        # live frame (what the camera sees right now), capped
        n = world.shape[0]
        if n > self.live_max:
            sel = np.random.choice(n, self.live_max, replace=False)
        else:
            sel = slice(None)
        live_rgb = rgb[sel] if rgb is not None else np.full((min(n, self.live_max), 3), 200, np.uint8)
        self.server.broadcast_latest("live", vc.pack_live(world[sel], live_rgb, cls[sel]))

        self._proc_ms = 0.8 * self._proc_ms + 0.2 * (time.perf_counter() - t0) * 1000.0
        self._proc_count += 1

    def _on_tier1(self, msg):
        now = time.monotonic()
        if now - self._last_tier1 < 0.1:
            return
        self._last_tier1 = now
        try:
            xyz, _ = vc.parse_cloud(bytes(msg.data), _fields(msg), msg.point_step, msg.width,
                                    msg.height, msg.is_bigendian, 1)
        except ValueError:
            return
        tf = self._lookup(msg.header.frame_id, Time.from_msg(msg.header.stamp))
        if tf is None:
            return
        if xyz.shape[0] > 20000:
            xyz = xyz[np.random.choice(xyz.shape[0], 20000, replace=False)]
        self.server.broadcast_latest("tier1", vc.pack_tier1(vc.transform_points(xyz, *tf)))

    def _on_plan(self, msg):
        if not msg.poses:
            return
        pts = np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z]
                        for p in msg.poses], np.float32)
        tf = self._lookup(msg.header.frame_id, Time())
        if tf is None:
            return
        pts = vc.transform_points(pts, *tf)
        if len(pts) > 400:
            pts = pts[np.linspace(0, len(pts) - 1, 400).astype(int)]
        self._last_plan = json.dumps({"type": "plan", "pts": np.round(pts, 3).tolist()})
        self.server.broadcast_reliable(self._last_plan)

    def _on_goal(self, msg):
        tf = self._lookup(msg.header.frame_id, Time())
        if tf is None:
            return
        pt = vc.transform_points(np.array([[msg.pose.position.x, msg.pose.position.y,
                                            msg.pose.position.z]], np.float32), *tf)[0]
        self._last_goal = json.dumps({"type": "goal", "pos": np.round(pt, 3).tolist()})
        self.server.broadcast_reliable(self._last_goal)

    # ------------------------------------------------------------------ periodic work
    def _on_timer(self):
        self._drain_commands()
        now = time.monotonic()
        dt = now - self._rate_t0
        if dt >= 2.0:
            self._rate = self._proc_count / dt
            self._proc_count = 0
            self._rate_t0 = now

        tf = None
        try:
            t = self.tf_buffer.lookup_transform(self.fixed, self.base, Time())
            tr, q = t.transform.translation, t.transform.rotation
            tf = ([tr.x, tr.y, tr.z], [q.x, q.y, q.z, q.w])
        except Exception:
            pass

        stats = {"points": len(self.map), "max_points": self.map.max_points,
                 "proc_ms": round(self._proc_ms, 1), "rate": round(self._rate, 1),
                 "clients": self.server.client_count, "tf_ok": tf is not None and self._tf_ok,
                 "fixed_frame": self.fixed}
        state = {"type": "state", "stats": stats}
        if tf is not None:
            state["pose"] = [round(v, 4) for v in tf[0] + tf[1]]
            last = self.traj[-1] if self.traj else None
            if last is None or math.dist(last, tf[0]) >= self.traj_step:
                self.traj.append([round(v, 3) for v in tf[0]])
                self.server.broadcast_reliable(json.dumps({"type": "traj_add",
                                                           "pts": [self.traj[-1]]}))
        self.server.broadcast_latest("state", json.dumps(state))

    def _drain_commands(self):
        while True:
            try:
                cmd = self._cmds.get_nowait()
            except queue.Empty:
                return
            name = cmd.get("cmd")
            if name == "clear_map":
                self.map.clear()
                self._map_full_reported = False
                self.server.broadcast_reliable(json.dumps({"type": "cleared", "what": "map"}))
            elif name == "clear_traj":
                self.traj.clear()
                self.server.broadcast_reliable(json.dumps({"type": "cleared", "what": "traj"}))
            elif name == "goal":
                self._publish_goal(float(cmd["x"]), float(cmd["y"]))

    def _publish_goal(self, x, y):
        yaw = 0.0
        try:
            t = self.tf_buffer.lookup_transform(self.fixed, self.base, Time())
            yaw = math.atan2(y - t.transform.translation.y, x - t.transform.translation.x)
        except Exception:
            pass
        goal = PoseStamped()
        goal.header.frame_id = self.fixed
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = x, y
        goal.pose.orientation.z, goal.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.goal_pub.publish(goal)
        self.get_logger().info(f"goal published from viewer: ({x:.2f}, {y:.2f}) in '{self.fixed}'")

    # ------------------------------------------------------------------ new-client snapshot
    def _snapshot(self):
        frames = [json.dumps({"type": "hello", "max_points": self.map.max_points,
                              "voxel": self.map.voxel, "fixed_frame": self.fixed})]
        frames += list(self.map.snapshot_chunks())
        frames.append(json.dumps({"type": "traj_set", "pts": list(self.traj)}))
        for cached in (self._last_plan, self._last_goal):
            if cached:
                frames.append(cached)
        return frames

    def destroy_node(self):
        try:
            self.server.stop()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ViewerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
