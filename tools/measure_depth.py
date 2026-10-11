#!/usr/bin/env python3
"""Measure stereo depth at the image centre and compare with a known distance.

Place a flat box/wall in front of the robot in Gazebo, measure the distance from
the camera to its face (e.g. from the ground-truth/model pose), then:

  python3 tools/measure_depth.py --truth 5.0 --frames 20

Prints median depth, std and error per frame batch. Run it for 1, 3, 5 and 8 m.
Not verified on a ROS machine yet.
"""
import argparse
import os
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import depth_core  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from sensor_msgs.msg import PointCloud2  # noqa: E402


class Probe(Node):
    def __init__(self, frames, frac):
        super().__init__("depth_probe", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.frames, self.frac, self.meds = frames, frac, []
        self.create_subscription(PointCloud2, "/points2", self.cb, 2)

    def cb(self, msg):
        if msg.height <= 1:
            self.get_logger().error("cloud is not organized (height == 1); cannot take a centre patch")
            self.meds = None
            return
        off = {f.name: f.offset for f in msg.fields}
        xyz = depth_core.cloud_xyz(bytes(msg.data), msg.point_step, off, msg.width, msg.height)
        s = depth_core.patch_stats(xyz, self.frac)
        if s:
            self.meds.append(s["median"])
        if len(self.meds) >= self.frames:
            self.meds = self.meds[: self.frames]


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--truth", type=float, help="true distance to the target, m")
    p.add_argument("--frames", type=int, default=20)
    p.add_argument("--frac", type=float, default=0.05, help="centre patch half-size (fraction)")
    a = p.parse_args()
    rclpy.init()
    n = Probe(a.frames, a.frac)
    while rclpy.ok() and n.meds is not None and len(n.meds) < a.frames:
        rclpy.spin_once(n, timeout_sec=0.5)
    if n.meds:
        m, sd = statistics.median(n.meds), statistics.pstdev(n.meds)
        msg = f"depth median {m:.3f} m  (frame-to-frame std {sd:.3f} m, {len(n.meds)} frames)"
        if a.truth:
            msg += f"  truth {a.truth:.2f} m  error {m - a.truth:+.3f} m ({100 * (m - a.truth) / a.truth:+.1f}%)"
        print(msg)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
