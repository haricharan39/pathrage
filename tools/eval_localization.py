#!/usr/bin/env python3
"""Localization error (ATE) of /localization/slam_pose against /ground_truth/odom.

Record while a route is driven (simulation + orbslam running):
    python3 tools/eval_localization.py record R1 --duration 80
    # in another terminal: python3 tools/drive_route.py R1 --speed 0.4
Ctrl-C ends the recording early; --duration is in SIM seconds.

Re-analyse saved CSVs without ROS:
    python3 tools/eval_localization.py analyze results/R1_slam.csv results/R1_gt.csv

Prints ATE RMSE after a rigid SE(2) alignment (no scale: stereo is metric), ATE as % of the
distance travelled, and the error per 10 m. Gate 1 asks for ATE well under 5 % of distance.
Ground truth is used here and never by the robot. Not run on a ROS machine yet.
"""
import argparse
import csv
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import eval_core  # noqa: E402

RESULTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results")


def write_csv(path, rows, header):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def read_csv(path):
    with open(path) as f:
        r = csv.reader(f)
        next(r)
        return np.array([[float(v) for v in row[:3]] for row in r if row], float).reshape(-1, 3)


def report(est, gt, label, max_dt, seg_m):
    try:
        s = eval_core.summarize(est, gt, max_dt=max_dt, seg_m=seg_m)
    except ValueError as exc:
        print(f"{label}: cannot evaluate: {exc}")
        return None
    print(f"\n=== {label} ===")
    print(f"  pairs {s['n_pairs']}, {s['duration_s']:.0f} s, ground-truth distance {s['distance_m']:.1f} m")
    print(f"  ATE RMSE {s['ate_rmse_m']:.3f} m  (max {s['ate_max_m']:.3f} m)  "
          f"= {s['ate_pct_of_distance']:.2f} % of distance")
    if s["drift_segments"]:
        print(f"  drift per {seg_m:.0f} m (displacement error, alignment-free):")
        for a, b, e in s["drift_segments"]:
            print(f"    {a:6.1f} - {b:6.1f} m : {e:.3f} m")
    verdict = "PASS" if s["ate_pct_of_distance"] < 5.0 else "FAIL"
    print(f"  Gate 1 (ATE < 5 % of distance): {verdict}   [sim only; 'well under' is your call]")
    return s


def cmd_analyze(a):
    report(read_csv(a.slam_csv), read_csv(a.gt_csv), os.path.basename(a.slam_csv), a.max_dt, a.segment)


def cmd_record(a):
    import rclpy
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from std_msgs.msg import Bool

    class Recorder(Node):
        def __init__(self):
            super().__init__("eval_localization", parameter_overrides=[Parameter("use_sim_time", value=True)])
            self.slam, self.gt, self.ok = [], [], []
            self.create_subscription(PoseWithCovarianceStamped, "/localization/slam_pose", self.on_slam, 50)
            self.create_subscription(Odometry, "/ground_truth/odom", self.on_gt, 100)
            self.create_subscription(Bool, "/orb_slam3/tracking_ok", self.on_ok, 50)

        @staticmethod
        def stamp(h):
            return h.stamp.sec + h.stamp.nanosec * 1e-9

        def on_slam(self, m):
            p = m.pose.pose.position
            self.slam.append((self.stamp(m.header), p.x, p.y))

        def on_gt(self, m):
            p = m.pose.pose.position
            self.gt.append((self.stamp(m.header), p.x, p.y))

        def on_ok(self, m):
            self.ok.append(bool(m.data))

    rclpy.init()
    node = Recorder()
    t_start = None
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            now = node.get_clock().now().nanoseconds * 1e-9
            if now == 0.0:
                continue
            t_start = now if t_start is None else t_start
            if a.duration and now - t_start >= a.duration:
                break
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

    slam_p, gt_p = os.path.join(a.out, f"{a.name}_slam.csv"), os.path.join(a.out, f"{a.name}_gt.csv")
    write_csv(slam_p, node.slam, ["t", "x", "y"])
    write_csv(gt_p, node.gt, ["t", "x", "y"])
    print(f"saved {len(node.slam)} slam poses -> {slam_p}, {len(node.gt)} ground-truth poses -> {gt_p}")
    if node.ok:
        flips = sum(1 for i in range(1, len(node.ok)) if node.ok[i - 1] and not node.ok[i])
        print(f"tracking OK in {100.0 * sum(node.ok) / len(node.ok):.0f} % of frames, lost {flips} time(s)")
    if not node.slam:
        print("no /localization/slam_pose received: ORB-SLAM3 never initialised (needs texture + motion)")
        return
    report(np.array(node.slam), np.array(node.gt), a.name, a.max_dt, a.segment)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("record", help="record live topics, save CSVs and print the report")
    r.add_argument("name", help="run name, e.g. R1")
    r.add_argument("--duration", type=float, default=0.0, help="sim seconds to record (0 = until Ctrl-C)")
    r.add_argument("--out", default=RESULTS, help="output directory (default: results/)")
    r.set_defaults(fn=cmd_record)
    z = sub.add_parser("analyze", help="evaluate two saved CSVs (t,x,y)")
    z.add_argument("slam_csv")
    z.add_argument("gt_csv")
    z.set_defaults(fn=cmd_analyze)
    for s in (r, z):
        s.add_argument("--max-dt", type=float, default=0.05, help="max time gap to ground truth (s)")
        s.add_argument("--segment", type=float, default=10.0, help="drift segment length (m)")
    a = p.parse_args()
    a.fn(a)


if __name__ == "__main__":
    main()
