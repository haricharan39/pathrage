#!/usr/bin/env python3
"""Phase 2 evaluation: noisy wheel odometry vs ORB-SLAM3 vs EKF fused, against ground truth.

Record one route (simulation running with orbslam:=true odom_noise:=true):
    python3 tools/eval_fusion.py record R1 --duration 80
    # in another terminal: python3 tools/drive_route.py R1 --speed 0.4
Kill ORB-SLAM3 mid-route to test the fallback (the stack must keep running, status -> degraded):
    python3 tools/eval_fusion.py record R2 --duration 220 --kill-orbslam-at 90
Build the results table from the saved runs (R1, R2, R3 by default):
    python3 tools/eval_fusion.py table
Re-analyse one saved run without ROS:
    python3 tools/eval_fusion.py analyze results/phase2/R1

Files per run in results/phase2/: <name>_noisy.csv (/odom_noisy), <name>_slam.csv
(/localization/slam_pose), <name>_fused.csv (/odometry/filtered), <name>_gt.csv
(/ground_truth/odom), <name>_status.csv (/localization/status). Columns t,x,y (status: t,json).

ATE is computed like in Phase 1 (rigid SE(2) alignment, no scale) so the numbers are comparable;
the alignment-free drift per 10 m is printed next to it because the alignment can hide a slowly
growing heading error. Ground truth is used here and never by the robot. Not run on a ROS machine.
"""
import argparse
import csv
import os
import subprocess
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fusion_core  # noqa: E402

RESULTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results", "phase2")
ORB_PROCESS_PATTERN = "lib/orbslam3/stereo"     # the wrapper's executable, see launch/orbslam.launch.py


def write_csv(path, rows, header):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def read_xy(path):
    if not os.path.exists(path):
        return np.zeros((0, 3))
    with open(path) as f:
        r = csv.reader(f)
        next(r, None)
        return np.array([[float(v) for v in row[:3]] for row in r if row], float).reshape(-1, 3)


def read_status(path):
    if not os.path.exists(path):
        return []
    with open(path) as f:
        r = csv.reader(f)
        next(r, None)
        return fusion_core.parse_status_rows([(row[0], row[1]) for row in r if len(row) >= 2])


def load_run(prefix):
    return {
        "gt": read_xy(prefix + "_gt.csv"),
        "sources": {n: read_xy(f"{prefix}_{n}.csv") for n in fusion_core.SOURCES},
        "status": read_status(prefix + "_status.csv"),
    }


def print_report(name, run, max_dt, seg_m, tolerance):
    summary = fusion_core.evaluate_sources(run["gt"], run["sources"], max_dt, seg_m)
    print(f"\n=== {name} ===")
    for src in fusion_core.SOURCES:
        s = summary[src]
        label = fusion_core.LABELS[src]
        if s is None:
            print(f"  {label:30s} no usable data ({len(run['sources'][src])} samples)")
            continue
        drift = fusion_core.drift_mean(s)
        print(f"  {label:30s} ATE {s['ate_rmse_m']:.3f} m (max {s['ate_max_m']:.3f}) = "
              f"{s['ate_pct_of_distance']:.2f} % of {s['distance_m']:.1f} m, "
              f"drift/10 m {'-' if drift is None else format(drift, '.3f')} m")
    ok, fused, best, best_name = fusion_core.fused_not_worse(summary, tolerance)
    if ok is None:
        print("  exit test: cannot evaluate (need the fused result and at least one single source)")
    else:
        print(f"  exit test: fused {fused:.3f} m vs best single source ({best_name}) {best:.3f} m "
              f"(tolerance {tolerance:.3f} m): {'PASS' if ok else 'FAIL'}")
    d = fusion_core.degraded_summary(run["status"])
    if d["n"]:
        print(f"  status: {d['n']} messages, degraded for {d['degraded_s']:.1f} s"
              + ("" if d["first_degraded_t"] is None else f", first degraded at t={d['first_degraded_t']:.1f}"))
    return summary


def cmd_analyze(a):
    run = load_run(a.prefix)
    print_report(os.path.basename(a.prefix), run, a.max_dt, a.segment, a.tolerance)


def cmd_table(a):
    per_route = {}
    for route in a.routes:
        prefix = os.path.join(a.dir, route)
        if not os.path.exists(prefix + "_gt.csv"):
            print(f"{route}: no {prefix}_gt.csv, skipped")
            continue
        per_route[route] = fusion_core.evaluate_sources(
            read_xy(prefix + "_gt.csv"),
            {n: read_xy(f"{prefix}_{n}.csv") for n in fusion_core.SOURCES}, a.max_dt, a.segment)
    if not per_route:
        print("nothing to tabulate")
        return
    rows = fusion_core.table_rows(per_route, a.tolerance)
    md = fusion_core.markdown_table(rows)
    print(md)
    for route, summary in per_route.items():
        ok, fused, best, best_name = fusion_core.fused_not_worse(summary, a.tolerance)
        verdict = "n/a" if ok is None else ("PASS" if ok else "FAIL")
        print(f"{route}: fused <= best single source (tolerance {a.tolerance:.3f} m): {verdict}")
    write_csv(os.path.join(a.dir, "ate_table.csv"),
              [[r["route"], r["source"], r["distance_m"], r["ate_rmse_m"], r["ate_max_m"], r["ate_pct"],
                r["drift_per_10m"], r["fused_not_worse"]] for r in rows],
              ["route", "source", "distance_m", "ate_rmse_m", "ate_max_m", "ate_pct", "drift_per_10m_m",
               "fused_not_worse"])
    with open(os.path.join(a.dir, "ate_table.md"), "w") as f:
        f.write(md + "\n")
    print(f"\nwritten: {os.path.join(a.dir, 'ate_table.csv')} and ate_table.md")


def cmd_record(a):
    import json
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from std_msgs.msg import Bool, String

    class Recorder(Node):
        def __init__(self):
            super().__init__("eval_fusion", parameter_overrides=[Parameter("use_sim_time", value=True)])
            self.data = {"noisy": [], "slam": [], "fused": [], "gt": []}
            self.status, self.ok = [], []
            self.create_subscription(Odometry, "/odom_noisy", lambda m: self.add("noisy", m), 100)
            self.create_subscription(PoseWithCovarianceStamped, "/localization/slam_pose",
                                     lambda m: self.add("slam", m), 50)
            self.create_subscription(Odometry, "/odometry/filtered", lambda m: self.add("fused", m), 100)
            self.create_subscription(Odometry, "/ground_truth/odom", lambda m: self.add("gt", m), 100)
            self.create_subscription(String, "/localization/status", self.on_status, 20)
            self.create_subscription(Bool, "/orb_slam3/tracking_ok", lambda m: self.ok.append(bool(m.data)), 50)

        @staticmethod
        def stamp(h):
            return h.stamp.sec + h.stamp.nanosec * 1e-9

        def add(self, key, m):
            p = m.pose.pose.position          # Odometry and PoseWithCovarianceStamped both have .pose.pose
            self.data[key].append((self.stamp(m.header), p.x, p.y))

        def on_status(self, m):
            now = self.get_clock().now().nanoseconds * 1e-9
            self.status.append((now, m.data))

    rclpy.init()
    node = Recorder()
    t_start, killed, t_kill = None, False, None
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            now = node.get_clock().now().nanoseconds * 1e-9
            if now == 0.0:
                continue
            t_start = now if t_start is None else t_start
            if a.kill_orbslam_at is not None and not killed and now - t_start >= a.kill_orbslam_at:
                killed = True
                r = subprocess.run(["pkill", "-f", ORB_PROCESS_PATTERN])
                print(f"t={now - t_start:.0f} s: pkill -f '{ORB_PROCESS_PATTERN}' -> "
                      + ("ORB-SLAM3 killed" if r.returncode == 0 else "no matching process (already gone?)"))
                t_kill = now
            if a.duration and now - t_start >= a.duration:
                break
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

    prefix = os.path.join(a.out, a.name)
    for key, rows in node.data.items():
        write_csv(f"{prefix}_{key}.csv", rows, ["t", "x", "y"])
    write_csv(f"{prefix}_status.csv", node.status, ["t", "json"])
    print("saved " + ", ".join(f"{len(rows)} {k}" for k, rows in node.data.items())
          + f", {len(node.status)} status messages -> {prefix}_*.csv")
    if not node.data["noisy"]:
        print("no /odom_noisy received: start the stack with odom_noise:=true (the EKF then fuses the noisy odometry)")
    if not node.data["fused"]:
        print("no /odometry/filtered received: is the EKF running (localization:=true)?")
    if not node.data["slam"]:
        print("no /localization/slam_pose received: ORB-SLAM3 never initialised (needs texture + motion)")
    if node.ok:
        flips = sum(1 for i in range(1, len(node.ok)) if node.ok[i - 1] and not node.ok[i])
        print(f"ORB-SLAM3 tracking OK in {100.0 * sum(node.ok) / len(node.ok):.0f} % of frames, lost {flips} time(s)")
    if killed:
        fused = np.array(node.data["fused"]).reshape(-1, 3)
        rate = fusion_core.rate_after(fused, t_kill)
        print(f"after the kill: /odometry/filtered still published at {rate:.1f} Hz "
              + ("(fused pose kept going)" if rate > 5 else "(NOT kept going: the fallback failed)"))
    run = {"gt": np.array(node.data["gt"]).reshape(-1, 3),
           "sources": {n: np.array(node.data[n]).reshape(-1, 3) for n in fusion_core.SOURCES},
           "status": fusion_core.parse_status_rows(node.status)}
    print_report(a.name, run, a.max_dt, a.segment, a.tolerance)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--max-dt", type=float, default=0.05, help="max time gap to ground truth (s)")
    common.add_argument("--segment", type=float, default=10.0, help="drift segment length (m)")
    common.add_argument("--tolerance", type=float, default=0.01,
                        help="exit test: fused may exceed the best single source by this many metres (default 0.01)")

    r = sub.add_parser("record", parents=[common], help="record live topics, save CSVs and print the report")
    r.add_argument("name", help="run name, e.g. R1")
    r.add_argument("--duration", type=float, default=0.0, help="sim seconds to record (0 = until Ctrl-C)")
    r.add_argument("--out", default=RESULTS, help="output directory (default: results/phase2/)")
    r.add_argument("--kill-orbslam-at", type=float, default=None, metavar="SECONDS",
                   help="sim seconds after the start at which to kill ORB-SLAM3 (fallback test)")
    r.set_defaults(fn=cmd_record)

    z = sub.add_parser("analyze", parents=[common], help="evaluate one saved run (prefix, e.g. results/phase2/R1)")
    z.add_argument("prefix")
    z.set_defaults(fn=cmd_analyze)

    t = sub.add_parser("table", parents=[common], help="results table for several saved runs")
    t.add_argument("--dir", default=RESULTS, help="directory with the saved runs (default: results/phase2/)")
    t.add_argument("--routes", nargs="+", default=["R1", "R2", "R3"])
    t.set_defaults(fn=cmd_table)

    a = p.parse_args()
    a.fn(a)


if __name__ == "__main__":
    main()
