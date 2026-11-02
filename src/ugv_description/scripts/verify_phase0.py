#!/usr/bin/env python3
"""verify_phase0.py - Gate 0 check for the VisionNav UGV simulation.

Run it while the simulator is up (ros2 launch ugv_description gazebo.launch.py):

    ros2 run ugv_description verify_phase0.py
    ros2 run ugv_description verify_phase0.py --no-drive        # skip the driving test
    ros2 run ugv_description verify_phase0.py --baseline 0.12   # if you changed camera_baseline

Checks
  1. Topic rates (in SIM time)  : stereo images + camera_info, IMU, odom, ground truth, joint_states
  2. Image format               : 640x480, correct optical frame ids
  3. Stereo sync                : left/right header stamps match
  4. Stereo geometry            : right camera_info P[0,3] gives the configured baseline
  5. TF tree                    : every frame resolves from base_footprint; optical-frame spacing = baseline
  6. Drive response (ground truth) : forward move, in-place turn, no flipping / sinking

Exit code 0 = all checks passed (Gate 0 technical criteria), 1 = at least one FAIL.
Ground truth (/ground_truth/odom) is used here for EVALUATION only.
"""
import argparse
import math
import struct
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.serialization import deserialize_message
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState
from tf2_ros import Buffer, TransformListener

BEST_EFFORT = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=50)
RELIABLE = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      history=HistoryPolicy.KEEP_LAST, depth=50)

# (topic, msg type, expected Hz)
RATE_TOPICS = [
    ('/stereo_camera/left/image_raw', Image, 30.0),
    ('/stereo_camera/right/image_raw', Image, 30.0),
    ('/stereo_camera/left/camera_info', CameraInfo, 30.0),
    ('/stereo_camera/right/camera_info', CameraInfo, 30.0),
    ('/camera/imu', Imu, 200.0),
    ('/odom', Odometry, 50.0),
    ('/ground_truth/odom', Odometry, 50.0),
    ('/joint_states', JointState, 50.0),
]

TF_FRAMES = [
    'base_link',
    'front_left_wheel_link', 'front_right_wheel_link',
    'rear_left_wheel_link', 'rear_right_wheel_link',
    'stereo_camera_link',
    'left_camera_frame', 'left_camera_optical_frame',
    'right_camera_frame', 'right_camera_optical_frame',
]


def quat_to_rpy(q):
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)
    sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
    pitch = math.asin(sinp)
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Verifier(Node):
    def __init__(self, baseline, image_qos=BEST_EFFORT):
        super().__init__('verify_phase0',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.baseline = baseline
        self.counts = {t: 0 for t, _, _ in RATE_TOPICS}
        self.first_msg = {}
        self.left_stamps = set()
        self.right_stamps = set()
        self.gt = None            # latest ground-truth Odometry
        self.odom = None          # latest /odom
        self.gt_attitude_max = 0.0   # max |roll|,|pitch| seen
        self.gt_z_max = 0.0
        self.counting = False

        for topic, typ, _ in RATE_TOPICS:
            if typ is Image:
                # Raw (serialized) subscription: a 920 KB image is NOT decoded in Python, so
                # the verifier keeps up at 30 Hz x 2 cameras. Only the header stamp is read.
                self.create_subscription(typ, topic, self._make_raw_image_cb(topic),
                                         image_qos, raw=True)
            else:
                self.create_subscription(typ, topic, self._make_cb(topic), BEST_EFFORT)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    # ------------------------------------------------------------------ callbacks
    def _make_cb(self, topic):
        def cb(msg):
            self.first_msg.setdefault(topic, msg)
            if topic == '/ground_truth/odom':
                self.gt = msg
                r, p, _ = quat_to_rpy(msg.pose.pose.orientation)
                self.gt_attitude_max = max(self.gt_attitude_max, abs(r), abs(p))
                self.gt_z_max = max(self.gt_z_max, abs(msg.pose.pose.position.z))
            elif topic == '/odom':
                self.odom = msg
            if self.counting:
                self.counts[topic] += 1
        return cb

    def _make_raw_image_cb(self, topic):
        is_left = topic.endswith('left/image_raw')

        def cb(data):
            data = bytes(data)
            if topic not in self.first_msg:          # decode only the first image (format check)
                self.first_msg[topic] = deserialize_message(data, Image)
            # CDR layout: 4-byte encapsulation header, then header.stamp {int32 sec, uint32 nanosec}
            fmt = '<iI' if data[1] == 1 else '>iI'
            stamp = struct.unpack_from(fmt, data, 4)
            if self.counting:
                self.counts[topic] += 1
                (self.left_stamps if is_left else self.right_stamps).add(stamp)
        return cb

    # ------------------------------------------------------------------ helpers
    def sim_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def spin_sim(self, sim_seconds, wall_timeout=None):
        """Spin until sim time advanced by sim_seconds. Returns False on wall timeout."""
        wall_timeout = wall_timeout or max(30.0, sim_seconds * 8)
        t0_sim, t0_wall = self.sim_now(), time.monotonic()
        while self.sim_now() - t0_sim < sim_seconds:
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() - t0_wall > wall_timeout:
                return False
        return True

    def drive(self, vx, wz, sim_seconds):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(vx), float(wz)
        t0 = self.sim_now()
        t0_wall = time.monotonic()
        while self.sim_now() - t0 < sim_seconds:
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)
            if time.monotonic() - t0_wall > max(30.0, sim_seconds * 8):
                break
        self.cmd_pub.publish(Twist())


class Report:
    def __init__(self):
        self.rows = []

    def add(self, name, ok, detail):
        self.rows.append((name, ok, detail))

    def print(self):
        width = max(len(r[0]) for r in self.rows) + 2
        print('\n' + '=' * (width + 60))
        print(' GATE 0 - PHASE 0 VERIFICATION')
        print('=' * (width + 60))
        for name, ok, detail in self.rows:
            print(f' [{"PASS" if ok else "FAIL"}] {name:<{width}} {detail}')
        print('=' * (width + 60))
        failed = [r for r in self.rows if not r[1]]
        if failed:
            print(f' RESULT: FAIL ({len(failed)} of {len(self.rows)} checks failed)')
        else:
            print(f' RESULT: PASS (all {len(self.rows)} checks passed)')
        print('=' * (width + 60) + '\n')
        return not failed


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--baseline', type=float, default=0.12, help='stereo baseline in metres')
    ap.add_argument('--window', type=float, default=5.0, help='rate measurement window (sim seconds)')
    ap.add_argument('--no-drive', action='store_true', help='skip the cmd_vel driving test')
    ap.add_argument('--qos', choices=['best_effort', 'reliable'], default='best_effort',
                    help='QoS of the image subscriptions (diagnostic: reliable avoids best-effort drops)')
    args = ap.parse_args()

    rclpy.init()
    node = Verifier(args.baseline, RELIABLE if args.qos == 'reliable' else BEST_EFFORT)
    rep = Report()

    # ---- 0. wait for /clock and data ------------------------------------------------
    print('Waiting for /clock and sensor data (Gazebo must be running and unpaused)...')
    t0 = time.monotonic()
    while time.monotonic() - t0 < 30.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.sim_now() > 0.0 and all(t in node.first_msg for t, _, _ in RATE_TOPICS):
            break
    clock_ok = node.sim_now() > 0.0
    rep.add('Sim clock (/clock)', clock_ok,
            f'sim time = {node.sim_now():.1f} s' if clock_ok else 'no /clock - is Gazebo running?')
    if not clock_ok:
        rep.print()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    rep.add('Image subscription QoS (info)', True, args.qos)

    # ---- 1. rates --------------------------------------------------------------------
    node.counts = {t: 0 for t, _, _ in RATE_TOPICS}
    node.counting = True
    t_start = node.sim_now()
    node.spin_sim(args.window)
    dt = max(node.sim_now() - t_start, 1e-6)
    node.counting = False
    for topic, _, expected in RATE_TOPICS:
        hz = node.counts[topic] / dt
        ok = hz >= 0.8 * expected
        rep.add(f'Rate {topic}', ok, f'{hz:6.1f} Hz (expected ~{expected:.0f}, min {0.8 * expected:.0f}, sim time)')

    # ---- 2. image format -------------------------------------------------------------
    for side in ('left', 'right'):
        img = node.first_msg.get(f'/stereo_camera/{side}/image_raw')
        if img is None:
            rep.add(f'Image format {side}', False, 'no image received')
            continue
        ok = (img.width == 640 and img.height == 480
              and img.header.frame_id == f'{side}_camera_optical_frame')
        rep.add(f'Image format {side}', ok,
                f'{img.width}x{img.height} {img.encoding}, frame_id={img.header.frame_id}')

    # ---- 3. stereo sync --------------------------------------------------------------
    if node.left_stamps and node.right_stamps:
        matched = len(node.left_stamps & node.right_stamps)
        frac = matched / max(1, min(len(node.left_stamps), len(node.right_stamps)))
        rep.add('Stereo sync (left/right stamps)', frac >= 0.9,
                f'{frac * 100:.0f}% of frames share an identical stamp')
    else:
        rep.add('Stereo sync (left/right stamps)', False, 'no images')

    # ---- 4. stereo geometry ----------------------------------------------------------
    ri = node.first_msg.get('/stereo_camera/right/camera_info')
    li = node.first_msg.get('/stereo_camera/left/camera_info')
    if ri is not None and li is not None:
        fx = ri.p[0]
        meas = (-ri.p[3] / fx) if fx else float('nan')
        ok = abs(meas - args.baseline) < 0.005 and abs(li.p[3]) < 1e-6
        rep.add('Stereo baseline (camera_info)', ok,
                f'-P[0,3]/fx = {meas:.4f} m (expected {args.baseline:.3f}); fx={fx:.1f} px')
    else:
        rep.add('Stereo baseline (camera_info)', False, 'no camera_info')

    # ---- 5. TF tree ------------------------------------------------------------------
    t0 = time.monotonic()
    while time.monotonic() - t0 < 8.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.tf_buffer.can_transform('base_footprint', 'right_camera_optical_frame', rclpy.time.Time()):
            break
    missing = [f for f in TF_FRAMES
               if not node.tf_buffer.can_transform('base_footprint', f, rclpy.time.Time())]
    rep.add('TF: frames resolve from base_footprint', not missing,
            'all %d frames OK' % len(TF_FRAMES) if not missing else 'missing: ' + ', '.join(missing))

    odom_tf = node.tf_buffer.can_transform('odom', 'base_footprint', rclpy.time.Time())
    rep.add('TF: odom -> base_footprint', odom_tf,
            'present' if odom_tf else
            'missing (Phase 0/1: plugin publishes it; Phase 2+: the EKF must be running)')

    try:
        tf = node.tf_buffer.lookup_transform('left_camera_optical_frame',
                                             'right_camera_optical_frame', rclpy.time.Time())
        t = tf.transform.translation
        ok = (abs(t.x - args.baseline) < 0.002 and abs(t.y) < 0.002 and abs(t.z) < 0.002)
        rep.add('TF: optical-frame spacing', ok,
                f'right in left optical = ({t.x:.3f}, {t.y:.3f}, {t.z:.3f}) m (expected x={args.baseline:.3f})')
    except Exception as exc:  # noqa: BLE001
        rep.add('TF: optical-frame spacing', False, f'lookup failed: {exc}')

    pubs = sorted({p.node_name for p in node.get_publishers_info_by_topic('/tf')})
    rep.add('TF: /tf publishers (info)', True, ', '.join(pubs) if pubs else 'none')

    # ---- 6. drive response -----------------------------------------------------------
    if args.no_drive:
        rep.add('Drive test', True, 'skipped (--no-drive)')
    elif node.gt is None or node.odom is None:
        rep.add('Drive test', False, 'no ground truth / odom to compare')
    else:
        node.gt_attitude_max = 0.0
        node.gt_z_max = 0.0
        node.drive(0.0, 0.0, 1.0)          # settle
        node.spin_sim(0.5)
        g0, o0 = node.gt.pose.pose, node.odom.pose.pose

        node.drive(0.3, 0.0, 3.0)          # 0.3 m/s for 3 s -> ~0.9 m
        node.spin_sim(1.0)
        g1, o1 = node.gt.pose.pose, node.odom.pose.pose
        d_gt = math.hypot(g1.position.x - g0.position.x, g1.position.y - g0.position.y)
        d_od = math.hypot(o1.position.x - o0.position.x, o1.position.y - o0.position.y)
        _, _, y0 = quat_to_rpy(g0.orientation)
        _, _, y1 = quat_to_rpy(g1.orientation)
        yaw_drift = abs(wrap(y1 - y0))
        ok = 0.5 < d_gt < 1.2 and yaw_drift < 0.2
        rep.add('Drive: forward 0.3 m/s x 3 s', ok,
                f'moved {d_gt:.2f} m (expected ~0.9), yaw drift {yaw_drift:.3f} rad')
        ok = d_gt > 0 and abs(d_od - d_gt) / d_gt < 0.25
        rep.add('Drive: /odom vs ground truth', ok,
                f'odom {d_od:.2f} m vs truth {d_gt:.2f} m')

        g2 = node.gt.pose.pose
        node.drive(0.0, 0.5, 3.0)          # spin in place 0.5 rad/s x 3 s
        node.spin_sim(1.0)
        g3 = node.gt.pose.pose
        _, _, ya = quat_to_rpy(g2.orientation)
        _, _, yb = quat_to_rpy(g3.orientation)
        dyaw = wrap(yb - ya)
        shift = math.hypot(g3.position.x - g2.position.x, g3.position.y - g2.position.y)
        ok = dyaw > 0.3 and shift < 0.5
        rep.add('Drive: turn in place 0.5 rad/s x 3 s', ok,
                f'turned {dyaw:.2f} rad (skid-steer slip makes it < 1.5), shifted {shift:.2f} m')

        ok = node.gt_attitude_max < 0.15 and node.gt_z_max < 0.15
        rep.add('Physics stable (no flip / sinking)', ok,
                f'max |roll,pitch| {node.gt_attitude_max:.3f} rad, max |z| {node.gt_z_max:.3f} m')

    node.cmd_pub.publish(Twist())
    all_ok = rep.print()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if all_ok else 1)


if __name__ == '__main__':
    main()
