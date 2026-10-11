#!/usr/bin/env python3
"""Drive a fixed test route by publishing /cmd_vel (open loop, simulation clock).

  python3 tools/drive_route.py R1 --speed 0.4
  python3 tools/drive_route.py R2 --speed 0.3 --turn-rate 0.3
  python3 tools/drive_route.py R3 --speed 0.3

Needs the simulation running. Not verified on a ROS machine yet.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import route_core  # noqa: E402

import rclpy  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402


class RouteDriver(Node):
    def __init__(self, segments, start_delay):
        super().__init__("route_driver", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.segments = segments
        self.start_delay = start_delay
        self.t0 = None
        self.done = False
        self.create_timer(0.05, self.tick)

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now == 0.0:
            return  # sim clock not running yet
        if self.t0 is None:
            self.t0 = now + self.start_delay
        t = now - self.t0
        msg = Twist()
        if t >= 0:
            for dur, v, w in self.segments:
                if t < dur:
                    msg.linear.x, msg.angular.z = v, w
                    break
                t -= dur
            else:
                self.pub.publish(Twist())  # stop
                self.done = True
                return
        self.pub.publish(msg)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("route", choices=route_core.ROUTES)
    p.add_argument("--speed", type=float, default=0.4, help="m/s")
    p.add_argument("--turn-rate", type=float, default=0.4, help="rad/s, R2 corners")
    p.add_argument("--start-delay", type=float, default=2.0, help="s before moving")
    a = p.parse_args()
    seg = route_core.build_route(a.route, a.speed, a.turn_rate)
    print(f"{a.route}: {route_core.total_distance(seg):.1f} m, "
          f"{sum(s[0] for s in seg):.0f} s at {a.speed} m/s")
    rclpy.init()
    node = RouteDriver(seg, a.start_delay)
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
