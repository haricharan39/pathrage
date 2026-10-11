"""ROS-free route definitions for repeatable test drives (R1, R2, R3).

A route is a list of (duration_s, linear_mps, angular_radps) segments. Open-loop:
the real path is judged against /ground_truth/odom, never against this schedule.
"""
import math

ROUTES = ("R1", "R2", "R3")


def build_route(name, speed=0.4, turn_rate=0.4, side=10.0, radius=3.0):
    """R1 straight 20 m | R2 square `side` m, left turns | R3 figure-8 of two circles."""
    if speed <= 0 or turn_rate <= 0:
        raise ValueError("speed and turn_rate must be > 0")
    if name == "R1":
        return [(20.0 / speed, speed, 0.0)]
    if name == "R2":
        seg = []
        for _ in range(4):
            seg.append((side / speed, speed, 0.0))
            seg.append(((math.pi / 2) / turn_rate, 0.0, turn_rate))
        return seg
    if name == "R3":
        w = speed / radius
        t = 2 * math.pi / w
        return [(t, speed, w), (t, speed, -w)]
    raise ValueError(f"unknown route {name!r}, use one of {ROUTES}")


def integrate(segments, dt=0.01):
    """Open-loop (x, y, yaw) end pose of a route, starting at the origin."""
    x = y = yaw = 0.0
    for dur, v, w in segments:
        steps = max(1, int(round(dur / dt)))
        h = dur / steps
        for _ in range(steps):
            x += v * math.cos(yaw) * h
            y += v * math.sin(yaw) * h
            yaw += w * h
    return x, y, yaw


def total_distance(segments):
    return sum(d * abs(v) for d, v, _ in segments)
