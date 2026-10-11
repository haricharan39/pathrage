#!/usr/bin/env python3
"""Generate a Gazebo Classic (SDF 1.7) off-road world with rocks, trees and slopes.

The world uses the textured ground model (src/ugv_description/models/terrain_textured),
keeps a clear circle around the start and the goal, and keeps obstacles far enough apart that
a 0.60 x 0.40 m robot can always pass between them (--min-gap).

Examples:
    # sparse rocks, 40 x 40 m, start (0,0) -> goal (15,0)
    python3 tools/make_world.py --name offroad_sparse --seed 1 --rocks 18 \
        --out src/ugv_description/worlds/offroad_sparse.world

    # forest-like: many rocks and tree trunks plus two slopes
    python3 tools/make_world.py --name offroad_forest --seed 2 --rocks 30 --trees 25 --ramps 2 \
        --goal 20,4 --out src/ugv_description/worlds/offroad_forest.world

Same --seed always gives the same world, so results are reproducible.
Only the standard library is needed.
"""
import argparse
import math
import pathlib
import random

ROCK_COLOURS = [(0.45, 0.43, 0.40), (0.35, 0.33, 0.31), (0.55, 0.50, 0.42), (0.40, 0.36, 0.30),
                (0.30, 0.30, 0.32)]
BARK = (0.30, 0.20, 0.12)
LEAF = (0.12, 0.30, 0.10)


def material(rgb):
    r, g, b = rgb
    return (f"<material><ambient>{r:.2f} {g:.2f} {b:.2f} 1</ambient>"
            f"<diffuse>{r:.2f} {g:.2f} {b:.2f} 1</diffuse>"
            f"<specular>0.02 0.02 0.02 1</specular></material>")


def static_model(name, pose, parts):
    """parts: list of (geometry_xml, rgb, collide, local_pose)"""
    x, y, z, roll, pitch, yaw = pose
    out = [f'    <model name="{name}">', "      <static>true</static>",
           f"      <pose>{x:.3f} {y:.3f} {z:.3f} {roll:.4f} {pitch:.4f} {yaw:.4f}</pose>",
           '      <link name="link">']
    for i, (geom, rgb, collide, lp) in enumerate(parts):
        lpx = " ".join(f"{v:.3f}" for v in lp)
        if collide:
            out.append(f'        <collision name="c{i}"><pose>{lpx} 0 0 0</pose>'
                       f"<geometry>{geom}</geometry></collision>")
        out.append(f'        <visual name="v{i}"><pose>{lpx} 0 0 0</pose>'
                   f"<geometry>{geom}</geometry>{material(rgb)}</visual>")
    out += ["      </link>", "    </model>"]
    return "\n".join(out)


def box(sx, sy, sz):
    return f"<box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box>"


def cyl(r, h):
    return f"<cylinder><radius>{r:.3f}</radius><length>{h:.3f}</length></cylinder>"


def sph(r):
    return f"<sphere><radius>{r:.3f}</radius></sphere>"


class Placer:
    """Rejection sampling with keep-out zones and a minimum gap between obstacles."""

    def __init__(self, rng, half, zones, min_gap):
        self.rng, self.half, self.zones, self.gap = rng, half, list(zones), min_gap
        self.placed = []  # (x, y, radius)

    def place(self, radius, tries=400):
        for _ in range(tries):
            x = self.rng.uniform(-self.half, self.half)
            y = self.rng.uniform(-self.half, self.half)
            if any(math.hypot(x - zx, y - zy) < zr + radius for zx, zy, zr in self.zones):
                continue
            if any(math.hypot(x - px, y - py) < pr + radius + self.gap for px, py, pr in self.placed):
                continue
            self.placed.append((x, y, radius))
            return x, y
        return None


def build(args):
    rng = random.Random(args.seed)
    sx, sy = (float(v) for v in args.start.split(","))
    gx, gy = (float(v) for v in args.goal.split(","))
    half = args.size / 2.0
    zones = [(sx, sy, args.clear_radius), (gx, gy, 1.2)]
    placer = Placer(rng, half, zones, args.min_gap)
    models, skipped = [], 0

    for i in range(args.ramps):  # slopes first: they are the largest footprint
        length, width, t = rng.uniform(2.5, 3.5), rng.uniform(1.5, 2.2), 0.04
        theta = math.radians(rng.uniform(6, 11))
        yaw = rng.uniform(-math.pi, math.pi)
        pos = placer.place(length / 2 + 0.2)
        if pos is None:
            skipped += 1
            continue
        zc = (length / 2) * math.sin(theta) - (t / 2) * math.cos(theta)  # low edge of top face at z=0
        models.append(static_model(f"slope_{i}", (pos[0], pos[1], zc, 0, -theta, yaw),
                                   [(box(length, width, t), (0.42, 0.34, 0.24), True, (0, 0, 0))]))

    for i in range(args.trees):
        r = rng.uniform(0.07, 0.14)
        h = rng.uniform(2.2, 3.2)
        pos = placer.place(r + 0.15)
        if pos is None:
            skipped += 1
            continue
        crown = rng.uniform(0.7, 1.1)
        models.append(static_model(f"tree_{i}", (pos[0], pos[1], 0, 0, 0, 0), [
            (cyl(r, h), BARK, True, (0, 0, h / 2)),
            (sph(crown), LEAF, False, (0, 0, h + crown * 0.5))]))

    for i in range(args.rocks):
        kind = rng.choice(["box", "box", "cyl", "sph"])
        h = rng.uniform(0.18, 0.55)          # > 0.15 m so they are real obstacles for the UGV
        w = rng.uniform(0.25, 0.7)
        colour = rng.choice(ROCK_COLOURS)
        yaw = rng.uniform(-math.pi, math.pi)
        pos = placer.place(max(w, h) * 0.75)
        if pos is None:
            skipped += 1
            continue
        if kind == "box":
            part = (box(w, w * rng.uniform(0.6, 1.2), h), colour, True, (0, 0, h / 2))
        elif kind == "cyl":
            part = (cyl(w / 2, h), colour, True, (0, 0, h / 2))
        else:
            r = max(w, h) / 2
            part = (sph(r), colour, True, (0, 0, r * 0.55))   # partly buried
        models.append(static_model(f"rock_{i}", (pos[0], pos[1], 0, 0, 0, yaw), [part]))

    goal = static_model("goal_marker", (gx, gy, 0.01, 0, 0, 0),
                        [(cyl(0.25, 0.02), (0.1, 0.9, 0.2), False, (0, 0, 0))])
    xml = f"""<?xml version="1.0"?>
<!--
  {args.name}: generated by tools/make_world.py (do not hand-edit; re-generate instead).
  seed={args.seed} size={args.size:g}x{args.size:g} m rocks={args.rocks} trees={args.trees} ramps={args.ramps}
  start=({sx:g},{sy:g}) goal=({gx:g},{gy:g}) clear_radius={args.clear_radius:g} min_gap={args.min_gap:g}
  Needs GAZEBO_MODEL_PATH to contain src/ugv_description/models (gazebo.launch.py sets this).
-->
<sdf version="1.7">
  <world name="{args.name}">
    <include><uri>model://sun</uri></include>
    <include><uri>model://terrain_textured</uri></include>

{goal}

{chr(10).join(models)}
  </world>
</sdf>
"""
    return xml, len(models), skipped


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--name", default="offroad_generated")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--size", type=float, default=40.0, help="side length of the obstacle field in m")
    ap.add_argument("--rocks", type=int, default=15)
    ap.add_argument("--trees", type=int, default=0)
    ap.add_argument("--ramps", type=int, default=0)
    ap.add_argument("--start", default="0,0", help="x,y of the robot spawn")
    ap.add_argument("--goal", default="15,0", help="x,y of the goal marker")
    ap.add_argument("--clear-radius", type=float, default=2.5, help="obstacle-free radius around start")
    ap.add_argument("--min-gap", type=float, default=0.9,
                    help="minimum free distance between obstacles (robot is 0.4 m wide)")
    ap.add_argument("--out", help="output .world path (default: print to stdout)")
    args = ap.parse_args()

    xml, placed, skipped = build(args)
    if args.out:
        p = pathlib.Path(args.out)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(xml)
        print(f"wrote {p}: {placed} obstacles placed" + (f", {skipped} did not fit (lower the counts or "
                                                           "raise --size)" if skipped else ""))
    else:
        print(xml)


if __name__ == "__main__":
    main()
