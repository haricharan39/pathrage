#!/usr/bin/env python3
"""Generate a tileable, feature-rich off-road ground texture (dirt / grass / gravel).

Why: a flat single-colour ground gives stereo matching and ORB-SLAM3 almost nothing to lock
onto. This texture has patches at several scales plus fine speckle and pebble-like blobs.

    python3 tools/make_ground_texture.py                       # default 1024 px, seed 7
    python3 tools/make_ground_texture.py --size 2048 --seed 3 \
        --out src/ugv_description/models/terrain_textured/materials/textures/ground.png

Requires numpy and Pillow (pip install numpy pillow).
"""
import argparse
import pathlib

import numpy as np
from PIL import Image


def periodic_noise(size, cells, rng):
    """Bilinear value noise on a wrapping grid, so the result tiles seamlessly."""
    grid = rng.random((cells, cells))
    xs = np.linspace(0, cells, size, endpoint=False)
    i0 = np.floor(xs).astype(int)
    f = xs - i0
    f = f * f * (3 - 2 * f)                       # smoothstep
    i1 = (i0 + 1) % cells
    top = grid[i0][:, i0] * (1 - f)[None, :] + grid[i0][:, i1] * f[None, :]
    bot = grid[i1][:, i0] * (1 - f)[None, :] + grid[i1][:, i1] * f[None, :]
    return top * (1 - f)[:, None] + bot * f[:, None]


def fbm(size, rng, octaves):
    out, amp, total = np.zeros((size, size)), 1.0, 0.0
    for cells, a in octaves:
        out += a * periodic_noise(size, cells, rng)
        total += a
    return out / total


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--size", type=int, default=1024)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--out", default="src/ugv_description/models/terrain_textured/materials/textures/ground.png")
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)
    n = a.size

    patch = fbm(n, rng, [(4, 1.0), (8, 0.6), (16, 0.35)])             # large dirt/grass regions
    detail = fbm(n, rng, [(32, 0.6), (64, 0.8), (128, 1.0), (256, 0.8)])
    speckle = rng.random((n, n))

    dirt = np.array([0.45, 0.34, 0.22])
    dry = np.array([0.62, 0.52, 0.36])
    grass = np.array([0.24, 0.38, 0.16])
    mix = np.clip((patch - 0.35) * 3.0, 0, 1)[..., None]               # 0 = dirt, 1 = grass
    base = dirt * (1 - mix) + grass * mix
    dryness = np.clip((periodic_noise(n, 6, rng) - 0.55) * 4, 0, 1)[..., None]
    base = base * (1 - 0.6 * dryness) + dry * 0.6 * dryness

    shade = 0.55 + 0.9 * detail                                         # fine brightness structure
    img = base * shade[..., None]
    img *= (0.88 + 0.24 * speckle)[..., None]                           # pixel-level grain

    # pebbles: small bright/dark blobs with high local contrast (good for feature detectors)
    for _ in range(n * n // 1800):
        cx, cy = rng.integers(0, n, 2)
        r = int(rng.integers(2, 6))
        tone = rng.choice([0.55, 0.75, 1.35, 1.6])
        ys, xs = np.ogrid[-r:r + 1, -r:r + 1]
        mask = (xs * xs + ys * ys) <= r * r
        yy = (np.arange(-r, r + 1) + cy) % n
        xx = (np.arange(-r, r + 1) + cx) % n
        sub = img[np.ix_(yy, xx)]
        sub[mask] = sub[mask] * tone
        img[np.ix_(yy, xx)] = sub

    out = (np.clip(img, 0, 1) * 255).astype(np.uint8)
    path = pathlib.Path(a.out)
    path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(out, "RGB").save(path, optimize=True)
    print(f"wrote {path} ({n}x{n}, {path.stat().st_size / 1024:.0f} KiB)")


if __name__ == "__main__":
    main()
