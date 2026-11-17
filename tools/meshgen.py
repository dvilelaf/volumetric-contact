# /// script
# requires-python = ">=3.11"
# dependencies = ["numpy"]
# ///
"""Procedural generator of the mesh assets of the scenario battery (docs/scenarios.md §1).

Writes <out>/<bodies|grounds>/<name>/surface.obj (whole closed surface, metres, 17
significant digits) and, for non-convex assets, <out>/<group>/<name>/pieces/NNN.obj (convex
pieces). Every asset is verified before it is written; a failed check aborts. Then calls
tools/volume_mesh.py for <name>/volume.vtk (tetrahedra; every asset except slabs) when that
script exists. Curved assets end in _seg<N>, the segments per full turn.

usage: uv run tools/meshgen.py [--out DIR] [--missing-volumes] [NAME ...]
"""

import argparse
import re
import subprocess
from collections import defaultdict
from dataclasses import dataclass
from itertools import pairwise
from pathlib import Path

import numpy as np

SEGMENTS = (32, 64, 128)
BODIES = ("box", "cylinder", "sphere", "frustum", "ring")
TEMPLATES = [
    "slab_40000x40000x1000",
    "box_1000x500x500", "box_400x400x200", "box_200x200x600", "box_400x100x100",
    "box_100x100x100", "box_400x400x50_o50x0x0", "box_400x400x100", "box_200x200x100",
    "box_200x200x200", "box_300x200x100",
    "cylinder_r250_h50_seg{n}", "cylinder_r250_h1000_seg{n}", "cylinder_r100_h20_seg{n}",
    "cylinder_r100_h400_seg{n}", "cylinder_r50_h200_seg{n}", "cylinder_r500_h100_seg{n}",
    "sphere_r100_seg{n}",
    "frustum_r50_r100_h100_seg{n}",
    "ring_ro250_ri150_h50_seg{n}",
    "vgroove_w300_l4000",
    "hole_s99.8_d100",
    "corner_s300",
    "table_1000x600x200",
    "table_1600x1600x200", "table_1600x1600x200_p2", "table_1600x1600x200_p8", "table_1600x1600x200_p64",
    "bowl_r300_t20_a60_seg{n}",
    "bowl_r105_t20_a60_seg64",  # no scenario: conforming seat of sphere_r100 for tests/test_invariants.cpp
    "trough_r300_t20_a60_l500_seg{n}",
    "bearing_r50_t20_l200_seg{n}", "bearing_r60_t20_l200_seg{n}",
    "seat_r50_r100_h100_t20_seg{n}",
]
EXACT = (-1e-9, 1e-9)


def all_names() -> list[str]:
    return [name for t in TEMPLATES for name in ([t.format(n=n) for n in SEGMENTS] if "{n}" in t else [t])]


@dataclass
class Asset:
    """Geometry in millimetres plus the analytic expectations it is checked against."""
    name: str
    pool: "Pool"
    polys: list
    pieces: list | None
    volume: float
    vol_range: tuple = EXACT  # allowed relative error (V - volume) / volume
    bounds: tuple | None = None
    centroid: tuple | None = None
    sd: object = None  # signed distance-like function of the true solid, <= 0 inside
    on_surface: bool = False  # every vertex lies on the true surface (sd == 0)
    cavity: object = None  # depth of a point inside the true cavity (> 0 inside)
    segments: int = 0


class Pool:
    def __init__(self):
        self.v = []

    def add(self, x, y, z):
        self.v.append((float(x), float(y), float(z)))
        return len(self.v) - 1


def trig(j, n):
    """cos and sin of 2πj/n, exact at quarter turns."""
    j = np.atleast_1d(np.asarray(j))
    a = 2 * np.pi * j / n
    c, s = np.cos(a), np.sin(a)
    q = (4 * j) % n == 0
    quad = (4 * j // n) % 4
    c[q] = np.array([1.0, 0, -1, 0])[quad[q]]
    s[q] = np.array([0.0, 1, 0, -1])[quad[q]]
    return c, s


def fan(polys):
    return [(p[0], p[i], p[i + 1]) for p in polys for i in range(1, len(p) - 1)]


def compact(pool, polys):
    idx = sorted({i for p in polys for i in p})
    loc = {i: k for k, i in enumerate(idx)}
    P = np.array([pool.v[i] for i in idx])
    T = np.array([[loc[i] for i in t] for t in fan(polys)])
    return P, T


def volume(P, T):
    return np.einsum("ij,ij->i", P[T[:, 0]], np.cross(P[T[:, 1]], P[T[:, 2]])).sum() / 6


def centroid(P, T):
    d = np.einsum("ij,ij->i", P[T[:, 0]], np.cross(P[T[:, 1]], P[T[:, 2]])) / 6
    return (d[:, None] * (P[T[:, 0]] + P[T[:, 1]] + P[T[:, 2]]) / 4).sum(0) / d.sum()


def orient(pool, polys):
    return polys if volume(*compact(pool, polys)) > 0 else [p[::-1] for p in polys]


def loft(A, B, caps=True):
    k = len(A)
    polys = [(A[i], A[(i + 1) % k], B[(i + 1) % k], B[i]) for i in range(k)]
    return polys + [tuple(A[::-1]), tuple(B)] if caps else polys


def boundary(solids):
    """Faces of the union of solids with disjoint interiors: faces seen twice cancel."""
    seen = defaultdict(list)
    for s in solids:
        for p in s:
            seen[tuple(sorted(p))].append(p)
    return [f[0] for f in seen.values() if len(f) == 1]


def box_cell(pool, lo, hi):
    (x0, y0, z0), (x1, y1, z1) = lo, hi
    ring = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    A = [pool.add(x, y, z0) for x, y in ring]
    B = [pool.add(x, y, z1) for x, y in ring]
    return orient(pool, loft(A, B))


def grid_surface(pool, boxes):
    """Boundary of a union of axis-aligned boxes, on the grid of all box coordinates."""
    axes = [np.unique([b[w][a] for b in boxes for w in (0, 1)]) for a in range(3)]
    shape = [len(x) - 1 for x in axes]
    solid = np.zeros(shape, bool)
    for lo, hi in boxes:
        sl = tuple(slice(np.searchsorted(axes[a], lo[a]), np.searchsorted(axes[a], hi[a])) for a in range(3))
        solid[sl] = True
    padded = np.pad(solid, 1)
    ids = {}

    def vid(c):
        if c not in ids:
            ids[c] = pool.add(*(axes[a][c[a]] for a in range(3)))
        return ids[c]

    polys = []
    for c in zip(*np.nonzero(solid)):
        for a in range(3):
            b, d = (a + 1) % 3, (a + 2) % 3
            for sign in (1, -1):
                nb = list(c)
                nb[a] += sign
                if padded[tuple(i + 1 for i in nb)]:
                    continue
                base = list(c)
                base[a] += 1 if sign > 0 else 0
                quad = []
                for db, dd in ((0, 0), (1, 0), (1, 1), (0, 1)):
                    q = list(base)
                    q[b] += db
                    q[d] += dd
                    quad.append(vid(tuple(q)))
                polys.append(tuple(quad) if sign > 0 else tuple(quad[::-1]))
    return polys


def strips(pool, sections, closed):
    """Convex cells between two sections; section = (inner points, outer points) along a ring."""
    ids = [([pool.add(*p) for p in inn], [pool.add(*p) for p in out]) for inn, out in sections]
    k = len(ids[0][0])
    cells = []
    for j in range(k if closed else k - 1):
        j1 = (j + 1) % k
        A, B = [[o[j], o[j1], i[j1], i[j]] for i, o in ids]
        cells.append(orient(pool, loft(A, B)))
    return cells


def revolve(pool, rings, n):
    """Surface of revolution through (radius, z) rings; radius 0 is a single vertex."""
    c, s = trig(np.arange(n), n)
    ids = [[pool.add(0, 0, z)] if r == 0 else [pool.add(r * c[k], r * s[k], z) for k in range(n)]
           for r, z in rings]
    polys = []
    for A, B in pairwise(ids):
        for k in range(n):
            k1 = (k + 1) % n
            if len(A) == 1:
                polys.append((A[0], B[k1], B[k]))
            elif len(B) == 1:
                polys.append((B[0], A[k], A[k1]))
            else:
                polys.append((A[k], A[k1], B[k1], B[k]))
    return polys


# ---------------------------------------------------------------- assets (mm)

def make_box(name, dims, off):
    pool = Pool()
    half = np.array(dims) / 2
    lo, hi = -half - off, half - off
    polys = box_cell(pool, lo, hi)
    return Asset(name, pool, polys, None, float(np.prod(dims)), bounds=(lo, hi), centroid=tuple(-off))


def make_slab(name, dims):
    pool = Pool()
    lo, hi = (-dims[0] / 2, -dims[1] / 2, -dims[2]), (dims[0] / 2, dims[1] / 2, 0.0)
    return Asset(name, pool, box_cell(pool, lo, hi), None, float(np.prod(dims)), bounds=(lo, hi))


def make_table(name, dims, n):
    """Box with its top at z = 0, centred in x and y; n > 1 splits it into a grid of n equal tiles
    (n a power of two: 2^ceil(k/2) along x by 2^floor(k/2) along y)."""
    pool = Pool()
    lo, hi = (-dims[0] / 2, -dims[1] / 2, -dims[2]), (dims[0] / 2, dims[1] / 2, 0.0)
    if n == 1:
        return Asset(name, pool, box_cell(pool, lo, hi), None, float(np.prod(dims)), bounds=(lo, hi))
    k = n.bit_length() - 1
    if 1 << k != n:
        raise SystemExit(f"{name}: the number of pieces must be a power of two")
    nx, ny = 1 << (k + 1) // 2, 1 << k // 2
    xs, ys = np.linspace(lo[0], hi[0], nx + 1), np.linspace(lo[1], hi[1], ny + 1)
    boxes = [((xs[i], ys[j], lo[2]), (xs[i + 1], ys[j + 1], 0.0)) for i in range(nx) for j in range(ny)]
    cells = [box_cell(pool, *b) for b in boxes]
    return Asset(name, pool, grid_surface(pool, boxes), cells, float(np.prod(dims)), bounds=(lo, hi))


def make_cylinder(name, r, h, n):
    pool = Pool()
    polys = revolve(pool, [(0, -h / 2), (r, -h / 2), (r, h / 2), (0, h / 2)], n)
    d = 2 * np.pi / n
    return Asset(name, pool, polys, None, np.pi * r * r * h, (-d * d / 5, 1e-9),
                 bounds=((-r, -r, -h / 2), (r, r, h / 2)), centroid=(0, 0, 0), on_surface=True, segments=n,
                 sd=lambda P: np.maximum(np.hypot(P[:, 0], P[:, 1]) - r, np.abs(P[:, 2]) - h / 2))


def make_sphere(name, r, n):
    pool = Pool()
    c, s = trig(np.arange(n // 2 + 1), n)
    polys = revolve(pool, [(0, -r)] + [(r * s[j], -r * c[j]) for j in range(1, n // 2)] + [(0, r)], n)
    d = 2 * np.pi / n
    return Asset(name, pool, polys, None, 4 / 3 * np.pi * r ** 3, (-d * d / 2, 1e-9),
                 bounds=((-r, -r, -r), (r, r, r)), centroid=(0, 0, 0), on_surface=True, segments=n,
                 sd=lambda P: np.linalg.norm(P, axis=1) - r)


def make_frustum(name, a, b, h, n):
    pool = Pool()
    zbar = h * (a * a + 2 * a * b + 3 * b * b) / (4 * (a * a + a * b + b * b))
    zb, zt = -zbar, h - zbar
    polys = revolve(pool, [(0, zb), (a, zb), (b, zt), (0, zt)], n)
    d = 2 * np.pi / n
    return Asset(name, pool, polys, None, np.pi * h * (a * a + a * b + b * b) / 3, (-d * d / 5, 1e-9),
                 bounds=((-b, -b, zb), (b, b, zt)), centroid=(0, 0, 0), on_surface=True, segments=n,
                 sd=lambda P: np.maximum.reduce([np.hypot(P[:, 0], P[:, 1]) - (a + (b - a) * (P[:, 2] - zb) / h),
                                                 zb - P[:, 2], P[:, 2] - zt]))


def make_ring(name, ro, ri, h, n):
    pool = Pool()
    d = 2 * np.pi / n
    c, s = trig(np.arange(n), n)
    rin = ri / np.cos(d / 2)
    secs = [([(rin * c[k], rin * s[k], z) for k in range(n)], [(ro * c[k], ro * s[k], z) for k in range(n)])
            for z in (-h / 2, h / 2)]
    cells = strips(pool, secs, True)
    return Asset(name, pool, boundary(cells), cells, np.pi * (ro * ro - ri * ri) * h, (-d * d / 2, 1e-9),
                 bounds=((-ro, -ro, -h / 2), (ro, ro, h / 2)), centroid=(0, 0, 0), segments=n,
                 sd=lambda P: np.maximum.reduce([np.hypot(P[:, 0], P[:, 1]) - ro, ri - np.hypot(P[:, 0], P[:, 1]),
                                                 np.abs(P[:, 2]) - h / 2]))


def make_vgroove(name, w, length):
    pool = Pool()
    hw, depth, hl = w / 2, 100.0, length / 2
    yz = [(-hw, hw), (0, 0), (hw, hw), (hw, -depth), (0, -depth), (-hw, -depth)]
    A = [pool.add(-hl, y, z) for y, z in yz]
    B = [pool.add(hl, y, z) for y, z in yz]
    cells = [orient(pool, loft([A[i] for i in q], [B[i] for i in q])) for q in ([1, 2, 3, 4], [0, 1, 4, 5])]
    return Asset(name, pool, boundary(cells), cells, 2 * (hw * hw / 2 + depth * hw) * length,
                 bounds=((-hl, -hw, -depth), (hl, hw, hw)))


def make_hole(name, s, depth):
    pool = Pool()
    h, t = s / 2, 100.0
    lo, hi = (-h - t, -h - t, -depth), (h + t, h + t, 0.0)
    boxes = [((-h - t, h, -depth), (h, h + t, 0.0))]  # pinwheel of four walls
    for _ in range(3):
        (x0, y0, z0), (x1, y1, z1) = boxes[-1]
        boxes.append(((-y1, x0, z0), (-y0, x1, z1)))
    cells = [box_cell(pool, *b) for b in boxes]
    return Asset(name, pool, grid_surface(pool, boxes), cells, ((s + 2 * t) ** 2 - s * s) * depth, bounds=(lo, hi))


def make_corner(name, s):
    boxes = [((-50, -50, -50), (s, s, 0)), ((-50, -50, 0), (0, s, s)), ((0, -50, 0), (s, 0, s))]
    pool = Pool()
    cells = [box_cell(pool, *b) for b in boxes]
    return Asset(name, pool, grid_surface(pool, boxes), cells,
                 float(sum(np.prod(np.subtract(h, l)) for l, h in boxes)), bounds=((-50, -50, -50), (s, s, s)))


def make_bowl(name, R, T, A, n):
    pool = Pool()
    d = 2 * np.pi / n
    m = round(A / 360 * n)
    ch = np.cos(d / 2)
    sc = (R + T) / R
    ck, sk = trig(np.arange(n), n)
    ct, st = trig(np.arange(m + 1), n)
    inner, outer = [], []
    for j in range(m + 1):
        a, z = R * st[j] / ch ** 2, R - R * ct[j] / ch
        pts = [(0.0, 0.0, z)] if j == 0 else [(a * ck[i], a * sk[i], z) for i in range(n)]
        inner.append([pool.add(*p) for p in pts])
        outer.append([pool.add(p[0] * sc, p[1] * sc, R + (p[2] - R) * sc) for p in pts])
    cells = []
    for j in range(m):
        for i in range(n):
            i1 = (i + 1) % n
            if j == 0:
                q = [(0, 0), (1, i), (1, i1)]
            else:
                q = [(j, i), (j, i1), (j + 1, i1), (j + 1, i)]
            cells.append(orient(pool, loft([inner[a][b] for a, b in q], [outer[a][b] for a, b in q])))
    vol = 2 * np.pi / 3 * ((R + T) ** 3 - R ** 3) * (1 - np.cos(m * d))
    return Asset(name, pool, boundary(cells), cells, vol, (-1e-9, d * d / 3), segments=n,
                 cavity=lambda P: R - np.linalg.norm(P - (0, 0, R), axis=1))


def make_trough(name, R, T, A, L, n):
    pool = Pool()
    d = 2 * np.pi / n
    m = round(A / 360 * n)
    c, s = trig(np.arange(-m, m + 1), n)
    ch = np.cos(d / 2)
    secs = [[[(rho * s[j], y, R - rho * c[j]) for j in range(len(c))] for rho in (R / ch, (R + T) / ch)]
            for y in (-L / 2, L / 2)]
    cells = strips(pool, secs, False)
    vol = m * d * ((R + T) ** 2 - R * R) * L
    return Asset(name, pool, boundary(cells), cells, vol, (-1e-9, d * d / 10), segments=n,
                 cavity=lambda P: R - np.hypot(P[:, 0], P[:, 2] - R))


def make_bearing(name, R, T, L, n):
    pool = Pool()
    d = 2 * np.pi / n
    c, s = trig(np.arange(n), n)
    ch = np.cos(d / 2)
    secs = [[[(rho * s[k], y, -rho * c[k]) for k in range(n)] for rho in (R / ch, (R + T) / ch)]
            for y in (-L / 2, L / 2)]
    cells = strips(pool, secs, True)
    return Asset(name, pool, boundary(cells), cells, np.pi * ((R + T) ** 2 - R * R) * L, (-1e-9, d * d / 10),
                 segments=n, cavity=lambda P: R - np.hypot(P[:, 0], P[:, 2]))


def make_seat(name, a, b, h, T, n):
    pool = Pool()
    d = 2 * np.pi / n
    zbar = h * (a * a + 2 * a * b + 3 * b * b) / (4 * (a * a + a * b + b * b))
    zb = -zbar
    ext = 10.0
    zlo, zhi = zb - ext, zb + h + ext

    def rad(z):
        return a + (b - a) * (np.asarray(z) - zb) / h

    off = T * np.hypot(h, b - a) / h  # horizontal offset of the apothem for thickness T along the normal
    c, s = trig(np.arange(n), n)
    ch = np.cos(d / 2)
    secs = [[[(rho * c[k], rho * s[k], z) for k in range(n)] for rho in (rad(z) / ch, (rad(z) + off) / ch)]
            for z in (zlo, zhi)]
    cells = strips(pool, secs, True)
    dz = zhi - zlo
    vol = np.pi * (off * (rad(zlo) + rad(zhi)) * dz + off * off * dz)
    return Asset(name, pool, boundary(cells), cells, vol, (-1e-9, d * d / 10), segments=n,
                 cavity=lambda P: rad(P[:, 2]) - np.hypot(P[:, 0], P[:, 1]))


def build(name: str) -> Asset:
    kind, *tok = name.split("_")
    off = np.zeros(3)
    if kind == "box" and tok[-1].startswith("o"):
        off = np.array([float(x) for x in tok.pop()[1:].split("x")])
    if kind in ("slab", "box"):
        dims = [float(x) for x in tok[0].split("x")]
        return make_slab(name, dims) if kind == "slab" else make_box(name, dims, off)
    if kind == "table":
        pieces = int(tok[1][1:]) if len(tok) > 1 else 1
        return make_table(name, [float(x) for x in tok[0].split("x")], pieces)
    v = []
    for t in tok:
        m = re.fullmatch(r"[a-z]+(-?[\d.]+)", t)
        if not m:
            raise SystemExit(f"cannot parse asset name {name}")
        v.append(float(m.group(1)))
    maker = {"cylinder": make_cylinder, "sphere": make_sphere, "frustum": make_frustum, "ring": make_ring,
             "vgroove": make_vgroove, "hole": make_hole, "corner": make_corner, "bowl": make_bowl,
             "trough": make_trough, "bearing": make_bearing, "seat": make_seat}.get(kind)
    if maker is None:
        raise SystemExit(f"unknown asset kind {kind}")
    if "_seg" in name:
        v[-1] = int(v[-1])
    return maker(name, *v)


# ---------------------------------------------------------------- checks

def fail(name, msg):
    raise SystemExit(f"{name}: FAILED {msg}")


def check_mesh(P, T, name, tol):
    nv = len(P)
    if len(np.unique(np.round(P / tol).astype(np.int64), axis=0)) != nv:
        fail(name, "duplicate vertices")
    if len(np.unique(T)) != nv:
        fail(name, "unused vertices")
    e = np.concatenate([T[:, [0, 1]], T[:, [1, 2]], T[:, [2, 0]]])
    code = e[:, 0].astype(np.int64) * nv + e[:, 1]
    if len(np.unique(code)) != len(code) or not np.isin(e[:, 1].astype(np.int64) * nv + e[:, 0], code).all():
        fail(name, "not closed and consistently oriented")
    link = defaultdict(dict)
    for a, b, c in T.tolist():
        link[a][b] = c
        link[b][c] = a
        link[c][a] = b
    for v, nxt in link.items():
        x, n = next(iter(nxt)), 0
        while True:
            x, n = nxt[x], n + 1
            if x == next(iter(nxt)):
                break
        if n != len(nxt):
            fail(name, f"vertex {v} not manifold")
    a, b, c = P[T[:, 0]], P[T[:, 1]], P[T[:, 2]]
    area = np.linalg.norm(np.cross(b - a, c - a), axis=1) / 2
    longest = np.max([((b - a) ** 2).sum(1), ((c - b) ** 2).sum(1), ((a - c) ** 2).sum(1)], axis=0)
    if (area < 1e-6 * longest).any():
        fail(name, "degenerate triangle")
    if volume(P, T) <= 0:
        fail(name, "not outward")


def samples(P, T, k=4):
    w = [(i, j, k - i - j) for i in range(k + 1) for j in range(k + 1 - i)]
    return np.concatenate([(P[T[:, 0]] * a + P[T[:, 1]] * b + P[T[:, 2]] * c) / k for a, b, c in w])


def unit_dirs(X):
    X = X[np.linalg.norm(X, axis=1) > 1e-9]
    X = X / np.linalg.norm(X, axis=1)[:, None]
    first = np.argmax(np.abs(X) > 1e-9, axis=1)
    X = X * np.sign(X[np.arange(len(X)), first])[:, None]
    return np.unique(np.round(X, 9), axis=0)


def separated(a, b, tol):
    Pa, na, ea = a
    Pb, nb, eb = b
    for axes in (np.concatenate([na, nb]), unit_dirs(np.cross(ea[:, None], eb[None]).reshape(-1, 3))):
        pa, pb = Pa @ axes.T, Pb @ axes.T
        if ((pa.max(0) <= pb.min(0) + tol) | (pb.max(0) <= pa.min(0) + tol)).any():
            return True
    return False


def check_pieces(parts, whole_volume, name, size):
    tol = 1e-9 * size
    info = []
    for k, (P, T) in enumerate(parts):
        label = f"{name} piece {k}"
        check_mesh(P, T, label, tol)
        nrm = np.cross(P[T[:, 1]] - P[T[:, 0]], P[T[:, 2]] - P[T[:, 0]])
        nrm /= np.linalg.norm(nrm, axis=1)[:, None]
        if (np.einsum("fvj,fj->fv", P[None] - P[T[:, 0]][:, None], nrm) > tol).any():
            fail(label, "not convex")
        edges = np.concatenate([P[T[:, 1]] - P[T[:, 0]], P[T[:, 2]] - P[T[:, 1]]])
        info.append((P, unit_dirs(nrm), unit_dirs(edges)))
    total = sum(volume(P, T) for P, T in parts)
    if abs(total - whole_volume) > 1e-9 * whole_volume:
        fail(name, f"pieces sum {total} != whole {whole_volume}")
    lo = np.array([P.min(0) for P, _, _ in info])
    hi = np.array([P.max(0) for P, _, _ in info])
    for i in range(len(info)):
        close = np.nonzero(((lo[i + 1:] < hi[i] - tol) & (hi[i + 1:] > lo[i] + tol)).all(1))[0] + i + 1
        for j in close:
            if not separated(info[i], info[j], tol):
                fail(name, f"pieces {i} and {j} overlap")


def meshes(asset):
    return compact(asset.pool, asset.polys), [compact(asset.pool, p) for p in asset.pieces or []]


def check(asset):
    name = asset.name
    (P, T), parts = meshes(asset)
    size = float(np.abs(P).max())
    tol = 1e-9 * size
    check_mesh(P, T, name, tol)
    V = volume(P, T)
    lo, hi = asset.vol_range
    err = (V - asset.volume) / asset.volume
    if not lo <= err <= hi:
        fail(name, f"volume {V} vs analytic {asset.volume}: relative error {err:.3g} outside [{lo:.3g}, {hi:.3g}]")
    if asset.bounds is not None and (np.abs(P.min(0) - asset.bounds[0]).max() > tol
                                     or np.abs(P.max(0) - asset.bounds[1]).max() > tol):
        fail(name, "bounding box differs from the specified frame")
    if asset.centroid is not None and np.abs(centroid(P, T) - asset.centroid).max() > tol:
        fail(name, "centroid differs from the specified origin")
    if asset.sd is not None:
        if asset.sd(samples(P, T)).max() > tol:
            fail(name, "surface leaves the true solid")
        if asset.on_surface and np.abs(asset.sd(P)).max() > tol:
            fail(name, "vertices off the true surface")
    if asset.cavity is not None and asset.cavity(samples(P, T)).max() > tol:
        fail(name, "surface enters the true cavity")
    if parts:
        check_pieces(parts, V, name, size)
    return V, len(parts)


# ---------------------------------------------------------------- output

def obj_text(P, T):
    f = lambda x: f"{x / 1000 + 0.0:.17g}"
    return "".join(f"v {f(x)} {f(y)} {f(z)}\n" for x, y, z in P) + \
        "".join(f"f {a + 1} {b + 1} {c + 1}\n" for a, b, c in T)


def asset_dir(out: Path, name: str) -> Path:
    return out / ("bodies" if name.split("_")[0] in BODIES else "grounds") / name


def write(asset, out: Path):
    (P, T), parts = meshes(asset)
    folder = asset_dir(out, asset.name)
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "surface.obj").write_text(obj_text(P, T))
    if parts:
        d = folder / "pieces"
        d.mkdir(exist_ok=True)
        for old in d.glob("*.obj"):
            old.unlink()
        width = max(3, len(str(len(parts) - 1)))
        for k, (Pk, Tk) in enumerate(parts):
            (d / f"{k:0{width}d}.obj").write_text(obj_text(Pk, Tk))


def missing_volumes(names: list[str], out: Path):
    """Write the surface (if absent) and volume.vtk of the assets that have no volume.vtk."""
    vm = Path(__file__).with_name("volume_mesh.py")
    todo = [n for n in names if not n.startswith("slab_") and not (asset_dir(out, n) / "volume.vtk").exists()]
    for name in todo:
        if not (asset_dir(out, name) / "surface.obj").exists():
            write(build(name), out)
    if todo:
        subprocess.run(["uv", "run", str(vm), *[str(asset_dir(out, n) / "surface.obj") for n in todo]], check=True)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("names", nargs="*")
    ap.add_argument("--out", type=Path, default=Path(__file__).resolve().parent.parent / "data")
    ap.add_argument("--missing-volumes", action="store_true",
                    help="write only the volume.vtk files that are absent; surfaces are left alone")
    args = ap.parse_args()
    names = args.names or all_names()
    if args.missing_volumes:
        missing_volumes(names, args.out)
        return
    for name in names:
        asset = build(name)
        V, k = check(asset)
        write(asset, args.out)
        print(f"{name}: volume {V / 1e9:.9g} m3" + (f", {k} pieces" if k else ""))
    vm = Path(__file__).with_name("volume_mesh.py")
    objs = [str(asset_dir(args.out, n) / "surface.obj") for n in names if not n.startswith("slab_")]
    if not vm.exists():
        print("tools/volume_mesh.py not found: tetrahedral meshes (volume.vtk) skipped")
    elif objs:
        subprocess.run(["uv", "run", str(vm), *objs], check=True)


if __name__ == "__main__":
    main()
