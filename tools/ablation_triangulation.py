# /// script
# requires-python = ">=3.11"
# dependencies = ["numpy", "scipy", "matplotlib"]
# ///
"""How the three friction quadratures depend on the triangulation of a flat face.

The battery's ablation (docs/baselines.md §3) compares the closed-form Coulomb friction of
`distributed` with the vertex rule and the one-point rule on the meshes of the battery, whose flat
caps are fans from a vertex of the rim (tools/meshgen.py, `fan`). This script settles what that
comparison owes to those fans. It re-implements, standalone and in the plane, the three rules of
core/src/distributed.cpp for one triangle with a linear pressure under a rigid slip field:

    closed    the closed form of the paper (polar moments about the instantaneous centre)
    vertex    friction at the three vertices, interpolated linearly (Hasegawa and Sato 2004)
    centroid  the whole load along the slip direction at the pressure centroid (one point)

and applies them to a disk and a ring, spinning in place under uniform pressure and sliding while
spinning under a tilted pressure, for several triangulations of the same face:

    rim-fan     fan from a vertex of the rim: what meshgen.py produces
    centre-fan  fan from the centre
    zigzag      a strip of slivers across the face
    hex         rings of nearly equilateral triangles, refined uniformly
    random      Delaunay triangulation of random interior points (seeded)
    quads       ring only: one layer of radial quadrilaterals split in two, as meshgen.py makes it
    layers      ring only: radial layers of nearly square quadrilaterals split in two

The reference of each row is the closed form on a reference triangulation of the same polygon; the
closed form's own rows show that it does not depend on the triangulation (differences at round-off).
Errors: `moment_error` is (M − M_ref)/M_ref about the centre of the face (negative: the rule gives
too little friction torque), `force_error` is |F − F_ref| over μ times the load.

    uv run tools/ablation_triangulation.py            # writes results/ablation_triangulation.csv
                                                      # and results/figures/ablation_triangulation.*
    uv run tools/ablation_triangulation.py --no-figure --out FILE
"""

import argparse
import csv
import math
from pathlib import Path

import numpy as np

MU = 1.0
RULES = ("closed", "vertex", "centroid")
SEED = 20261112


def perp(v):
    """v turned a quarter turn counter-clockwise."""
    return np.array([-v[1], v[0]])


def cross(a, b):
    return a[0] * b[1] - a[1] * b[0]


# ---- the three rules on one triangle ---------------------------------------------------------------
# x: 3×2 vertices, counter-clockwise; p: pressure at the vertices; the face slips with angular
# velocity omega (about +z) around the point q. Each rule returns the friction force on the face
# and its moment about the origin.


def slip_direction(x, q, omega):
    r = x - q
    d = np.linalg.norm(r)
    return np.sign(omega) * perp(r) / d if d > 0 else np.zeros(2)


def primitives(t):
    s = math.sqrt(1.0 + t * t)
    a = math.asinh(t)
    return a, s, 0.5 * (s * t + a), s**3 / 3.0


def polar_moments(q, x):
    """D = ∫|r|, R = ∫r|r|, U = ∫r/|r|, W = ∫r rᵀ/|r| over the triangle, r = x − q (the paper's Eq. edge)."""
    D, R, U, W = 0.0, np.zeros(2), np.zeros(2), np.zeros((2, 2))
    for a, b in ((x[0], x[1]), (x[1], x[2]), (x[2], x[0])):
        ra, rb, d = a - q, b - q, b - a
        foot = ra - (ra @ d) / (d @ d) * d
        h = np.linalg.norm(foot)
        if h <= 1e-14 * np.linalg.norm(d):
            continue
        e1 = foot / h
        e2 = perp(e1)
        Pa, Pb = primitives(ra @ e2 / h), primitives(rb @ e2 / h)
        i1, i2, i3, i4 = (Pb[k] - Pa[k] for k in range(4))
        D += h**3 / 3.0 * i3
        R += h**4 / 4.0 * (i3 * e1 + i4 * e2)
        U += h**2 / 2.0 * (i1 * e1 + i2 * e2)
        W += h**3 / 3.0 * (i1 * np.outer(e1, e1) + i2 * (np.outer(e1, e2) + np.outer(e2, e1)) + (i3 - i1) * np.outer(e2, e2))
    return D, R, U, W


def gradient(x, p):
    area2 = cross(x[1] - x[0], x[2] - x[0])
    return (p[0] * perp(x[2] - x[1]) + p[1] * perp(x[0] - x[2]) + p[2] * perp(x[1] - x[0])) / area2


def closed(x, p, q, omega):
    g = gradient(x, p)
    pq = p[0] + g @ (q - x[0])
    D, R, U, W = polar_moments(q, x)
    sigma = np.sign(omega)
    force = -MU * sigma * perp(pq * U + W @ g)
    return force, -MU * sigma * (pq * D + g @ R) + cross(q, force)


def vertex(x, p, q, omega):
    area = 0.5 * cross(x[1] - x[0], x[2] - x[0])
    f = [-MU * p[i] * slip_direction(x[i], q, omega) for i in range(3)]
    force = area / 3.0 * sum(f)
    moment = sum(area / 12.0 * (1 + (i == j)) * cross(x[j], f[i]) for i in range(3) for j in range(3))
    return force, moment


def centroid(x, p, q, omega):
    area = 0.5 * cross(x[1] - x[0], x[2] - x[0])
    load = area * p.sum() / 3.0
    c = area / 12.0 * (p.sum() * x.sum(axis=0) + p @ x) / load
    force = -MU * load * slip_direction(c, q, omega)
    return force, cross(c, force)


RULE = {"closed": closed, "vertex": vertex, "centroid": centroid}


def wrench(rule, points, triangles, pressure, q, omega=1.0):
    """Friction force and moment about the origin of a triangulated face; pressure(x) is linear."""
    force, moment = np.zeros(2), 0.0
    for tri in triangles:
        x = points[list(tri)]
        f, m = RULE[rule](x, np.array([pressure(v) for v in x]), q, omega)
        force += f
        moment += m
    return force, moment


def load(points, triangles, pressure):
    total = 0.0
    for tri in triangles:
        x = points[list(tri)]
        total += 0.5 * cross(x[1] - x[0], x[2] - x[0]) * sum(pressure(v) for v in x) / 3.0
    return total


def min_angle(points, triangles):
    worst = math.pi
    for tri in triangles:
        x = points[list(tri)]
        for k in range(3):
            a, b = x[(k + 1) % 3] - x[k], x[(k + 2) % 3] - x[k]
            worst = min(worst, math.acos(np.clip(a @ b / (np.linalg.norm(a) * np.linalg.norm(b)), -1, 1)))
    return math.degrees(worst)


# ---- triangulations ---------------------------------------------------------------------------------


def circle(n, radius=1.0, phase=0.0):
    a = phase + 2 * np.pi * np.arange(n) / n
    return np.c_[radius * np.cos(a), radius * np.sin(a)]


def rim_fan(n):
    """meshgen.py's `fan`: the polygon split from its first vertex."""
    return circle(n), [(0, i, i + 1) for i in range(1, n - 1)]


def centre_fan(n):
    return np.vstack([circle(n), [0.0, 0.0]]), [(n, i, (i + 1) % n) for i in range(n)]


def zigzag(n):
    """A strip across the polygon: 0, 1, n−1, 2, n−2, ...; every triangle is a sliver."""
    order = [0]
    lo, hi = 1, n - 1
    while lo <= hi:
        order.append(lo)
        if lo != hi:
            order.append(hi)
        lo, hi = lo + 1, hi - 1
    tris = []
    for a, b, c in zip(order, order[1:], order[2:]):
        tris.append((a, b, c) if cross(circle(n)[b] - circle(n)[a], circle(n)[c] - circle(n)[a]) > 0 else (a, c, b))
    return circle(n), tris


def hex_mesh(m):
    """Rings j = 1..m of 6j points around the centre: 6m² nearly equilateral triangles in a 6m-gon."""
    points, start = [np.zeros((1, 2))], [0]
    for j in range(1, m + 1):
        start.append(start[-1] + (1 if j == 1 else 6 * (j - 1)))
        points.append(circle(6 * j, j / m))
    points = np.vstack(points)
    ring = lambda j, i: 0 if j == 0 else start[j] + i % (6 * j)
    tris = []
    for j in range(m):
        for s in range(6):
            for i in range(j + 1):
                tris.append((ring(j, s * j + i), ring(j + 1, s * (j + 1) + i), ring(j + 1, s * (j + 1) + i + 1)))
            for i in range(j):
                tris.append((ring(j, s * j + i), ring(j + 1, s * (j + 1) + i + 1), ring(j, s * j + i + 1)))
    return points, tris


def random_mesh(n, interior, rng):
    """Delaunay triangulation of the n-gon's vertices and `interior` random points inside it."""
    from scipy.spatial import Delaunay

    r = math.cos(math.pi / n) * np.sqrt(rng.uniform(0, 0.98, interior))
    a = rng.uniform(0, 2 * np.pi, interior)
    points = np.vstack([circle(n), np.c_[r * np.cos(a), r * np.sin(a)]])
    tris = []
    for t in Delaunay(points).simplices:
        a, b, c = (int(i) for i in t)
        tris.append((a, b, c) if cross(points[b] - points[a], points[c] - points[a]) > 0 else (a, c, b))
    return points, tris


def ring_layers(n, layers, inner):
    """An annulus between the n-gons of radius `inner` and 1: radial quadrilaterals split in two, the
    way meshgen.py's `fan` splits a quadrilateral (from its first vertex)."""
    radii = np.linspace(inner, 1.0, layers + 1)
    points = np.vstack([circle(n, r) for r in radii])
    tris = []
    for k in range(layers):
        for i in range(n):
            a, b = k * n + i, k * n + (i + 1) % n
            c, d = (k + 1) * n + (i + 1) % n, (k + 1) * n + i
            tris += [(a, d, c), (a, c, b)]
    return points, tris


# ---- the experiment ---------------------------------------------------------------------------------

RING_INNER = 0.6  # the ring of the battery: inner radius 0.15 m, outer 0.25 m
CASES = {
    # name: (pressure, instantaneous centre of the slip)
    "spin": (lambda x: 1.0, np.zeros(2)),
    "slide+spin": (lambda x: 1.0 + 0.5 * x[0], np.array([0.0, 0.5])),
}


def meshes():
    """(shape, triangulation, parameter, points, triangles, reference points, reference triangles)."""
    rng = np.random.default_rng(SEED)
    out = []
    for n in (16, 32, 64, 128):
        ref = centre_fan(n)
        out.append(("disk", "rim-fan", n, *rim_fan(n), *ref))
        out.append(("disk", "centre-fan", n, *centre_fan(n), *ref))
        out.append(("disk", "zigzag", n, *zigzag(n), *ref))
    for m in (1, 2, 4, 8, 16):
        out.append(("disk", "hex", m, *hex_mesh(m), *centre_fan(6 * m)))
    for interior in (30, 300, 3000):
        out.append(("disk", "random", interior, *random_mesh(64, interior, rng), *centre_fan(64)))
    for n in (16, 32, 64, 128):
        out.append(("ring", "quads", n, *ring_layers(n, 1, RING_INNER), *ring_layers(n, 1, RING_INNER)))
    for layers in (1, 2, 4, 8):
        n = 16 * layers  # nearly square cells: 0.4/layers by about 2π·0.8/n
        out.append(("ring", "layers", layers, *ring_layers(n, layers, RING_INNER), *ring_layers(n, 1, RING_INNER)))
    return out


def run():
    rows = []
    for shape, name, parameter, points, tris, ref_points, ref_tris in meshes():
        for case, (pressure, q) in CASES.items():
            f_ref, m_ref = wrench("closed", ref_points, ref_tris, pressure, q)
            total = load(ref_points, ref_tris, pressure)
            for rule in RULES:
                f, m = wrench(rule, points, tris, pressure, q)
                rows.append(
                    {
                        "shape": shape,
                        "case": case,
                        "triangulation": name,
                        "parameter": parameter,
                        "triangles": len(tris),
                        "min_angle_deg": f"{min_angle(points, tris):.3g}",
                        "rule": rule,
                        "moment_error": f"{(m - m_ref) / m_ref:.3e}",
                        "force_error": f"{np.linalg.norm(f - f_ref) / (MU * total):.3e}",
                    }
                )
    return rows


def write(rows, path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]), lineterminator="\n")
        w.writeheader()
        w.writerows(rows)


STYLE = {  # triangulation -> (label, marker, linestyle, colour); Okabe-Ito
    "rim-fan": ("fan from the rim (the battery's)", "s", "-", "#D55E00"),
    "centre-fan": ("fan from the centre", "o", "-", "#0072B2"),
    "zigzag": ("strip of slivers", "v", ":", "#CC79A7"),
    "hex": ("nearly equilateral, refined", "D", "--", "#009E73"),
    "random": ("random points, Delaunay", "^", "-.", "#000000"),
}


def figure(rows, path, width=4.92):
    """Error of the two quadratures on the spinning disk against the number of triangles."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(width, 2.3), sharey=True)
    for ax, rule, title in zip(axes, ("centroid", "vertex"), ("One-point rule", "Vertex rule")):
        for name, (label, marker, ls, colour) in STYLE.items():
            pts = [
                (int(r["triangles"]), abs(float(r["moment_error"])))
                for r in rows
                if (r["shape"], r["case"], r["rule"], r["triangulation"]) == ("disk", "spin", rule, name)
            ]
            ax.plot(*zip(*sorted(pts)), marker=marker, linestyle=ls, color=colour, markersize=3.5, linewidth=0.9, label=label)
        ax.set_xscale("log")
        ax.set_yscale("log")
        ax.set_title(title, fontsize=8, pad=2)
        ax.set_xlabel("Triangles of the flat face", fontsize=8)
        ax.tick_params(labelsize=7)
        ax.grid(True, linewidth=0.3, color="#CCCCCC")
    axes[0].set_ylabel("Error of the spin torque", fontsize=8)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, ncol=2, loc="upper center", bbox_to_anchor=(0.5, -0.17), frameon=False, fontsize=6.5)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, bbox_inches="tight", metadata={"CreationDate": None, "ModDate": None, "Producer": None, "Creator": None} if path.suffix == ".pdf" else None, dpi=150)
    plt.close(fig)


def main(argv=None):
    root = Path(__file__).resolve().parents[1]
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", default=root / "results" / "ablation_triangulation.csv")
    ap.add_argument("--no-figure", action="store_true")
    a = ap.parse_args(argv)
    rows = run()
    write(rows, a.out)
    if not a.no_figure:
        for suffix in (".pdf", ".png"):
            figure(rows, Path(a.out).parent / "figures" / f"ablation_triangulation{suffix}")
    for r in rows:
        if r["rule"] != "closed":
            print(
                f"{r['shape']:5s} {r['case']:10s} {r['triangulation']:10s} {r['parameter']:>5} {r['triangles']:>6} tri  "
                f"{r['rule']:8s} moment {100 * float(r['moment_error']):+8.3f} %  force {100 * float(r['force_error']):7.3f} %"
            )
    worst = max(max(abs(float(r["moment_error"])), float(r["force_error"])) for r in rows if r["rule"] == "closed")
    print(f"closed form: largest difference between triangulations of one face {worst:.1e}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
