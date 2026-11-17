# /// script
# requires-python = ">=3.10"
# dependencies = ["numpy"]
# ///
"""Numerical check of the closed forms derived in the paper (Section "Friction").

Standalone: it re-implements the formulas as they are printed in the paper and
compares them with quadrature that does not use them. It does not call the C++
core (the core has its own tests, tests/test_polar_moments.cpp).

  1. The four polar moments of a triangle about a point q of its plane,
       D = ∫|r| dA,  R = ∫ r|r| dA,  U = ∫ r/|r| dA,  W = ∫ r rᵀ/|r| dA,  r = x − q,
     for q outside, inside, on an edge and at a vertex of the triangle.
  2. The Coulomb friction force and moment on a triangle with a linear pressure
     under a rigid slip field, against brute-force quadrature of −μ p v_t/|v_t|.
  3. The limit of a far instantaneous centre against the uniform-slip formula
     (second order in the force, first order in the moment).
  4. A spinning disk under uniform pressure: torque (2/3) μ N R.
  5. The integral of a product of two affine functions times a barycentric
     coordinate, and the linear pressure with the same force and moment.

Run: uv run paper/checks/check_polar_moments.py   (exit code 0 when every check passes)
"""

import sys

import numpy as np

rng = np.random.default_rng(20261111)
failures = []


def check(name, error, tolerance):
    ok = error <= tolerance
    print(f"  {'ok  ' if ok else 'FAIL'} {name}: error {error:.2e} (tolerance {tolerance:.0e})")
    if not ok:
        failures.append(name)


# --- The closed form, as in the paper ------------------------------------------


def primitives(t):
    """Antiderivatives in t = tan θ of sec θ, sec θ tan θ, sec³ θ, sec³ θ tan θ."""
    s = np.sqrt(1.0 + t * t)
    a = np.arcsinh(t)
    return a, s, 0.5 * (s * t + a), s**3 / 3.0


def polar_moments(q, p0, p1, p2, m):
    """D, R, U, W over the triangle (p0, p1, p2), counter-clockwise about the unit normal m."""
    D, R, U, W = 0.0, np.zeros(3), np.zeros(3), np.zeros((3, 3))
    for a, b in ((p0, p1), (p1, p2), (p2, p0)):
        ra, rb, d = a - q, b - q, b - a
        foot = ra - (ra @ d) / (d @ d) * d  # from q to the line of the edge
        h = np.linalg.norm(foot)
        if h <= 1e-15 * np.linalg.norm(d):
            continue  # q on the line of the edge: the sub-triangle has no area
        e1 = foot / h
        e2 = np.cross(m, e1)
        Pa, Pb = primitives(ra @ e2 / h), primitives(rb @ e2 / h)
        s1, st, s3, s3t = (Pb[k] - Pa[k] for k in range(4))
        D += h**3 / 3.0 * s3
        R += h**4 / 4.0 * (s3 * e1 + s3t * e2)
        U += h**2 / 2.0 * (s1 * e1 + st * e2)
        W += h**3 / 3.0 * (
            s1 * np.outer(e1, e1) + st * (np.outer(e1, e2) + np.outer(e2, e1)) + (s3 - s1) * np.outer(e2, e2)
        )
    return D, R, U, W


def friction_wrench(x, p, m, v0, w, mu):
    """Coulomb friction on the triangle x[0..2] (unit normal m) with pressure p[0..2] at its
    vertices, when the other body moves relative to it with velocity v0 at x[0] and angular
    velocity w. Returns the force and its moment about x[0]. Closed form of the paper."""
    area2 = np.linalg.norm(np.cross(x[1] - x[0], x[2] - x[0]))
    g = (p[0] * np.cross(m, x[2] - x[1]) + p[1] * np.cross(m, x[0] - x[2]) + p[2] * np.cross(m, x[1] - x[0])) / area2
    vt0 = v0 - (v0 @ m) * m
    wm = w @ m
    q = x[0] + np.cross(m, vt0) / wm
    pq = p[0] + g @ (q - x[0])
    D, R, U, W = polar_moments(q, x[0], x[1], x[2], m)
    sigma = np.sign(wm)
    force = -mu * sigma * np.cross(m, pq * U + W @ g)
    moment_q = -mu * sigma * (pq * D + g @ R) * m
    return force, moment_q + np.cross(q - x[0], force)


def friction_wrench_uniform_slip(x, p, m, v0, w, mu):
    """The same when the instantaneous centre is far: one slip direction, taken at the pressure centre."""
    area = 0.5 * np.linalg.norm(np.cross(x[1] - x[0], x[2] - x[0]))
    P = area * p.sum() / 3.0
    Px = area / 12.0 * (p.sum() * x.sum(axis=0) + p @ x)
    c = Px / P
    v = v0 + np.cross(w, c - x[0])
    vt = v - (v @ m) * m
    u = vt / np.linalg.norm(vt)
    return -mu * P * u, np.cross(Px - P * x[0], -mu * u)


# --- Quadrature that does not use the closed form ------------------------------


def midpoint_points(p0, p1, p2, n):
    """Centroids and equal weights of the n² congruent sub-triangles of a triangle."""
    i, j = np.meshgrid(np.arange(n), np.arange(n), indexing="ij")
    keep = i + j < n
    i, j = i[keep], j[keep]
    up = np.stack([(i + 1 / 3) / n, (j + 1 / 3) / n], axis=1)  # "upward" sub-triangles
    keep = i + j < n - 1
    down = np.stack([(i[keep] + 2 / 3) / n, (j[keep] + 2 / 3) / n], axis=1)
    uv = np.concatenate([up, down])
    pts = p0 + uv[:, :1] * (p1 - p0) + uv[:, 1:] * (p2 - p0)
    area = 0.5 * np.linalg.norm(np.cross(p1 - p0, p2 - p0))
    return pts, area / len(pts)


def gauss_points(p0, p1, p2, n):
    """Tensor Gauss–Legendre rule on the triangle through the collapsed (Duffy) square."""
    t, wt = np.polynomial.legendre.leggauss(n)
    t, wt = 0.5 * (t + 1.0), 0.5 * wt
    u, v = np.meshgrid(t, t, indexing="ij")
    w = np.outer(wt, wt) * (1.0 - u)
    a, b = u.ravel(), (v * (1.0 - u)).ravel()
    pts = p0 + a[:, None] * (p1 - p0) + b[:, None] * (p2 - p0)
    area2 = np.linalg.norm(np.cross(p1 - p0, p2 - p0))
    return pts, w.ravel() * area2


def moments_by_quadrature(q, pts, wts):
    r = pts - q
    d = np.linalg.norm(r, axis=1)
    wts = np.broadcast_to(wts, d.shape)
    rhat = r / d[:, None]
    return (
        wts @ d,
        (wts * d) @ r,
        wts @ rhat,
        np.einsum("k,ki,kj->ij", wts, r, rhat),
    )


def moment_error(got, ref, scale, area):
    """Largest error of the four moments, each over its natural magnitude."""
    D, R, U, W = got
    Dr, Rr, Ur, Wr = ref
    return max(
        abs(D - Dr) / (area * scale),
        np.abs(R - Rr).max() / (area * scale**2),
        np.abs(U - Ur).max() / area,
        np.abs(W - Wr).max() / (area * scale),
    )


def random_triangle():
    """A random triangle in a random plane, counter-clockwise about its normal m."""
    m = rng.normal(size=3)
    m /= np.linalg.norm(m)
    e1 = np.cross(m, rng.normal(size=3))
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(m, e1)
    while True:
        c = rng.uniform(-1, 1, size=(3, 2))
        twice_area = (c[1, 0] - c[0, 0]) * (c[2, 1] - c[0, 1]) - (c[1, 1] - c[0, 1]) * (c[2, 0] - c[0, 0])
        if abs(twice_area) > 0.3:
            break
    if twice_area < 0:
        c = c[::-1]
    origin = rng.normal(size=3)
    x = origin + c[:, :1] * e1 + c[:, 1:] * e2
    return x, m, e1, e2


# --- 1. The polar moments --------------------------------------------------------

print("1. Polar moments of a triangle")
worst = {"outside (Gauss)": 0.0, "outside": 0.0, "inside": 0.0, "on an edge": 0.0, "at a vertex": 0.0}
for _ in range(40):
    x, m, e1, e2 = random_triangle()
    area = 0.5 * np.linalg.norm(np.cross(x[1] - x[0], x[2] - x[0]))
    centre = x.mean(axis=0)
    size = max(np.linalg.norm(x[k] - centre) for k in range(3))
    lam = rng.dirichlet(np.ones(3))
    angle = rng.uniform(0, 2 * np.pi)
    cases = {
        # Far enough for the integrand to be analytic on the triangle: Gauss is exact to round-off.
        "outside (Gauss)": centre + rng.uniform(2.5, 6.0) * size * (np.cos(angle) * e1 + np.sin(angle) * e2),
        "outside": centre + rng.uniform(1.05, 1.5) * size * (np.cos(angle) * e1 + np.sin(angle) * e2),
        "inside": lam @ x,
        "on an edge": lam[0] * x[0] + (1 - lam[0]) * x[1],
        "at a vertex": x[2].copy(),
    }
    mid = midpoint_points(x[0], x[1], x[2], 500)
    gauss = gauss_points(x[0], x[1], x[2], 60)
    for name, q in cases.items():
        scale = max(size, np.linalg.norm(q - centre))
        ref = moments_by_quadrature(q, *(gauss if name == "outside (Gauss)" else mid))
        worst[name] = max(worst[name], moment_error(polar_moments(q, x[0], x[1], x[2], m), ref, scale, area))
check("q outside, against 60×60 Gauss–Legendre", worst["outside (Gauss)"], 1e-11)
for name in ("outside", "inside", "on an edge", "at a vertex"):
    check(f"q {name}, against 250 000 midpoint samples", worst[name], 2e-5)

# --- 2. The friction wrench --------------------------------------------------------

print("2. Coulomb friction wrench for a linear pressure under a rigid slip field")
worst_force = worst_moment = 0.0
for _ in range(40):
    x, m, e1, e2 = random_triangle()
    p = rng.uniform(0.2, 2.0, size=3)
    mu = rng.uniform(0.1, 1.0)
    w = rng.normal(size=3)  # any direction: only its component along m turns the slip
    v0 = rng.normal(size=3)  # any direction: its normal component does not slip
    force, moment = friction_wrench(x, p, m, v0, w, mu)
    pts, wt = midpoint_points(x[0], x[1], x[2], 500)
    lam = np.linalg.lstsq(np.c_[x[1] - x[0], x[2] - x[0]], (pts - x[0]).T, rcond=None)[0].T
    pp = p[0] + lam @ (p[1:] - p[0])
    v = v0 + np.cross(w, pts - x[0])
    vt = v - np.outer(v @ m, m)
    traction = -mu * pp[:, None] * vt / np.linalg.norm(vt, axis=1)[:, None]
    force_ref = wt * traction.sum(axis=0)
    moment_ref = wt * np.cross(pts - x[0], traction).sum(axis=0)
    load = mu * wt * pp.sum()
    size = np.linalg.norm(x - x.mean(axis=0), axis=1).max()
    worst_force = max(worst_force, np.abs(force - force_ref).max() / load)
    worst_moment = max(worst_moment, np.abs(moment - moment_ref).max() / (load * size))
check("force, over μ∫p dA", worst_force, 2e-5)
check("moment, over μ∫p dA × size", worst_moment, 2e-5)

# --- 3. Far instantaneous centre ---------------------------------------------------

print("3. Far instantaneous centre: closed form against the uniform-slip formula")
# With the centre at a distance ρ·size the slip direction turns by 1/ρ across the triangle. Taking
# it at the pressure centre cancels the first order in the force (error ~ρ⁻²) but drops the
# friction couple of the triangle about that centre, which is of first order in the moment.
for ratio, force_tolerance, moment_tolerance in ((1e2, 1e-4, 1e-2), (1e3, 1e-6, 1e-3)):
    worst_force = worst_moment = 0.0
    for _ in range(40):
        x, m, e1, e2 = random_triangle()
        p = rng.uniform(0.2, 2.0, size=3)
        size = np.sqrt(np.linalg.norm(np.cross(x[1] - x[0], x[2] - x[0])))
        angle = rng.uniform(0, 2 * np.pi)
        v0 = np.cos(angle) * e1 + np.sin(angle) * e2
        w = (1.0 / (ratio * size)) * m * rng.choice([-1.0, 1.0])  # |ω_m|·size = |v_t| / ratio
        fa, ma = friction_wrench(x, p, m, v0, w, 1.0)
        fb, mb = friction_wrench_uniform_slip(x, p, m, v0, w, 1.0)
        load = np.linalg.norm(fb)
        worst_force = max(worst_force, np.abs(fa - fb).max() / load)
        worst_moment = max(worst_moment, np.abs(ma - mb).max() / (load * size))
    check(f"|v_t| = {ratio:.0e}·|ω_m|·size: force, over μ∫p dA", worst_force, force_tolerance)
    check(f"|v_t| = {ratio:.0e}·|ω_m|·size: moment, over μ∫p dA × size", worst_moment, moment_tolerance)

# --- 4. Spinning disk --------------------------------------------------------------

print("4. Disk under uniform pressure (regular polygon as a triangle fan)")
n, radius, mu, pressure = 256, 0.25, 0.6, 3.0
m = np.array([0.0, 0.0, 1.0])
theta = 2 * np.pi * np.arange(n + 1) / n
rim = radius * np.c_[np.cos(theta), np.sin(theta), np.zeros(n + 1)]
centre = np.zeros(3)
area = 0.5 * n * radius**2 * np.sin(2 * np.pi / n)
normal_load = pressure * area


def disk_wrench(v0, w):
    force, moment = np.zeros(3), np.zeros(3)
    for k in range(n):
        x = np.array([centre, rim[k], rim[k + 1]])
        f, mo = friction_wrench(x, np.full(3, pressure), m, v0, w, mu)
        force += f
        moment += mo  # about the centre, x[0]
    return force, moment


f, mo = disk_wrench(np.zeros(3), np.array([0.0, 0.0, 5.0]))
# Exact for the polygon: ∫|r| dA over n triangles with apex angle 2π/n and apothem a.
apothem, half = radius * np.cos(np.pi / n), np.pi / n
polygon = n * apothem**3 / 3.0 * (np.tan(half) / np.cos(half) + np.arcsinh(np.tan(half)))
check("pure spin: torque against the polygon's ∫|r| dA", abs(-mo[2] - mu * pressure * polygon) / (mu * normal_load * radius), 1e-12)
check("pure spin: torque against (2/3) μ N R", abs(-mo[2] / (mu * normal_load * radius) - 2 / 3), 1e-4)
check("pure spin: no net force", np.abs(f).max() / (mu * normal_load), 1e-12)
# Sliding and spinning (Contensou): against brute-force quadrature on a polar grid of the disk polygon.
v0, w = np.array([1.0, 0.0, 0.0]), np.array([0.0, 0.0, 10.0])
f, mo = disk_wrench(v0, w)
f_ref, mo_ref = np.zeros(3), np.zeros(3)
for k in range(n):
    pts, wt = midpoint_points(centre, rim[k], rim[k + 1], 60)
    v = v0 + np.cross(w, pts)
    traction = -mu * pressure * v / np.linalg.norm(v, axis=1)[:, None]
    f_ref += wt * traction.sum(axis=0)
    mo_ref += wt * np.cross(pts, traction).sum(axis=0)
check("sliding and spinning: force", np.abs(f - f_ref).max() / (mu * normal_load), 2e-5)
check("sliding and spinning: torque", np.abs(mo - mo_ref).max() / (mu * normal_load * radius), 2e-5)
check("sliding and spinning: force below μN (the spin takes part of the friction)", np.linalg.norm(f) / (mu * normal_load) - 1.0, 0.0)

# --- 5. Product of two affine functions ----------------------------------------------

print("5. ∫ f g λ_k dA and the equivalent linear pressure")
# Degree-5 rule on the triangle (Radon / Dunavant, 7 points): exact for cubics.
a1, b1 = (6 - np.sqrt(15)) / 21, (9 + 2 * np.sqrt(15)) / 21
a2, b2 = (6 + np.sqrt(15)) / 21, (9 - 2 * np.sqrt(15)) / 21
w1, w2 = (155 - np.sqrt(15)) / 1200, (155 + np.sqrt(15)) / 1200
bary = np.array(
    [[1 / 3, 1 / 3, 1 / 3], [b1, a1, a1], [a1, b1, a1], [a1, a1, b1], [b2, a2, a2], [a2, b2, a2], [a2, a2, b2]]
)
weights = np.array([9 / 40, w1, w1, w1, w2, w2, w2])
worst_product = worst_linear = 0.0
for _ in range(200):
    f, g = rng.normal(size=3), rng.normal(size=3)
    area = rng.uniform(0.1, 3.0)
    F, G, S = f.sum(), g.sum(), f @ g
    closed = area / 60.0 * (F * G + S + g * F + f * G + 2 * f * g)
    reference = area * (weights * (bary @ f) * (bary @ g)) @ bary
    worst_product = max(worst_product, np.abs(closed - reference).max() / area)
    p_lin = (g * F + f * G + 2 * f * g) / 5.0 - (F * G + S) / 20.0
    linear = area / 12.0 * (p_lin.sum() + p_lin)  # ∫ p̃ λ_k dA
    worst_linear = max(worst_linear, np.abs(linear - reference).max() / area)
check("∫ f g λ_k dA = A/60 (FG + S + g_k F + f_k G + 2 f_k g_k)", worst_product, 1e-13)
check("the linear pressure p̃ has the same ∫ p λ_k dA", worst_linear, 1e-13)

print("all checks passed" if not failures else f"FAILED: {', '.join(failures)}")
sys.exit(1 if failures else 0)
