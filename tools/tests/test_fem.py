"""tools/fem: the finite-element reference of `disk` and `bearing`, verified against closed-form
contact solutions (Sneddon's flat punch, the Winkler limit of a thin layer, Hertz line contact)
and against its committed results.

Needs scikit-fem (skipped without it):
    uv run --with pytest --with numpy --with scipy --with scikit-fem --with matplotlib pytest tools/tests/test_fem.py
"""

import csv
import math
import sys
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("skfem")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "fem"))
import bearing
import disk
import solver

ROOT = Path(__file__).resolve().parents[2]


def test_flat_punch_on_a_deep_layer_is_sneddons_half_space():
    """Rigid flat punch on a layer 40 radii deep: p ∝ 1/√(1 − r²/R²), so f = π/4, and N = 2E*Rδ
    (the residual stiffening is the layer's finite depth)."""
    nu = 0.3
    res = disk.solve(40.0, nu, level=1)
    assert res.ratio == pytest.approx(math.pi / 4, rel=2e-3)
    assert res.force / (2 / (1 - nu**2) * disk.R * res.delta) == pytest.approx(1.0, rel=0.03)


def test_flat_punch_on_a_thin_compressible_layer_is_winkler():
    """ν = 0, H/R = 0.01: each fibre of the layer is a spring K = E/H, the pressure is uniform
    (f = 2/3) and N = K π R² δ, up to an edge zone of width ~H."""
    res = disk.solve(0.01, 0.0, level=0)
    assert res.ratio == pytest.approx(2 / 3, rel=3e-3)
    assert res.stiffness_vs_winkler == pytest.approx(1.0, rel=1e-2)


def hertz_line_contact(level, delta=1.6e-3, nu=0.3, radius=1.0, half_width=0.05):
    """A rigid cylinder pressed by δ onto a deep elastic block (plane strain, E = 1, half model)."""
    n = 8 * 2**level
    depth = 40 * half_width
    xs = np.concatenate([np.linspace(0, 2 * half_width, 2 * n + 1), solver.graded(2 * half_width, depth, n, 3.0, "a")[1:]])
    ys = solver.graded(-depth, 0.0, 2 * n, 3.0, "b")
    body = solver.elastic_body(solver.tensor_mesh(xs, ys), 1.0, nu)
    x, y = body.nodes
    tol = 1e-12
    fixed = np.concatenate([body.dofs[:, y < -depth + tol].ravel(), body.dofs[0, x < tol]])
    top = y > -tol
    order = np.argsort(x[top])
    points = body.nodes[:, top][:, order]
    normals, required = solver.rigid_circle_constraints(points, np.array([0.0, radius - delta]), radius)
    sol = solver.solve_contact(body, fixed, body.node_dofs(top)[:, order], normals, required)
    load = 2 * abs(sol.force @ normals[1])
    pressure = solver.boundary_pressure(points[0], sol.force * np.abs(normals[1]))
    return points[0], sol, load, pressure


def test_hertz_line_contact():
    """Contact half-width a = √(4PR/(πE*)) within a node spacing and p = p0 √(1 − x²/a²) within 1 %
    of p0, with the contact zone found by the active set (the rigid-circle constraint)."""
    x, sol, load, pressure = hertz_line_contact(level=1)
    nu = 0.3
    a = math.sqrt(4 * load * 1.0 / (math.pi / (1 - nu**2)))
    p0 = 2 * load / (math.pi * a)
    edge = x[sol.active & (sol.force > 0)].max()
    spacing = x[1] - x[0]
    assert abs(edge - a) <= spacing
    assert pressure[0] == pytest.approx(p0, rel=1e-2)
    near = x < 1.5 * a
    exact = p0 * np.sqrt(np.clip(1 - (x[near] / a) ** 2, 0, None))
    assert np.sqrt(np.mean((pressure[near] - exact) ** 2)) < 1e-2 * p0


def test_thin_sleeve_without_poisson_is_the_winkler_cosine():
    """A wall of r/25 with ν = 0 behaves as radial springs: p ∝ cos φ over the lower half, f = 4/π,
    and the stiffness of a ring of fibres, K = E/(r ln(1 + t/r)) (2 % above E/t for this wall)."""
    wall = bearing.RADIUS / 25
    res = bearing.solve(0.0, level=0, load=1e-3 * bearing.LOAD_BATTERY, wall=wall)
    assert res.ratio == pytest.approx(4 / math.pi, rel=5e-3)
    ring = wall / (bearing.RADIUS * math.log(1 + wall / bearing.RADIUS))
    assert res.stiffness_vs_winkler == pytest.approx(ring, rel=5e-3)
    assert math.degrees(res.contact_half_angle) == pytest.approx(90.0, abs=3.0)


def test_active_set_releases_the_upper_bore():
    """Zero clearance: the shaft moved down leaves the upper half of the bore, and no node ends in
    tension or penetrating."""
    sleeve = bearing.Sleeve(0.3, 0, bearing.winkler_modulus(0.3))
    sol, _, _ = sleeve.solve(1e-6)
    assert np.all(sleeve.phi[sol.active] < 0.6 * math.pi)
    assert np.all(sol.force >= 0)


def committed():
    with open(ROOT / "results" / "fem" / "fem_reference.csv") as f:
        return [r for r in csv.DictReader(f) if r["level"] == "0"]


def test_committed_reference_reproduces_at_the_coarsest_level():
    rows = committed()
    assert {r["case"] for r in rows} == {"disk", "bearing"}
    for r in rows:
        nu = float(r["nu"])
        if r["case"] == "disk":
            res = disk.solve(float(r["H_over_R"]), nu, 0)
        else:
            res = bearing.solve(nu, 0, float(r["load_N_per_m"]))
        assert res.ratio == pytest.approx(float(r["ratio"]), rel=1e-5)


def test_band_members_bracket_the_reference():
    """Every converged ratio lies inside its scenario's band (disk: 1/2 to π/4 up to the
    half-space; bearing: 1 to π/2)."""
    with open(ROOT / "results" / "fem" / "fem_reference.csv") as f:
        rows = [r for r in csv.DictReader(f) if r["level"] == "converged"]
    for r in rows:
        f_ = float(r["ratio"])
        if r["case"] == "disk":
            assert 0.5 < f_ < math.pi / 4 + 1e-3
        else:
            assert 1.0 < f_ < math.pi / 2
