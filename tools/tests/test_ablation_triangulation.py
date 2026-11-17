"""tools/ablation_triangulation.py: the three friction rules and the committed experiment."""

import csv
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import ablation_triangulation as at

ROOT = Path(__file__).resolve().parents[2]


def brute_force(x, p, q, omega, n=400):
    """Midpoint quadrature of −μ·p·v_t/|v_t| on n² sub-triangles: force and moment about the origin."""
    i, j = np.meshgrid(np.arange(n), np.arange(n), indexing="ij")
    up = np.stack([(i + 1 / 3) / n, (j + 1 / 3) / n], axis=-1)[i + j < n]
    down = np.stack([(i + 2 / 3) / n, (j + 2 / 3) / n], axis=-1)[i + j < n - 1]
    uv = np.concatenate([up, down])
    pts = x[0] + uv[:, :1] * (x[1] - x[0]) + uv[:, 1:] * (x[2] - x[0])
    pressure = p[0] + uv[:, 0] * (p[1] - p[0]) + uv[:, 1] * (p[2] - p[0])
    r = pts - q
    direction = np.sign(omega) * np.c_[-r[:, 1], r[:, 0]] / np.linalg.norm(r, axis=1)[:, None]
    w = 0.5 * at.cross(x[1] - x[0], x[2] - x[0]) / len(pts)
    f = -at.MU * w * pressure[:, None] * direction
    return f.sum(axis=0), (pts[:, 0] * f[:, 1] - pts[:, 1] * f[:, 0]).sum()


X = np.array([[0.1, 0.0], [1.0, 0.2], [0.3, 0.9]])
P = np.array([1.0, 2.0, 0.5])


@pytest.mark.parametrize("q", [np.array([2.0, -1.0]), np.array([0.45, 0.35]), X[1].copy()])
def test_closed_form_is_the_integral(q):
    force, moment = at.closed(X, P, q, 1.0)
    f_ref, m_ref = brute_force(X, P, q, 1.0)
    assert np.allclose(force, f_ref, atol=2e-4) and abs(moment - m_ref) < 2e-4


def test_the_three_rules_agree_when_the_slip_is_uniform():
    q = np.array([0.0, 1e3])  # a centre this far: one slip direction over the triangle, to 1e-3
    ref = at.closed(X, P, q, -1.0)
    for rule in (at.vertex, at.centroid):
        force, moment = rule(X, P, q, -1.0)
        assert np.allclose(force, ref[0], atol=3e-3) and abs(moment - ref[1]) < 3e-3


def spin_torque(rule, mesh):
    points, tris = mesh
    return at.wrench(rule, points, tris, lambda x: 1.0, np.zeros(2))[1]


def test_closed_form_does_not_depend_on_the_triangulation():
    ref = spin_torque("closed", at.centre_fan(64))
    rng = np.random.default_rng(1)
    for mesh in (at.rim_fan(64), at.zigzag(64), at.random_mesh(64, 200, rng)):
        assert abs(spin_torque("closed", mesh) / ref - 1) < 1e-12
    # and it is the torque of the polygon: (2/3)·μ·N·R up to the difference with the circle
    assert abs(-ref / (2 * math.pi / 3) - 1) < 3e-3


def test_one_point_rule_depends_on_where_the_fan_starts():
    ref = spin_torque("closed", at.centre_fan(64))
    rim = spin_torque("centroid", at.rim_fan(64)) / ref - 1
    centre = spin_torque("centroid", at.centre_fan(64)) / ref - 1
    assert -0.185 < rim < -0.175  # the battery's caps: 18 % too little torque
    assert abs(centre + (math.tan(math.pi / 64) ** 2) / 6) < 1e-5  # −tan²(π/n)/6 = −0.04 %
    # refining the rim does not help the rim fan, and it does help the centre fan
    assert abs(spin_torque("centroid", at.rim_fan(128)) / spin_torque("closed", at.centre_fan(128)) - 1) > 0.17


def test_vertex_rule_loses_a_quarter_of_the_torque_on_any_fan():
    ref = spin_torque("closed", at.centre_fan(64))
    for mesh in (at.rim_fan(64), at.centre_fan(64)):
        assert abs(spin_torque("vertex", mesh) / ref - 0.75) < 2e-3


def test_quadratures_converge_at_second_order_on_well_shaped_meshes():
    errors = {}
    for m in (4, 8, 16):
        ref = spin_torque("closed", at.centre_fan(6 * m))
        errors[m] = {rule: abs(spin_torque(rule, at.hex_mesh(m)) / ref - 1) for rule in ("vertex", "centroid")}
    for rule in ("vertex", "centroid"):
        assert 3.3 < errors[4][rule] / errors[8][rule] < 4.7
        assert 3.3 < errors[8][rule] / errors[16][rule] < 4.7


def test_meshes_cover_their_faces():
    for points, tris in (at.rim_fan(64), at.centre_fan(64), at.zigzag(64), at.hex_mesh(4)):
        n = 24 if len(tris) == 96 else 64
        area = sum(0.5 * at.cross(points[b] - points[a], points[c] - points[a]) for a, b, c in tris)
        assert abs(area - 0.5 * n * math.sin(2 * math.pi / n)) < 1e-12
        assert all(at.cross(points[b] - points[a], points[c] - points[a]) > 0 for a, b, c in tris)
    points, tris = at.ring_layers(32, 2, at.RING_INNER)
    area = sum(0.5 * at.cross(points[b] - points[a], points[c] - points[a]) for a, b, c in tris)
    assert abs(area - 0.5 * 32 * math.sin(2 * math.pi / 32) * (1 - at.RING_INNER**2)) < 1e-12


def test_committed_csv_is_what_the_script_produces():
    path = ROOT / "results" / "ablation_triangulation.csv"
    with path.open(newline="") as f:
        committed = list(csv.DictReader(f))
    fresh = at.run()
    assert len(committed) == len(fresh)
    for old, new in zip(committed, fresh):
        for key in ("shape", "case", "triangulation", "parameter", "triangles", "rule"):
            assert str(old[key]) == str(new[key])
        for key in ("moment_error", "force_error"):
            assert float(old[key]) == pytest.approx(float(new[key]), rel=1e-3, abs=1e-12)
