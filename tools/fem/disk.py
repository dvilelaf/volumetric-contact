# /// script
# requires-python = ">=3.11"
# dependencies = ["numpy", "scipy", "scikit-fem", "matplotlib"]
# ///
"""`disk`: a rigid flat cylindrical punch on an elastic layer bonded to a rigid base.

The class B scenario `disk` (docs/scenarios.md §4) spins a flat disk of radius R = 0.25 m on the
ground; the friction torque of Coulomb sliding everywhere is τ = μ ∫ p r dA, so the stop time goes
as 1/f with

    f = τ / (μ N R) = ∫ p r dA / (N R).

The nominal theory takes a Winkler foundation (uniform pressure, f = 2/3); its band lists the rigid
punch on an elastic half-space (p ∝ 1/√(1 − r²/R²), f = π/4) and uniform wear (p ∝ 1/r, f = 1/2).
This module computes f for a continuum: an axisymmetric linear-elastic layer of thickness H bonded
to a rigid base, laterally unbounded (truncated at R + 10·max(H, R), traction-free), pressed by a
rigid flat frictionless punch (u_z = −δ under the punch where it touches, released where the
contact pressure would be tensile). The punch is displacement-controlled; with zero initial gap the
problem is linear and homogeneous, so f is independent of δ and of E and depends on H/R and ν only.

The spin does not change the normal pressure: its friction traction is circumferential and
axisymmetric, which in linear elasticity only produces a torsional displacement u_θ(r, z),
uncoupled from the meridional (u_r, u_z) problem. So the frictionless normal solution is exact for
the spinning punch within small strain; the torque assumes sliding everywhere (no stick core), as
the scenario does.

Run: uv run tools/fem/disk.py  (prints the table; tools/fem/run.py writes the committed results)
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import solver

R = 0.25  # punch radius of the scenario, m
K_BATTERY = 1e6  # Winkler stiffness of the battery's ground, N/m³
H_BATTERY = 1.0  # thickness of the battery's slab, m


@dataclass
class DiskResult:
    H_over_R: float
    nu: float
    level: int
    dofs: int
    ratio: float  # f = ∫ p r dA / (N R)
    stiffness_vs_winkler: float  # N / (K π R² δ) with K = M / H, the Winkler mapping of the layer
    r: np.ndarray  # contact node radii
    pressure: np.ndarray  # recovered pressure at those radii
    force: float  # N
    delta: float


def mesh_counts(level: int) -> tuple[int, int, int]:
    """Elements under the punch, beyond it, and through the thickness."""
    n = 16 * 2**level
    return n, n, n


def solve(H_over_R: float, nu: float, level: int, E: float = 1.0, radius: float = R) -> DiskResult:
    H = H_over_R * radius
    L = radius + 10.0 * max(H, radius)
    n_in, n_out, n_z = mesh_counts(level)
    rs = np.concatenate(
        [solver.graded(0.0, radius, n_in, 2.0, "b"), solver.graded(radius, L, n_out, 3.0, "a")[1:]]
    )
    zs = solver.graded(0.0, H, n_z, 3.0 if H > radius else 2.0, "b")
    body = solver.elastic_body(solver.tensor_mesh(rs, zs), E, nu, axisymmetric=True)
    r, z = body.nodes
    tol = 1e-9 * max(H, radius)
    fixed = np.concatenate([body.dofs[:, z < tol].ravel(), body.dofs[0, r < tol]])
    under = (z > H - tol) & (r < radius + tol)
    order = np.argsort(r[under])
    contact = body.node_dofs(under)[:, order]
    delta = 1e-3 * min(H, radius)
    m = contact.shape[1]
    normals = np.vstack([np.zeros(m), -np.ones(m)])  # the punch pushes down
    sol = solver.solve_contact(body, fixed, contact, normals, np.full(m, delta))
    rc = r[under][order]
    force = sol.force.sum()
    ratio = float((sol.force * rc).sum() / (force * radius))
    winkler = solver.constrained_modulus(E, nu) / H * np.pi * radius**2 * delta
    pressure = solver.boundary_pressure(rc, sol.force, lambda x: 2 * np.pi * x)
    return DiskResult(
        H_over_R=H_over_R,
        nu=nu,
        level=level,
        dofs=body.K.shape[0],
        ratio=ratio,
        stiffness_vs_winkler=float(force / winkler),
        r=rc,
        pressure=pressure,
        force=float(force),
        delta=delta,
    )


def winkler_modulus(nu: float, K: float = K_BATTERY, H: float = H_BATTERY) -> float:
    """The Young's modulus whose layer of thickness H has the Winkler stiffness K (K = M/H)."""
    return K * H * (1 + nu) * (1 - 2 * nu) / (1 - nu)


if __name__ == "__main__":
    for nu in (0.3, 0.45):
        for hr in (0.05, 0.2, 1.0, 4.0):
            vals = []
            for level in range(4):
                res = solve(hr, nu, level)
                vals.append(res.ratio)
                print(f"nu={nu} H/R={hr} level={level} dofs={res.dofs} f={res.ratio:.5f} N/N_W={res.stiffness_vs_winkler:.4f}")
            p, f_inf = solver.observed_order(vals)
            print(f"   observed order {p:.2f}, extrapolated f = {f_inf:.5f}")
