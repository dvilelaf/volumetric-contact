# /// script
# requires-python = ">=3.11"
# dependencies = ["numpy", "scipy", "scikit-fem", "matplotlib"]
# ///
"""`bearing`: a rigid journal in a zero-clearance elastic sleeve bonded to a rigid housing.

The class B scenario `bearing` (docs/scenarios.md §4) spins a shaft of radius r = 0.05 m in the
hole of `bearing_r50_t20` (wall t = 0.02 m) under its weight. With Coulomb sliding everywhere and
the friction circle, the torque is τ = f·μ·N·r/√(1 + μ²) with

    f = ∫ p ds / |∫ p n ds|,

the total normal load over the magnitude of its resultant, for a pressure distribution p(φ)
symmetric about its centre. The nominal theory takes a Winkler sleeve (p ∝ cos φ over the lower
half, f = 4/π); its band lists a line contact (f = 1), p ∝ √cos φ (f = 1.371) and a uniform
pressure over the lower half (f = π/2). This module computes f for a continuum: a plane-strain
linear-elastic annulus r ≤ ρ ≤ r + t, bonded on ρ = r + t, against a rigid circle of radius r
whose centre has moved down by e (exact overlap r − |X − c| at each node, constraint linearised at
the node). By symmetry only the half x ≥ 0 is meshed (u_x = 0 on x = 0). The load is set by e: the
small load is 1e-3 of the battery's; the battery's is m g = 9.81 N per l = 0.2 m of length, with
the Young's modulus whose thin-layer (oedometric) stiffness is the battery's K = 1e6 N/m³:
E = K t (1 + ν)(1 − 2ν)/(1 − ν). With zero clearance the problem is homogeneous of degree one in e
up to the second-order term of the exact overlap, so the two loads give nearly the same f.

Assumptions: static, small strain, frictionless normal problem. For the spinning journal the
friction traction μ p is tangential to the bore; in plane strain it couples with the normal problem,
so the frictionless pressure is the μ → 0 limit (μ = 0.2 in the scenario), rotated by the friction
angle as the scenario's theory does.

Run: uv run tools/fem/bearing.py  (prints the table; tools/fem/run.py writes the committed results)
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import solver

RADIUS = 0.05  # shaft and bore radius, m
WALL = 0.02  # sleeve wall, m
LENGTH = 0.2  # bearing length, m
K_BATTERY = 1e6  # N/m³
LOAD_BATTERY = 1.0 * 9.81 / LENGTH  # N per metre of length
BAND = {"line_contact": 1.0, "winkler_cosine": 4 / np.pi, "sqrt_cosine": 1.3708, "uniform": np.pi / 2}


@dataclass
class BearingResult:
    nu: float
    load: float  # target resultant, N per metre of length
    level: int
    dofs: int
    ratio: float  # f = ∫ p ds / |∫ p n ds|
    stiffness_vs_winkler: float  # load / (K r e π/2): the Winkler sink over the continuum's
    contact_half_angle: float  # largest φ (from the bottom) with contact force, rad
    eccentricity: float
    phi: np.ndarray
    pressure: np.ndarray  # recovered p(φ), Pa


def winkler_modulus(nu: float, K: float = K_BATTERY, wall: float = WALL) -> float:
    """The Young's modulus whose thin bonded wall has the Winkler stiffness K (K = M/t)."""
    return K * wall * (1 + nu) * (1 - 2 * nu) / (1 - nu)


def mesh_counts(level: int) -> tuple[int, int]:
    """Elements through the wall and around the half bore."""
    return 4 * 2**level, 32 * 2**level


class Sleeve:
    """The meshed half sleeve of one level, solved for any eccentricity."""

    def __init__(self, nu: float, level: int, E: float, wall: float = WALL):
        n_rho, n_phi = mesh_counts(level)
        rhos = solver.graded(RADIUS, RADIUS + wall, n_rho, 1.5, "a")
        phis = np.linspace(0.0, np.pi, n_phi + 1)

        def to_xy(q):  # (ρ, φ from the bottom) ↦ (x, y), y up
            return np.vstack([q[0] * np.sin(q[1]), -q[0] * np.cos(q[1])])

        self.body = solver.elastic_body(solver.tensor_mesh(rhos, phis, to_xy), E, nu)
        x, y = self.body.nodes
        rho = np.hypot(x, y)
        tol = 1e-9 * RADIUS
        outer = rho > RADIUS + wall - tol
        self.fixed = np.concatenate([self.body.dofs[:, outer].ravel(), self.body.dofs[0, np.abs(x) < tol]])
        bore = np.abs(rho - RADIUS) < tol
        self.phi = np.arctan2(x[bore], -y[bore])
        order = np.argsort(self.phi)
        self.phi = self.phi[order]
        self.points = self.body.nodes[:, bore][:, order]
        self.contact = self.body.node_dofs(bore)[:, order]

    def solve(self, e: float):
        normals, required = solver.rigid_circle_constraints(self.points, np.array([0.0, -e]), RADIUS)
        sol = solver.solve_contact(self.body, self.fixed, self.contact, normals, required)
        resultant = 2 * abs(float(sol.force @ normals[1]))  # both halves, N per metre
        return sol, normals, resultant


def solve(nu: float, level: int, load: float = LOAD_BATTERY, K: float = K_BATTERY, wall: float = WALL) -> BearingResult:
    """f of the sleeve whose Young's modulus maps to the Winkler stiffness K, under `load` N/m."""
    E = winkler_modulus(nu, K, wall)
    sleeve = Sleeve(nu, level, E, wall)
    # Secant on e for the target resultant, from the Winkler sink e = load / (K r π/2).
    e = load / (K * RADIUS * np.pi / 2)
    sol, normals, resultant = sleeve.solve(e)
    for _ in range(20):
        if abs(resultant - load) < 1e-10 * load:
            break
        e *= load / resultant
        sol, normals, resultant = sleeve.solve(e)
    else:
        raise RuntimeError("eccentricity iteration did not converge")
    ratio = float(sol.force.sum() / abs(sol.force @ normals[1]))
    pressure = solver.boundary_pressure(RADIUS * sleeve.phi, sol.force)
    winkler_e = load / (K * RADIUS * np.pi / 2)
    return BearingResult(
        nu=nu,
        load=load,
        level=level,
        dofs=sleeve.body.K.shape[0],
        ratio=ratio,
        stiffness_vs_winkler=winkler_e / e,
        contact_half_angle=float(sleeve.phi[sol.active & (sol.force > 0)].max()),
        eccentricity=e,
        phi=sleeve.phi,
        pressure=pressure,
    )


if __name__ == "__main__":
    for nu in (0.3, 0.45):
        for load in (1e-3 * LOAD_BATTERY, LOAD_BATTERY):
            vals = []
            for level in range(4):
                res = solve(nu, level, load)
                vals.append(res.ratio)
                print(
                    f"nu={nu} load={load:.4g} level={level} dofs={res.dofs} f={res.ratio:.5f} "
                    f"phi_c={np.degrees(res.contact_half_angle):.1f} e={res.eccentricity:.3e} "
                    f"winkler/e={res.stiffness_vs_winkler:.3f}"
                )
            p, f_inf = solver.observed_order(vals)
            print(f"   observed order {p:.2f}, extrapolated f = {f_inf:.5f}")
