"""Small-strain linear elasticity with frictionless unilateral contact against a rigid body.

The finite-element reference for the class B scenarios (`docs/scenarios.md` §4) needs the contact
pressure of a continuum elastic support, which the contact laws under test do not provide. This
module solves the static problem

    K u = Cᵀ λ,   n_i · u_i ≥ d_i,   λ_i ≥ 0,   λ_i (n_i · u_i − d_i) = 0,

for a two-dimensional body (plane strain, or axisymmetric in (r, z)) meshed with 9-node quadratic
quadrilaterals (scikit-fem `ElementQuad2`, 3×3 Gauss points, no volumetric locking at the ν = 0.45
used here). Each contact node i may be pushed by the rigid body along the unit vector n_i (the
obstacle's normal) and must move at least d_i along it; λ_i is the nodal contact force. The
constraints are linearised at the reference configuration (small strain, small displacement), and
the problem is solved exactly by an active set: each iteration solves the saddle-point
system with the active constraints as equalities, then activates the nodes that would penetrate and
releases the ones in tension, until the set stops changing. Homogeneous Dirichlet conditions
(bonded or symmetry boundaries) are eliminated.

Everything is deterministic: tensor-product meshes with fixed grading, a direct sparse solver.
"""

from __future__ import annotations

import dataclasses
from dataclasses import dataclass

import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla
from skfem import (
    Basis,
    BilinearForm,
    ElementQuad2,
    ElementVector,
    MeshQuad,
    MeshQuad2,
    asm,
)


def lame(E: float, nu: float) -> tuple[float, float]:
    """Lamé constants (λ, μ) of an isotropic material."""
    return E * nu / ((1 + nu) * (1 - 2 * nu)), E / (2 * (1 + nu))


def constrained_modulus(E: float, nu: float) -> float:
    """Oedometric modulus M = E(1 − ν)/((1 + ν)(1 − 2ν)): a thin bonded layer of thickness t
    pressed uniformly is a Winkler foundation of stiffness K = M / t."""
    return E * (1 - nu) / ((1 + nu) * (1 - 2 * nu))


def graded(a: float, b: float, n: int, power: float = 1.0, toward: str = "b") -> np.ndarray:
    """n + 1 points from a to b, clustered toward one end by the power map s ↦ s^power.

    Refining by doubling n keeps the same map, so the meshes of a refinement study are nested in
    the parameter space and their element sizes halve everywhere."""
    s = np.linspace(0.0, 1.0, n + 1)
    if toward == "a":
        return a + (b - a) * s**power
    return b - (b - a) * (1.0 - s) ** power


def tensor_mesh(xs: np.ndarray, ys: np.ndarray, transform=None) -> MeshQuad2:
    """A quadratic quadrilateral mesh on the tensor grid xs × ys. `transform` maps the (x, y)
    parameters of every node, midside nodes included, so a mapped mesh has curved edges."""
    mesh = MeshQuad2.from_mesh(MeshQuad.init_tensor(np.asarray(xs), np.asarray(ys)))
    if transform is None:
        return mesh
    return dataclasses.replace(mesh, doflocs=np.asarray(transform(mesh.doflocs)))


@dataclass
class Body:
    """A meshed elastic body: the basis, its stiffness matrix and the positions of its nodes."""

    basis: Basis
    K: sp.csr_matrix
    nodes: np.ndarray  # (2, number of nodes): positions
    dofs: np.ndarray  # (2, number of nodes): the x and y (or r and z) dof of each node

    def node_dofs(self, mask: np.ndarray) -> np.ndarray:
        """The (2, k) dofs of the nodes selected by a boolean mask over the nodes."""
        return self.dofs[:, mask]


def elastic_body(mesh: MeshQuad2, E: float, nu: float, axisymmetric: bool = False) -> Body:
    """Assemble the stiffness of a plane-strain body (per unit length) or of an axisymmetric body
    (x = r, y = z; the whole ring, so nodal forces are ring forces)."""
    lam, mu = lame(E, nu)
    basis = Basis(mesh, ElementVector(ElementQuad2()), intorder=4)

    @BilinearForm
    def stiffness(u, v, w):
        def strain(f):
            hoop = f[0] / w.x[0] if axisymmetric else 0.0 * f[0]
            return f.grad[0, 0], f.grad[1, 1], hoop, f.grad[0, 1] + f.grad[1, 0]

        eu, ev = strain(u), strain(v)
        tu, tv = eu[0] + eu[1] + eu[2], ev[0] + ev[1] + ev[2]
        energy = lam * tu * tv + 2 * mu * (eu[0] * ev[0] + eu[1] * ev[1] + eu[2] * ev[2]) + mu * eu[3] * ev[3]
        return energy * (2 * np.pi * w.x[0] if axisymmetric else 1.0)

    K = asm(stiffness, basis).tocsr()
    ix, iy = basis.split_indices()
    nodes = basis.doflocs[:, ix]
    if not np.allclose(nodes, basis.doflocs[:, iy]):
        raise RuntimeError("the x and y dofs of the vector basis are not co-located")
    return Body(basis=basis, K=K, nodes=nodes, dofs=np.vstack([ix, iy]))


@dataclass
class ContactSolution:
    u: np.ndarray  # all dofs
    force: np.ndarray  # λ_i ≥ 0 per contact node, along its normal
    active: np.ndarray  # boolean per contact node
    iterations: int


def solve_contact(
    body: Body,
    fixed: np.ndarray,
    contact_dofs: np.ndarray,
    normals: np.ndarray,
    required: np.ndarray,
    max_iterations: int = 100,
) -> ContactSolution:
    """Frictionless unilateral contact by an active set.

    fixed: dofs held at zero. contact_dofs: (2, m) dofs of the contact nodes. normals: (2, m)
    unit vectors along which the rigid body pushes each node. required: (m,) the displacement each
    node must have at least along its normal (positive where the rigid body overlaps the
    undeformed body, negative where there is a gap).
    """
    n = body.K.shape[0]
    m = contact_dofs.shape[1]
    free = np.setdiff1d(np.arange(n), np.unique(fixed))
    index = -np.ones(n, dtype=int)
    index[free] = np.arange(free.size)
    Kff = body.K[free][:, free].tocsc()
    # Row i of C picks n_i · u_i; the columns of fixed dofs are dropped (their value is zero).
    rows, cols, vals = [], [], []
    for c in range(2):
        keep = index[contact_dofs[c]] >= 0
        rows.append(np.arange(m)[keep])
        cols.append(index[contact_dofs[c]][keep])
        vals.append(normals[c][keep])
    C = sp.csr_matrix((np.concatenate(vals), (np.concatenate(rows), np.concatenate(cols))), shape=(m, free.size))
    # A node with (nearly) zero force at zero gap is admissible either way: the axis node of a
    # quadratic axisymmetric face (∫ N₀ r dr = 0, so its consistent force sits at round-off and can
    # be slightly negative when the pressure grows outward), or a node at the edge of the contact.
    # The tolerances give the update a hysteresis that keeps such a node from flipping forever.
    gap_tol = 1e-9 * float(np.max(np.abs(required)))

    active = required > 0
    for iteration in range(1, max_iterations + 1):
        A = np.flatnonzero(active)
        CA = C[A]
        kkt = sp.bmat([[Kff, CA.T], [CA, None]], format="csc")
        rhs = np.concatenate([np.zeros(free.size), required[A]])
        x = spla.spsolve(kkt, rhs)
        uf = x[: free.size]
        force = np.zeros(m)
        force[A] = -x[free.size :]
        gap = C @ uf - required  # ≥ 0 where admissible
        force_tol = 1e-6 * float(np.max(np.abs(force)))
        # Release an active node in tension; activate an inactive node that penetrates.
        updated = np.where(active, force > -force_tol, gap < -gap_tol)
        if np.array_equal(updated, active):
            u = np.zeros(n)
            u[free] = uf
            return ContactSolution(u=u, force=np.maximum(force, 0.0), active=active, iterations=iteration)
        active = updated
    raise RuntimeError(f"active set did not converge in {max_iterations} iterations")


def rigid_circle_constraints(points: np.ndarray, centre: np.ndarray, radius: float):
    """Normals and required normal displacements of nodes against a rigid circle (a cylinder
    pressed onto a block, or a shaft inside a sleeve: the condition is the same).

    A node must stay at a distance ≥ radius from the centre; the constraint |X + u − c| ≥ radius is
    linearised at the node's reference position X, with the exact (not linearised) overlap."""
    offset = points - centre[:, None]
    dist = np.linalg.norm(offset, axis=0)
    return offset / dist, radius - dist


def boundary_pressure(s: np.ndarray, forces: np.ndarray, weight=None) -> np.ndarray:
    """Recover nodal pressures from consistent nodal forces along a line of quadratic edges.

    s: increasing nodal coordinates along the line (corner, midside, corner, ...: an odd number of
    points), forces: the nodal forces, weight(s): the measure (2πr for an axisymmetric face, 1 for
    plane strain). Solves M p = F with the quadratic boundary mass matrix."""
    if s.size % 2 == 0:
        raise ValueError("a line of quadratic edges has an odd number of nodes")
    gauss, gw = np.polynomial.legendre.leggauss(5)
    M = np.zeros((s.size, s.size))
    for e in range(0, s.size - 2, 2):
        a, b = s[e], s[e + 2]
        xi = gauss
        shapes = np.array([xi * (xi - 1) / 2, 1 - xi**2, xi * (xi + 1) / 2])
        x = (a + b) / 2 + (b - a) / 2 * xi
        wq = gw * (b - a) / 2 * (weight(x) if weight is not None else 1.0)
        M[e : e + 3, e : e + 3] += (shapes * wq) @ shapes.T
    return np.linalg.solve(M, forces)


def observed_order(values: list[float]) -> tuple[float, float]:
    """Observed order of convergence and Richardson extrapolation from three successive
    refinements with ratio 2: p = log2((f1 − f2)/(f2 − f3)), f∞ = f3 + (f3 − f2)/(2^p − 1)."""
    f1, f2, f3 = values[-3:]
    d1, d2 = f1 - f2, f2 - f3
    if d2 == 0 or d1 / d2 <= 0:
        return float("nan"), f3
    p = float(np.log2(d1 / d2))
    return p, f3 + (f3 - f2) / (2**p - 1)
