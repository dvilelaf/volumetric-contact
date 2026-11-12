# Audit of the contact core and engine coupling — volumetric-contact @ fe4684c (2026-11-11)

Read-only review of `core/`, `engines/` and the geometry/law tests, with every formula re-derived
before comparing it with the code. All evidence was produced under `/tmp/vc-research/diag/` with
small programs linked against the existing `build/core/libvolcontact_core.a`. The force-law variants
in that folder are copies of the code there (`dist_variant.cpp`), not changes to the repo:

| File | What it does |
|---|---|
| `main.cpp` → `bearing64.txt`, `cone64.txt`, `fine128.txt` | static breakdown of the wrench by surface category + a rigid-body simulation (semi-implicit Euler, dt = 1 ms, stiction on) with law variants |
| `loop.cpp`, `loop2.cpp` → `loop.txt`, `loop2.txt` | closed-cycle work of the elastic force (conservativeness) |
| `split.cpp` → `split.txt` | decomposition invariance: one slab against the same slab cut in two |
| `perf.cpp`, `perf2.cpp` → `perf*.txt` | BSP depth, cost of the distributed law, exits taken from ∂V |
| `run.sh` | `scripts/dev.sh` with `/tmp/vc-research` mounted (repo mounted read-only) |

The simulator in `main.cpp` reproduces the battery: bearing 0.1950 s (bench 0.194), cone 0.2820 s
(0.281), bearing fine 66.1 % (66.1 %), cone fine 54.3 % (54.4 %).

---

## 0. Headline: root cause of `bearing` and `cone`

**Root cause.** On the **seam faces** between convex pieces, the damping term of the pressure,
p = K·ℓ·(1 − h·v·n_face), produces a **viscous drag**. Friction plays no part in it.
(`core/src/distributed.cpp:111`, together with the use of each polygon's own face normal at
`distributed.cpp:184`.) For each strip piece B_i, ∂(A∩B_i) contains B_i's radial side faces (the
seams). Inside the overlap these faces are perpendicular to the sliding direction, so the normal
"approach velocity" seen on them is the full slip speed: v·n_face = ±ω·r = ±2 m/s in `bearing`
(±3–6 m/s in `cone`). With h = 5 s/m the factor (1 − h·v·n) is 1 + 10 on one face of every seam and
is clamped to 0 on the coincident face of the neighbouring piece. Their elastic parts cancel. Their
damping parts **add up to a tangential drag of ≈ 0.5·K·h·v·∫ℓ dA per seam** that opposes the spin.

**The owner's hypothesis is half right.** The seams are the culprit, but they carry **no
friction**: the slip is normal to the seam planes, so its projection on them is zero. Static
breakdown at the theoretical steady pose (`bearing64.txt`, e = 0.64 mm, φ = atan μ, ω = 40 rad/s):

| torque about the shaft axis [N·m] | h = 5 | h = 0 |
|---|---|---|
| total | −0.243 | −0.109 |
| of which friction (μ on − μ off) | −0.109 | −0.109 |
| of which non-friction (normal + damping) | **−0.134** | 0.000 |
| seam faces alone | **−0.129** | −0.00002 |

Theory τ = (4/π)μmgr/√(1+μ²) = 0.1225 N·m. Friction is right. The extra 0.13 N·m is seam damping,
which more than doubles the decelerating torque. The same mechanism also raises the climb angle:
bench 0.389 rad against theory 0.197, an apparent μ_eff = 0.41.

**Signature.** The drag grows with the number of seams, so refining the mesh makes the result
**worse**. The battery shows exactly that: bearing 52 % → 66 %, cone 27 % → 54 % from base to fine.
Gonthier is unaffected because it damps only along the pair normal at the centroid.

**Fix variants, simulated** (`bearing64.txt`, `cone64.txt`, `fine128.txt`; stop-time error):

| variant | bearing 64 | bearing 128 | cone 64 | cone 128 |
|---|---|---|---|---|
| current law | 51.8 % | 66.1 % | 26.8 % | 54.3 % |
| **seam faces skipped in the law** | **0.3 %** | **0.3 %** | 1.4 % | **0.1 %** |
| damping rate along the pair normal (seams kept) | 6.5 % | 6.5 % | 3.2 % | 3.5 % |
| damping along pair normal + seams skipped | 0.5 % | 1.0 % | **0.1 %** | 0.4 % |
| damping and force along pair normal (seams kept) | 3.9 % | 9.1 % | 0.7 % | 4.3 % |

With the seams skipped the bearing's climb angle is 0.204 rad (theory 0.197).

**Alternative hypotheses, refuted with evidence:**
- *Double counting of both surfaces.* A-side and B-side main faces each carry half: 4.503 N + 4.503 N
  vertical, ∫p dA = K·V. No double counting.
- *ℓ along a single pair normal on a wrap-around contact.* Each strip piece gives its own pair, so
  n is local (radial). The A and B main faces give exactly μ·∫p·r (−0.0548 − 0.0541 = −0.109 N·m at
  h = 0).
- *Wrong theory.* The friction-circle derivation re-checks: for p(θ) = p₀cosθ, the friction
  resultant is μ times the normal resultant rotated by 90°, ∫p dA = (4/π)N, and N√(1+μ²) = mg. With
  the seams skipped the simulation lands on it within 0.3 %. Gonthier (1.4 %) and Drake hydroelastic
  (0.4 %) agree too.

**Proposed general fix** (it touches the force law, so it needs the owner's approval, AGENTS.md):
1. **Tag the interior (seam) faces of multi-piece bodies** when the pieces are loaded. A face
   fragment of piece i is a seam where it lies on a face of another piece j of the same body with
   the opposite normal. Find these fragments with the existing BSP coplanar clipping (piece i's
   faces against piece j with an "opposite-coplanar" rule), so that partial overlaps such as CoACD
   output are handled. Store a per-triangle flag in `CollisionShape`, and carry it into
   `ContactSurface::Polygon` (`interior = true`).
2. In `distributed_force`, **skip interior polygons**. Keep them in `VolumeIntegrator`: V_i must
   stay closed for the divergence theorem. Excluding them from the pair-normal vector area is
   optional; it slightly improves n.
3. Optionally, and separately justified (finding S2), **measure the damping rate along the column
   direction n** instead of each face's normal.
4. Regression tests (one per criterion):
   - (a) *decomposition invariance*: a box sliding or spinning on one slab and on the same slab cut
     in two gets the same wrench with h > 0;
   - (b) a cylinder spinning in a multi-piece bearing with h = 5: the tangential resultant equals μ
     times the normal resultant;
   - (c) closed-cycle work ≈ 0 on the bowl pieces.

---

## 1. BUG

### B1. Seam faces of multi-piece bodies are integrated as contact surface (distributed law)
`core/src/intersection.cpp:125-138` records every face of B_i inside A, seams included.
`core/src/distributed.cpp:183-187` integrates all of them. This is the root cause in §0, with three
consequences that all disappear when seams are skipped:
- **Viscous drag ∝ number of seams** (§0).
- **Non-conservative elastic field.** Sphere in the bowl pieces (704 pieces, h = 0, μ = 0), loop in
  (x, z): ∮Q·dq = −3.0e-4 J against Σ|Q·dq| = 0.10 J, i.e. 0.3 % per cycle. With the seams skipped
  it is 2.9e-6 J (2.8e-5, the quadrature error). Box on the slab, one piece: 3e-5. Gonthier: < 1e-5.
  (`loop.txt`, `loop2.txt`). The coincident seam faces of neighbouring pieces measure ℓ along two
  different pair normals, so their elastic forces cancel only approximately, and the residual
  depends on the path.
- **The result depends on the decomposition.** Box 0.4 m, δ = 1 mm, sliding at 1 m/s, h = 5: one
  slab gives Fx = −80.7 N, the slab cut at x = 0.05 gives −81.3 N, and cut with seams skipped gives
  −80.7 N. Spinning at 10 rad/s: Tz = −12.611 / −12.713 / −12.603 N·m (`split.txt`).

Fix: §0. The volume, centroid and inertia already handle seams correctly.

### B2. Pressure is clamped at the vertices and then interpolated linearly (distributed law)
`core/src/distributed.cpp:109-120`: `pressure()` returns max(p, 0) at each vertex, and the
triangle integrals assume p is linear between them. Where the product K·ℓ·(1 − h·vn) changes sign
inside a triangle, the integral depends on how the polygon was triangulated: the fan from vertex 0
and the partition by exits.

Evidence:
- A **centred box spinning in place** on one slab (pure spin, h = 5) gets a net **lateral force
  Fx = −0.54 N** (0.34 % of Fz). By symmetry it should be 0 (`split.txt`). Opposite side faces have
  mirrored velocity profiles but differently oriented fans.
- With h = 5 the law's result changes by 2.7e-4 (relative) when only the exit set changes, with
  the same geometry (`perf2.txt`, sphere on slab). At h = 0 the results are identical.

Fix (general, exact):
- The two factors ℓ(x) and (1 − h·vn(x)) are affine on each piece. Clip each triangle along the
  line where the damping factor is zero (as `Sampler::clip` already does for exits).
- Integrate the product of the two affine functions exactly:
  ∫fg dA = A/12·(Σf_i g_i + Σf_i·Σg_i).
- For friction, either keep the linear approximation of the clipped triangle or subdivide.

### B3. Gonthier bristle drops the stick force at exactly zero slip speed
`core/src/gonthier.cpp:22-25`: `if (speed == 0.0) { store_anchor(); return 0; }`. A loaded bristle
(e.g. a block held on an incline) loses its whole spring force for any step in which the slip
velocity is exactly zero. Explicit integrators rarely produce an exact zero, but symmetric set-ups,
the first step from rest and implicit solvers that converge to exact zeros can. The result is a
one-step force jump. Only `f_slide` needs a direction; compute `elastic`/`viscous` with s = 1
whenever speed = 0. The distributed law does not have this problem.

### B4. Gonthier rolling-resistance torque survives the normal-force clamp
`core/src/gonthier.cpp:90` clamps fn ≥ 0, but `gonthier.cpp:104` still applies
−K·h·Ig·ω_t. A contact that separates faster than 1/h therefore gets a damping torque with no normal
load (a "sticky" torque). Apply the rolling term only when K·V·(1 − h·vn) > 0, or scale it with the
same clamp. (The spin term already scales with the clamped fn.)

---

## 2. SCIENTIFIC FLAW

### S1. The native contact is judged against Winkler theories, and Chrono's mapping is not equivalent
`docs/scenarios.md` §3–§4.
- Several theories are specific to the volumetric constitutive law:
  - `tip` (Winkler stability);
  - `drop` (m δ̈ = mg − KAδ(1 + hδ̇));
  - `hole` (z = 2mg/(4μKδa));
  - `corner`, `stack` (penetrations mg/(K a²));
  - `offset` (tilt from K·I_A);
  - `bearing` (the 4/π of the cosine Winkler pressure).
- **Drake hydroelastic** is Winkler-equivalent near the surface: a compliant body with E = K·H
  gives p = E·φ/H = K·φ, the half-space with E = K·1 m gives K·depth, the ground is compliant and
  the body rigid, and Hunt–Crossley uses d = h. The mapping is fair, which is why Drake can match
  these theories.
- **Chrono SMC Hertz** gives F ∝ δ^{3/2} at a few manifold points per convex-hull pair. Check the
  effective curvature radius Chrono uses for non-spherical shapes
  (`ChCollisionInfo::GetDefaultEffectiveCurvatureRadius`). "E = K·1 m" therefore has no
  equivalence to K. Its errors in the Winkler-specific scenarios mostly measure a different
  constitutive law, not the quality of a contact method.

Consequence: the README conclusion "on Chrono both volumetric laws are significantly more accurate
than its SMC contact" (`README.md:106,116`) is partly built into the set-up.

Recommendation:
- Split the battery into constitutive-independent scenarios (rigid-body Coulomb kinematics:
  `block`, `cylinder`, `push`, `billiard`, `sphere`, `vgroove`, `twoblocks`, partly `bowl` and
  `trough`) and Winkler-specific ones.
- Report the native methods against the first group, or derive Hertz theories for the second.
- Map Chrono's Hooke model with k_n from K·A_contact only where that is defined.

### S2. Hybrid Winkler/pressure kinematics in the distributed law
ℓ is the column length **along n** (Winkler springs along n), but the elastic force acts **along
each face's own normal** over the true area, and the damping rate uses **v·n_face**
(`distributed.cpp:111, 184, 201`). Derivation, for a single-valued column field with fixed n and
the A and B faces each carrying half:
- the **translational** force equals −∇ of the Winkler potential U = (K/2)∫ℓ² dA⊥ exactly (the
  tangential parts agree because ∫ℓ∇ℓ dA⊥ = 0 when ℓ = 0 on the contour);
- torques agree up to O(ℓ²) lever-arm terms. So the elastic part is consistent to first order, as
  confirmed by the box loop (3e-5).

Two inconsistencies remain:
- **Damping along the face normal** is not the rate of compression of a column. On faces whose
  normal is not n, tangential motion produces "damping". Side faces of a single-piece overlap
  give: box sliding at 1 m/s with δ = 1 mm, +0.6 N of drag (0.75 % of μN, `split.txt`); the spinning
  box's spurious Fx is B2 acting together with this. On seams the effect is B1. Fix: vn = v_rel·n,
  i.e. dℓ/dt for rigid motion. Same cost, and correct in the Winkler sense.
- **Side faces carry pressure but no normal load.** On faces nearly parallel to n, ℓ is the rest of
  the column above the point, so they add μ·∫p dA of friction without any n-component. The
  friction/normal ratio becomes μ·(1 + O(δ/w)): 80.1 N instead of 80.0 in the box test, already
  acknowledged in `tests/test_distributed.cpp:109`. On inclined faces the traction per projected
  area is 1/cos β too large for friction.

Options: keep the pressure-like traction (closer to an elastic half-space) and document the
O(δ/w) bias, or use the Winkler-consistent form, i.e. force K·ℓ·dA⊥ along n and friction
μ·K·ℓ·dA⊥, which is exactly conservative for fixed n. Do not adopt "force along n" without
skipping seams: it turns the seam pressure into normal load (+27 % in the bowl loop,
`loop2.txt`).

### S3. Gonthier's spinning law is viscous and dimensionally inconsistent
`core/src/gonthier.cpp:103`: τ = −μ_spin·(Fn/V)·Ig·ω_n has units N·m·(m/s). μ_spin silently
carries s/m, so the law is a viscous damper with exponential decay, not Coulomb spin friction
(τ = const·μFn). This is faithful to the thesis but explains Gonthier's `disk` (34 %) and
`contensou` (13 %) errors structurally. A regularized Coulomb form, τ = −μ·Fn·ρ_c·sat(ω_n·ρ_c/v_s),
with ρ_c a volume-derived radius (e.g. (∫|r|dV⊥)/V, computable in closed form from the same
clipped polygons), would be dimensionally consistent without tuning per scenario. Since the
distributed law already solves spin exactly, this is only worth doing if Gonthier must stay a
general law.

### S4. The pair normal can vanish, and the force with it
`core/src/intersection.cpp:146-148` and `contact_model.cpp:104-106`. n = normalized vector area of
B's part of ∂V. It is **zero** in these cases:
- (a) A is entirely inside a piece of B, so ∂V = ∂A and the B part is empty;
- (b) A pierces B through, e.g. a pin through a plate: B's top and bottom parts cancel;
- (c) the overlap is symmetric so that the B part closes on itself.

The pair is then dropped (`!normal.isZero()`), so the **force vanishes at the deepest
penetrations** and a fast or heavy body can tunnel. Near these configurations n is ill-conditioned
and flips with tiny perturbations: its norm → 0 while V > 0. Corner and edge contacts are well
defined (a weighted average of the face normals), but at a convex edge of B the normal of a sliver
overlap tilts strongly. Example: at the `hole` entry the wall's top face (0.1 mm wide) dominates
while the box is less than ~0.1 mm in, so n ≈ +z (45° at 0.1 mm), and the entering box is pushed up
instead of sideways.

Fix (general):
- keep the last valid normal in the per-pair state (8 more doubles) and fall back to it when
  |S_B| < ε·V^{2/3};
- for a first contact with no valid normal, use the direction from B's centroid to A's centroid,
  or the minimum-translation direction from the face planes of the convex pair.

---

## 3. RISK

**R1. Polygonal "ploughing" of rotating faceted bodies.** On a facet of width w rotating at ω,
material points move along the facet normal at ±ω·s, |s| ≤ w/2. With h·ω·w/2 ≥ 1 the damping factor
clamps on the trailing half, which adds normal load and a rolling-resistance-like drag, even
without seams.
- Cone at ω = 60 rad/s (h·ω·w/2 ≈ 1.1): the static torque of the main faces is −0.29 N·m at h = 5
  against −0.248 at h = 0 (`cone64.txt`).
- In the dynamic simulation with seams skipped it leaves ≤ 1.4 %.
- The drag converges as O(w²) = O(1/N²); the clamp-induced extra load converges as O(w). B2's fix
  makes it exact for the discretised geometry; damping along n (S2) also removes most of it.

**R2. Stiction energy.**
- *Gonthier* (`gonthier.cpp:31-57`):
  - Resetting the anchor never adds energy: after the reset the spring sits exactly at the limit.
  - The damper −c(u·dv)u acts along the 3-D bristle direction u. When the bristle has a normal
    component (the centroid moves along n as the overlap changes shape), u_t·dv ≠ u·dv, so its
    tangential projection is not strictly dissipative. Project d onto the tangent plane before
    computing u.
- *Both laws, total-force cap* (`gonthier.cpp:55`, `distributed.cpp:299,316-319`): when the cap
  scales spring + damper by κ < 1 **while the spring is being loaded**, the spring stores k·d·ḋ but
  the body pays only κ of it. Energy injection per load cycle ≤ μ_s|Fn|·d (d = 0.1 mm).
- *Load-proportional stiffness* k = μ_s|Fn|/d is parametric: oscillating Fn together with tangential
  vibration can pump energy, bounded by the same μ_s|Fn|·d per cycle.

Both effects are tiny but unbounded in count under sustained vibration. A test that runs a
vibrating stack for a long time would catch them; the cap could also be applied to the spring
increment instead of the force.

Positives:
- The distributed stick is objective (deflection stored in B's frame).
- It is independent of dt (it accumulates the actual material-point slip since the last
  evaluation).
- It is insensitive to movements of the pressure centre (it accumulates slip at whichever point is
  the current centre).
- Rolling without slip does not load it.

**R3. Friction state per piece pair.** `ContactModel` keeps one `FrictionState` per piece pair.
When the patch migrates across pieces (sphere in the 704-piece bowl, a body creeping over a split
ground), the new pairs start with zero deflection and the old ones drop theirs: for one step the
static hold is lost per piece transition. Each pair also gets its own torsional spring with the
radius of gyration of its own sub-patch. Proposal (general): lump the stick at the **body-pair**
level. Gather the triangles of all piece pairs of a (body A, body B) group, compute one pressure
centre, ρ, translational and torsional springs, and store one state per body pair.

**R4. Columns cut by seams.** For an A face in V_i, ℓ ends at B_i's seam plane when the column
crosses into the neighbouring piece; that column segment's continuation in V_{i+1} starts on a
seam face, which has ℓ = 0 on its exit side. The normal force is therefore undercounted wherever
seams are not parallel to n. This is negligible for the battery's radial and strip seams (cut zone
≈ ℓ·sin(π/64)), but it can be large for CoACD pieces. With seam tagging, the seam planes could be
excluded from the exit set (the column continues through the seam), at the cost of using B_i's
plane extension beyond the piece. Document it as a known limitation until then.

**R5. Convexity and mesh validation.** `intersection.cpp:72` accepts non-planarity of
1e-5 × diagonal (0.4 mm for the 40 m slab), so a large, mildly non-convex ground would pass as
convex and silently break the distributed law. There is no check of closedness, manifoldness or
consistent orientation: `outward()` only flips by the sign of the total volume. Add an edge-pairing
check and an absolute tolerance.

**R6. `ContactModel::evaluate` is `const` but lazily builds its groups** (`contact_model.hpp:96-99`).
It is not thread-safe before the first call. Build the groups in `add_pair`, or in an explicit
`finalize()`.

**R7. Drake cache-reuse heuristic** (`volumetric_contact_system.cpp:49-64`). It is correct as
written: the friction state is served from the pre-step evaluation whatever order Drake applies
updates in. It relies on bit equality of the inputs and on the current event ordering. A Drake
upgrade that changes that order would give a silent one-step lag in the friction state only. Keep
a test asserting that step k's forces use state k.

**R8. BSP tolerance.** The plane thickness is 1e-10 × the diagonal, and the pair uses the larger
of the two shapes' values (4e-9 m whenever the 40 m slab is involved). That is fine at the
battery's µm penetrations. Fragments smaller than the tolerance are routed by the coplanar rule
regardless of their true side, giving O(tol²) errors only.

**Verified correct (no action):**
- Eberly volume integrals (`volume_integrator.hpp`), including the inertia signs and products.
- Integration about the region centre.
- Coplanar rules (kInsideBehind for A, kInsideBothSides with `mixed` for B) for convex and
  non-convex meshes.
- ∂V is watertight up to round-off.
- n = (S_B − S_A)/|…| = S_B/|S_B| for a closed ∂V; for a planar contour this is the contour plane,
  because the vector area depends only on the boundary curve.
- `polar_moments` antiderivatives: ∫sec = asinh t, ∫sec·tan = sec, ∫sec³ = (sec·t + asinh t)/2,
  ∫sec³·tan = sec³/3. The four moments and the orientation sign about m check out, and the test
  already compares them with brute force, q outside the triangle included.
- Instantaneous centre q = x₀ + m × v_t0/ω_m.
- Kinetic friction force and torque in closed form for linear p.
- Exact partition of ℓ by the min of affine exits (exact for convex V).
- The K·V identity holds when the +n entry surface is all A and the −n entry surface all B, which
  is true for convex V of "simple" contacts. It fails slightly when B faces belong to the +n entry
  surface: sharp-wedge edges of B, where those B faces get ℓ = 0.
- Gonthier: Fn = K·V·(1 − h·n·v_c) is exact volumetric damping, since ∫v_n dV = V·v_n(c) for an
  affine v_n. The rolling term K·h·Ig·ω_t equals the exact K·h·[n]×M[n]×ᵀω up to the thickness
  term M_nn (negligible for thin overlaps). Sign conventions: n pushes A out of B; approaching
  means v·n < 0, which raises Fn.
- `ContactModel` broadphase is conservative (ball containing every A shape against B boxes and the
  face planes of convex B), so no pairs are missed. The result is deterministic under OpenMP: one
  output slot per pair, accumulation in sorted order.

---

## 4. Engine coupling

- **Chrono** (`chrono_engine.cpp:178-204`): wrench from the start-of-step state, applied at the COM
  (`GetPos`), with world torques; `UpdateForces` is called, so there is **no extra lag**. Forces
  are explicit and kinematics semi-implicit (EULER_IMPLICIT_LINEARIZED with constant external
  forces). The friction state advances once per step.
- **Drake** (`drake_engine.cpp:197-225`, `volumetric_contact_system.cpp`): a discrete plant with no
  SceneGraph for the volumetric laws. Forces come in as `ExternallyAppliedSpatialForce` at Bo = COM
  (`p_BoBq_B = 0`) and are explicit in the plant's step. The friction state is discrete state with
  the plant's period, updated from the same pre-step evaluation. **No lag; correct** (see R7).
- Both engines are explicit in the contact force. The stability limit is ω_n·dt < 2 with
  k_n = K·A (`hole`: K = 1e8 on a 0.01 m² wall gives ω_n ≈ 1000 rad/s, ω_n·dt ≈ 1 at 1 ms: close).
  Open problem 4 (Jacobians for Chrono's implicit integrator) is the right next step. The core
  could expose ∂F/∂(q, v) cheaply for Gonthier (K·S_A for translation; K·h·V for damping).
- Native mapping: see S1. Drake: fair (hydroelastic = Winkler near the surface; rigid–compliant
  only, so no K/2 series stiffness; Hunt–Crossley d = h; μ solved per geometry). Chrono: not
  equivalent to K.

---

## 5. Performance (IMPROVEMENT, ordered by expected gain)

1. **Take the column exits from the faces of ∂V, not from all face planes of both shapes**
   (`distributed.cpp:68-89`). For convex V every column exits through a face of ∂V, so the
   minimum is the same. Measured (`perf2.txt`): sphere on slab **0.38 → 0.010 ms per evaluation
   (37×)**, sphere in the bowl pieces **61 → 5.5 ms (11×)**, with **identical forces at h = 0**
   (with h > 0 they differ only by B2's partition dependence). This also shrinks the O(k²)
   exit-pair clipping in `Sampler::polygon`. Better still, partition each entry polygon by
   projecting the exit polygons of ∂V along the column: O(k) clips instead of O(k²).
2. **Balanced spatial structure for convex shapes.** The solid BSP of a convex mesh degenerates
   into a chain: every candidate splitter has 0 splits and front = 0. For the 64-segment sphere it
   is **2048 nodes deep** (`perf.txt`), and every B triangle clipped against it walks the whole
   chain. Use a kd-tree whose cells hold their few relevant face planes, or a Gauss-map bucketing
   of the face normals. Keep the BSP for non-convex shapes.
3. **Cache `pressure_integrals`.** It is computed three times per triangle
   (`distributed.cpp:199, 249, 264`).
4. **Temporal coherence.** Keep per pair the last separating plane (convex pairs) for O(1)
   rejection, and the last list of overlapping A triangles as the BVH query seed.
5. **Icospheres instead of UV spheres.** Sphere contacts near a pole touch a fan of 64 slivers, so
   the contact patch and the cost depend on orientation; geodesic meshes are isotropic and need
   fewer planes for the same accuracy. This is a `meshgen` change and makes the asset fairer too.
6. **The 704-piece bowl.** After (1)–(2) it is dominated by per-pair overhead. Merging the pieces'
   surfaces per body pair for the law (R3) also removes per-pair stick bookkeeping.
7. **Allocation and parallelism are fine.** Workspaces are reused, OpenMP has per-thread scratch,
   and the half-hardware thread cap is reasonable.

---

## 6. API and modularity (for a reusable library)

Minimal changes, no over-engineering:
- **Install and export.** `install(TARGETS volcontact_core EXPORT volcontactTargets)`,
  `install(DIRECTORY core/include/)`, a `volcontactConfig.cmake` with `find_dependency(Eigen3)`
  (and OpenMP when it is enabled), and `write_basic_package_version_file`.
  `VOLCONTACT_NATIVE` should default to **OFF** for installable builds: a static library built with
  `-march=native` is not portable.
- **Namespace and headers.** `vc` is too generic for a library; use `volcontact`, with `vc` as an
  optional alias. Move `bsp`, `bvh`, `volume_integrator`, `polar_moments` and the workspaces to
  `volcontact/detail/`, and keep `types`, `CollisionShape`, `ContactModel`, the law parameters and
  `FrictionState` public.
- **Geometry input.** `CollisionShape` from vertex and index arrays (not only OBJ), plus a
  **`Body`/`CompoundShape` = convex pieces + seam tagging** type. The B1 fix needs it, and it is
  the natural place for decomposition and validation (R5).
- **Thread safety.** `ContactModel::finalize()` (R6). Make `FrictionState` an opaque per-pair blob
  whose size depends on the law (Gonthier needs 4 doubles, distributed 20).
- **C API** (`volcontact_c.h`): opaque handles, poses as 12 doubles and twists as 6, an evaluate
  call that writes wrenches and the next friction blobs, error codes instead of exceptions.
  **Python** via nanobind over the C++ API (numpy in and out); enough for MuJoCo, PyBullet or
  Isaac prototyping.
- **Engine adapters.** Document the pattern (evaluate at the step start, apply the wrench, commit
  the state once per step) and the Drake cache trick as reusable adapters. Later, add an optional
  Jacobian output for implicit integrators.

---

## 7. Tests to add (one per acceptance criterion)

1. Decomposition invariance with h > 0, sliding and spinning: one slab against the same slab cut
   in two (fails today: −80.7 vs −81.3 N).
2. A centred box spinning in place gets zero net force (fails today: Fx = −0.54 N, B2).
3. A shaft spinning in a multi-piece bearing with h = 5: |F_t| = μ·|F_n| ± 2 % (fails today:
   ≈ 2×).
4. Closed-cycle work of the elastic force on the bowl pieces below 1e-4 of Σ|Q·dq| (fails today:
   3e-3).
5. Gonthier with a loaded bristle at exactly zero slip velocity keeps its force (B3).
6. A deep, enclosed overlap still gives a force (S4).
