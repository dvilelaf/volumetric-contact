# Baselines of the literature inside the core

Two things the comparison needs and the two laws alone do not give
(`docs/research/literature.md` §3 and §5):

1. an **ablation** that isolates the one element of `distributed` that is not in the literature,
   the closed-form Coulomb friction wrench of a linear pressure on a triangle under a rigid slip
   field, against the two quadratures the literature uses;
2. the **closest published competitor law**, the per-face elastic foundation (Hippmann's PCM,
   Simbody's `ElasticFoundationForce`), on the same geometry and with the same K, h and μ.

Both live in `core/` and run on every engine like the other laws. Bench labels:
`distributed-vertex`, `distributed-centroid`, `pcm`. Tests: `tests/test_baselines.cpp`.

## 1. Friction quadratures of `distributed`

`DistributedParams::friction_quadrature` (`core/include/volcontact/distributed.hpp`). The law
turns the contact surface into triangles that carry a linear pressure p (vertex values pᵢ) and
slip with the rigid relative velocity field v of the pair; v̂ is the direction of its part in the
triangle's plane. The switch changes only how the kinetic friction −μ·p·v̂ is integrated over
each triangle. The geometry, the pressure and its clamp, the normal wrench and the stick (with
its static capacity, which stays in closed form) are the same code.

| Value | Label | Friction of one triangle (area A, vertices xᵢ) | Source |
|---|---|---|---|
| `kClosedForm` (default) | `distributed` | ∫μ·p·v̂ dA and its moment, exactly: in the plane the slip is a rotation about an instantaneous centre q, and the integrals of r/\|r\|, r·rᵀ/\|r\| and \|r\| about q are elementary (`polar_moments.hpp`) | this repository |
| `kVertex` | `distributed-vertex` | fᵢ = −μ·pᵢ·v̂ᵢ at the three vertices (zero at a vertex that does not slip), interpolated linearly: force A/3·Σfᵢ, moment A/12·(Σxᵢ × Σfⱼ + Σxᵢ × fᵢ) = A·(¾·x̄ × f̄ + 1/12·Σxᵢ × fᵢ) | Hasegawa and Sato 2004, eq. 13 (the friction of a point) and eqs. 14–16 (the interpolation, "analytical integration of this equation is difficult") |
| `kCentroid` | `distributed-centroid` | −μ·(∫p dA)·v̂ at the pressure centroid ∫p·x dA / ∫p dA, with the slip direction there | what PCM (Hippmann 2024, "a regularised friction force law is applied … at the contact elements"), Simbody's elastic foundation and the discrete hydroelastic model do per element |

Departures and notes:

- Hasegawa and Sato's equations were taken as transcribed in `docs/research/literature.md` §2.1
  and §5; the paper could not be reached again for this work. The moment formula is the exact
  integral of a linear field (∫λᵢ dA = A/3, ∫λᵢ·λⱼ dA = A/12·(1 + δᵢⱼ)), which is what "interpolate"
  means; if their eq. 16 lumps it differently the difference is of higher order in the triangle
  size.
- Hasegawa and Sato's triangles are those of the intersection polyhedron projected on one contact
  plane, with pressure along the contact normal; here the triangles, the pressure and the plane of
  the slip are those of `distributed` (each face's own plane). Only their friction quadrature is
  reproduced, not their model.
- The vertex pressures are those of the linear pressure with the same force and moment as the
  exact one; a vertex value can be negative (it is for the closed form too).
- With a uniform slip direction (no relative spin about the face normal) the three coincide to
  round-off: μ·∫p dA against the slip, at the pressure centroid.

What the tests show (`tests/test_baselines.cpp`, numbers in their comments):

- A faceted disk spinning flat on a ground meshed alike, torque against μ·N·(mean radius of the
  n-gon), n segments by n/8 rings: closed form 3e-5 at every n (the friction of the thin side
  strips of the overlap, which is the model's); vertex −4.8 %, −0.76 %, −0.13 % and one point
  −0.45 %, −0.10 %, −0.024 % at n = 16, 32, 64. Both converge at second order or better and both
  underestimate. The vertex rule is the worse one because of the triangles at the centre of
  rotation: the vertex that does not slip carries no friction, and the interpolated force grows
  from zero where the true one is μ·p all the way (a fan triangle about the centre gives ½·μ·p·A·R
  instead of ⅔).
- A box sliding and spinning on a slab that is whole, cut in two or cut in eight: the closed form
  gives the same wrench (1e-15); the vertex rule changes by 17 % and 29 %, the one-point rule by
  42 % and 75 %. Cutting a body gives a quadrature other triangles; the exact integral does not
  see the cut. Without spin all three are independent of the cut.

## 2. Per-face elastic foundation: `pcm`

`PcmParams`, `pcm_force` (`core/include/volcontact/pcm.hpp`, `core/src/pcm.cpp`).

### Published formulations

**Hippmann's Polygonal Contact Model** (Hippmann 2004; equations as restated in Hippmann 2024,
*Polygonal contact model revisited*, Multibody System Dynamics):

- contact elements: the triangles of the base surface whose centroid is inside the target surface;
- penetration u_nk: from the centroid along the triangle's inward normal (the "penetration line")
  to the target surface;
- F_ck = c_l·A_k·u_nk (eq. 1), c_l = K/b the layer stiffness in N/m³ (eqs. 2–3);
- F_dk = d_l·A_k·u̇_nk (eq. 4);
- a regularized friction law perpendicular to the penetration line, per element; the total is
  the sum over the elements.

**Simbody's `ElasticFoundationForce`** (Sherman, Seth and Delp 2011; class documentation and
`ElasticFoundationForce.cpp`):

- a spring at the centre of each face of each mesh; it is displaced if its base is inside the
  other body, by the distance x to the nearest point of the other body's surface;
- f = k·a·x·(1 + c·v), v the rate of x, no tension (f > 0 only), along the displacement, applied
  at the nearest point;
- friction (Hollars): f·[min(v_s/v_t, 1)·(u_d + 2·(u_s − u_d)/(1 + (v_s/v_t)²)) + u_v·v_s] against
  the slip v_s;
- "when two meshes collide, the springs on each mesh are treated independently"; its source
  scales the spring areas by one half in that case (recalled from `ElasticFoundationForce.cpp`,
  not read again for this work; the half is in any case what makes the two sides add up to K·V,
  `literature.md` §5).

### What `pcm` does

For every triangle (area A, centroid c) of one body's surface inside the other body:

- **Penetration** x, `PcmParams::penetration`:
  - `kNearestPoint` (default, Simbody): the distance to the nearest point of the other body's
    surface; the force is along that displacement and acts at the nearest point;
  - `kFaceNormal` (PCM): the distance along the triangle's inward normal to the other body's
    surface; the force is along the triangle's normal and acts at c.
- **Normal force** F = (K/2)·A·x·(1 + h·ẋ), zero if negative: Simbody's law with k = K and
  c = h; PCM's eqs. 1 and 4 with c_l = K and the damping d_l = K·h·x (PCM's own d_l is a
  constant; the product form is the one that reproduces p = K·ℓ·(1 − h·vₙ) of the other laws).
- **Friction** F·min(v/v_t, 1)·(μ_d + 2·(μ_s − μ_d)/(1 + (v/v_t)²)) against the slip at the same
  point: Simbody's law without the viscous term. v_t = 24.5 mm/s, the `v_stick` of the other laws.
- **Both bodies** carry springs, each with half the stiffness (Simbody's half area), so that a
  flat contact carries K·A·δ = K·V like the other laws (`literature.md` §5).
- **No stiction.** Neither publication has an equivalent of the lumped stick of `distributed`
  (Simbody: "this model cannot represent stiction"), so `pcm` has the publications' regularized
  friction only and keeps no state between steps: a body on a slope below the friction angle
  creeps at up to v_t. This is the model, and it shows in every scenario that must hold still.

### Departures from the publications

1. **Elements are the triangles of the exact contact surface**, the part of each face inside the
   other body (fan triangles of the clipped polygons, the same that `distributed` integrates on),
   instead of the whole faces whose centroid is inside. A face wholly inside is exactly the
   publications' element. Reason: the battery's boxes and grounds have two triangles per face; a
   ground triangle has its centroid metres away from the body and would never be in contact, and
   a box face would switch on whole. The publications ask for a fine base mesh instead; the
   clipping gives PCM that resolution at the boundary of the patch for free and removes its force
   jumps (Hippmann 2024, Fig. 2), so it **favours the baseline**. What remains of the published
   model's discretization is one sample of depth, velocity and slip per element.
2. **Default penetration is Simbody's nearest point, not PCM's penetration line.** With clipped
   elements, the thin side strips of an overlap (a box's side wall, a millimetre into the ground)
   are elements; their inward normal is parallel to the ground, and PCM's line runs through the
   whole ground (metres). The published PCM has the same defect once the centroid of a side face
   is inside the target. The nearest point gives those strips their depth below the surface. On
   faces that face the other surface the two measures agree to first order. `kFaceNormal` is kept
   and tested, and is usable where every element faces the other body.
3. **The target is a convex piece**: its nearest point and the end of a penetration line are
   found on its face planes, leaving out the seams against sibling pieces (as `distributed`
   does). A line or a nearest point that would fall past a seam is measured to the plane of the
   piece's own surface.
4. **No viscous friction term** (u_v = 0).
5. **The explicit coupling bounds** the damping h and the transition velocity v_t exactly as for
   the other laws (`StepLimits`; there is no stick to bound).
6. The intersection volume is computed first, as for the other laws; `pcm` uses it for the
   broad-phase result and the step bounds only.

### Tests

- A flat face: K·A·δ, plus the K·P·δ²/4 of the strips of the side walls (perimeter P), to 1e-9;
  (1 + h·ẋ) on every spring; no tension.
- Two triangles worked by hand, at rest, sliding and closing, below the transition velocity and
  with μ_s ≠ μ_d, for Simbody's and for Hippmann's penetration.
- The step bounds: the normal damper of a 1 kg block loaded 1000 times its weight takes at most
  the velocity of a step, and the friction of a step at most the sliding velocity, for `pcm`,
  `distributed-vertex` and `distributed-centroid`.

## 3. Results

Single runs on Chrono at 1 ms (2026-11-11; not the battery), error of each scenario's metric in
percent. `mesh32`, `base` and `mesh128` are 32, 64 and 128 segments on the curved assets; the flat
faces keep their triangles. The rows are in `build/ablation/ablation.csv` when reproduced with
`volcontact_bench --engine chrono --scenario <name> --contact
distributed,distributed-vertex,distributed-centroid,pcm --variant base,mesh32,mesh128`.

| Scenario | Variant | `distributed` (closed form) | `distributed-vertex` | `distributed-centroid` | `pcm` |
|---|---|---|---|---|---|
| `disk` | mesh32 | 0.365 | 34.2 | 10.4 | 10.4 |
| `disk` | base | 0.123 | 33.6 | 9.98 | 10 |
| `disk` | mesh128 | 0.0627 | 33.4 | 9.88 | 9.9 |
| `ring` | mesh32 | 0.204 | 0.528 | 0.313 | 0.333 |
| `ring` | base | 0.0721 | 0.153 | 0.0994 | 0.119 |
| `ring` | mesh128 | 0.0389 | 0.0593 | 0.0458 | 0.0652 |
| `contensou` | mesh32 | 0.172 | 12.9 | 5.18 | 6.95 |
| `contensou` | base | 0.0641 | 12.7 | 4.94 | 6.67 |
| `contensou` | mesh128 | 0.0373 | 12.6 | 4.88 | 6.62 |
| `cone` | mesh32 | 0.68 | 0.984 | 0.67 | 8.21 |
| `cone` | base | 0.0746 | 0.248 | 0.0698 | 2.17 |
| `cone` | mesh128 | 0.319 | 0.262 | 0.322 | 0.373 |
| `bearing` | mesh32 | 3.54 | 3.54 | 3.54 | 0.996 |
| `bearing` | base | 1.6 | 1.6 | 1.6 | 0.176 |
| `bearing` | mesh128 | 0.483 | 0.483 | 0.483 | 0.207 |
| `journal` | mesh32 | 5.27 | 5.27 | 5.27 | 7.39 |
| `journal` | base | 4.57 | 4.57 | 4.57 | 2.48 |
| `journal` | mesh128 | 5.05 | 5.05 | 5.05 | 1.8 |
| `block` | base | 0.618 | 0.618 | 0.618 | 1.05 |
| `push` | base | 0.0775 | 0.0773 | 0.0775 | 95.8 |
| `turntable` | mesh32 | 81.4 | 81.4 | 81.4 | 56.6 |
| `turntable` | base | 69 | 69 | 69 | 56.8 |
| `turntable` | mesh128 | 68.6 | 68.6 | 68.6 | 56 |
| `energy` | mesh32 | 5.3 | 5.3 | 5.3 | 4.05 |
| `energy` | base | 0.357 | 0.357 | 0.357 | 0.414 |
| `energy` | mesh128 | 0.137 | 0.137 | 0.137 | 0.167 |
| `pieces64` | base | 0.164 | 14 | 30.1 | 30.3 |
| `bowl` | mesh32 | 21.5 | 21.4 | 21.5 | 20.4 |
| `bowl` | base | 14.6 | 14.6 | 14.6 | 15.3 |
| `bowl` | mesh128 | 13.7 | 13.7 | 13.7 | 15.7 |
| `sphere` | mesh32 | 1.23 | 1.23 | 1.23 | 1.2 |
| `sphere` | base | 0.596 | 0.596 | 0.596 | 1.67 |
| `sphere` | mesh128 | 0.533 | 0.533 | 0.533 | 1.92 |

Reading it:

- **Spinning flat contacts** (`disk`, `contensou`, `pieces64`) are where the friction rule
  matters: 0.06–0.4 % in closed form against 10–34 % with either quadrature on `disk`, 5–13 % on
  `contensou`, 14–30 % on `pieces64`. This is a statement about the battery's triangulation, not
  about quadratures in general: the flat caps are fans from a rim vertex, which the segment count
  does not refine (the segments refine the rim and not the flat face), and on such a fan the
  one-point and vertex rules lose 18 % and 25 % of the spin torque at any number of segments
  (`tools/ablation_triangulation.py`, `results/ablation_triangulation.csv`). On a centre fan the
  one-point rule is within 0.04 %, and on well-shaped meshes both rules converge at second order.
  The closed form is exact and independent of the triangulation and of how the bodies are cut:
  `pieces64` is the same disk on a ground cut into 64 pieces, the closed form does not see the cut
  (0.16 %), the quadratures change with it. A quadrature needs the flat faces remeshed; the
  closed form does not. It is not "one to two orders of magnitude" more accurate in general.
- **`ring`** (an annulus: the centre of rotation is outside the contact) costs a quadrature
  little: 0.07 % against 0.10–0.15 %.
- **Where nothing spins about the face normal** (`block`, `push`, `energy`, `bowl`, `sphere`,
  `turntable`, `bearing`, `journal`) the three quadratures are the same to three digits, as they
  must be.
- **`pcm` on flat spinning contacts** behaves as `distributed-centroid` (10 % on `disk`): its
  friction is the one-point rule.
- **`pcm` on conforming curved contacts** is better than `distributed`: `bearing` 0.18 % against
  1.6 %, `journal` 2.5 % against 4.6 % at 64 segments. Its depth and its force follow the local
  surface, where `distributed` has one normal per pair of pieces (open problem 1). On `cone` it is
  worse at 32 and 64 segments (8 % and 2 % against 0.7 % and 0.07 %) and the same at 128.
- **`pcm` cannot hold**: `push` 96 % (the block creeps under any tangential load, having no
  stiction), and `block` 1.1 % against 0.6 %.
- **Curved on flat** (`sphere`): `pcm` does not converge with the mesh (1.2, 1.7, 1.9 %), since the
  flat ground under the sphere is one polygon sampled at a few points, however fine the sphere.
- `turntable` and `bowl` are dominated by the model's own physics for every Winkler law (rolling
  resistance of the damping; see `docs/history.md`), not by the quadrature.

Cost, contact time per step from the same single runs (a shared machine with a battery running:
indicative, not a timing pass):

| Scenario | Segments | `distributed` | `distributed-vertex` | `distributed-centroid` | `pcm` |
|---|---|---|---|---|---|
| `disk` | 32 | 86 µs | 88 µs | 70 µs | 20 µs |
| `disk` | 64 | 188 µs | 192 µs | 198 µs | 42 µs |
| `disk` | 128 | 528 µs | 508 µs | 532 µs | 91 µs |
| `sphere` | 32 | 36 µs | 32 µs | 30 µs | 14 µs |
| `sphere` | 64 | 164 µs | 152 µs | 145 µs | 57 µs |
| `sphere` | 128 | 1303 µs | 1286 µs | 1256 µs | 301 µs |

The closed form costs nothing that these runs can measure against either quadrature (within
±10 %, the noise of the machine): the time goes into the intersection and the column partition of
the pressure. `pcm` is 3 to 6 times cheaper per step because it has no partition: it samples the
depth once per triangle.
