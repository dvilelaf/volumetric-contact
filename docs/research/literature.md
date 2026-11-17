# Literature review: novelty of `distributed`, baselines, benchmarks, and whether there is a paper

Date: 2026-11-11. Scope: the `volumetric-contact` repository (two force laws, `gonthier` and
`distributed`, on the exact boolean intersection of triangle meshes; 20 analytic scenarios; Drake
hydroelastic and Chrono SMC as native baselines).

**Short answer.** `distributed` is not a new contact model. Its structure (exact intersection of
convex polyhedra, one pair normal from the vector area, a bed of springs along that normal
integrated exactly triangle by triangle, distributed Coulomb friction, lumped translational plus
torsional stiction springs, convex decomposition for non-convex bodies) was published by Hasegawa
and Sato in 2004. What the repository adds is narrower: a closed-form Coulomb friction wrench for a
linear pressure on a triangle, an elastic-foundation normalisation consistent with Gonthier and
with Drake, a careful implementation on general meshes, and a cross-engine battery with analytic
references. The defensible paper is a benchmark and comparison paper with the exactly integrated
law as its reference implementation, not a "new model" paper. Two problems in the present
comparison must be fixed first (section 4): the `gonthier` baseline omits Gonthier's Coulomb
spinning friction, and seven scenarios take their "theory" from the Winkler model itself.

---

## 0. Method and verification levels

Every reference in `references.bib` carries a `note` that says how it was verified. Four levels:

| Level | Meaning |
|---|---|
| **Read** | DOI or stable URL fetched and the full text read in this session. |
| **Read (local)** | DOI verified online; full text read from a copy in the owner's library. |
| **Metadata** | DOI resolved through Crossref (title, authors, venue, pages); the paper is paywalled and its text was **not** read. Statements about its content come from a secondary source that is named. |
| **UNVERIFIED** | No DOI or stable URL could be fetched (bot-gated repository or proceedings not online). Marked as such wherever used. |

Sources that could not be opened and that matter:

- **Hippmann 2004** (the original PCM paper): paywalled. PCM is described from Hippmann's own
  open-access 2024 paper, which restates the model and its equations.
- **Gonthier's PhD thesis (2007)**: UWSpace and Library and Archives Canada both sit behind bot
  checks, which were not circumvented. Gonthier's model is described from the 2005 ASME paper
  (local copy), the 2009 ECCOMAS paper (local copy) and Drake's documentation, which summarises
  §4.3 of the thesis.
- **Gonthier, McPhee, Lange 2007** (DETC2007-35311, Coulomb friction in the volumetric model):
  metadata only. This is the paper that specialises Gonthier's friction to Coulomb; see 4.1.
- **Luo and Nahon 2006, 2011**, **Wakisaka and Sugihara 2017**, **Hasegawa et al. 2003**: metadata
  only; their content is taken from Elandt et al. 2019 (§II-C) and Elandt's thesis (Table 3.1),
  both read.
- **Faure et al. 2008**: HAL is bot-gated; metadata from search-engine and dblp snippets only.

---

## 1. What `distributed` is, element by element

Restated from `core/src/distributed.cpp`, `polar_moments.cpp` and `intersection.cpp`, so that the
comparison below is checkable.

| # | Element | In the code |
|---|---|---|
| E1 | Geometry | Exact boolean intersection V = A∩B of two closed triangle meshes (BVH + BSP clipping). Output: volume, centroid, inertia, and the polygons of ∂V labelled by the body they lie on. |
| E2 | Pair normal | n = normalised vector area of the part of ∂V on B minus the part on A. One normal per pair of convex pieces. |
| E3 | Pressure | p(x) = K·ℓ(x)·(1 − h·vₙ(x)), clamped at zero; ℓ is the length of the column of V through x along ±n; K in N/m³, h in s/m. |
| E4 | Integration | ℓ is the minimum of affine functions (one per face plane of the two convex shapes), each polygon is split where the minimising plane changes, so ∫p and ∫p·x are exact for the elastic term; the damping product is interpolated from the vertices. |
| E5 | Traction direction, symmetry | The pressure acts along each face's own normal (not along n); A's faces and B's faces each carry half of K·ℓ. |
| E6 | Kinetic friction | Coulomb, μ·p against the local slip. In a triangle's plane a rigid relative motion is a rotation about an instantaneous centre q, and ∫p·r̂ dA and ∫p·|r| dA are evaluated in closed form for linear p. |
| E7 | Stiction | Lumped: one translational and one torsional spring–damper at the centre of pressure, k = μₛ·|Fₙ|/d, torsional k·ρ², capped at μₛ·|Fₙ| and at the static spin capacity μₛ∫p·|r| dA; blended with kinetic friction by exp(−u²/vₛ²). |
| E8 | Non-convex bodies | Offline convex decomposition; one pair per combination of pieces. The law requires convex pieces. |
| E9 | Coupling | Explicit external forces applied to the engine each step; friction state per pair. |

---

## 2. Prior art, with the key equations

### 2.1 Hasegawa and Sato 2004 — the closest prior art [Read]

*Real-time rigid body simulation for haptic interactions based on contact volume of polygonal
objects*, Computer Graphics Forum 23(3). Earlier version: Hasegawa, Fujii, Koike, Sato 2003
[Metadata].

What the paper does, in its own words and equations:

- **Geometry.** "For efficiency of analysis on the contact status of polyhedra, we decompose
  polyhedra into convex polyhedra preliminarily." Contact detection with GJK; the common part of
  two convex polyhedra is built exactly with the Muller–Preparata dual transform and QuickHull:
  "This algorithm yields the geometry of contact volume (vertices and faces)."
- **One contact plane and normal per convex pair**, and "we put spring and damper models for
  normal and friction forces on this contact plane."
- **Distributed spring, integrated exactly per triangle of the intersection polyhedron.** With hₚ
  the distance from a point of a triangle of the contact volume to the contact plane (linear on
  the triangle), S_t the projected triangle area and S_c the contact area:

  > f_tri^Ns = (k^N/S_c) ∫_{S_t} hₚ n dS = k^N (S_t/S_c) h̄ n  (eq. 7)
  >
  > m_tri^Ns = (k^N/S_c) ∫_{S_t} p × (hₚ n) dS = k^N (S_t/S_c) (¾ h̄ p̄ + (1/12)(h_{p1}p₁ + h_{p2}p₂ + h_{p3}p₃)) × n  (eq. 8)

  "Dividing penetration dₚ into that of front-sides and back-sides of the contact plane hₚ, we
  integrate them for each triangle." The two sides add up to the full column through the overlap.
- **Distributed damper**, integrated in closed form (eqs. 9–12).
- **Distributed Coulomb friction.** Δf_p^D = μ Δf_p^N (v_p^T/‖v_p^T‖) (eq. 13), followed by:
  "However, analytical integration of this equation is difficult. Therefore, we interpolate
  dynamic friction forces on the three vertices of the triangle" (eqs. 14–16).
- **Static friction** as a distributed spring–damper whose stretch is a rigid displacement
  l_p = r + θ × p (eq. 20), integrated in closed form into a translational and a rotational spring
  (eqs. 21–24, 28–29); a two-state machine resets (r, θ) so that the static force equals the
  dynamic one when the maximum static friction is exceeded.
- **Evaluation**: stacked blocks, a cube falling on a floor, a stick–slip experiment against a
  real measurement, and a 6-DOF haptic demonstration.

Element by element against `distributed`:

| Element | Hasegawa–Sato 2004 | `distributed` | Verdict |
|---|---|---|---|
| E1 exact intersection | Convex pairs only (GJK + dual transform + QuickHull) | Any closed mesh (BVH + BSP), convex or not | Same idea; the repository's is more general for the volume, but the force law still needs convex pieces. |
| E2 one normal per pair | Contact normal from a separation/closest-point iteration | Vector area of ∂V | Different construction, same role. The vector-area normal is itself prior art (Hasegawa 2003 and Luo–Nahon 2006, per Elandt 2019 §II-C: they "assume that vector area defined by the boundary of the overlap region determines patch direction"). |
| E3 pressure ∝ column depth | Yes: depth to the contact plane on both sides, i.e. the column | Yes: ℓ along n | **Identical.** |
| Stiffness normalisation | k^N/S_c: the spring is "before distribution on contact area", so Fₙ = k^N × mean depth | K in N/m³: Fₙ = K·V (Winkler, as Gonthier) | Different, and the repository's is the physically meaningful one. Elandt's Table 3.1 lists Hasegawa as "∝ V", which is the 2003 version. |
| E4 exact integration | Yes: depth is linear on each triangle of the intersection polyhedron, integrated in closed form | Yes: via a min-of-affine partition of each face polygon | **Identical result for the normal wrench.** Hasegawa's route is simpler: each side's height to the contact plane is already linear on its triangle, so no partition is needed. |
| E5 traction direction | All along the contact normal n, on the contact plane | Along each face's own normal, on the faces | Differs. This is the "hydrostatic versus elastic-foundation direction" distinction that Elandt 2019 §IV draws; it is a modelling choice, not a new idea. |
| E5 symmetry | Symmetric by construction (plane between the bodies) | Symmetric (½ per side) | Same property. |
| E6 kinetic friction | Distributed Coulomb, **vertex interpolation** because "analytical integration … is difficult" | Distributed Coulomb, **closed form** about the instantaneous centre | **This is the genuine difference.** |
| E7 stiction | Translational + rotational spring from a distributed bristle, constant stiffness, two-state machine | Translational + torsional spring, load-proportional stiffness, smooth blend, capped | Same structure; the load-proportional stiffness and the capacity cap are refinements. |
| E8 convex decomposition | Yes | Yes | Identical. |
| Validation | Three demonstrations, one experiment | 20 analytic scenarios, two engines, native baselines | The repository is far stronger here. |

Elandt's thesis (2022, §2.0.7) describes exactly this design and names its difficulty:

> "An EF model implementation doesn't need to use discrete springs. An implementation could first
> calculate the polyhedral contact surface exactly from the overlapping polyhedral geometry. Then,
> it could integrate the traction analytically over this contact surface to obtain the net contact
> wrench. Although conceptually straightforward, this approach is not easy to implement in
> practice. As mentioned in [Hasegawa], analytic integration becomes difficult in the presence of
> friction."

A reviewer who knows this literature will read `distributed` as "Hasegawa–Sato with Winkler
scaling and the friction integral done analytically". That is the honest description, and the
paper should say it first.

### 2.2 Hippmann's Polygonal Contact Model (PCM) [2004: Metadata; 2024: Read]

From Hippmann 2024, *Polygonal contact model revisited*, Multibody System Dynamics (open access),
which restates the 2004 model:

- Base/target (formerly master/slave) surfaces. "All triangles of the base surface for which the
  geometric barycentres (centroids) are located on the inner side of the target surface are
  processed as contact elements. At each contact element a penetration line is constructed. It
  starts at the triangle barycentre and points along its normal direction to the inside of the base
  surface. … The distances between the penetration line start points … and the intersection points
  are used as penetrations of the contact elements."
- Elastic foundation force per contact element k:

  > F_ck = c_l · A_k · u_nk  (eq. 1),  c_l = K/b  (eq. 2),  K = (1 − ν)·E / ((1 + ν)(1 − 2ν))  (eq. 3)

  with layer stiffness c_l [N/m³], element area A_k, penetration u_nk, layer thickness b.
- Damping: F_dk = d_l · A_k · u̇_nk (eq. 4), with d_l the area-related layer damping.
- "A regularised friction force law is applied perpendicular to the penetration lines of the
  contact elements. The total contact force is calculated as a sum of all contact element forces."
- "The polygonal discretisation of the base surface defines the scanning resolution of the contact
  patch", so large polygons (planar regions) must be remeshed or the base/target roles swapped.
- The classic algorithm builds intersection polygons and active regions; the 2024 paper replaces
  it with direct ray–triangle "penetration detection".

Against `distributed`:

| Aspect | PCM | `distributed` |
|---|---|---|
| Foundation | Winkler, c_l in N/m³ | Winkler, K in N/m³ — **identical law** |
| Depth direction | Along each base triangle's **local normal** | Along the **single pair normal** |
| Quadrature | One sample at each triangle centroid; whole triangles switch on and off | Exact over clipped polygons |
| Force continuity | Discontinuous when a triangle centroid crosses the target surface (Hippmann 2024, Fig. 2, discusses the resulting force jumps and the chamfering/role-swap remedies) | Continuous in the pose |
| Symmetry | Asymmetric (base/target) | Symmetric |
| Non-convex shapes | Handled directly | Needs convex decomposition |
| Curved conforming contact | Natural (local normals) | The single normal is the weak point (`bearing`, `cone`) |
| Friction | Regularised Coulomb per element at the centroid | Closed form over the element |
| Mesh requirement | A fine base mesh | Any mesh; one large triangle is enough |

So `distributed` improves on PCM in quadrature, continuity, symmetry and mesh independence, and is
worse on curved conforming contact and non-convex shapes. Both are the same physical model.

### 2.3 Elastic foundation in biomechanics and Simbody/OpenSim [Read: Simbody source; Metadata + abstracts: the rest]

Blankevoort et al. 1991, Li et al. 1997, Fregly et al. 2003, Bei and Fregly 2004,
Pérez-González et al. 2008 use the elastic foundation model for articular and prosthetic knee
contact: springs on a discretised surface, pressure p = (1 − ν)E/((1 + ν)(1 − 2ν)) · d/h per
element (the same formula as PCM's eqs. 2–3, which Hippmann attributes to Johnson). Fregly 2003
validated it against static pressure experiments; Pérez-González 2008 compared Hertz, EFM and FEM
and proposed a modified EFM because EFM "predictions can differ from FEM in some circumstances".

Simbody's `ElasticFoundationForce` (Sherman, Seth, Delp 2011; class documentation in the source):

> "A spring is placed at the center of each face of each triangle mesh. When a mesh collides with
> any other object, the other object is considered to be rigid and the displacement of each spring
> is calculated independently. A spring is considered to be displaced if its base location is
> inside the other object, and the contact point is taken to be the nearest point on that object's
> surface. When two meshes collide, the springs on each mesh are treated independently."
>
> f = k·a·x·(1 + c·v),  k = (1 − p)·E / ((1 + p)(1 − 2p)·h)
>
> Friction (Hollars): f = fₙ·[min(vₛ/vₜ, 1)·(u_d + 2(uₛ − u_d)/(1 + (vₛ/vₜ)²)) + u_v·vₛ]. "Because the
> friction force is a continuous function of the slip velocity, this model cannot represent
> stiction."

Note that Simbody's normal law f = k·a·x·(1 + c·v) is the repository's p = K·ℓ·(1 − h·vₙ) with a
different sign convention, sampled once per face. The differences from `distributed` are the same
as for PCM (one sample per face, nearest-point depth, fine mesh needed, no stiction).

### 2.4 Gonthier's volumetric model and its descendants [2005, 2009: Read (local); 2004: Read (local); thesis and DETC 2007: not read]

Gonthier, McPhee, Lange, Piedbœuf 2005 derive everything from the Winkler foundation
integrated over a **flat** contact surface: "The proposed model uses a Winkler elastic foundation
approximation … and assumes that the contact surface S is flat, or can be reasonably well
approximated by a flat contact surface. … Under these assumptions, all objects must be decomposed
in convex sub-parts." The results, in the 2009 notation:

> fₙ = k_v·V·(1 + a·v_cn)·n,  τ_r = k_v·a·J_v·ω_t  (eq. 1)
>
> f_t = −μ·fₙ·v_ct,  τ_s = −(μ·fₙ/V)·J_v·ω_n  (eq. 2)

with V, p_c, J_v the volume, centroid and inertia tensor of the overlap. In the 2005 derivation
the friction element is dfₜ = μ·vₜ·dfₙ (eq. 41), "where μ is some constant that will be defined in
the model implementation … if the magnitude of vₜ is set to one, then μ would represent either the
static or kinetic friction coefficient", and an average surface velocity

> v_avg² = v_ct·v_ct + ω_n·(J_c/V)·ω_n  (eq. 32),  r_gyr² = n·(J_c/V)·n

is defined to drive stick–slip globally. The 2009 paper states that the model "factors-in the
effect of relative motion on the magnitude of the Coulomb friction force and torque [Gonthier,
McPhee, Lange 2007]", and Boos and McPhee (2010, local copy) that "the Contensou effect is
included". So the published Gonthier model has Coulomb sliding and Coulomb spinning friction
coupled through v_avg.

The 2005 paper also names the alternative that `distributed` takes: "Hippmann (2003) uses this
approach to deal with complex, i.e., conforming, polygonal, geometries", by "numerically summing
up the force contribution from each contacting polygon" (2009). `distributed` is therefore the
unlumped form of Gonthier's own hypothesis: the same integrand, evaluated on the mesh instead of
through V, p_c and J_v. That is a clean way to present it, and it is not new as an idea.

Descendants (metadata only): Boos and McPhee 2013 (experimental validation of the normal force,
cylinder and sphere on plane); Sandhu and McPhee 2010, Shourijeh and McPhee 2015, Brown and McPhee
2018 (foot–ground contact with analytic sphere and ellipsoid volumes); Petersen and McPhee 2013
(wheel–soil); Roy and Carretero 2012 (damping from material properties). All use closed-form
volumes of primitives, none a general mesh intersection. Bouchard et al. 2015 [Read] do the
graphics counterpart: normal from the volume gradient, position from the first moment, patch extent
from the second moment, feeding a constraint solver with rolling and spinning friction.

What the 2018 thesis and this repository add to that line is the **mesh-based** evaluation of V,
p_c, J_v (exact boolean of arbitrary triangle meshes), which the Waterloo group did not publish.

### 2.5 Other overlap-volume methods

- **Luo and Nahon 2006, 2011** [Metadata; content from Elandt]: polyhedral bodies, normal from the
  vector area, a lumped force at the patch centroid, magnitude approximately ∝ V.
- **Wakisaka and Sugihara 2017** [Metadata; content from Elandt]: "combines a volumetric
  intersection computation with kinematic constraints".
- **Faure et al. 2008** [UNVERIFIED, snippets only] and **Allard et al. 2010** [Metadata]: GPU
  layered depth images give the intersection volume and its gradient; forces are a uniform pressure
  on the boundary of the overlap (penalty in 2008, constraints in 2010). Resolution-limited, no
  exact geometry, handles deformable and non-convex objects.
- **Weller and Zachmann 2009** [Metadata]: inner sphere trees approximate the penetration volume
  for penalty forces at haptic rates. The 2018 thesis's sphere-based variant is in this family.
- **MSC Adams** solid-to-solid contact [Read: Adams View 2023.2 user guide]: "Adams Solver computes
  the volumes of intersection … finds the centroid of the intersection volume … finds the closest
  point on each solid to the centroid. The distance between these two points is the penetration
  depth", then F = K·(distance)ⁿ. An industrial code already computes mesh intersection volumes,
  but uses them only to place a point force.
- **Yamane and Nakamura 2006** [Read]: penalty contact at vertices of polygonal objects; not a
  volume method. Included only because it was on the list.

Elandt et al. 2019 (§II-C) classify all of these: "These methods rely on a single representative
patch normal, so they do not extend well to non-convex contact (e.g., a peg in a hole)." That
sentence applies to `distributed` as well, and `bearing`, `cone` and `hole` are where it shows.

### 2.6 Pressure field / hydroelastic contact [Read]

Elandt, Drumwright, Sherman, Ruina 2019: "The model combines and generalizes two ideas: a bed of
springs (an 'elastic foundation') and hydrostatic pressure. … continuous pressure fields are
computed offline for the interior of each nominally rigid object. … When two objects nominally
overlap, a contact surface is defined where the two pressure fields are equal." Tractions:
normal pressure p₀ plus a Hunt–Crossley-type rate term (eq. 3) and Coulomb shear of magnitude
μ·p against the projected slip (eq. 7); the wrench is the surface integral, evaluated by quadrature
on a fan triangulation of each polygon. Stated distinguishing features: "(i) its output varies
continuously with changing geometry, (ii) it is conservative, (iii) it does not depend on small
relative angles of the contacting surfaces, nor even simply connected regions of overlap, and (iv)
it can be tuned to make contact stiffness penetration dependent." On elastic foundations: "for
large relative surface angles, both continuous and discrete elastic foundation models are
generally not conservative".

Masterjohn, Guoy, Shepherd, Castro 2022 (Drake's discrete version): each polygon of the contact
surface becomes one compliant point contact at its centroid,

> f_{n,e} = A·p_c  (eq. 12, exact for linear p),  k = g·A,  φ₀ = −p_{c,0}/g  (eq. 16)

with g the effective pressure gradient in Pa/m, and friction solved at that point by the discrete
solver (TAMSI, SAP; later the "similar" and "lagged" convex approximations of Castro, Han,
Masterjohn 2025). Friction torque inside one polygon is therefore not represented; it converges
with mesh refinement.

Drake's documentation ("Estimation of Hydroelastic Parameters") makes the link explicit:
"we can see hydroelastic contact as a modern rendition of EFM … [Gonthier, 2007] finds … fₙ = κ·V,
where κ = κᵃ·κᵇ/(κᵃ + κᵇ) … κᵃ = Eᵃ/Hᵃ". It also lists what hydroelastic adds over EFM:
"generalizes to arbitrary non-convex geometry; large 'deformations' are allowed; efficient
algorithm to compute continuous contact patches; continuously differentiable contact forces", and a
limitation: "You can't get contact surfaces between two rigid hydroelastic geometries."

Against `distributed`:

| Aspect | Hydroelastic (Drake) | `distributed` |
|---|---|---|
| Small-penetration normal force | κ·V with κ = E/H | K·V — **the same** (K = E/H) |
| Input geometry | Tetrahedral pressure field per compliant body (or half-space, or rigid surface); offline meshing | Surface meshes only; no volume mesh, no SDF |
| Contact surface | Equal-pressure surface, curved, inside both bodies | The faces of ∂V; one flat reference direction per convex pair |
| Non-convex, multi-patch, peg-in-hole | Native | Needs convex decomposition and suffers at seams |
| Two stiff bodies | Not between two "rigid hydroelastic" geometries | Any pair |
| Friction quadrature | One point per polygon (discrete) or Gauss points (continuous) | Closed form per triangle |
| Solver coupling | Implicit, convex (SAP and successors); also error-controlled (CENIC, Kurtz and Castro 2025) | Explicit external force |
| Stiction | Resolved by the solver | Lumped bristle |
| Differentiability, conservative forces | Designed for both | Not analysed |

Newer work in this line (2023–2026), all hydroelastic-style rather than exact-polyhedral:
Castro, Han, Masterjohn 2025 (irrotational contact fields; convex approximations with
Hunt–Crossley and Coulomb); Han, Masterjohn, Castro 2023 (rigid–deformable); Kurtz and Castro 2025
(CENIC, error-controlled integration with contact); **Newton** (open source, GPU, built on NVIDIA
Warp) ships "hydroelastic contacts, inspired by [the] Drake contact model", built on sparse SDFs
and marching cubes, with a stiffness k_h in Pa/m, i.e. the same unit as K (NVIDIA blog, 2026); the
HolidayFactory 1.0 report (arXiv:2611.11789, November 2026) extends Newton's SDF-hydroelastic
contact with other constitutive laws; Nechyporenko et al. 2026 (arXiv:2608.00263) build a compliant
sphere lattice for distributed patches. The direction of travel in robotics is pressure fields on
SDFs or tetrahedra, on the GPU, inside implicit solvers.

### 2.7 Engines without a patch model

- **MuJoCo** [Read: documentation]: convex soft constraints; "meshes specified by the user can be
  non-convex … for collision purposes however they are replaced with their convex hulls"; several
  points per pair with `multiccd`; SDF plugins are the exception. Torsional and rolling friction
  are per-contact coefficients (`condim` 4 and 6), "useful for modeling soft fingers", i.e. patch
  effects enter as user parameters with a unit of length.
- **Incremental Potential Contact** and its rigid and affine variants (Li et al. 2020; Ferguson
  et al. 2021; Lan et al. 2022) [Metadata]: barrier potentials that forbid intersection, so there
  is no overlap volume at all; friction is a smoothed, lagged Coulomb model on primitive pairs.
  Relevant only as the opposite design choice.
- **Chrono SMC** (Tasora et al. 2016; Fleischmann et al. 2016) [Metadata]: point contacts with
  Hertz or Hooke laws and tangential displacement history.

### 2.8 Coulomb friction over a patch with planar rigid motion

- **Contensou 1963**; **Zhuravlev 1998**; **Kireenkov 2008**; **Leine and Glocker 2003**
  [Metadata]: the sliding–spinning coupling for a circular patch with a given (Hertz or uniform)
  pressure, in closed form or Padé form, and as a set-valued law.
- **Goyal, Ruina, Papadopoulos 1991**; **Howe and Cutkosky 1996**; **Xydas and Kao 1999**;
  **Zhou et al. 2018** [Metadata]: the limit surface of a planar patch, ellipsoidal and polynomial
  approximations, soft-finger models.
- **Jia 1997 (thesis) and Jia and Erdmann 1999** [Read: thesis]: the friction force and moment on
  a sliding polygon about its instantaneous rotation centre **in closed form**: "For polygonal
  shapes, the closed forms of I₁ and I₂ are given in Appendix C" and "Closed forms of integral Γ
  exist for polygons but not for ellipses. … For each polygon edge integrate over the triangular
  region defined by the instantaneous rotation center and the edge; then add the integral to or
  subtract it from the total." This is the same edge-by-edge decomposition as `polar_moments.cpp`,
  for **uniform** pressure.
- **Wilton et al. 1984** [Metadata]: closed-form potential integrals for uniform **and linear**
  densities on polygons, the standard boundary-element result with the same integrand family
  (asinh and sec³ primitives).

So the closed form for uniform pressure on a polygon is from 1997, and the linear-density
integrals are textbook in another field. What I did not find, in any contact-simulation paper, is
the combination used here: Coulomb friction force and moment for a **linear** pressure on a
triangle about the instantaneous centre, used as the friction quadrature of a compliant contact
law. Hasegawa (2004) and Elandt (2022) both say the integral is difficult and avoid it. This is a
small, real contribution. "Not found" is the correct strength of the claim; the search was not
exhaustive in the Russian-language tribology literature.

### 2.9 Pre-sliding and bristle friction with load-proportional stiffness

Dahl and LuGre models (Canudas de Wit et al. 1995; Olsson et al. 1998), the elasto-plastic model
(Dupont et al. 2002; Hayward and Armstrong 2000) and the bristle model the thesis used (Dopico et
al. 2011) [all Metadata]. In the tyre form of LuGre (Canudas de Wit and Tsiotras 1999) the friction
force is the normalised bristle state times the normal load, which is a load-proportional bristle
stiffness. Gonthier's 2004 paper is itself about dwell-time-dependent friction. The repository's
k = μₛ·|Fₙ|/d is therefore standard practice, correctly applied; it is not a contribution.

---

## 3. Novelty verdict

### Claims that would not survive a hostile reviewer

1. **"A new contact model."** No. Same foundation law as PCM, Simbody EFM and Gonthier; same
   architecture as Hasegawa–Sato 2004.
2. **"Exact piecewise-affine column integration."** Hasegawa–Sato integrate the same linear depth
   exactly per triangle (eqs. 7–8), with a simpler construction.
3. **"Pair normal from the vector area."** Hasegawa 2003 and Luo–Nahon 2006.
4. **"Symmetric treatment."** Any overlap-volume method is symmetric; only PCM-type methods are not.
5. **"Distributed friction gives spin and Contensou coupling without shape factors."** Hasegawa–Sato
   state this as their motivation; hydroelastic contact does it too.
6. **"Lumped translational and torsional stiction."** Hasegawa–Sato eqs. 20–24.
7. **"Load-proportional stiction stiffness."** LuGre.
8. **"Works on surface meshes without tetrahedra or SDFs."** True, and true of PCM, Simbody and
   Hasegawa. It is an advantage over hydroelastic contact, not a novelty.
9. **"The thesis's idea holds / volumetric beats native."** As evidence it is currently weakened
   by the issues in section 4.

### Claims that would survive, if stated modestly

1. **Closed-form Coulomb friction wrench for linear pressure on a triangle under rigid slip**, and
   its use as the friction quadrature of an elastic-foundation law. Prior art says the integral is
   difficult; Jia 1997 solved the uniform case. To make it a result, show what it buys: accuracy
   and smoothness against vertex interpolation (Hasegawa), one point per face (PCM, Drake) and
   Gauss rules, at equal cost, on `disk`, `ring`, `contensou`.
2. **An elastic-foundation law whose quadrature error is zero for the elastic term on any mesh**,
   so that one triangle per face is enough. This is a property of Hasegawa's scheme too, but nobody
   has benchmarked it against PCM and hydroelastic contact.
3. **Mesh-based exact evaluation of Gonthier's V, p_c, J_v** for arbitrary closed meshes (the
   2018 thesis). Unpublished outside the thesis as far as this search found.
4. **The cross-engine analytic battery for conforming contact**, with fixed, scenario-independent
   mapping rules to each engine's native contact, including hydroelastic. I found no published
   benchmark that targets patch or conforming contact with analytic references across engines
   (section 6).
5. **Engine independence**: the same law gives the same numbers in Drake and Chrono, which
   separates model error from integrator error. Useful methodology, modest novelty.

---

## 4. Problems in the present comparison that a reviewer would find

These are findings about the repository, not about the literature, but they decide whether the
paper stands.

### 4.1 The `gonthier` baseline is not Gonthier's published friction model

`core/src/gonthier.cpp` computes the spinning torque as
`-(mu_spin * fn / V) * (J * w_n)`: eq. 2 of Gonthier 2009 with μ taken as a constant. That is the
viscous reading of dfₜ = μ·vₜ·dfₙ (torque ∝ ω_n). The sliding force, in contrast, is Coulomb with a
bristle evaluated at the centroid velocity only. Gonthier's Coulomb specialisation (DETC2007-35311,
and "the Contensou effect is included" in Boos and McPhee) normalises both by the average surface
velocity. I could not open that paper, so the exact published form must be checked, but with
eqs. 32, 47 and 55 of the 2005 paper the Coulomb form is

> f_t = −μ·fₙ·v_ct / v_avg,  τ_s = −μ·(fₙ/V)·J_c·ω_n / v_avg,  v_avg² = |v_ct|² + r_gyr²·ω_n².

For pure spin this gives τ = μ·fₙ·r_gyr. For a thin disk r_gyr = R/√2, against the exact
(2/3)·μ·fₙ·R: a torque 6 % too high, a stop time about 6 % too short (my calculation), not the
34 % the battery reports for `disk`. The same applies to `ring`, `contensou`, `cone` and `bearing`.
The thesis itself notes that its spinning law depends on ω. As it stands, the headline result
"`gonthier` fails with spin" compares against a weaker model than the one in the literature, and
the McPhee group would see it at once. **Implement the Coulomb form with v_avg before any
comparison is published**, and keep the thesis variant as a labelled option.

### 4.2 Seven scenarios test consistency with the Winkler model, not physics

The theories of `tip`, `drop`, `hole`, `bearing`, `corner`, `offset` and `stack` contain K: they
are the solutions of the Winkler foundation itself. `gonthier` and `distributed` are that model;
hydroelastic contact matches it only at small penetration through E = K·H; Chrono's Hertz contact
does not have that form at all. Their errors there (`offset` 113 %, `hole` 87 %) measure the
distance between two models, not from the truth. A reviewer will call this circular. Split the
battery into (a) rigid-limit scenarios whose theory is independent of the contact stiffness
(`block`, `disk`, `cylinder`, `push`, `billiard`, `contensou`, `vgroove`, `ring`, `bowl`, `trough`,
`twoblocks`, `cone`, `sphere`) and (b) foundation-consistency scenarios, and report them
separately. For (b), an independent reference (FEM of a thin elastic layer, or an experiment) is
what makes the comparison meaningful.

### 4.3 The statistics do not support the strongest sentences

On Drake the three methods are not distinguishable (Friedman p = 0.086; no pair significant after
Holm). "At least as accurate as the engines' own contact" is fair; "the best law overall" is not
yet. Twenty heterogeneous relative errors, some unbounded (inf for diverged runs), ranked together,
are a thin basis. Report per-scenario errors with their dt and mesh convergence, effect sizes, and
keep the rank test as a summary.

### 4.4 Known failure of `distributed` on the very contacts in the thesis title

`bearing` (52 %) and `cone` (27 %) are conforming curved contacts, where hydroelastic contact is
best. The single pair normal and the seams between convex pieces are structural consequences of
the Hasegawa/Gonthier design (Elandt 2019: such methods "do not extend well to non-convex
contact"). Either fix it in general or state it as the model's limit; a paper titled around
conforming contact cannot leave it as an open problem.

### 4.5 Speed claims

"10 to 1000 times faster than hydroelastic" compares an explicit force element with an implicit
convex solver on tetrahedral meshes of a chosen resolution. Reviewers expect cost against accuracy
(Pareto fronts over dt and mesh), the same accuracy target for all, and Drake configured as its
authors recommend (resolution hints, compliant–rigid assignment, discrete approximation, and the
error-controlled integrator of Kurtz and Castro 2025).

### 4.6 Other points to expect

- **Explicit coupling.** Stability limits at stiff contacts; show convergence in dt and the limit.
- **Constants.** d = 0.1 mm and τ = 5 ms are global constants; show the sensitivity.
- **Energy.** Elandt notes that elastic-foundation models are not conservative at large relative
  angles; measure the energy drift of a frictionless, undamped rocking or rolling case.
- **Continuity.** State and test that the wrench is continuous in the pose (the main advantage
  over PCM), including when the minimising face changes.
- **"Exact" wording.** The damping product and the stiction are not exact; the friction is exact
  only within each flat triangle.

---

## 5. Baselines worth benchmarking, ranked

Ranking by relevance to the claims first, cost second. "Core" means implemented on the existing
intersection surface in `core/`; "engine" means a backend's native contact.

| Rank | Baseline | Where | Cost | Why |
|---|---|---|---|---|
| 1 | Gonthier with Coulomb spinning friction (v_avg) | Core | Low | Without it the `gonthier` column is not the published model (4.1). |
| 2 | Hasegawa–Sato friction quadrature (vertex interpolation) and one-point-per-face quadrature, as switches of `distributed` | Core | Low | The ablation that isolates the one genuine contribution (closed-form friction). |
| 3 | PCM / per-triangle elastic foundation (Hippmann; Simbody) | Core | Medium | The established multibody baseline for conforming contact; same K; shows mesh dependence and force jumps. |
| 4 | Drake hydroelastic | Engine | Done | The state of the art; keep, and configure as recommended (4.5). |
| 5 | MuJoCo | Engine | Medium–high | The most used robotics engine; a convex-hull, multi-point, soft-constraint model. Parameter mapping is the difficulty. |
| 6 | Chrono SMC | Engine | Done | Point-contact reference; check the mapping before reporting divergence. |
| 7 | Drake point contact (compliant, same solver) | Engine | Low | Separates "patch versus point" from "Drake versus Chrono". |
| 8 | Hunt–Crossley point contact at the deepest point | Core | Low | Only if a core point-contact reference is wanted; no general mapping from K. |
| 9 | Simbody `ElasticFoundationForce` as an engine | Engine | High | Redundant with 3. |
| 10 | Newton SDF-hydroelastic, IPC/ABD | Engine | High | Context; different regime (GPU, barrier). Mention, do not benchmark. |

### Formulations and parameter mapping from (K [N/m³], h [s/m], μ)

**1. Gonthier, Coulomb form** (Gonthier et al. 2005 eqs. 32, 36, 40, 47, 55; Coulomb
specialisation per DETC2007-35311, to be confirmed against that paper or the thesis):

- fₙ = K·V·(1 − h·v_cn)·n; τ_r = −K·h·J·ω_t (already in the code).
- v_avg = √(|v_ct|² + ω_n·(J/V)·ω_n); f_t = −μ·fₙ·v_ct/v_avg; τ_s = −μ·(fₙ/V)·J·ω_n/v_avg;
  stick–slip blending driven by v_avg instead of |v_ct|.
- Mapping: identity.

**2. Friction quadrature switches inside `distributed`:**

- Hasegawa–Sato (eqs. 13–16): evaluate fᴰ = μ·p·v̂ₜ at the three vertices and interpolate:
  f_tri = A·f̄ᴰ, m_tri = A·(¾·p̄ × f̄ᴰ + (1/12)·Σ pᵢ × fᴰᵢ).
- One point: μ·(∫p dA)·v̂ₜ at the centroid (PCM, Simbody, discrete hydroelastic).
- Mapping: identity.

**3. PCM / per-triangle elastic foundation** (Hippmann 2024 eqs. 1–4; Simbody class documentation):

- For each triangle k of the base surface whose centroid is inside the target: u_k = distance from
  the centroid along the inward normal to the target surface (PCM), or to the nearest point of the
  target (Simbody).
- Fₖ = c_l·Aₖ·uₖ + d_l·Aₖ·u̇ₖ (PCM), or Fₖ = k·aₖ·xₖ·(1 + c·vₖ) (Simbody), along the penetration
  line; regularised Coulomb friction μ·Fₖ at the centroid.
- Mapping: c_l = k = K (the layer stiffness E′/b is K by definition). Damping: Simbody's c = h
  reproduces p = K·ℓ·(1 − h·vₙ) exactly; for PCM's linear damper use d_l = K·h·uₖ (the
  Hunt–Crossley-like option; PCM "has options to avoid discontinuous and tensile contact forces").
  μ unchanged. When both bodies are meshes and both act as base (Simbody treats "the springs on
  each mesh independently"), use K/2 on each so that the total is K·V, the same convention as the
  ½ weights in `distributed`. For two compliant layers in series the effective stiffness is
  κᵃκᵇ/(κᵃ + κᵇ) (Drake documentation, after Gonthier).
- Mesh rule needed: PCM's accuracy depends on the base mesh; report it at the battery's two
  resolutions and say that planar faces had to be subdivided.

**4. Drake hydroelastic** (Elandt 2019; Masterjohn 2022; Drake documentation):

- fₙ = κ·V at small penetration, κ = E/H per compliant body, κ = κᵃκᵇ/(κᵃ + κᵇ) for two compliant
  bodies; H is the foundation depth of the pressure field (half-space: slab thickness; box:
  smallest half side; sphere: R; cylinder: R for L > R per the documentation's table).
- Mapping: rigid–compliant pair: E = K·H (what the repository does). Compliant–compliant: E = 2·K·H
  each for equal H. Dissipation: `hunt_crossley_dissipation` d = h [s/m] with the continuous plant
  and the "similar" and "lagged" discrete approximations; with SAP the model is linear
  Kelvin–Voigt with a relaxation time, and there is no exact mapping (state which was used). μ: as
  the repository already solves, from Drake's combination 2μᵢμⱼ/(μᵢ + μⱼ).

**5. MuJoCo** (documentation, "Computation" and "Modeling"):

- Contacts are soft constraints solved by convex optimisation. Each scalar constraint follows a
  reference acceleration a_ref = −b·v − k·r, with (k, b) from `solref`, either
  (timeconst, dampratio) or the direct "(−stiffness, −damping)" format, scaled by the impedance d
  from `solimp`. With constant impedance, "the penetration depth at rest is
  r = a_u·(1 − d)·timeconst²·dampratio²".
- There is **no mapping from a foundation stiffness in N/m³**: stiffness is per unit of constraint
  mass and acts on a few points of the convex hull, so the static penetration depends on the
  acceleration, not on load per area. A generic rule has to be declared, for example: direct
  format with k = K·A_c/m_c per pair, A_c the nominal contact area and m_c the lighter body's mass,
  and b from h evaluated at the static penetration. This is weaker than the Drake rule and must be
  presented as such.
- μ: `friction[0]`, `condim` 3. Torsional friction (`condim` 4) needs a coefficient with units of
  length; the theoretical value (for a disk (2/3)·μ·R) would be calibration to the scenario, so run
  MuJoCo with `condim` 3 and `multiccd`, and optionally report `condim` 4 as an "informed" variant.
- Non-convex grounds need the same convex pieces; the 704-piece bowl will be expensive.

**6. Chrono SMC** (Tasora 2016; Fleischmann 2016; Chrono documentation): already mapped as Hertz
with E = K·1 m. Open problem 6 of `AGENTS.md` (divergence in six scenarios) has to be resolved
before publication; a native baseline that diverges because of the mapping is not a result.

**8. Hunt–Crossley point contact** (Hunt and Crossley 1975; the form below is Drake's
parameterisation, with d in s/m): F = k·δⁿ·(1 + d·δ̇) at the deepest point, d = h. The Winkler
equivalent depends on the geometry (flat face: k = K·A, n = 1; sphere:
k = π·K·R, n = 2, from Drake's table V = π·R·x²), so there is no scenario-independent rule. Use it
only as an illustration of why point contact needs per-geometry calibration.

---

## 6. Benchmarks in the literature

### What exists

| Work | Engines or models | Reference | Contact type | Level |
|---|---|---|---|---|
| González et al. 2006, 2010 (LIM, A Coruña) | MBS formulations | Reference solutions | Mechanisms, no contact | Metadata |
| IFToMM Library of Computational Benchmark Problems | Community | Submitted solutions | Mostly constrained mechanisms; I did not find conforming frictional contact problems (landing page read; catalogue not audited) | Read (site) |
| Ruggiu and González 2023 | MBS formulations | Analytic | Singular mechanism, no contact | Metadata |
| Flores, Ambrósio, Lankarani 2023 | Contact-force models | Classic problems | Contact–impact with friction | Metadata |
| Erez, Tassa, Todorov 2015 | Bullet, Havok, MuJoCo, ODE, PhysX | None: consistency under time-step refinement, speed–accuracy curves | Robotics scenes | Metadata |
| SimBenchmark (Hwangbo et al., RaiSim) | RaiSim, Bullet, ODE, MuJoCo, DART | Analytic trajectories for single bodies, energy and momentum otherwise; speed–accuracy curves | Point contacts | Read (site) |
| Chung and Pollard 2016 | Several engines | Predictability | Point contacts | Metadata |
| Fazeli et al. 2017 | Impact models | Experiment (planar impact) | Point | Metadata |
| Horak and Trinkle 2019 | Model classes (complementarity, convex, compliant) | Analysis and simple cases | Point | Metadata |
| Acosta, Yang, Posa 2022 | Robotics simulators (per the title; the list of engines was not re-read) | Real-world impact data | Impacts | DOI + arXiv record |
| Guo et al. 2023 | Analytical, learned, hybrid models | Synthetic dataset | Rigid bodies | Read (abstract) |
| Le Lidec et al. 2024 (ContactBench) | Solvers of the main robotics engines | "Theoretically grounded quantitative criteria" (Signorini, Coulomb, maximum dissipation) and timings | Point | Read (abstract) |
| Elandt 2019; Masterjohn 2022; Castro 2023 | Hydroelastic, SAP | Hertz, FEM trends, convergence, accuracy metrics | Patch | Read |
| Boos and McPhee 2013 | Volumetric model | Experiment (normal force) | Patch | Metadata |
| Erleben 2018 | Mesh-based contact point generation | Methodology | Point sets | Metadata |
| Reviews: Collins 2021; Liu and Negrut 2021; Yoon 2023; Roennau 2013 | Many | Mixed | Mixed | Metadata |

The gap: the robotics benchmarks test solvers on point contacts and avoid ground truth for patch
effects; the multibody benchmark tradition (which the owner's lab helped start) has no library of
conforming-contact problems; hydroelastic contact was validated by its authors on a few cases. A
battery of conforming and patch-contact problems with closed-form references, run across engines
and model classes with declared mapping rules, does not appear to exist. This is "not found", not
"proven absent".

### Is the 20-scenario battery publishable by itself?

As the core of a comparison paper, yes. Alone, as it stands, no. What reviewers of such papers ask
for, from the works above:

1. **Independent references.** Theories that do not embed the model under test (4.2); FEM or
   experiment for the foundation-dependent cases. At least one experimental dataset: real
   impact data such as that used by Acosta et al. (availability not checked here), a
   Hasegawa-style stick–slip test, or a spinning disk filmed on a table.
2. **Convergence studies.** Error against dt and against mesh resolution for every method, not one
   `dt` and one `fine` variant folded into a median.
3. **Cost–accuracy Pareto fronts** (the Erez 2015 and SimBenchmark format), per scenario class.
4. **Fair native configurations**, written down, ideally checked with each engine's developers,
   and no baseline that diverges because of the mapping.
5. **More than two engines**: MuJoCo at least; three model classes (point, per-triangle foundation,
   pressure field) plus the volumetric ones.
6. **Parameter sensitivity** (K, h, the stiction constants) and failure cases reported as such.
7. **Reproducibility**: public code, container, pinned versions, one command per table.
8. **A scenario specification others can implement**: `docs/scenarios.md` is already close to
   this, and it is the most reusable asset in the repository.

---

## 7. Is there a paper? Framings and what each needs

| Framing | Defensible? | Venue type | Additional work |
|---|---|---|---|
| A. New model or method ("a distributed volumetric contact model") | **No.** Hasegawa–Sato 2004, PCM and Gonthier cover it. | — | — |
| B. Benchmark and comparison ("conforming-contact benchmark battery; volumetric, elastic-foundation and pressure-field contact compared across engines") | **Yes — the strongest framing.** | Multibody System Dynamics, Mechanism and Machine Theory, J. Computational and Nonlinear Dynamics; robotics letters only with a manipulation or locomotion case | Section 4 fixes; baselines 1–5 and 7; convergence and Pareto studies; the split of the battery; one experimental or FEM reference; MuJoCo backend; resolve Chrono's divergence. |
| C. Software paper (JOSS or similar) | **Feasible later.** | JOSS | Public repository with history, licence, documentation, tests, contribution guidelines and a statement of need; check JOSS's current criteria on scholarly effort and public development history (the criteria page was reached but its text was not audited here). A software paper does not need novelty of the model. |
| D. Short communication or technical brief ("closed-form Coulomb friction wrench for linearly varying pressure on polygonal contact patches") | **Yes, small.** | Technical brief in a multibody or tribology journal | The derivation, the comparison with vertex, one-point and Gauss quadrature, continuity and cost; the Contensou and annulus checks; a careful prior-art paragraph (Jia 1997; Wilton 1984; Contensou; Hasegawa). |

**Recommendation.** Write B, with D as its method section. Present `distributed` as an *exactly
integrated elastic-foundation reference law* in the line Hasegawa–Sato → Gonthier → PCM →
hydroelastic, not as a new model; state its one new ingredient (the closed-form friction integral)
and its limit (one normal per convex pair). The contribution list would be:

1. A specification and open implementation of a battery of conforming and patch-contact problems
   with analytic references, engine-independent, with declared mapping rules.
2. A comparison of five model classes on it (lumped volumetric, exactly integrated foundation,
   per-triangle foundation, pressure field, point contact) in three engines, with convergence and
   cost.
3. The closed-form friction wrench and the evidence of what it buys.
4. Findings: where each class fails and why (single normal on curved conforming contact; viscous
   versus Coulomb spin in lumped models; one friction point per face; point contact on patches).

The thesis committee was chaired by Paulo Flores, and the benchmark tradition belongs to the
owner's lab (González et al. 2006): the multibody community is the natural audience.

If the section 4 fixes shrink the gaps (very likely for `gonthier` on `disk`, `ring`, `contensou`),
the paper's message changes from "distributed is best" to "exact integration matters where the
pressure distribution matters, lumped volumetric models are enough elsewhere, pressure fields win
on curved conforming contact". That message is more modest and much harder to attack.

---

## 8. Things stated here that still need checking

- The exact Coulomb form of Gonthier's friction (DETC2007-35311 or thesis chapter 5): not read.
- Luo and Nahon's and Wakisaka and Sugihara's formulations: known only through Elandt.
- Chrono's SMC force equations were not re-read in this session; no equation is quoted.
- The IFToMM catalogue was not audited problem by problem.
- JOSS's current submission criteria were not audited.
- Gonthier et al. 2009 (ECCOMAS, Warsaw) and Boos and McPhee 2010 (IMSD, Lappeenranta) were read
  from local copies; no online record was fetched, so they are marked UNVERIFIED in the bib file.
- The 6 % estimate for Gonthier's Coulomb spin on a thin disk is my own calculation
  (r_gyr = R/√2 against 2R/3), not a simulation.
- Scope statements for the "Metadata" rows of the benchmark table (Erez 2015, Horak and Trinkle
  2019, Chung and Pollard 2016, Fazeli 2017, Flores 2023, the reviews) come from titles and general
  knowledge of those papers, not from a reading in this session; re-read them before citing detail.

---

## 9. BibTeX keys

`references.bib` uses AuthorYear keys. Those quoted or relied on above: `Hasegawa2004`,
`Hasegawa2003`, `Hippmann2004`, `Hippmann2024`, `SimbodyEFF`, `Sherman2011`, `Blankevoort1991`,
`Li1997`, `Fregly2003`, `Bei2004`, `PerezGonzalez2008`, `Johnson1985`, `Gonthier2004`,
`Gonthier2005`, `Gonthier2007detc`, `Gonthier2007thesis`, `Gonthier2009`, `Boos2010`, `Boos2013`,
`Brown2018`, `Shourijeh2015`, `Petersen2013`, `Roy2012`, `Bouchard2015`, `Luo2006`, `Luo2011`,
`Wakisaka2017`, `Faure2008`, `Allard2010`, `Weller2009rss`, `Weller2009unified`, `AdamsView2023`,
`Yamane2006`, `Elandt2019`, `ElandtThesis2022`, `Masterjohn2022`, `Castro2020`, `Castro2023`,
`Castro2025`, `Han2023`, `Kurtz2025cenic`, `DrakeHydroGuide`, `DrakeHydroParams`, `NewtonRepo`,
`NewtonBlog2026`, `HolidayFactory2026`, `Nechyporenko2026`, `MuJoCoDocs`, `Li2020ipc`,
`Ferguson2021`, `Lan2022`, `Tasora2016`, `Fleischmann2016`, `Contensou1963`, `Zhuravlev1998`,
`Kireenkov2008`, `Leine2003`, `Goyal1991`, `Howe1996`, `Xydas1999`, `Zhou2018`, `Jia1997`,
`Jia1999`, `Wilton1984`, `CanudasDeWit1995`, `CanudasDeWit1999`, `Olsson1998`, `Dupont2002`,
`Hayward2000`, `Dopico2011`, `HuntCrossley1975`, `Muller1978`, `Gonzalez2006`, `Gonzalez2010`,
`IFToMMBenchmark`, `Ruggiu2023`, `Flores2023`, `Erez2015`, `SimBenchmark`, `Hwangbo2018`,
`Chung2016`, `Fazeli2017`, `Horak2019`, `Acosta2022`, `Guo2023`, `LeLidec2024`, `ContactBench`,
`Erleben2018`, `Collins2021`, `Liu2021`, `Yoon2023`, `Roennau2013`, `Vilela2018`, `JOSSCriteria`.

Also in the file, verified by DOI and useful for a related-work section but not relied on above:
`Vehvilainen2024` (PCM applied to bearings), `Flores2011`, `Machado2012`, `Skrinjar2018`,
`Corral2021`, `Flores2022` (reviews of continuous contact-force models), `Todorov2012`,
`Todorov2014` (MuJoCo), `Kurtz2025` (contact-implicit MPC in Drake), `Macklin2020` (SDF collision),
`Wei2022coacd` (CoACD), `Mirtich1996` (polyhedral mass properties), `Mason1986` (pushing),
`Drake`, `MuJoCoWarp`, `ChronoDocs`.
