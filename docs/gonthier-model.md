# The `gonthier` law and its sources

Date: 2026-11-11. This file states which equations `core/src/gonthier.cpp` implements, where each
one is published, and where the implementation departs from the publication.

The law has two friction formulations, selected with `GonthierParams::friction_model`:

- `kPublished` (default): Gonthier's volumetric friction model, with Coulomb sliding and Coulomb
  spinning friction coupled through the average surface velocity. Report it as
  "Gonthier (published)".
- `kThesis2018`: the form implemented in Vilela's 2018 thesis, with a viscous spinning torque that
  is independent of the sliding force. Report it as "Gonthier (thesis 2018)".

The normal force and the rolling resistance are the same in both.

## Sources

| Key | Reference | How it was read |
|---|---|---|
| **T** | Y. Gonthier, *Contact Dynamics Modelling for Robotic Task Simulation*, PhD thesis, University of Waterloo, 2007 (UWSpace, open access) | Full text, fetched 2026-11-11 from the UWSpace bitstream `dspacemainprd01.lib.uwaterloo.ca/server/api/core/bitstreams/f1ba0293-e2bc-4a72-a258-318c42d1e105/content`. §3.2.2 (point bristle model), §4.2–4.3 (normal force, rolling, friction, spin), §4.4 (Coulomb friction, average surface velocity, volumetric bristle model) and §4.5 were read; equation numbers below are the thesis's. |
| **P05** | Gonthier, McPhee, Lange, Piedbœuf, "A contact modeling method based on volumetric properties", ASME IDETC 2005, DETC2005-84610 | Full text, owner's local copy. |
| **P04** | Gonthier, McPhee, Lange, Piedbœuf, "A regularized contact model with asymmetric damping and dwell-time dependent friction", Multibody System Dynamics 11 (2004) 209–233 | Full text, owner's local copy (point-contact bristle model, the origin of T §3.2.2). |
| **P09** | Gonthier, Lange, McPhee, "A volumetric contact model implemented using polynomial geometry", ECCOMAS Multibody Dynamics 2009 | Full text, owner's local copy (restates eqs. 1–2 and cites the two 2007 papers for stick–slip and for the Coulomb magnitude). |
| **B10** | Boos, McPhee, "Volumetric contact models and experimental validation", IMSD 2010 | Full text, owner's local copy ("the Contensou effect is included"). |
| **V18** | D. Vilela, PhD thesis, 2018, chapter `contact-models.tex` | Full text, owner's local copy. |

**Not read** (paywalled or unreachable; nothing below relies on them): Gonthier, McPhee, Lange,
"On the implementation of Coulomb friction in a volumetric-based model for contact dynamics",
DETC2007-35311; Gonthier, Lange, McPhee, "On implementing a bristle friction model in a contact
model based on volumetric properties", ECCOMAS 2007; Boos and McPhee, JCND 2013; Brown and McPhee,
MSD 2018. The two 2007 papers are the conference versions of T §4.4, which was defended the same
year and is the thesis author's complete statement of the model; that they contain the same equations is
an inference from P09's citations and from T's abstract, **not verified**.

## Notation

V, p_c, J: volume, centroid and inertia tensor (unit density, about p_c) of the overlap of the two
undeformed bodies. n: contact normal, from B to A. v_c: velocity of A relative to B at p_c;
v_cn = n·v_c, v_ct = v_c − v_cn·n. ω: angular velocity of A relative to B; ω_n = n·ω,
ω_t = ω − ω_n·n. The publication writes the normal force as k_v·V·(1 + a·v_cn) with v_cn positive
when approaching; the code uses v_cn positive when separating, hence 1 − h·v_cn. K = k_v, h = a.

## Equations implemented, both formulations

| | Equation | Source |
|---|---|---|
| Normal force | f_n = K·V·(1 − h·v_cn), along n, at p_c | T (4.62), (4.178); P05 (36) |
| Rolling resistance | τ_r = −K·h·J·ω_t | T (4.67); P05 (40) |

The publication requires 1 + a·v_n ≥ 0 (T (3.20)). The code clamps f_n at zero by default; when the
clamp is active the pressure of the foundation is zero, so the rolling torque, which is the moment
of that pressure, is zero too (it was applied regardless before 2026-11-11).

## Equations implemented, `kPublished`

Radius of gyration of the overlap about the normal and average surface velocity:

    r_gyr² = n·J·n / V                                  T (4.201); P05 (30)
    v_avg² = |v_ct|² + r_gyr²·ω_n²                      T (4.212); P05 (32)

Coulomb friction force and spinning torque, with the Contensou factors:

    f_tc = −μ_C·f_n·C_v·v_ct/|v_ct|,     C_v = |v_ct| / v_avg          T (4.205)
    τ_sc = −μ_C·r_gyr·f_n·C_ω·sgn(ω_n),  C_ω = r_gyr·|ω_n| / v_avg     T (4.207), (4.208)
    C_v² + C_ω² = 1                                                    T (4.211)

so the force and the torque share one friction budget: pure sliding gives μ_C·f_n and no torque,
pure spinning gives μ_C·f_n·r_gyr and no force, whatever the speed.

Stick–slip:

    s = exp(−v_avg² / v_S²)                             T (4.221)
    C_v,s = s + (1 − s)·C_v,   C_ω,s = s + (1 − s)·C_ω  T (4.223), (4.235)
    C_v = |v_ct| / max(v_avg, v_ε),  C_ω = r_gyr·|ω_n| / max(v_avg, v_ε),  v_ε = v_S/100
                                                        T (4.224), (4.236), §4.4.4
    sliding targets:  μ_C·f_n·C_v,s  and  μ_C·r_gyr·f_n·C_ω,s          T (4.225), (4.237)
    stick limits:     μ_max·f_n·C_v,s  and  μ_max·r_gyr·f_n·C_ω,s      T (4.228), (4.240)
    bristles: force f_n·(σ_o·z + σ_1·ż), torque f_n·r_gyr²·(σ_o·θ_n + σ_1·θ̇_n)
                                                        T (4.219), (4.233)
    σ_o = μ_S / δz, with δz the pre-sliding displacement               T (3.46)
    viscous terms: −f_n·σ_2·v_ct and −f_n·r_gyr²·σ_2·ω_n               T (4.230), (4.243)

In the code: v_S = `v_stick`, μ_C = `mu_dynamic`, μ_max = `mu_static`, δz = `preslide_travel`,
σ_1/σ_o = `preslide_time`. The load-proportional bristle the repository adopted on 2026-11-10
(k = μ_s·|f_n|/d, c = k·τ) is therefore Gonthier's own bristle, f_n·σ_o with σ_o from (3.46); the
torsional bristle has k·r_gyr² and c·r_gyr², as (4.233).

The friction velocity is evaluated at p_c (T: v_sct, at the centroid of the contact surface, "can
be substituted by v_ct without affecting the friction model behaviour significantly", after
(4.75)), or at a body point if `FrictionPoint` says so.

### Departures from the publication in `kPublished`

1. **Stick–slip regularization.** T integrates a bristle state with an ODE: ż = s·v + (1 − s)·ż_sl,
   re-computed on saturation (T (4.227), (4.229), (4.239), (4.242)), and the friction force is the
   saturated bristle force alone. The code keeps the blend it already had (V18, after Dopico et
   al. 2011): F = s·F_stick + (1 − s)·F_slide, with F_slide the sliding target above and F_stick a
   spring–damper on the bristle deflection z, scaled back to the stick limit when it exceeds it.
   Both forms give the published sliding target for v_avg ≫ v_S and the published bristle with the
   published limit for v_avg ≪ v_S; they differ in the transition, and the code needs no time step
   or extra ODE state. The total stick force (spring + damper) is capped at the stick limit, as
   since 2026-11-10.

   **The bristle state** (`BristleFriction::deflection`, since 2026-11-11). T's z integrates the
   relative velocity of the two bodies at the contact point, ż = v_ct while sticking: a material
   quantity. `kIntegrated` (the default of `kPublished`) does that without a time step: z grows
   by the displacement, relative to B, of the material point of A that is at the friction point,
   from one evaluation to the next (stored in B, with the pose of A in B); the damper acts on
   v_ct, as σ_1·ż. The torsional bristle was already integrated this way. Until then the code
   used V18's bristle in both formulations, now `kAnchored` and kept by `kThesis2018`: the
   distance from the friction point to an anchor fixed in B. With the friction point at the
   centroid of the overlap that is not T's z: the centroid is not a material point, and it moves
   whenever the overlap changes shape. For a flat contact of width a and depth δ that rocks by θ
   the centroid moves a²·θ/(12·δ): 2 mm for a = 0.4 m, δ = 60 µm, θ = 10 µrad, twenty times the
   pre-sliding travel, with nothing sliding. A body that tilts or rocks therefore saw its bristle
   saturate and was pushed along (`tower`: 135 m in 10 s at every step; `stack`, `push`, `tip`,
   `twoblocks`, `wedge`, `hole`: creep). See `docs/history.md`, stability entry.
2. **No dwell time.** μ_max = μ_S at all times (T (3.36)–(3.37) with τ_dw → 0). Not implemented.
3. **Viscous term.** `mu_viscous` is a plain coefficient [N·s/m], not σ_2·f_n; zero by default.
4. **Direction function.** T's relaxed direction `dir(v, v_ε)` (3.33) is replaced by v/|v| (zero at
   v = 0): below v_ε the sliding term has weight 1 − s < 10⁻⁴.
5. **Normal.** T takes n as the eigenvector of J closest to a geometric estimate (T §4.3.7), which
   makes J·ω_n parallel to n. The repository's n is the vector-area normal of the overlap surface
   (see `intersection_properties.hpp`); the code uses the scalar forms (4.208), (4.233) along that
   n, so the spinning torque is always parallel to n.
6. **Overlap properties.** T approximates V and J with voxels; the repository computes them exactly
   from the meshes. J is the volume inertia J_v, which T also uses in place of the surface inertia
   J_s (J_v·ω_n = J_s·ω_n exactly, T after (4.47)).

### What the published model predicts (closed forms used by the tests)

- Pure spin (v_ct = 0, r_gyr·|ω_n| ≫ v_S): |τ| = μ_C·f_n·r_gyr, independent of ω_n. A disk of
  inertia I spinning at ω₀ stops at t = I·ω₀ / (μ_C·f_n·r_gyr), decelerating linearly.
- For a flat circular patch of radius R with uniform depth, r_gyr = R/√2, against the exact Coulomb
  integral (2/3)·R for uniform pressure: the published torque is 3/(2√2) = 1.061 times the exact
  one. This 6 % is a property of the published model (the mean-value step that replaces |v_t| by
  one v_avg), not of the implementation. T §5 itself reports "a spinning friction torque about 35 %
  greater" than Leine and Glocker's set-valued Coulomb–Contensou law in the Tippe-Top example.
- Combined motion: |f_t| = μ_C·f_n·|v_ct|/v_avg and |τ| = μ_C·f_n·r_gyr²·|ω_n|/v_avg, so
  (|f_t|/(μ_C·f_n))² + (|τ|/(μ_C·f_n·r_gyr))² = 1.

## Equations implemented, `kThesis2018`

    τ_s = −μ_spin·(f_n / V)·J·ω_n            V18 eq. "spinning friction"; P05 (55), P09 (2)
    F_t = s·F_stick + (1 − s)·F_slide − μ_visc·v_t,   s = exp(−|v_t|² / v_stick²)
                                              V18, after Dopico et al. 2011
    F_slide = −μ_d·f_n·v_t/|v_t|

P05 (41) defines the friction element as df_t = μ·v_t·df_n and says that μ "will be defined in the
model implementation": "if the normal force is kept constant, then μ will be a viscous friction
coefficient, while if the magnitude of v_t is set to one, then μ would represent either the static
or kinetic friction coefficient". Read with a constant μ, (55) is a viscous torque, proportional
to ω_n, and `mu_spin` has units of s/m. This is what V18 implemented and what `kThesis2018`
keeps; the Coulomb specialisation is T §4.4, i.e. `kPublished`. There is no torsional bristle and
the spinning torque does not interact with the sliding force.

## Fixed in both formulations (2026-11-11)

- A bristle loaded at exactly zero slip speed keeps its force (it returned zero before).
- No rolling-resistance torque while the normal force is clamped at zero.
- When the stick spring saturates while the slip runs against it (the bristle is unloading and
  the limit has dropped), the anchor is pulled in along the bristle. It was placed behind the point
  along the slip velocity, which flipped the spring from one limit to the other.

## Single runs (Chrono, `base`, relative error of each scenario's metric)

| Scenario | Before (thesis form) | `kPublished` | `kThesis2018` after the fixes |
|---|---|---|---|
| `disk` | 0.344 | 0.056 | 0.344 |
| `contensou` | 0.129 | 0.074 | 0.134 |
| `ring` | 0.031 | 0.009 | 0.031 |
| `cone` | 0.091 | 0.039 | 0.091 |
| `bearing` | 0.0142 | 0.0134 | 0.0139 |
| `block` | 0.0042 | 0.0042 | 0.0042 |
| `push` | 0.0201 | 0.0203 | 0.0200 |
| `twoblocks` | 0.00079 | 0.00084 | 0.00032 |
| `billiard` | 0.060 | 0.060 | 0.060 |

`disk` lands on the 6 % the published model predicts for a flat circular patch. The whole battery
has not been rerun.
