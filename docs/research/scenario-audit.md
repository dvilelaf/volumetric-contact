# Audit of the 20-scenario battery as a scientific instrument

Reviewer: independent (mechanics). Read-only on the repo at commit `fe4684c`. Sources read: `AGENTS.md`,
`README.md`, `docs/history.md`, `docs/scenarios.md`, `scenarios/src/*.cpp`, `apps/bench.cpp`,
`tools/stats.py`, both engine backends, `results/battery.csv`, and the per-run scalars stored in
`results/traj/**.json` (dumped to `/tmp/vc-research/scalars.txt`).
Own checks: `/tmp/vc-research/checks.py` (derivations), `/tmp/vc-research/stats2.py` (re-analysis of
`battery.csv`), six single Drake-native runs on a refined copy of the box meshes (`/tmp/vc-research/data2`,
outputs `out2/`, `out3/`), and a 40-line Chrono program (`/tmp/vc-research/cc/t.cpp`) run in the project image.
Nothing in the repository was changed.

## 0. Verdict in one paragraph

Every formula in `docs/scenarios.md` §4 matches its code and is algebraically correct **under its own
assumptions**. The problem is the assumptions. Only 9 of the 20 theories are model-independent rigid-body
results; 5 presuppose a pressure distribution (uniform or Winkler cosine); 6 are the Winkler law's own
prediction (penetrations, tilt, tipping angle, the law's own impact ODE, the law's own interference force).
In addition, the two native contacts are configured in ways their developers would not accept: Drake's rigid
boxes have 12 triangles and rest on a half-space, which gives **two contact quadrature points** under each box
(refining only the box mesh takes Drake native from 13 % to 1.4 % in `block`, 113 % to 1.0 % in `offset`,
2.0 % to 0.1 % in `tip`); Chrono's hull–hull collision gives **one contact point** for a face-on-face contact
and `SetContactStiff(true)` has no effect in the build. Several metrics do not see the failure their scenario
is about (35 mm of creep in `push`, 98 mm of sliding in `tip`, 50 mm of creep in `stack`, 62 % amplitude loss
in `trough` all score ≤ 2 %). The statistics treat four non-independent variants as replicates and twenty
correlated scenarios as independent blocks. The headline claim ("at least as accurate as the engines' own
contact") is **not supported as stated**; what the data do support is narrower and is listed in §6.

## 1. Table: one row per scenario

Classes: **A** = model-independent rigid-body result (any contact model must converge to it as K → ∞);
**B** = needs an assumed pressure distribution; **C** = the Winkler law's own prediction (depends on K, or
integrates the law's own force).

| # | Scenario | Formula correct? | Class | Model-independent? | Metric OK? | Action |
|---|---|---|---|---|---|---|
| 1 | `block` | yes | A | yes | yes, but error is amplified 6.5× with respect to μ | keep; report effective μ |
| 2 | `disk` | yes (uniform p) | B | **no**: τ/(μNR) = 0.667 uniform, 0.785 rigid punch on a half-space, 0.5 uniform wear | stop time fine | keep, label B; prefer `ring` for cross-model ranking |
| 3 | `cylinder` | yes | A | yes in the rigid limit; the law's own damping gives rolling resistance of the size of the measured error | final v accumulates a legitimate model effect | add K and h sweep; measure early or with h → 0 |
| 4 | `push` | yes | A | yes | **no**: final v hides 30–35 mm of creep before onset (Gonthier, Chrono native) | error = max(rel. v error, onset-time error, creep/L) |
| 5 | `tip` | yes (Winkler) | C | **no**: θ* = 16.2° is K-dependent; a perfect rigid solver scores 13.8 % | **no**: hides sliding (Gonthier slides 98 mm and scores best) | label C; add slide to the error; add a rigid-limit version (K sweep) |
| 6 | `drop` | yes (law's ODE) | C | **no**: the theory is the law integrated with RK4 | rebound fine | label C (integration test); for natives define the scenario by a target restitution |
| 7 | `billiard` | yes | A | yes at rolling onset; after it the law's damping decelerates the ball: predicted 4.52 %, measured 4.51 % | **no**: "final velocity at 1 s" measures h, not the law | measure v at rolling onset (or end at 0.35 s); fix the initial condition for natives |
| 8 | `contensou` | yes (verified by 2-D quadrature: 0.45179 s) | B | **no**: uniform pressure | fine | keep, label B |
| 9 | `vgroove` | yes | A | yes | yes | keep |
| 10 | `ring` | yes | B (weak) | nearly: uniform pressure vs uniform wear differ 2 % | fine | keep; state the 2 % band |
| 11 | `hole` | yes (law's force) | C | **no**: 4μKδa·z is the law's interference force | final depth; was once right by accident (history) | label C; replace for ranking by a clearance peg (wedging) test |
| 12 | `bowl` | yes | A | yes, but R − r = 5 mm is smaller than what the test can resolve | **no**: period ignores 14–80 % amplitude decay | redesign (R − r ≥ 50 mm); add amplitude to the error |
| 13 | `trough` | yes | A | as `bowl` | **no**: Gonthier keeps 38 % of the amplitude and scores best (0.15 %) | as `bowl` |
| 14 | `bearing` | yes (Winkler cosine, zero clearance) | B | **no**: τ/(μN_res r) = 1.00 line contact, 1.27 cosine, 1.37 √cos, 1.57 uniform | fine | keep, label B; add a clearance version (factor 1, class A) |
| 15 | `corner` | **wrong for μ = 0.5** (frictionless theory, friction-indeterminate statics) | C | no | **no**: any N_x/mg in [0, 1.36] is a valid equilibrium | μ = 0 and label C, or measure reactions |
| 16 | `offset` | yes (Winkler) | C | **no**: tilt = load/(K·I_A) | fine | label C; a model-independent observable is the centre of pressure (= e) |
| 17 | `twoblocks` | yes | A | yes | **no**: lower-block velocity hides 7–10 mm of relative slip | error = max(rel. v error, slip/L) |
| 18 | `stack` | yes (Winkler) | C | **no**: compressions = load/(K·A) | **no**: ignores creep (7.5 mm distributed, 50 mm Gonthier in 3 s) | label C; put creep in the error; add a long tower |
| 19 | `cone` | yes (uniform p) | B (weak) | nearly: uniform pressure vs uniform wear differ 3.6 % | fine | keep; state the band |
| 20 | `sphere` | yes | A | yes in the rigid limit; same rolling-resistance bias as `billiard` (predicted 4.3 %, measured 3.1–4.7 %) | as `cylinder` | as `cylinder` |

Count: A = 9, B = 5 (2 of them weakly), C = 6. In class C only Winkler-type laws can score well by
construction; Drake's hydroelastic contact with E = K·H is itself a Winkler pressure field (§5), Chrono's
Hertz SMC is not.

## 2. Theory, scenario by scenario

### 2.1 Class A (rigid-body results)

- **`block`**: a = g(sin θ − μ cos θ) = 0.657 m/s²; v(4 s) = 2.629 m/s. Correct. Because θ = 30° is close to
  atan μ = 26.6°, d ln v / d ln μ = −6.46: a 0.5 % error means the effective μ is right to 0.08 %.
  The same quantity is −0.96 in `vgroove`, −0.53 in `twoblocks`, −3.17 in `push`. Relative errors are
  therefore not commensurable even among the sliding scenarios.
- **`push`**: t* = μmg/c = 2.4525 s, v(4 s) = c(t − t*)²/(2m) = 2.395 m/s. Correct.
- **`vgroove`**: each face carries mg cos θ/√2, friction √2 μ mg cos θ, a = 2.502 m/s². Correct.
- **`twoblocks`**: a = g(sin θ − μ₁ cos θ) = 3.206 m/s²; interface friction needed μ₁ m₂ g cos θ < μ₂ m₂ g cos θ.
  Correct.
- **`cylinder`** (2/3 g sin θ; needs μ ≥ tan θ/3 = 0.089), **`sphere`** (5/7 g sin θ; needs μ ≥ 0.104),
  **`billiard`** (angular momentum about the contact point: v = 5v₀/7 at t = 2v₀/(7μg) = 0.2913 s).
  Correct for a rigid body touching at a point.
- **`bowl`**: T = 2π√(7(R − r)/(5g)) = 0.16784 s. **`trough`**: T = 2π√(3(R − r)/(2g)) = 0.17373 s. Correct
  for rigid rolling without slipping at small amplitude.

**The damping of the law gives rolling resistance, and that is what the rolling scenarios measure.** A
Winkler pressure K·ℓ·(1 − h·v_n) under a rolling body is larger on the leading half than on the trailing
half. For a spherical cap (penetration δ, patch radius a, a² = 2rδ) the resisting moment is
M = K h ω ∫ℓ x² dA = N·h·ω·a²/6; for a cylinder M = N·h·ω·a²/5. With the battery's h = 5 s/m:

| Scenario | δ [mm] | predicted error from this effect alone | battery (distributed / Gonthier) |
|---|---|---|---|
| `billiard` (0.71 s of rolling after onset) | 0.56 | 4.52 % | 4.51 % / 5.98 % |
| `sphere` | 0.54 | 4.3 % | 3.06 % / 4.73 % |
| `cylinder` | 0.47 | 4.0 % | 2.14 % / 4.43 % |

All measured final velocities are below theory, as predicted. `billiard` agrees to three digits. So the
"error" of the volumetric laws in these three scenarios is the law doing exactly what its equations say;
the rigid theory just does not contain the effect. A law with h = 0 would score near 0, a law with
h = 10 would score twice as badly. These scenarios rank the damping parameter (and Drake's
Hunt–Crossley d = h gives the same effect), not the contact law. Fix: measure before the effect
accumulates (`billiard`: v at rolling onset), or run a K and h sweep and report the extrapolation to
the rigid limit.

**`bowl` and `trough` cannot resolve what they test.** R − r = 5 mm. For the bowl the conforming contact has
R_eff = 2.1 m, so δ = 0.12 mm and the patch radius is 22.6 mm; the mesh sag is 0.12 mm (sphere, inscribed) plus
0.13 mm (bowl, circumscribed). Penetration and sag both increase the radius of the centre's path, by
0.12–0.25 mm, i.e. the period by +1.2 to +2.4 %; the measured errors are +1.9 % (distributed) and +2.5–3 %
(Gonthier), all with the period longer than theory. The motion itself is 0.5 mm of centre travel and 10.5 mm
of arc on the bowl, which is one facet (10.3 mm) and half a patch radius. The scenario measures mesh and
compliance offsets on a 5 mm lever arm, and its "rigid point contact" theory does not apply to a patch that is
13° of the sphere. Redesign with R − r ≥ 50 mm (e.g. R = 0.3 m, T ≈ 1.06 s) and a travel of many facets.

**Initial conditions favour Winkler laws.** `billiard` and `sphere` start at the Winkler static penetration
(cap volume, 0.56 mm). For Chrono's Hertz contact with E = 1e8 Pa the static penetration is 0.056 mm, so the
ball starts with ten times its equilibrium penetration and a contact force of about 31 mg. Start every body
at zero penetration and let it settle before the event, or give each method its own static state.

### 2.2 Class B (a pressure distribution is assumed)

- **`disk`**: τ = (2/3)μmgR, t = 3Rω₀/(4μg) = 0.5003 s. Correct for uniform pressure, which is what a rigid
  flat punch produces on a Winkler foundation. On an elastic half-space the rigid punch has
  p ∝ 1/√(1 − r²/R²) and τ = (π/4)μmgR (+17.8 %); uniform wear gives τ = μmgR/2 (−25 %). The "theory" is
  one point in a ±20 % band of defensible answers.
- **`contensou`**: same uniform-pressure assumption; my independent 2-D quadrature of the Coulomb disk gives
  the same stop time (0.45179 s) and ratio (0.692 at |v| = 0.1, tending to 0.653). The theory also ignores
  the pressure shift caused by friction acting below the centre of mass (μh/R = 0.06), which a Winkler law
  models and which bends the path; the effect is small here.
- **`ring`**: uniform pressure 0.20417 μmg, uniform wear 0.20000 μmg: 2 % apart. This is the spin test to
  use across models.
- **`cone`**: uniform pressure follows from axial sinking; correct, t = 0.389 s. Uniform wear differs by
  3.6 %. The sinking of 1.115 mm reported by all three Winkler-type methods against a theory of 0.931 mm is
  the mesh sag (inscribed frustum, circumscribed seat: 0.18 mm axial), not a law error.
- **`bearing`**: journal displaced by e in a zero-clearance hole: interference e·cos φ over the half
  circumference, p = K e cos φ. Normal resultant N = K e r L π/2; the tangential tractions μp sum to μN at
  right angles to it, so the contact force is N√(1 + μ²) = mg, tilted atan μ. Friction torque
  τ = μ r ∫p dA = 2μK e r²L = (4/π)μNr = (4/π)μ m g r/√(1 + μ²). The doc and the code are correct.
  But 4/π is the Winkler cosine value. With the same friction circle: line contact (any clearance, rigid)
  gives 1.00; a Hertz-like √cos distribution gives 1.37; uniform pressure over 180° (the "run-in bearing" of
  machine design texts) gives π/2 = 1.57. A clearance-free rigid journal is statically indeterminate (any
  pressure with the right resultant is admissible), so there is no rigid answer; the answer is set by the
  compliance model. Drake native reproduces 4/π to 0.4 % because it is the same pressure model.
  A journal with clearance (say 1 mm) would have the model-independent answer μ m g r/√(1 + μ²).

### 2.3 Class C (the law's own prediction)

- **`tip`**: N = K a φ c²/2 for a triangular pressure over a strip c, centre of pressure at c/3; equilibrium
  while max_φ (a/2 − h_c φ − c/3) ≥ h_c tan θ. Correct; I get θ* = 16.199° with a lean of 0.0143 rad at the
  limit. The rigid answer is 18.435°. θ* approaches it as K^(−1/3): 13.5° (K = 1e6), 16.2° (1e7), 17.4° (1e8),
  18.0° (1e9). A rigid-contact engine that tips at exactly the rigid angle would be charged 13.8 % error.
- **`drop`**: m δ̈ = mg − K A δ(1 + h δ̇) is the normal force of both laws for a flat face. I reproduce the
  rebound 0.13651 m/s (restitution 0.218 at 0.626 m/s). This is a test of time integration, not of physics.
  Chrono cannot match it by construction (its restitution is set to 0.1 by rule, and its stiffness is Hertz).
  Drake native's 8.7 % falls to 2.0 % at dt = 0.2 ms: it is a step-size error of the first-order solver.
- **`hole`**: 0.2 mm interference, p = Kδ, friction 4μKδa·z, z_stop = 2mg/k = 9.81 mm at t = 70.2 ms; the cube
  then needs mg and has 2mg available, so it holds. Correct, and entirely the law's own force.
- **`corner`**: the three penetrations m g|dᵢ|/(K a²) are the frictionless equilibrium. With μ = 0.5 the
  statics are indeterminate: a linear program over the three normal forces and the six friction components
  (force balance, friction pyramids) gives N_x/mg ∈ [0, 1.36] and N_z/mg ∈ [0.33, 1.63] against the frictionless
  0.408 and 0.816. Where the cube ends depends on the path and on the stiction model. The theory is wrong for
  the scenario as posed. With μ = 0 it becomes correct, but stays class C.
- **`offset`**: K I_A θ = m g (e + z_c θ). Correct. Model-independent content: the centre of pressure lies
  under the centre of mass. The tilt is 2.3 mrad and exists only because of K.
- **`stack`**: n·mg cos θ/(K a²). Correct, and the middle-third condition holds (0.026 < 0.033 m). Same remark.

For the C scenarios the statement "method X has error ε" means "method X is (or is not) a Winkler foundation
of stiffness K". That is a consistency check for the two laws and worth keeping as such. It is not evidence
about accuracy against other contact models.

## 3. Error metrics

**Failures the metric does not see** (base variant, from `results/traj`):

| Scenario | Run | What happened | Error charged |
|---|---|---|---|
| `push` | Gonthier (both engines) | moves at 0.035 s instead of 2.48 s; 35 mm of creep before onset | 2.0 % |
| `push` | Chrono native | 30 mm of creep | 2.5 % |
| `tip` | Gonthier | slides 98 mm before tipping (μ = 0.8, tan θ = 0.29) | 1.4 % (best of six) |
| `stack` | distributed | top cube creeps 7.5 mm in 3 s on a 5° tilt | 4.0 % |
| `stack` | Gonthier | top cube creeps 50 mm | 47 % (from compression, not creep) |
| `twoblocks` | Gonthier | 7.2 mm of relative slip | 0.08 % (best of six) |
| `trough` | Gonthier | amplitude 0.038 of 0.1 rad after 1 s | 0.15 % (best of six) |
| `bowl` | Drake native | amplitude decays 80 % | 15 % (from period) |
| `block` | Drake native | block yaws 0.13 rad | 13 % |

The scalars that would catch these are already computed (`creep`, `slide`, `relative_slip_end_m`,
`amplitude_end`, `onset_time`); they are just not in `error`. A defensible rule: `error` is the largest of
the normalized deviations of every quantity the theory fixes, each divided by a natural scale (a length of
the body for creep and slip, the theory value otherwise).

**Other points.**

- *Stop time at 1 % of the initial speed*: well posed for Coulomb decay (0.99 of the full time). It is
  sensitive to the tail for any model with a viscous-like regime near rest (regularized friction, stick
  dampers). The slope of ω(t) between 90 % and 10 %, or the total angle turned (ω₀t/2), measures the torque
  directly and is more robust.
- *Final velocity*: equals the error of the mean acceleration; mixes onset and steady state. Fine for
  `block`, `vgroove`; wrong tool for `push`, `billiard`, `twoblocks` (above).
- *Last-window averages* (`hole`, `corner`, `offset`, `stack`): fine if the run has settled; residual speed is
  recorded but not checked. A run that is still drifting is averaged as if it were static.
- *Tip detection at 0.05 rad*: the equilibrium lean at the limit is 0.014 rad and the ramp is 0.5°/s, so some
  delay is built in. It is small: Drake native on a refined mesh hits θ* within 0.1 %. The +2.3 % of
  `distributed` is therefore real (later tip), not a detection artefact.
- *Relative error across scenarios*: not comparable. Sensitivities to μ range from 0.5 to 6.5; class C
  errors are relative to 40 µm penetrations and 2 mrad tilts; `hole` under Chrono native is 2850. Ranks within
  a scenario survive this; anything that uses magnitudes across scenarios (Wilcoxon on raw differences, the
  "median error over the scenarios" quoted in the README) does not.
- *Accidental good scores*: `hole` before the stiction change (documented in `docs/history.md`);
  Gonthier in `tip`, `twoblocks`, `trough` (above). *Accidental bad scores*: `corner` for any method with
  working stiction; `tip` for any rigid-limit method; `billiard`/`sphere`/`cylinder` for any method with
  damping.

## 4. Variants and statistics

**The variants are not replicates.**

- `fine` changes nothing in the ten scenarios without curved meshes (`block`, `push`, `tip`, `drop`,
  `vgroove`, `hole`, `corner`, `offset`, `twoblocks`, `stack`): the row is a copy of `base`. `perturbed` (1 mm/s,
  0.01 rad/s) is within a few percent of `base` almost everywhere. The "median over 4 variants" is then the
  mean of two near-identical numbers; it is the base value under another name.
- Where the variants differ, they are the result, and the median hides it. `distributed` **gets worse under
  mesh refinement**: `cone` 27 % → 54.6 %, `bearing` 52 % → 66 %, `ring` 1.6 % → 3.1 %, `trough` 0.44 % → 1.03 %,
  `cylinder` 2.1 % → 2.9 %, `sphere` 3.1 % → 3.9 %, `billiard` 4.5 % → 5.5 %. Doubling the number of pieces
  doubles the excess in `cone`: this supports the seam hypothesis of open problem 1 and says the law is not
  convergent there. The median reports 27 %.
- 48 of 120 cells have a spread larger than 2× across variants or a failed variant.
- Drake native improves strongly with dt (`block` 13 % → 2.5 %, `drop` 8.7 % → 2.0 %, `push` 4.3 % → 1.6 %,
  `billiard` 11.7 % → 8.2 %): at 1 ms its error is step-size error. The median reports the 1 ms value.

**The tests.**

- *Blocks are not independent.* Five scenarios are "friction torque of a spinning conforming contact", four
  are "Coulomb sliding on planes", five are "rolling", six are "Winkler statics". The effective number of
  independent blocks is nearer 4–6 than 20. Friedman and Wilcoxon p-values assume 20.
- *Pseudo-replication across engines.* The same law on two engines gives the same number (median
  |log₁₀ ratio| 0.0005 for Gonthier, 0.0002 for distributed; Spearman 0.997–0.9985). The six-way Friedman
  therefore has three treatments, two of them entered twice; "Most accurate: chrono-distributed (2.65 vs
  2.75)" is decided by the fourth digit. The 15 Holm comparisons spend power on pairs that are identical by
  construction.
- *Ranks reward noise.* 0.12 % vs 0.26 % in `vgroove` moves a rank as much as 0.06 % vs 34 % in `disk`.
- *Wilcoxon on raw differences* weights scenarios by the size of their errors, which is set by the scenario's
  sensitivity (§3), and the inf replacement (10·(max + 1)) puts Chrono's failures at the top of the rank order.
- *The battery was also the development set.* `docs/history.md` shows the stiction model was changed twice
  and accepted on the basis of this battery's scores. The natives got one fixed mapping and no iteration.
  Scores on the battery are training error for the two laws and test error for the natives.
- *Power.* With n = 20 and k = 3 Friedman detects only large, consistent effects; the Drake result
  (p = 0.086) flips to 0.034 or 0.15 by leaving out one scenario (`bearing`, `sphere`).

**Re-analysis by class** (`stats2.py`, variant medians as in the repo, mean ranks, 1 = best):

| Engine | Subset | n | Friedman p | Gonthier | distributed | native |
|---|---|---|---|---|---|---|
| Drake | all | 20 | 0.086 | 2.00 | 1.65 | 2.35 |
| Drake | A | 9 | 0.001 | 1.56 | 1.44 | 3.00 |
| Drake | B | 5 | 0.17 | 2.60 | 2.00 | 1.40 |
| Drake | C | 6 | 0.61 | 2.17 | 1.67 | 2.17 |
| Chrono | all | 20 | 0.0005 | 1.75 | 1.55 | 2.70 |
| Chrono | A | 9 | 0.016 | 1.67 | 1.56 | 2.78 |
| Chrono | B | 5 | 0.82 | 2.00 | 1.80 | 2.20 |
| Chrono | C | 6 | 0.009 | 1.67 | 1.33 | 3.00 |

Reading: on the pressure-distribution scenarios Drake's hydroelastic contact is the best method; on class A
it is last in all nine, but four of those nine are boxes on a half-space (two-point support, §5) and the
other five carry the rolling-resistance and step-size effects. Distributed against Gonthier never separates
(sign test 8/20, p = 0.50; median error ratio 1.4, bootstrap 95 % interval 0.95–4.7). Pass rates say the same
thing more plainly: scenarios within 5 % of theory: distributed 17/20, Gonthier 13/20, Drake native 10/20,
Chrono native 7/20; within 20 %: 17, 16, 18, 11.

**What a statistician would ask for instead.**

1. No omnibus test on 20 hand-picked, correlated scenarios. Report the full table with per-class summaries
   and effect sizes (error ratios with bootstrap intervals, resampling scenarios within class).
2. Variants as experiments, not replicates: for each scenario and method a convergence plot against mesh
   (N = 32, 64, 128, 256) and dt (2, 1, 0.5, 0.2, 0.1 ms) with the observed order, and the value extrapolated
   to the limit as the reported error. Perturbations: 20–30 random draws, reported as an interval.
3. A tolerance of practical equivalence (errors below it tie), performance profiles (share of scenarios
   solved within a factor of the best), and pass rates at 1, 5, 20 %.
4. Class A, B and C reported separately; class C never used to rank against non-Winkler models.
5. If a test is wanted: paired comparison of log error ratios, one law against one native per engine, with
   scenarios clustered by mechanism (permutation test over clusters), plus a critical-difference diagram
   only as an illustration.
6. A held-out set of scenarios written before looking at the laws' results, and all parameters (d, τ, h)
   frozen before it is run.
7. Work-precision diagrams (error against wall time) instead of separate accuracy and timing tables.

## 5. Fairness of the native contacts

### Drake (hydroelastic, `kLagged`, dt = 1 ms)

- `DiscreteContactApproximation::kLagged` with the SAP solver is Drake's current default and a reasonable
  choice. Hunt–Crossley dissipation d = h matches the law's (1 + h·δ̇) form. E = K·H on compliant meshes and
  E = K·1 m on a 1 m half-space slab give p = K·depth: with a rigid counterpart this **is** a Winkler
  foundation of the same K, evaluated on the rigid surface. The mapping is fair, and it also means that
  "volumetric law vs hydroelastic" compares two discretizations of nearly the same pressure model. A paper
  must say so and position `distributed` against Elandt et al. (2019) and Masterjohn et al. (2022).
- **The rigid boxes are too coarse, and this decides several scenarios.** Bodies are registered as rigid
  `Mesh(surface.obj)`; every box has 12 triangles. Drake's user guide: "Half spaces are not represented by
  meshes ... The contact surface's refinement will depend on the other geometry's level of refinement", and
  "Getting moments out of the contact depends on how many elements are in the contact surface". In discrete
  mode each contact polygon becomes one point contact at its centroid, so a box on the half-space stands on
  two points, at (±a/6, ±b/6) on one diagonal, and has no stiffness about that diagonal. I ran Drake native
  on a copy of the data with each box triangle split into 64 (768 triangles; nothing else changed):

  | Scenario | battery (12 triangles) | refined box (768 triangles) |
  |---|---|---|
  | `block` | 13.0 % (yaw 0.13 rad) | 1.37 % (yaw 0.004 rad) |
  | `offset` | 113 % | 1.05 % |
  | `push` | 4.3 % | 1.67 % |
  | `tip` | 1.96 % | 0.10 % |
  | `drop` | 8.7 % | 8.7 % (step-size error, 2.0 % at 0.2 ms) |
  | `twoblocks` | 2.0 % | 0.78 % |
  | `stack` | 19.5 % (top creep 8.4 mm) | 9.4 % (top creep 4.5 mm) |
  | `hole`, `corner`, `vgroove` | 87 %, 9.5 %, 0.77 % | unchanged (tetrahedral grounds already refine the contact surface) |

  With these seven values in place of the battery's, Drake native is within 2 % in `block`, `offset`, `push`,
  `tip` and `twoblocks`; it is still behind `distributed` in `block`, `push` and `twoblocks` (0.3–0.5 %) and
  ahead in `tip`. `hole` at 87 % is not a mesh effect and I did not find its cause.

  A Drake developer would not accept the 12-triangle configuration; the fix is one line of mesh generation
  (or a compliant box on a rigid ground, which the guide recommends). `--native-primitives` does not help:
  the resolution hint has no effect on boxes.
- The rule "planar ground compliant, bodies rigid" is the worst case for Drake for the reason above; the
  2-colouring is otherwise sound.
- dt = 1 ms: Drake's convex solver is first order; its dt-variant errors are 2–5× smaller. A fair comparison
  gives each method the step at which it converges, or compares at equal cost.
- Speed: the rigid-vs-tetrahedral-ground cases are slow because of the volume meshes; unrelated to accuracy.

### Chrono (SMC, Hertz, convex hulls, `EULER_IMPLICIT_LINEARIZED`, default solver)

- **One contact point per hull pair.** A cube hull resting flat on a ground hull reports `GetNumContacts() = 1`
  at every step (`cc/t.cpp`). A face contact carried by one wandering point has no moment support; this is the
  likely cause of the poor results in `offset`, `stack`, `corner`, `ring` and `disk` (not verified run by run). Chrono
  users build such models with box, cylinder and sphere primitives or triangle meshes, which give contact
  manifolds; nobody would accept hull–hull as "Chrono's contact".
- **`SetContactStiff(true)` does nothing here.** The comment in `chrono_engine.cpp` says it enables the force
  Jacobians ("Chrono's most accurate smooth-contact setting"). In the image the system descriptor never
  holds a KRM block and the trajectories are identical with the flag on or off and with PSOR, MINRES or
  SPARSE_QR (`cc/t.cpp`). The default solver, PSOR, does not support stiffness matrices in any case (Chrono
  docs: "For problems that involve a stiffness matrix: GMRES, MINRES"). So native Chrono contact is
  integrated explicitly at 1 ms. This is probably part of open problem 6 and should be checked first.
- **E = K·1 m into a Hertz law is arbitrary.** Chrono's force is (4/3)E*√(R_eff δ)·δ per contact point with
  R_eff = 0.1 m by default for hulls (it happens to equal the sphere radius used in three scenarios, and is
  wrong for every other shape). There is no K for which a Hertz point force reproduces K·V, so every class C
  theory is out of reach by construction (`drop` 186 %, `stack` 74 %, `offset` 40 %, `corner` 613 %).
- **Restitution 0.1** is a rule with no relation to h = 5 s/m; `drop` needs 0.218 at its impact speed.
- **Initial conditions** at the Winkler static penetration (see §2.1).
- A Chrono developer would configure: primitives or meshes, explicit normal stiffness and damping
  (`UseMaterialProperties(false)`, Hooke, kn and gn chosen to match the scenario's load–penetration scale and
  restitution), a step of 1e-4 s or less for E = 1e8, and would point out that Chrono's standard answer for
  rigid bodies is NSC (complementarity, APGD or Barzilai–Borwein), which is absent from the comparison.

### Does the theory choice disadvantage the natives?

Yes for Chrono in all six class C scenarios and in `bearing`; no for Drake, which is a Winkler model too.
The parameter mapping disadvantages both: Drake through mesh resolution, Chrono through collision shapes,
stiffness law and restitution. The conclusion "on Chrono both volumetric laws are significantly more
accurate than its SMC contact" is a statement about this configuration of Chrono, not about Chrono.

## 6. What the data support, and what they do not

Supported:

- Both laws give the same numbers on both engines: the implementation is engine-independent.
- `distributed` reproduces every Winkler-foundation result on flat contacts to well under 1 % (`offset`,
  `disk`, `contensou`, `push`, `drop`), where the single-force Gonthier law fails on spin and multi-face
  statics. This is the solid, publishable result: distributed against Gonthier, on class B and C.
- `distributed` is wrong, and not convergent under refinement, on curved conforming contacts made of
  pieces (`cone`, `bearing`).
- A correctly meshed Drake hydroelastic contact is as accurate as `distributed` on flat contacts (1 % level)
  and better on curved conforming ones; the volumetric laws are much faster on non-planar grounds.

Not supported:

- "At least as accurate as the engines' own contact" and any ranking against Chrono: the natives were not
  configured fairly and a third of the theories are the law's own.
- "`distributed` is the best law overall" as a statistical statement: it does not separate from Gonthier
  (p = 0.10–0.17) and its rolling-scenario scores reflect h.
- Any statement from the six-way Friedman test.

## 7. Coverage gaps and prioritized changes

### Changes to the existing battery (in order)

1. **Fix the native configurations and rerun**: refined rigid meshes for Drake (or compliant bodies on a
   rigid ground); primitives or meshes, explicit stiffness, matched restitution, working implicit
   integration or a smaller step for Chrono; add Chrono NSC and Drake point contact as baselines.
2. **Tag every scenario A, B or C** in `docs/scenarios.md` and in `battery.csv`; report by class; rank against
   natives on class A only (plus B with the band of admissible answers stated).
3. **Put the hidden failures in the error**: creep and onset in `push`, slide in `tip`, slip in `twoblocks`,
   creep in `stack`, amplitude in `bowl` and `trough`, residual speed in the static scenarios.
4. **`corner`**: set μ = 0 (the theory then holds; tag C). Better, add contact reactions to the trajectory
   output and compare forces, which are model-independent; with friction, replace the scenario by a
   determinate one (a wedge that must self-lock iff tan α < μ; a ladder that slips at a known angle).
5. **Rolling scenarios**: `billiard` measured at rolling onset; `cylinder` and `sphere` with a K and h sweep
   and the rigid-limit extrapolation reported; `bowl` and `trough` redesigned with R − r ≥ 50 mm.
6. **Initial conditions**: zero penetration plus a settling phase for every method.
7. **Variants**: replace `fine`/`dt`/`perturbed` medians by convergence series and report the limit; drop
   `fine` where it changes nothing.
8. **`tip`, `hole`, `drop`, `offset`, `stack`**: keep as law-consistency tests (class C); for `tip` add the K
   sweep towards the rigid angle; for `drop` define the natives by the target restitution.
9. **`bearing`**: add a version with clearance (answer μmgr/√(1 + μ²), class A).
10. **Statistics**: as in §4; delete the six-way test.

### Missing regimes for a paper asking "better than existing methods, always, when?"

Minimal set, most valuable first:

1. **Decomposition invariance** (new, class A, targets open problem 1 directly): the same convex body or
   ground split into 1, 2, 8, 64 convex pieces must give the same result. No theory needed.
2. **Convergence under refinement and stiffness sweep** for every scenario (the instrument's own
   calibration; shows where explicit coupling at 1 ms breaks).
3. **Impact with restitution**: sphere dropped on a plane over a range of speeds (restitution curve of the
   law against the Hunt–Crossley closed form), and an oblique impact with friction (rigid impulse theory
   gives the rebound angle and spin for sliding and for sticking impacts).
4. **Edge and vertex contacts**: a cube landing on an edge and on a corner, a box sliding off a table edge,
   a box tipping over a step. The pair normal of a volumetric law is least defined here.
5. **Wedging and jamming** (class A, strongly discriminating): self-locking wedge, peg in a hole with
   clearance (Whitney's jamming and wedging conditions), ladder.
6. **Rolling with spin and no slip in 3-D**: ball on a turntable (orbit rate 2/7 of the table's), rolling
   coin; exact rigid results that test pivoting friction.
7. **Long stacks and piles**: a 10-cube tower held for 10 s on a tilt (drift), a 1:1000 mass-ratio stack,
   a small pile settling (many simultaneous pairs, energy decay).
8. **Energy audit**: frictionless, undamped versions (sphere sliding in the bowl, T = 2π√((R − r)/g)) with
   energy drift reported for every run.
9. **General non-convex meshes** through CoACD (a gear pair or a mug on a hook), compared between methods
   and under remeshing, with mesh-quality perturbations (slivers, non-uniform triangles).
10. **Thin bodies and fast motion**: a plate dropped on a plate, a coin; tunnelling at 1 ms.
11. **Grasping**: parallel jaws holding a box under gravity (slip at F = mg/(2μ)) and under a twisting
    load (soft-finger limit surface; class B).
