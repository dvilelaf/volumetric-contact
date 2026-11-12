# Referee report 1

**Manuscript:** "An analytic benchmark for conforming contact: volumetric and elastic-foundation laws on exact mesh intersections against the native contact models of Drake, Chrono and MuJoCo" (Multibody System Dynamics; sn-jnl, 70 pages)

**Reviewed against:** repository at `int/paper` (HEAD 58728ba), `results/battery.csv` (sha256 48dece37…, 11 640 rows), `results/terms.csv`, `results/stats.json`, `results/timing.csv`, `paper/generated/*`, `core/src/*.cpp`, `docs/*`.

**Independent checks:** scripts in `/tmp/vc-review/scripts/`. `recompute.py` recomputes the statuses, pass rates and engine independence. `tables.py` prints the per-scenario errors. `ratios.py` recomputes the paired ratios and stratified bootstrap and runs sensitivity variants. `fan.py` tests the quadratures on the disk triangulation. I also ran `paper/checks/check_polar_moments.py`, and every check passed. Crossref and DataCite were queried for 51 bibliography entries.

---

## 1. Recommendation: **major revision**

The manuscript describes a careful and unusually candid piece of work. Its main product is a battery of 33 contact scenarios with analytic references, classified by what each reference assumes, with documented native configurations and open data. That product would be useful to this journal's readers. The arithmetic of the paper is almost entirely reproducible: nearly every generated number I recomputed from `results/battery.csv` matches `paper/generated/numbers.tex`. The paper cannot be accepted in its present form, for four reasons.

1. **The inferential summary does not survive reasonable alternative treatments of the same data.** The headline statements are in the abstract, §7.7, §8 and Conclusion 2. They say that on class A "neither law differs significantly from any native model", and that on A+B `distributed` is not significantly different from hydroelastic contact, SMC, NSC or MuJoCo soft. These statements are driven by one scenario. `massratio` enters the class A log-mean at about 6.8 decades: the error of the laws is 6.4×10⁵ % against a floored 0.1 % for the natives. Failed native runs, by contrast, are dropped, not counted. With `massratio` left out, or with every error capped at 100 % and failures counted as 100 %, several "non-significant" comparisons become "significant" (Table R1 below). There are about 66 intervals and no multiplicity control. The statistic is therefore not informative, and the paper's prose leans on it.
2. **Fairness and selection effects are not controlled.** Several design choices and native configurations favour the laws or penalise particular natives:
   - The stiffness K of every scenario was chosen so that the *explicit* laws are stable at 1 ms (§5), which keeps the battery 3–5 orders of magnitude softer than engineering materials.
   - Several class A references cannot be met by any damped compliant model at the chosen K and h (`bowl`, `trough`, `turntable`, `edge`), so they measure parameter choices, not contact laws.
   - Drake point contact gives boxes corner spheres but gives disks one point (§6.6, §7.1: "one point cannot hold a flat face"), which is a configuration choice, not "a property of the model".
   - Chrono NSC's base runs on `disk` and `turntable` are favourable draws from a very wide distribution. The paper nevertheless quotes NSC's 5.2 % on `turntable` as the evidence for rigid-contact solvers (§8 (ii), Table 15).
   - The laws were tuned on 21 of the 33 scenarios and changed after the first run of the held-out 12. The paper says so in the body, but not in the abstract.
3. **The one claimed methodological novelty is modest, and its demonstrated benefit depends on the triangulation.** The closed-form Coulomb wrench for a linear pressure on a triangle (§4.4, App. A) is correct. It is, however, a direct extension of Jia's uniform-pressure polygon result, using the same primitives. The "one to two orders of magnitude" (abstract, Conclusion 4) is measured against quadratures applied to flat faces that the mesh generator fans from a *rim vertex* (`tools/meshgen.py` `fan()`, `loft(..., caps=True)`). On a centre fan of the same 64-gon, the one-point rule is within 0.14 % of the exact torque; on the rim fan it is off by −17.8 % (my `fan.py`). The paper attributes the error to "a fan of long triangles around the centre of rotation" (§7.8), which misdescribes the mesh. Contribution 3, engine independence, mostly shows that three engines integrate free rigid bodies under explicit forces with the same semi-implicit Euler step. It cannot, by itself, "separate the error of a law from the error of an integrator".
4. **Defects in the laws themselves remain unexplained, and the manuscript is not in a submittable state.**
   - `gonthier` gains 3.1 % energy in a frictionless, undamped scenario, and its result changed from 0.16 % to 3.1 % between code versions for a reason the authors could not find.
   - The flat disk rocks about 60 times more at K×10 (Conclusion 6 calls these "two defects that we could not explain").
   - The paper is 70 pages long, with heavy repetition and duplicated tables and figures.
   - It still contains `[TODO]` markers (acknowledgements, AI-use statement, funding, competing interests, CRediT, DOI), a placeholder repository URL (`github.com/OWNER`) and a provisional author list that the co-authors have not accepted (`main.tex:21-24`).

---

## 2. Claims audit

### 2.1 Numbers recomputed and found correct

Recomputed from `results/battery.csv` with my own code (base variant):

- **Run accounting:** 11 640 runs; 11 467 ok, 145 nan, 17 timeout, 11 failed. The laws have one not-ok run, the natives 148, the other library methods 24.
- **Engine independence:** gonthier spread 0.0205 pp and distributed 0.0268 pp, both largest on `block`. The largest relative difference is 4.2 % (`block`), the median 5.55×10⁻⁵, with 32 scenarios held and 1 lost. These match `independence/worst = 0.03 pp`.
- **Pass counts** (Table 9): all entries for G, G18, D, PCM and the six natives match, for example D on A at 12/16/19 and Drake point on A at 12/13/17.
- **Class A, B and C errors** cited in §7.3–§7.6: block, cylinder, push, twoblocks, sphere, vgroove, billiard, bowl, trough, journal, ring, bearing, disk, contensou, cone, hole, stack, tip, offset, wedge, oblique, edge, turntable, energy, tower and massratio all match the `err/*` keys.
- **Held-out pass counts:** D 8/11 and G 6/11 on the held-out class A scenarios; D 8/10 and G 8/10 on the development class A scenarios.
- **Ratios:** my bootstrap with another seed reproduces Table 10 to the second digit, for example D/Drake point A+B at 0.31 (0.09–0.87) against the paper's (0.089–0.84).
- **Convergence counts** (Table 12), the massratio dt series (24 % at 0.2 ms and 16 % at 0.1 ms on all three engines) and the energy dt and mesh series of gonthier (3.26, 3.06, 1.44, 0.61, 2.82; 5.1, 3.1, 0.07) all match.
- **Cost medians** (Table 14) match `timing.csv`.

### 2.2 Numbers or statements that do not match

| # | Location | Statement | What the data or code say |
|---|---|---|---|
| N1 | §9 "Explicit coupling" | "a body of mass m held between two flat contacts of area A needs dt ≤ √(m/(2KA))" | With the paper's own condition K·A·Σw·dt² < 2 and Σw = 2/m, the bound is dt < √(m/(KA)). For massratio (KA = 1.6×10⁷ N/m, 1 kg) that is 0.25 ms, which is also the value in `docs/history.md`. The printed formula gives 0.18 ms. |
| N2 | §7.6 "Energy" | "Every native model but Chrono SMC has an error of 24 % or more here" | Drake point: 23.6 %. This number was typed by hand, against the repo rule "no number typed by hand"; §7 contains many such numbers ("0.4 %", "0.6 %", "7 %", "24 %", "0.2 %", "0.5 %", "0.7 %"). |
| N3 | Table 10 against §7.8 | Table 10 bolds Distributed/vertex rule on class A (0.57, 0.30–1.00); the text says "with an interval that touches 1" | `stats.json` gives an upper bound of 10^−0.0012 = 0.997 < 1. By the paper's rule it is "significant", so the text and the table contradict each other. The result also rests entirely on `pieces2/8/64` (see R1). |
| N4 | §7.2 | "in massratio at the base step the bodies fly apart, and where they end depends on the engine" | For `distributed` the three engines give 638 557, 638 662 and 638 561 %, a relative spread of 1.6×10⁻⁴. The engines agree; only the absolute spread in pp is large. |
| N5 | §7.11 | Base errors are "representative ... for the laws and for hydroelastic contact, and less so for the point-contact models" | MuJoCo soft, a point-contact model, has a perturbation sensitivity of 1.7 %, the same as hydroelastic (1.6 %) and below `distributed` (4.3 %). |
| N6 | §8 (ii), Table 15 | "a rigid-contact solver keeps the orbit within 5.2 %" (NSC, `turntable`) | NSC `turntable`: perturbations 5.2–81 % (median about 23 %); dt series 70 % (2 ms), 117 % (0.5 ms), 25 % (0.2 ms), 21 % (0.1 ms); mesh variants 20.5 %. The base value is the best of 16 runs. Drake point (1.2–17 %, stable around 6–7 % under perturbation) would be a defensible example. |
| N7 | §7.9, §8 (iii), Table 15 | "Drake's hydroelastic contact, Chrono NSC and MuJoCo, whose solvers are implicit in the contact forces, hold it"; "A solver that is implicit in the contact forces" is recommended | Drake *point* contact, which uses the same discrete solver as hydroelastic, loses `massratio` (6.8×10³ %) and `tower` (2.3×10⁵ %). The paper never mentions this. Implicitness is not sufficient, and the recommendation is not supported as worded. |
| N8 | §6.4 | "The native models, in contrast, received one configuration and no iteration on scores" | §6.6 and `docs/history.md` (2026-11-11 entries) show that native options were chosen from errors on the battery: Drake refinement to L/8 (`block` 13 → 1.4 %), corner spheres (`block` 98 → 0.1 %), MuJoCo tolerance and reference area (`stack` 33 → 0.16 %), NSC solver choice. The sweeps were generic, but they were score-driven iteration on the development set. |
| N9 | §7.8 | "its flat face remains a fan of long triangles around the centre of rotation" | The caps are polygon fans from their first vertex, which lies on the rim (`tools/meshgen.py:94`, `loft`). See R2. |
| N10 | §4.1 and §4.3.3 (also `core/src/distributed.cpp:34-35`) | "v_n is the rate at which the column lengthens"; "−n·v is the rate at which the column shortens" | With Table 1's convention (v_n = n·v > 0 when separating, n pushes A out of B), separation *shortens* the column: dℓ/dt = −n·v. Both sentences have the sign reversed. The formula g = 1 − h·n·v is correct and matches the code. |
| N11 | §4.2 | the centroid moves by a²θ/(12δ), "which is millimetres for microradians" | With a = 0.4 m and δ = 60 µm the shift is 0.22 mm per µrad, so millimetres need tens of µrad (`history.md` uses 10 µrad → 2 mm). |
| N12 | §4.5 "Limits" | in gonthier "the limits of the force and of the torque are μ_sF_n and μ_sF_n r_g times the shares of Eq. (gonthier-friction)" | The code (`gonthier.cpp`, `coupling()`), like Gonthier's thesis (C_{v,s} = s + (1−s)C_v), uses blended shares that tend to 1 at stick. The static limit is therefore a box, which can hold √2 times the ellipse. The paper criticises exactly this for `distributed` ("would let a patch hold up to √2 times what its friction can") without saying that `gonthier` has it. |

### 2.3 Overclaims and unsupported or misleading statements

- **Abstract, "On class A neither law differs significantly from any native model".** This is an artefact of how `massratio` and failures enter the ratio (R1). The defensible statement is a per-scenario one: the laws have fewer large errors except where the explicit coupling fails.
- **Abstract and Conclusion 2, "point contact is as accurate where a body slides or rolls on one plane".** This holds for Drake point and Chrono NSC (below 0.4 % in 5 of 7). It does not hold for MuJoCo soft (block 7.8 %, twoblocks 8.2 %, billiard 7.1 %), MuJoCo default (push 46 %, twoblocks 13 %) or Drake point on `vgroove` (7.0 %). Name the models.
- **Abstract and Conclusion 3, "point-contact models are off by 1.8 % to 97 %" contrasted with "within 1.6 % and 2.9 %".**
  - The lower end, Chrono SMC on `ring` at 1.8 %, is better than hydroelastic contact's 2.9 % on `cone`.
  - The nominal class B reference *is* the rigid-body-on-Winkler distribution (§6.1). For `bearing` (cosine) and `cone` the agreement of two Winkler discretizations with it is partly by construction.
  - The discriminating quantity is the band. The battery stores unsigned errors, so 18 of the 50 class B cells carry "may be outside" asterisks. Every D and hydroelastic error on `ring` and `cone` is asterisked, because the narrow side of those bands is 0.
  - The sign is available from the stop time. Record it and report in-band and out-of-band counts.
- **Abstract, "12 were written before any model was run on them".** The abstract should also say that the laws were changed after the first run of those 12, so their held-out errors are not out-of-sample (§6.4, §7.6 and §9 say so).
- **Contribution 3 and §5 "If the same law gives the same result on engines that share no code, what is measured is the law".** All three engines apply v⁺ = v + dt·M⁻¹f(x, v), x⁺ = x + dt·v⁺ (§4.6). A common-mode time-discretization error would be identical on all three. The evidence that errors are not step errors is the dt series (flat in 25/33 and 28/33), not cross-engine agreement. Reword contribution 3 as an implementation-consistency check.
- **§7.4 and Conclusion 3 ("integrating the foundation over the contact surface is what matters").** On class B the nominal references are the Winkler model's own prediction for `bearing` and `cone`, so this is a consistency statement, as §7.4 itself concedes. The conclusion overstates it.
- **Table 15, row "Load distribution on a compliant support → distributed or gonthier".** The evidence is class C, which §9 calls circular "by design" and excludes "from every comparison with models of another kind". A circular check should not drive a recommendation.
- **§7.3, bowl and trough, and §7.6, turntable.** The losses are "close to what the rolling resistance of the damped foundation predicts", and "the derivation is in the repository's history". These predictions are used to excuse 10–69 % errors. They must be in the paper (or a supplement), with the predicted against measured numbers. `history.md` gives +77 % predicted against +69 % measured for `turntable`.
- **§7.10, "a diagnostic run showed the disk rocking ... more than an order of magnitude larger".** History gives axis tilt 1.8×10⁻⁵ against 1.15–1.19×10⁻³ rad, a factor of about 65. State the number.
- **§2.3 and §4.8.** The description of Hasegawa & Sato, the closest prior art, and the `vertex` baseline rest on a transcription that was not checked against the paper again (`history.md`: "the paper was not reachable again"; §9: "its equations were taken from our notes"). Hasegawa & Fujii, Luo & Nahon and Wakisaka & Sugihara are characterised only through Elandt et al. (`docs/research/literature.md` §0), and Hippmann 2004 was not read. A journal paper must verify claims about its closest prior art from the primary source.

### 2.4 Misused or fragile statistics

**R1, sensitivity of the paired ratios.** My computation, stratified bootstrap, 10⁴ resamples; "cap" = errors and failures capped at 100 %; "−pieces" = `pieces2/8/64` dropped so that the cut ground counts once.

| Comparison | Paper | Without `massratio` | Cap | Cap, −pieces |
|---|---|---|---|---|
| D / Drake hydro, A | 0.56 (0.16–4.1) | **0.24 (0.14–0.40)** | **0.33 (0.15–0.92)** | 0.36 (0.14–1.19) |
| D / Drake hydro, A+B | 0.52 (0.18–2.6) | **0.26 (0.16–0.43)** | **0.34 (0.17–0.79)** | **0.36 (0.16–0.95)** |
| D / Chrono SMC, A+B | 0.40 (0.11–1.6) | **0.25 (0.09–0.78)** | **0.20 (0.08–0.54)** | **0.27 (0.10–0.71)** |
| D / Chrono NSC, A+B | 0.35 (0.08–2.1) | **0.17 (0.06–0.48)** | **0.23 (0.07–0.75)** | 0.44 (0.18–1.13) |
| D / MuJoCo soft, A+B | 0.36 (0.11–1.7) | **0.19 (0.09–0.38)** | **0.26 (0.11–0.65)** | **0.26 (0.10–0.73)** |
| D / MuJoCo default, A | 0.28 (0.05–2.0) | **0.13 (0.04–0.43)** | **0.18 (0.05–0.66)** | 0.34 (0.09–1.22) |
| G / Drake hydro, A+B | 1.1 (0.37–5.4) | 0.58 (0.33–0.99) | 0.70 (0.34–1.6) | 0.67 (0.31–1.75) |
| D / vertex rule, A | 0.57 (0.30–1.00) | – | 0.57 | **1.00 (0.99–1.00)** |
| D / G, A+B | 0.48 (0.30–0.75) | – | 0.48 | 0.54 (0.36–0.81) |
| G18 / G, A | 5.1 (1.7–19) | – | 3.6 (1.6–9.2) | 3.5 (1.4–10) |

What this shows:

1. The class A and A+B conclusions on laws against natives flip with one scenario or with the treatment of failures, so the paper's sentences built on "the interval contains 1" are not robust in either direction.
   - A failure that finishes with a huge error (`massratio`, 6.8 decades) weighs more than a failure that times out or returns nan (0 decades, excluded).
   - The magnitude of a catastrophic error (how far the bodies flew) is arbitrary and should not carry weight.
2. Only the D/G, G18/G and D/PCM comparisons are robust.
3. The ablation on class A rests entirely on the three cut-ground scenarios.
4. About 66 intervals are computed in Table 10 and `stats.json`, with 2–3 false exclusions expected at 95 % even under independence, and the text highlights the few that exclude 1. Use Holm or Bonferroni, or 99 % intervals, or state the multiplicity explicitly.
5. The percentile bootstrap within class B resamples 5 scenarios (3 125 distinct resamples) and is not reliable at that size. Class A contains 4 near-duplicates (`pieces1/2/8/64`) and 3 scenarios driven by the same rolling-resistance mechanism. §6.7 and §9 acknowledge this, but the abstract does not.
6. The "W/L" counts have no tie tolerance. Differences of 10⁻⁵ relative between D and its quadrature variants count as wins or losses: D/vertex on class A is "7/10" although 17 of 21 errors agree within 1 % (12 within 10⁻⁴), and only `pieces2/8/64` differ materially. Use a tie band (for example a ratio within 1.25 counts as a tie) and a sign test.
7. A single base run is used where some natives are highly non-deterministic or step-sensitive.
   - Chrono NSC on `disk`: base 17.7 %, perturbations 0.4–186 %, dt series 19–182 %.
   - Its `mesh32` and `mesh128` runs of `disk` and `turntable` are identical to each other (75.0 % and 20.5 %) but differ from base (17.7 % and 5.2 %), although the shapes are analytic primitives. Something other than the mesh differs between the base and series configurations of NSC, or the base run is not reproducible. Explain.
   - For such cells, report the median over the 8 perturbations next to the base value.

### 2.5 Class definitions

- **Class A includes references that no damped compliant model can meet at the chosen K and h:** `bowl`, `trough` and `turntable` (rolling resistance), and `edge` (compliance). Their error is mostly a function of the arbitrary h = 5 s/m and of K. They are legitimate scenarios, but in a class meant "to compare a volumetric law with a contact model of another kind on equal terms" they compare parameter choices. Either give their references the rolling-resistance term, which the authors derived, or report them in a separate class or with h → 0.
- **Class B nominal references are the Winkler pressure** (uniform for flat faces, cosine for the bearing). The class sits between A and C. The band is the only model-independent part and must be the primary metric (see 2.3).
- **Class C:** the treatment in §7.5 and §9 is honest. Remove class C from the decision table (2.3).

---

## 3. Novelty and positioning

The framing as "a benchmark and a comparison, not a new model" is honest. Credit to Hasegawa & Sato, Gonthier, Hippmann, Sherman et al., Elandt et al. and Masterjohn et al. is explicit and specific (§1, §2.3), which is commendable. Contribution by contribution:

1. **Closed-form Coulomb wrench for a linear pressure on a triangle.** Correct (checked in §5 below). The novelty is incremental: the linear pressure adds the moments R and W, whose primitives are the same four functions as in Jia's uniform case, and the integrals are of the classical BEM family (Wilton et al.). Its value is shown only against quadratures on the battery's rim-fan triangulation. The paper should present it as a short technical result, qualify the 1–2 orders of magnitude claim as triangulation-dependent (R2), and report the cost per triangle against a one-point rule on a well-shaped triangulation of equal accuracy.
2. **Exact mesh-based evaluation:** exact V, c and J; partition into affine pieces; exact clamp; seam treatment. This is solid engineering. The decomposition independence, asserted to 10⁻⁹ and shown on `pieces`, is the most convincing technical contribution. It requires exact, non-overlapping convex decompositions, so the reference to CoACD in §3.6 promises more than the method supports.
3. **Engine-independent library.** This is useful software, but as evidence it is weak (2.3).
4. **The battery.** This is the real contribution and the reason to publish. Its reuse value depends on the fairness fixes in §4 of this report.
5. **Findings.** Mixed; see §2.

**R2, the ablation depends on the triangulation.** For a 64-gon under uniform pressure spinning about its centre, the error of each quadrature against the exact torque is:

| Triangulation of the flat face | One-point rule | Vertex rule |
|---|---|---|
| Fan from a rim vertex (what `meshgen.py` produces) | −17.8 % | −24.9 % |
| Centre fan | +0.14 % | −24.9 % |

The battery's 10 % for the one-point rule on `disk` is consistent with a mixture of the two sides of the contact. Drake hydroelastic's 0.33 % on `disk`, obtained with one point per contact polygon, shows that per-element quadrature on reasonable polygons is accurate. The vertex rule's error is triangulation-independent here; it comes from interpolating unit vectors.

**Missing related work** (checked online):

- **Conformal cylindrical (clearance-joint) contact in MBS, with FEM or analytic references.** This bears directly on `journal` and `bearing` and on the claim that no such problem set was found:
  - Liu, Zhang & Yang, *Mech. Mach. Theory* 42 (2007), doi:10.1016/j.mechmachtheory.2006.02.006.
  - Pereira, Ramalho & Ambrósio, *Nonlinear Dyn.* (2011), doi:10.1007/s11071-010-9830-3.
  - Pereira, Ramalho & Ambrósio, *Multibody Syst. Dyn.* (2015), doi:10.1007/s11044-015-9463-x.
  - The broader clearance-joint literature of Flores, Ambrósio et al.
- **Patch friction and rolling resistance integrated over the contact area, with experimental validation:**
  - Kudra & Awrejcewicz, *IJBC* (2011), doi:10.1142/s0218127411030222.
  - Kudra & Awrejcewicz, *Acta Mech.* (2015), doi:10.1007/s00707-015-1353-z.
  - These are relevant both to contribution 1 and to the "no experimental reference" limitation.
- **Friction and rolling-friction reviews in MSD:**
  - Marques et al., *Multibody Syst. Dyn.* (2019), doi:10.1007/s11044-018-09640-6.
  - Marques et al., *Nonlinear Dyn.* (2016), doi:10.1007/s11071-016-2999-3.
  - The rolling-resistance discussion (§6.3, §7.3) needs this context.
- **Benchmarks of frictional contact solvers:** Acary, Brémond & Huber, FCLIB (2018), doi:10.1007/978-3-319-75972-2_10.
- **Comparisons of contact models:** Drumwright & Shell, ICRA 2011, doi:10.1109/icra.2011.5980402.
- **Patch torsional friction from a pressure distribution in grasping:** Ciocarlie, Lackner & Allen, WHC 2007, doi:10.1109/whc.2007.103.

---

## 4. Methodological soundness

### 4.1 Fairness of the native configurations

- **Drake point.** Disks and cylinders receive no rim spheres, while boxes receive corner spheres "since one point per pair cannot hold a face". This is inconsistent. The class B errors and nan runs of Drake point on `disk`, `contensou` and `cone` are partly a configuration artefact. Give every flat face the same device, or state the asymmetry as a limitation of the comparison.
- **MuJoCo soft.** It has 7.8 % error on `block`, the simplest scenario, against 0.14 % for Drake point. `history.md` attributes this to hopping in the convex cone. A configuration that fails a block on an incline by 8 % needs a reviewed setting (impratio, noslip iterations, solref/solimp) or an explicit statement that it is not representative of expert use.
- **Chrono SMC.** It is sub-stepped by the backend (1–15 sub-steps), so its "1 ms" is not the same discretisation as the others'. State this in the cost and convergence sections.
- **Stiffness mapping.** The area A_ref of one sixth of the surface is a scale. For non-cubic bodies (the plates of `offset` and `tower`, the disk) the native point stiffness differs from the foundation by factors of 2–3, as in `history.md` on `offset`. Report the effective static penetration of each native against the foundation's per scenario. A one-line check per scenario would remove the doubt about Drake hydroelastic's unexplained 11–87 % in `corner`, `stack` and `hole`, for example whether H matches the extent of the field of a compliant cube.
- **Score-driven configuration.** The native configurations were iterated on development scores (N8). The asymmetry "favours the laws" is real but smaller than claimed.

### 4.2 Parameter choices and the explicit coupling

- **K was chosen per scenario so that 1 ms is stable for the explicit laws** (§5). That is a selection effect. Implicit solvers (hydroelastic, NSC, MuJoCo) would be competitive at realistic stiffness, where the laws need dt ∝ K^−½. The single K×10 point is insufficient to call this "towards the rigid limit". Add at least one more decade, at the step the laws need, for the class A scenarios.
- **h = 5 s/m drives the class A rolling errors.** Its choice is not justified physically. Tie it to a restitution coefficient, or show the sensitivity.
- **d and τ were never swept.** The paper says so. The stick spring and damper are stated to be "the same for every contact", but the step bounds (Eq. bounds) change them per load and per step. A referee needs to know, for every scenario and step, whether the bounds were active and by how much: minimum applied h/h₀, maximum d_eff/d, maximum v_s,eff/v_s. Above the threshold the benchmarked object is "the law regularized by the step", and a flat dt series is no longer evidence of convergence of the law. This is the main threat to the claim that the battery measures "the law".
- **`massratio`.** It was "written to find this limit" (§9) yet counts as held-out class A. That is acceptable, but say so where the held-out protocol is described.

### 4.3 Held-out set

Its contamination is disclosed carefully in §6.4, §7.6, Table 8 and §9.

- Table 8's "first run" values come from single Chrono runs transcribed from `history.md` into `provenance.json`. They are not reproducible from `results/`. Give the commit of that first run.
- The abstract must carry the caveat (2.3).
- A truly out-of-sample check is cheap: write 3–5 new scenarios now, freeze them, and run them once.

### 4.4 Convergence evidence

- The dt series is good. The mesh series has three points (32, 64, 128) with "observed orders" fitted to three points. That is acceptable descriptively, but quote no orders to two digits.
- **`journal`.** 5.3, 4.6 and 5.1 % at 32, 64 and 128 segments: non-convergence of a curved conforming contact is a structural limit (one normal per pair of pieces). It conflicts with the claim of an "exact" method. Make the explanation quantitative with a 256-segment run and the angle term.

### 4.5 The admitted anomalies

Are they disqualifying? Not individually. Together they mean that two of the four law-and-coupling combinations have an unexplained physical defect (energy gain, spurious rocking) in the code that produced the battery. Minimal evidence a referee should demand:

- **`gonthier` energy gain.**
  - The 0.16 % → 3.1 % change between code versions must be bisected; the authors know both commits, and one scenario at the base variant takes seconds.
  - A plausible lead from the paper itself: `gonthier` keeps the *full* vector area, seams included (§3.6). The paper states that this normal "tilts with the depth of the overlap at the edge of a piece", which is a non-conservative mechanism on the faceted, decomposed bowl. The decay with mesh refinement (5.1 → 3.1 → 0.07 %) fits it.
  - Test `gonthier` with the seam-free normal used by `distributed`.
- **K×10 rocking of the disk.**
  - Run at K×10 with dt = 0.1 and 0.05 ms; with the step bounds disabled; with friction off; with the torsional bristle off. Report the tilt amplitude.
  - This separates the coupling from the law. Both laws share the bristle and the bounds, and hydroelastic and PCM (no bristle) are unaffected, which points at the stick state.
- **`journal`:** see 4.4.
- **NSC base-run anomalies:** see R1 item 7.

### 4.6 Experimental or FEM reference

Their absence is acknowledged. For a journal in this field, at least the class B conforming cases (`bearing`, `cone`) could be given an FEM elastic-contact solution, since the band is meant to bracket the truth. Without it, class B is consistency plus a band of guesses. The clearance-joint FEM literature (Liu et al. 2007) offers a template.

### 4.7 Reproducibility

Good. Data, scripts, a container with pinned engines and generated numbers are provided. Remaining gaps:

- the first held-out run (4.3);
- the placeholder repository URL and the missing archive DOI;
- the timing pass on one machine, which is acceptable;
- AGENTS.md states that the battery itself may not be rerun without permission, which is a process rule rather than a scientific one, but the paper should give the total CPU cost of a battery pass.

---

## 5. Correctness of the mathematics

Checked line by line.

**Correct:**

- Eq. (gonthier-normal): exact volumetric damping, because v_n is affine.
- τ_r = −K·h·J·ω_t, to O(depth²): the in-plane block of ∫(r×n)(r×n)ᵀdV equals J's tangential block up to the ∫r_z² terms. Say "to first order in the depth" in §4.2 as is done in §4.3.
- Gonthier friction and the ellipse identity; r_g = R/√2; the ratio 3/(2√2) = 1.061; "5.7 % early".
- Column length, Eq. (column).
- Constancy of n·v along a column, and ∫Kℓ(1 − h·n·v)dA = KV(1 − h·n·v(c)).
- Eq. (product). The coefficients c_ijk = 6, 2, 1 follow from ∫λ₁^aλ₂^bλ₃^c = 2A·a!b!c!/(a+b+c+2)!, and the decomposition c_ijk = 1 + δ_ij + δ_ik + δ_jk + 2δ_ijk gives the printed sum.
- Eq. (equivalent): Σ_k gives P̃ = (FG+S)/4, hence p̃_k = (g_kF + f_kG + 2f_kg_k)/5 − (FG+S)/20.
- Eq. (centre): q = x₀ + m×v_t0/ω_m satisfies v_t(q) = 0.
- Eqs. (friction-force) and (friction-moment), with r×(m×r) = |r|²m.
- Eq. (edge), all four moments, including the W components sec³cos² = sec, sec³cos·sin = sec·tan and sec³sin² = sec³ − sec.
- Eq. (primitives).
- The degenerate limits.
- The disk check D → (2π/3)R³.
- The gradient formula of App. A, assuming the vertices are counter-clockwise about m; state this.
- The code `core/src/polar_moments.cpp` matches Eq. (edge) term by term. `check_polar_moments.py` passes, including 3.35×10⁻⁵ for the 256-gon.
- Eq. (stability). The matrix, the characteristic polynomial and the Jury conditions give 0 < b < 2 and a + 2b < 4. "If and only if" needs b > 0; at b = 0 the scheme is only marginally stable.
- The numerical examples: a + 2b = 0.54N, stable to N ≈ 7.4, b = 0.049N, the transition slope maximum √2·e^−½/v_s = 0.858/v_s, b = 0.17N, ω² = μ_s·g/d, ζ ≈ 0.55, d/τ = 20 mm/s.
- Eq. (bounds): units and the sum argument.

**Errors and inconsistencies:**

1. The sign of the column rate in words (N10), in §4.1, §4.3.3 and the code comment.
2. The step bound in §9 is off by √2 (N1).
3. T_max is described as "the moment D ... summed over the triangles". It is Σ(p_q·D + g_p·R) with q = c_p, the pressure-weighted moment, not D.
4. The gonthier static limit is a box, not the shares of Eq. (gonthier-friction) (N12).
5. "Exact" covers less than the abstract and §1 imply. The friction uses the linear equivalent p̃ wherever ℓ and g both vary, and the seam rule counts a partly-seam face as surface. §9 states both; §1 contribution 2 should too.
6. **Notation clashes**, which must be resolved for a journal:
   - h: damping factor [s/m] and distance from q to an edge (§4.4, App. A);
   - A: body and triangle area;
   - a: stability parameter, width of a contact and Gonthier's hysteretic factor;
   - b: stability parameter and facet width;
   - w: inverse mass (Table 1, §4.6) and share of a column (Eq. pressure);
   - g: gravity, damping factor g(x) and pressure gradient g_p;
   - τ: stick time constant, torques τ_r and τ_s, and the profile factor τ (Fig. 10);
   - d: pre-sliding displacement and plane offset d_f;
   - S: load acceleration, vector area S_B and the sum S of Eq. (product);
   - D: polar moment and the abbreviation of `distributed`.
7. §4.3.5 ("Normal wrench"): the order-δ/a lateral pressure on the side faces makes the friction-to-load ratio μ(1 + O(δ/a)). State the magnitude for the battery's scenarios: δ/a ≈ 10⁻³–10⁻², which is below the class A thresholds, but it is a systematic bias of the law against pure Coulomb.

---

## 6. Presentation

**Length.** 70 pages; the main body is about 54 pages; there are 115 references. A normal MSD length is 25–35 pages. Concrete cuts:

- **§3 (pp. 7–11).** Keep the normal (Eq. normal), seams and fallback in about 2 pages. Move the coplanar rules, BSP and fast-path details and the cost paragraph to a supplement. Drop Fig. 1 (architecture).
- **§4 (pp. 11–23, 12 pages).** Keep §4.1–4.4 and the stiction equations, about 6 pages. Move the derivation of the step limits (§4.6) to an appendix, keeping one paragraph and the bounds. Shorten §4.7 to the list. Shorten §4.8 to one page.
- **§6.** Remove the narrative of earlier versions: "The first version of this battery configured the native models carelessly…", "This definition replaced an earlier one…", "An earlier analysis ranked medians with Friedman and Wilcoxon tests… withdrawn", and "An earlier version of this work compared…" in §4.2, §7.7 and §8. A journal article reports the final method; the development history belongs in the repository.
- **§7 (16 pages).** Tables 4–7 duplicate Figs. 7–8 cell for cell, so keep one of each pair and move the other to a supplement. Table 2 (status), Table 3 with Fig. 6 (independence), Table 13 (stiffness) and §7.11 (perturbations) go to the supplement, with one sentence each in the text. Table 10 and Fig. 9 are also duplicates.
- **§8–§10 (pp. 46–56).** They repeat each other and §1. Merge Discussion and Limitations. Keep the decision table (corrected) and cut the repeated disclaimers: "properties of the model, not defects of the engines" appears 4 times, "the configurations are ours" 5 times, and the class C circularity is explained 5 times.
- **References.** Cut to about 70: the biomechanics list of 5 to 2, the simulator reviews of 4 to 1, and the peripheral IPC, SDF, Newton, CENIC and Han 2023 entries. Add the missing works of §3.

**Figures.**

- Fig. 12 plots errors as fractions (10⁰ = 100 %) while the caption, text and tables use percent. Use one unit throughout.
- Several figure captions do not state the engine on which the library methods are shown (Chrono per §7.1).
- Fig. 10 has 11 overlapping step lines and is hard to read. Show 5–6 methods, or use small multiples.
- Fig. 7's colour bar is legible. The hatched-cell legend should name the "t/o" status.

**Tables.**

- Table 5 (class B) relies on † and * markers whose meaning collapses when the narrow side of a band is 0 (`ring`, `cone`); use signed errors.
- Table 15 (decision) has the circular row and the NSC and implicit-solver issues (N6, N7).

**Terminology.**

- One name per method. The paper alternates between "the distributed law", "the integrated law" (abstract) and "D"; between "gonthier-thesis", "the 2018 variant" and "G18"; and between "MuJoCo soft", "MuJoCo's recommended configuration" and "Mj-s".
- "Point contact" lumps a complementarity solver (NSC), soft constraints (MuJoCo) and penalty models (SMC, Drake point). Differentiate them where conclusions differ.
- "held", "holds", "lost" and "a run that holds" are undefined jargon until §7.2.

**Language.** Grammatical, clear and precise, but stylistically repetitive. Several passages read as generated boilerplate and should be rewritten in a conventional register:

- rhetorical Q&A paragraph heads ("Are these laws better than the existing ones?", "Always?", "When, then?");
- formulaic hedges ("We state this as the result of a search, not as a proof of absence", "The consequences must be stated plainly", "which is their purpose, and the protocol requires that we report them", "This is a modest, specific result");
- the constant "X, and not Y" constructions.

**Back matter.**

- Remove the `[TODO]` markers and fill the AI-use statement. The AI-use disclosure is extensive, and whether its extent is acceptable is an editorial decision. The authors must state what they personally verified (derivations, code of the laws, scenario references); the current text leaves this as a TODO.
- The author list is marked provisional, and the co-authors have not confirmed authorship. That must be settled before submission.
- Replace `github.com/OWNER/...` with a real URL and archive the release with a DOI.

---

## 7. References check

I verified 51 entries against Crossref, DataCite, PMLR and arXiv, covering authors, title, venue, year, volume, pages and DOI:

> Hippmann2004, Hippmann2024, Sherman2011, Fregly2003, Elandt2019, Masterjohn2022, Castro2020, Castro2023, Castro2025, Han2023, Gonthier2004, Gonthier2005, Gonthier2007detc, Boos2013, Brown2018, Shourijeh2015, Petersen2013, Roy2012, Luo2006, Luo2011, Wakisaka2017, Hasegawa2003, Hasegawa2004, Contensou1963, Zhuravlev1998, Kireenkov2008, Leine2003, Howe1996, Jia1999, Dopico2011, Flores2022, Flores2023, Fleischmann2016, LeLidec2024, Acosta2022, Gonzalez2006, Gonzalez2010, Ruggiu2023, Vehvilainen2024, Bouchard2015, Weller2009, Corral2021, Skrinjar2018, Yoon2023, Erleben2018, Chung2016, Anitescu2010, Mazhar2015, Tasora2016, ElandtThesis2022, Guo2023, Kurtz2025cenic, Faure2008.

**All DOIs resolve to the cited works, and titles and authors match.** The differences are minor:

- **Online-first against issue year.** Crossref "issued" differs from the bib year for Hippmann2024 (2023 online), Brown2018 (2017), Flores2022 (2021), Dopico2011 (2010), Luo2011 (2010), Luo2006 (2005), Boos2013 (2012), Gonzalez2010 (2009), Anitescu2010 (2008) and Fleischmann2016 (2015). The bib uses the issue year, which is the correct convention; no change is needed.
- **Tasora2016.** Crossref lists the container as *Lecture Notes in Computer Science* (vol. 9611). Add the series and volume.
- **Petersen2013.** The pages are missing. The article number is V07AT10A051, in Vol. 7A of IDETC 2013.
- **Gonthier2005.** The paper number DETC2005-84610 and Vol. 6 are not shown. Add them; Crossref has no pages.
- **ElandtThesis2022 and Bouchard2015.** These are DataCite DOIs (Cornell eCommons and the Canadian HCC Society). Both are correct, but `doi.org` lookup via Crossref fails, so the paper's reference-check script should use DataCite as well.
- **SimBenchmark.** No year, so the reference prints without a date. Add the year of the page or of the code release.
- **Web documentation entries** (Drake, MuJoCo, Chrono, Simbody, Newton, IFToMM). Give the software version (Drake 1.58, MuJoCo 4.0, Chrono 10.0) and the access date in each entry, not only "2026".
- **Gonthier2007thesis.** `literature.md` §0 says the thesis was not reachable, while `docs/gonthier-model.md` says it was read in full on 2026-11-11. Reconcile the internal record; the paper's use is consistent with the latter.

No erroneous reference was found. The substantive problem is not the metadata but the *reading*: some content attributions are second-hand, listed in 2.3.

---

## 8. Required changes

### 8a. Actionable in the text or with data already in `results/`

1. **Rewrite the statistical summary.**
   - Add Table R1-style sensitivity results to the paper: errors capped at 100 % with failures counted as capped; leave-one-scenario-out, with `massratio` reported; the cut ground counted once.
   - Add a multiplicity statement or correction.
   - Report W/L with a tie band.
   - Remove or qualify every sentence that rests on "the interval contains 1" (abstract, §7.7, §8 ¶1, Conclusion 2).
2. **Abstract and conclusions:** remove the class A "no significant difference" claim, or state its fragility. Name the point-contact models that are as accurate on one plane. Replace "off by 1.8 % to 97 %" with band-based statements. Add the held-out caveat. Qualify the "1–2 orders of magnitude" as being on the battery's rim-fan triangulation.
3. **Signed class B errors.** Compute them from `terms.csv` or the stop times. Report in-band and out-of-band counts, and drop the † and * ambiguity.
4. **Correct N1–N12:** the √(m/(KA)) bound; the hand-typed 24 %; the Table 10 bolding against the text; the massratio engine statement; the MuJoCo soft sensitivity; NSC `turntable`; Drake point losing `massratio` and `tower`; "no iteration on scores"; the triangulation description; the sign of the column rate (text and code comment); the centroid-shift wording; the gonthier box limit.
5. **Remove class C from the recommendations** (Table 15 row 6). Reword Conclusion 3 as consistency on class B.
6. **Report the base value and the median over the eight perturbations** for every native cell whose perturbation range exceeds a factor of 2 (for example NSC `disk` and `turntable`, Drake point `disk`). Explain why NSC's `mesh32` and `mesh128` runs on analytic primitives differ from its base run.
7. **Move the rolling-resistance predictions** for `bowl`, `trough` and `turntable` from `history.md` into the paper, with predicted and measured values side by side.
8. **Reword contribution 3 and §5.** Engine agreement is an implementation-consistency check; the dt series is the evidence about integration error.
9. **State the Drake point configuration asymmetry** (no rim points on disks) and the Chrono SMC sub-stepping.
10. **Fix the notation clashes** (§5 item 6), unify the method names, and use one error unit in all figures.
11. **Add the missing related work** (§3). Verify the Hasegawa & Sato equations, and any claim about Luo & Nahon, Hasegawa & Fujii, Wakisaka & Sugihara and Hippmann 2004, against the primary sources.
12. **Cut to about 35 pages:** follow §6 above; remove the development-history narrative; deduplicate tables and figures; merge §8 and §9; move the details to supplementary material.
13. **Complete the back matter:** all TODOs, the authorship and CRediT confirmation, what the authors verified themselves, the repository URL and archive DOI, and the commit of the first held-out run.

### 8b. Needing new computation

1. **Log and report, for every scenario and step, whether the step bounds were active**, and the extreme ratios of the applied parameters to the nominal ones (h, d_eff, v_s,eff, stick k). Mark runs where they are.
2. **Diagnose the `gonthier` energy gain:** bisect between the first-run commit and 2b917f6, and test the seam-free normal for `gonthier`.
3. **Diagnose the K×10 disk rocking:** dt 0.1 and 0.05 ms; bounds off; friction off; bristle off.
4. **Quadrature ablation on a well-shaped triangulation** of the flat faces (centre fan, Delaunay with interior points), and cost at equal accuracy.
5. **Native fairness reruns:** Drake point with rim spheres on cylinder faces; MuJoCo soft `block` with a reviewed impratio and noslip setting; NSC repeated runs (or more iterations) on `disk` and `turntable`; a static-penetration check of every native, hydroelastic included (`corner`, `stack`, `hole`).
6. **One more stiffness decade** (K×100) for class A, at the step the laws need.
7. **`journal` at 256 segments** with the angle term reported, to make the non-convergence quantitative.
8. **A small, newly written and frozen set of 3–5 scenarios**, run once, to give genuinely out-of-sample evidence.
9. **At least one FEM reference** for a class B conforming case (`bearing` or `cone`), to place the band and the Winkler nominal against elastic contact.
10. **Sweeps of d and τ** over the stick-relevant scenarios (`push`, `wedge`, `tower`, `stack`, `tip`, `hole`).
