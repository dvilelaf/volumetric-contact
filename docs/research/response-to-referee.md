# Response to referee report 1

Revision of 2026-11-12 (branch `agent/revision`). The report is `docs/research/referee-1.md`; numbers below refer to
its sections. Status: **done**, **partly**, **not done**. Nothing here needed a rerun of the battery; the only new
computation is the triangulation experiment (seconds of CPU). Every number quoted is generated
(`results/stats.md`, `paper/generated/`).

## The four reasons for "major revision"

| # | Point | Status | What was done |
|---|---|---|---|
| 1 | The inferential summary does not survive other treatments of the data | done | `tools/stats.py` computes robust comparisons: errors capped at 100 %, failed runs counted at the cap for either method, `pieces*` one unit, tie band 1.25, leave-one-out range, sign test, 14 primary comparisons with Holm; everything else is exploratory. The paper prints the first analysis next to the robust one (Table "Primary comparisons") and draws conclusions from the robust one only. Result: two comparisons survive, both on A+B (`distributed`/`gonthier` 0.51, 0.34–0.77; `distributed`/Chrono SMC 0.23, 0.085–0.62); none on class A, where the laws nevertheless have the smaller error in most scenarios. Abstract, §7.6, §8.1 and Conclusion 2 rewritten. The primary family was chosen after seeing the data; the paper says so. |
| 2 | Fairness and selection effects | partly | Stated in the text: K chosen for the explicit coupling at 1 ms and h arbitrary (§6.3, §8.3); `bowl`, `trough`, `turntable`, `edge` measure h and K, and class A is also reported without them (Table pass counts, Table S "reduced"); Drake point's corner spheres on boxes against one point under a disk (§6.5, S3); NSC `turntable` quoted with median and range, Drake point named instead (§7.3, §8.1, decision table); held-out contamination in the abstract. Not done: any rerun of a native model with a fairer configuration. |
| 3 | The novelty is modest and its benefit depends on the triangulation | done | `tools/ablation_triangulation.py` (tested) compares the three rules on rim fans, centre fans, slivers, well-shaped refined meshes and random Delaunay meshes of a disk, and on a ring. Rim fan: one point −18 %, vertex −25 %, at any number of segments; centre fan: one point −0.04 %, vertex −25 %; well-shaped meshes: second order; closed form identical on all (4e-15). The paper now claims independence of the triangulation and of the decomposition, not orders of magnitude, in the abstract, contribution 1, §7.7, §8.1 and Conclusion 4, and calls the result a direct extension of Jia's. §7.7 describes the mesh correctly (fan from a rim vertex). Contribution 3 is reworded as an implementation-consistency check. |
| 4 | Unexplained defects; not submittable | partly | The two anomalies are diagnosed (`docs/research/anomalies.md`) and the paper says so with the evidence: friction-induced flutter with its condition, threshold (1.9e6 N/m³) and predicted against measured growth rates (§7.9, §8.2); the `energy` measure omits the contact spring, and K·V·n is not conservative on facets (§7.3, §8.2). Open and stated: why `pcm` does not flutter; `cone`/`bearing`; the 0.16 % → 3.1 % change is not pinned to a commit. Length: 61 pages in all, about 43 of main text (from 70 and about 54), plus a 17-page supplement; the 35–40 aimed at was not reached. The `[TODO]` markers of the back matter, the author list and the repository URL are the owner's and remain. |

## §2.2 Numbers and statements that did not match

| # | Status | Change |
|---|---|---|
| N1 | done | dt < √(m/(K·A_c)), with Σw = 2/m written out (§8.2, S2); 0.25 ms for `massratio`. README and AGENTS agree. |
| N2 | done | The sentence now reads "above 20 %" and quotes Drake point's value from a generated key. Hand-typed values that remain are design parameters or diagnostic single runs, each with a comment naming its source. |
| N3 | done | The vertex-rule ratio on class A is reported under the robust rules (0.85, 17 ties in 18 units, the one unit that differs being the cut ground); no bold, no "touches 1". |
| N4 | done | §7.2: in `massratio` the three engines agree in relative terms. |
| N5 | done | S4 "Sensitivity to perturbations": representative for the laws, Drake hydroelastic and MuJoCo soft; less so for Drake point and Chrono's models. |
| N6 | done | NSC `turntable`: base value with median and range of the perturbed runs wherever quoted; Drake point is the example in §8.1 and the decision table. A generated table lists every native cell that scatters by more than a factor 2. |
| N7 | done | §7.3, §7.8, decision table: Drake point loses `massratio` and `tower`; "implicit in the contact forces is not sufficient"; the recommendation names Chrono NSC and MuJoCo soft. |
| N8 | done | §6.4, §6.5 and S3 say that the native configurations were revised after first results on the development scenarios, with the four examples of `docs/history.md`. |
| N9 | done | §6.3 and §7.7 describe the caps as fans from a rim vertex. |
| N10 | done | §4.1, §4.3.3 and the comment in `core/src/distributed.cpp`: n·v is the rate at which the column shortens. |
| N11 | done | 0.22 mm per µrad for L = 0.4 m and δ = 60 µm; 2 mm for 10 µrad. |
| N12 | done | §4.5: checked in `core/src/gonthier.cpp` (`coupling()`): the shares are s + (1 − s)·C, both 1 at stick, so the static limit of `gonthier` is a box, up to √2 times the ellipse; the text says so and no longer criticises only the alternative. T_max is described as the pressure-weighted polar moment. |

## §2.3–2.5 Overclaims, statistics, classes

| Point | Status | Change |
|---|---|---|
| "Neither law differs significantly" (abstract) | done | Replaced by the robust result and its fragility. |
| "Point contact is as accurate on one plane" | done | Names Drake point and Chrono NSC; gives the MuJoCo and `vgroove` counter-examples. |
| "Off by 1.8 % to 97 %" | done | Replaced by band counts (inside / undetermined / outside / no stop time, Table "band"); the 1.8 % of Chrono SMC on `ring` is said to be below hydroelastic's 2.9 % on `cone`. |
| Signed class B errors | not done | Not in `results/`: `terms.csv` has the size of `stop_time` only, and the band scalars of the run reports were not kept. The paper states what can be known without the sign and lists the logging as needed (§8.4). |
| Held-out caveat in the abstract | done | Last sentence of the abstract. |
| Engine agreement as evidence | done | §5, §7.2, Conclusion 1: an implementation check; the step series is the evidence about integration error. The three integrators were checked in the backends before asserting that they share the step. |
| Class B agreement is partly by construction | done | §6.1, §7.4, Conclusion 3: consistency, not validation. |
| Class C in the decision table | done | Row removed; the caption says classes A and B only. |
| Rolling-resistance predictions in the paper | partly | §7.3 gives the torque, the growth law of `turntable` (77 % predicted, 69 % measured) and `bowl` (12 % against 15 %). `trough` has no recorded prediction. |
| 60× rocking | done | "about 65 times", with the two amplitudes in a comment. |
| Hasegawa and Sato from the primary source | not done | The paper was not accessible again. §4.8 and §8.3 say that the vertex rule follows our reading and was not re-checked against Eqs. 13–16, and that the statements about Hasegawa and Fujii, Luo and Nahon, Wakisaka and Sugihara rest on Elandt et al. |
| R1.4 multiplicity | done | Holm over 14 primary comparisons. |
| R1.5 bootstrap on five scenarios, near-duplicates | partly | `pieces*` is one cluster; the text says the intervals are descriptive and that class B has five scenarios. The three rolling scenarios are not clustered; the reduced class A removes them. |
| R1.6 tie band | done | Factor 1.25, with W/T/L and a sign test. |
| R1.7 single base runs; NSC mesh variants | partly | Median and range reported for scattering cells. Why NSC's `mesh32`/`mesh128` runs differ from base on analytic primitives is not explained; S4 says so. |
| §2.5 class A with parameter-bound references | done | Reduced class A (17 scenarios) in the pass table and the primary comparisons. |

## §3 Novelty and related work

| Point | Status | Change |
|---|---|---|
| Present the closed form as a short technical result | done | Contribution 1, §8.1. |
| Cost per triangle against a one-point rule at equal accuracy | not done | Listed in §8.4. |
| CoACD promises more than supported | done | §3.4: approximate decompositions overlap or leave gaps and are not covered. |
| Missing related work | done | Liu et al. 2007, Pereira et al. 2011 and 2015, Kudra and Awrejcewicz 2011 and 2015, Marques et al. 2016 and 2019, Acary et al. 2018 (FCLIB), Drumwright and Shell 2011, Ciocarlie et al. 2007: each checked against Crossref (`paper/references-verification.md`) and cited for what its title and abstract state; none was read in full. |

## §4 Methodological soundness

| Point | Status | Change |
|---|---|---|
| 4.1 Drake point asymmetry, SMC sub-stepping, stiffness mapping, MuJoCo soft on `block` | partly | Stated (S3, §6.5, §7.8, §7.10). No per-scenario static penetration check and no rerun. |
| 4.2 K chosen for the laws; h unjustified; d and τ not swept; bounds active? | partly | Stated (§6.3, §4.6, §8.3). Where the bounds act is inferred from the loads (`tower`, `massratio`, `stack`), not logged. No K × 100, no sweeps. |
| 4.3 Held-out set | partly | Commit of the first run given (4d5a0da); `massratio` said to have been written to find the limit. No new frozen scenarios. |
| 4.4 Orders from three points; `journal` | partly | No order is quoted for the mesh series; "exact" is qualified. No 256-segment run. |
| 4.5 Anomalies | partly | Diagnosed with existing single runs (anomalies.md). The bisect and the seam-free normal for `gonthier` were not run. |
| 4.6 FEM reference | not done | Listed in §8.4; the clearance-joint literature is cited as the template. |
| 4.7 Total CPU cost of a pass | done | Generated key (`battery/cpu-hours`), §6.5 and §7.10. |

## §5 Mathematics

| Point | Status |
|---|---|
| 1 sign of the column rate | done |
| 2 step bound | done |
| 3 T_max | done |
| 4 box limit of `gonthier` | done |
| 5 what "exact" covers, in contribution 2 | done |
| 6 notation clashes | done: η (distance to an edge), A_c and A_T (areas), L (width of a contact), γ (share of a column), φ (damping factor), ∇p, M_r and M_s (torques), e_f (plane offset), Λ (load acceleration), Q (sum of products), α and β (stability parameters, supplement); table abbreviations Gon and Dis; the performance profiles, whose factor was τ, are no longer in the paper |
| 7 magnitude of the side-face bias | done (§4.3.5) |
| "to first order in the depth" in §4.2; "if and only if" needs k > 0; vertices counter-clockwise in App. A | done |

## §6 Presentation

| Point | Status | Change |
|---|---|---|
| Length | partly | 61 pages (main text about 43, appendices 3, references 13) and a 17-page supplement, from 70. Moved to the supplement: acceleration structures and coincident faces, the derivation of the step limits, the properties list, the costs of the laws, the native configurations, the status, independence, error, held-out, ratio, perturbation, stiffness and cost tables and five figures. Removed: Fig. 1 (architecture), the performance profiles, the development-history narrative ("an earlier version…"), repeated disclaimers; Discussion and Limitations merged. |
| References cut to about 70 | not done | 109 entries are cited (115 before): ten added, sixteen no longer cited. |
| Figures in one unit; engine stated; fewer lines | done | Percent everywhere; captions and §7.1 say Chrono; six methods in the step series. |
| Table 5 markers | partly | Band counts in the main text; the per-scenario table (supplement) keeps † and * because there is no sign. |
| Terminology | done | One name per method (§6.5); "held", "lost", "no value" defined in §7.1. |
| Language | partly | Q&A paragraph heads and the formulaic hedges named by the referee are gone; the rest was not restyled. |
| Back matter | not done | Owner: acknowledgements, AI models, what the authors verified, funding, competing interests, DOI, CRediT, authorship, repository URL. |

## §7 References

Tasora2016 (series volume), Petersen2013 (article number), SimBenchmark (year 2018, from the repository's creation
date), software versions in the documentation entries: done. Gonthier2005 already carried its paper number and
volume. DataCite in the reference-check script and the Gonthier2007thesis note in `literature.md`: not done.

## §8b Needing new computation: not done, listed in the paper's §8.4

Bounds-active logging; `energy` bisect and seam-free normal; K × 10 disk runs beyond those of `anomalies.md`;
native fairness reruns; K × 100; `journal` at 256 segments; new frozen scenarios; FEM reference; d and τ sweeps.
Item 4 (quadratures on well-shaped triangulations) is done outside the battery, on single faces; inside the
library, with cost at equal accuracy, it is not.
