# Anomalies: diagnosis (2026-11-12, int/paper at 58728ba, read-only)

Runs: 27 single runs, Chrono, existing build; outputs in `build/anom/` (ignored). Linear model:
`/tmp/vc-review/disk_flutter.py` (numpy, standalone). Nothing in the repo was edited.

## 1. Class B on a stiffer foundation (`disk`, `cone`)

### Verdict

**Not the explicit coupling, not a mesh effect, not an implementation defect on `disk`: it is a
friction-induced flutter (self-excited wobble) of the continuous model**, i.e. of a damped Winkler
foundation with Coulomb friction under a sliding body whose centre of mass is above the contact.
It is predicted by a linear stability condition, quantitatively.

### Evidence (`disk`, Chrono; error = stop time)

| K × | dt [ms] | `gonthier` | `distributed` | max tilt [rad] |
|---|---|---|---|---|
| 1 | 1 / 0.2 | 5.6 % | 0.12 % | 1.8e-5, decaying |
| 3 | 0.2 | 6.5 % | 0.21 % | 1.5e-4–2.1e-4, growing e^(6.6 t) while it spins |
| 10 | 1 | 22.5 % | 21.8 % | 1.8e-3 |
| 10 | 0.2 (`stiff`) | 19.6 % | 18.1 % | 1.15e-3 / 1.19e-3 |
| 10 | 0.1 | 19.4 % | 17.8 % | 1.10e-3 / 1.14e-3 |
| 10 | 0.05 | 19.3 % | 17.7 % | 1.08e-3 / 1.12e-3 |
| 30 | 0.2 | 25.5 % | 25.0 % | 2.5e-3 |
| 10, 30, 100 | 0.2, `pcm` | 10.0 % (unchanged) | | 7e-6 … 3e-6, decaying |

- The error does **not** fall with the step (0.2 → 0.05 ms: 18.1 → 17.7 %): hypotheses (a) and (b)
  are out. Their numbers at K × 10, 0.2 ms: rocking k = K·I_A = 3.07e4 N·m/rad on
  I_t = 0.0158 kg·m² gives ω_rock = 1392 rad/s, a = ω²·dt² = 0.078; vertical a = K·A·dt²/m = 0.079;
  rocking damper c = K·h·J_t = h·m·g·r²/4 = 0.766 N·m·s, b = c·dt/I_t = 0.0097. All far below
  a + 2b < 4.
- Tilt history (trajectory, 10 ms samples): exponential growth from the first step while the disk
  spins (K × 10: 1.6e-5 → 1.1e-4 in 0.06 s, about 32 s⁻¹), saturation at 1.2e-3 rad, where the rim
  travels 0.30 mm against a static penetration of 5 µm: the disk rides on its rim (centre of mass
  lifted 190 µm), and it decays as soon as the spin stops (2.7e-4 → 1e-7 in 0.25 s). On the rim the
  friction torque tends to μ·N·R instead of (2/3)·μ·N·R: hence the stop 18–20 % early (the limit
  is 33 %).
- Mechanism. A tilt θ shifts the pressure, p = K·(δ + θ × ρ); with the face spinning, the friction
  of the heavier side no longer cancels: net friction force F = −μ·K·(π·r³/3)·θ (distributed;
  Gonthier's lumped force at the shifted centroid gives 0.354 instead of 0.333·π·r³), acting h_d/2
  below the centre of mass. Its moment, N·ẑ × θ with **N = μ·K·(π·r³/3)·(h_d/2)**, is perpendicular
  to the tilt: a circulatory (non-conservative) stiffness. For I_t·z̈ + c·ż + (k − i·N)·z = 0 the
  whirl grows when **N > c·ω_rock**, at the rate (N/ω_rock − c)/(2·I_t).
  - K: N = 245, c·ω_rock = 337 → stable (ratio 1.37). Threshold K_c = c²·I_A/(I_t·(N/K)²) =
    **1.9e6 N/m³**, twice the scenario's K.
  - K × 3: predicted +6.3 s⁻¹ (numerical 10-state linearization with translation and gyroscopic
    terms: +5.7); measured +6.6 s⁻¹.
  - K × 10: predicted +31 s⁻¹; measured about 32 s⁻¹.
  - Linearization checks: friction applied at the height of the centre of mass → stable; h = 20 s/m
    → stable; h = 0 → +56 s⁻¹; μ = 0.3 → +3.3 s⁻¹. Independent of the spin speed (Coulomb), so it
    lasts until the disk stops.
  - Scaling: N ∝ K, c ∝ load (independent of K), ω_rock ∝ √K: N/(c·ω_rock) ∝ μ·h_d·√K/h. A stiffer
    foundation with the same h is less damped in rocking (ζ = 5.5 % at K, 1.7 % at 10 K) while the
    circulatory term grows.
- So (c) the stick spring plays no role (the face slides throughout) and (d) is excluded on the
  flat disk (flat face, no sag).
- **Not explained: why `pcm` does not flutter** (stable up to K × 100, tilt decaying). Its normal
  law, damping and friction lever are nominally the same; it samples pressure and friction at one
  point per triangle of the patch. Not investigated further (needs its discrete sums in the linear
  model). Drake's hydroelastic contact: different dissipation and an implicit solver; not looked at.
  The claim "property of the continuous model" therefore rests on the linear model matching the
  two laws' growth rates, not on every Winkler method showing it.

### `cone`

`stiff`: `gonthier` 35.7 %, `distributed` 38.9 %, `pcm` 28.1 %; `distributed` at 0.05 ms 39.0 %
(step-independent). The frustum wobbles 0.065 rad (3.7°), climbs 2.3 mm out of the seat
(sinking −1.3 mm against +0.09 mm expected) and stops late. The paper attributes it to the facet
sag (0.12 mm of sag at 64 segments against 0.093 mm of penetration, which is true), but a 3.7°
wobble is the same kind of self-excited motion; the sag is the likely seed and the reason `pcm`
and hydroelastic contact degrade there too. Not separated here: one run of `cone` `stiff` at 128
segments would tell (sag 0.03 mm).

### General fix (not implemented)

None inside the law that keeps the model: the instability is the model's. Options, all general:
(1) scale the damping with the stiffness so that the rocking damping ratio is kept, e.g. state the
dissipation as a time constant (pressure K·(d + τ·ḋ) instead of K·d·(1 + h·ḋ)): then c ∝ K and
N/(c·ω_rock) ∝ 1/√K, stable for stiffer foundations; this changes the published law and every
result; (2) keep the law and state the condition **μ·K·S_ρ·z_c < c_rock·ω_rock** (S_ρ = ∫ρ·dA of
the patch, z_c height of the centre of mass over it) as a limit of validity for sliding conforming
contact; (3) in the benchmark, raise h with √K in the stiffness series, or read the `stiff`
variant of class B as outside that limit. Recommendation: (2) in the paper now, (1) as future work.

### Statement for the paper

> With a foundation ten times stiffer the spinning disk wobbles and stops 18 % early with both
> laws, at every step from 1 to 0.05 ms: it is not a limit of the explicit coupling (the rocking
> and vertical modes have K·w·dt² = 0.08 at 0.2 ms) but a friction-induced flutter of the model.
> A tilt θ unbalances the friction of the sliding face by μ·K·(π·R³/3)·θ, which acts below the
> centre of mass and turns the tilt; the whirl grows when μ·K·(π·R³/3)·(H/2) exceeds the rocking
> damping h·m·g·R²/4 times the rocking frequency √(K·I_A/I_t), that is above K = 1.9e6 N/m³ here,
> and the predicted growth rates (6 s⁻¹ at 3 K, 31 s⁻¹ at 10 K) are those measured (6.6 and 32).
> The per-face baseline does not show it up to 100 K, which we have not explained.

### Sentences to change

- `07-results.tex` 569–574 ("a diagnostic run showed the disk rocking … we did not find what
  excites it. We report it as an open defect of the two laws or of their explicit coupling"):
  replace with the statement above; "more than an order of magnitude" is 60× (1.8e-5 → 1.2e-3).
- `07-results.tex` 563–569 (`cone`, `bearing`: "the penetration is … no longer large against the
  sag … That is a limit of the resolution, not of the stiffness"): too categorical for `cone`
  (3.7° wobble, climbs out of the seat, step-independent); say the sag is one cause and the wobble
  of the stiffer, relatively less damped foundation another, not separated.
- `08-discussion.tex` 35–36 ("A fifth, the rocking … is an open defect"): it is a diagnosed
  limit of the model (friction-induced flutter), not a defect of the implementation.
- `09-limitations.tex` 99–100 ("The rocking of the spinning disk on the stiffer foundation … not
  explained"): remove from the undiagnosed list; add a limitation with the stability condition.
- `09-limitations.tex` 71–73 ("the facets must be small against the penetration"): keep, but it
  no longer explains all of the class B degradation.
- `README.md` 135, `AGENTS.md` open problem 3: "not diagnosed" → diagnosed; what stays open is
  `pcm`'s immunity and `cone`/`bearing`.

## 2. `gonthier` "gains" 3.1 % in `energy`

### Verdict

**The 3.1 % is mostly not energy created: it is the scenario's metric reading the energy that
moves between the swing and the undamped bounce on the contact spring, which the metric does not
count.** It is not specific to `gonthier` (the `pcm` series is as irregular: 0.4–4.6 %). A small
true non-conservation exists for the published model on faceted geometry (derivation below), but
its size was not isolated. No implementation defect was found.

### Evidence

- The metric is mean(KE + PE_gravity) over the last period minus the same over the first, with the
  contact's elastic energy left out on the assumption that it cancels. It does not: h = 0, so the
  bounce excited by the start (1 µm of penetration instead of the static 0.46 mm) and by every
  facet crossing is never damped. Sinking 0.46 mm releases m·g·δ = 4.5e-3 J = **2.9 % of the
  swing's energy** (0.155 J): the scale of the anomaly.
- `gonthier`, 1 ms (`build/anom/e1`): the series dips to 0.78 and its mean over the first period
  is 0.956, over the last 0.986: +3.06 %. Its upper envelope (instants of least stored energy)
  stays between 0.95 and 1.018 with no trend; the run never holds more than 1.8 % above its start,
  and the start itself sits 1 % low by the missing sink. At 0.2 ms the same means are 0.958 and
  0.952 (−0.6 %): the first period is the same, the last one catches the bounce in another state.
  `distributed`, 1 ms: means 0.981 and 0.977, envelope flat at 1.004–1.010.
- With K × 10 (penetration and its energy 3 times smaller, bounce faster): drift 0.016 %
  (`gonthier`), 0.03 % (`distributed`). With 128 segments (smaller kicks): 0.07 %.
- Battery, Chrono, energy-drift term [%], dt 2 / 1 / 0.5 / 0.2 / 0.1 ms: `gonthier`
  3.3 / 3.1 / 1.4 / 0.6 / 2.8; `pcm` 3.2 / 0.4 / 1.3 / 4.6 / 2.0 (and 3.5 at `stiff`);
  `distributed` 0.29 / 0.36 / 0.08 / 0.14 / 0.69. Eight perturbed starts: `gonthier` 0.14–3.0
  (median 1.2), `pcm` 0.07–4.0, `distributed` 0.25–0.73 (mostly its period term). No trend with
  the step: (c) and (d) are out as causes (damping is 0, so the bounds are inactive; mu = 0).
  Engines: Chrono and MuJoCo agree to 1e-9, Drake to 0.01 points: deterministic, not round-off.
- (a) Derivation. For a rigid displacement δx of A, dV = A_vec·δx with A_vec the vector area of
  the contact surface, so ∇V = A·n and Gonthier's force is F = K·V·n = K·V·∇V/|∇V|. Then
  curl F = −K·(V/A²)·∇A × ∇V: F derives from a potential only where the projected area A is a
  function of V alone (sphere on plane, ideal sphere in ideal bowl: U = K·∫V/A dV; flat contact:
  U = K·A·δ²/2, not K·V²/2). On faceted or multi-piece geometry A varies along the surfaces of
  constant V (over an edge or a seam against the middle of a facet), so the published normal force
  is not conservative there, and the force through the volume centroid adds a moment (spin up to
  0.45 rad/s, 13 times `distributed`'s). The code does exactly this (`gonthier_force`:
  `fn·n` at `geo.centroid`): a property of the published model on faceted geometry, not a defect.
  It vanishes with the facets (0.07 % at 128 segments). `distributed` with one normal per pair has
  the potential (K/2)·∫d²·dA while the pair normal is fixed, and loses it only where the normal
  changes between pieces: consistent with its flat envelope.
- (b) Normal fallback / transitions: not examined separately; the spin and the 1 ms jumps of up to
  ±4 % of the series at facet crossings are the same for the three laws (`distributed` ±7 %).

### Why the earlier run gave 0.16 %

Not pinned to a commit. Read: `energy.cpp`, `common.hpp`, the 64-segment meshes and
`gonthier.cpp` are unchanged since the stability commit (0313067), whose entry says "identical to
three digits"; after it the core changed only for `pcm`, the baselines' quadratures and
`finalize()`. The eight perturbed starts span 0.14–3.0 %, so 0.16 % and 3.06 % are both ordinary
draws of this metric; but since the run is deterministic, something did change the nominal
trajectory between the held-out run (4d5a0da) and the battery (2b917f6), and the candidates are
the merges of the series/variants (e32c51c, bench and scenario options) and of the native-model
work, or the worktree builds having been made with other flags. A bisect with one run per commit
would settle it (it needs builds, outside this task). The bounds and the integrated bristle are
excluded: with h = 0 and μ = 0 neither acts.

### General fix (not implemented)

The metric, not the law: (1) count the stored energy, or compare the series at instants of equal
contact state (upper envelope, or means over several periods at both ends); (2) start the body at
its static penetration and give the scenario a damping that acts only on the bounce, or accept
K × 10 as the scenario's stiffness (there all laws are within 0.04 %); (3) for the law: nothing to
fix without changing the published model; a conservative variant would take the force as −∇U of
U = (K/2)·∫d²·dA, which is what `distributed` does per pair.

### Statement for the paper

> In `energy` the measured drift is that of kinetic plus gravitational energy between the first
> and the last period; the energy held by the undamped bounce on the contact spring is not
> counted, and sinking to the static penetration alone moves 2.9 % of the swing's energy, so the
> metric varies irregularly with the step and with the start for every law (`gonthier` 0.14–3.3 %,
> `pcm` 0.07–4.6 %, `distributed` 0.1–0.7 %) and falls to 0.02–0.03 % with ten times the
> stiffness and to 0.07 % with 128 segments. The 3.1 % of `gonthier` at the base step is such a
> value and not a steady gain: the series never exceeds its start by more than 1.8 %. Gonthier's
> normal force K·V·n derives from a potential only where the contact area is a function of the
> volume alone, which faceted surfaces break, so a residual non-conservation of the published
> model is expected; it was not isolated from the metric's noise.

### Sentences to change

- `07-results.tex` 299–300 ("`distributed` and `pcm` keep the energy … within 0.5 %"): false for
  `pcm` beyond the base run (3.2 % at 2 ms, 4.6 % at 0.2 ms, 3.5 % at `stiff`, up to 4.0 %
  perturbed).
- `07-results.tex` 300–305 ("`gonthier` gains … we have not found what changed, and report the
  battery's value"): replace with the statement above; "gains" is wrong as a steady gain (−0.6 %
  at 0.2 ms).
- `07-results.tex` 305–306 ("Every native model but Chrono SMC has an error of 24 % or more here,
  in the energy"): reread against the same metric; not checked here.
- `07-results.tex` 484–486 (`energy` "classified as diverging … irregularly; that of `gonthier`
  is irregular too, at a higher level"): add the cause (uncounted bounce energy) and `pcm`.
- `07-results.tex` 556 (`energy` among the scenarios that improve with K × 10 "at errors below
  0.5 %"): consistent; it is the evidence for the cause and can say so.
- `09-limitations.tex` 100–101 ("the energy gained by `gonthier` in the faceted bowl and its
  irregular dependence on the step … not explained"): remove from the undiagnosed list; add the
  non-conservativeness of K·V·n on faceted geometry as a property of the published model.
- `README.md` 135–136 and `AGENTS.md` open problem 4: same; what stays open is which change moved
  the nominal run from 0.16 % to 3.1 % and the size of the true non-conservation.
- `docs/scenarios.md` §4 (`energy`): the comment "the energy the contact stores cancels in the
  difference" (also in `energy.cpp`) does not hold while the bounce is undamped.
