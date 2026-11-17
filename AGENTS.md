# AGENTS.md

Instructions for any agent that reviews, validates or improves this repo. Read this file first.

## What this is

An engine-independent C++20 library of two elastic-foundation contact laws evaluated on the exact
intersection of triangle meshes: `gonthier` (Gonthier's published volumetric model; `gonthier-thesis`
keeps the friction of David Vilela's 2018 PhD thesis) and `distributed` (the same Winkler pressure
integrated over the contact surface, with a closed-form Coulomb friction wrench per triangle). It has
Drake 1.58, Chrono 10.0 and MuJoCo 4.0 backends, literature baselines (`pcm` and two friction
quadratures), a battery of 33 scenarios with analytic references (classes A, B, C; 12 held out) that
compares them with six native contact models, and a paper (`paper/`) whose every number is generated
from the battery. The paper is a benchmark and a comparison; it does not claim a new contact model.

## Read before changing anything

1. `README.md`: what the repo is, the laws, how to build and run, limitations;
   `docs/architecture.md`: architecture and design decisions.
2. `docs/history.md`: what was done, in order, and why each decision was taken.
3. `docs/scenarios.md`: the battery's contract (mesh criteria, scenario API, native-contact
   rules, the 33 scenarios with their theory and error metric); `docs/baselines.md`: the
   literature baselines and their departures.
4. `data/README.md`: mesh assets (names, frames, criteria).
5. `results/stats.md` (accuracy and statistics of the committed battery; conclusions come from
   its robust comparisons: capped errors, failures counted, Holm over the primary family). Timings are
   `results/timing.csv` (the timing pass of 2026-11-12 on an idle machine; `results/load.txt`).
   `results/fem/README.md`: the finite-element reference of `disk` and `bearing` (`tools/fem`).
6. `paper/README.md`: how results enter the paper (`tools/paper_results.py`, `paper/generated/`).

## Rules

- **English** for everything in the repo: code, comments, docs, commit messages, UI.
- **Only general improvements.** Nothing tuned to one scenario (no per-test formulas, constants
  or calibrations). A change must make sense for scenarios that do not exist yet.
- **The force laws** (`core/src/gonthier.cpp`, `core/src/distributed.cpp`, `core/src/pcm.cpp` and
  their parameters) may change when the change is general, backed by evidence, covered by tests and
  documented in `docs/history.md` (the owner's standing approval of 2026-11-11). A change that moves
  reported results needs the battery rerun, `paper/generated/` regenerated and the paper's text
  reread against it; say so to the owner instead of leaving the paper stale.
- **Build and test only through Docker**: `scripts/dev.sh <cmd>` (the host cannot run Drake's
  binaries). `scripts/verify changed` before every commit (the pre-commit hook runs it; never
  bypass it); `scripts/verify full` to close a piece of work.
- **The machine is shared** with other agents. Do not run the battery (`scripts/bench.sh`),
  timing passes (`--timing`) or loops of benchmarks without the owner's go-ahead; a few single
  runs (`volcontact_bench --scenario X --engine E --contact C --variant base`) are fine. Never
  leave servers or background jobs running when you finish.
- **Timings** are valid only from `scripts/bench.sh --timing` on an idle machine. Accuracy does not
  depend on the load.
- **Meshes** come from `uv run tools/meshgen.py` (deterministic); never edit `data/` by hand.
  `volume.vtk` files are generated (`--missing-volumes`), not committed.
- **Git:** work on a branch; parallel agents use `git worktree` folders outside the main checkout.
  Never push or publish without the owner's explicit instruction.
- **Document every change**: a dated entry in `docs/history.md` (problem, evidence, change,
  reasons, results), and `README.md` when results, decisions or open problems change.
- **No number typed by hand** in the paper or the README's results block: they come from
  `tools/paper_results.py` and `tools/results_summary.py`.
- **Tests**: one per acceptance criterion, through the public interface; a regression test for
  every bug fixed. Bench output goes under `build/` and is never committed.

## How to validate

- `scripts/verify full`: build without warnings, all doctest cases, a quick run of every scenario.
- Read each scenario's theory in `docs/scenarios.md` §4 against `scenarios/src/<name>.cpp`.
- With the owner's go-ahead: `scripts/bench.sh` (accuracy) and `uv run tools/stats.py
  results/battery.csv`; compare with the committed `results/battery.csv`.
- `scripts/viewer.sh` and `http://localhost:8000/tools/viewer/` to watch any run against theory.

## Open problems, by priority

The evidence for each is in the latest entries of `docs/history.md` and in Sections 5 and 7 of the
paper.

1. **Explicit coupling.** K·A·Σw·dt² ≤ 2 (dt ≤ √(m/(K·A)) between two flat contacts) bounds the step for a body held between stiff contacts:
   `massratio` is lost at 2, 1 and 0.5 ms and holds from 0.2 ms with 24 % of error (residual speed:
   the dampers through the light body are bounded by its mass over the step). The fix is implicit
   integration with the force Jacobians (specified in `docs/history.md`, not started).
2. **One normal per pair of convex pieces.** On curved conforming contact `distributed` does not
   converge with the mesh in `journal` (4.6 % at every resolution) and is not monotone in `cone`;
   `pcm` (local normals) and Drake's hydroelastic contact do better there.
3. **Friction-induced flutter on a stiffer foundation** (diagnosed, `docs/research/anomalies.md` §1).
   With K × 10 the class B errors of `gonthier` and `distributed` grow (`disk` 0.12 % → 18 %) at every
   step from 1 to 0.05 ms: a tilt unbalances the friction of the sliding face, which acts below the
   centre of mass, and the whirl grows when μ·K·(π·R³/3)·(H/2) > (h·m·g·R²/4)·√(K·I_A/I_t), that is
   above K ≈ 1.9e6 N/m³ for `disk` (growth predicted 6 and 31 s⁻¹ at 3 K and 10 K, measured 6.6 and
   32). A limit of validity of the damped Winkler law with sliding friction, not a coupling defect.
   Open: why `pcm` does not flutter (up to 100 K); how much of `cone` and `bearing` at 10 K is this
   and how much the facet sag (one run of `cone` `stiff` at 128 segments would tell); whether to
   state the damping as a time constant (changes the law and every result).
4. **`energy`: the metric, and a small real non-conservation** (diagnosed, `anomalies.md` §2). The
   energy term omits the energy stored in the undamped contact spring, so it varies irregularly
   with the step and the start for every law (`gonthier` 0.14–3.3 %, `pcm` 0.07–4.6 %,
   `distributed` 0.1–0.7 %); the 3.1 % of `gonthier` at 1 ms is not a steady gain. F = K·V·n is
   not conservative on faceted geometry (curl −K·(V/A²)·∇A×∇V); its size was not isolated. Open:
   a metric that counts the stored energy; which change moved the nominal run from 0.16 % to 3.1 %
   (needs a bisect between 4d5a0da and 2b917f6).
5. **Rolling resistance of the damping term** (`turntable` 69 %, `bowl`, `trough`): any damped
   Winkler foundation has it, so a class A theory without it cannot be met while h > 0. It falls with
   K × 10. Give the theory the term or run with small h.
6. **One static finite-element reference, no experimental one.** `tools/fem` (`results/fem/README.md`, paper
   Section 5.4) covers `disk` and `bearing`: the thick slab of `disk` is a half-space (rigid-punch pressure), and
   against it every Winkler-type law, Drake hydroelastic included, is off (`distributed` 18 %); `bearing` confirms the
   Winkler cosine. Open: `ring`, `contensou`, `cone`; friction and dynamics; measured data. Class C checks consistency
   only.
7. **Timings.** `results/timing.csv` covers the base variant only; the two friction quadratures were
   not timed, and Drake hydroelastic `tower` exceeds the limit (its cost in the paper is a lower bound
   from the accuracy pass). A new pass needs the owner's go-ahead and an idle machine.
8. **Native contacts, unexplained results.** Drake hydroelastic: `hole` 87 %, `corner` 11 %, `stack`
   15 % (step-independent), `tower` and the small steps of `stack` and `massratio` time out. Chrono
   NSC: `oblique` and most of `restitution` end without a rebound value, `edge` 32 %. MuJoCo `default`:
   `edge` 30 %. The configurations are ours and have not been reviewed by the engines' developers.
9. **Not measured.** Sensitivity to the stiction constants d and τ; the friction quadratures on
   remeshed flat faces inside the library, at equal cost (`tools/ablation_triangulation.py` settles
   their accuracy on a single face: 18–25 % on the battery's rim fans, second order on well-shaped
   meshes); PCM's published sampling (whole faces by centroid); where the step bounds are active;
   the sign of the class B errors (known for `disk` and `bearing`, `results/fem/stop_times.csv`). The list of runs the referee asked for is in the paper's
   Section 8.4 and `docs/research/response-to-referee.md`.
