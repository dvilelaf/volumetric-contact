# Architecture and design notes

The long form of what the [README](../README.md) summarises: how the code is organised, why each design decision was taken, and how to build, run and inspect everything. The history of every decision is in [`history.md`](history.md); results are generated (`results/stats.md`) and discussed in the paper, not here.

## The two laws

They share the same exact mesh intersection:

- **`gonthier`:** Gonthier's volumetric model. A single force K·V at the volume centroid, with rolling resistance and Gonthier's published friction: Coulomb sliding and Coulomb spinning that share one friction budget through an average surface velocity ([`gonthier-model.md`](gonthier-model.md) gives the equations and their sources). The form of the 2018 thesis, with a viscous spinning torque, stays selectable (`GonthierParams::friction_model = kThesis2018`).
- **`distributed`:** the same Winkler foundation, integrated point by point over the contact surface. Each point carries a pressure K·ℓ·(1 − h·vn), with ℓ the length of the overlap column along the normal, and Coulomb friction from its own slip. The centre of pressure, spinning, rolling and combined slip come out on their own, with no shape factors.

Alongside the two is the `native` method: the engine's own contact, with fixed rules for all scenarios ([`scenarios.md`](scenarios.md) §3).

## Architecture

```
core/        Geometry and force laws. Depends only on Eigen (and OpenMP, optional).
  mesh             TriMesh and OBJ reader
  bvh              flattened AABB tree
  bsp              solid BSP: containment and polygon clipping; coplanar-face rule
                   correct also for non-convex meshes
  volume_integrator volume integrals by divergence (Eberly)
  intersection     CollisionShape (mesh + BVH + BSP + planes if convex) and intersect():
                   V, centroid, inertia, normal and, on request, the surface of ∂(A∩B)
  gonthier         Gonthier's law (published friction, or the 2018 thesis form) with bristle
                   stiction; FrictionState (pack/unpack)
  distributed      distributed law: exact partition of each polygon by the face where
                   the column ends, exact linear pressure and closed-form friction
  polar_moments    closed-form integrals ∫|r|, ∫r/|r|… over triangles (Coulomb friction
                   with rigid slip = rotation about an instantaneous centre)
  contact_model    pairs → one wrench per body; law per pair; pairs in parallel (OpenMP)
scenarios/   Battery of 33 scenarios, engine-independent: setup, theory and metric
engines/     Engine interface and backends drake_backend/ chrono_backend/ and mujoco_backend/
apps/bench   scenarios × engines × method (gonthier | distributed | native) × variant → CSV + JSON
tests/       doctest: analytic geometry, laws, engines and one test per scenario
tools/       meshgen.py, volume_mesh.py, decompose.py (see "Meshes"), stats.py, viewer/ (see "Viewer")
```

Design decisions (the full history is in [`history.md`](history.md)):

- **Pair normal:** the vector area of the part of ∂(A∩B) that belongs to B. It matches the thesis's least-squares plane without extracting the contour.
- **Column length:** ℓ is the minimum of affine functions, one per face where the column can end. Each polygon is split where the active face changes, so the elastic pressure is integrated exactly, without subdividing.
- **Stiction in the distributed model:** it uses a translational spring and a torsional one at the centre of pressure. Their deformation grows with the displacement of the material points between steps, so rolling without slipping does not load them. They never exceed the static limit: there is no 80 % re-engagement as in the thesis.
- **Stiction stiffness proportional to the load (both laws):** k = μ_s·|Fn| / d, c = k·τ, with d = 0.1 mm and τ = 5 ms for every contact, so a contact at rest reaches its static limit after the same displacement whatever its load; the total stick force (spring + damper) never exceeds μ_s·|Fn|. The thesis's constant spring (400 N/m) stays selectable (`preslide_travel = 0`, `travel = 0`).
- **Non-convex bodies:** they are split offline into convex pieces (the asset's `pieces/`, see `data/README.md`) and one pair is generated for each combination of pieces. The distributed model requires convex pieces.
- **Parallelism:** OpenMP over the pairs, with a broad phase and a thread cap (half of the hardware threads by default); with 16 threads on a loaded machine it was 200 times slower.
- **Drake integration:** forces enter as `ExternallyAppliedSpatialForce`. The geometry is computed once per step and the friction state is discrete state. Drake applies that state's update before the plant reads the forces; the system detects it and reuses the result.
- **Chrono integration:** forces enter through accumulators. `UpdateForces` must be called after accumulating, otherwise they would be applied one step late.
- **Native contact:** one generic, documented configuration per native model, the same for all scenarios ([`scenarios.md`](scenarios.md) §3, with the sources). Drake: `hydroelastic` (default; compliant half-space or compliant tetrahedral grounds with E = K·H, rigid bodies refined to edges of 1/8 of their size, 2-coloured body–body contacts) and `point` (compliant point contact, k = K·A of a reference area, corner spheres on boxes). Chrono: `smc` (default; primitives and convex pieces, Hooke with kn = K·A/4, sub-stepped because it is explicit) and `nsc` (complementarity, Barzilai–Borwein). MuJoCo: `soft`. `--contact native:<model>` selects one.

## The battery

Thirty-three scenarios, each with an analytic (or integrable) theory and an error metric, in three classes of theory (A a rigid-body result, B an assumed pressure distribution, C the elastic-foundation model itself). Twelve of them are a held-out set, written before any law was run on them. The first three are the thesis's. The full specification (meshes, API, native contact rules, scenario table) is [`scenarios.md`](scenarios.md).

Each scenario runs the base variant plus the series of `scenarios.md` §2: meshes with 32 and 128 segments (curved scenarios), time steps of 2, 0.5, 0.2 and 0.1 ms, stiffness × 10 (not class C) and eight seeded perturbations. Methods: `gonthier`, `gonthier-thesis`, `distributed`, the ablation and baseline laws of [`baselines.md`](baselines.md) and every native model. Every run has a wall-clock limit (`--time-limit`) and a slow one is recorded as `timeout`.

## Build, run, verify

Drake's binaries need Ubuntu 24.04, so everything that touches an engine runs in Docker:

```
scripts/dev.sh image                  # once: Ubuntu 24.04 + Drake 1.58 + Chrono 10.0 + MuJoCo 4.0
scripts/dev.sh cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DVOLCONTACT_WITH_CHRONO=ON -DVOLCONTACT_WITH_MUJOCO=ON
scripts/dev.sh cmake --build build
scripts/dev.sh ./build/apps/volcontact_bench --list
scripts/dev.sh ./build/apps/volcontact_bench --scenario block --engine drake --contact distributed
scripts/bench.sh                      # whole battery (accuracy) + statistics → results/
scripts/bench.sh --timing             # the same plus timings: only on an idle machine
scripts/bench.sh --quick | --resume   # base variant only | continue an interrupted pass (limits: LAW_LIMIT, NATIVE_LIMIT, NATIVE_SERIES_LIMIT, seconds)
uv run tools/stats.py results/battery.csv   # tables and figures (--no-figures to skip matplotlib)
scripts/viewer.sh                     # trajectories viewer, http://localhost:8000/tools/viewer/
scripts/verify changed                # builds, no warnings, tests (pre-commit hook)
scripts/verify full                   # the same + quick bench run
```

The image is called `volumetric-contact-dev:drake-1.58.0-chrono-10.0.0-mujoco-4.0.0` (change it with `VC_IMAGE`). MuJoCo is the third engine (`--engine mujoco`, `engines/mujoco_backend/`). Accuracy does not depend on the machine's load; timings (`--timing`, a separate serial pass) do, so measure them only on an idle machine.

## Meshes

The meshes are in `data/` (format and criteria in `data/README.md`). `surface.obj` and `pieces/` are versioned; `volume.vtk` (tetrahedral mesh for Drake's hydroelastic contact) is generated and not versioned.

```
uv run tools/meshgen.py                          # all assets, deterministic, verified
uv run tools/meshgen.py NAME ...                 # some of them
uv run tools/decompose.py mesh.obj folder        # arbitrary mesh → convex pieces (CoACD)
```

For the battery the convex pieces are procedural (`meshgen.py`); `decompose.py` is for arbitrary meshes.

## Statistics

`uv run tools/stats.py results/battery.csv` (called by `scripts/bench.sh`; `--no-figures` skips matplotlib). It writes `results/stats.md`, `stats.json` and `results/figures`: pass rates at 1/5/20 % per theory class, paired error ratios with stratified-bootstrap intervals, performance profiles, convergence slopes with the step and the mesh, the stiffness series, engine independence and the spread over perturbations. See [`scenarios.md`](scenarios.md) §2.

## Viewer

`scripts/bench.sh` also writes every run's trajectory to `results/traj/` (not versioned) and its index, `results/traj/index.json` (`uv run tools/viewer/index.py results/traj` rebuilds it). `scripts/viewer.sh [PORT]` serves the repo; open `http://localhost:8000/tools/viewer/`.

- **Scenario and run.** The title is the scenario selector; under it, what the scenario is, what it tests, its theory and the class of the theory (A, B, C: [`scenarios.md`](scenarios.md) §4). The buttons below choose the variant of the run (base, fine mesh, small step, perturbed start), each with a line that says what it changes.
- **Two boxes of the same size.** Left, the runs overlaid in 3D: drag to rotate, right-drag to pan, scroll to zoom, double-click or `R` to reset the view; a switch draws everything solid, solid with edges or as wireframe. When the bodies travel far the view rides along with them, and the controls keep working. Right, one plot per series: the theory is the thick dark line drawn over everything and named in each plot; click a plot to move the time.
- **Playback.** Restart, one sample back, play or pause, one sample forward, a scrubber and the playback speed; it starts again at the end. Keys: space, ←/→, Home/End.
- **Table.** The theory is the first row; then one row per engine × contact method with its compute speed (×200 is 200 times faster than real time; `≈` marks a value from the parallel accuracy pass, exact ones come from `results/timing.csv`), its error with the terms of the error when the run has them, and the measured values under their theory. The eye hides a run from the 3D view and the plots (`only`, or Alt-click, leaves that run alone); click a column to sort by it.
- **Colours.** Hue is the contact method, shade and dash are the engine, the same in the table, the 3D view and the plots. Engines, methods, scenarios and variants are whatever `index.json` lists; `tools/viewer/descriptions.js` only adds texts (scenario descriptions, variant names).
- **Static site.** The page has no server side and only relative paths: it needs `tools/viewer/`, `results/traj/` and the `surface.obj` files of `data/`. `data-traj` and `data-assets` on the `<html>` tag of `index.html` say where the last two are when they are published elsewhere. A trajectory is fetched when it is first shown.

## Repo layout

```
core/ scenarios/ engines/ apps/ tests/ tools/   see "Architecture"
AGENTS.md    rules and open problems for agents that review or improve the repo
data/        battery meshes (data/README.md)
docs/        integration.md (use the core in your engine), scenarios.md (specification), history.md (history and decisions), gonthier-model.md (the gonthier law's equations and sources), thesis-report.md (the thesis and its code)
docker/      build image
scripts/     dev.sh (commands in Docker), bench.sh (battery), verify (checks), check-install.sh (install + example)
examples/    minimal: the core used from outside the repo (find_package)
results/     output of bench.sh: battery.csv, terms.csv, stats.md, stats.json, load.txt, and timing.csv after a timing pass (traj/ and parts/ are not versioned)
tools/viewer the trajectories viewer (static page, vendored three.js, uPlot and Barlow)
```

## Published site

`scripts/site.sh` assembles `build/site/` (landing page, the viewer under `viewer/`, the slim trajectories under
`data/traj/`, the meshes, the paper PDF if built). The slim dataset is `site/data/traj`, written by
`uv run tools/viewer/export_site_data.py` from a full `results/traj`: the base variant of every scenario, every engine and
method, at most 300 samples per run and rounded, a few megabytes in all. Regenerate and commit it after a battery run.
`scripts/site.sh --serve` previews it at `http://localhost:8000/`. Variants of a run (mesh, time step, stiffness, perturbed
start) appear in the viewer when the full local `results/traj` is served with `scripts/viewer.sh`.
