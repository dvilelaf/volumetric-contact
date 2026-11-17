# Contributing

Thanks for your interest. This project is a research code base with a benchmark whose results are
cited in a paper, so changes are held to a few rules that keep it reproducible.

## Build and test

The full build needs Drake, Project Chrono and MuJoCo, which run on Ubuntu 24.04. Everything goes
through one container so that the versions are pinned:

```
just image                  # once (or pull ghcr.io/dvilelaf/volumetric-contact-dev, if available; see the README)
just build                  # configure and build inside the container
just verify                 # build without warnings, unit tests, Python tests, lint, secret scan
just verify-full            # everything, plus an install check and a smoke run of every scenario
```

`just` alone lists the other recipes (`test`, `run`, `lint`, `py-test`, `meshes`, ...); they wrap the scripts in `scripts/`,
which are the source of truth (`scripts/verify`, `scripts/dev.sh`).

The core library alone (geometry and the force laws) needs only Eigen 3.4 and CMake, and builds and
tests without Docker (`just core`, `just core-test`, `just example`; see "Use it in your engine" in the README). The GitHub CI runs that part, the Python tools
and the secret scan; the engine backends are checked by `scripts/verify full` on a machine with the
container. Enable the pre-commit hook (it runs `scripts/verify changed --staged`) with
`git config core.hooksPath .githooks`, and do not bypass it.

## Rules

- **English** for code, comments, documents, commit messages and the viewer.
- **General improvements only.** Nothing tuned to one scenario: no per-test formulas, constants or
  calibrations. A change must make sense for scenarios that do not exist yet.
- **The force laws** (`core/src/gonthier.cpp`, `core/src/distributed.cpp`, `core/src/pcm.cpp` and their
  parameters) change only after discussion with the maintainer. Open an issue with the evidence
  first (a failing scenario, the cause, the proposed change and its effect on the rest of the battery).
- **Tests.** One per acceptance criterion, through the public interface, and a regression test for
  every bug you fix. Tests that need an engine skip it when it is absent.
- **Document the change** in `docs/history.md` (dated entry: problem, evidence, change, reasons,
  results), and in `README.md` when results, decisions or limitations change.
- **Meshes** come from `uv run tools/meshgen.py` (deterministic); never edit `data/` by hand.
- **Results.** `results/battery.csv` and the files derived from it come from `scripts/bench.sh` (`just bench`).
  Timings are valid only from `scripts/bench.sh --timing` (`just timing`) on an otherwise idle machine. Do not edit
  generated results by hand.
- **Style.** Match the code around yours: naming, comment density, idiom. Python is formatted and
  linted with `ruff`.

## Pull requests

Work on a branch, keep commits logical, and describe what changed and why. Run `just verify`
before pushing. By contributing you agree that your work is released under the BSD
3-Clause licence of the project (`LICENSE`).

## Reporting problems

Open an issue with the command you ran, the output, and the commit. For a wrong result, name the
scenario, engine, contact method and variant (`volcontact_bench --scenario X --engine E --contact C
--variant base` reproduces a single run). Be respectful (`CODE_OF_CONDUCT.md`).
