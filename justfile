# One-word commands for everything this repository exposes. The recipes only call the scripts in
# scripts/ and the tools in tools/, which stay the source of truth. `just` lists them.
#   Container work (build, test, run) needs Docker; the core, the Python tools and the paper do not.

set shell := ["bash", "-uc"]

dev := "scripts/dev.sh"
configure := "cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DVOLCONTACT_NATIVE=ON -DVOLCONTACT_WITH_CHRONO=ON -DVOLCONTACT_WITH_MUJOCO=ON -DCMAKE_CXX_COMPILER_LAUNCHER=ccache"

# List the recipes.
default:
    @just --list --unsorted

# ---- Container: build, test, run -----------------------------------------------------------

# Build the Docker image with Drake, Chrono and MuJoCo (once; or pull ghcr.io/dvilelaf/volumetric-contact-dev, see README).
[group('container')]
image:
    {{dev}} image

# Configure if needed and build everything in build/ inside the container.
[group('container')]
build:
    [ -f build/CMakeCache.txt ] || {{dev}} {{configure}}
    {{dev}} cmake --build build

# Run the unit tests (core and scenarios) inside the container; builds first.
[group('container')]
test: build
    {{dev}} ./build/tests/volcontact_core_tests
    {{dev}} ./build/tests/volcontact_tests

# Quality checks of what this branch changed: build without warnings, tests, lint, secret scan.
[group('container')]
verify:
    scripts/verify changed

# Every check, plus the install check and a smoke run of every scenario (closes a piece of work).
[group('container')]
verify-full:
    scripts/verify full

# List the scenarios of the battery (needs `just build`).
[group('container')]
scenarios:
    {{dev}} ./build/apps/volcontact_bench --list

# Run one scenario once (base variant) and print its metrics. CONTACT: a law (gonthier, distributed, pcm, ...) or native:<model>.
[group('container')]
run SCENARIO ENGINE='chrono' CONTACT='distributed':
    {{dev}} ./build/apps/volcontact_bench --scenario {{SCENARIO}} --engine {{ENGINE}} --contact {{CONTACT}} --variant base

# ---- The core alone, on the host (CMake, Eigen, a C++20 compiler; no Docker) ----------------

# Build and install the engine-independent core into build-core/ (prefix build-core/prefix).
[group('core')]
core:
    cmake -S . -B build-core -G Ninja -DCMAKE_BUILD_TYPE=Release -DVOLCONTACT_BUILD_BENCHMARK=OFF -DVOLCONTACT_BUILD_TESTS=OFF -DCMAKE_INSTALL_PREFIX=build-core/prefix
    cmake --build build-core
    cmake --install build-core

# Build and run the core's unit tests on the host, in build-core-test/ (fetches doctest).
[group('core')]
core-test:
    cmake -S . -B build-core-test -G Ninja -DCMAKE_BUILD_TYPE=Release -DVOLCONTACT_BUILD_BENCHMARK=OFF -DVOLCONTACT_BUILD_TESTS=ON
    cmake --build build-core-test --target volcontact_core_tests
    ctest --test-dir build-core-test --output-on-failure

# Build examples/minimal against the installed core and run it: a box dropped on a slab, no engine.
[group('core')]
example: core
    cmake -S examples/minimal -B build-core/example -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$PWD/build-core/prefix"
    cmake --build build-core/example
    ./build-core/example/minimal

# ---- The battery -----------------------------------------------------------------------------

# The accuracy battery: every engine, contact model, scenario and variant. It takes HOURS and the machine is shared: run it only with the owner's permission. Extra bench.sh options go in ARGS.
[group('battery')]
bench *ARGS:
    scripts/bench.sh {{ARGS}}

# The battery on the base variant only (a check of the harness, not a result); still long.
[group('battery')]
bench-quick *ARGS:
    scripts/bench.sh --quick {{ARGS}}

# The timing pass (serial, 5 repeats): valid ONLY on an idle machine and only with the owner's go-ahead.
[group('battery')]
timing:
    scripts/bench.sh --timing

# Statistics, ranking and figures from results/battery.csv.
[group('battery')]
stats:
    uv run tools/stats.py results/battery.csv

# ---- Viewer, site, paper -------------------------------------------------------------------

# Serve the trajectory viewer on results/traj at http://localhost:PORT/tools/viewer/.
[group('publish')]
viewer PORT='8000':
    scripts/viewer.sh {{PORT}}

# Assemble the static site into build/site.
[group('publish')]
site:
    scripts/site.sh

# Assemble the site and serve it at http://localhost:PORT/.
[group('publish')]
site-serve PORT='8000':
    scripts/site.sh --serve {{PORT}}

# Build paper/main.pdf and paper/supplement.pdf (Tectonic, downloaded once).
[group('publish')]
paper:
    scripts/paper.sh

# Numerical checks of the derivations in the paper.
[group('publish')]
paper-check:
    scripts/paper.sh check

# ---- Python tools --------------------------------------------------------------------------

# Regenerate the meshes of data/ and the volume meshes that are missing (deterministic).
[group('python')]
meshes:
    uv run tools/meshgen.py
    uv run tools/meshgen.py --missing-volumes

# Lint the Python code with ruff.
[group('python')]
lint:
    uvx ruff check tools paper site

# Run the Python tests of tools/.
[group('python')]
py-test:
    uv run --quiet --with pytest --with numpy --with scipy --with matplotlib --with trimesh pytest -q tools/tests
