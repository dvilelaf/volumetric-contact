#!/usr/bin/env bash
# Installs the core to a temporary prefix, builds examples/minimal against it with
# find_package(volcontact) and runs it: the check that the core is usable without this repository's
# build tree, benchmark or any physics engine. Run it in the container: scripts/dev.sh scripts/check-install.sh;
# CI runs it on Linux, macOS and Windows (Git Bash, MSVC environment).
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
# The repository root as CMake reads it (Git Bash on Windows needs C:/... rather than /c/...).
root=$PWD
if command -v cygpath >/dev/null; then root=$(cygpath -m "$PWD"); fi

work=$(mktemp -d "${TMPDIR:-/tmp}/volcontact-install.XXXXXX")
work=$(command -v cygpath >/dev/null && cygpath -m "$work" || echo "$work")
trap 'rm -rf "$work"' EXIT

# VOLCONTACT_CHECK_SHARED lists the BUILD_SHARED_LIBS values to try (CI sets OFF on Windows).
for shared in ${VOLCONTACT_CHECK_SHARED:-OFF ON}; do
  echo "== core only, BUILD_SHARED_LIBS=$shared"
  cmake -S . -B "$work/core-$shared" -G Ninja -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS=$shared -DVOLCONTACT_BUILD_BENCHMARK=OFF -DVOLCONTACT_BUILD_TESTS=OFF \
    -DVOLCONTACT_WITH_DRAKE=OFF -DVOLCONTACT_WITH_CHRONO=OFF -DVOLCONTACT_WITH_MUJOCO=OFF \
    -DCMAKE_INSTALL_PREFIX="$work/prefix-$shared"
  cmake --build "$work/core-$shared"
  cmake --install "$work/core-$shared"

  echo "== examples/minimal against the installed package"
  cmake -S examples/minimal -B "$work/example-$shared" -G Ninja -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="$work/prefix-$shared"
  cmake --build "$work/example-$shared"
  LD_LIBRARY_PATH="$work/prefix-$shared/lib:${LD_LIBRARY_PATH:-}" "$work/example-$shared/minimal"
done
# The same through FetchContent (the source tree stands in for the download): only the core builds.
echo "== examples/minimal through FetchContent"
mkdir "$work/fetch"
cat > "$work/fetch/CMakeLists.txt" <<CMAKE
cmake_minimum_required(VERSION 3.22)
project(fetch_check LANGUAGES CXX)
include(FetchContent)
FetchContent_Declare(volcontact SOURCE_DIR "$root")
FetchContent_MakeAvailable(volcontact)
add_executable(minimal "$root/examples/minimal/main.cpp")
target_link_libraries(minimal PRIVATE volcontact::core)
CMAKE
cmake -S "$work/fetch" -B "$work/fetch/build" -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build "$work/fetch/build"
"$work/fetch/build/minimal"
echo "check-install: ok"
