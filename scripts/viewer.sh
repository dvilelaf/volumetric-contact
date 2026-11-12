#!/usr/bin/env bash
# Web viewer of the trajectories written by scripts/bench.sh (results/traj).
#   scripts/viewer.sh [PORT]     # then open http://localhost:PORT/tools/viewer/
# Serves the repository root read-only, so the meshes in data/ are reachable.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

port=${1:-8000}
[ -f results/traj/index.json ] || echo "no results/traj/index.json: run scripts/bench.sh first" >&2
echo "http://localhost:$port/tools/viewer/"
exec python3 -m http.server "$port" --bind 127.0.0.1
