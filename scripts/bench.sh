#!/usr/bin/env bash
# The whole battery: every engine × contact method × scenario × variant, then the statistics.
#   scripts/bench.sh [--scenario NAME[,NAME...]] [-j N] [--quick] [--resume] [--timing]
#                    [extra volcontact_bench options]
# Accuracy pass: one process per scenario, in parallel, one repeat, rows merged into
# results/battery.csv and the terms of every error into results/terms.csv; trajectories for
# scripts/viewer.sh go to results/traj. The matrix (docs/scenarios.md §2):
#   methods   gonthier, gonthier-thesis, distributed, distributed-vertex, distributed-centroid, pcm, and every native model of each engine
#   variants  base; mesh32, mesh128 (curved scenarios only); dt2, dt0.5, dt0.2, dt0.1;
#             stiff (K × 10 at 0.2 ms; not class C); perturbed1 … perturbed8
# Every run has a wall-clock limit; one that exceeds it is aborted and recorded as `timeout`,
# so a slow combination never blocks the rest. Seconds, overridable from the environment:
#   LAW_LIMIT=1800            the volumetric laws, every variant
#   NATIVE_LIMIT=3600         the native models, base variant
#   NATIVE_SERIES_LIMIT=600   the native models, every other variant (they are the slowest runs
#                             and the series multiply their cost: a Drake native run at dt 0.1 ms
#                             would take ten times its base wall time)
# --quick runs the base variant only (a check of the harness, not a result). --resume keeps the
# rows already in results/parts with status ok and runs only the rest (after an interruption or
# a raised limit). With --timing, a serial timing pass follows (base variant, 5 repeats, rows in
# results/timing.csv): run it only on an idle machine, and only with the owner's go-ahead.
# Then results/stats.md, stats.json and results/figures.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

LAW_LIMIT=${LAW_LIMIT:-1800}
NATIVE_LIMIT=${NATIVE_LIMIT:-3600}
NATIVE_SERIES_LIMIT=${NATIVE_SERIES_LIMIT:-600}
LAWS=gonthier,gonthier-thesis,distributed,distributed-vertex,distributed-centroid,pcm
SERIES=mesh32,mesh128,dt2,dt0.5,dt0.2,dt0.1,stiff,perturbed1,perturbed2,perturbed3,perturbed4,perturbed5,perturbed6,perturbed7,perturbed8

parallel=$(( $(nproc) / 2 ))
[ "$parallel" -ge 1 ] || parallel=1
scenarios=all
timing=0
quick=0
resume=0
extra=()
while [ $# -gt 0 ]; do
  case $1 in
    --scenario) scenarios=$2; shift 2 ;;
    -j) parallel=$2; shift 2 ;;
    --timing) timing=1; shift ;;
    --quick) quick=1; shift ;;
    --resume) resume=1; shift ;;
    *) extra+=("$1"); shift ;;
  esac
done

uv run tools/meshgen.py --missing-volumes   # Drake native contact needs every volume.vtk
scripts/dev.sh cmake --build build
mkdir -p results/parts/terms
if (( resume )); then
  # Everything is kept; only the merged outputs are rebuilt.
  rm -f results/battery.csv results/terms.csv results/stats.md results/stats.json
else
  find results/parts -type f -delete
  rm -rf results/traj results/figures
  rm -f results/battery.csv results/terms.csv results/timing.csv results/timing.log results/load.txt results/stats.md results/stats.json
  echo "load before: $(cut -d' ' -f1-3 /proc/loadavg)" | tee results/load.txt
fi

if [ "$scenarios" = all ]; then
  mapfile -t names < <(scripts/dev.sh ./build/apps/volcontact_bench --list)
else
  IFS=, read -ra names <<< "$scenarios"
fi

# One bench process per call; a failure is reported and the rest goes on.
bench() { # bench NAME LABEL bench-options...: rows into the scenario's parts
  local name=$1 label=$2; shift 2
  local resume_flag=()
  (( resume )) && [ -s "results/parts/$name.csv" ] && resume_flag=(--resume)
  scripts/dev.sh ./build/apps/volcontact_bench --repeat 1 --scenario "$name" \
    --csv "results/parts/$name.csv" --terms "results/parts/terms/$name.csv" --traj results/traj \
    "${resume_flag[@]}" "$@" "${extra[@]}" </dev/null >>"results/parts/$name.log" 2>&1 ||
    echo "$name ($label): bench exited $? (see results/parts/$name.log)" >&2
}
accuracy() { # accuracy NAME: the matrix of one scenario
  local name=$1
  if (( quick )); then
    bench "$name" quick --contact "$LAWS,native:all" --variant base --time-limit "$NATIVE_LIMIT"
  else
    bench "$name" laws --contact "$LAWS" --variant all --time-limit "$LAW_LIMIT"
    bench "$name" natives-base --contact native:all --variant base --time-limit "$NATIVE_LIMIT"
    bench "$name" natives-series --contact native:all --variant "$SERIES" --time-limit "$NATIVE_SERIES_LIMIT"
  fi
}
echo "== accuracy pass: ${#names[@]} scenarios, $parallel at a time$( ((quick)) && echo ', quick (base only)')"
for name in "${names[@]}"; do
  while (( $(jobs -rp | wc -l) >= parallel )); do wait -n || true; done
  accuracy "$name" &
done
wait

parts=(results/parts/*.csv)
[ -e "${parts[0]}" ] || { echo "no battery rows" >&2; exit 1; }
{ head -n 1 "${parts[0]}"; for p in "${parts[@]}"; do tail -n +2 "$p"; done; } > results/battery.csv
terms=(results/parts/terms/*.csv)
if [ -e "${terms[0]}" ]; then
  { head -n 1 "${terms[0]}"; for p in "${terms[@]}"; do tail -n +2 "$p"; done; } > results/terms.csv
fi

if (( timing )); then
  echo "== timing pass: base variant, 5 repeats, serial"
  for name in "${names[@]}"; do
    scripts/dev.sh ./build/apps/volcontact_bench --variant base --repeat 5 --scenario "$name" \
      --contact "gonthier,gonthier-thesis,distributed,pcm,native:all" --time-limit "$NATIVE_LIMIT" \
      --csv results/timing.csv "${extra[@]}" </dev/null >>results/timing.log 2>&1 ||
      echo "$name: timing exited $? (see results/timing.log)" >&2
  done
fi

uv run tools/viewer/index.py results/traj || echo "viewer index failed" >&2
echo "load after: $(cut -d' ' -f1-3 /proc/loadavg)" | tee -a results/load.txt
uv run tools/stats.py results/battery.csv
