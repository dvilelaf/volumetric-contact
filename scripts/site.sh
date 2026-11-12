#!/usr/bin/env bash
# Assembles the static site into build/site/ (GitHub Pages serves it as it is).
#   scripts/site.sh                 assemble build/site
#   scripts/site.sh --serve [PORT]  assemble, then serve it at http://localhost:PORT/ (default 8000)
#
# Layout of build/site:
#   index.html            landing page (site/), results table generated from results/stats.json
#   viewer/               tools/viewer, reading ../data/traj and the meshes from ../data
#   data/traj/            slim trajectories (site/data/traj, from tools/viewer/export_site_data.py)
#   data/bodies, grounds  the surface.obj meshes the trajectories refer to
#   paper.pdf, supplement.pdf   only if scripts/paper.sh has built them (paper/main.pdf, paper/supplement.pdf)
#   figures/              only if results/figures exists
# Environment: VC_STATS (default results/stats.json), VC_FIGURES (default results/figures).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$ROOT/build/site"
STATS="${VC_STATS:-$ROOT/results/stats.json}"
FIGURES="${VC_FIGURES:-$ROOT/results/figures}"

[ -f "$ROOT/site/data/traj/index.json" ] || {
  echo "site.sh: site/data/traj is missing; run: uv run tools/viewer/export_site_data.py" >&2
  exit 2
}

rm -rf "$OUT"
mkdir -p "$OUT/data"

# The viewer, pointed at the slim data one level up.
cp -r "$ROOT/tools/viewer" "$OUT/viewer"
rm -rf "$OUT/viewer/__pycache__" "$OUT/viewer/"*.py
sed -i 's|data-traj="[^"]*"|data-traj="../data/traj/"|; s|data-assets="[^"]*"|data-assets="../"|' "$OUT/viewer/index.html"

cp -r "$ROOT/site/data/traj" "$OUT/data/traj"
(cd "$ROOT" && find data -name surface.obj -print0 | xargs -0 cp --parents -t "$OUT")

cp "$ROOT/site/site.css" "$OUT/site.css"
[ -f "$ROOT/paper/main.pdf" ] && cp "$ROOT/paper/main.pdf" "$OUT/paper.pdf"
[ -f "$ROOT/paper/supplement.pdf" ] && cp "$ROOT/paper/supplement.pdf" "$OUT/supplement.pdf"
if [ -d "$FIGURES" ]; then
  mkdir -p "$OUT/figures"
  find "$FIGURES" -maxdepth 1 \( -name '*.png' -o -name '*.svg' \) -exec cp {} "$OUT/figures/" \;
fi
touch "$OUT/.nojekyll"

uv run "$ROOT/site/build_page.py" --out "$OUT" --stats "$STATS" --figures "$FIGURES"
echo "site.sh: $(du -sh "$OUT" | cut -f1) in $OUT"

if [ "${1:-}" = "--serve" ]; then
  PORT="${2:-8000}"
  echo "site.sh: http://localhost:$PORT/"
  exec uv run python -m http.server "$PORT" --directory "$OUT"
fi
