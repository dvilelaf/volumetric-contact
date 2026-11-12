#!/usr/bin/env bash
# Builds the paper (paper/main.tex) and its supplementary material (paper/supplement.tex).
#   scripts/paper.sh            build paper/main.pdf and paper/supplement.pdf
#   scripts/paper.sh check      run the numerical checks of the derivations (paper/checks)
#   scripts/paper.sh refs       re-verify paper/references.bib against Crossref (needs network)
#   scripts/paper.sh clean      remove the build products
# Nothing is installed on the host: the TeX engine is Tectonic, one static binary
# downloaded once (checksum pinned) to a cache directory, which fetches the TeX
# packages it needs from its own versioned bundle. The same script runs in CI.
# Set TECTONIC to use a binary of your own, VC_PAPER_CACHE to move the cache.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PAPER="$ROOT/paper"
VERSION=0.17.0
SHA256=8533d07f9ccbd7a65824b9e0459041bca34af1eb33daba48f59215593753a3b7 # x86_64-unknown-linux-musl
CACHE="${VC_PAPER_CACHE:-${XDG_CACHE_HOME:-$HOME/.cache}/volcontact-paper}"

tectonic() {
  if [ -n "${TECTONIC:-}" ]; then
    "$TECTONIC" "$@"
    return
  fi
  local dir="$CACHE/tectonic-$VERSION"
  if [ ! -x "$dir/tectonic" ]; then
    [ "$(uname -sm)" = "Linux x86_64" ] || {
      echo "paper.sh: no pinned Tectonic for $(uname -sm); install Tectonic $VERSION and set TECTONIC" >&2
      exit 2
    }
    mkdir -p "$dir"
    local archive="$dir/tectonic.tar.gz"
    curl -fsSL -o "$archive" \
      "https://github.com/tectonic-typesetting/tectonic/releases/download/tectonic%40$VERSION/tectonic-$VERSION-x86_64-unknown-linux-musl.tar.gz"
    echo "$SHA256  $archive" | sha256sum -c --quiet
    tar -xzf "$archive" -C "$dir" tectonic
    rm "$archive"
  fi
  "$dir/tectonic" "$@"
}

case "${1:-build}" in
build)
  out="$PAPER/build"
  mkdir -p "$out"
  cd "$PAPER"
  for doc in main supplement; do
    tectonic -X compile "$doc.tex" --outdir "$out" --keep-logs -Z search-path=template
    cp "$out/$doc.pdf" "$PAPER/$doc.pdf"
    # A document with a dangling citation or reference is not built.
    if grep -E "Citation .* undefined|Reference .* undefined|I didn't find a database entry" "$out/$doc.log" "$out/$doc.blg"; then
      echo "paper.sh: undefined citations or references in $doc.tex (above)" >&2
      exit 1
    fi
    grep -E "^(TODO markers|Output written)" "$out/$doc.log" || true
    echo "paper.sh: $PAPER/$doc.pdf"
  done
  ;;
check)
  for script in "$PAPER"/checks/check_*.py; do
    echo "== $(basename "$script")"
    uv run --quiet "$script"
  done
  ;;
refs)
  uv run --quiet "$PAPER/checks/verify_references.py" "$PAPER/references.bib"
  ;;
clean)
  rm -rf "$PAPER/build" "$PAPER/main.pdf" "$PAPER/supplement.pdf"
  ;;
*)
  sed -n '2,9p' "${BASH_SOURCE[0]}" >&2
  exit 2
  ;;
esac
