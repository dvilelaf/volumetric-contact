#!/usr/bin/env bash
# Sets the GitHub account that hosts the public repository, in every tracked file that links to it.
#   scripts/set-owner.sh ACCOUNT        # replaces the placeholder OWNER (e.g. in CITATION.cff, README)
#   scripts/set-owner.sh --check        # lists the files that still hold the placeholder
# The placeholder is the word OWNER in "OWNER/volumetric-contact" and "OWNER.github.io"; nothing
# else is touched. Run it once, before publishing, and commit the result.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

pattern='OWNER/volumetric-contact\|OWNER\.github\.io'
files=$(git grep -lI -e "$pattern" -- . ':!scripts/set-owner.sh' ':!docs/publishing.md' || true)

case "${1:-}" in
--check)
  if [ -n "$files" ]; then echo "placeholder OWNER still in:"; echo "$files"; else echo "no placeholder left"; fi
  ;;
"" | -h | --help)
  sed -n '2,6p' "${BASH_SOURCE[0]}" >&2
  exit 2
  ;;
*)
  account=$1
  [[ $account =~ ^[A-Za-z0-9]([A-Za-z0-9-]*[A-Za-z0-9])?$ ]] || { echo "set-owner: not a GitHub account name: $account" >&2; exit 2; }
  [ -n "$files" ] || { echo "set-owner: nothing to replace"; exit 0; }
  # shellcheck disable=SC2086
  sed -i "s#OWNER/volumetric-contact#$account/volumetric-contact#g; s#OWNER\.github\.io#$account.github.io#g" $files
  echo "set-owner: $account in:"; echo "$files"
  ;;
esac
