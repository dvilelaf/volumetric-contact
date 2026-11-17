#!/usr/bin/env bash
# Runs a command inside the build container with the repo mounted at /src.
#   scripts/dev.sh image                 # build the image
#   scripts/dev.sh cmake --preset ...    # any command
set -euo pipefail

IMAGE="${VC_IMAGE:-volumetric-contact-dev:drake-1.58.0-chrono-10.0.0-mujoco-4.0.0}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ "${1:-}" == "image" ]]; then
  exec docker build --pull -t "$IMAGE" "$ROOT/docker"
fi

# No local image: pull the published one and give it the local name.
if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  REMOTE="${VC_REMOTE_IMAGE:-ghcr.io/dvilelaf/volumetric-contact-dev:${IMAGE##*:}}"
  echo "Image $IMAGE not found; pulling $REMOTE" >&2
  if docker pull "$REMOTE" >&2; then
    docker tag "$REMOTE" "$IMAGE"
  else
    echo "Could not pull it. Build it locally with: scripts/dev.sh image" >&2
    exit 1
  fi
fi

# The compiler cache is shared by all worktrees of this machine.
CCACHE="${VC_CCACHE_DIR:-$HOME/.cache/volcontact-ccache}"
mkdir -p "$CCACHE"

exec docker run --rm -i \
  --user "$(id -u):$(id -g)" \
  -e HOME=/tmp -e CCACHE_DIR=/ccache \
  -v "$ROOT:/src" -v "$CCACHE:/ccache" -w /src \
  "$IMAGE" "$@"
