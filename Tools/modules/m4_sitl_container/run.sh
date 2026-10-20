#!/bin/bash
# Runs one command in the linux/amd64 SITL container, in /work: a copy of this
# worktree refreshed before every run (build/ persists in the volume, so the
# Mac's build/sitl is never shared). usage: run.sh '<command>'
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../../.." && pwd)"
IMAGE=module-m4-sitl:1
VOLUME=module-m4-sitl-work
docker info > /dev/null 2>&1 || { echo "run.sh: Docker is not running" >&2; exit 2; }
docker image inspect "$IMAGE" > /dev/null 2>&1 || docker build --platform linux/amd64 -t "$IMAGE" "$HERE"
docker volume inspect "$VOLUME" > /dev/null 2>&1 || docker volume create "$VOLUME" > /dev/null
# waf reads the version from git: /work gets a one-commit repository of its own
exec docker run --rm --platform linux/amd64 --name module-m4-sitl \
    -v "$ROOT:/src:ro" -v "$VOLUME:/work" -w /work "$IMAGE" bash -c '
    set -e
    rsync -a --delete --exclude /build/ --exclude .git --exclude __pycache__/ /src/ /work/
    if [ ! -d /work/.git ]; then
        git init -q /work && git -C /work -c user.name=sitl -c user.email=sitl@localhost commit -q --allow-empty -m snapshot
    fi
    '"$1"
