#!/bin/bash
# Runs one command in the linux/amd64 SITL container, in /work: a copy of this
# worktree refreshed before every run (build/ persists in the volume, so the
# Mac's build/sitl is never shared). usage: run.sh '<command>'
# run.sh --udp-out '<command>': the command also gets
# MODULE_M4_SITL_OUT=udp:host.docker.internal:14550, the host's UDP port 14550, for a SITL
# MAVLink output (outbound only: no port is published)
set -euo pipefail
ENVIRONMENT=()
if [ "${1:-}" = --udp-out ]; then
    ENVIRONMENT=(-e MODULE_M4_SITL_OUT=udp:host.docker.internal:14550)
    shift
fi
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../../.." && pwd)"
# the tag follows the Dockerfile, so an edited Dockerfile builds a new image
TAG="$( (sha256sum "$HERE/Dockerfile" 2> /dev/null || shasum -a 256 "$HERE/Dockerfile") | cut -c1-12)"
IMAGE="module-m4-sitl:$TAG"
NAME=module-m4-sitl
VOLUME=module-m4-sitl-work
docker info > /dev/null 2>&1 || { echo "run.sh: Docker is not running" >&2; exit 2; }
docker image inspect "$IMAGE" > /dev/null 2>&1 || docker build --platform linux/amd64 -t "$IMAGE" "$HERE"
docker volume inspect "$VOLUME" > /dev/null 2>&1 || docker volume create "$VOLUME" > /dev/null
# a run whose client was killed can leave its container behind
if docker container inspect "$NAME" > /dev/null 2>&1; then
    docker rm -f "$NAME" > /dev/null
fi
# waf reads the version from git: /work gets a one-commit repository of its own
exec docker run --rm --init --platform linux/amd64 --name "$NAME" ${ENVIRONMENT[@]+"${ENVIRONMENT[@]}"} \
    -v "$ROOT:/src:ro" -v "$VOLUME:/work" -w /work "$IMAGE" bash -c '
    set -e
    rsync -a --delete --exclude /build/ --exclude .git --exclude __pycache__/ /src/ /work/
    if [ ! -d /work/.git ]; then
        git init -q /work && git -C /work -c user.name=sitl -c user.email=sitl@localhost commit -q --allow-empty -m snapshot
    fi
    '"$1"
