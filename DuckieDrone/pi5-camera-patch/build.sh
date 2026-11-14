#!/usr/bin/env bash
# Build the patched dt-duckiebot-interface image ON THE DRONE, tagged separately from the official one.
#
#   ./build.sh          # start the build in the background on the drone (survives SSH drops)
#   ./build.sh status   # show progress / result
#
# Reproduces the official build inputs (same base image, same build args as the labels on
# duckietown/dt-duckiebot-interface:ente-arm64v8), changing only the source commit.
set -euo pipefail

DRONE="${DRONE:-duckie@amelia02.local}"
REPO="https://github.com/MarcusYearwood/dt-duckiebot-interface.git"
BRANCH="pi5-camera-pisp"
# Upstream is the source of the Git LFS files (icons); the drone has no git-lfs.
LFS_MEDIA="https://media.githubusercontent.com/media/duckietown/dt-duckiebot-interface"
LFS_REF="5d0d705b59dff9fffbf6b4e8f1d12e2a1b988518"  # base commit of the patch = official image source
TAG="local/dt-duckiebot-interface:ente-pi5"
DIR="/home/duckie/build/dt-duckiebot-interface"
LOG="/home/duckie/build/build.log"

on_drone() { ssh -o BatchMode=yes -o ConnectTimeout=10 "$DRONE" "$@"; }

if [ "${1:-}" = status ]; then
    on_drone "tail -n 15 $LOG; echo; docker images $TAG --format '{{.Repository}}:{{.Tag}} {{.ID}} {{.CreatedSince}} {{.Size}}'"
    exit 0
fi

on_drone "bash -s" <<EOF
set -euo pipefail
mkdir -p /home/duckie/build
rm -rf "$DIR"
git clone --quiet --depth 1 --branch "$BRANCH" "$REPO" "$DIR"
cd "$DIR"
SHA=\$(git rev-parse HEAD)
echo "source: $REPO @ $BRANCH = \$SHA"

# Replace Git LFS pointer files with the real content, verifying each sha256 against the pointer.
for f in \$(grep -rl --include='*.png' --include='*.jpg' --include='*.gif' '^version https://git-lfs' . || true); do
    oid=\$(sed -n 's/^oid sha256://p' "\$f")
    curl -fsSL "$LFS_MEDIA/$LFS_REF/\${f#./}" -o "\$f.tmp"
    echo "\$oid  \$f.tmp" | sha256sum -c --quiet - && mv "\$f.tmp" "\$f" || { echo "LFS checksum mismatch: \$f"; exit 1; }
done
echo "LFS files resolved: \$(grep -rl '^version https://git-lfs' . | wc -l) pointers left"

nohup docker build -t "$TAG" \
    --build-arg PROJECT_NAME=dt-duckiebot-interface \
    --build-arg PROJECT_DESCRIPTION="Contains all the drivers needed to communicate with sensors and actuators on a Duckietown device" \
    --build-arg PROJECT_MAINTAINER="Andrea F. Daniele (afdaniele@duckietown.com)" \
    --build-arg PROJECT_ICON=wrench \
    --build-arg PROJECT_FORMAT_VERSION=4 \
    --build-arg ARCH=arm64v8 \
    --build-arg DISTRO=ente \
    --build-arg DOCKER_REGISTRY=docker.io \
    --build-arg BASE_REPOSITORY=dt-commons \
    --build-arg BASE_TAG=ente-arm64v8 \
    --build-arg TARGETPLATFORM=linux/arm64 --build-arg TARGETOS=linux --build-arg TARGETARCH=arm64 \
    --label org.duckietown.label.code.sha=\$SHA \
    --label org.duckietown.label.code.vcs=git \
    . > "$LOG" 2>&1 &
echo "build started (pid \$!); log: $LOG"
EOF
