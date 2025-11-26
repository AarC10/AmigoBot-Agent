#!/usr/bin/env bash
set -euo pipefail

WORKSPACE=${CATKIN_WS:-/workspace}
cd "$WORKSPACE"

echo "Removing build artifacts in $WORKSPACE"
rm -rf build devel install

echo "Clean finished."
