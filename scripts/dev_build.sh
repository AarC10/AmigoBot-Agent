#!/usr/bin/env bash
set -euo pipefail

# Dev build helper for running inside the container.
# Usage: from inside container: /workspace/scripts/dev_build.sh

WORKSPACE=${CATKIN_WS:-/workspace}
cd "$WORKSPACE"

echo "Workspace: $WORKSPACE"

# Ensure src exists
if [ ! -d src ]; then
  echo "Creating src/"
  mkdir -p src
fi

# Update rosdep and install package dependencies
if command -v rosdep >/dev/null 2>&1; then
  echo "Updating rosdep (may require network)..."
  sudo rosdep update || true
  echo "Installing system dependencies for packages in src/ (may ask for sudo)..."
  rosdep install --from-paths src --ignore-src -r -y || true
else
  echo "rosdep not found in container PATH. Make sure ROS is installed." >&2
fi

# Ensure /workspace/build, devel, install exist and are writable by current user
for d in build devel install; do
  if [ ! -d "$WORKSPACE/$d" ]; then
    mkdir -p "$WORKSPACE/$d" || true
  fi
  if [ ! -w "$WORKSPACE/$d" ]; then
    echo "Directory $WORKSPACE/$d is not writable by $(id -u): trying to fix with sudo chown..."
    sudo chown -R $(id -u):$(id -g) "$WORKSPACE/$d" || true
  fi
done

# Build
echo "Running catkin_make in $WORKSPACE"
catkin_make -j"$(nproc)"

# Source overlay
if [ -f "$WORKSPACE/devel/setup.bash" ]; then
  echo "Sourcing $WORKSPACE/devel/setup.bash"
  # shellcheck disable=SC1090
  source "$WORKSPACE/devel/setup.bash"
fi

echo "Build finished."
