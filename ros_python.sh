#!/usr/bin/env bash

# using this for python wrapper for IDE support
set -e

source /opt/ros/noetic/setup.bash

if [ -f /workspace/devel/setup.bash ]; then
  source /workspace/devel/setup.bash
fi

exec python3 "$@"
