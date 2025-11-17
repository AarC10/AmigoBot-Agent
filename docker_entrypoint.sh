#!/usr/bin/env bash
set -e

export DISPLAY=${DISPLAY:-:0}
export QT_X11_NO_MITSHM=1

HOST_UID=${HOST_UID:-1000}
HOST_GID=${HOST_GID:-1000}
HOST_USER=${HOST_USER:-developer}

if [ "$(id -u)" = '0' ]; then
  if ! getent group ${HOST_GID} >/dev/null 2>&1; then
    groupadd -g ${HOST_GID} ${HOST_USER} || true
  fi

  if ! id -u ${HOST_UID} >/dev/null 2>&1; then
    useradd -m -u ${HOST_UID} -g ${HOST_GID} -s /bin/bash ${HOST_USER} || true
  fi
fi

WORKSPACE=${CATKIN_WS:-/workspace}
if [ "$(id -u)" = '0' ]; then
  mkdir -p "${WORKSPACE}" /workspace/build /workspace/devel 2>/dev/null || true
  chown -R ${HOST_UID}:${HOST_GID} "${WORKSPACE}" /workspace/build /workspace/devel 2>/dev/null || true
else
  if [ -w "$(dirname "${WORKSPACE}")" ] || [ -w "${WORKSPACE}" ]; then
    mkdir -p "${WORKSPACE}" /workspace/build /workspace/devel 2>/dev/null || true
  else
    echo "[warning] Cannot create ${WORKSPACE} or cache dirs: running as non-root and parent directory is not writable."
    echo "[hint] This script is intended to run inside the container. Don't run it directly on the host; instead run the container (docker-compose or docker run)."
  fi
fi

source /opt/ros/noetic/setup.bash || true
if [ -f ${WORKSPACE}/devel/setup.bash ]; then
  source ${WORKSPACE}/devel/setup.bash
fi

if [ "$(id -u)" = '0' ]; then
  if [ $# -eq 0 ]; then
    exec su - ${HOST_USER} -s /bin/bash -c "bash"
  else
    exec su - ${HOST_USER} -s /bin/bash -c "exec \"$@\""
  fi
else
  if [ $# -eq 0 ]; then
    exec bash
  else
    exec "$@"
  fi
fi
