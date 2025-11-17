#!/usr/bin/env bash
set -e

export DISPLAY=${DISPLAY:-:0}
export QT_X11_NO_MITSHM=1

HOST_UID=${HOST_UID:-1000}
HOST_GID=${HOST_GID:-1000}
HOST_USER=${HOST_USER:-developer}

if ! getent group ${HOST_GID} >/dev/null 2>&1; then
  groupadd -g ${HOST_GID} ${HOST_USER} || true
fi

if ! id -u ${HOST_UID} >/dev/null 2>&1; then
  useradd -m -u ${HOST_UID} -g ${HOST_GID} -s /bin/bash ${HOST_USER} || true
fi

WORKSPACE=${CATKIN_WS:-/workspace}
mkdir -p ${WORKSPACE} /workspace/build /workspace/devel
chown -R ${HOST_UID}:${HOST_GID} ${WORKSPACE} /workspace/build /workspace/devel || true

source /opt/ros/noetic/setup.bash || true
if [ -f ${WORKSPACE}/devel/setup.bash ]; then
  source ${WORKSPACE}/devel/setup.bash
fi

if [ "$(id -u)" = '0' ]; then
  if [ $# -eq 0 ]; then
    exec su - ${HOST_USER} -s /bin/bash -c "bash"
  else
    # join args and run under the user
    exec su - ${HOST_USER} -s /bin/bash -c "exec \"$@\""
  fi
else
  exec "$@"
fi
