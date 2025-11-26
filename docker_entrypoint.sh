#!/usr/bin/env bash
set -e

export DISPLAY=${DISPLAY:-:0}
export QT_X11_NO_MITSHM=1

HOST_UID=${HOST_UID:-1000}
HOST_GID=${HOST_GID:-1000}
HOST_USER=${HOST_USER:-rosdev}

# Only run user/group management as root
if [ "$(id -u)" = '0' ]; then
  # If the group id is already present map to it; otherwise create the group
  if getent group ${HOST_GID} >/dev/null 2>&1; then
    EXISTING_GROUP=$(getent group ${HOST_GID} | cut -d: -f1)
    # If existing group name differs from intended, reuse it
    GROUP_NAME=${EXISTING_GROUP}
  else
    groupadd -g ${HOST_GID} ${HOST_USER} || true
    GROUP_NAME=${HOST_USER}
  fi

  # If user with HOST_UID exists, use it; otherwise create or modify rosdev
  if id -u ${HOST_UID} >/dev/null 2>&1; then
    EXISTING_USER=$(getent passwd ${HOST_UID} | cut -d: -f1)
    USER_NAME=${EXISTING_USER}
  else
    # If our default user exists, try to modify it; otherwise create a new user
    if id -u ${HOST_USER} >/dev/null 2>&1; then
      usermod -u ${HOST_UID} -g ${GROUP_NAME} ${HOST_USER} || true
      USER_NAME=${HOST_USER}
    else
      useradd -m -u ${HOST_UID} -g ${GROUP_NAME} -s /bin/bash ${HOST_USER} || true
      USER_NAME=${HOST_USER}
    fi
  fi
fi

WORKSPACE=${CATKIN_WS:-/workspace}
# Ensure workspace and common catkin-derived dirs exist and have correct ownership
if [ "$(id -u)" = '0' ]; then
  mkdir -p "${WORKSPACE}" "${WORKSPACE}/build" "${WORKSPACE}/devel" "${WORKSPACE}/install" 2>/dev/null || true
  # Only chown build artifacts if they are currently root-owned (cheap)
  for d in build devel install; do
    TARGET="${WORKSPACE}/${d}"
    if [ -d "${TARGET}" ]; then
      OWNER_UID=$(stat -c %u "${TARGET}" 2>/dev/null || true)
      if [ "${OWNER_UID}" = "0" ]; then
        chown -R ${HOST_UID}:${HOST_GID} "${TARGET}" 2>/dev/null || true
      fi
    else
      mkdir -p "${TARGET}" && chown ${HOST_UID}:${HOST_GID} "${TARGET}" 2>/dev/null || true
    fi
  done
  # If workspace root itself is owned by root (e.g., image), change ownership to host user
  WS_OWNER_UID=$(stat -c %u "${WORKSPACE}" 2>/dev/null || true)
  if [ "${WS_OWNER_UID}" = "0" ]; then
    chown ${HOST_UID}:${HOST_GID} "${WORKSPACE}" 2>/dev/null || true
  fi
else
  # Running as non-root inside container (rare) - try to create dirs if parent is writable
  if [ -w "$(dirname "${WORKSPACE}")" ] || [ -w "${WORKSPACE}" ]; then
    mkdir -p "${WORKSPACE}" "${WORKSPACE}/build" "${WORKSPACE}/devel" "${WORKSPACE}/install" 2>/dev/null || true
  else
    echo "[warning] Cannot create ${WORKSPACE} or cache dirs: running as non-root and parent directory is not writable."
    echo "[hint] This script is intended to run inside the container. Don't run it directly on the host; instead run the container (docker-compose or docker run)."
  fi
fi

# Load ROS environment and workspace setup if present
source /opt/ros/noetic/setup.bash || true
if [ -f ${WORKSPACE}/devel/setup.bash ]; then
  source ${WORKSPACE}/devel/setup.bash
fi

# Drop privileges and execute the requested command as the mapped user
if [ "$(id -u)" = '0' ]; then
  if [ $# -eq 0 ]; then
    exec gosu ${USER_NAME} bash
  else
    exec gosu ${USER_NAME} "$@"
  fi
else
  if [ $# -eq 0 ]; then
    exec bash
  else
    exec "$@"
  fi
fi
