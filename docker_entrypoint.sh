#!/usr/bin/env bash
set -e
export DISPLAY=${DISPLAY:-:0}
export QT_X11_NO_MITSHM=1
source /opt/ros/noetic/setup.bash
if [ -f /root/catkin_ws/devel/setup.bash ]; then
  source /root/catkin_ws/devel/setup.bash
fi
exec "$@"
