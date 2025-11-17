#!/usr/bin/env bash
# Helper: prints instructions for adding a Docker Compose interpreter in PyCharm
# Optionally uses the JetBrains Toolbox / IDE commandline plugin to automate if available.

echo "This script helps you configure PyCharm to use the 'amigo' service from docker-compose as a remote interpreter."

echo "1) Start the compose services (if not already running):"
echo "   docker-compose up -d"

echo "2) In PyCharm: Preferences -> Project -> Python Interpreter -> Add -> Docker Compose"
echo "   - Choose the repository's docker-compose.yml"
echo "   - Select the service: amigo"
echo "   - Interpreter path: /usr/bin/python3"
echo "   - Path mapping: <local project root> -> /root/catkin_ws/src"

echo "If you have the JetBrains 'remote-dev' or CLI tools installed, you can automate parts of this, but that's environment specific."

echo "Quick test: run this to exec into the container and check python:"

echo "  docker-compose exec amigo bash -lc 'python3 -c \"import sys; print(sys.executable); import rospy; print(\'rospy OK\')\"'"
version: '3.8'
services:
  amigo:
    environment:
      - DISPLAY=${DISPLAY}
      - QT_X11_NO_MITSHM=1
      - LIBGL_ALWAYS_SOFTWARE=1
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ./:/root/catkin_ws/src:rw
      - catkin_build:/root/catkin_ws/build
      - catkin_devel:/root/catkin_ws/devel
    network_mode: host

