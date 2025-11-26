FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive \
    TERM=xterm-256color \
    ROS_DISTRO=noetic \
    CATKIN_WS=/workspace

RUN apt-get update && apt-get install -y --no-install-recommends \
    git build-essential cmake make python3-pip python3-rosdep \
    ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control \
    ros-noetic-joint-state-publisher-gui ros-noetic-robot-state-publisher \
    ros-noetic-xacro doxygen graphviz \
    sudo procps locales gosu \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true && rosdep update

# Workspace skeleton
RUN mkdir -p ${CATKIN_WS}/src
WORKDIR ${CATKIN_WS}

RUN groupadd -g 1000 rosdev || true \
    && useradd -m -u 1000 -g 1000 -s /bin/bash rosdev || true

RUN git clone --depth=1 https://github.com/reedhedges/AriaCoda.git /tmp/AriaCoda && \
    make -C /tmp/AriaCoda -j"$(nproc)" && \
    make -C /tmp/AriaCoda install && \
    ldconfig
ENV LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}

COPY docker_entrypoint.sh /ros_entry.sh
RUN chmod +x /ros_entry.sh
ENTRYPOINT ["/ros_entry.sh"]
CMD ["bash"]
