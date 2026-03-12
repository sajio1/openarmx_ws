FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-controller-manager \
    ros-humble-rosbridge-server \
    ros-humble-rviz2 \
    ros-humble-moveit \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    can-utils \
    iproute2 \
    git \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir \
    placo \
    numpy \
    PySide6 \
    && pip3 install --no-cache-dir placo-utils 2>/dev/null || true

COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

WORKDIR /ws
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
