ARG OMEGA_PRIME_VERSION=
ARG PERCEPTION_INTERFACES_VERSION=

FROM osrf/ros:jazzy-desktop

ARG OMEGA_PRIME_VERSION
ARG PERCEPTION_INTERFACES_VERSION

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    ROS_DISTRO=jazzy

# System deps and ROS packages needed to build/use perception_msgs and utils
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-venv \
    python3-colcon-common-extensions \
    ros-jazzy-rosidl-default-generators \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-tf-transformations \
    ros-jazzy-rosbag2-py \
    ros-jazzy-rosbag2-storage-mcap \
    && rm -rf /var/lib/apt/lists/*

# Workspace for ROS Python messages and utils
WORKDIR /opt/ws
RUN mkdir -p /opt/ws/src

# Fetch perception_interfaces from GitHub so the image can be built standalone.
SHELL ["/bin/bash", "-c"]
RUN git clone https://github.com/ika-rwth-aachen/perception_interfaces.git /opt/ws/src/perception_interfaces && \
    if [ -n "${PERCEPTION_INTERFACES_VERSION}" ]; then \
      cd /opt/ws/src/perception_interfaces && \
      git checkout ${PERCEPTION_INTERFACES_VERSION}; \
    fi

# Build only the required packages
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build \
      --merge-install \
      --symlink-install \
      --packages-up-to \
        perception_msgs \
        perception_msgs_utils \
        tf2_perception_msgs

# Python deps: omega-prime from PyPI inside isolated venv
WORKDIR /opt
RUN python3 -m venv /opt/venv
ENV VIRTUAL_ENV=/opt/venv
ENV PATH="/opt/venv/bin:${PATH}"
RUN python -m pip install --upgrade pip && \
    python -m pip install --upgrade scipy pyyaml transforms3d && \
    if [ -n "${OMEGA_PRIME_VERSION}" ]; then \
      python -m pip install "omega-prime==${OMEGA_PRIME_VERSION}"; \
    else \
      python -m pip install omega-prime; \
    fi

# Include the converter inside the image
RUN mkdir -p /opt/omega-prime-ros
COPY ros_to_omega_prime.py /opt/omega-prime-ros/ros_to_omega_prime.py

# Convenience entrypoint to ensure ROS env is sourced
RUN printf '%s\n' \
    '#!/bin/bash' \
    'set -e' \
    'source /opt/ros/${ROS_DISTRO}/setup.bash' \
    'if [ -f /opt/ws/install/setup.bash ]; then' \
    '  source /opt/ws/install/setup.bash' \
    'fi' \
    'exec "$@"' \
    > /ros_entrypoint.sh && chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["python3", "/opt/omega-prime-ros/ros_to_omega_prime.py"]
