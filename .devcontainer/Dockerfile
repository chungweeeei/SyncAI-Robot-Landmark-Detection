ARG ROS_DISTRO=humble
FROM ros:humble-ros-base
ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive

# - libopencv-dev / libopencv-contrib-dev: cv::aruco is in the contrib module, and Ubuntu 22.04's
#   OpenCV 4.5.4 has the pre-4.7 aruco API (cv::aruco::detectMarkers, not cv::aruco::ArucoDetector),
#   the same OpenCV the robot image gets from apt. calib3d (solvePnP, fisheye) is in libopencv-dev.
# - ros-*-pluginlib: the detector plugins (one per detection method) are loaded through it
# - ros-*-tf2-ros / tf2-geometry-msgs: the camera -> target_frame transform at the image stamp
# - rmw-cyclonedds-cpp: the RMW devcontainer.json selects via RMW_IMPLEMENTATION; without it rcl
#   refuses to start
# - clang-format: the version CI's format job also pins (Ubuntu 22.04's clang-format 14)
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        clang-format \
        cmake \
        curl \
        gdb \
        git \
        libopencv-contrib-dev \
        libopencv-dev \
        python3-pip \
        python3-vcstool \
        sudo \
        tmux \
        vim \
        ros-${ROS_DISTRO}-pluginlib \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
        ros-${ROS_DISTRO}-tf2-geometry-msgs \
        ros-${ROS_DISTRO}-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} --create-home --shell /bin/bash ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# The repo is bind-mounted at /workspace/src/syncai_landmark_detection on top of a named volume at
# /workspace. Create the mount point here so the volume is seeded with ros-owned directories;
# otherwise Docker creates src/ as root and nothing (e.g. a vcs-imported package) can be added
# next to this one.
RUN mkdir -p /workspace/src/syncai_landmark_detection /workspace/build /workspace/install \
             /workspace/log \
    && chown -R ${USER_UID}:${USER_GID} /workspace /home/${USERNAME}

COPY setup_env.sh /opt/ros_setup.sh
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh \
    && printf '. /opt/ros_setup.sh\n' > /etc/profile.d/10-ros.sh \
    && printf '. /opt/ros_setup.sh\n' >> /home/${USERNAME}/.bashrc

USER ${USERNAME}
ENV HOME=/home/${USERNAME}

ARG INSTALL_CLAUDE_CODE=false
ENV CLAUDE_CONFIG_DIR=${HOME}/.claude
ENV PATH=${HOME}/.local/bin:${PATH}
RUN mkdir -p ${CLAUDE_CONFIG_DIR} \
    && if [ "${INSTALL_CLAUDE_CODE}" = "true" ]; then \
        curl -fsSL https://claude.ai/install.sh | bash; \
    fi

WORKDIR /workspace
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
