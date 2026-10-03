ARG ROS_DISTRO=humble
FROM ros:humble-ros-base
ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive

# rmw-cyclonedds-cpp: the RMW devcontainer.json selects via RMW_IMPLEMENTATION; without it rcl
# refuses to start (including cargo test binaries)
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        clang \
        cmake \
        curl \
        git \
        libclang-dev \
        python3-pip \
        python3-vcstool \
        sudo \
        tmux \
        vim \
        ros-${ROS_DISTRO}-example-interfaces \
        ros-${ROS_DISTRO}-test-msgs \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*


RUN if [ "${ROS_DISTRO}" = "humble" ]; then \
        pip install --no-cache-dir --upgrade pytest \
        && pip install --no-cache-dir \
            git+https://github.com/colcon/colcon-cargo.git \
            git+https://github.com/colcon/colcon-ros-cargo.git; \
    else \
        pip install --break-system-packages --no-cache-dir pytest \
            git+https://github.com/colcon/colcon-ros-cargo.git \
            git+https://github.com/colcon/colcon-cargo.git; \
    fi

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} --create-home --shell /bin/bash ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

ENV ROS2_RUST_UNDERLAY=/opt/ros2_rust_underlay
# The repo is bind-mounted at /workspace/src/syncai_lio_bridge on top of a named volume at
# /workspace. Create the mount point here so the volume is seeded with ros-owned directories;
# otherwise Docker creates src/ as root and colcon cannot write build/ install/ log/.
RUN mkdir -p /workspace/src/syncai_lio_bridge /workspace/build /workspace/install /workspace/log \
             /home/${USERNAME}/.cargo/registry /home/${USERNAME}/.cargo/git \
             ${ROS2_RUST_UNDERLAY} \
    && chown -R ${USER_UID}:${USER_GID} /workspace /home/${USERNAME} ${ROS2_RUST_UNDERLAY}

USER ${USERNAME}
ENV HOME=/home/${USERNAME}

ARG RUST_VERSION=1.85.0
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs \
      | sh -s -- --default-toolchain ${RUST_VERSION} -y --no-modify-path --component rustfmt --component clippy
ENV PATH=${HOME}/.cargo/bin:${PATH}
# rclrs declares rust-version 1.85 but its dependencies keep moving: on a fresh resolve cargo picks
# the newest `uuid`, which already needs rustc 1.89, and the underlay build fails with "rustc
# 1.85.0 is not supported". With the fallback policy cargo prefers the newest dependency version
# whose own rust-version still fits the toolchain, so the pin above stays honest without a
# committed lock file for the underlay. (Cargo 1.84+; this is the `[resolver]
# incompatible-rust-versions` config key as an environment variable.)
ENV CARGO_RESOLVER_INCOMPATIBLE_RUST_VERSIONS=fallback


RUN mkdir -p ${ROS2_RUST_UNDERLAY}/src \
    && cd ${ROS2_RUST_UNDERLAY} \
    && curl -fsSL -o ros2_rust.repos \
        https://raw.githubusercontent.com/ros2-rust/ros2_rust/main/ros2_rust_${ROS_DISTRO}.repos \
    && vcs import src < ros2_rust.repos \
    && rm -rf src/ros2-rust/examples \
    && git clone --depth 1 https://github.com/ros2-rust/ros2_rust.git src/ros2-rust/ros2_rust

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && cd ${ROS2_RUST_UNDERLAY} \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf ${ROS2_RUST_UNDERLAY}/build ${ROS2_RUST_UNDERLAY}/log

USER root
COPY setup_env.sh /opt/ros2_rust_setup.sh
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh \
    && printf '. /opt/ros2_rust_setup.sh\n' > /etc/profile.d/10-ros2-rust.sh \
    && printf '. /opt/ros2_rust_setup.sh\n' >> /home/${USERNAME}/.bashrc

USER ${USERNAME}

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
