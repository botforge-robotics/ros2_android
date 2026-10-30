# Multi-architecture Dockerfile to build Rosdroid ARM64 Ubuntu 24.04 + ROS 2 Jazzy Rootfs
FROM --platform=linux/arm64 ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=jazzy

# 1. Base packages & locales
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    ca-certificates \
    build-essential \
    cmake \
    git \
    python3-pip \
    net-tools \
    iproute2 \
    sudo \
    nano \
    dropbear \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# 2. Add ROS 2 Jazzy Repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" > /etc/apt/sources.list.d/ros2.list

# 3. Install ROS 2 Jazzy minimal & CycloneDDS
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-base \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# 4. Initialize rosdep
RUN rosdep init || true \
    && rosdep update

# 5. Build Micro-ROS Agent
RUN mkdir -p /root/uros_ws/src \
    && cd /root/uros_ws/src \
    && git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git \
    && cd /root/uros_ws \
    && . /opt/ros/jazzy/setup.sh \
    && colcon build \
    && . install/setup.sh \
    && ros2 run micro_ros_setup create_agent_ws.sh \
    && ros2 run micro_ros_setup build_agent.sh \
    && rm -rf /root/uros_ws/build /root/uros_ws/log

# 6. Copy and compile patch_getifaddrs
COPY configs/patch_getifaddrs.c /tmp/patch_getifaddrs.c
RUN gcc -shared -fPIC -o /usr/local/lib/patch_getifaddrs.so /tmp/patch_getifaddrs.c -ldl \
    && echo "/usr/local/lib/patch_getifaddrs.so" > /etc/ld.so.preload \
    && rm /tmp/patch_getifaddrs.c

# 7. Create default rosdroid user
RUN useradd -m -s /bin/bash -u 1000 rosdroid \
    && echo "rosdroid:rosdroid" | chpasswd \
    && echo "root:rosdroid" | chpasswd \
    && echo "rosdroid ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && useradd -m -s /bin/bash -u 1001 -g rosdroid robot \
    && echo "robot:rosdroid" | chpasswd \
    && echo "robot ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && ln -s /home/rosdroid /home/robot_alias

# 8. Setup default environment
COPY configs/cyclonedds.xml /etc/cyclonedds/cyclonedds.xml
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc \
    && echo "source /root/uros_ws/install/setup.bash" >> /root/.bashrc \
    && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc \
    && echo "export ROS_DOMAIN_ID=156" >> /root/.bashrc \
    && echo "export ROS_LOCALHOST_ONLY=0" >> /root/.bashrc

CMD ["/bin/bash"]
