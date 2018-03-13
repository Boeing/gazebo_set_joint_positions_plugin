FROM ros:kinetic-ros-base-xenial

ENV PROJECT_NAME=gazebo_set_joint_positions
RUN mkdir -p /root/ros/src/${PROJECT_NAME}

# Utilities and build dependencies
RUN set -ex \
    && apt-get update \
    && apt-get install -y \
        python-pip \
    && rm -rf /var/lib/apt/lists/* \
    && pip install --no-cache-dir catkin-tools

COPY entrypoint.sh /root/ros/entrypoint.sh

# Clone internal deps from other Boeing projects
COPY ./.rosinstall /root/ros/src/.rosinstall
RUN set -ex \
    && cd /root/ros/src \
    && wstool update --target-workspace /root/ros/src \
    && apt-get update \
    && rosdep update \
    && rosdep install --from-paths /root/ros/src --ignore-src --as-root apt:false -y \
    && rm -rf /var/lib/apt/lists/*

# Install rosdeps for project workspace
COPY package.xml /root/ros/src/${PROJECT_NAME}/package.xml
RUN set -ex \
    && apt-get update \
    && rosdep update \
    && rosdep install --from-paths /root/ros/src --ignore-src --as-root apt:false -y \
    && rm -rf /var/lib/apt/lists/*

# Build project source
COPY . /root/ros/src/${PROJECT_NAME}/

WORKDIR /root/ros/
RUN set -ex \
    && catkin init \
    && catkin config --extend /opt/ros/kinetic \
    && catkin build

ENTRYPOINT [ "/root/ros/entrypoint.sh" ]
