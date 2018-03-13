FROM ros:kinetic-ros-base-xenial

ENV PROJECT_NAME=some_project
RUN mkdir -p /root/ros/src/${PROJECT_NAME}

# Utilities and build dependencies
RUN set -ex \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates \
        openssl \
        curl \
        python-pip \
    && rm -rf /var/lib/apt/lists/* \
    # git-lfs
    && mkdir -p /tmp/git-lfs \
    && curl -SL https://github.com/git-lfs/git-lfs/releases/download/v2.3.4/git-lfs-linux-amd64-2.3.4.tar.gz | tar -xz --strip-components=1 -C /tmp/git-lfs -f - \
    && mv /tmp/git-lfs/git-lfs /usr/bin \
    && rm -rf /tmp/git-lfs \
    && git lfs install \
    # SSL certs for Gitlab
    && curl http://www.boeing.com/crl/The%20Boeing%20Company%20Root%20Certificate%20Authority.crt | openssl x509 -inform DER -out /usr/local/share/ca-certificates/boeing-root-ca.crt \
    && curl http://www.boeing.com/crl/Boeing%20Basic%20Assurance%20Software%20Root%20CA%20G2.crt | openssl x509 -inform DER -out /usr/local/share/ca-certificates/boeing-software-root-ca.crt \
    && update-ca-certificates \
    && pip install --no-cache-dir catkin-tools

#######
# Install source deps here
#######

#######
# ROS setup
#######
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
