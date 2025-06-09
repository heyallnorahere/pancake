#!/bin/bash
# use ubuntu!
# run as root!

TARGETARCH=$1
BUILDARCH=$2

apt-get update
apt-get install -y jq

if [[ "$TARGETARCH" != "$BUILDARCH" ]]; then
    DPKGARCH=$(cat scripts/platforms.json | jq -r ".Architecture.$TARGETARCH")
    dpkg --add-architecture $DPKGARCH
fi

apt-get update
add-apt-repository universe

apt-get update
apt-get install -y curl build-essential cmake

ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $UBUNTU_CODENAME)_all.deb"
apt install -y /tmp/ros2-apt-source.deb

apt-get update
apt-get upgrade -y
apt-get install -y ros-jazzy-ros-base colcon