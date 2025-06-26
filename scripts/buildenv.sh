#!/bin/bash

TARGETOS=$1
TARGETARCH=$2
BUILDARCH=$3
DISTRO=$4

SCRIPTDIR=$(realpath $(dirname $0))
DOCKERDIR=$(realpath "$SCRIPTDIR/../docker")

echo "Configuring build environment"
echo "Target: $TARGETOS/$TARGETARCH"
echo "Build: $BUILDARCH"
echo "ROS2 distro: $DISTRO"

apt-get update
apt-get install -y jq curl bzip2 tar

if [[ $? -ne 0 ]]; then
    exit 1
fi

COMPILERARCH=$(cat $SCRIPTDIR/platforms.json | jq -r ".Architecture.$TARGETARCH")
if [[ "$TARGETARCH" != "$BUILDARCH" ]]; then
    apt-get install -y gcc-$COMPILERARCH-linux-gnu g++-$COMPILERARCH-linux-gnu binutils-$COMPILERARCH-linux-gnu
    if [[ $? -ne 0 ]]; then
        exit 1
    fi

    dpkg --add-architecture $TARGETARCH
    if [[ $? -ne 0 ]]; then
        exit 1
    fi

    cp -rf $DOCKERDIR/sources.list.d /etc/apt/
    apt-get update
    if [[ $? -ne 0 ]]; then
        exit 1
    fi

    apt-get install --no-install-recommends -y libpython3-dev:$TARGETARCH liblttng-ust-dev:$TARGETARCH libyaml-dev:$TARGETARCH libspdlog-dev:$TARGETARCH libssl-dev:$TARGETARCH
    if [[ $? -ne 0 ]]; then
        exit 1
    fi

    echo "Downloading ROS2 ($TARGETOS/$TARGETARCH $DISTRO)"
    curl -L $(cat $SCRIPTDIR/ros.json | jq -r ".$DISTRO.$TARGETOS.$TARGETARCH") -o $DISTRO.tar.bz2

    echo "Extracting ROS2"
    mkdir -p /opt/ros/$DISTRO

    tar -xf $DISTRO.tar.bz2 -C /opt/ros/$DISTRO --strip-components=1
    if [[ $? -ne 0 ]]; then
        exit 1
    fi
fi

rm $DISTRO.tar.bz2
echo "Build environment set up!"