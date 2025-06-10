#!/bin/bash

TARGETOS=$1
TARGETARCH=$2
BUILDARCH=$3
SCRIPTDIR=$(realpath $(dirname $0))

# run on ubuntu

apt-get update
apt-get install -y jq

COMPILERARCH=$(cat scripts/platforms.json | jq -r ".Architecture.$TARGETARCH")
CMAKE_ARGS="-DSDL_VIDEO=OFF"

if [[ "$TARGETARCH" != "$BUILDARCH" ]]; then
    apt-get install -y gcc-$COMPILERARCH-linux-gnu g++-$COMPILERARCH-linux-gnu binutils-$COMPILERARCH-linux-gnu

    dpkg --add-architecture arm64
    cp -rf $SCRIPTDIR/sources.list.d /etc/apt/

    apt-get update
    apt-get install -y libpython3-dev:$TARGETARCH

    if [[ $? -ne 0 ]]; then
        exit 1
    fi

    CMAKE_ARGS="$CMAKE_ARGS -DCMAKE_TOOLCHAIN_FILE=$SCRIPTDIR/$TARGETARCH.cmake"
fi

source /opt/ros/jazzy/setup.bash
colcon build --cmake-args $CMAKE_ARGS

if [[ "$TAGETARCH" == "$BUILDARCH" ]]; then
    colcon test --ctest-args --output-on-failure --packages-select pancake
    colcon test-result --verbose --all
fi