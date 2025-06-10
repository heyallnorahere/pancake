#!/bin/bash

TARGETOS=$1
TARGETARCH=$2
BUILDARCH=$3

SCRIPTDIR=$(realpath $(dirname $0))
DOCKERDIR=$(realpath "$SCRIPTDIR/../docker")

# run on ubuntu

COMPILERARCH=$(cat $SCRIPTDIR/platforms.json | jq -r ".Architecture.$TARGETARCH")
COMPILEROS=$(cat $SCRIPTDIR/platforms.json | jq -r ".OS.$TARGETOS")
CMAKE_ARGS="-DSDL_VIDEO=OFF"

if [[ "$TARGETARCH" != "$BUILDARCH" ]]; then
    CMAKE_ARGS="$CMAKE_ARGS -DCMAKE_SYSTEM_NAME=$COMPILEROS"
    CMAKE_ARGS="$CMAKE_ARGS -DCMAKE_SYSTEM_PROCESSOR=$COMPILERARCH"
    CMAKE_ARGS="$CMAKE_ARGS -DCMAKE_C_COMPILER=$COMPILERARCH-linux-gnu-gcc"
    CMAKE_ARGS="$CMAKE_ARGS -DCMAKE_CXX_COMPILER=$COMPILERARCH-linux-gnu-g++"
    CMAKE_ARGS="$CMAKE_ARGS -DCMAKE_FIND_ROOT_PATH=/usr/$COMPILERARCH-linux-gnu"
fi

source /opt/ros/jazzy/setup.bash
colcon build --cmake-args $CMAKE_ARGS

if [[ "$TARGETARCH" == "$BUILDARCH" ]]; then
    colcon test --ctest-args --output-on-failure --packages-select pancake
    colcon test-result --verbose --all
fi