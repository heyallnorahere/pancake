#!/bin/bash

TARGETOS=$1
TARGETARCH=$2
BUILDARCH=$3

SCRIPTDIR=$(realpath $(dirname $0))
DOCKERDIR=$(realpath "$SCRIPTDIR/../docker")

apt-get update
apt-get install -y jq

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

    apt-get install -y libpython3-dev:$TARGETARCH
    if [[ $? -ne 0 ]]; then
        exit 1
    fi
fi