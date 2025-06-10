#!/bin/bash

TARGETARCH=$1
BUILDARCH=$2

if [[ "$TARGETARCH" != "$BUILDARCH" ]]; then
    colcon test --ctest-args --output-on-failure --packages-select pancake
    colcon test-result --verbose --all

    if [[ $? -ne 0 ]]; then
        exit 1
    fi
fi