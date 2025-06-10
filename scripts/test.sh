#!/bin/bash

DISTRO=$1

source /opt/ros/$DISTRO/setup.bash
source /pancake/install/setup.bash

colcon test --ctest-args --output-on-failure
colcon test-result --verbose --all