#!/bin/bash

SCRIPT_DIR=$(realpath $(dirname $0))
source $SCRIPT_DIR/install/setup.bash
ros2 launch pancake $1.xml