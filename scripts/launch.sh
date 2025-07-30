#!/bin/bash

SCRIPT_DIR=$(realpath $(dirname $0))
source $SCRIPT_DIR/../install/setup.bash
ros2 launch pancake $1.xml &
ROS_PID=$!

MAX_TRIES=10
COUNT=0

GDB_SERVER=:4444
while [[ $COUNT -lt $MAX_TRIES ]]; do
    SWERVE_PID=$(pgrep swerve)
    if [[ $? -eq 0 ]]; then
        echo "Running GDB server..."
        sudo gdbserver --attach $GDB_SERVER $SWERVE_PID

        break
    fi

    sleep 0.005
    COUNT=($COUNT + 1)
done

if [[ $COUNT -eq $MAX_TRIES ]]; then
    echo "Failed to find swerve node! skipping GDB server" >&2
fi

wait
