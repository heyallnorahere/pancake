#!/bin/bash

SCRIPT_DIR=$(realpath $(dirname $0))
CONFIG_DIR=$(realpath $SCRIPT_DIR/../config)

if CAN_NETWORK=$(cat "$CONFIG_DIR/swerve.json" | jq -r ".Network"); then
    # it gets mad on RPi startup if we dont do this
    ifconfig $CAN_NETWORK txqueuelen 1000
fi

source $SCRIPT_DIR/../install/setup.bash
ros2 launch pancake $1.xml &

if command -v gdbserver &> /dev/null; then
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
else
    echo "Could not find gdbserver. Skipping"
fi

wait
