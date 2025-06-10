#!/bin/bash

source /pancake/install/setup.bash
colcon test --ctest-args --output-on-failure
colcon test-result --verbose --all

if [[ $? -ne 0 ]]; then
    exit 1
fi