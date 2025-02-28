#!/bin/bash

set -e


source /opt/ros/jazzy/setup.bash
rm -rf build install log
colcon build --symlink-install --cmake-clean-cache
source install/setup.bash
