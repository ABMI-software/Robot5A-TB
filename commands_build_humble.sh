#!/bin/bash

set -e

cd ~/Robot5A-TB
source /opt/ros/humble/setup.bash
rm -rf build install log
colcon build --cmake-clean-cache
source install/setup.bash