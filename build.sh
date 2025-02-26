#!/bin/bash

set -e


source /opt/ros/jazzy/setup.bash
rm -rf build install log
exec colcon build --symlink-install --cmake-clean-cache
