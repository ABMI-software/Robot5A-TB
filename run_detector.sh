#!/bin/bash

set -e

source install/setup.bash

exec ros2 run robot_visual aruco_detector_single 


