#!/bin/bash

set -e

source install/setup.bash

exec ros2 launch robot_control_tb visual_control.launch.py open_loop:=true
