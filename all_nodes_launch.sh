#!/bin/bash

set -e

source install/setup.bash

exec ros2 launch chariot_control chariot_control_launch.py
