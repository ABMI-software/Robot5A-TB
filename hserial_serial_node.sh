#!/bin/bash

set -e


cd ~/Robot5A-TB
source install/setup.bash
ros2 run chariot_control serial_node 

