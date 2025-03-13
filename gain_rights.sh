#!/bin/bash

set -e


source /opt/ros/jazzy/setup.bash
source install/setup.bash
sudo chmod 0666 /dev/video*
sudo chmod 0666 /dev/tty*