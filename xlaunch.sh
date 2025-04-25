#!/bin/bash

set -e

source install/setup.bash
exec ros2 launch slush_engine_communication joint_sync_moveit.launch.py
