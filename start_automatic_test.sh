#!/bin/bash

set -e

source install/setup.bash
ros2 run slush_engine_communication joint_sync_moveit_node
