#!/bin/bash
# Copyright (c) 2024- Octa Robotics, Inc. All Rights Reserved.

DOOR_NAME="/lci/simulator/1F/1"

#LOG_LEVEL=debug
LOG_LEVEL=info

# To build only src directory that newly mounted when docker run
source /opt/ros/humble/setup.sh
colcon build --symlink-install --packages-skip-build-finished --packages-select rmf_lift_msgs rmf_door_msgs lci_rmf_adapter

source install/setup.bash
ros2 run lci_rmf_adapter lci_rmf_adapter_door_test --ros-args -p "door_name:=$DOOR_NAME" --log-level $LOG_LEVEL
