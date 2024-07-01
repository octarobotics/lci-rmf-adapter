#!/bin/bash
# Copyright (c) 2024- Octa Robotics, Inc. All Rights Reserved.

LIFT_NAME="/lci/simulator/1/1"
ORIGINATION="B1"
DESTINATION="M5"

# ORIGINATION="R_r"
# DESTINATION="8F_r"

#LOG_LEVEL=debug
LOG_LEVEL=info

# To build only src directory that newly mounted when docker run
source /opt/ros/humble/setup.sh
colcon build --symlink-install --packages-skip-build-finished --packages-select rmf_lift_msgs rmf_door_msgs lci_rmf_adapter

source install/setup.bash
ros2 run lci_rmf_adapter lci_rmf_adapter_lift_test --ros-args -p "lift_name:=$LIFT_NAME" -p "origination:=$ORIGINATION" -p "destination:=$DESTINATION" --log-level $LOG_LEVEL
