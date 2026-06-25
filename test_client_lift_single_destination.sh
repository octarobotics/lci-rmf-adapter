#!/bin/bash
# Copyright (c) 2024- Octa Robotics, Inc. All Rights Reserved.

LIFT_NAME="/lci/simulator/1/1"

# If using another separater from "/", please set LCI_DEVICE_NAME_SEPARATER in start.sh
# LIFT_NAME="_lci_simulator_1_1"

ORIGINATION="B1"
DESTINATION="M5"

# If RMF uses the level-based identifiers such as L1, L2, L3, 
# please specify floor_name_aliases in the configuration file. See README.md for more detail.
# ORIGINATION="L1"
# DESTINATION="L2"

# ORIGINATION="R_r"
# DESTINATION="8F_r"

#LOG_LEVEL=debug
LOG_LEVEL=info

# To build only src directory that newly mounted when docker run
source /opt/ros/${ROS_DISTRO}/setup.sh
colcon build --symlink-install --packages-skip-build-finished --packages-select rmf_lift_msgs rmf_door_msgs lci_rmf_adapter

source install/setup.bash
ros2 run lci_rmf_adapter lci_rmf_adapter_lift_test_single_destination --ros-args -p "lift_name:=$LIFT_NAME" -p "origination:=$ORIGINATION" -p "destination:=$DESTINATION" --log-level $LOG_LEVEL
