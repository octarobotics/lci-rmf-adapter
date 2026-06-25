#!/bin/bash
# Copyright (c) 2024- Octa Robotics, Inc. All Rights Reserved.

DOOR_NAME="/lci/simulator/1F/1"

# If using another separater from "/", please set LCI_DEVICE_NAME_SEPARATER in start.sh
# DOOR_NAME="_lci_simulator_1F_1"

# If RMF uses the level-based identifiers such as L1, L2, L3, 
# please specify floor_name_aliases in the configuration file. See README.md for more detail.
# DOOR_NAME="/lci/simulator/L1/1"

# RMF defines two topics for DoorRequest: "/door_requests" and "/adapter_door_requests".
# To support different RMF deployments, these topics can be configured using environment variables.
# RMF_DOOR_REQUESTS_TOPIC="door_requests"
RMF_DOOR_REQUESTS_TOPIC="adapter_door_requests" # default

#LOG_LEVEL=debug
LOG_LEVEL=info

# To build only src directory that newly mounted when docker run
source /opt/ros/${ROS_DISTRO}/setup.sh
colcon build --symlink-install --packages-skip-build-finished --packages-select rmf_lift_msgs rmf_door_msgs lci_rmf_adapter

source install/setup.bash
ros2 run lci_rmf_adapter lci_rmf_adapter_door_test \
    --ros-args -p "door_name:=$DOOR_NAME" \
    -p "rmf_door_requests_topic:=$RMF_DOOR_REQUESTS_TOPIC" \
    --log-level $LOG_LEVEL
