#!/bin/bash
# Copyright (c) 2026- Octa Robotics, Inc. All Rights Reserved.

RESOURCE_NAME="/lci/simulator/sem/area-1"

# If using another separater from "/", please set LCI_DEVICE_NAME_SEPARATER in start.sh
# RESOURCE_NAME="_lci_simulator_sem_area-1"
#LOG_LEVEL=debug
LOG_LEVEL=info

# To build only src directory that newly mounted when docker run
source /opt/ros/${ROS_DISTRO}/setup.sh
colcon build --symlink-install --packages-skip-build-finished --packages-select rmf_lift_msgs rmf_door_msgs lci_rmf_adapter

source install/setup.bash
ros2 run lci_rmf_adapter lci_rmf_adapter_sem_virtual_door_test --ros-args -p "resource_name:=$RESOURCE_NAME" --log-level $LOG_LEVEL
