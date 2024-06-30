#!/bin/bash
# Copyright (c) 2024- Octa Robotics, Inc. All Rights Reserved.

LCI_SERVER_CONFIG_YAML="src/lci_config/server_config_simulator.yaml"
LCI_CERT_DIR="lci_cert_dir:=src/cert"

#LOG_LEVEL=debug
LOG_LEVEL=info

# To build only src directory that newly mounted when docker run
source /opt/ros/humble/setup.sh
colcon build --symlink-install --packages-skip-build-finished --packages-select rmf_lift_msgs rmf_door_msgs
colcon build --symlink-install --packages-select lci_rmf_adapter

source install/setup.bash
ros2 run lci_rmf_adapter lci_rmf_adapter --ros-args -p "lci_server_config:=$LCI_SERVER_CONFIG_YAML" -p "$LCI_CERT_DIR" --log-level $LOG_LEVEL
