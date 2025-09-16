#!/bin/bash
# Copyright (c) 2024- Octa Robotics, Inc. All Rights Reserved.

LCI_SERVER_CONFIG_YAML="src/lci_config/server_config_simulator.yaml"
LCI_CERT_DIR="src/cert"

# RMF Web have a trouble with slash separated lift_name and door_name because it may require those values to be URL safe
# To avoid their limitation, the separater replacement function is added.
# LCI_DEVICE_NAME_SEPARATER="_"
LCI_DEVICE_NAME_SEPARATER="/" # default

#LOG_LEVEL=debug
LOG_LEVEL=info

# To build only src directory that newly mounted when docker run
source /opt/ros/${ROS_DISTRO}/setup.sh
colcon build --symlink-install --packages-skip-build-finished --packages-select rmf_lift_msgs rmf_door_msgs
colcon build --symlink-install --packages-select lci_rmf_adapter

source install/setup.bash
ros2 run lci_rmf_adapter lci_rmf_adapter \
    --ros-args -p "lci_server_config:=$LCI_SERVER_CONFIG_YAML" \
    -p "lci_cert_dir:=$LCI_CERT_DIR" \
    -p "lci_device_name_separater:=$LCI_DEVICE_NAME_SEPARATER" \
    --log-level $LOG_LEVEL
