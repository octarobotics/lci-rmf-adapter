# Copyright (c) 2024- Octa Robotics, Inc. All Rights Reserved.
ARG ROS_DISTRO=humble
FROM osrf/ros:${ROS_DISTRO}-desktop

WORKDIR /root

RUN apt update && apt install -y python3-pip

# To support jazzy (Ubuntu 24.04 or later)
ARG PIP_BREAK_SYSTEM_PACKAGES=1

RUN pip install "paho-mqtt==1.6.1"
RUN pip install ruamel.yaml
RUN pip install packaging


RUN git clone https://github.com/open-rmf/rmf_internal_msgs.git

# To reduce build time, build only msgs for lift and door
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --symlink-install --packages-select rmf_lift_msgs rmf_door_msgs

# Install CycloneDDS when RMW_IMPLEMENTATION is "rmw_cyclonedds_cpp".
ARG RMW_IMPLEMENTATION=rmw_fastrtps_cpp
RUN if [ "${RMW_IMPLEMENTATION}" = "rmw_cyclonedds_cpp" ]; then \
        apt update && apt install -y ros-${ROS_DISTRO}-rmw-cyclonedds-cpp; \
    fi

COPY start.sh test_client_*.sh ./
RUN chmod 755 *.sh

ENV ROS_DISTRO=${ROS_DISTRO}
ENV RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}

CMD ["/bin/bash"]
