# Copyright (c) 2024- Octa Robotics, Inc. All Rights Reserved.

FROM osrf/ros:humble-desktop

WORKDIR /root

RUN apt update && apt install -y python3-pip
RUN pip install "paho-mqtt<2.0.0" 
RUN pip install ruamel.yaml


RUN git clone https://github.com/open-rmf/rmf_internal_msgs.git

# To reduce build time, build only msgs for lift and door
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install --packages-select rmf_lift_msgs rmf_door_msgs


COPY start.sh test_client_lift.sh test_client_door.sh ./
RUN chmod 755 *.sh

CMD ["/bin/bash"]
