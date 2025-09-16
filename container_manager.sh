#!/bin/bash
# Copyright (c) 2024- Octa Robotics, Inc. All Rights Reserved.

ROS_DISTRO=humble
# ROS_DISTRO=jazzy


build() { 
    docker compose build --build-arg ROS_DISTRO=${ROS_DISTRO}
}

start() {
    build
    docker compose up --remove-orphans -d
}

stop() {
    docker compose down
}

restart() {
    stop
    start
}

status() {
    docker compose ps --services
}

login() {
    docker compose exec lci_rmf_adapter bash
}

logs() {
    docker logs lci_rmf_adapter
}

prune() {
    docker system prune --volumes -f
}

if [ ${EUID:-${UID}} != 0 ]; then
    echo "This script must be run as root"
else
    $1 $2
fi
