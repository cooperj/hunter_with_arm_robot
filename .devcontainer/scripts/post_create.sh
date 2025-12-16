#!/bin/bash

set -e -x

# Remove directories if they exist
rm -rf /home/ros/aoc_hunter_ws/install
rm -rf /home/ros/aoc_hunter_ws/build
rm -rf /home/ros/aoc_hunter_ws/log

bash /home/ros/aoc_hunter_ws/src/.devcontainer/scripts/build_workspace.sh
bash /home/ros/aoc_hunter_ws/src/.devcontainer/scripts/startup.sh
bash /home/ros/aoc_hunter_ws/src/.devcontainer/scripts/vnc.sh

# Create directories
mkdir -p /home/ros/aoc_hunter_ws/install
mkdir -p /home/ros/aoc_hunter_ws/build
mkdir -p /home/ros/aoc_hunter_ws/log

# Set ownership to ros user and group
chown -R ros:ros /home/ros/aoc_hunter_ws/install
chown -R ros:ros /home/ros/aoc_hunter_ws/build
chown -R ros:ros /home/ros/aoc_hunter_ws/log

cd /home/ros/aoc_hunter_ws/ && colcon build --symlink-install
