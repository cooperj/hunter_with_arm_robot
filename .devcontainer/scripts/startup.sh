#!/bin/bash

# It sets the shell options to:
# -e: Exit immediately if a command exits with a non-zero status.
# -x: Print commands and their arguments as they are executed.
set -e

# Color definitions
RED='\033[31m'
GREEN='\033[32m'
YELLOW='\033[33m'
BLUE='\033[34m'
RESET='\033[0m'  # Reset to default color
BOLD='\033[1m'

# Share internet for the physical robot
#if [ "$AOC_PRIVILEGED" == "1" ]; then
#    echo -e "${GREEN}Sharing internet...${RESET}"
#    bash ${AOC_SERVICE_DIR}/.devcontainer/docker/ros2/config/network_config/sharing_internet.sh
#fi

# Manually Starting MongoDB
## Context
# In containerized environments, starting MongoDB with `systemctl` or `systemd` is often 
# not feasible, as these environments might not utilize `systemd` for service management.
# Consequently, MongoDB must be started manually in such cases.


# enable multicast on loopback interface
echo -e "${BLUE}set multicast on loopback interface${RESET}"

# allow to fail if already set
sudo ifconfig lo multicast || true
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo || true 

echo -e "\nStarting mongodb.."

db_dir=${AOC_SERVICE_DIR}/db
# create db_dir if it does not exist
if [ ! -d "$db_dir" ]; then
    mkdir -p $db_dir
fi

# Run MongoDB in the Background 
screen -dmS mongo -- mongod --dbpath $db_dir --port ${MONGOD_PORT:-27018} | tee /tmp/mongo.log 2>&1 
sleep 1
# Verify MongoDB is Running
if ps aux | grep -v grep | grep mongod > /dev/null
then
    echo -e "${GREEN}Mongod is initialized and running successfully${RESET}"
else
    echo -e "${RED}Mongod process not found${RESET}"
fi