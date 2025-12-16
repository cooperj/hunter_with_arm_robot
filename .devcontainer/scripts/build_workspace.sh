#!/bin/bash

# This script is executed after the creation of the development container.
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

# Print the value of ROBOT_PLATFORM
echo -e "${BLUE}ROBOT_PLATFORM: ${YELLOW}${ROBOT_PLATFORM}${RESET}"

# Build the project
cd ${COLCON_WS}

echo -e "${GREEN}Install any dependencies...${RESET}"
rosdep update --rosdistro $ROS_DISTRO
sudo apt-get update
rosdep install --from-paths src --ignore-src -y
echo -e "${GREEN}Install complete!${RESET}"

# Check if nvidia-smi is available, if not we need to use stubs to build
if ! which nvidia-smi > /dev/null; then 
    OLD_LD_LIBRARY_PATH=$LD_LIBRARY_PATH
    echo -e "${YELLOW}nvidia-smi not found, using stubs and allow undefined references at build time.${RESET}"
    # add cuda stubs to the LD_LIBRARY_PATH temporarily
    export LD_LIBRARY_PATH=/usr/local/cuda/lib64/stubs/:$LD_LIBRARY_PATH
    # we need to allow undefined symbols for shared libraries if CUDA is not present
    cmake_args='--cmake-args -D CMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"'
else
    cmake_args=''
fi

echo -e "${GREEN}Building the project...${RESET}"
# first try buidling from previous build directory, if it fails, clean and build again
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelwithDebInfo $cmake_args || rm -rf build/* install/* && colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelwithDebInfo $cmake_args
echo -e "${GREEN}Build completed successfully${RESET}"

if ! which nvidia-smi > /dev/null; then 
    # restore the LD_LIBRARY_PATH
    export LD_LIBRARY_PATH=$OLD_LD_LIBRARY_PATH
    cmake_args=''
fi

source install/setup.bash
echo -e "${BLUE}ALL AVAILABLE ROS PACKAGES:\n`ros2 pkg list`${RESET}"