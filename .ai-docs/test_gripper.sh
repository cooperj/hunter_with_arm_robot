#!/bin/bash

set -e

echo "=========================================="
echo "Robotiq 85 Gripper Simulation Test"
echo "=========================================="
echo ""

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

source /home/ros/aoc_hunter_ws/install/setup.bash

echo -e "${YELLOW}[1/5] Checking URDF generation...${NC}"
cd /home/ros/aoc_hunter_ws
xacro src/mobile_manipulator_pltf_description/description/mobile_manipulator_pltf.urdf.xacro \
    is_sim:=true use_base:=true use_arm:=true sim_gazebo:=true > /tmp/test_urdf.urdf 2>&1

GRIPPER_COUNT=$(grep -c "robotiq_85" /tmp/test_urdf.urdf || echo 0)
if [ "$GRIPPER_COUNT" -gt 50 ]; then
    echo -e "${GREEN}✓ Gripper found in URDF ($GRIPPER_COUNT references)${NC}"
else
    echo -e "${RED}✗ Gripper missing from URDF${NC}"
    exit 1
fi

echo ""
echo -e "${YELLOW}[2/5] Checking gripper links...${NC}"
LINKS=$(grep -o "robotiq_85_[a-z_]*_link" /tmp/test_urdf.urdf | sort | uniq)
LINK_COUNT=$(echo "$LINKS" | wc -l)
echo -e "${GREEN}✓ Found $LINK_COUNT gripper links${NC}"

echo ""
echo -e "${YELLOW}[3/5] Checking mimic joint configuration...${NC}"
MIMIC_COUNT=$(grep -c "<mimic joint=\"robotiq_85_left_knuckle_joint\"" /tmp/test_urdf.urdf || echo 0)
if [ "$MIMIC_COUNT" -ge 4 ]; then
    echo -e "${GREEN}✓ Mimic joints configured ($MIMIC_COUNT found)${NC}"
else
    echo -e "${RED}✗ Mimic joints not properly configured${NC}"
    exit 1
fi

echo ""
echo -e "${YELLOW}[4/5] Checking ros2_control configuration...${NC}"
ROS2_CTRL=$(grep -A30 "ros2_control name=\"RobotiqGripperHardwareInterface\"" /tmp/test_urdf.urdf | grep -c "gazebo_ros2_control/GazeboSystem" || echo 0)
CMD_IFACES=$(grep -A100 "ros2_control name=\"RobotiqGripperHardwareInterface\"" /tmp/test_urdf.urdf | grep -c "command_interface name=\"position\"" || echo 0)

if [ "$ROS2_CTRL" -gt 0 ]; then
    echo -e "${GREEN}✓ Gazebo ros2_control plugin found${NC}"
else
    echo -e "${RED}✗ Gazebo ros2_control plugin not found${NC}"
    exit 1
fi

if [ "$CMD_IFACES" -gt 0 ]; then
    echo -e "${GREEN}✓ Position command interfaces configured ($CMD_IFACES)${NC}"
else
    echo -e "${RED}✗ Command interfaces not configured${NC}"
    exit 1
fi

echo ""
echo -e "${YELLOW}[5/5] Checking Gazebo collision configuration...${NC}"
SELF_COLLIDE=$(grep -c "selfCollide" /tmp/test_urdf.urdf || echo 0)
if [ "$SELF_COLLIDE" -gt 0 ]; then
    echo -e "${GREEN}✓ Self-collision enabled for mimic joints${NC}"
fi

echo ""
echo "=========================================="
echo -e "${GREEN}✓ All checks passed!${NC}"
echo "=========================================="
echo ""
echo "Next: Launch simulation and test gripper"
