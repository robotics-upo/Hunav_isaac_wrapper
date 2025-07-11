#!/bin/bash
"""
setup_workspace.sh

Script to set up the ROS2 workspace for hunav_isaac_wrapper package.
"""

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}Setting up HuNav Isaac Wrapper ROS2 Package...${NC}"

# Check if we're in the right directory
if [ ! -f "package.xml" ]; then
    echo -e "${RED}Error: package.xml not found. Please run this script from the package root directory.${NC}"
    exit 1
fi

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}Warning: ROS2 not sourced. Please source your ROS2 installation first:${NC}"
    echo "source /opt/ros/humble/setup.bash  # or your ROS2 distro"
    exit 1
fi

echo -e "${GREEN}ROS2 Distribution: $ROS_DISTRO${NC}"

# Check for required dependencies
echo -e "${GREEN}Checking dependencies...${NC}"

# Check for hunav_msgs
if ! ros2 pkg list | grep -q "hunav_msgs"; then
    echo -e "${YELLOW}Warning: hunav_msgs package not found. Please install it first.${NC}"
    echo "You can install it from: https://github.com/robotics-upo/hunav_sim"
fi

# Check for other dependencies
DEPS=("geometry_msgs" "std_msgs" "nav_msgs" "sensor_msgs" "tf2_ros")
for dep in "${DEPS[@]}"; do
    if ! ros2 pkg list | grep -q "$dep"; then
        echo -e "${YELLOW}Warning: $dep package not found${NC}"
    else
        echo -e "${GREEN}✓ $dep found${NC}"
    fi
done

# Build the package
echo -e "${GREEN}Building package...${NC}"
if [ -n "$COLCON_WS" ]; then
    # If we're in a colcon workspace
    cd "$COLCON_WS"
    colcon build --packages-select hunav_isaac_wrapper
    echo -e "${GREEN}Package built successfully!${NC}"
    echo -e "${YELLOW}Don't forget to source the workspace:${NC}"
    echo "source install/setup.bash"
else
    echo -e "${YELLOW}Not in a colcon workspace. To build:${NC}"
    echo "1. Create a ROS2 workspace:"
    echo "   mkdir -p ~/ros2_ws/src"
    echo "   cd ~/ros2_ws/src"
    echo "   ln -s $(pwd) ."
    echo "2. Build the package:"
    echo "   cd ~/ros2_ws"
    echo "   colcon build --packages-select hunav_isaac_wrapper"
    echo "   source install/setup.bash"
fi

echo -e "${GREEN}Setup complete!${NC}"

