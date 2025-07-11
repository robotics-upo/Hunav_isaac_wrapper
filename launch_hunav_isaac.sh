#!/bin/bash
#
# launch_hunav_isaac.sh
#
# Simple launcher script that mimics the original usage:
# bash ~/isaacsim/python.sh ~/Hunav_isaac_wrapper/main.py
#
# This script can be used in three ways:
# 1. Interactive mode: ./launch_hunav_isaac.sh
# 2. With scenario: ./launch_hunav_isaac.sh warehouse_agents.yaml
# 3. Isaac Sim style: bash ~/isaacsim/python.sh ~/Hunav_isaac_wrapper/scripts/main.py
#

set -e

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAIN_SCRIPT="$SCRIPT_DIR/src/scripts/main.py"

# Check if main script exists
if [ ! -f "$MAIN_SCRIPT" ]; then
    echo -e "${YELLOW}Warning: main.py not found at $MAIN_SCRIPT${NC}"
    echo "Trying package installation location..."
    
    # Try to find via ROS2 package
    if command -v ros2 &> /dev/null; then
        PACKAGE_PATH=$(ros2 pkg prefix hunav_isaac_wrapper 2>/dev/null || echo "")
        if [ -n "$PACKAGE_PATH" ]; then
            MAIN_SCRIPT="$PACKAGE_PATH/lib/hunav_isaac_wrapper/main.py"
        fi
    fi
    
    if [ ! -f "$MAIN_SCRIPT" ]; then
        echo "Error: Cannot find main.py script"
        exit 1
    fi
fi

echo -e "${GREEN}HuNav Isaac Wrapper Launcher${NC}"
echo "Using script: $MAIN_SCRIPT"

# Check if Isaac Sim python is available
ISAAC_PYTHON=""
ISAAC_SIM_PATH="$HOME/isaacsim/python.sh"
if [ -f "$ISAAC_SIM_PATH" ]; then
    ISAAC_PYTHON="bash $ISAAC_SIM_PATH"
    echo "Using Isaac Sim python: $ISAAC_PYTHON"
elif [ -f "$HOME/.local/share/ov/pkg/isaac_sim-"*/python.sh ]; then
    ISAAC_SIM_PATH=$(ls "$HOME/.local/share/ov/pkg/isaac_sim-"*/python.sh 2>/dev/null | head -1)
    if [ -f "$ISAAC_SIM_PATH" ]; then
        ISAAC_PYTHON="bash $ISAAC_SIM_PATH"
        echo "Using Isaac Sim python: $ISAAC_PYTHON"
    fi
elif command -v isaacsim &> /dev/null; then
    ISAAC_PYTHON="isaacsim"
    echo "Using Isaac Sim python: $ISAAC_PYTHON"
fi

if [ -z "$ISAAC_PYTHON" ]; then
    echo -e "${YELLOW}Warning: Isaac Sim python not found.${NC}"
    echo -e "${YELLOW}Falling back to system python3. Some Isaac Sim features may not work.${NC}"
    echo -e "${YELLOW}For best results, please install Isaac Sim and source its environment.${NC}"
    echo ""
    ISAAC_PYTHON="python3"
    echo "Using system python: $ISAAC_PYTHON"
fi

# Parse arguments
if [ "$1" = "--help" ] || [ "$1" = "-h" ]; then
    echo "HuNav Isaac Wrapper Launcher"
    echo ""
    echo "Usage:"
    echo "  $0                    # Interactive mode"
    echo "  $0 [scenario.yaml]    # Launch with specific scenario"
    echo "  $0 [args...]          # Pass arguments to main.py"
    echo ""
    echo "Examples:"
    echo "  $0                              # Show interactive menu"
    echo "  $0 warehouse_agents.yaml        # Launch warehouse scenario"
    echo "  $0 --scenario myfile.yaml --batch  # Batch mode"
    echo ""
    exit 0
elif [ $# -eq 0 ]; then
    # Interactive mode
    echo -e "${GREEN}Launching in interactive mode...${NC}"
    $ISAAC_PYTHON "$MAIN_SCRIPT"
elif [ $# -eq 1 ]; then
    # Scenario specified
    echo -e "${GREEN}Launching with scenario: $1${NC}"
    $ISAAC_PYTHON "$MAIN_SCRIPT" --scenario "$1" --batch
else
    # Pass all arguments
    echo -e "${GREEN}Launching with arguments: $@${NC}"
    $ISAAC_PYTHON "$MAIN_SCRIPT" "$@"
fi
