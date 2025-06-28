#!/bin/bash

# DDSM115 Driver Build Script

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}DDSM115 Driver Build Script${NC}"
echo "================================="

# Check if running in ROS2 environment
if [ -n "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}ROS2 environment detected: $ROS_DISTRO${NC}"
    BUILD_ROS2=true
else
    echo -e "${YELLOW}No ROS2 environment detected, building standalone library only${NC}"
    BUILD_ROS2=false
fi

# Build standalone library
echo -e "${GREEN}Building standalone library...${NC}"
mkdir -p build
cd build

# Set compilers explicitly to avoid ccache issues
export CC=/usr/bin/gcc
export CXX=/usr/bin/g++

cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=$CC -DCMAKE_CXX_COMPILER=$CXX
make -j$(nproc)

echo -e "${GREEN}Standalone library built successfully!${NC}"

# Build examples
echo -e "${GREEN}Building examples...${NC}"
if [ -d "examples" ]; then
    echo "Examples built and available in build/examples/"
    ls -la examples/
fi

# Test basic functionality (if requested)
if [ "$1" = "test" ]; then
    echo -e "${YELLOW}Running basic tests...${NC}"
    # Add test commands here if needed
fi

# Build ROS2 wrapper if in ROS2 environment
if [ "$BUILD_ROS2" = true ]; then
    echo -e "${GREEN}Building ROS2 wrapper...${NC}"
    cd ..
    
    # Check if we're in a ROS2 workspace
    if [ -f "../../CMakeLists.txt" ] && grep -q "ament" "../../CMakeLists.txt"; then
        echo "Building in ROS2 workspace..."
        cd ../..
        colcon build --packages-select ddsm115_driver ddsm115_ros2
        echo -e "${GREEN}ROS2 packages built successfully!${NC}"
        echo "Source the workspace: source install/setup.bash"
    else
        echo -e "${YELLOW}Not in a ROS2 workspace, copying ROS2 wrapper for manual integration${NC}"
    fi
fi

echo -e "${GREEN}Build completed successfully!${NC}"
echo ""
echo "Usage examples:"
echo "  Standalone: ./build/examples/basic_example"
if [ "$BUILD_ROS2" = true ]; then
    echo "  ROS2: ros2 launch ddsm115_ros2 ddsm115_single_motor.launch.py"
fi
echo ""
echo "For more information, see README.md"
