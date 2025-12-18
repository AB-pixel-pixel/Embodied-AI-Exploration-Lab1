#!/bin/bash

# Stop on error
set -e

echo "=========================================="
echo "Building ROS Course Workspace"
echo "=========================================="

# Check for src directory
if [ ! -d "src" ]; then
    echo "Error: 'src' directory not found. Please run this script from the workspace root."
    exit 1
fi

# Initialize Workspace (if needed)
# This creates the symlink for src/CMakeLists.txt if it doesn't exist
if [ ! -L src/CMakeLists.txt ] && [ ! -f src/CMakeLists.txt ]; then
    echo "Initializing workspace (creating src/CMakeLists.txt)..."
    cd src
    catkin_init_workspace
    cd ..
fi

# Build the workspace
echo "Compiling code..."
catkin_make

echo "=========================================="
echo "Build complete!"
echo "To start using the code, please run:"
echo ""
echo "    source devel/setup.bash"
echo ""
echo "=========================================="
