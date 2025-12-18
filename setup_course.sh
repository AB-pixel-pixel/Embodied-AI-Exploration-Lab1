#!/bin/bash

# Stop on error
set -e

echo "=========================================="
echo "Starting ROS Course Environment Setup"
echo "=========================================="

# 0. Check for ROS installation
if ! command -v catkin_make &> /dev/null; then
    echo "Error: ROS is not installed or not in PATH."
    echo "Please install ROS Noetic (or your distribution) first."
    exit 1
fi

# 1. Initialize Workspace
echo "[1/4] Initializing workspace..."
if [ ! -L src/CMakeLists.txt ] && [ ! -f src/CMakeLists.txt ]; then
    echo "Creating src/CMakeLists.txt..."
    cd src
    catkin_init_workspace
    cd ..
fi

# 2. Install dependencies
echo "[2/4] Installing dependencies..."
# Check if rosdep is initialized
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    echo "Initializing rosdep..."
    sudo rosdep init
fi

echo "Updating rosdep..."
rosdep update

echo "Installing dependencies for packages..."
# Install dependencies for all packages in src
rosdep install --from-paths src --ignore-src -r -y

# 3. Build the workspace
echo "[3/4] Building the workspace..."
catkin_make

# 4. Environment setup
echo "[4/4] Setup complete!"
echo "=========================================="
echo "To start using the code, please run:"
echo ""
echo "    source devel/setup.bash"
echo ""
echo "Tip: You can add this line to your ~/.bashrc to source it automatically."
echo "=========================================="
