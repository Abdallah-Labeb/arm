#!/bin/bash

# Setup script for Color Sorting Arm Project
# This script helps set up the ROS workspace and dependencies

echo "=========================================="
echo "Color Sorting Arm - Setup Script"
echo "=========================================="

# Check if ROS is installed
if [ -z "$ROS_DISTRO" ]; then
    echo "ERROR: ROS is not installed or not sourced!"
    echo "Please install ROS Noetic first."
    exit 1
fi

echo "ROS Distribution: $ROS_DISTRO"

# Install dependencies
echo ""
echo "Installing dependencies..."
sudo apt update
sudo apt install -y \
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    ros-$ROS_DISTRO-gazebo-ros-control \
    ros-$ROS_DISTRO-ros-control \
    ros-$ROS_DISTRO-ros-controllers \
    ros-$ROS_DISTRO-joint-state-controller \
    ros-$ROS_DISTRO-position-controllers \
    ros-$ROS_DISTRO-effort-controllers \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-xacro \
    python3-opencv \
    python3-numpy

# Make scripts executable
echo ""
echo "Making Python scripts executable..."
chmod +x scripts/*.py

# Check if in catkin workspace
if [ ! -f "../../devel/setup.bash" ]; then
    echo ""
    echo "WARNING: Not in a catkin workspace!"
    echo "Please create a catkin workspace first:"
    echo "  mkdir -p ~/catkin_ws/src"
    echo "  cd ~/catkin_ws/src"
    echo "  # Copy this package here"
    echo "  cd ~/catkin_ws"
    echo "  catkin_make"
fi

echo ""
echo "=========================================="
echo "Setup complete!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "1. If not already done, build the workspace:"
echo "   cd ~/catkin_ws"
echo "   catkin_make"
echo ""
echo "2. Source the workspace:"
echo "   source ~/catkin_ws/devel/setup.bash"
echo ""
echo "3. Launch the project:"
echo "   roslaunch color_sorting_arm complete_system.launch"
echo ""
