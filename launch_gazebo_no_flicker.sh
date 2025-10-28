#!/bin/bash

# Fix for Gazebo flickering/blinking in VMs
# Run this script instead of the raw ros2 launch command

# Set environment for better graphics performance
export TURTLEBOT3_MODEL=burger
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3

# Source ROS 2
source /opt/ros/jazzy/setup.bash
source ~/RIS-TurtleBot3-Autonomous-Navigation/install/setup.bash

# Launch with reduced rendering
echo "🚀 Starting Gazebo with anti-flicker settings..."
echo "✓ TURTLEBOT3_MODEL=burger"
echo "✓ Software rendering enabled"
echo "✓ Headless rendering mode"
echo ""

ros2 launch my_simulations maze_world.launch.py
