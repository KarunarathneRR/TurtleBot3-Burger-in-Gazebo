#!/bin/bash

# Clean RViz startup script - suppresses unnecessary stereo warnings
# Usage: ./start_rviz.sh

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  🎨 Starting RViz2 with Navigation Config"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash
source /home/shrinath/RIS-TurtleBot3-Autonomous-Navigation/install/setup.bash

# Get the config file path
CONFIG_FILE="/home/shrinath/RIS-TurtleBot3-Autonomous-Navigation/config/nav2_default_view.rviz"

if [ ! -f "$CONFIG_FILE" ]; then
    echo "❌ Error: Config file not found at $CONFIG_FILE"
    exit 1
fi

echo "✓ Using config: nav2_default_view.rviz"
echo "✓ Launching RViz2..."
echo ""
echo "Expected visualizations:"
echo "  • Grid (gray)"
echo "  • RobotModel (TurtleBot3)"
echo "  • LaserScan (red dots)"
echo "  • Map (black/white)"
echo "  • Amcl Particle Swarm (green arrows)"
echo "  • Amcl Pose (yellow arrow) ← NEW!"
echo "  • Global/Local Costmaps"
echo "  • Global Planner (green path)"
echo "  • Controller (red path)"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# Launch RViz and filter out stereo warnings
ros2 run rviz2 rviz2 -d "$CONFIG_FILE" 2>&1 | grep -v "Stereo is NOT SUPPORTED"
