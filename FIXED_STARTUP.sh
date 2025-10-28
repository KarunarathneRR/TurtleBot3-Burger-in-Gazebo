#!/bin/bash

# FIXED COMPLETE STARTUP SCRIPT FOR TURTLEBOT3 NAVIGATION
# This script fixes the map loading and Gazebo-RViz synchronization issues

set -e  # Exit on error

echo "=================================="
echo "TurtleBot3 Navigation Startup"
echo "=================================="
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Source ROS2 environment
echo -e "${YELLOW}[1/5] Sourcing ROS2 environment...${NC}"
source /opt/ros/jazzy/setup.bash
source ~/RIS-TurtleBot3-Autonomous-Navigation/install/setup.bash
export TURTLEBOT3_MODEL=burger

echo -e "${GREEN}✓ Environment configured${NC}"
echo ""

# Launch Gazebo in a new terminal
echo -e "${YELLOW}[2/5] Launching Gazebo...${NC}"
gnome-terminal --title="Gazebo - TurtleBot3" -- bash -c "
cd ~/RIS-TurtleBot3-Autonomous-Navigation
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
./launch_gazebo_no_flicker.sh
exec bash"

echo "   Waiting 20 seconds for Gazebo to initialize..."
sleep 20

# Check if Gazebo topics are available
echo -e "${YELLOW}   Checking Gazebo topics...${NC}"
if ros2 topic list | grep -q "/scan"; then
    echo -e "${GREEN}✓ Gazebo is running with TurtleBot3${NC}"
else
    echo -e "${RED}✗ Error: Gazebo topics not found. Please check Terminal 1.${NC}"
    exit 1
fi
echo ""

# Launch Nav2 in a new terminal
echo -e "${YELLOW}[3/5] Launching Navigation2...${NC}"
gnome-terminal --title="Nav2 - Navigation Stack" -- bash -c "
cd ~/RIS-TurtleBot3-Autonomous-Navigation
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch launch/nav2_maze.launch.py
exec bash"

echo "   Waiting 20 seconds for Nav2 to initialize..."
sleep 20

# Check if Nav2 nodes are running
echo -e "${YELLOW}   Checking Nav2 nodes...${NC}"
if ros2 node list | grep -q "map_server"; then
    echo -e "${GREEN}✓ Navigation2 is running${NC}"
else
    echo -e "${RED}✗ Warning: Nav2 nodes not fully started yet${NC}"
fi
echo ""

# Set initial pose
echo -e "${YELLOW}[4/5] Setting initial robot pose...${NC}"
sleep 2
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'},
  pose: {
    pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},
    covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467]
  }
}" > /dev/null 2>&1

echo -e "${GREEN}✓ Initial pose set to (0, 0, 0)${NC}"
echo ""

# Launch RViz in a new terminal
echo -e "${YELLOW}[5/5] Launching RViz2...${NC}"
gnome-terminal --title="RViz2 - Visualization" -- bash -c "
cd ~/RIS-TurtleBot3-Autonomous-Navigation
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
rviz2 -d config/nav2_default_view.rviz
exec bash"

sleep 3
echo ""
echo "=================================="
echo -e "${GREEN}✓ All systems launched!${NC}"
echo "=================================="
echo ""
echo "VERIFICATION STEPS:"
echo "1. In RViz, you should see:"
echo "   - The map loaded (black/white maze)"
echo "   - Red laser scan from the LIDAR"
echo "   - Green particle cloud (AMCL localization)"
echo "   - Robot model at origin"
echo ""
echo "2. If the map is not visible:"
echo "   - Wait a few more seconds"
echo "   - Check Terminal 2 for Nav2 errors"
echo "   - Ensure map_server node is active"
echo ""
echo "3. To set a navigation goal:"
echo "   - Click 'Nav2 Goal' button in RViz toolbar"
echo "   - Click a location on the map"
echo "   - Robot should navigate in both RViz AND Gazebo"
echo ""
echo "4. To monitor system status, run:"
echo "   ros2 node list"
echo "   ros2 topic list"
echo "   ros2 topic hz /scan"
echo ""
echo "Press Ctrl+C in each terminal to stop components"
echo "=================================="
