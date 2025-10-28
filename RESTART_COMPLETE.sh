#!/bin/bash

# COMPLETE FIX - Clean restart script
# Stops everything and restarts with proper configuration

set -e

echo "========================================"
echo "Complete System Restart with QoS Fix"
echo "========================================"
echo ""

# Stop all ROS2 nodes
echo "1. Stopping all existing processes..."
pkill -f "gz sim" || true
pkill -f gazebo || true
pkill -f ros2 || true
pkill -f rviz2 || true
sleep 3

# Source environment
echo "2. Sourcing ROS2 environment..."
source /opt/ros/jazzy/setup.bash
source ~/RIS-TurtleBot3-Autonomous-Navigation/install/setup.bash
export TURTLEBOT3_MODEL=burger

echo "3. Starting Gazebo..."
gnome-terminal --title="[1] Gazebo" -- bash -c "
cd ~/RIS-TurtleBot3-Autonomous-Navigation
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
./launch_gazebo_no_flicker.sh
exec bash"

echo "   Waiting 25 seconds for Gazebo..."
sleep 25

# Verify Gazebo is running
if ! ros2 topic list | grep -q "/scan"; then
    echo "ERROR: Gazebo failed to start!"
    exit 1
fi
echo "   ✓ Gazebo running"

echo "4. Starting Navigation2..."
gnome-terminal --title="[2] Nav2" -- bash -c "
cd ~/RIS-TurtleBot3-Autonomous-Navigation
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch launch/nav2_maze.launch.py
exec bash"

echo "   Waiting 25 seconds for Nav2..."
sleep 25

# Verify Nav2 is running
if ! ros2 node list | grep -q "map_server"; then
    echo "ERROR: Nav2 failed to start!"
    exit 1
fi
echo "   ✓ Nav2 running"

echo "5. Setting initial pose..."
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
echo "   ✓ Initial pose set"

echo "6. Manually activating bt_navigator..."
sleep 3
# Try multiple times
for i in {1..5}; do
    if ros2 lifecycle set /bt_navigator activate 2>&1 | grep -q "successful"; then
        echo "   ✓ bt_navigator activated"
        break
    fi
    echo "   Attempt $i failed, waiting..."
    sleep 2
done

echo "7. Starting RViz with fixed QoS..."
gnome-terminal --title="[3] RViz" -- bash -c "
cd ~/RIS-TurtleBot3-Autonomous-Navigation
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
rviz2 -d config/nav2_default_view.rviz
exec bash"

sleep 5

echo ""
echo "========================================"
echo "✅ SYSTEM STARTED"
echo "========================================"
echo ""
echo "Check in RViz:"
echo "  • Map should be visible (black/white maze)"
echo "  • Red laser scan from LIDAR"
echo "  • Green AMCL particle cloud"
echo "  • Robot model at origin"
echo ""
echo "To navigate:"
echo "  1. Click 'Nav2 Goal' in RViz toolbar"
echo "  2. Click destination on map"
echo "  3. Robot navigates in Gazebo and RViz"
echo ""
echo "To check status:"
echo "  ./verify_system.sh"
echo ""
