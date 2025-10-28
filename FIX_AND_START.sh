#!/bin/bash

# DEFINITIVE FIX AND START SCRIPT
# This script fixes all issues and starts the complete system

echo "============================================"
echo "TurtleBot3 Navigation - Complete Fix & Start"
echo "============================================"
echo ""

# Kill any existing processes
echo "Step 1: Cleaning up existing processes..."
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f gazebo 2>/dev/null || true
pkill -9 -f rviz2 2>/dev/null || true
pkill -9 -f ros2 2>/dev/null || true
pkill -9 -f ruby 2>/dev/null || true
sleep 5
echo "  ✓ Cleanup complete"
echo ""

# Source environment
cd ~/RIS-TurtleBot3-Autonomous-Navigation
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger

echo "Step 2: Starting Gazebo (Terminal 1)..."
gnome-terminal --title="■ [1] GAZEBO" --geometry=80x24+0+0 -- bash -c '
echo "=========================================="
echo "GAZEBO - TurtleBot3 Maze World"
echo "=========================================="
cd ~/RIS-TurtleBot3-Autonomous-Navigation
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
echo ""
echo "Starting Gazebo..."
./launch_gazebo_no_flicker.sh
'

echo "  Waiting 30 seconds for Gazebo to fully initialize..."
for i in {30..1}; do
    echo -ne "  $i seconds remaining...\r"
    sleep 1
done
echo -e "\n"

# Verify Gazebo
if ros2 topic list 2>/dev/null | grep -q "/scan"; then
    echo "  ✓ Gazebo is running - /scan topic detected"
else
    echo "  ✗ ERROR: Gazebo not responding!"
    echo "    Please check Terminal 1 for errors"
    exit 1
fi
echo ""

echo "Step 3: Starting Navigation2 (Terminal 2)..."
gnome-terminal --title="■ [2] NAVIGATION2" --geometry=80x24+700+0 -- bash -c '
echo "=========================================="
echo "NAVIGATION 2 - Nav Stack"
echo "=========================================="
cd ~/RIS-TurtleBot3-Autonomous-Navigation
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
echo ""
echo "Starting Navigation2..."
ros2 launch launch/nav2_maze.launch.py
'

echo "  Waiting 30 seconds for Nav2 to initialize..."
for i in {30..1}; do
    echo -ne "  $i seconds remaining...\r"
    sleep 1
done
echo -e "\n"

# Verify Nav2
if ros2 node list 2>/dev/null | grep -q "map_server"; then
    echo "  ✓ Nav2 is running - map_server detected"
else
    echo "  ✗ WARNING: Nav2 may not be fully started"
fi
echo ""

echo "Step 4: Checking system status..."
sleep 3

# Check lifecycle states
echo "  Checking critical nodes..."
NODES_OK=0

if ros2 lifecycle get /map_server 2>/dev/null | grep -q "active"; then
    echo "    ✓ map_server: active"
    NODES_OK=$((NODES_OK+1))
else
    echo "    ✗ map_server: NOT active"
fi

if ros2 lifecycle get /amcl 2>/dev/null | grep -q "active"; then
    echo "    ✓ amcl: active"
    NODES_OK=$((NODES_OK+1))
else
    echo "    ✗ amcl: NOT active"
fi

if ros2 lifecycle get /controller_server 2>/dev/null | grep -q "active"; then
    echo "    ✓ controller_server: active"
    NODES_OK=$((NODES_OK+1))
else
    echo "    ✗ controller_server: NOT active"
fi

echo ""

echo "Step 5: Setting initial robot pose..."
sleep 2
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"},
  pose: {
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    },
    covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467]
  }
}' >/dev/null 2>&1

echo "  ✓ Initial pose published to /initialpose"
echo ""

sleep 3

echo "Step 6: Verifying map publication..."
if timeout 10 ros2 topic echo /map --once >/dev/null 2>&1; then
    MAP_DATA=$(ros2 topic echo /map --once 2>/dev/null | grep -E "width|height|resolution" | head -3)
    echo "  ✓ Map is publishing:"
    echo "$MAP_DATA" | sed 's/^/    /'
else
    echo "  ✗ Map not publishing (may need more time)"
fi
echo ""

echo "Step 7: Checking QoS compatibility..."
TOPIC_INFO=$(ros2 topic info /map -v 2>/dev/null)
PUB_QOS=$(echo "$TOPIC_INFO" | grep -A 10 "Node name: map_server" | grep "Durability:" | awk '{print $2}')
echo "  Map publisher QoS: $PUB_QOS"

if [ "$PUB_QOS" == "TRANSIENT_LOCAL" ]; then
    echo "  ✓ Correct QoS (TRANSIENT_LOCAL)"
else
    echo "  ⚠ Unexpected QoS: $PUB_QOS"
fi
echo ""

echo "Step 8: Starting RViz (Terminal 3)..."
sleep 2
gnome-terminal --title="■ [3] RVIZ2" --geometry=80x24+1400+0 -- bash -c '
echo "=========================================="
echo "RVIZ2 - Visualization"
echo "=========================================="
cd ~/RIS-TurtleBot3-Autonomous-Navigation
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
echo ""
echo "Starting RViz2..."
echo "Config: config/nav2_default_view.rviz"
echo ""
rviz2 -d config/nav2_default_view.rviz
'

sleep 8

echo "Step 9: Verifying RViz subscription..."
if ros2 node list 2>/dev/null | grep -q "rviz"; then
    echo "  ✓ RViz is running"
    
    # Check RViz QoS
    SUB_QOS=$(ros2 topic info /map -v 2>/dev/null | grep -A 10 "Node name: rviz" | grep "Durability:" | awk '{print $2}')
    echo "  RViz map subscriber QoS: $SUB_QOS"
    
    if [ "$SUB_QOS" == "TRANSIENT_LOCAL" ]; then
        echo "  ✓ QoS matches! Map should be visible in RViz"
    else
        echo "  ✗ QoS mismatch! Map won't be received"
        echo "    Fix: Update config/nav2_default_view.rviz"
    fi
else
    echo "  ⚠ RViz not detected yet (may still be loading)"
fi
echo ""

echo "============================================"
echo "✅ STARTUP COMPLETE"
echo "============================================"
echo ""
echo "WHAT YOU SHOULD SEE IN RVIZ:"
echo ""
echo "  ✓ Black/white maze map (if QoS is correct)"
echo "  ✓ Red laser scan from LIDAR"
echo "  ✓ Green particle cloud (AMCL localization)"
echo "  ✓ Robot model at origin (0, 0)"
echo "  ✓ TF frames: map → odom → base_footprint"
echo ""
echo "IF MAP NOT VISIBLE IN RVIZ:"
echo ""
echo "  1. Wait 10 more seconds (map may be loading)"
echo "  2. Check 'Map' display is enabled (left panel)"
echo "  3. Check 'Topic' shows '/map'"
echo "  4. Check 'Status' doesn't show 'No messages received'"
echo ""
echo "TO NAVIGATE:"
echo ""
echo "  1. Click 'Nav2 Goal' button (top toolbar)"
echo "  2. Click destination on map"
echo "  3. Arrow appears → robot plans path"
echo "  4. Robot moves in BOTH Gazebo and RViz"
echo ""
echo "TROUBLESHOOTING:"
echo ""
echo "  Run: ./verify_system.sh"
echo "  Check terminals for errors"
echo ""
echo "TERMINALS:"
echo "  [1] Gazebo   - Simulation"
echo "  [2] Nav2     - Navigation stack"
echo "  [3] RViz     - Visualization"
echo ""
echo "Press Ctrl+C in each terminal to stop"
echo "============================================"
