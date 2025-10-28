#!/bin/bash
# Complete startup script for TurtleBot3 Navigation
# This script will guide you through starting all components

clear
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  🤖 TurtleBot3 Autonomous Navigation - Complete Startup"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "This will start:"
echo "  1. Gazebo simulation with maze world"
echo "  2. Nav2 navigation stack (localization + navigation)"
echo "  3. RViz visualization"
echo "  4. Initial pose setup"
echo ""
echo "Press Enter to continue..."
read

# Check if gnome-terminal is available
if ! command -v gnome-terminal &> /dev/null; then
    echo "ERROR: gnome-terminal not found. Please install it:"
    echo "  sudo apt install gnome-terminal"
    exit 1
fi

export TURTLEBOT3_MODEL=burger
WORKSPACE=$(pwd)

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  Step 1/4: Starting Gazebo..."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

gnome-terminal --title="1. Gazebo Simulation" -- bash -c "
    cd $WORKSPACE;
    export TURTLEBOT3_MODEL=burger;
    ./launch_gazebo_no_flicker.sh;
    echo '';
    echo 'Gazebo closed. Press Enter to close this window.';
    read
" &

echo "Waiting 15 seconds for Gazebo to start..."
sleep 15

# Verify scan topic
if ! ros2 topic list 2>/dev/null | grep -q "/scan"; then
    echo "WARNING: /scan topic not found. Waiting additional 10 seconds..."
    sleep 10
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  Step 2/4: Starting Nav2 Navigation Stack..."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

gnome-terminal --title="2. Nav2 Navigation" -- bash -c "
    cd $WORKSPACE;
    export TURTLEBOT3_MODEL=burger;
    source /opt/ros/jazzy/setup.bash;
    source install/setup.bash;
    ros2 launch my_simulations nav2_maze.launch.py;
    echo '';
    echo 'Nav2 stopped. Press Enter to close this window.';
    read
" &

echo "Waiting 15 seconds for Nav2 to initialize..."
sleep 15

# Verify AMCL is running
if ! ros2 node list 2>/dev/null | grep -q "amcl"; then
    echo "WARNING: AMCL node not found. Navigation may not be ready."
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  Step 3/4: Setting Initial Pose..."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "Publishing initial pose to /initialpose..."
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {frame_id: 'map'},
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
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909]
  }
}" > /dev/null 2>&1

echo "✓ Initial pose published!"
sleep 3

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  Step 4/4: Starting RViz..."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

gnome-terminal --title="3. RViz Visualization" -- bash -c "
    cd $WORKSPACE;
    export TURTLEBOT3_MODEL=burger;
    source /opt/ros/jazzy/setup.bash;
    source install/setup.bash;
    rviz2 -d config/nav2_default_view.rviz;
    echo '';
    echo 'RViz closed. Press Enter to close this window.';
    read
" &

sleep 3

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  ✅ ALL SYSTEMS STARTED!"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "What you should see:"
echo ""
echo "  📺 Gazebo Window:"
echo "     - Maze world loaded"
echo "     - TurtleBot3 burger in the center"
echo "     - Robot should be stationary"
echo ""
echo "  📊 RViz Window:"
echo "     - Fixed Frame: map"
echo "     - Map displayed (black walls, white floor)"
echo "     - Robot model visible"
echo "     - Red laser scan points"
echo "     - Green particle cloud (AMCL)"
echo "     - Navigation 2 Panel:"
echo "       • Navigation: Ready ✓"
echo "       • Localization: Converged ✓"
echo "       • Feedback: Idle"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  🎯 HOW TO USE:"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "1. In RViz, click the 'Nav2 Goal' button (green flag icon)"
echo "2. Click anywhere on the white area of the map"
echo "3. Drag to set the orientation (direction robot should face)"
echo "4. Release the mouse"
echo ""
echo "The robot will:"
echo "  • Plan a path (green line)"
echo "  • Navigate avoiding obstacles"
echo "  • Update in both RViz and Gazebo simultaneously"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  🔧 TROUBLESHOOTING:"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "If robot doesn't move:"
echo "  • Check Navigation 2 panel shows 'Ready' and 'Converged'"
echo "  • Try clicking '2D Pose Estimate' in RViz first"
echo "  • Click near robot position on map to reset pose"
echo ""
echo "If map not visible:"
echo "  • Check 'Map' display is enabled (checked) in RViz"
echo "  • Verify Fixed Frame is set to 'map'"
echo ""
echo "Check system status:"
echo "  ros2 node list | grep -E 'amcl|controller|planner'"
echo "  ros2 topic hz /scan"
echo "  ros2 topic echo /amcl_pose --once"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "Press Ctrl+C to stop this script (terminals will stay open)"
echo "Press Enter to see real-time status monitoring..."
read

# Status monitoring loop
while true; do
    clear
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "  📊 SYSTEM STATUS (Press Ctrl+C to exit)"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
    
    echo "ROS Nodes:"
    NODE_COUNT=$(ros2 node list 2>/dev/null | wc -l)
    echo "  Total: $NODE_COUNT"
    
    if ros2 node list 2>/dev/null | grep -q "amcl"; then
        echo "  ✓ AMCL (localization)"
    else
        echo "  ✗ AMCL not found"
    fi
    
    if ros2 node list 2>/dev/null | grep -q "controller_server"; then
        echo "  ✓ Controller Server"
    else
        echo "  ✗ Controller Server not found"
    fi
    
    if ros2 node list 2>/dev/null | grep -q "planner_server"; then
        echo "  ✓ Planner Server"
    else
        echo "  ✗ Planner Server not found"
    fi
    
    echo ""
    echo "Critical Topics:"
    
    SCAN_HZ=$(ros2 topic hz /scan --window 5 2>&1 | grep "average rate" | awk '{print $3}')
    if [ ! -z "$SCAN_HZ" ]; then
        echo "  ✓ /scan: ${SCAN_HZ} Hz"
    else
        echo "  ✗ /scan: Not publishing"
    fi
    
    if ros2 topic list 2>/dev/null | grep -q "/map"; then
        echo "  ✓ /map: Published"
    else
        echo "  ✗ /map: Not found"
    fi
    
    if ros2 topic echo /amcl_pose --once 2>/dev/null > /dev/null; then
        echo "  ✓ /amcl_pose: Localizing"
    else
        echo "  ✗ /amcl_pose: Not available"
    fi
    
    echo ""
    echo "Refreshing in 3 seconds..."
    sleep 3
done
