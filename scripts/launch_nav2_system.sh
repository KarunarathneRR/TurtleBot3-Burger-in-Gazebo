#!/usr/bin/env bash
# Complete launcher with proper Nav2 initialization and RViz config
# Usage: ./scripts/launch_nav2_system.sh

set -e

echo "=========================================="
echo " TurtleBot3 Nav2 Complete System Launcher"
echo "=========================================="
echo ""

# Check dependencies
if ! command -v gnome-terminal &> /dev/null; then
    echo "ERROR: gnome-terminal not found. Install with: sudo apt install gnome-terminal"
    exit 1
fi

if [ ! -f "package.xml" ]; then
    echo "ERROR: Run this from the workspace root!"
    exit 1
fi

# Set environment
export TURTLEBOT3_MODEL=burger
WORKSPACE=$(pwd)
ROS_SETUP="/opt/ros/jazzy/setup.bash"
RVIZ_CONFIG="$WORKSPACE/config/nav2_default_view.rviz"

echo "Configuration:"
echo "  TURTLEBOT3_MODEL: $TURTLEBOT3_MODEL"
echo "  Workspace: $WORKSPACE"
echo "  RViz Config: $RVIZ_CONFIG"
echo ""

# Source ROS and workspace
source $ROS_SETUP
source install/setup.bash

echo "Step 1/4: Launching Gazebo with maze world..."
gnome-terminal --title="Gazebo World" -- bash -c "
    export TURTLEBOT3_MODEL=burger;
    source $ROS_SETUP;
    source $WORKSPACE/install/setup.bash;
    echo '';
    echo '=== GAZEBO WORLD ===';
    echo 'Starting maze simulation...';
    echo '';
    ros2 launch my_simulations maze_world.launch.py;
    echo '';
    echo 'Gazebo closed. Press Enter to close this window.';
    read
" &

echo "Waiting 15 seconds for Gazebo to initialize..."
sleep 15

# Verify Gazebo topics
echo "Verifying Gazebo topics..."
if ! ros2 topic list | grep -q "/scan"; then
    echo "WARNING: /scan topic not found. Gazebo may not be ready."
    echo "Waiting additional 10 seconds..."
    sleep 10
fi

echo ""
echo "Step 2/4: Launching Nav2 navigation stack..."
gnome-terminal --title="Nav2 Stack" -- bash -c "
    export TURTLEBOT3_MODEL=burger;
    source $ROS_SETUP;
    source $WORKSPACE/install/setup.bash;
    echo '';
    echo '=== NAV2 NAVIGATION STACK ===';
    echo 'Starting Nav2 nodes...';
    echo '';
    echo 'This will start:';
    echo '  - AMCL (localization)';
    echo '  - Map Server';
    echo '  - Planner Server';
    echo '  - Controller Server';
    echo '  - BT Navigator';
    echo '  - Lifecycle Manager';
    echo '';
    ros2 launch my_simulations nav2_maze.launch.py;
    echo '';
    echo 'Nav2 stopped. Press Enter to close this window.';
    read
" &

echo "Waiting 15 seconds for Nav2 to initialize..."
sleep 15

# Verify Nav2 nodes
echo "Verifying Nav2 nodes..."
if ! ros2 node list | grep -q "amcl"; then
    echo "WARNING: AMCL node not found. Nav2 may not be ready."
    echo "Checking what's running:"
    ros2 node list || true
fi

echo ""
echo "Step 3/4: Launching RViz with Nav2 configuration..."
gnome-terminal --title="RViz Visualization" -- bash -c "
    export TURTLEBOT3_MODEL=burger;
    source $ROS_SETUP;
    source $WORKSPACE/install/setup.bash;
    echo '';
    echo '=== RVIZ VISUALIZATION ===';
    echo 'Starting RViz with Nav2 config...';
    echo '';
    echo 'RViz Tips:';
    echo '  1. Fixed Frame should be: map';
    echo '  2. Use \"2D Pose Estimate\" to set robot initial pose';
    echo '  3. Use \"Nav2 Goal\" to set navigation goals';
    echo '  4. Check Navigation 2 panel for status';
    echo '';
    if [ -f '$RVIZ_CONFIG' ]; then
        rviz2 -d '$RVIZ_CONFIG';
    else
        echo 'RViz config not found, starting with defaults...';
        rviz2;
    fi;
    echo '';
    echo 'RViz closed. Press Enter to close this window.';
    read
" &

echo "Waiting 5 seconds for RViz to start..."
sleep 5

echo ""
echo "Step 4/4: Opening status monitor..."
gnome-terminal --title="System Status" -- bash -c "
    export TURTLEBOT3_MODEL=burger;
    source $ROS_SETUP;
    source $WORKSPACE/install/setup.bash;
    echo '';
    echo '=== SYSTEM STATUS ===';
    echo '';
    echo '--- Running Nodes ---';
    ros2 node list;
    echo '';
    echo '--- Active Topics ---';
    ros2 topic list | grep -E 'scan|cmd_vel|map|amcl|plan|costmap' || echo 'No key topics found!';
    echo '';
    echo '--- LaserScan Status ---';
    timeout 2 ros2 topic hz /scan || echo '/scan not publishing';
    echo '';
    echo '================================';
    echo 'System Check Complete';
    echo '================================';
    echo '';
    echo 'Expected Status:';
    echo '  ✓ 10+ nodes running';
    echo '  ✓ /scan publishing at ~10 Hz';
    echo '  ✓ /map published';
    echo '  ✓ AMCL localizing';
    echo '';
    echo 'In RViz Navigation 2 panel should show:';
    echo '  - Navigation: Ready';
    echo '  - Localization: Converged';
    echo '  - Feedback: Navigating (when goal set)';
    echo '';
    echo '================================';
    echo 'HOW TO USE:';
    echo '================================';
    echo '1. In RViz, click \"2D Pose Estimate\"';
    echo '   - Click on map where robot is (near 0,0)';
    echo '   - This helps AMCL localize';
    echo '';
    echo '2. Wait for AMCL particle cloud to converge';
    echo '   - Should see green arrows around robot';
    echo '';
    echo '3. Click \"Nav2 Goal\" button';
    echo '   - Click destination on map';
    echo '   - Robot will plan and navigate!';
    echo '';
    echo 'Press Enter to run continuous monitoring...';
    read;
    echo '';
    echo '--- Continuous Monitoring ---';
    echo 'Ctrl+C to stop';
    echo '';
    while true; do
        clear;
        echo 'System Status (refreshing every 3s)';
        echo '==================================';
        echo '';
        echo 'Nodes: ' \$(ros2 node list | wc -l);
        echo 'Topics: ' \$(ros2 topic list | wc -l);
        echo '';
        echo 'Critical Topics:';
        ros2 topic hz /scan --window 10 2>&1 | head -2 || echo '  /scan: NOT PUBLISHING';
        ros2 topic hz /cmd_vel --window 10 2>&1 | head -2 || echo '  /cmd_vel: NOT PUBLISHING';
        echo '';
        echo 'Nav2 Nodes:';
        ros2 node list | grep -E 'amcl|planner|controller|bt_navigator' || echo '  No Nav2 nodes found!';
        echo '';
        sleep 3;
    done;
    exec bash
" &

echo ""
echo "=========================================="
echo "✅ ALL COMPONENTS LAUNCHED!"
echo "=========================================="
echo ""
echo "Four terminal windows have opened:"
echo "  1. Gazebo - Simulation world"
echo "  2. Nav2 - Navigation stack"
echo "  3. RViz - Visualization"
echo "  4. Status - System monitor"
echo ""
echo "IMPORTANT STEPS IN RVIZ:"
echo "  1. Set initial pose with \"2D Pose Estimate\""
echo "  2. Wait for AMCL to converge (green arrows)"
echo "  3. Set goal with \"Nav2 Goal\" button"
echo "  4. Watch robot navigate!"
echo ""
echo "If Navigation panel still shows 'unknown':"
echo "  - Check Nav2 terminal for errors"
echo "  - Run: ros2 node list | grep nav2"
echo "  - Make sure all 4 windows fully loaded"
echo ""
echo "Press Ctrl+C to stop this monitor..."
trap "echo 'To stop all components, close each terminal window.'; exit" INT
sleep infinity
