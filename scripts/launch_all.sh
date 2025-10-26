#!/usr/bin/env bash
# Simple launch script that runs all commands in sequence
# This is easier than tmux for beginners

set -e

echo "=================================="
echo "TurtleBot3 Maze Navigation Launcher"
echo "=================================="
echo ""
echo "This script will open 4 terminal windows:"
echo "  1. Gazebo with maze world"
echo "  2. Nav2 navigation stack"
echo "  3. RViz visualization"
echo "  4. Status checker"
echo ""
echo "Press Ctrl+C to stop all processes"
echo "=================================="
echo ""

# Check if we're in the right directory
if [ ! -f "package.xml" ]; then
    echo "ERROR: Run this from the workspace root!"
    exit 1
fi

# Set environment
export TURTLEBOT3_MODEL=burger
WORKSPACE=$(pwd)
ROS_SETUP="/opt/ros/jazzy/setup.bash"

# Source ROS and workspace
source $ROS_SETUP
source install/setup.bash

echo "Starting Gazebo in 2 seconds..."
sleep 2

# Launch Gazebo in background with logging
gnome-terminal -- bash -c "
    export TURTLEBOT3_MODEL=burger;
    source $ROS_SETUP;
    source $WORKSPACE/install/setup.bash;
    echo '=== TERMINAL 1: Gazebo World ===';
    echo 'Starting maze world...';
    ros2 launch my_simulations maze_world.launch.py;
    exec bash
" 2>&1 &

echo "Waiting 15 seconds for Gazebo to fully start..."
sleep 15

# Launch Nav2
gnome-terminal -- bash -c "
    export TURTLEBOT3_MODEL=burger;
    source $ROS_SETUP;
    source $WORKSPACE/install/setup.bash;
    echo '=== TERMINAL 2: Nav2 Stack ===';
    echo 'Starting Nav2...';
    ros2 launch my_simulations nav2_maze.launch.py;
    exec bash
" 2>&1 &

echo "Waiting 10 seconds for Nav2 to initialize..."
sleep 10

# Launch RViz
gnome-terminal -- bash -c "
    export TURTLEBOT3_MODEL=burger;
    source $ROS_SETUP;
    source $WORKSPACE/install/setup.bash;
    echo '=== TERMINAL 3: RViz ===';
    echo 'Starting RViz...';
    echo '';
    echo 'In RViz:';
    echo '  1. Set Fixed Frame to: map';
    echo '  2. Use Nav2 Goal button to set destination';
    echo '  3. Watch robot navigate!';
    echo '';
    rviz2;
    exec bash
" 2>&1 &

echo "Waiting 5 seconds for RViz to start..."
sleep 5

# Open status checker
gnome-terminal -- bash -c "
    export TURTLEBOT3_MODEL=burger;
    source $ROS_SETUP;
    source $WORKSPACE/install/setup.bash;
    echo '=== TERMINAL 4: System Status ===';
    echo '';
    echo 'Checking nodes...';
    ros2 node list;
    echo '';
    echo 'Checking topics...';
    ros2 topic list | grep -E 'scan|cmd_vel|map|amcl';
    echo '';
    echo '================================';
    echo 'If you see nodes and topics above, the system is running!';
    echo '';
    echo 'To test navigation:';
    echo '  1. Go to RViz window';
    echo '  2. Click Nav2 Goal button';
    echo '  3. Click on map to set destination';
    echo '  4. Robot should navigate to goal';
    echo '';
    echo 'To monitor in real-time:';
    echo '  ros2 topic echo /cmd_vel';
    echo '  ros2 topic hz /scan';
    echo '';
    echo 'Press Enter to run continuous status check...';
    read;
    echo 'Monitoring topics (Ctrl+C to stop):';
    watch -n 2 'ros2 topic list | wc -l; ros2 node list | wc -l';
    exec bash
" &

echo ""
echo "=================================="
echo "✅ All components launched!"
echo "=================================="
echo ""
echo "What to do next:"
echo "  1. Wait for all windows to fully load (30-60 seconds total)"
echo "  2. Check Terminal 4 (Status) - should show nodes and topics"
echo "  3. In RViz window:"
echo "     - Verify Fixed Frame is set to 'map'"
echo "     - You should see the maze map"
echo "     - Navigation panel should show 'Ready'"
echo "  4. Click 'Nav2 Goal' button in RViz toolbar"
echo "  5. Click on the map where you want the robot to go"
echo "  6. Watch the robot navigate!"
echo ""
echo "To stop everything: Close all terminal windows or press Ctrl+C in each"
echo ""

# Wait for user interrupt
echo "Press Ctrl+C to stop monitoring..."
trap "echo 'Launcher stopped. Close individual terminal windows to stop processes.'; exit" INT
sleep infinity
