#!/usr/bin/env bash
# Quick diagnostic to check Nav2 system status
# Usage: ./scripts/check_nav2_status.sh

source /opt/ros/jazzy/setup.bash
source /home/shrinath/RIS-TurtleBot3-Autonomous-Navigation/install/setup.bash

echo "=========================================="
echo " Nav2 System Diagnostic"
echo "=========================================="
echo ""

echo "1. Checking ROS 2 daemon..."
if pgrep -f "ros2-daemon" > /dev/null; then
    echo "   ✓ ROS 2 daemon is running"
else
    echo "   ✗ ROS 2 daemon not running"
fi

echo ""
echo "2. Checking running nodes..."
NODE_COUNT=$(ros2 node list 2>/dev/null | wc -l)
if [ "$NODE_COUNT" -gt 0 ]; then
    echo "   ✓ Found $NODE_COUNT nodes:"
    ros2 node list | sed 's/^/     /'
else
    echo "   ✗ No ROS 2 nodes detected"
    echo "   → Make sure Gazebo and Nav2 are running!"
fi

echo ""
echo "3. Checking critical topics..."
TOPICS=$(ros2 topic list 2>/dev/null)
for topic in /scan /cmd_vel /odom /map /tf /amcl_pose; do
    if echo "$TOPICS" | grep -q "^$topic$"; then
        echo "   ✓ $topic exists"
    else
        echo "   ✗ $topic missing"
    fi
done

echo ""
echo "4. Checking Nav2 nodes..."
NAV2_NODES=("amcl" "planner_server" "controller_server" "bt_navigator" "map_server")
NODES_LIST=$(ros2 node list 2>/dev/null)
for node in "${NAV2_NODES[@]}"; do
    if echo "$NODES_LIST" | grep -q "$node"; then
        echo "   ✓ $node running"
    else
        echo "   ✗ $node NOT running"
    fi
done

echo ""
echo "5. Checking LaserScan data..."
if timeout 2 ros2 topic echo /scan --once >/dev/null 2>&1; then
    echo "   ✓ /scan is publishing data"
    ros2 topic hz /scan --window 5 2>&1 | head -2 | sed 's/^/     /'
else
    echo "   ✗ /scan not publishing or timeout"
fi

echo ""
echo "6. Checking map..."
if timeout 2 ros2 topic echo /map --once >/dev/null 2>&1; then
    echo "   ✓ /map is published"
else
    echo "   ✗ /map not available"
fi

echo ""
echo "7. Checking TF transforms..."
if timeout 2 ros2 topic echo /tf --once >/dev/null 2>&1; then
    echo "   ✓ /tf is publishing"
    echo "   Available frames:"
    timeout 3 ros2 run tf2_ros tf2_echo map base_link 2>&1 | grep -A 5 "Translation" | sed 's/^/     /' || echo "     → Cannot transform map->base_link"
else
    echo "   ✗ /tf not publishing"
fi

echo ""
echo "=========================================="
echo " Diagnosis Summary"
echo "=========================================="

if [ "$NODE_COUNT" -eq 0 ]; then
    echo "❌ PROBLEM: No nodes running"
    echo "   SOLUTION: Start Gazebo and Nav2:"
    echo "   Terminal 1: ros2 launch my_simulations maze_world.launch.py"
    echo "   Terminal 2: ros2 launch my_simulations nav2_maze.launch.py"
elif ! echo "$NODES_LIST" | grep -q "amcl"; then
    echo "❌ PROBLEM: Nav2 not running"
    echo "   SOLUTION: Start Nav2 in a separate terminal:"
    echo "   export TURTLEBOT3_MODEL=burger"
    echo "   source /opt/ros/jazzy/setup.bash"
    echo "   source ~/RIS-TurtleBot3-Autonomous-Navigation/install/setup.bash"
    echo "   ros2 launch my_simulations nav2_maze.launch.py"
elif ! echo "$TOPICS" | grep -q "/scan"; then
    echo "❌ PROBLEM: Gazebo not running or sensor not working"
    echo "   SOLUTION: Start Gazebo:"
    echo "   export TURTLEBOT3_MODEL=burger"
    echo "   source /opt/ros/jazzy/setup.bash"
    echo "   source ~/RIS-TurtleBot3-Autonomous-Navigation/install/setup.bash"
    echo "   ros2 launch my_simulations maze_world.launch.py"
else
    echo "✅ System appears to be running correctly!"
    echo ""
    echo "If RViz still shows 'unknown':"
    echo "  1. Make sure Fixed Frame is set to 'map' in RViz"
    echo "  2. Give initial pose estimate with '2D Pose Estimate' button"
    echo "  3. Wait 5-10 seconds for AMCL to converge"
    echo "  4. Try setting a Nav2 Goal"
fi

echo ""
