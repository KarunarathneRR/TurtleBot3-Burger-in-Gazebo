#!/bin/bash

echo "╔═══════════════════════════════════════════════════════╗"
echo "║  COMPLETE FIX - RESTART EVERYTHING PROPERLY          ║"
echo "╚═══════════════════════════════════════════════════════╝"
echo ""

# Step 1: Kill everything
echo "Step 1: Killing all processes..."
killall -9 gzserver gzclient gz ruby rviz2 robot_state_publisher 2>/dev/null
pkill -9 -f "ros2 launch" 2>/dev/null
sleep 3
echo "✓ All processes killed"
echo ""

# Step 2: Source environment
echo "Step 2: Setting up environment..."
export TURTLEBOT3_MODEL=burger
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
source /opt/ros/jazzy/setup.bash
source ~/RIS-TurtleBot3-Autonomous-Navigation/install/setup.bash
echo "✓ Environment ready"
echo ""

# Step 3: Start Gazebo
echo "Step 3: Starting Gazebo..."
echo "This will open Gazebo window - wait for it!"
ros2 launch my_simulations maze_world.launch.py &
GAZEBO_PID=$!
echo "Gazebo PID: $GAZEBO_PID"
echo ""

# Wait for Gazebo
echo "Waiting for Gazebo to fully start..."
for i in {1..60}; do
    if ros2 topic hz /scan --once 2>/dev/null | grep -q "rate"; then
        echo "✓ Gazebo is ready and /scan is publishing!"
        break
    fi
    echo -n "."
    sleep 2
done
echo ""
echo ""

# Step 4: Start Nav2
echo "Step 4: Starting Nav2..."
ros2 launch my_simulations nav2_maze.launch.py &
NAV2_PID=$!
echo "Nav2 PID: $NAV2_PID"
echo ""

# Wait for Nav2
echo "Waiting for Nav2 to activate..."
sleep 15
echo "✓ Nav2 should be ready"
echo ""

# Step 5: Verify
echo "Step 5: System verification..."
echo ""

echo "Checking topics:"
ros2 topic list | grep -E "scan|odom|map" | while read topic; do
    echo "  ✓ $topic"
done
echo ""

echo "Checking nodes:"
ros2 node list | grep -E "amcl|planner|controller|bt_navigator" | while read node; do
    echo "  ✓ $node"
done
echo ""

echo "╔═══════════════════════════════════════════════════════╗"
echo "║              ✓ SYSTEM IS READY!                      ║"
echo "╚═══════════════════════════════════════════════════════╝"
echo ""
echo "NOW DO THIS:"
echo ""
echo "1. Open NEW terminal and run:"
echo "   export TURTLEBOT3_MODEL=burger"
echo "   source /opt/ros/jazzy/setup.bash"
echo "   source ~/RIS-TurtleBot3-Autonomous-Navigation/install/setup.bash"
echo "   rviz2"
echo ""
echo "2. In RViz:"
echo "   - Set Fixed Frame to 'map'"
echo "   - Click Add → Map (topic: /map)"
echo "   - Click Add → RobotModel"
echo "   - Click Add → LaserScan (topic: /scan)"
echo "   - Click Add → TF"
echo "   - Add Navigation 2 panel"
echo "   - Click '2D Pose Estimate' and click on map"
echo "   - Wait 10 seconds"
echo "   - Status should change to 'Ready'"
echo ""
echo "Press Ctrl+C to stop everything"
echo ""

# Keep running
wait
