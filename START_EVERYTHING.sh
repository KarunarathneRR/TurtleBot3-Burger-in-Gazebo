#!/bin/bash

# ===================================================
# COMPLETE STARTUP SCRIPT FOR TURTLEBOT3 NAVIGATION
# ===================================================

echo "========================================"
echo "  TurtleBot3 Navigation Startup Script"
echo "========================================"

# Set environment
export TURTLEBOT3_MODEL=burger
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3

# Source ROS and workspace
source /opt/ros/jazzy/setup.bash
source ~/RIS-TurtleBot3-Autonomous-Navigation/install/setup.bash

echo ""
echo "Step 1: Killing any existing processes..."
pkill -9 -f "ros2 launch"
pkill -9 -f "gzserver"
pkill -9 -f "rviz2"
sleep 2

echo ""
echo "Step 2: Starting Gazebo with maze world..."
echo "   (This will take about 20 seconds)"
ros2 launch ~/RIS-TurtleBot3-Autonomous-Navigation/launch/maze_world.launch.py &
GAZEBO_PID=$!
sleep 20

echo ""
echo "Step 3: Verifying Gazebo topics..."
timeout 5 ros2 topic echo /scan --once > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "   ✓ Gazebo is publishing /scan topic"
else
    echo "   ✗ ERROR: Gazebo not publishing. Please restart!"
    exit 1
fi

echo ""
echo "Step 4: Starting Nav2 navigation stack..."
ros2 launch ~/RIS-TurtleBot3-Autonomous-Navigation/launch/nav2_maze.launch.py &
NAV2_PID=$!
sleep 15

echo ""
echo "Step 5: Verifying Nav2 is ready..."
timeout 5 ros2 topic echo /map --once > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "   ✓ Nav2 is publishing /map topic"
else
    echo "   ✗ WARNING: /map not ready yet, but continuing..."
fi

echo ""
echo "Step 6: Starting RViz2..."
echo ""
echo "========================================"
echo "  IMPORTANT SETUP INSTRUCTIONS"
echo "========================================"
echo ""
echo "When RViz opens, do the following:"
echo ""
echo "1. Change 'Fixed Frame' from 'map' to 'odom' temporarily"
echo "   (RViz left panel → Global Options → Fixed Frame)"
echo ""
echo "2. Click 'Add' button (bottom left)"
echo ""
echo "3. Add these displays:"
echo "   - By display type → Map → Topic: /map"
echo "   - By display type → LaserScan → Topic: /scan, Size: 0.05"
echo "   - By display type → RobotModel"
echo "   - By display type → TF"
echo ""
echo "4. In menu: Panels → Add New Panel → Navigation 2"
echo ""
echo "5. NOW change Fixed Frame back to 'map'"
echo ""
echo "6. **CRITICAL**: Click '2D Pose Estimate' button (top toolbar)"
echo "   - Click on the white area in the center"
echo "   - Drag to set orientation"
echo "   - Wait 10-15 seconds"
echo ""
echo "7. Check Navigation 2 panel:"
echo "   - Should show 'Ready' instead of 'unknown'"
echo ""
echo "8. Test navigation:"
echo "   - Click 'Nav2 Goal' button"
echo "   - Click destination on map"
echo "   - Robot should move!"
echo ""
echo "========================================"
echo ""
echo "Press ENTER to start RViz2..."
read

rviz2

echo ""
echo "All processes stopped."
