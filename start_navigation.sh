#!/bin/bash

# Function to check if topics are available
check_topics() {
    echo "Checking for required topics..."
    ros2 topic list | grep -q "/scan" && \
    ros2 topic list | grep -q "/odom" && \
    ros2 topic list | grep -q "/map"
    return $?
}

# Function to check if transforms are available
check_transforms() {
    echo "Checking for required transforms..."
    ros2 topic echo /tf --once 2>/dev/null | grep -q "map" && \
    ros2 topic echo /tf --once 2>/dev/null | grep -q "odom" && \
    ros2 topic echo /tf --once 2>/dev/null | grep -q "base_footprint"
    return $?
}

# First terminal: Start Gazebo
echo "🚀 Starting Gazebo simulation..."
export TURTLEBOT3_MODEL=burger
export LIBGL_ALWAYS_SOFTWARE=1
source /opt/ros/jazzy/setup.bash
source ~/RIS-TurtleBot3-Autonomous-Navigation/install/setup.bash
ros2 launch my_simulations maze_world.launch.py &
GAZEBO_PID=$!

# Wait for Gazebo to be ready (wait for /scan topic)
echo "⏳ Waiting for Gazebo and robot topics..."
while ! ros2 topic list | grep -q "/scan"; do
    sleep 1
done

# Second terminal: Start Nav2
echo "🎯 Starting Navigation2..."
ros2 launch my_simulations nav2_maze.launch.py &
NAV2_PID=$!

# Wait for Nav2 to be ready
echo "⏳ Waiting for Navigation2 stack..."
while ! ros2 node list | grep -q "amcl"; do
    sleep 1
done

# Verify all required components are running
echo "🔍 Verifying system state..."
if check_topics && check_transforms; then
    echo "✅ All systems ready!"
    echo "You can now start RViz2 and set the initial pose."
    echo ""
    echo "Instructions:"
    echo "1. In a new terminal, run: rviz2"
    echo "2. Set Fixed Frame to 'map'"
    echo "3. Add required displays (Map, RobotModel, LaserScan)"
    echo "4. Click '2D Pose Estimate' and set robot's position"
    echo "5. Use 'Navigation2 Goal' to set destination"
    echo ""
    echo "Press Ctrl+C to shut down everything cleanly"
else
    echo "❌ System not ready! Missing required topics or transforms"
    kill $GAZEBO_PID
    kill $NAV2_PID
    exit 1
fi

# Wait for Ctrl+C
wait

# Cleanup
kill $GAZEBO_PID
kill $NAV2_PID