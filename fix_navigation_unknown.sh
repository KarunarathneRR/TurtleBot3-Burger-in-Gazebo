#!/bin/bash

echo "╔════════════════════════════════════════════════════════╗"
echo "║  FIXING NAVIGATION 'UNKNOWN' ISSUE - STEP BY STEP     ║"
echo "╚════════════════════════════════════════════════════════╝"
echo ""

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Step 1: Kill everything
echo -e "${YELLOW}Step 1: Cleaning up old processes...${NC}"
killall -9 gzserver gzclient gz rviz2 robot_state_publisher 2>/dev/null
sleep 2
echo -e "${GREEN}✓ Cleanup complete${NC}"
echo ""

# Step 2: Source environment
echo -e "${YELLOW}Step 2: Setting up environment...${NC}"
export TURTLEBOT3_MODEL=burger
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
source /opt/ros/jazzy/setup.bash
source ~/RIS-TurtleBot3-Autonomous-Navigation/install/setup.bash
echo -e "${GREEN}✓ Environment ready${NC}"
echo ""

# Step 3: Start Gazebo
echo -e "${YELLOW}Step 3: Starting Gazebo simulation...${NC}"
ros2 launch my_simulations maze_world.launch.py > /tmp/gazebo.log 2>&1 &
GAZEBO_PID=$!
echo "Gazebo PID: $GAZEBO_PID"

# Wait for Gazebo topics
echo "Waiting for Gazebo to be ready..."
for i in {1..30}; do
    if ros2 topic list 2>/dev/null | grep -q "/scan"; then
        echo -e "${GREEN}✓ Gazebo is ready!${NC}"
        break
    fi
    echo -n "."
    sleep 2
done
echo ""

# Verify scan is publishing
echo "Checking /scan topic..."
if ros2 topic hz /scan --once --timeout 5 2>/dev/null; then
    echo -e "${GREEN}✓ LaserScan is publishing${NC}"
else
    echo -e "${RED}✗ LaserScan NOT publishing - check Gazebo!${NC}"
    tail -20 /tmp/gazebo.log
    exit 1
fi
echo ""

# Step 4: Start Nav2
echo -e "${YELLOW}Step 4: Starting Nav2 navigation stack...${NC}"
ros2 launch my_simulations nav2_maze.launch.py > /tmp/nav2.log 2>&1 &
NAV2_PID=$!
echo "Nav2 PID: $NAV2_PID"

# Wait for Nav2 to be ready
echo "Waiting for Nav2 nodes to activate..."
for i in {1..60}; do
    if ros2 node list 2>/dev/null | grep -q "amcl"; then
        sleep 5  # Give extra time for full initialization
        if grep -q "All lifecycle nodes are active" /tmp/nav2.log 2>/dev/null; then
            echo -e "${GREEN}✓ Nav2 is fully active!${NC}"
            break
        fi
    fi
    echo -n "."
    sleep 2
done
echo ""

# Step 5: Verify all critical topics
echo -e "${YELLOW}Step 5: Verifying system state...${NC}"
echo "Checking topics:"

TOPICS=("/scan" "/odom" "/map" "/amcl_pose" "/tf" "/tf_static")
ALL_OK=true

for topic in "${TOPICS[@]}"; do
    if ros2 topic list 2>/dev/null | grep -q "^${topic}$"; then
        echo -e "  ${GREEN}✓${NC} $topic"
    else
        echo -e "  ${RED}✗${NC} $topic - MISSING!"
        ALL_OK=false
    fi
done
echo ""

# Check nodes
echo "Checking Nav2 nodes:"
NODES=("amcl" "map_server" "planner_server" "controller_server" "bt_navigator")
for node in "${NODES[@]}"; do
    if ros2 node list 2>/dev/null | grep -q "$node"; then
        echo -e "  ${GREEN}✓${NC} /$node"
    else
        echo -e "  ${RED}✗${NC} /$node - MISSING!"
        ALL_OK=false
    fi
done
echo ""

if [ "$ALL_OK" = true ]; then
    echo -e "${GREEN}════════════════════════════════════════════════════${NC}"
    echo -e "${GREEN}✓ ALL SYSTEMS READY!${NC}"
    echo -e "${GREEN}════════════════════════════════════════════════════${NC}"
    echo ""
    echo "NOW DO THIS IN RVIZ:"
    echo "1. Make sure Fixed Frame = 'map'"
    echo "2. Click '2D Pose Estimate' button"
    echo "3. Click on map where robot is (center, around 0,0)"
    echo "4. Drag to set orientation"
    echo "5. WAIT 10 SECONDS for AMCL to converge"
    echo "6. Navigation panel should show 'Ready' instead of 'unknown'"
    echo ""
    echo "If RViz not open yet, run in new terminal:"
    echo "  export TURTLEBOT3_MODEL=burger"
    echo "  source /opt/ros/jazzy/setup.bash"
    echo "  source ~/RIS-TurtleBot3-Autonomous-Navigation/install/setup.bash"
    echo "  rviz2"
    echo ""
    echo "Press Ctrl+C to stop everything"
    wait
elsegive m
    echo -e "${RED}════════════════════════════════════════════════════${NC}"
    echo -e "${RED}✗ SOME SYSTEMS FAILED TO START${NC}"
    echo -e "${RED}════════════════════════════════════════════════════${NC}"
    echo ""
    echo "Check logs:"
    echo "  cat /tmp/gazebo.log"
    echo "  cat /tmp/nav2.log"
    
    # Kill processes
    kill $GAZEBO_PID 2>/dev/null
    kill $NAV2_PID 2>/dev/null
    exit 1
fi
