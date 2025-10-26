# Complete Step-by-Step Guide to Run the TurtleBot3 Maze Navigation Project

## ⚠️ IMPORTANT: Read this entire section before starting

### Current Problem Diagnosis
From your RViz screenshot, I can see:
- ✅ Map is loaded and visible
- ✅ RViz displays are configured (RobotModel, LaserScan, Map, AMCL, etc.)
- ❌ Navigation panel shows "unknown" - **Nav2 nodes are NOT running**
- ❌ You need to launch Nav2 separately

### The Fix: You Must Run 4 Separate Terminals

---

## Terminal 1: Launch Gazebo World with TurtleBot3

```bash
# Set TurtleBot3 model (REQUIRED!)
export TURTLEBOT3_MODEL=burger

# Source ROS and workspace
source /opt/ros/jazzy/setup.bash
source /home/shrinath/RIS-TurtleBot3-Autonomous-Navigation/install/setup.bash

# Launch the maze world
ros2 launch my_simulations maze_world.launch.py
```

**Expected output:** Gazebo window opens showing the maze world with TurtleBot3 spawned

**Verification:**
```bash
# In another terminal, check if topics exist:
ros2 topic list | grep -E "scan|cmd_vel|odom"
# Should see: /scan, /cmd_vel, /odom, /tf, etc.
```

---

## Terminal 2: Launch Nav2 Stack

**⚠️ WAIT** until Terminal 1 is fully running (Gazebo is open and robot is spawned)

```bash
# Set TurtleBot3 model
export TURTLEBOT3_MODEL=burger

# Source ROS and workspace
source /opt/ros/jazzy/setup.bash
source /home/shrinath/RIS-TurtleBot3-Autonomous-Navigation/install/setup.bash

# Launch Nav2
ros2 launch my_simulations nav2_maze.launch.py
```

**Expected output:** Many nodes starting including:
- `map_server`
- `amcl`
- `planner_server`
- `controller_server`
- `bt_navigator`
- `lifecycle_manager`

**Verification:**
```bash
# In another terminal:
ros2 node list
# Should see multiple nav2 nodes listed

# Check if map is published:
ros2 topic echo /map --once

# Check AMCL is localizing:
ros2 topic echo /amcl_pose --once
```

---

## Terminal 3: Launch RViz

**⚠️ WAIT** until both Terminal 1 and Terminal 2 are running

```bash
# Set TurtleBot3 model
export TURTLEBOT3_MODEL=burger

# Source ROS and workspace
source /opt/ros/jazzy/setup.bash
source /home/shrinath/RIS-TurtleBot3-Autonomous-Navigation/install/setup.bash

# Launch RViz
rviz2
```

**In RViz:**
1. Set **Fixed Frame** to `map` (top left, Global Options)
2. Your displays should now show data (not "unknown")
3. The Navigation 2 panel should show:
   - Navigation: Ready
   - Localization: Converged or Localizing
   - Feedback: Active

**Setting a Navigation Goal:**
1. Click the "Nav2 Goal" button (or "2D Nav Goal" in toolbar)
2. Click on the map where you want the robot to go
3. You should see:
   - A green line (global plan path)
   - The robot starts moving along the path
   - The robot avoids walls using the LaserScan data

---

## Terminal 4 (Optional): Launch DQN Runner

**⚠️ Only if you want DQN control instead of manual Nav2 goals**

```bash
# Set TurtleBot3 model
export TURTLEBOT3_MODEL=burger

# Source ROS and workspace
source /opt/ros/jazzy/setup.bash
source /home/shrinath/RIS-TurtleBot3-Autonomous-Navigation/install/setup.bash

# Launch DQN runner (requires trained model)
ros2 launch tb3_dqn_nav dqn_run.launch.py
```

**Note:** This requires a trained model at `~/.tb3_dqn.pt`. To train:
```bash
ros2 run tb3_dqn_nav dqn_train --episodes 200 --device cpu
```

---

## Complete Verification Checklist

Run these commands in a 5th terminal to verify everything is working:

```bash
source /opt/ros/jazzy/setup.bash
source /home/shrinath/RIS-TurtleBot3-Autonomous-Navigation/install/setup.bash

# 1. Check all nodes are running
echo "=== ROS 2 Nodes ==="
ros2 node list

# Expected nodes:
# /amcl
# /bt_navigator
# /controller_server
# /planner_server
# /map_server
# /lifecycle_manager_localization
# /lifecycle_manager_navigation
# /robot_state_publisher
# ... and more

# 2. Check critical topics exist
echo "=== Critical Topics ==="
ros2 topic list | grep -E "scan|cmd_vel|odom|map|amcl|plan"

# 3. Check LaserScan data
echo "=== LaserScan Test ==="
ros2 topic echo /scan --once

# 4. Check map is published
echo "=== Map Test ==="
ros2 topic info /map

# 5. Check TF tree
echo "=== TF Tree ==="
ros2 run tf2_tools view_frames
# This creates frames.pdf showing transform tree
```

---

## Troubleshooting Common Issues

### Issue 1: "Package 'my_simulations' not found"
**Solution:** You forgot to source the workspace
```bash
source /home/shrinath/RIS-TurtleBot3-Autonomous-Navigation/install/setup.bash
```

### Issue 2: Nav2 shows "unknown" in RViz
**Solution:** Nav2 nodes aren't running. Make sure Terminal 2 (nav2_maze.launch.py) is running without errors.

Check logs:
```bash
ros2 topic echo /diagnostics
```

### Issue 3: Robot passes through walls
**Possible causes:**
1. Costmap not configured properly
2. LaserScan not being received
3. Controller parameters too aggressive

**Solutions:**
```bash
# Check LaserScan is working:
ros2 topic hz /scan
# Should show ~10-30 Hz

# Check costmaps are published:
ros2 topic list | grep costmap

# Visualize costmaps in RViz:
# Add display: Map -> Topic: /global_costmap/costmap
# Add display: Map -> Topic: /local_costmap/costmap
```

### Issue 4: Gazebo flickering (VM issue)
The headless rendering flag should help, but if it still flickers:
```bash
# Alternative: run without GUI
ros2 launch my_simulations maze_world.launch.py gui:=false
```

### Issue 5: AMCL not localizing
**Solution:** Give AMCL an initial pose estimate
```bash
# In RViz, use "2D Pose Estimate" button
# Click on the map where the robot actually is
# This helps AMCL converge faster
```

---

## Quick Start Script (All in One Terminal using tmux)

If you have tmux installed:

```bash
# Make sure TURTLEBOT3_MODEL is exported for all panes
export TURTLEBOT3_MODEL=burger

# Run the automated launcher
./scripts/run_all.sh
```

**Tmux controls:**
- `Ctrl+b` then `arrow keys` - switch between panes
- `Ctrl+b` then `d` - detach from session
- `tmux attach -t tb3_maze` - reattach to session
- `Ctrl+b` then `x` then `y` - kill current pane

---

## Expected Behavior When Working

1. **Gazebo:** Shows maze with TurtleBot3, no excessive flickering
2. **RViz:** 
   - Map visible (black walls, white free space)
   - Robot model visible at correct position
   - LaserScan shows red dots detecting walls
   - AMCL particle cloud visible (green arrows)
   - Navigation panel shows "Ready" and "Converged"
3. **Navigation:**
   - Click "Nav2 Goal" in RViz
   - Click destination on map
   - Green path appears
   - Robot follows path
   - Robot smoothly avoids obstacles
   - Robot stops at goal

---

## Performance Notes

- **First launch:** Nav2 may take 10-30 seconds to fully initialize
- **AMCL convergence:** May take 5-10 seconds after launch
- **Path planning:** Should compute path in < 1 second
- **DQN training:** CPU training is slow (1-2 minutes per episode), GPU recommended for serious training

---

## Next Steps After Successful Run

1. **Tune Nav2 parameters:** Edit `config/nav2_params.yaml` to adjust:
   - Robot footprint
   - Planner algorithm
   - Controller gains
   - Costmap inflation

2. **Train DQN policy:**
   ```bash
   ros2 run tb3_dqn_nav dqn_train --episodes 500 --device cuda
   ```

3. **Create custom maps:**
   - Drive robot around in Gazebo
   - Use SLAM to create new maps:
   ```bash
   ros2 launch turtlebot3_cartographer cartographer.launch.py
   ```

4. **Add more complex behaviors:**
   - Waypoint following
   - Multi-robot coordination
   - Dynamic obstacle avoidance

---

## Critical Commands Summary

```bash
# Always run these FIRST in every terminal:
export TURTLEBOT3_MODEL=burger
source /opt/ros/jazzy/setup.bash
source /home/shrinath/RIS-TurtleBot3-Autonomous-Navigation/install/setup.bash

# Then run ONE of these per terminal:
ros2 launch my_simulations maze_world.launch.py          # Terminal 1
ros2 launch my_simulations nav2_maze.launch.py           # Terminal 2
rviz2                                                     # Terminal 3
ros2 launch tb3_dqn_nav dqn_run.launch.py               # Terminal 4 (optional)
```

---

## Success Criteria ✅

You'll know it's working when:
- [ ] Gazebo opens without crashing
- [ ] Robot spawns in maze
- [ ] `ros2 node list` shows 10+ nodes
- [ ] `/scan` topic publishes LaserScan data
- [ ] RViz shows map, robot, and laser scans
- [ ] Nav2 panel in RViz shows "Ready" and "Converged"
- [ ] Setting a Nav2 goal makes the robot move
- [ ] Robot follows planned path
- [ ] Robot stops at obstacles (doesn't pass through walls)
- [ ] Robot reaches goal position

If all checkboxes are true, **the project is running 100% successfully!** 🎉
