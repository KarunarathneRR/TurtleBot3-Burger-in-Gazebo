# 🚀 START HERE - Complete Instructions to Run the Project

## ⚠️ CRITICAL: Read This First

Your project has 3 main components that MUST be started in this exact order:
1. **Gazebo** - The maze world simulation with the TurtleBot3 robot
2. **Nav2** - The navigation stack that plans paths and controls the robot
3. **RViz** - The visualization tool where you set navigation goals

**If you skip Gazebo, Nav2 will show "unknown" in RViz!**

---

## 📋 Prerequisites (One-Time Setup)

Make sure you have built the workspace:
```bash
cd /home/shrinath/RIS-TurtleBot3-Autonomous-Navigation
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

---

## 🎯 STEP-BY-STEP INSTRUCTIONS (Follow Exactly)

### Step 1: Open 3 Terminal Windows

Open 3 separate terminal windows (Ctrl+Alt+T three times).

---

### Step 2: Terminal 1 - Start Gazebo Simulation

In Terminal 1, copy and paste these commands:

```bash
export TURTLEBOT3_MODEL=burger
source /opt/ros/jazzy/setup.bash
source /home/shrinath/RIS-TurtleBot3-Autonomous-Navigation/install/setup.bash
ros2 launch my_simulations maze_world.launch.py
```

**What to expect:**
- Gazebo window opens (may take 10-20 seconds)
- You see a maze world
- A small burger robot (TurtleBot3) appears in the maze
- Terminal shows "robot_state_publisher" and bridge messages

**⚠️ DO NOT PROCEED until Gazebo is fully loaded and robot is visible!**

---

### Step 3: Terminal 2 - Start Nav2 Navigation Stack

**WAIT 15 seconds after Step 2**, then in Terminal 2:

```bash
export TURTLEBOT3_MODEL=burger
source /opt/ros/jazzy/setup.bash
source /home/shrinath/RIS-TurtleBot3-Autonomous-Navigation/install/setup.bash
ros2 launch my_simulations nav2_maze.launch.py
```

**What to expect:**
- Many lines of text showing nodes starting
- You should see messages about:
  - "amcl" (localization)
  - "map_server" (loading the map)
  - "planner_server" (path planning)
  - "controller_server" (robot control)
  - "bt_navigator" (behavior tree)
- Eventually says "All lifecycle nodes are active"

**⚠️ DO NOT PROCEED until you see lifecycle nodes are active!**

---

### Step 4: Terminal 3 - Start RViz (Fixed Version)

**WAIT 10 seconds after Step 3**, then in Terminal 3:

```bash
export TURTLEBOT3_MODEL=burger
source /opt/ros/jazzy/setup.bash
source /home/shrinath/RIS-TurtleBot3-Autonomous-Navigation/install/setup.bash

# Start RViz without the config first (to avoid crash)
rviz2
```

**What to expect:**
- RViz window opens
- You should see a dark gray 3D view

---

### Step 5: Configure RViz Manually

Once RViz is open, follow these steps:

#### A. Set Fixed Frame
1. Look at left panel, "Global Options" section
2. Find "Fixed Frame" dropdown
3. Change from "base_link" to **"map"**

#### B. Add Map Display
1. Click "Add" button (bottom left)
2. Select "By display type" tab
3. Find and select "Map"
4. Click OK
5. In the new Map display:
   - Topic: `/map`
   - Color Scheme: map

#### C. Add Robot Model
1. Click "Add" button
2. Select "RobotModel"
3. Click OK
4. Topic: `/robot_description`

#### D. Add LaserScan
1. Click "Add" button
2. Select "LaserScan"
3. Click OK
4. Topic: `/scan`
5. Color: Red (or any bright color)
6. Size: 0.05

#### E. Add TF (Transforms)
1. Click "Add" button
2. Select "TF"
3. Click OK

#### F. Add Navigation 2 Panel
1. Click "Panels" menu (top)
2. Select "Add New Panel"
3. Find "Navigation 2"
4. Click OK
5. You should see Navigation 2 panel on the left

#### G. Add Path Displays (Optional but Recommended)
1. Add → Path → Topic: `/plan` (global plan, make it green)
2. Add → Path → Topic: `/local_plan` (local plan, make it red)

---

### Step 6: Give Initial Pose to AMCL

**This is CRITICAL - AMCL needs to know where the robot starts!**

1. In RViz toolbar, find "2D Pose Estimate" button
2. Click it
3. Click on the map approximately where the robot is (near center, coordinates 0,0)
4. Drag mouse to set the direction the robot faces
5. Release

**What to expect:**
- Green arrows appear around the robot (AMCL particle cloud)
- After 5-10 seconds, arrows converge to robot position
- Navigation 2 panel changes:
  - From "Localization: unknown" → "Localization: Converged"
  - From "Navigation: unknown" → "Navigation: Ready"

---

### Step 7: Set a Navigation Goal

Now you can command the robot!

1. Click "Nav2 Goal" or "2D Nav Goal" button in RViz toolbar
2. Click anywhere on the **white area** of the map (not black walls)
3. Drag to set goal orientation
4. Release

**What to expect:**
- Green line appears (global path plan)
- Red line appears (local path plan)
- Robot starts moving in Gazebo!
- Robot follows the path
- Robot avoids obstacles (walls)
- Robot reaches the goal
- In RViz, Navigation 2 panel shows "Feedback: Navigating"

---

## ✅ Verification Commands

Open a 4th terminal to verify everything is working:

```bash
source /opt/ros/jazzy/setup.bash
source /home/shrinath/RIS-TurtleBot3-Autonomous-Navigation/install/setup.bash

# Check how many nodes are running (should be 15+)
echo "Nodes running:"
ros2 node list | wc -l
ros2 node list

# Check if LaserScan is working
echo "LaserScan rate:"
ros2 topic hz /scan

# Check if map is published
echo "Map info:"
ros2 topic info /map

# Check Nav2 nodes specifically
echo "Nav2 nodes:"
ros2 node list | grep -E "amcl|planner|controller|navigator"
```

**Expected results:**
- 15+ nodes running
- /scan publishing at ~10-30 Hz
- /map topic exists with subscribers
- amcl, planner_server, controller_server, bt_navigator all listed

---

## 🐛 Troubleshooting

### Problem: "Package 'my_simulations' not found"
**Solution:** You didn't source the workspace
```bash
source /home/shrinath/RIS-TurtleBot3-Autonomous-Navigation/install/setup.bash
```

### Problem: RViz crashes immediately
**Solution:** 
- Don't use the config file initially
- Just run `rviz2` without `-d` flag
- Configure manually as shown in Step 5

### Problem: Navigation panel shows "unknown"
**Causes:**
1. Gazebo not running → Go back to Step 2
2. Nav2 not running → Go back to Step 3
3. Didn't set initial pose → Do Step 6

**Check what's running:**
```bash
ros2 node list
```
Should see: amcl, planner_server, controller_server, bt_navigator

### Problem: Robot passes through walls
**Causes:**
1. LaserScan not working
   ```bash
   ros2 topic echo /scan --once
   ```
   Should see ranges data

2. Costmaps not configured
   - In RViz, add Map displays for:
     - `/global_costmap/costmap`
     - `/local_costmap/costmap`
   - You should see walls marked in colors

3. Controller too aggressive
   - Lower max velocities in `config/nav2_params.yaml`

### Problem: Gazebo is flickering/slow
**Already fixed:** Launch file uses `--headless-rendering` flag
If still slow, you can:
```bash
# Run without GUI
ros2 launch my_simulations maze_world.launch.py gui:=false
```

### Problem: AMCL won't converge (stays "unknown")
**Solutions:**
1. Give better initial pose estimate (click closer to actual position)
2. Drive robot manually to help AMCL:
   ```bash
   # In new terminal:
   export TURTLEBOT3_MODEL=burger
   source /opt/ros/jazzy/setup.bash
   ros2 run turtlebot3_teleop teleop_keyboard
   ```
   Use keyboard to move robot around a bit
3. Wait longer (can take 10-20 seconds)

---

## 📊 Success Checklist

Your project is working correctly when ALL these are true:

- [ ] Gazebo window shows maze with robot
- [ ] Terminal 1 shows no errors
- [ ] Terminal 2 shows "All lifecycle nodes are active"
- [ ] `ros2 node list` shows 15+ nodes
- [ ] `ros2 topic echo /scan --once` returns data
- [ ] RViz shows map in white/black
- [ ] RViz shows robot model
- [ ] RViz shows red laser scan points
- [ ] Navigation 2 panel shows "Ready" (not "unknown")
- [ ] Navigation 2 panel shows "Converged" (not "unknown")
- [ ] Setting Nav2 Goal creates green path
- [ ] Robot moves in Gazebo when goal is set
- [ ] Robot follows the path
- [ ] Robot avoids walls (doesn't clip through)
- [ ] Robot reaches destination
- [ ] Navigation panel shows "Success" when done

**If ALL checkboxes are true: 🎉 PROJECT IS WORKING 100% CORRECTLY! 🎉**

---

## 🎮 Advanced Usage

### Manual Control (Teleoperation)
```bash
export TURTLEBOT3_MODEL=burger
source /opt/ros/jazzy/setup.bash
ros2 run turtlebot3_teleop teleop_keyboard
```
Use W/A/S/D/X keys to drive

### Save RViz Configuration
Once you have RViz configured:
1. File → Save Config As
2. Save to: `config/my_rviz.rviz`
3. Next time: `rviz2 -d config/my_rviz.rviz`

### View TF Tree
```bash
ros2 run tf2_tools view_frames
# Creates frames.pdf
evince frames.pdf
```
Should show: map → odom → base_footprint → base_link

### Monitor Topics
```bash
# Real-time topic rates
ros2 topic hz /scan
ros2 topic hz /cmd_vel
ros2 topic hz /odom

# Echo topic data
ros2 topic echo /amcl_pose
ros2 topic echo /plan
```

### Debug Navigation
```bash
# Check diagnostics
ros2 topic echo /diagnostics

# Check costmaps
ros2 topic echo /global_costmap/costmap --once
ros2 topic echo /local_costmap/costmap --once

# Test navigation via command line
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 1.0, y: 1.0, z: 0.0},
    orientation: {w: 1.0}
  }
}"
```

---

## 🔧 Alternative: Use Automatic Launcher

If manual steps are tedious, use the automatic launcher:

```bash
cd /home/shrinath/RIS-TurtleBot3-Autonomous-Navigation
./scripts/launch_nav2_system.sh
```

This opens 4 windows automatically but may have timing issues.

---

## 📞 Quick Help Script

```bash
# Check current system status
./scripts/check_nav2_status.sh

# Show simple copy-paste commands
./SIMPLE_RUN.sh
```

---

## 🎓 Understanding the System

**Gazebo**: Physics simulator, provides:
- Robot model
- Sensor data (/scan, /odom, /imu)
- Receives commands (/cmd_vel)

**Nav2**: Navigation stack, provides:
- Localization (AMCL) - "Where am I?"
- Global planner - "What path should I take?"
- Local planner - "How do I follow the path?"
- Controller - "What velocity commands to send?"

**RViz**: Visualization only
- Displays sensor data, maps, paths
- Allows you to set goals
- Shows robot state
- Does NOT control the robot directly

**Data Flow:**
1. Gazebo publishes /scan (laser)
2. AMCL uses /scan + /map to localize robot
3. You set goal in RViz
4. Planner creates path to goal
5. Controller sends /cmd_vel to follow path
6. Gazebo receives /cmd_vel and moves robot
7. Loop continues until goal reached

---

## 🚀 Quick Reference Card

```
┌─────────────────────────────────────────────┐
│         TERMINAL COMMANDS SUMMARY           │
├─────────────────────────────────────────────┤
│ ALWAYS START WITH (in every terminal):     │
│ export TURTLEBOT3_MODEL=burger              │
│ source /opt/ros/jazzy/setup.bash            │
│ source ~/...Navigation/install/setup.bash   │
├─────────────────────────────────────────────┤
│ TERMINAL 1 - Gazebo:                        │
│ ros2 launch my_simulations                  │
│              maze_world.launch.py           │
├─────────────────────────────────────────────┤
│ TERMINAL 2 - Nav2:                          │
│ ros2 launch my_simulations                  │
│              nav2_maze.launch.py            │
├─────────────────────────────────────────────┤
│ TERMINAL 3 - RViz:                          │
│ rviz2                                       │
│ Then configure manually!                    │
├─────────────────────────────────────────────┤
│ IN RVIZ:                                    │
│ 1. Fixed Frame → "map"                      │
│ 2. Add displays (Map, TF, LaserScan, etc.)  │
│ 3. Click "2D Pose Estimate"                 │
│ 4. Click "Nav2 Goal"                        │
│ 5. Robot navigates!                         │
└─────────────────────────────────────────────┘
```

---

**NOW FOLLOW THE STEPS ABOVE AND YOUR ROBOT WILL WORK PERFECTLY!** ✅
