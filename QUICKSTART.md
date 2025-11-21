# Quick Start Guide - Controller 6D Pose Publisher

## Prerequisites

✅ HTC Vive Pro controller powered on and paired  
✅ HTC Vive tracker powered on and paired  
✅ SteamVR running  
✅ Base stations tracking both devices  

## 5-Minute Quick Start

### Step 1: Build the Package

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select htc_tracker_controller
source install/setup.bash
```

### Step 2: Run the Node

```bash
ros2 run htc_tracker_controller controller_6d_pose_publisher
```

You should see:
```
[INFO] VR System initialized successfully
[INFO] Both controller_1 and tracker_1 detected!
[INFO] Publishing controller 6D pose relative to tracker at 100.0 Hz
```

### Step 3: View the Output

Open a new terminal:

```bash
source /root/ros2_ws/install/setup.bash

# Watch the continuous 6D pose
ros2 topic echo /controller_relative_pose_6d
```

### Step 4: Test Delta Pose Tracking

In another terminal:

```bash
source /root/ros2_ws/install/setup.bash

# Watch delta poses (only published while trigger is held down)
ros2 topic echo /controller_delta_pose
```

Now **press and hold the trigger button** on the controller. You should see delta pose messages appearing! When you release the trigger, the messages will stop immediately.

## Visualize in RViz (Optional)

### Terminal 1: Run the node
```bash
source /root/ros2_ws/install/setup.bash
ros2 run htc_tracker_controller controller_6d_pose_publisher
```

### Terminal 2: Launch RViz
```bash
source /root/ros2_ws/install/setup.bash
rviz2 -d /root/ros2_ws/src/htc_tracker_controller/rviz/controller_6d_pose.rviz
```

You should see:
- 🟠 Orange arrow: Current controller pose
- 🟢 Green arrow: Delta pose (appears when trigger is pressed)
- 📐 Coordinate frames showing tracker and world frames

## Common Commands

### Check if topics are publishing
```bash
ros2 topic list
```

### Check publishing frequency
```bash
ros2 topic hz /controller_relative_pose_6d
```

### See topic info
```bash
ros2 topic info /controller_relative_pose_6d
```

### Record data to a bag file
```bash
ros2 bag record /controller_relative_pose_6d /controller_delta_pose
```

### Play back recorded data
```bash
ros2 bag play <bag_file_name>
```

## Custom Configuration

### Change device names
```bash
ros2 run htc_tracker_controller controller_6d_pose_publisher \
    --ros-args \
    -p controller_name:=controller_2 \
    -p tracker_name:=tracker_1
```

### Increase publishing rate to 200 Hz
```bash
ros2 run htc_tracker_controller controller_6d_pose_publisher \
    --ros-args \
    -p publish_rate:=200.0
```

### Enable verbose output
```bash
ros2 run htc_tracker_controller controller_6d_pose_publisher \
    --ros-args \
    -p verbose:=true
```

## Using the Launch File

```bash
# Basic launch
ros2 launch htc_tracker_controller controller_6d_pose.launch.py

# With custom parameters
ros2 launch htc_tracker_controller controller_6d_pose.launch.py \
    controller_name:=controller_1 \
    tracker_name:=tracker_1 \
    publish_rate:=100.0
```

## Test Scenarios

### 1. Basic Position Tracking
- Run the node
- Move the controller around
- Watch the position values change in `/controller_relative_pose_6d`

### 2. Orientation Tracking
- Run the node
- Rotate the controller (without moving position)
- Watch the quaternion values change

### 3. Delta Tracking for Waypoints
- Run the node
- Subscribe to `/controller_delta_pose`
- Press trigger at desired waypoint locations
- Record the delta poses to learn movement patterns

### 4. Continuous Motion Capture
- Run the node with bag recording
- Press and hold trigger
- Move controller through desired trajectory
- Stop trigger and recording
- Play back to analyze motion

## Troubleshooting

| Issue | Solution |
|-------|----------|
| "No devices detected" | Check SteamVR is running, devices are on and paired |
| "controller_1 not detected" | Check controller is powered on, or change parameter name |
| "tracker_1 not detected" | Check tracker is powered on, or change parameter name |
| No delta poses | Make sure you're pressing the **trigger button** (bottom button) |
| Low publishing rate | Check CPU load, reduce publish_rate parameter |
| Topics not visible | Make sure to source the workspace: `source install/setup.bash` |

## Example Output

### Continuous Pose Topic
```yaml
header:
  stamp:
    sec: 1700000000
    nanosec: 123456789
  frame_id: "tracker_1"
pose:
  position:
    x: 0.234    # meters
    y: -0.156
    z: 0.412
  orientation:
    w: 0.987
    x: 0.012
    y: -0.089
    z: 0.134
```

### Delta Pose Topic (when trigger pressed)
```yaml
header:
  stamp:
    sec: 1700000000
    nanosec: 133456789
  frame_id: "tracker_1"
pose:
  position:
    x: 0.0012   # 1.2mm movement since last frame
    y: -0.0008
    z: 0.0005
  orientation:
    w: 0.9998   # Small rotation
    x: 0.0001
    y: -0.0002
    z: 0.0003
```

## Next Steps

1. **Integrate with your application**: Subscribe to the topics in your own ROS2 nodes
2. **Process the data**: Use the 6D pose to control robots, log trajectories, etc.
3. **Extend functionality**: Add more button handlers, filter the data, or compute velocities
4. **Read the full documentation**: See `CONTROLLER_6D_POSE_README.md` for detailed info

## Getting Help

- Check the README: `CONTROLLER_6D_POSE_README.md`
- Compare nodes: `NODES_COMPARISON.md`
- View node logs for error messages
- Ensure all dependencies are installed

## Success Checklist

- [ ] Package built successfully
- [ ] Node starts without errors
- [ ] Both devices detected on startup
- [ ] `/controller_relative_pose_6d` topic is publishing
- [ ] Position values change when moving controller
- [ ] Orientation values change when rotating controller
- [ ] Trigger button press is detected
- [ ] `/controller_delta_pose` publishes when trigger is pressed
- [ ] Delta poses stop when trigger is released

Congratulations! You're now tracking your HTC Vive Pro controller in 6D! 🎉
