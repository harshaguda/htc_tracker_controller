# Controller 6D Pose Publisher

ROS2 Humble node for publishing 6D pose (position + orientation) of HTC Vive Pro controller relative to a tracker, with delta pose tracking when the trigger button is pressed.

## Features

- **Continuous 6D Pose Publishing**: Publishes the full pose (position + quaternion orientation) of the controller relative to the tracker frame
- **Delta Pose Tracking**: When the trigger button is pressed, publishes the difference between the current and previous poses
- **High-Frequency Updates**: Configurable publishing rate (default: 100 Hz)
- **Transform Broadcasting**: Publishes static transforms for RViz visualization

## Topics

### Published Topics

- `/controller_relative_pose_6d` (`geometry_msgs/msg/PoseStamped`)
  - Continuously published relative 6D pose of the controller
  - Frame ID: tracker name (default: `tracker_1`)
  - Contains:
    - `position.{x,y,z}`: Position in meters relative to tracker
    - `orientation.{w,x,y,z}`: Quaternion orientation relative to tracker

- `/controller_delta_pose` (`geometry_msgs/msg/PoseStamped`)
  - Published only when trigger button is pressed
  - Frame ID: tracker name (default: `tracker_1`)
  - Contains:
    - `position.{x,y,z}`: Position delta (change from previous frame)
    - `orientation.{w,x,y,z}`: Orientation delta as quaternion (q_current * q_previous^-1)

## Parameters

- `controller_name` (string, default: "controller_1")
  - Name of the HTC Vive controller device
  
- `tracker_name` (string, default: "tracker_1")
  - Name of the HTC Vive tracker device (used as reference frame)
  
- `publish_rate` (double, default: 100.0)
  - Publishing frequency in Hz
  
- `verbose` (bool, default: false)
  - Enable verbose output from the VR system

## Usage

### Method 1: Run directly

```bash
# Source the workspace
source /root/ros2_ws/install/setup.bash

# Run the node
ros2 run htc_tracker_controller controller_6d_pose_publisher
```

### Method 2: Use launch file

```bash
# Source the workspace
source /root/ros2_ws/install/setup.bash

# Launch with default parameters
ros2 launch htc_tracker_controller controller_6d_pose.launch.py

# Launch with custom parameters
ros2 launch htc_tracker_controller controller_6d_pose.launch.py \
    controller_name:=controller_1 \
    tracker_name:=tracker_1 \
    publish_rate:=100.0 \
    verbose:=false
```

### Method 3: Set parameters at runtime

```bash
ros2 run htc_tracker_controller controller_6d_pose_publisher \
    --ros-args \
    -p controller_name:=controller_1 \
    -p tracker_name:=tracker_1 \
    -p publish_rate:=100.0 \
    -p verbose:=false
```

## Monitoring Topics

### View continuous pose

```bash
ros2 topic echo /controller_relative_pose_6d
```

### View delta pose (only when trigger is pressed)

```bash
ros2 topic echo /controller_delta_pose
```

### Check topic frequency

```bash
ros2 topic hz /controller_relative_pose_6d
```

## Understanding the Output

### Pose Message Structure

```yaml
header:
  stamp:
    sec: 1234567890
    nanosec: 123456789
  frame_id: "tracker_1"
pose:
  position:
    x: 0.123  # meters
    y: 0.456  # meters
    z: 0.789  # meters
  orientation:
    w: 1.0    # quaternion components
    x: 0.0
    y: 0.0
    z: 0.0
```

### Delta Pose Interpretation

When the trigger is pressed:
- **Position Delta**: Shows how much the controller moved since the previous frame
  - Values in meters
  - Example: `x: 0.001` means moved 1mm in the x-direction
  
- **Orientation Delta**: Shows how much the controller rotated since the previous frame
  - Represented as a quaternion
  - Identity quaternion `[1, 0, 0, 0]` means no rotation
  - Small values indicate small rotational changes

## Use Cases

1. **Robot Control**: Use the 6D pose to control a robot's end-effector position and orientation
2. **Teleoperation**: Map controller movements to remote system control
3. **Motion Capture**: Record controller trajectories for analysis
4. **Teaching by Demonstration**: Capture poses when trigger is pressed to teach waypoints
5. **Gesture Recognition**: Analyze delta poses to recognize motion patterns

## Coordinate Frames

```
world
  └─ tracker_1 (reference frame)
       └─ controller_1 (published pose)
```

- The tracker frame is defined as the origin
- All controller poses are relative to this tracker frame
- X, Y, Z axes follow the OpenVR coordinate system

## Trigger Button Behavior

- **Press Trigger**: Delta pose tracking starts
  - Node logs: "🎮 Trigger pressed - Starting delta pose tracking"
  - `/controller_delta_pose` topic begins publishing
  
- **Hold Trigger**: Delta poses are published continuously while held
  - Each message shows the change from the previous frame
  - Delta poses are **only** published while the trigger is actively pressed
  
- **Release Trigger**: Delta pose tracking stops immediately
  - Node logs: "🎮 Trigger released - Stopped delta pose tracking"
  - `/controller_delta_pose` topic stops publishing immediately

## Troubleshooting

### Node fails to start

1. Check if SteamVR is running:
   ```bash
   ps aux | grep steam
   ```

2. Verify devices are connected:
   - Check that both controller and tracker are powered on
   - Ensure they are paired with the base stations
   - Look for green LED indicators

3. Check device names:
   ```bash
   # The node will print detected devices on startup
   # Adjust controller_name and tracker_name parameters accordingly
   ```

### No delta poses published

- Make sure you're pressing the **trigger button** (not touchpad or grip)
- Check if the trigger events are being detected in the node logs
- Verify `/controller_delta_pose` topic exists: `ros2 topic list`

### Low publishing rate

- Check system load: `htop`
- Verify the `publish_rate` parameter is set correctly
- Ensure USB bandwidth is sufficient for tracking data

## Building

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select htc_tracker_controller
source install/setup.bash
```

## Dependencies

- ROS2 Humble
- rclcpp
- geometry_msgs
- tf2_ros
- htc_vive_tracker library
- OpenVR API

## Related Nodes

- `htc_tracker_pos_publisher`: Publishes only position (3D) without orientation

## License

Apache-2.0
