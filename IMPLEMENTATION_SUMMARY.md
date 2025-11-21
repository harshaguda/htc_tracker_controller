# Controller 6D Pose Publisher - Implementation Summary

## Overview

A new ROS2 Humble node has been created to publish the 6D pose (position + orientation) of an HTC Vive Pro controller relative to a tracker, with delta pose tracking functionality triggered by the controller's trigger button.

## Created Files

### 1. Main Node Implementation
**File:** `/root/ros2_ws/src/htc_tracker_controller/src/controller_6d_pose_publisher.cpp`

**Key Features:**
- Full 6D pose tracking (position + quaternion orientation)
- Relative pose computation using transformation matrices
- Delta pose calculation (difference between current and previous poses)
- Trigger button state detection using OpenVR API
- Delta poses published **only while trigger button is held down**
- High-frequency updates (default: 100 Hz)

**Topics Published:**
- `/controller_relative_pose_6d` (geometry_msgs/PoseStamped)
  - Continuously publishes the full 6D pose
  - Position: x, y, z in meters
  - Orientation: quaternion (w, x, y, z)

- `/controller_delta_pose` (geometry_msgs/PoseStamped)
  - Published only when trigger button is pressed and held
  - Position delta: change in position since previous frame
  - Orientation delta: rotational change as quaternion

**Parameters:**
- `controller_name` (string, default: "controller_1")
- `tracker_name` (string, default: "tracker_1")
- `publish_rate` (double, default: 100.0 Hz)
- `verbose` (bool, default: false)

### 2. Launch File
**File:** `/root/ros2_ws/src/htc_tracker_controller/launch/controller_6d_pose.launch.py`

Provides easy launching with configurable parameters.

### 3. RViz Configuration
**File:** `/root/ros2_ws/src/htc_tracker_controller/rviz/controller_6d_pose.rviz`

Pre-configured visualization showing:
- Controller pose (orange arrow)
- Delta pose (green arrow, visible when trigger pressed)
- TF frames

### 4. Documentation
**Files:**
- `CONTROLLER_6D_POSE_README.md` - Comprehensive usage guide
- `QUICKSTART.md` - Quick start guide for new users
- `NODES_COMPARISON.md` - Comparison with the position-only node

## Technical Implementation Details

### Trigger Button Detection

The implementation uses OpenVR's `GetControllerState()` API to check the real-time state of the trigger button:

```cpp
vr::VRControllerState_t controller_state;
vr::VRSystem()->GetControllerState(controller_index, &controller_state, sizeof(controller_state));
uint64_t trigger_mask = vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Trigger);
trigger_currently_pressed = (controller_state.ulButtonPressed & trigger_mask) != 0;
```

**Key Behavior:**
- Delta poses are published **only** while the trigger is actively held down
- When trigger is released, delta pose publishing stops immediately
- Edge detection provides user feedback via log messages

### Pose Computation

**Relative Pose Calculation:**
1. Get world poses of both controller and tracker
2. Convert poses to 4x4 transformation matrices
3. Compute inverse of tracker transform
4. Multiply to get relative transform: T_relative = T_tracker^-1 * T_controller
5. Extract 6D pose (position + quaternion) from result

**Delta Pose Calculation:**
- Position delta: Simple subtraction (current - previous)
- Orientation delta: Quaternion multiplication (q_current * q_previous^-1)

### Coordinate Frames

```
world
  └─ tracker_1 (reference frame, origin)
       └─ controller_1 (published pose relative to tracker)
```

## Build Integration

Updated `CMakeLists.txt` to:
- Add new executable: `controller_6d_pose_publisher`
- Install launch files directory
- Install rviz configuration directory
- Link against htc_vive_tracker and OpenVR libraries

## Usage Examples

### Basic Run
```bash
source /root/ros2_ws/install/setup.bash
ros2 run htc_tracker_controller controller_6d_pose_publisher
```

### Launch with Parameters
```bash
ros2 launch htc_tracker_controller controller_6d_pose.launch.py \
    publish_rate:=200.0
```

### Monitor Topics
```bash
# Continuous pose
ros2 topic echo /controller_relative_pose_6d

# Delta pose (hold trigger to see output)
ros2 topic echo /controller_delta_pose
```

## Use Cases

1. **Robot Teleoperation** - Use 6D pose to control robot end-effector
2. **Waypoint Teaching** - Record poses when trigger is pressed
3. **Motion Capture** - Full trajectory recording with orientation
4. **Gesture Recognition** - Analyze delta poses for pattern recognition
5. **VR-to-Robot Mapping** - Direct pose mapping for manipulation tasks

## Advantages Over Position-Only Node

| Feature | Position-Only | 6D Pose Publisher |
|---------|---------------|-------------------|
| Orientation | ❌ | ✅ Full quaternion |
| Delta Tracking | ❌ | ✅ With trigger |
| Update Rate | 10 Hz | 100 Hz (configurable) |
| Trigger Detection | Basic | Real-time state |
| Use Cases | Simple tracking | Full manipulation |

## Testing

The node has been successfully built and is ready for testing with:
- HTC Vive Pro controller
- HTC Vive tracker
- SteamVR runtime

## Future Enhancements

Possible improvements:
- Add velocity computation from delta poses
- Support multiple controllers simultaneously
- Add filtering options (Kalman filter, moving average)
- Configurable button assignments (not just trigger)
- Record/playback functionality for trajectories

## Files Modified/Created Summary

**New Files:**
- `src/controller_6d_pose_publisher.cpp` (416 lines)
- `launch/controller_6d_pose.launch.py`
- `rviz/controller_6d_pose.rviz`
- `CONTROLLER_6D_POSE_README.md`
- `QUICKSTART.md`
- `NODES_COMPARISON.md`

**Modified Files:**
- `CMakeLists.txt` (added new executable and install rules)

**Build Status:** ✅ Successfully compiled
**Package Status:** ✅ Ready for deployment
