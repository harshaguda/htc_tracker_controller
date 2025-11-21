# HTC Tracker Controller Nodes Comparison

## Overview

This package provides two ROS2 nodes for tracking HTC Vive Pro controller relative to a tracker:

1. **htc_tracker_pos_publisher** - Position-only tracking (3D)
2. **controller_6d_pose_publisher** - Full 6D pose tracking with delta pose support ⭐ NEW

## Feature Comparison

| Feature | htc_tracker_pos_publisher | controller_6d_pose_publisher |
|---------|---------------------------|------------------------------|
| Position (X, Y, Z) | ✅ | ✅ |
| Orientation (Quaternion) | ❌ | ✅ |
| Full 6D Pose | ❌ | ✅ |
| Delta Pose Tracking | ❌ | ✅ |
| Trigger Button Detection | ⚠️ Basic | ✅ Full Support |
| Publishing Rate | 10 Hz default | 100 Hz default |
| Distance Calculation | ✅ | ✅ |

## Topics Comparison

### htc_tracker_pos_publisher

```
/controller_relative_position    (geometry_msgs/PointStamped)
/controller_relative_pose        (geometry_msgs/PoseStamped) *orientation always identity
```

### controller_6d_pose_publisher

```
/controller_relative_pose_6d     (geometry_msgs/PoseStamped) *full orientation
/controller_delta_pose           (geometry_msgs/PoseStamped) *published when trigger pressed
```

## When to Use Each Node

### Use `htc_tracker_pos_publisher` when:
- ✅ You only need 3D position tracking
- ✅ Orientation doesn't matter for your application
- ✅ Lower update rate is sufficient (10 Hz)
- ✅ You want simpler output data
- ✅ You're doing basic distance measurements

**Example applications:**
- Simple proximity detection
- 2D navigation with altitude
- Basic position logging
- Distance-based triggers

### Use `controller_6d_pose_publisher` when:
- ✅ You need full 6D pose (position + orientation)
- ✅ You want to track rotational movements
- ✅ You need delta pose (movement increments)
- ✅ Higher update rate is required (100 Hz)
- ✅ You want trigger button integration
- ✅ You're doing precise manipulation tasks

**Example applications:**
- Robot arm teleoperation
- Teaching robot waypoints
- Full motion capture
- VR-to-robot mapping
- Gesture recognition
- Precision assembly tasks

## Migration Guide

### From htc_tracker_pos_publisher to controller_6d_pose_publisher

If you're currently using `htc_tracker_pos_publisher` and want to upgrade:

#### 1. Update your launch command

**Old:**
```bash
ros2 run htc_tracker_controller htc_tracker_pos_publisher
```

**New:**
```bash
ros2 run htc_tracker_controller controller_6d_pose_publisher
```

#### 2. Update your topic subscriptions

**Old code:**
```python
# Only position available
self.subscription = self.create_subscription(
    PointStamped,
    '/controller_relative_position',
    self.listener_callback,
    10)

def listener_callback(self, msg):
    x = msg.point.x
    y = msg.point.y
    z = msg.point.z
    # No orientation information
```

**New code:**
```python
# Full 6D pose available
self.subscription = self.create_subscription(
    PoseStamped,
    '/controller_relative_pose_6d',
    self.listener_callback,
    10)

def listener_callback(self, msg):
    # Position
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z
    
    # Orientation (quaternion)
    qw = msg.pose.orientation.w
    qx = msg.pose.orientation.x
    qy = msg.pose.orientation.y
    qz = msg.pose.orientation.z
```

#### 3. Add delta pose tracking (optional)

**New feature:**
```python
# Subscribe to delta poses when trigger is pressed
self.delta_subscription = self.create_subscription(
    PoseStamped,
    '/controller_delta_pose',
    self.delta_callback,
    10)

def delta_callback(self, msg):
    # This only fires when trigger is pressed
    dx = msg.pose.position.x  # Change in position
    dy = msg.pose.position.y
    dz = msg.pose.position.z
    
    # Orientation change as quaternion
    dq_w = msg.pose.orientation.w
    dq_x = msg.pose.orientation.x
    dq_y = msg.pose.orientation.y
    dq_z = msg.pose.orientation.z
```

## Performance Considerations

| Metric | htc_tracker_pos_publisher | controller_6d_pose_publisher |
|--------|---------------------------|------------------------------|
| CPU Usage | Lower | Slightly Higher |
| Network Bandwidth | Lower | Slightly Higher |
| Latency | ~100ms (10Hz) | ~10ms (100Hz) |
| Data Size/msg | Smaller | Larger |
| Math Operations | Fewer | More (matrix ops) |

## Code Examples

### Python Subscriber Example

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np

class Controller6DListener(Node):
    def __init__(self):
        super().__init__('controller_6d_listener')
        
        # Subscribe to full 6D pose
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/controller_relative_pose_6d',
            self.pose_callback,
            10)
        
        # Subscribe to delta pose
        self.delta_sub = self.create_subscription(
            PoseStamped,
            '/controller_delta_pose',
            self.delta_callback,
            10)
    
    def pose_callback(self, msg):
        pos = msg.pose.position
        ori = msg.pose.orientation
        self.get_logger().info(
            f'Pose: [{pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}] '
            f'Quat: [{ori.w:.3f}, {ori.x:.3f}, {ori.y:.3f}, {ori.z:.3f}]')
    
    def delta_callback(self, msg):
        # This only fires when trigger is pressed!
        dpos = msg.pose.position
        self.get_logger().info(
            f'Delta: [{dpos.x:.5f}, {dpos.y:.5f}, {dpos.z:.5f}]')

def main():
    rclpy.init()
    node = Controller6DListener()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### C++ Subscriber Example

```cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class Controller6DListener : public rclcpp::Node {
public:
    Controller6DListener() : Node("controller_6d_listener") {
        // Subscribe to full 6D pose
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/controller_relative_pose_6d", 10,
            std::bind(&Controller6DListener::pose_callback, this, std::placeholders::_1));
        
        // Subscribe to delta pose
        delta_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/controller_delta_pose", 10,
            std::bind(&Controller6DListener::delta_callback, this, std::placeholders::_1));
    }

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), 
            "Pose: [%.3f, %.3f, %.3f] Quat: [%.3f, %.3f, %.3f, %.3f]",
            msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
            msg->pose.orientation.w, msg->pose.orientation.x,
            msg->pose.orientation.y, msg->pose.orientation.z);
    }
    
    void delta_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // Only fires when trigger is pressed
        RCLCPP_INFO(this->get_logger(), "Delta: [%.5f, %.5f, %.5f]",
            msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    }
    
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr delta_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller6DListener>());
    rclcpp::shutdown();
    return 0;
}
```

## Summary

- **htc_tracker_pos_publisher**: Simple, lightweight position tracking
- **controller_6d_pose_publisher**: Advanced, full 6D pose with delta tracking and trigger button integration

Choose based on your application requirements!
