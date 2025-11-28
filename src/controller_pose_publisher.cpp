#include "htc_vive_tracker.h"
#include <openvr.h>
#include <cmath>

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include "geometry_msgs/msg/pose.hpp"


// Structure to hold 6D pose (position + orientation)
struct Pose6D {
    double position[3];      // x, y, z
    double quaternion[4];    // w, x, y, z
};

// Function to compute delta pose (difference between two poses)
void ComputeDeltaPose(const Pose6D& current_pose, const Pose6D& previous_pose, Pose6D& delta_pose) {
    // Compute position delta (simple subtraction)
    delta_pose.position[0] = current_pose.position[0] - previous_pose.position[0];
    delta_pose.position[1] = current_pose.position[1] - previous_pose.position[1];
    delta_pose.position[2] = current_pose.position[2] - previous_pose.position[2];
    
    // Compute orientation delta: q_delta = q_current * q_previous^-1
    // For unit quaternions, inverse is the conjugate
    double q_prev_inv[4] = {
        previous_pose.quaternion[0],   // w (same)
        -previous_pose.quaternion[1],  // -x
        -previous_pose.quaternion[2],  // -y
        -previous_pose.quaternion[3]   // -z
    };
    
    // Quaternion multiplication: q_delta = q_current * q_prev_inv
    double w1 = current_pose.quaternion[0], x1 = current_pose.quaternion[1];
    double y1 = current_pose.quaternion[2], z1 = current_pose.quaternion[3];
    double w2 = q_prev_inv[0], x2 = q_prev_inv[1];
    double y2 = q_prev_inv[2], z2 = q_prev_inv[3];
    
    delta_pose.quaternion[0] = w1*w2 - x1*x2 - y1*y2 - z1*z2;  // w
    delta_pose.quaternion[1] = w1*x2 + x1*w2 + y1*z2 - z1*y2;  // x
    delta_pose.quaternion[2] = w1*y2 - x1*z2 + y1*w2 + z1*x2;  // y
    delta_pose.quaternion[3] = w1*z2 + x1*y2 - y1*x2 + z1*w2;  // z
}

class ControllerPosePublisher : public rclcpp::Node {
public:
    ControllerPosePublisher() 
        : Node("controller_pose_publisher"), 
          vt_(),
          controller_name_("controller_1"),
          trigger_pressed_(false),
          grip_pressed_(false),
          trigger_start_pose_captured_(false),
          delta_duration_(0.2),
          gripper_width_(0.0),
          gripper_closed_threshold_(0.01)
    {
        // Declare and get parameters
        this->declare_parameter("controller_name", "controller_1");
        this->declare_parameter("publish_rate", 100.0);
        this->declare_parameter("verbose", false);
        this->declare_parameter("frame_id", "world");
        this->declare_parameter("delta_duration", 0.2);
        
        controller_name_ = this->get_parameter("controller_name").as_string();
        double publish_rate = this->get_parameter("publish_rate").as_double();
        bool verbose = this->get_parameter("verbose").as_bool();
        frame_id_ = this->get_parameter("frame_id").as_string();
        delta_duration_ = this->get_parameter("delta_duration").as_double();
        
        // Initialize HTC Vive Tracker
        if (!vt_.InitializeVR(verbose)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize VR system");
            throw std::runtime_error("VR initialization failed");
        }
        
        // Check if required devices are detected
        if (vt_.GetAllDeviceNames().size() == 1) {
            RCLCPP_ERROR(this->get_logger(), "No devices detected. Check that devices are connected and paired");
            throw std::runtime_error("No VR devices detected");
        }
        
        RCLCPP_INFO(this->get_logger(), "VR System initialized successfully");
        vt_.PrintAllDetectedDevices();
        
        // Check if controller is detected
        if (!vt_.IsDeviceDetected(controller_name_)) {
            RCLCPP_ERROR(this->get_logger(), "%s not detected!", controller_name_.c_str());
            throw std::runtime_error("Controller not detected");
        }
        
        RCLCPP_INFO(this->get_logger(), "Controller %s detected!", controller_name_.c_str());
        
        // Create publishers
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "controller_pose", 10);
        
        delta_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "delta_pose", 10);
        
        vr_desired_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "vr_desired_pose", 10);
        
        gripper_command_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "gripper_command", 10);
        
        // Create subscriber for gripper joint states
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/franka_gripper/joint_states", 10,
            std::bind(&ControllerPosePublisher::joint_state_callback, this, std::placeholders::_1));
        
        // Create transform broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        
        // Create timer for publishing
        auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate));
        timer_ = this->create_wall_timer(
            timer_period,
            std::bind(&ControllerPosePublisher::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Publishing controller pose at %.1f Hz", publish_rate);
        RCLCPP_INFO(this->get_logger(), "Topics:");
        RCLCPP_INFO(this->get_logger(), "  - /controller_pose (continuous)");
        RCLCPP_INFO(this->get_logger(), "  - /vr_desired_pose (absolute pose while trigger held)");
        RCLCPP_INFO(this->get_logger(), "  - /delta_pose (delta every %.2f seconds while trigger held)", delta_duration_);
        RCLCPP_INFO(this->get_logger(), "  - /gripper_command (published when grip button pressed)");
        RCLCPP_INFO(this->get_logger(), "Frame: %s -> %s", frame_id_.c_str(), controller_name_.c_str());
    }

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // Find gripper finger joint and extract width
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i].find("finger") != std::string::npos) {
                // Gripper width is 2x the finger joint position
                gripper_width_ = msg->position[i] * 2.0;
                break;
            }
        }
    }
    
    void timer_callback() {
        // Update device poses
        vt_.Update();
        
        // Poll for button events (drain the event queue)
        while (vt_.EventPolling()) {
            // Just drain the event queue
        }
        
        // Get controller pose and quaternion
        double position[3], quaternion[4];
        if (!vt_.GetDevicePoseQuaternion(controller_name_, position, quaternion)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Could not get pose for %s", controller_name_.c_str());
            return;
        }
        
        // Store current pose in Pose6D structure
        Pose6D current_pose;
        current_pose.position[0] = position[0];
        current_pose.position[1] = position[1];
        current_pose.position[2] = position[2];
        current_pose.quaternion[0] = quaternion[0];
        current_pose.quaternion[1] = quaternion[1];
        current_pose.quaternion[2] = quaternion[2];
        current_pose.quaternion[3] = quaternion[3];
        
        auto now = this->now();
        
        // Check button states using OpenVR API
        bool trigger_currently_pressed = false;
        bool grip_currently_pressed = false;
        
        // Get the controller device index
        auto device_names = vt_.GetAllDeviceNames();
        vr::TrackedDeviceIndex_t controller_index = vr::k_unTrackedDeviceIndexInvalid;
        
        static bool logged_device_info = false;
        if (!logged_device_info) {
            RCLCPP_INFO(this->get_logger(), "Searching for controller: %s", controller_name_.c_str());
            RCLCPP_INFO(this->get_logger(), "Total devices detected: %zu", device_names.size());
            for (size_t i = 0; i < device_names.size(); i++) {
                RCLCPP_INFO(this->get_logger(), "  Device[%zu]: %s", i, device_names[i].c_str());
            }
            logged_device_info = true;
        }
        
        for (vr::TrackedDeviceIndex_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
            if (i < device_names.size() && device_names[i] == controller_name_) {
                controller_index = i;
                break;
            }
        }
        
        // Check if trigger and grip buttons are currently pressed
        static int button_check_counter = 0;
        if (controller_index != vr::k_unTrackedDeviceIndexInvalid) {
            vr::VRControllerState_t controller_state;
            if (vr::VRSystem()->GetControllerState(controller_index, &controller_state, sizeof(controller_state))) {
                // Check if trigger button is pressed
                uint64_t trigger_mask = vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Trigger) | vr::ButtonMaskFromId(vr::k_EButton_Axis1);
                trigger_currently_pressed = (controller_state.ulButtonPressed & trigger_mask) != 0;
                
                // Check if grip button is pressed
                uint64_t grip_mask = vr::ButtonMaskFromId(vr::k_EButton_Grip);
                grip_currently_pressed = (controller_state.ulButtonPressed & grip_mask) != 0;
                
                // Log button states periodically
                if (button_check_counter % 100 == 0) {
                    RCLCPP_INFO(this->get_logger(), 
                               "Controller[%u] State - Trigger: %s, Grip: %s, ButtonPressed: 0x%lx",
                               controller_index,
                               trigger_currently_pressed ? "PRESSED" : "released",
                               grip_currently_pressed ? "PRESSED" : "released",
                               (unsigned long)controller_state.ulButtonPressed);
                }
            } else {
                if (button_check_counter % 100 == 0) {
                    RCLCPP_WARN(this->get_logger(), "Failed to get controller state for index %u", controller_index);
                }
            }
        } else {
            if (button_check_counter % 100 == 0) {
                RCLCPP_WARN(this->get_logger(), "Controller index is invalid!");
            }
        }
        button_check_counter++;
        
        // Detect trigger press/release edges
        if (trigger_currently_pressed && !trigger_pressed_) {
            trigger_pressed_ = true;
            trigger_start_pose_captured_ = false;
            last_delta_publish_time_ = this->now();
            RCLCPP_INFO(this->get_logger(), "Trigger pressed - Starting robot control");
        } else if (!trigger_currently_pressed && trigger_pressed_) {
            trigger_pressed_ = false;
            trigger_start_pose_captured_ = false;
            RCLCPP_INFO(this->get_logger(), "Trigger released - Stopped robot control");
        }
        
        // Publish VR desired pose continuously while trigger is held (for admittance control)
        if (trigger_pressed_) {
            auto vr_desired_msg = geometry_msgs::msg::PoseStamped();
            vr_desired_msg.header.stamp = now;
            vr_desired_msg.header.frame_id = frame_id_;
            vr_desired_msg.pose.position.x = position[0];
            vr_desired_msg.pose.position.y = position[1];
            vr_desired_msg.pose.position.z = position[2];
            vr_desired_msg.pose.orientation.w = quaternion[0];
            vr_desired_msg.pose.orientation.x = quaternion[1];
            vr_desired_msg.pose.orientation.y = quaternion[2];
            vr_desired_msg.pose.orientation.z = quaternion[3];
            vr_desired_pose_pub_->publish(vr_desired_msg);
        }
        
        // Detect grip button press and publish gripper command
        if (grip_currently_pressed && !grip_pressed_) {
            grip_pressed_ = true;
            
            // Determine if gripper is closed (width < threshold) or open
            bool is_gripper_closed = (gripper_width_ < gripper_closed_threshold_);
            
            auto gripper_msg = std_msgs::msg::Bool();
            gripper_msg.data = is_gripper_closed;  // true if closed, false if open
            gripper_command_pub_->publish(gripper_msg);
            
            RCLCPP_INFO(this->get_logger(), "Grip button pressed - Gripper state: %s (width: %.4f m)",
                       is_gripper_closed ? "CLOSED" : "OPEN", gripper_width_);
        } else if (!grip_currently_pressed && grip_pressed_) {
            grip_pressed_ = false;
        }
        
        // Delta pose tracking with configurable duration intervals
        if (trigger_pressed_) {
            auto time_since_last_delta = (this->now() - last_delta_publish_time_).seconds();
            
            // Log delta timing info
            static int delta_counter = 0;
            if (delta_counter % 10 == 0) {
                RCLCPP_INFO(this->get_logger(), 
                           "Delta tracking: time_since_last=%.3f, delta_duration=%.3f, captured=%s",
                           time_since_last_delta, delta_duration_, 
                           trigger_start_pose_captured_ ? "YES" : "NO");
            }
            delta_counter++;
            
            // Check if it's time to capture/update and publish delta
            if (time_since_last_delta >= delta_duration_) {
                if (!trigger_start_pose_captured_) {
                    // First delta interval - capture the reference pose
                    trigger_start_pose_ = current_pose;
                    trigger_start_pose_captured_ = true;
                    last_delta_publish_time_ = this->now();
                    RCLCPP_INFO(this->get_logger(), "Captured reference pose after %.2f seconds", delta_duration_);
                } else {
                    // Subsequent intervals - compute and publish delta from reference
                    Pose6D delta_pose;
                    ComputeDeltaPose(current_pose, trigger_start_pose_, delta_pose);
                    
                    // Limit delta position magnitude to 0.04 meters
                    double delta_magnitude = sqrt(delta_pose.position[0]*delta_pose.position[0] +
                                                 delta_pose.position[1]*delta_pose.position[1] +
                                                 delta_pose.position[2]*delta_pose.position[2]);
                    RCLCPP_INFO(this->get_logger(), "Computed delta magnitude: %.4f m", delta_magnitude);
                    const double max_delta = 0.20;  // meters
                    if (delta_magnitude > max_delta) {
                        double scale_factor = max_delta / delta_magnitude;
                        delta_pose.position[0] *= scale_factor;
                        delta_pose.position[1] *= scale_factor;
                        delta_pose.position[2] *= scale_factor;
                    }
                    
                    auto delta_msg = geometry_msgs::msg::PoseStamped();
                    delta_msg.header.stamp = now;
                    delta_msg.header.frame_id = frame_id_;
                    delta_msg.pose.position.x = delta_pose.position[0];
                    delta_msg.pose.position.y = delta_pose.position[1];
                    delta_msg.pose.position.z = delta_pose.position[2];
                    // delta_msg.pose.orientation.x = delta_pose.quaternion[1];
                    // delta_msg.pose   .orientation.y = delta_pose.quaternion[2];
                    // delta_msg.pose.orientation.z = delta_pose.quaternion[3];
                    delta_msg.pose.orientation.w = 1.0;
                    delta_msg.pose.orientation.x = 0.0;
                    delta_msg.pose.orientation.y = 0.0;
                    delta_msg.pose.orientation.z = 0.0;
                    delta_pose_pub_->publish(delta_msg);
                    
                    // Log delta information
                    double pos_delta = sqrt(delta_pose.position[0]*delta_pose.position[0] +
                                           delta_pose.position[1]*delta_pose.position[1] +
                                           delta_pose.position[2]*delta_pose.position[2]);
                    RCLCPP_INFO(this->get_logger(), 
                               "Delta (%.2fs): ΔPos=%.4f m, ΔOri=[%.3f, %.3f, %.3f, %.3f]",
                               time_since_last_delta,
                               pos_delta,
                               delta_pose.quaternion[0], delta_pose.quaternion[1],
                               delta_pose.quaternion[2], delta_pose.quaternion[3]);
                    
                    // Update reference pose for next interval
                    trigger_start_pose_ = current_pose;
                    last_delta_publish_time_ = this->now();
                }
            }
        }
        
        // Create and publish PoseStamped message
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header.stamp = now;
        pose_msg.header.frame_id = frame_id_;
        pose_msg.pose.position.x = position[0];
        pose_msg.pose.position.y = position[1];
        pose_msg.pose.position.z = position[2];
        pose_msg.pose.orientation.w = quaternion[0];
        pose_msg.pose.orientation.x = quaternion[1];
        pose_msg.pose.orientation.y = quaternion[2];
        pose_msg.pose.orientation.z = quaternion[3];
        pose_pub_->publish(pose_msg);
        
        // Broadcast TF transform
        geometry_msgs::msg::TransformStamped transform_msg;
        transform_msg.header.stamp = now;
        transform_msg.header.frame_id = frame_id_;
        transform_msg.child_frame_id = controller_name_;
        transform_msg.transform.translation.x = position[0];
        transform_msg.transform.translation.y = position[1];
        transform_msg.transform.translation.z = position[2];
        transform_msg.transform.rotation.w = quaternion[0];
        transform_msg.transform.rotation.x = quaternion[1];
        transform_msg.transform.rotation.y = quaternion[2];
        transform_msg.transform.rotation.z = quaternion[3];
        tf_broadcaster_->sendTransform(transform_msg);
        
        // Log pose periodically (every 2 seconds)
        static int counter = 0;
        if (counter % 200 == 0) {
            double distance = sqrt(position[0]*position[0] +
                                  position[1]*position[1] +
                                  position[2]*position[2]);
            RCLCPP_INFO(this->get_logger(), 
                       "Controller Pose: Pos=[%.3f, %.3f, %.3f]m (dist=%.3f), Ori=[w=%.3f, x=%.3f, y=%.3f, z=%.3f]",
                       position[0], position[1], position[2], distance,
                       quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
        }
        counter++;
    }
    
    CHtc_Vive_Tracker vt_;
    std::string controller_name_;
    std::string frame_id_;
    
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr delta_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vr_desired_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_command_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    bool trigger_pressed_;
    bool grip_pressed_;
    
    // Delta pose tracking
    bool trigger_start_pose_captured_;
    Pose6D trigger_start_pose_;
    rclcpp::Time last_delta_publish_time_;
    double delta_duration_;
    
    // Gripper state tracking
    double gripper_width_;
    double gripper_closed_threshold_;
};

int main(int argc, char *argv[]) {
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<ControllerPosePublisher>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("controller_pose_publisher"), 
                     "Exception: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
