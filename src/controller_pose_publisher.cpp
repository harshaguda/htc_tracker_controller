#include "htc_vive_tracker.h"
#include <openvr.h>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

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

static inline void NormalizeQuaternion(double q[4]) {
    const double n = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (std::isfinite(n) && n > 1e-12) {
        q[0] /= n;
        q[1] /= n;
        q[2] /= n;
        q[3] /= n;
    } else {
        q[0] = 1.0;
        q[1] = 0.0;
        q[2] = 0.0;
        q[3] = 0.0;
    }
}

// Scales the rotation represented by q (unit quaternion) by factor s via axis-angle.
// Equivalent to slerp(Identity, q, s). Assumes q is unit-length.
static inline void ScaleQuaternionRotation(double q[4], double s) {
    s = std::clamp(s, 0.0, 1e9);
    // Ensure shortest path
    if (q[0] < 0.0) {
        q[0] = -q[0];
        q[1] = -q[1];
        q[2] = -q[2];
        q[3] = -q[3];
    }

    const double vx = q[1];
    const double vy = q[2];
    const double vz = q[3];
    const double v_norm = std::sqrt(vx*vx + vy*vy + vz*vz);
    if (!(v_norm > 1e-12)) {
        q[0] = 1.0;
        q[1] = 0.0;
        q[2] = 0.0;
        q[3] = 0.0;
        return;
    }

    const double angle = 2.0 * std::atan2(v_norm, q[0]);
    const double scaled_angle = angle * s;
    const double half = 0.5 * scaled_angle;
    const double sin_half = std::sin(half);
    const double ax = vx / v_norm;
    const double ay = vy / v_norm;
    const double az = vz / v_norm;
    q[0] = std::cos(half);
    q[1] = ax * sin_half;
    q[2] = ay * sin_half;
    q[3] = az * sin_half;
}

static inline void ClampQuaternionRotation(double q[4], double max_angle_rad) {
    if (!(max_angle_rad > 0.0) || !std::isfinite(max_angle_rad)) {
        return;
    }
    // Ensure shortest path
    if (q[0] < 0.0) {
        q[0] = -q[0];
        q[1] = -q[1];
        q[2] = -q[2];
        q[3] = -q[3];
    }
    const double vx = q[1];
    const double vy = q[2];
    const double vz = q[3];
    const double v_norm = std::sqrt(vx*vx + vy*vy + vz*vz);
    if (!(v_norm > 1e-12)) {
        return;
    }
    const double angle = 2.0 * std::atan2(v_norm, q[0]);
    if (angle <= max_angle_rad) {
        return;
    }
    const double s = max_angle_rad / angle;
    ScaleQuaternionRotation(q, s);
}

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

    NormalizeQuaternion(delta_pose.quaternion);
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
          delta_duration_(0.1),
          gripper_width_(0.0),
          gripper_closed_threshold_(0.01)
    {
        // Declare and get parameters
        this->declare_parameter("controller_name", "controller_1");
        this->declare_parameter("publish_rate", 10.0);
        this->declare_parameter("verbose", false);
        this->declare_parameter("frame_id", "world");
        this->declare_parameter("delta_duration", 0.1);
        this->declare_parameter("trigger_axis_threshold", 0.8);

        // /delta_pose shaping (helps avoid franka reflexes).
        // Semantics: published delta is an offset from a fixed reference pose captured on trigger press.
        this->declare_parameter("delta_translation_scale", 1.0);
        this->declare_parameter("delta_rotation_scale", 1.0);
        this->declare_parameter("max_delta_translation", 0.25);  // [m]
        this->declare_parameter("max_delta_rotation", 0.7);      // [rad]
        
        controller_name_ = this->get_parameter("controller_name").as_string();
        double publish_rate = this->get_parameter("publish_rate").as_double();
        bool verbose = this->get_parameter("verbose").as_bool();
        frame_id_ = this->get_parameter("frame_id").as_string();
        delta_duration_ = this->get_parameter("delta_duration").as_double();
        trigger_axis_threshold_ = this->get_parameter("trigger_axis_threshold").as_double();

        delta_translation_scale_ = this->get_parameter("delta_translation_scale").as_double();
        delta_rotation_scale_ = this->get_parameter("delta_rotation_scale").as_double();
        max_delta_translation_ = this->get_parameter("max_delta_translation").as_double();
        max_delta_rotation_ = this->get_parameter("max_delta_rotation").as_double();
        
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
            "/NS_1/franka_gripper/joint_states", 10, // dirty fix
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
    static std::optional<int> parse_controller_ordinal(const std::string& controller_name) {
        static const std::string kPrefix = "controller_";
        if (controller_name.rfind(kPrefix, 0) != 0) {
            return std::nullopt;
        }
        try {
            int ordinal = std::stoi(controller_name.substr(kPrefix.size()));
            if (ordinal <= 0) {
                return std::nullopt;
            }
            return ordinal;
        } catch (...) {
            return std::nullopt;
        }
    }

    static std::optional<vr::TrackedDeviceIndex_t> resolve_controller_index(const std::string& controller_name) {
        if (!vr::VRSystem()) {
            return std::nullopt;
        }

        const auto ordinal_opt = parse_controller_ordinal(controller_name);
        if (!ordinal_opt) {
            return std::nullopt;
        }
        const int ordinal = *ordinal_opt;

        std::vector<vr::TrackedDeviceIndex_t> controller_indices;
        controller_indices.reserve(4);

        for (vr::TrackedDeviceIndex_t i = 0; i < vr::k_unMaxTrackedDeviceCount; ++i) {
            if (!vr::VRSystem()->IsTrackedDeviceConnected(i)) {
                continue;
            }
            if (vr::VRSystem()->GetTrackedDeviceClass(i) != vr::TrackedDeviceClass_Controller) {
                continue;
            }
            controller_indices.push_back(i);
        }

        if (ordinal > static_cast<int>(controller_indices.size())) {
            return std::nullopt;
        }
        return controller_indices.at(static_cast<size_t>(ordinal - 1));
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // Find gripper finger joint and extract width
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i].find("finger") != std::string::npos) {
                // Gripper width is 2x the finger joint position
                gripper_width_ = msg->position[i] * 2.0;
                RCLCPP_INFO(this->get_logger(), "Gripper width: %.4f m", gripper_width_);
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

        // Resolve (and cache) OpenVR tracked-device index for this controller.
        // Do NOT assume any ordering relationship between wrapper "device names" and OpenVR indices.
        if (controller_index_ == vr::k_unTrackedDeviceIndexInvalid || (button_check_counter_ % 200 == 0)) {
            const auto idx_opt = resolve_controller_index(controller_name_);
            controller_index_ = idx_opt.value_or(vr::k_unTrackedDeviceIndexInvalid);
        }
        
        // Check if trigger and grip buttons are currently pressed
        if (controller_index_ != vr::k_unTrackedDeviceIndexInvalid) {
            vr::VRControllerState_t controller_state;
            if (vr::VRSystem()->GetControllerState(controller_index_, &controller_state, sizeof(controller_state))) {
                const uint64_t trigger_button_mask = vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Trigger);
                const bool trigger_button_pressed =
                    (controller_state.ulButtonPressed & trigger_button_mask) != 0;

                // Many Vive controllers report trigger as an analog axis (commonly Axis1.x in [0,1]).
                const double trigger_axis_value = static_cast<double>(controller_state.rAxis[1].x);
                const double threshold = std::clamp(trigger_axis_threshold_, 0.0, 1.0);
                const bool trigger_axis_pressed = trigger_axis_value >= threshold;

                trigger_currently_pressed = trigger_button_pressed || trigger_axis_pressed;

                const uint64_t grip_mask = vr::ButtonMaskFromId(vr::k_EButton_Grip);
                grip_currently_pressed = (controller_state.ulButtonPressed & grip_mask) != 0;
                
                // Log button states periodically
                if (button_check_counter_ % 500 == 0) {
                    RCLCPP_INFO(this->get_logger(), 
                               "Controller[%u] Trigger: %s (button=%s axis=%.2f thr=%.2f) Grip: %s ButtonPressed: 0x%lx",
                               controller_index_,
                               trigger_currently_pressed ? "PRESSED" : "released",
                               trigger_button_pressed ? "PRESSED" : "released",
                               trigger_axis_value,
                               threshold,
                               grip_currently_pressed ? "PRESSED" : "released",
                               (unsigned long)controller_state.ulButtonPressed);
                }
            } else {
                if (button_check_counter_ % 500 == 0) {
                    RCLCPP_WARN(this->get_logger(), "Failed to get controller state for index %u", controller_index_);
                }
            }
        } else {
            if (button_check_counter_ % 500 == 0) {
                RCLCPP_WARN(this->get_logger(),
                            "Controller index is invalid for '%s' (expected 'controller_#').",
                            controller_name_.c_str());
            }
        }
        button_check_counter_++;
        
        // Detect trigger press/release edges
        if (trigger_currently_pressed && !trigger_pressed_) {
            trigger_pressed_ = true;
            trigger_start_pose_ = current_pose;
            trigger_start_pose_captured_ = true;
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
            bool is_gripper_closed = (gripper_width_ > gripper_closed_threshold_);
            
            auto gripper_msg = std_msgs::msg::Bool();
            gripper_msg.data = is_gripper_closed;  // true if closed, false if open
            gripper_command_pub_->publish(gripper_msg);
            
            RCLCPP_INFO(this->get_logger(), "Grip button pressed - Gripper state: %s (width: %.4f m)",
                       is_gripper_closed ? "CLOSED" : "OPEN", gripper_width_);
        } else if (!grip_currently_pressed && grip_pressed_) {
            grip_pressed_ = false;
        }
        
        // Delta pose tracking with configurable duration intervals.
        // Publish delta relative to a fixed reference captured on trigger press.
        if (trigger_pressed_) {
            auto time_since_last_delta = (this->now() - last_delta_publish_time_).seconds();
            
            // Log delta timing info
            // static int delta_counter = 0;
            // if (delta_counter % 10 == 0) {
            //     RCLCPP_INFO(this->get_logger(), 
            //                "Delta tracking: time_since_last=%.3f, delta_duration=%.3f, captured=%s",
            //                time_since_last_delta, delta_duration_, 
            //                trigger_start_pose_captured_ ? "YES" : "NO");
            // }
            // delta_counter++;
            
            if (trigger_start_pose_captured_ && time_since_last_delta >= delta_duration_) {
                Pose6D delta_pose;
                ComputeDeltaPose(current_pose, trigger_start_pose_, delta_pose);

                // Apply user scaling.
                const double t_scale = std::max(0.0, delta_translation_scale_);
                delta_pose.position[0] *= t_scale;
                delta_pose.position[1] *= t_scale;
                delta_pose.position[2] *= t_scale;

                // Clamp translation magnitude.
                const double max_t = max_delta_translation_;
                double delta_magnitude = std::sqrt(delta_pose.position[0]*delta_pose.position[0] +
                                                   delta_pose.position[1]*delta_pose.position[1] +
                                                   delta_pose.position[2]*delta_pose.position[2]);
                if (std::isfinite(max_t) && max_t > 0.0 && delta_magnitude > max_t) {
                    const double s = max_t / delta_magnitude;
                    delta_pose.position[0] *= s;
                    delta_pose.position[1] *= s;
                    delta_pose.position[2] *= s;
                    delta_magnitude = max_t;
                }

                // Scale + clamp rotation (as angle scaling, i.e., slerp from identity).
                const double r_scale = std::max(0.0, delta_rotation_scale_);
                if (r_scale != 1.0) {
                    // Implement scaling as angle scaling: q <- q^(r_scale)
                    // Our helper takes a factor; allow >1 by reconstructing via axis-angle.
                    // Do it by scaling and then clamping.
                    // Convert to axis-angle inside helper by clamping with a large max.
                    // Here: ScaleQuaternionRotation expects factor on angle; emulate by rebuilding.
                    // Use ClampQuaternionRotation after rebuilding.
                    // Rebuild via axis-angle scaling:
                    // (handled by ScaleQuaternionRotation for any s; it doesn't clamp s)
                    ScaleQuaternionRotation(delta_pose.quaternion, r_scale);
                }
                ClampQuaternionRotation(delta_pose.quaternion, max_delta_rotation_);
                NormalizeQuaternion(delta_pose.quaternion);

                auto delta_msg = geometry_msgs::msg::PoseStamped();
                delta_msg.header.stamp = now;
                delta_msg.header.frame_id = frame_id_;
                delta_msg.pose.position.x = delta_pose.position[0];
                delta_msg.pose.position.y = delta_pose.position[1];
                delta_msg.pose.position.z = delta_pose.position[2];
                delta_msg.pose.orientation.x = delta_pose.quaternion[1];
                delta_msg.pose.orientation.y = delta_pose.quaternion[2];
                delta_msg.pose.orientation.z = delta_pose.quaternion[3];
                delta_msg.pose.orientation.w = delta_pose.quaternion[0];
                delta_pose_pub_->publish(delta_msg);

                RCLCPP_DEBUG(this->get_logger(),
                             "Delta: |Δp|=%.4f m, q=[%.3f %.3f %.3f %.3f]",
                             delta_magnitude,
                             delta_pose.quaternion[0], delta_pose.quaternion[1],
                             delta_pose.quaternion[2], delta_pose.quaternion[3]);

                last_delta_publish_time_ = this->now();
            }
        }
        
        // Create and publish PoseStamped message
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header.stamp = now;
        pose_msg.header.frame_id = frame_id_;
        // pose_msg.pose.position.x = position[0];
        // pose_msg.pose.position.y = position[1];
        // pose_msg.pose.position.z = position[2];
        pose_msg.pose.position.x = 0.0;
        pose_msg.pose.position.y = 0.0;
        pose_msg.pose.position.z = 0.0;
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
        // static int counter = 0;
        // if (counter % 200 == 0) {
        //     double distance = sqrt(position[0]*position[0] +
        //                           position[1]*position[1] +
        //     RCLCPP_INFO(this->get_logger(), 
        //                "Controller Pose: Pos=[%.3f, %.3f, %.3f]m (dist=%.3f), Ori=[w=%.3f, x=%.3f, y=%.3f, z=%.3f]",
        //                position[0], position[1], position[2], distance,
        //                quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
        // }
        // counter++;
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

    // /delta_pose shaping
    double delta_translation_scale_{1.0};
    double delta_rotation_scale_{1.0};
    double max_delta_translation_{0.25};
    double max_delta_rotation_{0.7};
    
    // Gripper state tracking
    double gripper_width_;
    double gripper_closed_threshold_;

    // OpenVR button polling
    vr::TrackedDeviceIndex_t controller_index_{vr::k_unTrackedDeviceIndexInvalid};
    uint64_t button_check_counter_{0};
    double trigger_axis_threshold_{0.8};
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
