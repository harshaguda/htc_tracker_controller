#include "htc_vive_tracker.h"
#include <openvr.h>
#include <cmath>
#include <iomanip>
#include <unistd.h>

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// Structure to hold 6D pose (position + orientation)
struct Pose6D {
    double position[3];      // x, y, z
    double quaternion[4];    // w, x, y, z
};

// Structure to hold transformation matrix
struct Transform {
    double matrix[3][4];  // 3x4 transformation matrix
};

// Function to convert pose and quaternion to transformation matrix
void PoseQuatToMatrix(const double pose[3], const double quat[4], Transform& transform) {
    // quat = [w, x, y, z]
    double w = quat[0];
    double x = quat[1];
    double y = quat[2];
    double z = quat[3];
    
    // Convert quaternion to rotation matrix
    transform.matrix[0][0] = 1 - 2*y*y - 2*z*z;
    transform.matrix[0][1] = 2*x*y - 2*w*z;
    transform.matrix[0][2] = 2*x*z + 2*w*y;
    transform.matrix[0][3] = pose[0];
    
    transform.matrix[1][0] = 2*x*y + 2*w*z;
    transform.matrix[1][1] = 1 - 2*x*x - 2*z*z;
    transform.matrix[1][2] = 2*y*z - 2*w*x;
    transform.matrix[1][3] = pose[1];
    
    transform.matrix[2][0] = 2*x*z - 2*w*y;
    transform.matrix[2][1] = 2*y*z + 2*w*x;
    transform.matrix[2][2] = 1 - 2*x*x - 2*y*y;
    transform.matrix[2][3] = pose[2];
}

// Function to invert a transformation matrix
void InvertTransform(const Transform& input, Transform& output) {
    // For a transformation matrix T = [R | t], the inverse is [R^T | -R^T * t]
    // Since R is a rotation matrix, R^T = R^-1
    
    // Transpose the rotation part
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            output.matrix[i][j] = input.matrix[j][i];
        }
    }
    
    // Compute -R^T * t
    for (int i = 0; i < 3; i++) {
        output.matrix[i][3] = 0;
        for (int j = 0; j < 3; j++) {
            output.matrix[i][3] -= output.matrix[i][j] * input.matrix[j][3];
        }
    }
}

// Function to multiply two transformation matrices
void MultiplyTransforms(const Transform& t1, const Transform& t2, Transform& result) {
    // result = t1 * t2
    
    // Multiply rotation parts (3x3 * 3x3)
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result.matrix[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                result.matrix[i][j] += t1.matrix[i][k] * t2.matrix[k][j];
            }
        }
    }
    
    // Compute translation: R1 * t2 + t1
    for (int i = 0; i < 3; i++) {
        result.matrix[i][3] = t1.matrix[i][3];
        for (int j = 0; j < 3; j++) {
            result.matrix[i][3] += t1.matrix[i][j] * t2.matrix[j][3];
        }
    }
}

// Function to extract pose (position + quaternion) from transformation matrix
void TransformToPose6D(const Transform& transform, Pose6D& pose) {
    // Extract position
    pose.position[0] = transform.matrix[0][3];
    pose.position[1] = transform.matrix[1][3];
    pose.position[2] = transform.matrix[2][3];
    
    // Extract quaternion from rotation matrix
    // Using method from: https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
    double trace = transform.matrix[0][0] + transform.matrix[1][1] + transform.matrix[2][2];
    
    if (trace > 0) {
        double s = 0.5 / sqrt(trace + 1.0);
        pose.quaternion[0] = 0.25 / s;  // w
        pose.quaternion[1] = (transform.matrix[2][1] - transform.matrix[1][2]) * s;  // x
        pose.quaternion[2] = (transform.matrix[0][2] - transform.matrix[2][0]) * s;  // y
        pose.quaternion[3] = (transform.matrix[1][0] - transform.matrix[0][1]) * s;  // z
    } else if (transform.matrix[0][0] > transform.matrix[1][1] && transform.matrix[0][0] > transform.matrix[2][2]) {
        double s = 2.0 * sqrt(1.0 + transform.matrix[0][0] - transform.matrix[1][1] - transform.matrix[2][2]);
        pose.quaternion[0] = (transform.matrix[2][1] - transform.matrix[1][2]) / s;  // w
        pose.quaternion[1] = 0.25 * s;  // x
        pose.quaternion[2] = (transform.matrix[0][1] + transform.matrix[1][0]) / s;  // y
        pose.quaternion[3] = (transform.matrix[0][2] + transform.matrix[2][0]) / s;  // z
    } else if (transform.matrix[1][1] > transform.matrix[2][2]) {
        double s = 2.0 * sqrt(1.0 + transform.matrix[1][1] - transform.matrix[0][0] - transform.matrix[2][2]);
        pose.quaternion[0] = (transform.matrix[0][2] - transform.matrix[2][0]) / s;  // w
        pose.quaternion[1] = (transform.matrix[0][1] + transform.matrix[1][0]) / s;  // x
        pose.quaternion[2] = 0.25 * s;  // y
        pose.quaternion[3] = (transform.matrix[1][2] + transform.matrix[2][1]) / s;  // z
    } else {
        double s = 2.0 * sqrt(1.0 + transform.matrix[2][2] - transform.matrix[0][0] - transform.matrix[1][1]);
        pose.quaternion[0] = (transform.matrix[1][0] - transform.matrix[0][1]) / s;  // w
        pose.quaternion[1] = (transform.matrix[0][2] + transform.matrix[2][0]) / s;  // x
        pose.quaternion[2] = (transform.matrix[1][2] + transform.matrix[2][1]) / s;  // y
        pose.quaternion[3] = 0.25 * s;  // z
    }
}

// Function to compute relative 6D pose of controller with respect to tracker
bool ComputeRelativePose6D(CHtc_Vive_Tracker& vt, 
                           const std::string& controller_name,
                           const std::string& tracker_name,
                           Pose6D& relative_pose) {
    double controller_pose[3], controller_quat[4];
    double tracker_pose[3], tracker_quat[4];
    
    // Get absolute poses
    bool controller_success = vt.GetDevicePoseQuaternion(controller_name, controller_pose, controller_quat);
    bool tracker_success = vt.GetDevicePoseQuaternion(tracker_name, tracker_pose, tracker_quat);
    
    if (!controller_success || !tracker_success) {
        return false;
    }
    
    // Convert to transformation matrices
    Transform T_world_controller, T_world_tracker;
    PoseQuatToMatrix(controller_pose, controller_quat, T_world_controller);
    PoseQuatToMatrix(tracker_pose, tracker_quat, T_world_tracker);
    
    // Compute inverse of tracker transform
    Transform T_tracker_world;
    InvertTransform(T_world_tracker, T_tracker_world);
    
    // Compute relative transform: T_tracker_controller = T_tracker_world * T_world_controller
    Transform T_tracker_controller;
    MultiplyTransforms(T_tracker_world, T_world_controller, T_tracker_controller);
    
    // Extract 6D pose from transformation matrix
    TransformToPose6D(T_tracker_controller, relative_pose);
    
    return true;
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
}

class Controller6DPosePublisher : public rclcpp::Node {
public:
    Controller6DPosePublisher() 
        : Node("controller_6d_pose_publisher"), 
          vt_(),
          controller_name_("controller_1"),
          tracker_name_("tracker_1"),
          trigger_pressed_(false),
          has_previous_pose_(false)
    {
        // Declare and get parameters
        this->declare_parameter("controller_name", "controller_1");
        this->declare_parameter("tracker_name", "tracker_1");
        this->declare_parameter("publish_rate", 100.0);
        this->declare_parameter("verbose", false);
        
        controller_name_ = this->get_parameter("controller_name").as_string();
        tracker_name_ = this->get_parameter("tracker_name").as_string();
        double publish_rate = this->get_parameter("publish_rate").as_double();
        bool verbose = this->get_parameter("verbose").as_bool();
        
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
        
        // Check if both devices are detected
        if (!vt_.IsDeviceDetected(controller_name_)) {
            RCLCPP_ERROR(this->get_logger(), "%s not detected!", controller_name_.c_str());
            throw std::runtime_error("Controller not detected");
        }
        
        if (!vt_.IsDeviceDetected(tracker_name_)) {
            RCLCPP_ERROR(this->get_logger(), "%s not detected!", tracker_name_.c_str());
            throw std::runtime_error("Tracker not detected");
        }
        
        RCLCPP_INFO(this->get_logger(), "Both %s and %s detected!", 
                    controller_name_.c_str(), tracker_name_.c_str());
        
        // Create publishers
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "controller_relative_pose_6d", 10);
        
        delta_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "controller_delta_pose", 10);
        
        // Create static transform broadcaster
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        
        // Publish static transform from world to tracker frame
        geometry_msgs::msg::TransformStamped static_transform;
        static_transform.header.stamp = this->now();
        static_transform.header.frame_id = "world";
        static_transform.child_frame_id = tracker_name_;
        static_transform.transform.translation.x = 0.0;
        static_transform.transform.translation.y = 0.0;
        static_transform.transform.translation.z = 0.0;
        static_transform.transform.rotation.w = 1.0;
        static_transform.transform.rotation.x = 0.0;
        static_transform.transform.rotation.y = 0.0;
        static_transform.transform.rotation.z = 0.0;
        tf_static_broadcaster_->sendTransform(static_transform);
        
        RCLCPP_INFO(this->get_logger(), "Published static transform: world -> %s", tracker_name_.c_str());
        
        // Create timer for publishing
        auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate));
        timer_ = this->create_wall_timer(
            timer_period,
            std::bind(&Controller6DPosePublisher::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Publishing controller 6D pose relative to tracker at %.1f Hz", publish_rate);
        RCLCPP_INFO(this->get_logger(), "Topics:");
        RCLCPP_INFO(this->get_logger(), "  - /controller_relative_pose_6d (continuous)");
        RCLCPP_INFO(this->get_logger(), "  - /controller_delta_pose (published when trigger is pressed)");
    }

private:
    void timer_callback() {
        // Update device poses
        vt_.Update();
        
        // Poll for button events (drain the event queue)
        while (vt_.EventPolling()) {
            // Just drain the event queue
        }
        
        // Check current trigger button state using OpenVR API
        bool trigger_currently_pressed = false;
        
        // Get the controller device index
        auto device_names = vt_.GetAllDeviceNames();
        vr::TrackedDeviceIndex_t controller_index = vr::k_unTrackedDeviceIndexInvalid;
        
        for (vr::TrackedDeviceIndex_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
            if (i < device_names.size() && device_names[i] == controller_name_) {
                controller_index = i;
                break;
            }
        }
        
        // Check if trigger button is currently pressed
        if (controller_index != vr::k_unTrackedDeviceIndexInvalid) {
            vr::VRControllerState_t controller_state;
            if (vr::VRSystem()->GetControllerState(controller_index, &controller_state, sizeof(controller_state))) {
                // Check if trigger button is pressed (bit 33 = k_EButton_SteamVR_Trigger)
                uint64_t trigger_mask = vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Trigger);
                trigger_currently_pressed = (controller_state.ulButtonPressed & trigger_mask) != 0;
            }
        }
        
        // Detect trigger press/release edges
        if (trigger_currently_pressed && !trigger_pressed_) {
            trigger_pressed_ = true;
            RCLCPP_INFO(this->get_logger(), "🎮 Trigger pressed - Starting delta pose tracking");
        } else if (!trigger_currently_pressed && trigger_pressed_) {
            trigger_pressed_ = false;
            RCLCPP_INFO(this->get_logger(), "🎮 Trigger released - Stopped delta pose tracking");
        }
        
        // Compute current relative 6D pose
        Pose6D current_pose;
        if (!ComputeRelativePose6D(vt_, controller_name_, tracker_name_, current_pose)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Could not get poses for both devices");
            return;
        }
        
        // Create and publish current pose
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = tracker_name_;
        pose_msg.pose.position.x = current_pose.position[0];
        pose_msg.pose.position.y = current_pose.position[1];
        pose_msg.pose.position.z = current_pose.position[2];
        pose_msg.pose.orientation.w = current_pose.quaternion[0];
        pose_msg.pose.orientation.x = current_pose.quaternion[1];
        pose_msg.pose.orientation.y = current_pose.quaternion[2];
        pose_msg.pose.orientation.z = current_pose.quaternion[3];
        pose_pub_->publish(pose_msg);
        
        // If trigger is pressed and we have a previous pose, compute and publish delta
        if (trigger_pressed_ && has_previous_pose_) {
            Pose6D delta_pose;
            ComputeDeltaPose(current_pose, previous_pose_, delta_pose);
            
            auto delta_msg = geometry_msgs::msg::PoseStamped();
            delta_msg.header.stamp = this->now();
            delta_msg.header.frame_id = tracker_name_;
            delta_msg.pose.position.x = delta_pose.position[0];
            delta_msg.pose.position.y = delta_pose.position[1];
            delta_msg.pose.position.z = delta_pose.position[2];
            delta_msg.pose.orientation.w = delta_pose.quaternion[0];
            delta_msg.pose.orientation.x = delta_pose.quaternion[1];
            delta_msg.pose.orientation.y = delta_pose.quaternion[2];
            delta_msg.pose.orientation.z = delta_pose.quaternion[3];
            delta_pose_pub_->publish(delta_msg);
            
            // Log delta information periodically
            static int delta_counter = 0;
            if (delta_counter % 50 == 0) {
                double pos_delta = sqrt(delta_pose.position[0]*delta_pose.position[0] +
                                       delta_pose.position[1]*delta_pose.position[1] +
                                       delta_pose.position[2]*delta_pose.position[2]);
                RCLCPP_INFO(this->get_logger(), 
                           "Delta: ΔPos=%.4f m, ΔOri=[%.3f, %.3f, %.3f, %.3f]",
                           pos_delta,
                           delta_pose.quaternion[0], delta_pose.quaternion[1],
                           delta_pose.quaternion[2], delta_pose.quaternion[3]);
            }
            delta_counter++;
        }
        
        // Update previous pose
        previous_pose_ = current_pose;
        has_previous_pose_ = true;
        
        // Log current pose periodically (every 2 seconds)
        static int counter = 0;
        if (counter % 200 == 0) {
            double distance = sqrt(current_pose.position[0]*current_pose.position[0] +
                                  current_pose.position[1]*current_pose.position[1] +
                                  current_pose.position[2]*current_pose.position[2]);
            RCLCPP_INFO(this->get_logger(), 
                       "Pose: Pos=[%.3f, %.3f, %.3f]m (dist=%.3f), Ori=[%.3f, %.3f, %.3f, %.3f]",
                       current_pose.position[0], current_pose.position[1], current_pose.position[2],
                       distance,
                       current_pose.quaternion[0], current_pose.quaternion[1],
                       current_pose.quaternion[2], current_pose.quaternion[3]);
        }
        counter++;
    }
    
    CHtc_Vive_Tracker vt_;
    std::string controller_name_;
    std::string tracker_name_;
    
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr delta_pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    
    bool trigger_pressed_;
    bool has_previous_pose_;
    Pose6D previous_pose_;
};

int main(int argc, char *argv[]) {
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<Controller6DPosePublisher>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("controller_6d_pose_publisher"), 
                     "Exception: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
