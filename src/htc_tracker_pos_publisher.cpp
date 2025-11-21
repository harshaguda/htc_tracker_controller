#include "htc_vive_tracker.h"
#include <openvr.h>
#include <cmath>
#include <iomanip>
#include <unistd.h>

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

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

// Function to extract position from transformation matrix
void ExtractPosition(const Transform& transform, double position[3]) {
    position[0] = transform.matrix[0][3];
    position[1] = transform.matrix[1][3];
    position[2] = transform.matrix[2][3];
}

// Function to compute relative position of controller with respect to tracker
void ComputeRelativePosition(CHtc_Vive_Tracker& vt, 
                              const std::string& controller_name,
                              const std::string& tracker_name,
                              double relative_position[3]) {
    double controller_pose[3], controller_quat[4];
    double tracker_pose[3], tracker_quat[4];
    
    // Get absolute poses
    bool controller_success = vt.GetDevicePoseQuaternion(controller_name, controller_pose, controller_quat);
    bool tracker_success = vt.GetDevicePoseQuaternion(tracker_name, tracker_pose, tracker_quat);
    
    if (!controller_success || !tracker_success) {
        RCLCPP_WARN(rclcpp::get_logger("controller_relative_publisher"), 
                    "Could not get poses for both devices");
        relative_position[0] = relative_position[1] = relative_position[2] = 0.0;
        return;
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
    
    // Extract position
    ExtractPosition(T_tracker_controller, relative_position);
}

// Function to compute distance between controller and tracker
double ComputeDistance(const double relative_position[3]) {
    return sqrt(relative_position[0] * relative_position[0] +
                relative_position[1] * relative_position[1] +
                relative_position[2] * relative_position[2]);
}

class ControllerRelativePublisher : public rclcpp::Node {
public:
    ControllerRelativePublisher() 
        : Node("controller_relative_publisher"), 
          vt_(),
          controller_name_("controller_1"),
          tracker_name_("tracker_1")
    {
        // Declare and get parameters
        this->declare_parameter("controller_name", "controller_1");
        this->declare_parameter("tracker_name", "tracker_1");
        this->declare_parameter("publish_rate", 10.0);
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
        position_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "controller_relative_position", 10);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "controller_relative_pose", 10);
        
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
            std::bind(&ControllerRelativePublisher::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Publishing controller position relative to tracker at %.1f Hz", publish_rate);
        RCLCPP_INFO(this->get_logger(), "Topics: /controller_relative_position and /controller_relative_pose");
    }

private:
    void timer_callback() {
        // Update device poses
        vt_.Update();
        
        // // Poll for button events
        while (vt_.EventPolling()) {
            // Check if the controller has a button press
            std::string button_name = vt_.GetLastButtonPressedString(controller_name_);
            // std::string unbutton_name = vt_.GetLastButtonUnpressedString(controller_name_);
            vr::EVRButtonId button_id = vt_.GetLastButtonPressedEnum(controller_name_);
            
            if (!button_name.empty()) {
                RCLCPP_INFO(this->get_logger(), " PRESSED on %s!", button_name.c_str());
                // RCLCPP_INFO(this->get_logger(), "UNPRESSED on %s!", unbutton_name.c_str());

            }
            // Check if trigger button was pressed (k_EButton_SteamVR_Trigger = 33)
            // if (button_id == vr::k_EButton_SteamVR_Trigger) {
            //     RCLCPP_INFO(this->get_logger(), "🎮 PRESSED on %s!", button_name.c_str());
                
            //     // Optional: trigger haptic feedback
            //     // vt_.HapticPulse(controller_name_, 0, 1000);
            // }
            
            // Print all button presses for debugging
            // if (button_id != vr::k_EButton_Max) {
            //     RCLCPP_INFO(this->get_logger(), "Button pressed: %s (ID: %d)", 
            //                button_name.c_str(), static_cast<int>(button_id));
            // }
        }
        
        // Compute relative position
        double relative_position[3];
        ComputeRelativePosition(vt_, controller_name_, tracker_name_, relative_position);
        
        // Compute distance
        double distance = ComputeDistance(relative_position);
        
        // Create and publish PointStamped message
        auto point_msg = geometry_msgs::msg::PointStamped();
        point_msg.header.stamp = this->now();
        point_msg.header.frame_id = tracker_name_;
        point_msg.point.x = relative_position[0];
        point_msg.point.y = relative_position[1];
        point_msg.point.z = relative_position[2];
        position_pub_->publish(point_msg);
        
        // Create and publish PoseStamped message (position only, orientation set to identity)
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = tracker_name_;
        pose_msg.pose.position.x = relative_position[0];
        pose_msg.pose.position.y = relative_position[1];
        pose_msg.pose.position.z = relative_position[2];
        pose_msg.pose.orientation.w = 1.0;
        pose_msg.pose.orientation.x = 0.0;
        pose_msg.pose.orientation.y = 0.0;
        pose_msg.pose.orientation.z = 0.0;
        pose_pub_->publish(pose_msg);
        
        // Log info periodically (every 2 seconds)
        static int counter = 0;
        if (counter % 20 == 0) {
            RCLCPP_INFO(this->get_logger(), 
                        "Publishing: X=%.3f, Y=%.3f, Z=%.3f, Distance=%.3f m",
                        relative_position[0], relative_position[1], 
                        relative_position[2], distance);
        }
        counter++;
    }
    
    CHtc_Vive_Tracker vt_;
    std::string controller_name_;
    std::string tracker_name_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr position_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

int main(int argc, char *argv[]) {
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<ControllerRelativePublisher>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("controller_relative_publisher"), 
                     "Exception: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
