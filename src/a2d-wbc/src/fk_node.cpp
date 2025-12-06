#include "wbc/fk_node.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <genie_msgs/msg/waist_state.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace wbc {

FKWrapperNode::FKWrapperNode() 
    : Node("fk_wrapper_node")
    , has_waist_data_(false)
    , has_arm_data_(false)
{
    RCLCPP_INFO(this->get_logger(), "Initializing FK Wrapper Node...");
    
    if (!initializeParameters()) 
    {
        RCLCPP_FATAL(this->get_logger(), "error,Failed to initialize parameters");
        return;
    }
     
    if (!initializeController()) 
    {
        RCLCPP_FATAL(this->get_logger(), "error,Failed to initialize controller");
        return;
    }

    initializeROS();
    
    RCLCPP_INFO(this->get_logger(), "Frequency: %.1f Hz", frequency_);
    RCLCPP_INFO(this->get_logger(), "Left EE: %s", left_ee_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "Right EE: %s", right_ee_frame_.c_str());
}

bool FKWrapperNode::initializeParameters() 
{
    this->declare_parameter<std::string>("robot_urdf_path", "a2d-tele/assets/A2D_NoHand/A2D_NoHand_Flattened.urdf");
    this->declare_parameter<std::string>("base_frame", "map");
    this->declare_parameter<std::string>("left_end_effector", "Link7_l");
    this->declare_parameter<std::string>("right_end_effector", "Link7_r");
    this->declare_parameter<double>("frequency", 100.0);
    this->declare_parameter<bool>("publish_tf", true);
    this->declare_parameter<bool>("enable_robot_state_publisher", true);
    
    urdf_path_ = this->get_parameter("robot_urdf_path").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    left_ee_frame_ = this->get_parameter("left_end_effector").as_string();
    right_ee_frame_ = this->get_parameter("right_end_effector").as_string();
    frequency_ = this->get_parameter("frequency").as_double();
    publish_tf_ = this->get_parameter("publish_tf").as_bool();
    enable_robot_state_publisher_ = this->get_parameter("enable_robot_state_publisher").as_bool();
    
    joint_names_ = {
        "joint_lift_body", "joint_body_pitch",  // Waist: 2 joints
        "joint1_l", "joint2_l", "joint3_l", "joint4_l", "joint5_l", "joint6_l", "joint7_l",  // Left arm: 7 joints
        "joint1_r", "joint2_r", "joint3_r", "joint4_r", "joint5_r", "joint6_r", "joint7_r"   // Right arm: 7 joints
    };
    
    // Initialize joint positions to zero
    current_joint_positions_.resize(16, 0.0);

    RCLCPP_INFO(this->get_logger(), "enable_robot_state_publisher: %d", enable_robot_state_publisher_);
    RCLCPP_INFO(this->get_logger(), "publish_tf: %d", publish_tf_);
    
    return true;
}

bool FKWrapperNode::initializeController() 
{
    try 
    {
        controller_ = std::make_unique<RobotKinematicsTool>();
        
        if (!controller_->robotKinematicsInit(urdf_path_, left_ee_frame_, right_ee_frame_)) 
        {
            RCLCPP_ERROR(this->get_logger(), "error,Failed to initialize controller");
            return false;
        }
        
        // Configure joint limits (same as IK node for consistency)
        std::vector<double> lower_limits = {
            0.399, 0.435,  // Waist
            -3.14, -1.92, -3.14, -1.75, -3.14, -1.7, -3.14,  // Left arm
            -3.14, -1.92, -3.14, -1.75, -3.14, -1.7, -3.14   // Right arm
        };
        
        std::vector<double> upper_limits = {
            0.401, 0.437,  // Waist
            3.14, 1.81, 3.14, 1.45, 3.14, 1.7, 3.14,   // Left arm
            3.14, 1.81, 3.14, 1.45, 3.14, 1.7, 3.14    // Right arm
        };
        
        if (!controller_->setJointPositionLimits(lower_limits, upper_limits))
        {
            RCLCPP_ERROR(this->get_logger(), "error, Failed to set joint limits");
            return false;
        }
        
        // RobotKinematicsTool has fixed joint names, no need to set them
        // Verify joint names match
        auto tool_joint_names = controller_->getJointNames();
        if (tool_joint_names.size() != joint_names_.size()) 
        {
            RCLCPP_ERROR(this->get_logger(), "error, Joint names size mismatch: expected %zu, got %zu", 
                        joint_names_.size(), tool_joint_names.size());
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "Controller initialized (FK mode) with %zu joints", tool_joint_names.size());
        return true;
        
    } catch (const std::exception& e) 
    {
        RCLCPP_ERROR(this->get_logger(), "error, Controller initialization error: %s", e.what());
        return false;
    }
}

void FKWrapperNode::initializeROS() 
{
      
    waist_sub_ = this->create_subscription<genie_msgs::msg::WaistState>(
        "/hal/waist_state", 10,
        std::bind(&FKWrapperNode::waistStateCallback, this, std::placeholders::_1));
        
    arm_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/hal/arm_joint_state", 10,
        std::bind(&FKWrapperNode::armJointStateCallback, this, std::placeholders::_1));

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/fk/joint_states", 100);
    
    left_tcp_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/mink_fk/left_tcp_pose", 10);
            
    right_tcp_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/mink_fk/right_tcp_pose", 10);
    
   
    if (publish_tf_) 
    {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    }
    
    auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / frequency_));
    timer_ = this->create_wall_timer(period, std::bind(&FKWrapperNode::timerCallback, this));
    
    RCLCPP_INFO(this->get_logger(), "📡 ROS interfaces initialized");
}

void FKWrapperNode::waistStateCallback(const genie_msgs::msg::WaistState::SharedPtr msg) 
{ 
    std::lock_guard<std::mutex> lock(data_mutex_);

    // motor_states 数组中：[0] = joint_body_pitch, [1] = joint_lift_body
    if (msg->motor_states.size() >= 2) {
        // 根据 name 数组确定关节顺序，或直接使用固定顺序
        // 注意：需要按照 joint_names_ 的顺序：[joint_lift_body, joint_body_pitch]
        int lift_idx = -1, pitch_idx = -1;
        for (size_t i = 0; i < msg->name.size(); ++i) 
        {
            if (msg->name[i] == "joint_lift_body") 
            {
                lift_idx = i;
            } else if (msg->name[i] == "joint_body_pitch") 
            {
                pitch_idx = i;
            }
        }
        
        if (lift_idx >= 0 && pitch_idx >= 0) 
        {
            current_joint_positions_[0] = msg->motor_states[lift_idx].position;  // joint_lift_body
            current_joint_positions_[1] = msg->motor_states[pitch_idx].position;  // joint_body_pitch
            has_waist_data_ = true;
            
            RCLCPP_DEBUG(this->get_logger(), "Received waist joints: lift=%.3f, pitch=%.3f", current_joint_positions_[0], current_joint_positions_[1]);
        }
        else 
        {
            RCLCPP_WARN(this->get_logger(), "error, Could not find required waist joints in message");
            has_waist_data_ = false;  // 确保状态正确
        }
    } 
    else 
    {
        RCLCPP_WARN(this->get_logger(), "error, WaistState has insufficient motor_states: %zu", msg->motor_states.size());
        has_waist_data_ = false;  // 确保状态正确
    }
}

void FKWrapperNode::armJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) 
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    // Update arm joints (indices 2-15)
    // Assuming msg order: [joint1_l, ..., joint7_l, joint1_r, ..., joint7_r]
    const size_t expected_arm_joints = 14;
    if (msg->position.size() >= expected_arm_joints) 
    {
        for (size_t i = 0; i < expected_arm_joints; ++i) 
        {
            current_joint_positions_[i + 2] = msg->position[i];
        }
        has_arm_data_ = true;
    }
    else 
    {
        RCLCPP_WARN(this->get_logger(), "error, Arm joint state has insufficient positions: %zu (expected %zu)", 
                   msg->position.size(), expected_arm_joints);
        has_arm_data_ = false;
    }
}

void FKWrapperNode::timerCallback() 
{
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!has_waist_data_ || !has_arm_data_)
         {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,"⏳ Waiting for joint data (waist: %s, arm: %s)",
                has_waist_data_ ? "✓" : "✗",
                has_arm_data_ ? "✓" : "✗");
            return;
        }
    }

    computeAndPublishFK();
}

void FKWrapperNode::computeAndPublishFK() 
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    geometry_msgs::msg::Pose left_pose;
    geometry_msgs::msg::Pose right_pose;
    double solve_time = 0.0;
    
    if (!controller_->fkSolve(current_joint_positions_, left_pose, right_pose, solve_time))
    {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "error, FK computation failed (solve time: %.2f ms)", solve_time);
        return;
    }
    
    RCLCPP_DEBUG(this->get_logger(), "FK solve time: %.2f ms", solve_time);
    
    // Publish as PoseStamped
    auto stamp = this->now();
    
    auto left_msg = poseToPoseStamped(left_pose, base_frame_);
    left_msg.header.stamp = stamp;
    left_tcp_pub_->publish(left_msg);
    
    auto right_msg = poseToPoseStamped(right_pose, base_frame_);
    right_msg.header.stamp = stamp;
    right_tcp_pub_->publish(right_msg);
    
    // Publish TF if enabled
    if (publish_tf_&&enable_robot_state_publisher_) 
    {
        // Left TCP TF (with fk/ prefix to avoid conflicts with IK)
        geometry_msgs::msg::TransformStamped left_tf;
        left_tf.header = left_msg.header;
        left_tf.child_frame_id = "fk/left_tcp";
        left_tf.transform.translation.x = left_pose.position.x;
        left_tf.transform.translation.y = left_pose.position.y;
        left_tf.transform.translation.z = left_pose.position.z;
        left_tf.transform.rotation = left_pose.orientation;
        tf_broadcaster_->sendTransform(left_tf);
        
        // Right TCP TF (with fk/ prefix to avoid conflicts with IK)
        geometry_msgs::msg::TransformStamped right_tf;
        right_tf.header = right_msg.header;
        right_tf.child_frame_id = "fk/right_tcp";
        right_tf.transform.translation.x = right_pose.position.x;
        right_tf.transform.translation.y = right_pose.position.y;
        right_tf.transform.translation.z = right_pose.position.z;
        right_tf.transform.rotation = right_pose.orientation;
        tf_broadcaster_->sendTransform(right_tf);
    }
    


    if (enable_robot_state_publisher_)
    {
        // Publish joint states for robot_state_publisher 2025-10-27
        sensor_msgs::msg::JointState joint_state_msg;
        joint_state_msg.header.stamp = stamp;
        joint_state_msg.header.frame_id = base_frame_;
        joint_state_msg.name = joint_names_;
        joint_state_msg.position = current_joint_positions_;
        joint_state_pub_->publish(joint_state_msg);
    }
   
        
    RCLCPP_DEBUG(this->get_logger(), "ok, Published FK results and data_collection topics");
}

geometry_msgs::msg::PoseStamped FKWrapperNode::poseToPoseStamped(const geometry_msgs::msg::Pose& pose, const std::string& frame_id)
{  
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = frame_id;
    pose_stamped.pose = pose;
    return pose_stamped;
}

}

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<wbc::FKWrapperNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}