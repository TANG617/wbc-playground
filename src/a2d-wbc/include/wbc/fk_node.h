#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "wbc/robot_kinematics_tool.h" 
#include <memory>
#include <mutex>

#include <genie_msgs/msg/waist_state.hpp>  

namespace wbc {

class FKWrapperNode : public rclcpp::Node 
{
public:
    FKWrapperNode();
    ~FKWrapperNode() = default;

private:
    bool initializeParameters();
    bool initializeController();
    void initializeROS();
    void waistStateCallback(const genie_msgs::msg::WaistState::SharedPtr msg);
    void armJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void timerCallback();
    void computeAndPublishFK();
    geometry_msgs::msg::PoseStamped poseToPoseStamped(const geometry_msgs::msg::Pose& pose, const std::string& frame_id);
    
    std::unique_ptr<RobotKinematicsTool> controller_; 
    std::string urdf_path_;
    std::string base_frame_;
    std::string left_ee_frame_;
    std::string right_ee_frame_;
    double frequency_;
    bool publish_tf_;
    bool enable_robot_state_publisher_;
    
    // 16 joints: 2 waist + 14 arm
    std::vector<double> current_joint_positions_; 
    std::vector<std::string> joint_names_;
    
    std::mutex data_mutex_;
    bool has_waist_data_;
    bool has_arm_data_;
    
    rclcpp::Subscription<genie_msgs::msg::WaistState>::SharedPtr waist_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arm_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_tcp_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_tcp_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
} 