#pragma once

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <genie_msgs/msg/waist_state.hpp>
#include "wbc/ruckigcontroller.h"
#include "wbc/jointLimitManager.h"
#include <memory>
#include <vector>
#include <array>
#include <chrono>
#include <thread>
#include <atomic>

/*
author:tangyong
date:2025.10.30
des:Arm Control
*/

class ArmControlNode : public rclcpp::Node
{
public:
    ArmControlNode();
    ~ArmControlNode();
    void resetRobot();
private:
    void initParameters();
    void controlLoop();
    void armStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void waistStateCallback(const genie_msgs::msg::WaistState::SharedPtr msg);
    void targetJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void handle_arm_stream_control(const std_msgs::msg::Bool::SharedPtr msg);
    void checkWaistJointValid(const std::vector<double> &jointsWaist);
    void publishJointCommands(const std::vector<double> &waistJoints, const std::vector<double> &armJoints); 
   
private:
    double m_nControlFrequency{100.0};
    int m_nDof;
    double m_fVelScale{0.7};
    double m_fAccScale{1.0};
    double m_fJerkScale{1.0};
    std::string m_strRobotUrdfPath;

    std::atomic<bool> m_bEnableTimer{false};
    std::unique_ptr<RuckigController<16>> m_spRuckigController;
    std::unique_ptr<JointLimitsManager> m_spJointLimitsManager;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_armStateSub;
    rclcpp::Subscription<genie_msgs::msg::WaistState>::SharedPtr m_waistStateSub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_targetJointsSub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_realignTriggerSub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_armStreamControlSub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_armCommandPub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_waistCommandPub;
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::CallbackGroup::SharedPtr m_subscriptionGroup;
    rclcpp::CallbackGroup::SharedPtr m_timerGroup;

    std::vector<double> m_currentArmJoints;
    std::vector<double> m_currentWaistJoints;
    std::vector<double> m_targetArmJoints;
    std::vector<double> m_targetWaistJoints;
};