#pragma once
#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp> 

/*
author:tangyong
date:2025.10.30
des:关节角度限制管理
*/

class JointLimitsManager {
private:
    rclcpp::Logger m_logger;
    std::vector<double> m_vcJointLowerLimits;
    std::vector<double> m_vcJointUpperLimits;
    std::vector<std::string> m_vcJointNames;
    void loadFromURDF(std::string strUrdfPath);
    void useDefaultLimits();
public:
    explicit JointLimitsManager(rclcpp::Logger logger,std::string strUrdfPath = "");  
    std::vector<double> applyLimits(const std::vector<double>& targetJoints) const; 
    void checkLimits(const std::vector<double>& targetJoints, double tolerance = 0.01) const; // Check and warn but don't clamp
    void printJointLimits() const;
};