#include "wbc/jointLimitManager.h"
#include "wbc/tinyxml2.h"
#include <filesystem>
#include <map>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <chrono>
namespace fs = std::filesystem;
using namespace tinyxml2;

JointLimitsManager::JointLimitsManager(rclcpp::Logger logger,std::string strUrdfPath):m_logger(logger)
{
    m_vcJointNames = {
        "joint1_l", "joint2_l", "joint3_l", "joint4_l", "joint5_l", "joint6_l", "joint7_l", // Left arm
        "joint1_r", "joint2_r", "joint3_r", "joint4_r", "joint5_r", "joint6_r", "joint7_r"  // Right arm
    };

    m_vcJointLowerLimits.resize(14, 0);
    m_vcJointUpperLimits.resize(14, 0);

    //loadFromURDF(strUrdfPath);
    useDefaultLimits();
}


void JointLimitsManager::useDefaultLimits()
{
    // Based on physical measurement data (left/right arms identical)
    // joint4 upper limit is critical: physical max is 1.47 rad (84°), using 1.45 for 0.02 rad safety buffer
    m_vcJointLowerLimits = {
        -3.14, -1.92, -3.14, -1.75, -3.14, -1.7, -3.14, // Left arm: j1-j7 based on measured physical limits
        -3.14, -1.92, -3.14, -1.75, -3.14, -1.7, -3.14  // Right arm: j1-j7 (same as left)
    };

    m_vcJointUpperLimits = {
        3.14, 1.81, 3.14, 1.45, 3.14, 1.7, 3.14,  // Left arm: j4 max 1.45 to avoid hitting physical limit at 1.47
        3.14, 1.81, 3.14, 1.45, 3.14, 1.7, 3.14   // Right arm: j4 max 1.45 (same as left)
    };

    RCLCPP_WARN(m_logger, "Using default joint limits");
}

void JointLimitsManager::loadFromURDF(std::string strUrdfPath)
{
    if (fs::exists(strUrdfPath) && fs::is_regular_file(strUrdfPath))
    {
        XMLDocument doc;
        XMLError err = doc.LoadFile(strUrdfPath.c_str());
        if (err != XML_SUCCESS)
        {
            RCLCPP_ERROR(m_logger, "Failed to load URDF file: %s", strUrdfPath.c_str());
            useDefaultLimits();
            return;
        }

        XMLElement *root = doc.RootElement();
        if (!root)
        {
            RCLCPP_ERROR(m_logger, "Invalid URDF file: no root element");
            useDefaultLimits();
            return;
        }

        std::map<std::string, std::pair<double, double>> map_arm_joint_limits;
        XMLElement *pJoint = root->FirstChildElement("joint");
        while (pJoint)
        {
            const char *joint_name = pJoint->Attribute("name");
            if (joint_name)
            {
                std::string strJointName = joint_name;
                auto it = std::find_if(m_vcJointNames.begin(), m_vcJointNames.end(), [strJointName](const std::string &name)
                                       { return strJointName == name; });
                if (it != m_vcJointNames.end())
                {
                    tinyxml2::XMLElement *limit = pJoint->FirstChildElement("limit");
                    if (limit)
                    {
                        const char *lower_str = limit->Attribute("lower");
                        const char *upper_str = limit->Attribute("upper");

                        if (lower_str && upper_str)
                        {
                            try
                            {
                                double flower = std::stod(lower_str);
                                double fupper = std::stod(upper_str);

                                if (flower < fupper)
                                {
                                    map_arm_joint_limits[strJointName] = std::make_pair(flower, fupper);
                                }
                                else
                                {
                                    RCLCPP_ERROR(m_logger, "Invalid joint limits for %s: lower (%.3f) >= upper (%.3f)", 
                                        joint_name, flower, fupper);
                                    useDefaultLimits();
                                    return;
                                }
                            }
                            catch (const std::exception &e)
                            {
                                RCLCPP_WARN(m_logger, "Invalid limit values for joint %s: %s", joint_name, e.what());
                                useDefaultLimits();
                                return;
                            }
                        }
                    }
                }
            }

            pJoint = pJoint->NextSiblingElement("joint");
        }

      
        useDefaultLimits();
        
        int nSize = m_vcJointNames.size();
        for (int i = 0; i < nSize; i++)
        {
            auto it = map_arm_joint_limits.find(m_vcJointNames[i]);
            if (it != map_arm_joint_limits.end())
            {
                m_vcJointLowerLimits[i] = it->second.first;
                m_vcJointUpperLimits[i] = it->second.second;
            }
            else
            {
                RCLCPP_WARN(m_logger, "Joint %s not found in URDF, using default limits [%.3f, %.3f]", 
                    m_vcJointNames[i].c_str(), m_vcJointLowerLimits[i], m_vcJointUpperLimits[i]);
            }
        }
        
       
        for (int i = 0; i < nSize; i++)
        {
            if (m_vcJointNames[i] == "joint2_l" || m_vcJointNames[i] == "joint2_r")
            {
                m_vcJointUpperLimits[i] = 1.81;
                RCLCPP_INFO(m_logger, "Overridden %s upper limit to 1.81 (measured physical limit, URDF has 2.0)", 
                    m_vcJointNames[i].c_str());
            }
        }
        
        RCLCPP_INFO(m_logger, "Successfully loaded joint limits from URDF");
    }
    else
    {
        RCLCPP_WARN(m_logger, "URDF file not found: %s, using default limits", strUrdfPath.c_str());
        useDefaultLimits();
        return;
    }
}

std::vector<double> JointLimitsManager::applyLimits(const std::vector<double> &targetJoints) const
{
    if (targetJoints.size() != m_vcJointLowerLimits.size())
    {
        
        RCLCPP_ERROR(m_logger, "Target arm joints size (%zu) doesn't match arm joint limits size (%zu)", 
                     targetJoints.size(), m_vcJointLowerLimits.size());
        return targetJoints;
    }

    std::vector<double> resultJoints = targetJoints;

    for (size_t i = 0; i < targetJoints.size(); ++i)
    {
        double original_value = targetJoints[i];

        if (targetJoints[i] < m_vcJointLowerLimits[i])
        {
            resultJoints[i] = m_vcJointLowerLimits[i];
            
            // 避免刷屏日志
            static std::map<size_t, std::chrono::steady_clock::time_point> last_warn_time;
            auto now = std::chrono::steady_clock::now();
            auto& last_time = last_warn_time[i];
            
            if (now - last_time > std::chrono::seconds(1))
            {
                RCLCPP_WARN(m_logger, "Arm joint %s clamped to lower limit: %.3f -> %.3f (limits: [%.3f, %.3f])",
                    m_vcJointNames[i].c_str(), original_value, m_vcJointLowerLimits[i],
                    m_vcJointLowerLimits[i], m_vcJointUpperLimits[i]);
                last_time = now;
            }
        }
        else if (targetJoints[i] > m_vcJointUpperLimits[i])
        {
            resultJoints[i] = m_vcJointUpperLimits[i];
            
           // 避免刷屏日志
            static std::map<size_t, std::chrono::steady_clock::time_point> last_warn_time;
            auto now = std::chrono::steady_clock::now();
            auto& last_time = last_warn_time[i];
            
            if (now - last_time > std::chrono::seconds(1))
            {
                RCLCPP_WARN(m_logger, "Arm joint %s clamped to upper limit: %.3f -> %.3f (limits: [%.3f, %.3f])",
                    m_vcJointNames[i].c_str(), original_value, m_vcJointUpperLimits[i],
                    m_vcJointLowerLimits[i], m_vcJointUpperLimits[i]);
                last_time = now;
            }
        }
    }

    return resultJoints;
}

void JointLimitsManager::checkLimits(const std::vector<double>& targetJoints, double tolerance) const
{
    if (targetJoints.size() != m_vcJointLowerLimits.size())
    {
        RCLCPP_ERROR(m_logger, "Target arm joints size (%zu) doesn't match arm joint limits size (%zu)", 
                     targetJoints.size(), m_vcJointLowerLimits.size());
        return;
    }

    for (size_t i = 0; i < targetJoints.size(); ++i)
    {
        double violation = 0.0;
        bool isViolated = false;
        
        if (targetJoints[i] < m_vcJointLowerLimits[i])
        {
            violation = m_vcJointLowerLimits[i] - targetJoints[i];
            isViolated = true;
        }
        else if (targetJoints[i] > m_vcJointUpperLimits[i])
        {
            violation = targetJoints[i] - m_vcJointUpperLimits[i];
            isViolated = true;
        }
        
        if (isViolated && violation > tolerance)
        {
            static std::map<size_t, std::chrono::steady_clock::time_point> last_warn_time;
            auto now = std::chrono::steady_clock::now();
            auto& last_time = last_warn_time[i];
            
            if (now - last_time > std::chrono::seconds(1))
            {
                if (targetJoints[i] < m_vcJointLowerLimits[i])
                {
                    RCLCPP_WARN(m_logger,
                        "IK output joint %s exceeds lower limit: %.3f < %.3f (violation: %.3f rad). "
                        "This may prevent reaching target pose. Will be clamped at output stage.",
                        m_vcJointNames[i].c_str(), targetJoints[i], m_vcJointLowerLimits[i], violation);
                }
                else
                {
                    RCLCPP_WARN(m_logger,
                        "IK output joint %s exceeds upper limit: %.3f > %.3f (violation: %.3f rad). "
                        "This may prevent reaching target pose. Will be clamped at output stage.",
                        m_vcJointNames[i].c_str(), targetJoints[i], m_vcJointUpperLimits[i], violation);
                }
                last_time = now;
            }
        }
    }
}

void JointLimitsManager::printJointLimits() const
{
    RCLCPP_INFO(m_logger, "Arm Joint Limits:");
    for (size_t i = 0; i < m_vcJointNames.size(); ++i)
    {
        RCLCPP_INFO(m_logger, "  %s: [%.3f, %.3f]", 
            m_vcJointNames[i].c_str(), 
            m_vcJointLowerLimits[i], 
            m_vcJointUpperLimits[i]);
    }
}