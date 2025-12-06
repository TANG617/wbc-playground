#include "wbc/robot_kinematics_tool.h"
#include <stdexcept>
#include <algorithm>
#include <cmath>

namespace wbc
{

RobotKinematicsTool::RobotKinematicsTool()
{
}

bool RobotKinematicsTool::robotKinematicsInit(
    const std::string& modelPath,
    const std::string& leftEndEffector,
    const std::string& rightEndEffector,
    bool enableSelfCollision,
    double frameTaskWeight,
    double kineticEnergyWeight
)
{
    try
    {
        // 加载机器人模型（使用正确的 Placo API）
        m_upRobotWrapper = std::make_unique<placo::model::RobotWrapper>(
            modelPath, placo::model::RobotWrapper::Flags::COLLISION_AS_VISUAL);
        
        if (!m_upRobotWrapper)
        {
            RCLCPP_ERROR(rclcpp::get_logger("RobotKinematicsTool"), 
                        "机器人模型加载失败：%s", modelPath.c_str());
            return false;
        }

        // 存储末端执行器框架名称
        m_strLeftEeFrame = leftEndEffector;
        m_strRightEeFrame = rightEndEffector;

        // 校验所有可动关节是否存在于URDF中
        for (const auto& jointName : m_vecJointNames)
        {
            try
            {
                m_upRobotWrapper->get_joint(jointName);
            }
            catch (const std::exception& e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("RobotKinematicsTool"), 
                            "URDF中不存在关节：%s", jointName.c_str());
                return false;
            }
        }

        // 校验末端执行器帧是否存在
        try
        {
            m_upRobotWrapper->get_frame_index(m_strLeftEeFrame);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("RobotKinematicsTool"), 
                        "左臂末端执行器帧不存在：%s", m_strLeftEeFrame.c_str());
            return false;
        }

        try
        {
            m_upRobotWrapper->get_frame_index(m_strRightEeFrame);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("RobotKinematicsTool"), 
                        "右臂末端执行器帧不存在：%s", m_strRightEeFrame.c_str());
            return false;
        }

        // 初始化运动学求解器
        m_upKinematicsSolver = std::make_unique<placo::kinematics::KinematicsSolver>(*m_upRobotWrapper);
        
        if (!m_upKinematicsSolver)
        {
            RCLCPP_ERROR(rclcpp::get_logger("RobotKinematicsTool"), 
                        "运动学求解器创建失败");
            return false;
        }

        // 禁用浮动基座（机器人固定在地面）
        m_upKinematicsSolver->mask_fbase(true);

        // 禁用稀疏性以提高实时性能
        m_upKinematicsSolver->problem.use_sparsity = false;
        m_upKinematicsSolver->problem.rewrite_equalities = false;

        // 启用关节和速度限制
        m_upKinematicsSolver->enable_joint_limits(true);
        m_upKinematicsSolver->enable_velocity_limits(true);

        // 添加正则化任务（最低优先级）
        m_upKinematicsSolver->add_regularization_task(0.12).configure("regularization", "soft", 1e-2);

        // 保存配置参数
        m_dFrameTaskWeight = frameTaskWeight;
        m_dKineticEnergyWeight = kineticEnergyWeight;

        // 添加框架任务用于末端执行器控制（6DOF位姿跟踪）
        m_leftFrameTask = m_upKinematicsSolver->add_frame_task(m_strLeftEeFrame);
        m_rightFrameTask = m_upKinematicsSolver->add_frame_task(m_strRightEeFrame);

        // 配置框架任务权重（从参数读取）
        m_leftFrameTask.configure("left_ee", "soft", m_dFrameTaskWeight, 0.5);
        m_rightFrameTask.configure("right_ee", "soft", m_dFrameTaskWeight, 0.5);

        // 添加姿态任务（保持当前关节姿态，减少跳跃）
        // 使用全关节约束策略：所有关节协同工作，避免过度补偿
        m_pPostureTask = &m_upKinematicsSolver->add_joints_task();
        m_pPostureTask->configure("posture", "soft", 0.02);  // 全关节约束权重
        
        // 姿态任务的参考值将在机器人状态初始化后设置（见下方）

        // Add kinetic energy regularization task with configuration (from parameter)
        m_upKinematicsSolver->add_kinetic_energy_regularization_task(m_dKineticEnergyWeight).configure("kinetic_energy", "soft", m_dKineticEnergyWeight);

        // 保存自碰撞配置
        m_bEnableSelfCollision = enableSelfCollision;

        // 配置自碰撞避免 (temporarily disabled)
        // if (m_bEnableSelfCollision)
        // {
        //     m_spSelfCollisionConstraint = std::make_shared<placo::kinematics::AvoidSelfCollisionsKinematicsConstraint>(
        //         m_upRobotWrapper.get(), m_dSelfCollisionDistance);
        //     m_upKinematicsSolver->add_constraint(m_spSelfCollisionConstraint);
        // }

        // 初始化机器人状态为零
        for (const auto& jointName : m_vecJointNames)
        {
            try
            {
                m_upRobotWrapper->set_joint(jointName, 0.0);
            }
            catch (const std::exception&)
            {
                // 关节可能不存在，跳过
            }
        }
        m_upRobotWrapper->update_kinematics();

        // 现在机器人状态已初始化为零位，设置姿态任务的参考值
        // 为所有关节（腰部+双臂全部7自由度）设置姿态目标
        std::vector<std::string> all_joints = {
            "joint_lift_body", "joint_body_pitch",  // 腰部 2个
            "joint1_l", "joint2_l", "joint3_l", "joint4_l", "joint5_l", "joint6_l", "joint7_l",  // 左臂 7个
            "joint1_r", "joint2_r", "joint3_r", "joint4_r", "joint5_r", "joint6_r", "joint7_r"   // 右臂 7个
        };
        
        for (const auto& jointName : all_joints)
        {
            try {
                double jointValue = m_upRobotWrapper->get_joint(jointName);  // 获取零位值
                m_pPostureTask->set_joint(jointName, jointValue);
            } catch (const std::exception& e) {
                // If joint doesn't exist in model, skip it
            }
        }
        
        RCLCPP_INFO(rclcpp::get_logger("RobotKinematicsTool"), 
                   "姿态任务配置完成：全关节约束策略（参考零位），共%zu个关节", 
                   all_joints.size());

        m_bIsInitialized = true;
        RCLCPP_INFO(rclcpp::get_logger("RobotKinematicsTool"), 
                   "初始化成功，A2D机器人%zu个可动关节加载完成", m_vecJointNames.size());
        RCLCPP_INFO(rclcpp::get_logger("RobotKinematicsTool"), 
                   "IK Solver权重配置: FrameTaskWeight=%.3f, KineticEnergyWeight=%.3f", 
                   m_dFrameTaskWeight, m_dKineticEnergyWeight);
        return true;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("RobotKinematicsTool"), 
                    "初始化异常：%s", e.what());
        m_bIsInitialized = false;
        return false;
    }
}

void RobotKinematicsTool::configureIKSolver(
    int maxIterations,
    double precision,
    double selfCollisionDistance,
    double regularizationWeight
)
{
    m_iIkMaxIterations = (maxIterations > 0) ? maxIterations : 200;
    m_dIkPrecision = (precision > 0) ? precision : 5e-6;
    m_dSelfCollisionDistance = (selfCollisionDistance > 0) ? selfCollisionDistance : 0.02;
    m_dRegularizationWeight = (regularizationWeight > 0) ? regularizationWeight : 1.2;

    // 更新自碰撞安全距离 (temporarily disabled)
    // if (m_spSelfCollisionConstraint)
    // {
    //     m_spSelfCollisionConstraint->distance = m_dSelfCollisionDistance;
    // }

    RCLCPP_INFO(rclcpp::get_logger("RobotKinematicsTool"), 
               "IK求解器配置完成：迭代次数=%d 精度=%.2e 自碰撞距离=%.3f", 
               m_iIkMaxIterations, m_dIkPrecision, m_dSelfCollisionDistance);
}

bool RobotKinematicsTool::setJointPositionLimits(const std::vector<double>& lowerLimits, const std::vector<double>& upperLimits)
{
    if (!checkInitStatus()) return false;

    // 校验限位数量与关节数量一致
    size_t jointCount = m_vecJointNames.size();
    if (lowerLimits.size() != jointCount || upperLimits.size() != jointCount)
    {
        RCLCPP_ERROR(rclcpp::get_logger("RobotKinematicsTool"), 
                    "位置限位数量错误：期望%zu个，实际下限位%zu个，上限位%zu个", 
                    jointCount, lowerLimits.size(), upperLimits.size());
        return false;
    }

    // 批量配置位置限位（覆盖URDF默认值）
    for (size_t i = 0; i < jointCount; i++)
    {
        const std::string& jointName = m_vecJointNames[i];
        double lower = lowerLimits[i];
        double upper = upperLimits[i];

        if (lower >= upper)
        {
            RCLCPP_ERROR(rclcpp::get_logger("RobotKinematicsTool"), 
                        "关节%s位置限位无效：min=%.3f ≥ max=%.3f", 
                        jointName.c_str(), lower, upper);
            return false;
        }

        m_mapPositionLimits[jointName] = {lower, upper};
        
        // 在机器人模型中设置限制（添加软限位margin以避免硬碰边界）
        // 软限位margin：让IK求解器在真正触及限位前就开始"软着陆"
        double soft_limit_margin = 0.08;  // 0.08 rad (~4.6°) margin
        
        // 对于腰部关节（范围极窄），不应用margin
        if (jointName == "joint_lift_body" || jointName == "joint_body_pitch")
        {
            soft_limit_margin = 0.0;
        }
        
        double placo_lower = lower + soft_limit_margin;
        double placo_upper = upper - soft_limit_margin;
        
        // 确保软限位不会反转
        if (placo_lower >= placo_upper)
        {
            placo_lower = lower;
            placo_upper = upper;
        }
        
        try
        {
            m_upRobotWrapper->set_joint_limits(jointName, placo_lower, placo_upper);
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN(rclcpp::get_logger("RobotKinematicsTool"), 
                       "无法设置关节%s的限制：%s", jointName.c_str(), e.what());
        }

        // 特殊关节注释
        if (jointName == "joint_lift_body" || jointName == "joint_body_pitch")
        {
            RCLCPP_INFO(rclcpp::get_logger("RobotKinematicsTool"), 
                       "腰部关节%s位置限位：[%.3f,%.3f]（窄范围保持稳定）", 
                       jointName.c_str(), lower, upper);
        }
        else if (jointName == "joint2_l" || jointName == "joint2_r" || 
                 jointName == "joint4_l" || jointName == "joint4_r")
        {
            RCLCPP_INFO(rclcpp::get_logger("RobotKinematicsTool"), 
                       "臂关节%s限位：外部[%.3f,%.3f] IK软限位[%.3f,%.3f] margin=%.2f", 
                       jointName.c_str(), lower, upper, placo_lower, placo_upper, soft_limit_margin);
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("RobotKinematicsTool"), 
               "%zu个关节位置限位配置完成", jointCount);
    return true;
}

bool RobotKinematicsTool::setJointVelocityLimits(const std::vector<double>& velocityLimits)
{
    if (!checkInitStatus()) return false;

    // 校验速度限位数量与关节数量一致
    size_t jointCount = m_vecJointNames.size();
    if (velocityLimits.size() != jointCount)
    {
        RCLCPP_ERROR(rclcpp::get_logger("RobotKinematicsTool"), 
                    "速度限位数量错误：期望%zu个，实际%zu个", 
                    jointCount, velocityLimits.size());
        return false;
    }

    // 批量配置速度限位
    for (size_t i = 0; i < jointCount; i++)
    {
        const std::string& jointName = m_vecJointNames[i];
        double maxVel = velocityLimits[i];

        if (maxVel <= 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("RobotKinematicsTool"), 
                        "关节%s速度限位无效：%.3f", jointName.c_str(), maxVel);
            return false;
        }

        m_mapVelocityLimits[jointName] = maxVel;
        
        // 在机器人模型中设置速度限制
        try
        {
            m_upRobotWrapper->set_velocity_limit(jointName, maxVel);
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN(rclcpp::get_logger("RobotKinematicsTool"), 
                       "无法设置关节%s的速度限制：%s", jointName.c_str(), e.what());
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("RobotKinematicsTool"), 
               "%zu个关节速度限位配置完成", jointCount);
    return true;
}

bool RobotKinematicsTool::setInitialJointPositions(const std::vector<double>& initialPositions)
{
    if (!checkInitStatus()) return false;
    
    size_t jointCount = m_vecJointNames.size();
    if (initialPositions.size() != jointCount)
    {
        RCLCPP_ERROR(rclcpp::get_logger("RobotKinematicsTool"), 
                    "初始关节位置数量错误：期望%zu个，实际%zu个", 
                    jointCount, initialPositions.size());
        return false;
    }
    
    try
    {
        for (size_t i = 0; i < jointCount; ++i)
        {
            try
            {
                m_upRobotWrapper->set_joint(m_vecJointNames[i], initialPositions[i]);
            }
            catch (const std::exception& e)
            {
                RCLCPP_WARN(rclcpp::get_logger("RobotKinematicsTool"), 
                           "无法设置关节%s的初始位置：%s", m_vecJointNames[i].c_str(), e.what());
                return false;
            }
        }
        
        // 更新运动学
        m_upRobotWrapper->update_kinematics();
        
        RCLCPP_INFO(rclcpp::get_logger("RobotKinematicsTool"), 
                   "初始关节位置设置成功（%zu个关节）", jointCount);
        return true;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("RobotKinematicsTool"), 
                    "设置初始关节位置异常：%s", e.what());
        return false;
    }
}

bool RobotKinematicsTool::ikSolve(
    const sensor_msgs::msg::JointState& currentJointState,
    const geometry_msgs::msg::PoseStamped& leftTargetPose,
    const geometry_msgs::msg::PoseStamped& rightTargetPose,
    sensor_msgs::msg::JointState& outputJointState,
    double& solveTime,
    double dt
)
{
    // Print IK solver configuration parameters (throttled to once per 5 seconds to avoid log spam)
    static auto logger = rclcpp::get_logger("RobotKinematicsTool");
    static auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
    RCLCPP_INFO_THROTTLE(logger, *clock, 5000,
                         "IK Solver权重: FrameTaskWeight=%.3f, KineticEnergyWeight=%.3f", 
                         m_dFrameTaskWeight, m_dKineticEnergyWeight);

    solveTime = 0.0;
    if (!checkInitStatus()) return false;
    if (m_mapPositionLimits.empty() || m_mapVelocityLimits.empty())
    {
        RCLCPP_ERROR(rclcpp::get_logger("RobotKinematicsTool"), 
                    "未配置关节限位，请先调用setJointPositionLimits和setJointVelocityLimits");
        return false;
    }

    auto start = std::chrono::high_resolution_clock::now();

    try
    {
        // 1. 更新机器人状态并校验当前关节状态的位置和速度限位
        for (size_t i = 0; i < currentJointState.name.size(); i++)
        {
            const std::string& jointName = currentJointState.name[i];
            if (!checkJointPositionLimit(jointName, currentJointState.position[i])) 
                return false;

            // 校验速度（若提供）
            if (i < currentJointState.velocity.size() && 
                !checkJointVelocityLimit(jointName, currentJointState.velocity[i]))
            {
                return false;
            }

            try
            {
                m_upRobotWrapper->set_joint(jointName, currentJointState.position[i]);
            }
            catch (const std::exception& e)
            {
                RCLCPP_WARN(rclcpp::get_logger("RobotKinematicsTool"), 
                           "无法设置关节%s：%s", jointName.c_str(), e.what());
            }
        }

        // 设置求解器时间步长
        m_upKinematicsSolver->dt = dt;

        // 更新运动学
        m_upRobotWrapper->update_kinematics();

        // 2. 姿态任务使用固定的零位参考（在robotKinematicsInit中已设置）
        // 
        // 作用：作为固定"锚点"，防止IK解在冗余空间中漂移到等效解（如180度翻转）
        // 
        // 机制详解：
        //   1) 机器人当前状态已在第393-415行设置为真实状态，所以初始位置正确
        //   2) 姿态任务参考值固定为零位，权重0.05提供轻微的"拉力"
        //   3) 末端任务权重3.0远大于姿态任务0.05，主要满足末端跟踪需求
        //   4) 姿态任务的固定参考防止在冗余空间中累积漂移
        // 
        // 为什么不能动态更新参考值？
        //   - 如果每次都更新参考值=当前状态，姿态任务会"跟随"当前状态
        //   - 这会失去固定"锚点"作用，IK解会在零空间中自由漂移
        //   - 最终导致：遥操作一段时间后，关节会漂移到等效解（180度翻转）
        // 
        // 注意：绝对不要在这里动态更新姿态任务的参考值！

        // 3. 设置框架任务目标（6DOF位姿跟踪）
        Eigen::Affine3d leftTarget = rosPoseToAffine3d(leftTargetPose.pose);
        Eigen::Affine3d rightTarget = rosPoseToAffine3d(rightTargetPose.pose);

        m_leftFrameTask.set_T_world_frame(leftTarget);
        m_rightFrameTask.set_T_world_frame(rightTarget);

        // 4. 执行IK求解
        Eigen::VectorXd deltaQ = m_upKinematicsSolver->solve(true); // 应用到机器人状态

        // 5. 提取解并校验位置限位
        outputJointState.name.clear();
        outputJointState.position.clear();
        outputJointState.velocity.clear();
        outputJointState.effort.clear();
        outputJointState.name.reserve(m_vecJointNames.size());
        outputJointState.position.reserve(m_vecJointNames.size());
        outputJointState.velocity.reserve(m_vecJointNames.size());

        // 获取 Placo 的关节名称
        auto placoJointNames = m_upRobotWrapper->joint_names(false);

        for (const auto& jointName : m_vecJointNames)
        {
            double solvedPos = 0.0;
            double solvedVel = 0.0;

            try
            {
                // 获取求解后的位置
                solvedPos = m_upRobotWrapper->get_joint(jointName);
                
                // 从求解器解中获取速度
                for (size_t j = 0; j < placoJointNames.size() && j < static_cast<size_t>(deltaQ.size()); ++j)
                {
                    if (placoJointNames[j] == jointName)
                    {
                        solvedVel = deltaQ(j);
                        break;
                    }
                }
            }
            catch (const std::exception& e)
            {
                RCLCPP_WARN(rclcpp::get_logger("RobotKinematicsTool"), 
                           "无法获取关节%s的解：%s", jointName.c_str(), e.what());
                // 使用当前位置作为回退
                auto it = std::find(currentJointState.name.begin(), 
                                   currentJointState.name.end(), jointName);
                if (it != currentJointState.name.end())
                {
                    size_t idx = std::distance(currentJointState.name.begin(), it);
                    solvedPos = currentJointState.position[idx];
                }
            }

            // 校验位置限位
            if (!checkJointPositionLimit(jointName, solvedPos)) 
                return false;

            // 应用速度限制
            if (m_mapVelocityLimits.find(jointName) != m_mapVelocityLimits.end())
            {
                double maxVel = m_mapVelocityLimits.at(jointName);
                solvedVel = std::clamp(solvedVel, -maxVel, maxVel);
            }

            // 校验速度限位
            if (!checkJointVelocityLimit(jointName, solvedVel))
            {
                RCLCPP_ERROR(rclcpp::get_logger("RobotKinematicsTool"), 
                            "IK求解速度超标：%s 速度=%.3f", jointName.c_str(), solvedVel);
                return false;
            }

            outputJointState.name.push_back(jointName);
            outputJointState.position.push_back(solvedPos);
            outputJointState.velocity.push_back(solvedVel);
        }

        // 5. 耗时校验
        auto end = std::chrono::high_resolution_clock::now();
        solveTime = std::chrono::duration<double, std::milli>(end - start).count();
        if (solveTime > 18.0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("RobotKinematicsTool"), 
                        "IK求解耗时超标：%.2fms（阈值18ms）", solveTime);
            return false;
        }

        return true;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("RobotKinematicsTool"), 
                    "IK求解异常：%s", e.what());
        return false;
    }
}

bool RobotKinematicsTool::fkSolve(
    const std::vector<double>& jointPositions,
    geometry_msgs::msg::Pose& leftPose,
    geometry_msgs::msg::Pose& rightPose,
    double& solveTime
)
{
    solveTime = 0.0;
    if (!checkInitStatus()) return false;
    if (m_mapPositionLimits.empty())
    {
        RCLCPP_ERROR(rclcpp::get_logger("RobotKinematicsTool"), 
                    "未配置关节位置限位，请先调用setJointPositionLimits");
        return false;
    }

    auto start = std::chrono::high_resolution_clock::now();

    try
    {
        // 校验关节数量和位置限位
        size_t jointCount = m_vecJointNames.size();
        if (jointPositions.size() != jointCount)
        {
            RCLCPP_ERROR(rclcpp::get_logger("RobotKinematicsTool"), 
                        "FK求解关节数量错误：期望%zu个，实际%zu个", 
                        jointCount, jointPositions.size());
            return false;
        }

        for (size_t i = 0; i < jointCount; i++)
        {
            const std::string& jointName = m_vecJointNames[i];
            double pos = jointPositions[i];
            if (!checkJointPositionLimit(jointName, pos)) 
                return false;
            
            try
            {
                m_upRobotWrapper->set_joint(jointName, pos);
            }
            catch (const std::exception& e)
            {
                RCLCPP_WARN(rclcpp::get_logger("RobotKinematicsTool"), 
                           "无法设置关节%s：%s", jointName.c_str(), e.what());
            }
        }

        // 更新运动学
        m_upRobotWrapper->update_kinematics();

        // 计算末端位姿
        Eigen::Affine3d leftTransform = m_upRobotWrapper->get_T_world_frame(m_strLeftEeFrame);
        Eigen::Affine3d rightTransform = m_upRobotWrapper->get_T_world_frame(m_strRightEeFrame);
        leftPose = affine3dToRosPose(leftTransform);
        rightPose = affine3dToRosPose(rightTransform);

        // 耗时校验
        auto end = std::chrono::high_resolution_clock::now();
        solveTime = std::chrono::duration<double, std::milli>(end - start).count();
        if (solveTime > 18.0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("RobotKinematicsTool"), 
                        "FK求解耗时超标：%.2fms（阈值18ms）", solveTime);
            return false;
        }

        return true;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("RobotKinematicsTool"), 
                    "FK求解异常：%s", e.what());
        return false;
    }
}

bool RobotKinematicsTool::checkInitStatus() const
{
    if (!m_bIsInitialized)
    {
        RCLCPP_ERROR(rclcpp::get_logger("RobotKinematicsTool"), 
                    "未初始化！请先调用robotKinematicsInit");
        return false;
    }
    return true;
}

bool RobotKinematicsTool::checkJointPositionLimit(const std::string& jointName, double position) const
{
    auto it = m_mapPositionLimits.find(jointName);
    if (it == m_mapPositionLimits.end())
    {
        RCLCPP_ERROR(rclcpp::get_logger("RobotKinematicsTool"), 
                    "未配置关节%s的位置限位", jointName.c_str());
        return false;
    }

    double lower = it->second.first;
    double upper = it->second.second;
    if (position < lower - 1e-4 || position > upper + 1e-4) // 允许1e-4微小误差
    {
        RCLCPP_ERROR(rclcpp::get_logger("RobotKinematicsTool"), 
                    "关节%s位置超限位：当前=%.3f 范围=[%.3f,%.3f]", 
                    jointName.c_str(), position, lower, upper);
        return false;
    }
    return true;
}

bool RobotKinematicsTool::checkJointVelocityLimit(const std::string& jointName, double velocity) const
{
    auto it = m_mapVelocityLimits.find(jointName);
    if (it == m_mapVelocityLimits.end())
    {
        RCLCPP_ERROR(rclcpp::get_logger("RobotKinematicsTool"), 
                    "未配置关节%s的速度限位", jointName.c_str());
        return false;
    }

    double maxVel = it->second;
    if (std::fabs(velocity) > maxVel + 1e-4)
    {
        RCLCPP_ERROR(rclcpp::get_logger("RobotKinematicsTool"), 
                    "关节%s速度超限位：当前=%.3f 最大允许=%.3f", 
                    jointName.c_str(), velocity, maxVel);
        return false;
    }
    return true;
}

Eigen::Affine3d RobotKinematicsTool::rosPoseToAffine3d(const geometry_msgs::msg::Pose& pose) const
{
    Eigen::Vector3d position(pose.position.x, pose.position.y, pose.position.z);
    Eigen::Quaterniond orientation(pose.orientation.w, pose.orientation.x, 
                                   pose.orientation.y, pose.orientation.z);
    orientation.normalize();

    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.translation() = position;
    transform.linear() = orientation.toRotationMatrix();
    return transform;
}

geometry_msgs::msg::Pose RobotKinematicsTool::affine3dToRosPose(const Eigen::Affine3d& transform) const
{
    geometry_msgs::msg::Pose pose;
    Eigen::Vector3d position = transform.translation();
    Eigen::Quaterniond orientation(transform.rotation());
    orientation.normalize();

    pose.position.x = position.x();
    pose.position.y = position.y();
    pose.position.z = position.z();
    pose.orientation.x = orientation.x();
    pose.orientation.y = orientation.y();
    pose.orientation.z = orientation.z();
    pose.orientation.w = orientation.w();

    return pose;
}

} // namespace wbc
