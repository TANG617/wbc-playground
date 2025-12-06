#ifndef ROBOT_KINEMATICS_TOOL_H
#define ROBOT_KINEMATICS_TOOL_H

#include <string>
#include <vector>
#include <map>
#include <tuple>
#include <memory>
#include <chrono>
#include <Eigen/Dense>

// ROS消息头文件
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

// Placo库头文件（使用正确的命名空间）
#include "placo/model/robot_wrapper.h"
#include "placo/kinematics/kinematics_solver.h"
#include "placo/kinematics/frame_task.h"
#include "placo/kinematics/joints_task.h"
// Temporarily disabled for compilation without placo collision avoidance support
// #include "placo/kinematics/avoid_self_collisions_kinematics_constraint.h"

namespace wbc
{

class RobotKinematicsTool
{
public:
    RobotKinematicsTool();
    ~RobotKinematicsTool() = default;

    /**
     * @brief 初始化机器人运动学环境（适配A2D机器人URDF，100Hz实时性）
     * @param modelPath URDF文件路径
     * @param leftEndEffector 左臂末端执行器框架名称
     * @param rightEndEffector 右臂末端执行器框架名称
     * @param enableSelfCollision 是否启用自碰撞避免（默认true）
     * @param frameTaskWeight 框架任务权重（默认3.0）
     * @param kineticEnergyWeight 动能正则化权重（默认0.1）
     * @return bool 初始化成功返回true
     */
    bool robotKinematicsInit(
        const std::string& modelPath,
        const std::string& leftEndEffector,
        const std::string& rightEndEffector,
        bool enableSelfCollision = true,
        double frameTaskWeight = 3.0,
        double kineticEnergyWeight = 0.1
    );

    /**
     * @brief 配置IK求解参数（适配100Hz控制频率）
     * @param maxIterations 最大迭代次数（默认200次）
     * @param precision 求解精度（默认5e-6）
     * @param selfCollisionDistance 自碰撞安全距离（默认0.02m）
     * @param regularizationWeight 姿态正则化权重（默认1.2）
     */
    void configureIKSolver(
        int maxIterations = 200,
        double precision = 5e-6,
        double selfCollisionDistance = 0.02,
        double regularizationWeight = 1.2
    );

    /**
     * @brief 批量配置关节位置限位（用户提供的精准配置，覆盖URDF默认值）
     * @param lowerLimits 关节的最小位置（顺序与内置关节列表一致）
     * @param upperLimits 关节的最大位置（顺序与内置关节列表一致）
     * @return bool 配置成功返回true
     */
    bool setJointPositionLimits(const std::vector<double>& lowerLimits, const std::vector<double>& upperLimits);

    /**
     * @brief 批量配置关节速度限位（用户提供的参数）
     * @param velocityLimits 关节的最大速度（顺序与内置关节列表一致）
     * @return bool 配置成功返回true
     */
    bool setJointVelocityLimits(const std::vector<double>& velocityLimits);

    /**
     * @brief 逆运动学求解（双臂目标位姿，含位置/速度限位校验）
     * @param currentJointState 输入：当前关节状态（含名称、位置、速度）
     * @param leftTargetPose 输入：左臂目标位姿
     * @param rightTargetPose 输入：右臂目标位姿
     * @param outputJointState 输出：求解后的关节状态
     * @param solveTime 输出：求解耗时（毫秒）
     * @param dt 时间步长（用于速度计算，默认0.01s）
     * @return bool 求解成功且满足所有约束、耗时≤10ms返回true
     */
    bool ikSolve(
        const sensor_msgs::msg::JointState& currentJointState,
        const geometry_msgs::msg::PoseStamped& leftTargetPose,
        const geometry_msgs::msg::PoseStamped& rightTargetPose,
        sensor_msgs::msg::JointState& outputJointState,
        double& solveTime,
        double dt = 0.01
    );

    /**
     * @brief 正运动学求解（含位置限位校验）
     * @param jointPositions 输入：关节位置列表（顺序与内置关节列表一致）
     * @param leftPose 输出：左臂末端位姿
     * @param rightPose 输出：右臂末端位姿
     * @param solveTime 输出：求解耗时（毫秒）
     * @return bool 求解成功且满足位置限位、耗时≤10ms返回true
     */
    bool fkSolve(
        const std::vector<double>& jointPositions,
        geometry_msgs::msg::Pose& leftPose,
        geometry_msgs::msg::Pose& rightPose,
        double& solveTime
    );

    /**
     * @brief 获取内置的可动关节名称列表
     * @return 关节名称向量（顺序与限位配置一致）
     */
    std::vector<std::string> getJointNames() const { return m_vecJointNames; }

    /**
     * @brief 检查是否已初始化
     * @return 已初始化返回true
     */
    bool isInitialized() const { return m_bIsInitialized; }

    /**
     * @brief 设置初始关节位置到机器人模型
     * @param initialPositions 初始关节位置向量（顺序与内置关节列表一致）
     * @return bool 设置成功返回true
     */
    bool setInitialJointPositions(const std::vector<double>& initialPositions);

private:
    // 检查初始化状态
    bool checkInitStatus() const;

    // 校验关节位置/速度是否在限位内
    bool checkJointPositionLimit(const std::string& jointName, double position) const;
    bool checkJointVelocityLimit(const std::string& jointName, double velocity) const;

    // ROS Pose与Eigen Affine3d转换
    Eigen::Affine3d rosPoseToAffine3d(const geometry_msgs::msg::Pose& pose) const;
    geometry_msgs::msg::Pose affine3dToRosPose(const Eigen::Affine3d& transform) const;

    // 成员变量（m_ + 类型简写 + 语义含义，驼峰法）
    std::unique_ptr<placo::model::RobotWrapper> m_upRobotWrapper;          // 机器人模型包装器
    std::unique_ptr<placo::kinematics::KinematicsSolver> m_upKinematicsSolver;  // 运动学求解器
    
    // 框架任务（用于末端执行器控制）
    placo::kinematics::FrameTask m_leftFrameTask;
    placo::kinematics::FrameTask m_rightFrameTask;
    
    // 姿态任务（用于保持当前关节姿态，减少跳跃）
    placo::kinematics::JointsTask* m_pPostureTask{nullptr};
    
    // 自碰撞约束 (temporarily disabled)
    // std::shared_ptr<placo::kinematics::AvoidSelfCollisionsKinematicsConstraint> m_spSelfCollisionConstraint;
    
    std::string m_strLeftEeFrame;              // 左臂末端执行器帧（URDF中定义）
    std::string m_strRightEeFrame;             // 右臂末端执行器帧（URDF中定义）
    const std::vector<std::string> m_vecJointNames = {              // 可动关节列表（与限位顺序严格对应）
        "joint_lift_body",    // 0：移动关节（腰部升降）
        "joint_body_pitch",   // 1：旋转关节（腰部俯仰）
        "joint1_l",           // 2：旋转关节（左臂1轴）
        "joint2_l",           // 3：旋转关节（左臂2轴，物理极限1.81）
        "joint3_l",           // 4：旋转关节（左臂3轴）
        "joint4_l",           // 5：旋转关节（左臂4轴）
        "joint5_l",           // 6：旋转关节（左臂5轴）
        "joint6_l",           // 7：旋转关节（左臂6轴）
        "joint7_l",           // 8：旋转关节（左臂7轴）
        "joint1_r",           // 9：旋转关节（右臂1轴）
        "joint2_r",           // 10：旋转关节（右臂2轴，物理极限1.81）
        "joint3_r",           // 11：旋转关节（右臂3轴）
        "joint4_r",           // 12：旋转关节（右臂4轴）
        "joint5_r",           // 13：旋转关节（右臂5轴）
        "joint6_r",           // 14：旋转关节（右臂6轴）
        "joint7_r"            // 15：旋转关节（右臂7轴）
    };
    bool m_bEnableSelfCollision = true;                             // 是否启用自碰撞避免
    bool m_bIsInitialized = false;                                  // 初始化标志

    // 关节限位存储（用户提供的精准配置）
    std::map<std::string, std::pair<double, double>> m_mapPositionLimits; // 位置限位：<关节名, <下限位, 上限位>>
    std::map<std::string, double> m_mapVelocityLimits;                   // 速度限位：<关节名, 最大速度>

    // IK求解配置参数（适配100Hz）
    int m_iIkMaxIterations = 200;          // 最大迭代次数
    double m_dIkPrecision = 5e-3;          // 求解精度（调整为0.005，对应0.5cm可接受误差
    double m_dSelfCollisionDistance = 0.02;// 自碰撞安全距离
    double m_dRegularizationWeight = 1.2;  // 姿态正则化权重
    double m_dFrameTaskWeight = 8.0;       // 框架任务权重（从配置文件读取）
    double m_dKineticEnergyWeight = 0.1;   // 动能正则化权重（从配置文件读取）
};

} // namespace wbc

#endif // ROBOT_KINEMATICS_TOOL_H
