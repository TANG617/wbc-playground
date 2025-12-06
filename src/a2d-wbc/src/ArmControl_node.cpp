#include "wbc/ArmControl_node.h"
#include <thread>
#include <chrono>
#include <cmath>
#include <rclcpp/callback_group.hpp>

ArmControlNode::ArmControlNode(): Node("arm_control_node")
{
    initParameters();
    m_spJointLimitsManager = std::make_unique<JointLimitsManager>(get_logger(),m_strRobotUrdfPath);
    m_spJointLimitsManager->printJointLimits();

    std::array<double, 16> maxVelocity;
    std::array<double, 16> maxAcceleration;
    std::array<double, 16> maxJerk;

    maxVelocity.fill(4.0);
    maxAcceleration.fill(10.0);
    maxJerk.fill(100.0);

    for (size_t i = 0; i < 16; ++i)
    {
        maxVelocity[i] *= m_fVelScale;
        maxAcceleration[i] *= m_fAccScale;
        maxJerk[i] *= m_fJerkScale;
    }

    double fdt = 1.0 / m_nControlFrequency;

    m_spRuckigController = std::make_unique<RuckigController<16>>(maxVelocity, maxAcceleration, maxJerk, fdt);

    m_subscriptionGroup = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    m_timerGroup = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    m_timer = create_wall_timer(std::chrono::duration<double>(fdt), std::bind(&ArmControlNode::controlLoop, this), m_timerGroup);

    rclcpp::SubscriptionOptions options;
    options.callback_group = m_subscriptionGroup;
    m_armStateSub = create_subscription<sensor_msgs::msg::JointState>("/hal/arm_joint_state", 100,
                    std::bind(&ArmControlNode::armStateCallback, this, std::placeholders::_1), options);

    m_waistStateSub = create_subscription<genie_msgs::msg::WaistState>("/hal/waist_state", 100,
                    std::bind(&ArmControlNode::waistStateCallback, this, std::placeholders::_1), options);

    m_targetJointsSub = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 100,
                    std::bind(&ArmControlNode::targetJointStateCallback, this, std::placeholders::_1), options);

    // Subscribe to arm stream realign trigger for resetting robot (F2 only)
    m_realignTriggerSub = create_subscription<std_msgs::msg::Bool>(
        "/arm_stream_realign_trigger",
        rclcpp::QoS(10),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            if (msg->data) {
                RCLCPP_INFO(this->get_logger(), 
                    "[F2] Arm stream realign trigger received - Calling resetRobot()");
                resetRobot();
            }
        },
        options);

    // Subscribe to arm stream control for safety (disable control loop when stream is closed)
    m_armStreamControlSub = create_subscription<std_msgs::msg::Bool>(
        "/arm_stream_control",
        rclcpp::QoS(10),
        std::bind(&ArmControlNode::handle_arm_stream_control, this, std::placeholders::_1),
        options);

    m_armCommandPub = this->create_publisher<sensor_msgs::msg::JointState>("/wbc/arm_command", 100);
    
    m_waistCommandPub = this->create_publisher<sensor_msgs::msg::JointState>("/wbc/waist_command", 100);
    
    RCLCPP_INFO(get_logger(), "Subscribed to /arm_stream_control for safety control");
}

void ArmControlNode::initParameters()
{
    declare_parameter("dof", 16);
    declare_parameter("control_frequency", 100.0);
    declare_parameter("vel_scale", 1.0);
    declare_parameter("acc_scale", 1.0);
    declare_parameter("jerk_scale", 1.0);
    declare_parameter("robot_urdf_path", "a2d-tele/modules/control/wbc/assets/A2D_NoHand/A2D_NoHand_Flattened.urdf");

    m_nDof = get_parameter("dof").as_int();
    m_nControlFrequency = get_parameter("control_frequency").as_double();
    m_fVelScale = get_parameter("vel_scale").as_double();
    m_fAccScale = get_parameter("acc_scale").as_double();
    m_fJerkScale = get_parameter("jerk_scale").as_double();
    m_strRobotUrdfPath = get_parameter("robot_urdf_path").as_string();

    if (m_nDof != 16)
    {
        RCLCPP_ERROR(get_logger(), "DOF should be 16, but got %d", m_nDof);
        throw std::runtime_error("Invalid DOF parameter");
    }

    RCLCPP_INFO(get_logger(), "Parameters: m_fVelScale=%.1f, m_fAccScale=%.1f, m_fJerkScale=%.1f",
                m_fVelScale, m_fAccScale, m_fJerkScale);
}

ArmControlNode::~ArmControlNode()
{
}

void ArmControlNode::controlLoop()
{
    if (m_targetArmJoints.empty() || m_targetWaistJoints.empty() || !m_bEnableTimer)
    {
        // Add throttled log to show control loop is disabled
        static auto last_warn_time = get_clock()->now();
        auto now = get_clock()->now();
        if ((now - last_warn_time).seconds() >= 5.0) {
            if (!m_bEnableTimer) {
                RCLCPP_INFO(get_logger(), "[Control Loop] Disabled - waiting for arm stream to open (F2)");
            } else if (m_targetArmJoints.empty() || m_targetWaistJoints.empty()) {
                RCLCPP_INFO(get_logger(), "[Control Loop] Waiting for target joint data");
            }
            last_warn_time = now;
        }
        return;
    }
   
    std::array<double, 16> target_joints;
    std::copy(m_targetWaistJoints.begin(), m_targetWaistJoints.end(), target_joints.begin());
    std::copy(m_targetArmJoints.begin(), m_targetArmJoints.end(), target_joints.begin() + 2);
    
    std::vector<double> targetArmJointsLimited = m_spJointLimitsManager->applyLimits(m_targetArmJoints);
    std::copy(targetArmJointsLimited.begin(), targetArmJointsLimited.end(), target_joints.begin() + 2);
    
    m_spRuckigController->setTarget(target_joints);

    std::array<double, 16> posCurrent, velCurrent;
    bool ruckig_success = m_spRuckigController->update(posCurrent, velCurrent);
    
    if (!ruckig_success)
    {   
        static rclcpp::Clock error_clock;
        RCLCPP_WARN_THROTTLE(get_logger(), error_clock, 2000,"WARN,Ruckig update failed (error state), holding current position");
        
        if (!m_currentArmJoints.empty() && !m_currentWaistJoints.empty())
        {
            publishJointCommands(m_currentWaistJoints, m_currentArmJoints);
        }
        return;
    }
    
  
    bool data_valid = true;
    for (size_t i = 0; i < posCurrent.size(); ++i)
    {
        if (std::isnan(posCurrent[i]) || std::isinf(posCurrent[i]) || 
            std::isnan(velCurrent[i]) || std::isinf(velCurrent[i]))
        {
            data_valid = false;
            break;
        }
    }
    
    if (!data_valid)
    {      
        static rclcpp::Clock error_clock;
        RCLCPP_WARN_THROTTLE(get_logger(), error_clock, 2000,"WARN,Ruckig output contains NaN/Inf, holding current position");
        
        if (!m_currentArmJoints.empty() && !m_currentWaistJoints.empty())
        {
            publishJointCommands(m_currentWaistJoints, m_currentArmJoints);
        }
        return;
    }

    std::vector<double> posWaist(posCurrent.begin(), posCurrent.begin() + 2);
    std::vector<double> posArm(posCurrent.begin() + 2, posCurrent.end());
    publishJointCommands(posWaist, posArm);
}

void ArmControlNode::armStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (msg->position.empty())
    {
        RCLCPP_WARN(get_logger(), "WARN,Received empty arm joint state");
        return;
    }

    m_currentArmJoints = msg->position;
}

void ArmControlNode::waistStateCallback(const genie_msgs::msg::WaistState::SharedPtr msg)
{
    if (msg->motor_states.empty() || msg->motor_states.size() < 2)
    {
        RCLCPP_WARN(get_logger(), "WARN,Received invalid waist state");
        return;
    }

    double fWaistPitch = msg->motor_states[0].position;
    double fwaistLift = msg->motor_states[1].position;

    checkWaistJointValid({fWaistPitch, fwaistLift});

    m_currentWaistJoints = {fWaistPitch, fwaistLift};
}

void ArmControlNode::targetJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (msg->position.empty())
    {
        RCLCPP_WARN(get_logger(), "WARN,Received empty target joints");
        return;
    }

    if (msg->position.size() != 16)
    {
        RCLCPP_ERROR(get_logger(), "Target joints dimension %zu doesn't match DOF 16",msg->position.size());
        return;
    }

    std::vector<double> rawWaistJoints = {msg->position[1], msg->position[0]}; // pitch, lift
    std::vector<double> rawArmJoints(msg->position.begin() + 2, msg->position.end());

    //检查IK的角度
    m_spJointLimitsManager->checkLimits(rawArmJoints,0.001);
    
    // Directly use raw target values without rate limiting
    m_targetWaistJoints = rawWaistJoints;
    m_targetArmJoints = rawArmJoints;
}

void ArmControlNode::checkWaistJointValid(const std::vector<double> &jointsWaist)
{
    if (jointsWaist.size() < 2)
    {
        throw std::runtime_error("Invalid waist joint size");
    }

    double fPitch = jointsWaist[0];
    double fLift = jointsWaist[1];

    if (fPitch < 0.429 || fPitch > 0.441)
    {
        RCLCPP_ERROR(get_logger(), "Waist pitch %.3f out of range [0.429, 0.441]", fPitch);
    }

    if (fLift < 0.29 || fLift > 0.481)
    {
        RCLCPP_ERROR(get_logger(), "Waist lift %.3f out of range [0.29, 0.481]", fLift);
    }
}

void ArmControlNode::publishJointCommands(const std::vector<double> &waistJoints,
                                          const std::vector<double> &armJoints)
{
    auto stamp = this->get_clock()->now();
    
    auto posArm = m_spJointLimitsManager->applyLimits(armJoints);
    auto armMsg = sensor_msgs::msg::JointState();
    armMsg.header.stamp = stamp;
    armMsg.position = posArm;
    m_armCommandPub->publish(armMsg);

    checkWaistJointValid(waistJoints);
    auto waistMsg = sensor_msgs::msg::JointState();
    waistMsg.header.stamp = stamp;
    waistMsg.position = waistJoints;
    m_waistCommandPub->publish(waistMsg);
}

void ArmControlNode::handle_arm_stream_control(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!msg->data) {
        // Data stream closed - immediately disable control loop for safety
        m_bEnableTimer = false;
        RCLCPP_WARN(get_logger(), "========================================");
        RCLCPP_WARN(get_logger(), "[SAFETY] ARM STREAM CLOSED - CONTROL DISABLED");
        RCLCPP_WARN(get_logger(), "[SAFETY] Control loop disabled for safety");
        RCLCPP_WARN(get_logger(), "[SAFETY] Robot will stop sending commands");
        RCLCPP_WARN(get_logger(), "[SAFETY] Press F2 again to re-enable control");
        RCLCPP_WARN(get_logger(), "========================================");
    } else {
        // Data stream opened
        RCLCPP_INFO(get_logger(), "========================================");
        RCLCPP_INFO(get_logger(), "[ARM STREAM] Stream opened");
        RCLCPP_INFO(get_logger(), "[ARM STREAM] Waiting for realign_trigger to enable control...");
        RCLCPP_INFO(get_logger(), "========================================");
    }
}

void ArmControlNode::resetRobot()
{
    RCLCPP_INFO(get_logger(), "========================================");
    RCLCPP_INFO(get_logger(), "[RESET] Robot reset sequence started");
    RCLCPP_INFO(get_logger(), "========================================");

    auto last_log_time = get_clock()->now();
    while (rclcpp::ok())
    {
        if (m_currentArmJoints.empty() || m_currentWaistJoints.empty())
        {
            auto now = this->get_clock()->now();
            if ((now - last_log_time).seconds() >= 5.0)
            {
                RCLCPP_WARN(get_logger(), "Still waiting for joint states from /hal/arm_joint_state and /hal/waist_state...");
                last_log_time = now;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        else
        {
            break;
        }
    }

    if (!rclcpp::ok())
    {
        RCLCPP_WARN(get_logger(), "ROS shutdown detected, aborting reset");
        return;
    }

    // Enable control loop immediately after hardware data is ready
    m_bEnableTimer = true;
    RCLCPP_INFO(get_logger(), "========================================");
    RCLCPP_INFO(get_logger(), "[RESET] Control loop ENABLED");
    RCLCPP_INFO(get_logger(), "[RESET] Starting robot reset to initial position...");
    RCLCPP_INFO(get_logger(), "========================================");

    std::array<double, 16> posInit{
        0.436, 0.4,
        1.1459065675735474, -0.7884084582328796, -0.5154555439949036, -1.3812546730041504, 0.7422706484794617, -1.1475818157196045, 0.17758865654468536,
        -1.209982991218567, 0.6525776386260986, 0.6103312373161316, 1.0545557737350464, -1.2333834171295166, 0.9266473650932312, 0.1652689427137375};
    
    std::array<double, 16> currentJoints;
    std::copy(m_currentWaistJoints.begin(), m_currentWaistJoints.end(), currentJoints.begin());
    std::copy(m_currentArmJoints.begin(), m_currentArmJoints.end(), currentJoints.begin() + 2);
    m_spRuckigController->setCurrentState(currentJoints);
    
    m_spRuckigController->setTarget(posInit);
    
    //  [pitch, lift]
    m_targetWaistJoints = {posInit[0], posInit[1]}; // pitch, lift
    m_targetArmJoints.assign(posInit.begin() + 2, posInit.end());

    bool isFinish = false;
    while (!isFinish && rclcpp::ok())
    {
        std::array<double, 16> currentPos, currentVel;
        isFinish = m_spRuckigController->update(currentPos, currentVel);

        std::vector<double> waistPos(currentPos.begin(), currentPos.begin() + 2);
        std::vector<double> armPos(currentPos.begin() + 2, currentPos.end());

        publishJointCommands(waistPos, armPos);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    m_targetWaistJoints = {posInit[0], posInit[1]};
    m_targetArmJoints.assign(posInit.begin() + 2, posInit.end());
  
    RCLCPP_INFO(get_logger(), "========================================");
    RCLCPP_INFO(get_logger(), "[RESET] Robot reset completed successfully");
    RCLCPP_INFO(get_logger(), "[RESET] Control loop is active and ready");
    RCLCPP_INFO(get_logger(), "========================================");
}


int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmControlNode>();
    
    // 使用多线程执行器以支持回调组
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    
    RCLCPP_INFO(node->get_logger(), "Starting arm control node...");
    RCLCPP_INFO(node->get_logger(), "Arm control node initialized - waiting for /arm_stream_realign_trigger (F2 to start)");

    // Run executor (resetRobot will be triggered by /arm_stream_realign_trigger when F2 is pressed)
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}