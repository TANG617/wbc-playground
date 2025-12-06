#include "wbc/ik_node.h"

#include <yaml-cpp/yaml.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <functional>
#include <algorithm>
#include <map>

namespace wbc
{

    IKWrapperNode::IKWrapperNode()
        : Node("ik_wrapper_node"), controller_active_(false), has_left_target_pose_(false), has_right_target_pose_(false), has_left_tracker_init_(false), has_right_tracker_init_(false), has_robot_init_poses_(false), last_computation_time_(0.0)
    {
        RCLCPP_INFO(this->get_logger(), "Initializing Dual-Arm WBC Controller Node...");

        // Initialize IK logger (controlled by enable_ik_logging_ flag)
        if (enable_ik_logging_)
        {
            ik_solve_logger_ = std::make_unique<IKSolveLogger>(this->get_logger(), 1000);
            if (!ik_solve_logger_->initialize("/var/psi/configuration"))
            {
                RCLCPP_WARN(this->get_logger(), "WARN, IK logger initialization failed, will run without logging");
                ik_solve_logger_.reset();
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "IK logging is disabled");
        }

        // Initialize parameters
        if (!initializeParameters())
        {
        RCLCPP_FATAL(this->get_logger(), "ERROR, Failed to initialize parameters");
        return;
    }
    
        // Initialize joint state
        initializeJointState();

        // Initialize ROS interfaces
        initializeROS();

        // Initialize WBC controller
        if (!initializeController())
        {
            RCLCPP_FATAL(this->get_logger(), "ERROR, Failed to initialize WBC controller");
        return;
    }
    
        // Start control loop
        auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / control_frequency_));
        control_timer_ = this->create_wall_timer(
            timer_period, std::bind(&IKWrapperNode::controlLoop, this));

        if (debug_mode_)
        {
            RCLCPP_INFO(this->get_logger(), "Debug mode enabled - waiting for PSI Glove tracker data");
        }

        controller_active_ = true;

        // Compute and log initial end-effector poses
        computeAndLogInitialEEPoses();

        // Initialize robot initial poses for VR tracker relative control
        initializeRobotInitialPoses();
        RCLCPP_INFO(this->get_logger(), "VR tracker relative control initialized");

        RCLCPP_INFO(this->get_logger(), "  Dual-Arm WBC Controller Node initialized successfully!");
        RCLCPP_INFO(this->get_logger(), "  Control frequency: %.1f Hz", control_frequency_);
        RCLCPP_INFO(this->get_logger(), "  Total joints: %zu", joint_names_.size());
        RCLCPP_INFO(this->get_logger(), "  Debug mode: %s", debug_mode_ ? "ON" : "OFF");
        RCLCPP_INFO(this->get_logger(), "  VR tracker mode: ON");
    }

    IKWrapperNode::~IKWrapperNode()
    {
        controller_active_ = false;
        RCLCPP_INFO(this->get_logger(), " Dual-Arm WBC Controller Node shutdown");
    }

    bool IKWrapperNode::initializeParameters()
    {
        try
        {
    // Declare and get parameters
    this->declare_parameter<std::string>("robot_urdf_path", 
        "a2d-tele/assets/A2D_NoHand/A2D_NoHand_Flattened.urdf");
            this->declare_parameter<std::string>("base_link", "base_link");
    this->declare_parameter<std::string>("left_end_effector", "Link7_l");
    this->declare_parameter<std::string>("right_end_effector", "Link7_r");
            this->declare_parameter<double>("control.frequency", 100.0);
            this->declare_parameter<bool>("debug_mode", true);
            this->declare_parameter<bool>("enable_debug_output", true);

            // Control parameters
            this->declare_parameter<int>("control.task_check_frequency", 200);
            this->declare_parameter<int>("control.target_log_frequency", 100);
            this->declare_parameter<int>("control.debug_log_frequency", 500);
            this->declare_parameter<int>("control.state_log_frequency", 100);
            this->declare_parameter<int>("control.solve_log_frequency", 200);

            // Workspace limits (used in validateWorkspaceLimits())
            this->declare_parameter<double>("workspace.x_min", -2.0);
            this->declare_parameter<double>("workspace.x_max", 2.0);
            this->declare_parameter<double>("workspace.y_min", -2.0);
            this->declare_parameter<double>("workspace.y_max", 2.0);
            this->declare_parameter<double>("workspace.z_min", 0.1);
            this->declare_parameter<double>("workspace.z_max", 1.5);

            robot_urdf_path_ = this->get_parameter("robot_urdf_path").as_string();
            base_link_ = this->get_parameter("base_link").as_string();
            left_end_effector_ = this->get_parameter("left_end_effector").as_string();
            right_end_effector_ = this->get_parameter("right_end_effector").as_string();
            control_frequency_ = this->get_parameter("control.frequency").as_double();

            // Read control parameters
            task_check_frequency_ = this->get_parameter("control.task_check_frequency").as_int();
            target_log_frequency_ = this->get_parameter("control.target_log_frequency").as_int();
            debug_log_frequency_ = this->get_parameter("control.debug_log_frequency").as_int();
            state_log_frequency_ = this->get_parameter("control.state_log_frequency").as_int();
            solve_log_frequency_ = this->get_parameter("control.solve_log_frequency").as_int();

            // Read IK solver parameters (from params_private.yaml)
            this->declare_parameter<double>("ik_solver.frame_task_weight", 8.0);
            this->declare_parameter<double>("ik_solver.kinetic_energy_weight", 0.1);
            frame_task_weight_ = this->get_parameter("ik_solver.frame_task_weight").as_double();
            kinetic_energy_weight_ = this->get_parameter("ik_solver.kinetic_energy_weight").as_double();
            RCLCPP_INFO(this->get_logger(), " IK Solver Params: FrameTaskWeight=%.3f, KineticEnergyWeight=%.3f", 
                        frame_task_weight_, kinetic_energy_weight_);

            // Read workspace limits (used in validateWorkspaceLimits())
            workspace_x_min_ = this->get_parameter("workspace.x_min").as_double();
            workspace_x_max_ = this->get_parameter("workspace.x_max").as_double();
            workspace_y_min_ = this->get_parameter("workspace.y_min").as_double();
            workspace_y_max_ = this->get_parameter("workspace.y_max").as_double();
            workspace_z_min_ = this->get_parameter("workspace.z_min").as_double();
            workspace_z_max_ = this->get_parameter("workspace.z_max").as_double();

            debug_mode_ = this->get_parameter("debug_mode").as_bool();
            enable_debug_output_ = this->get_parameter("enable_debug_output").as_bool();

            // Define complete joint list
    joint_names_ = {
                "joint_lift_body", "joint_body_pitch",                                              // Body joints
                "joint1_l", "joint2_l", "joint3_l", "joint4_l", "joint5_l", "joint6_l", "joint7_l", // Left arm
                "joint1_r", "joint2_r", "joint3_r", "joint4_r", "joint5_r", "joint6_r", "joint7_r"  // Right arm
            };

            // Joint limits (16 joints total)
            // 修正后的关节限制（基于URDF文件）
            joint_lower_limits_ = {0.399, 0.435, 
                                   -3.1399998, -1.9, -3.1399998, -1.7, -3.1399998, -1.7, -3.13,
                                   -3.1399998, -1.9, -3.1399998, -1.7, -3.1399998, -1.7, -3.13};
            joint_upper_limits_ = {0.401, 0.437, 
                                   3.1399998, 1.8, 3.1399998, 1.7, 3.1399998, 1.7, 3.13,
                                   3.1399998, 1.8, 3.1399998, 1.7, 3.1399998, 1.7, 3.13};
            joint_velocity_limits_ = {0.5, 3.14, 3.14, 3.14, 3.14, 3.14, 3.14, 3.14, 3.14,
                                      3.14, 3.14, 3.14, 3.14, 3.14, 3.14, 3.14};

            // Try to read joint limits from config file (override hardcoded values)
            try
            {
                // Declare joint limit parameters
                this->declare_parameter<std::vector<double>>("robot.joint_limits.lower", std::vector<double>());
                this->declare_parameter<std::vector<double>>("robot.joint_limits.upper", std::vector<double>());
                this->declare_parameter<std::vector<double>>("robot.joint_limits.velocity", std::vector<double>());

                // Read joint limits from config
                auto config_lower_limits = this->get_parameter("robot.joint_limits.lower").as_double_array();
                auto config_upper_limits = this->get_parameter("robot.joint_limits.upper").as_double_array();
                auto config_velocity_limits = this->get_parameter("robot.joint_limits.velocity").as_double_array();

                // Apply config limits if provided and size matches
                if (config_lower_limits.size() == joint_names_.size())
                {
                    joint_lower_limits_ = config_lower_limits;
                    RCLCPP_INFO(this->get_logger(), " Loaded %zu lower joint limits from config", config_lower_limits.size());
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "ERROR, Config lower limits size (%zu) != joint count (%zu), using defaults",
                                config_lower_limits.size(), joint_names_.size());
                }

                if (config_upper_limits.size() == joint_names_.size())
                {
                    joint_upper_limits_ = config_upper_limits;
                    RCLCPP_INFO(this->get_logger(), "Loaded %zu upper joint limits from config", config_upper_limits.size());
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "WARN, Config upper limits size (%zu) != joint count (%zu), using defaults",
                                config_upper_limits.size(), joint_names_.size());
                }

                if (config_velocity_limits.size() == joint_names_.size())
                {
                    joint_velocity_limits_ = config_velocity_limits;
                    RCLCPP_INFO(this->get_logger(), "Loaded %zu velocity joint limits from config", config_velocity_limits.size());
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "WARN, Config velocity limits size (%zu) != joint count (%zu), using defaults",
                                config_velocity_limits.size(), joint_names_.size());
                }

                // Debug: Print some limits
                RCLCPP_INFO(this->get_logger(), "🔧 Joint limits loaded:");
                for (size_t i = 0; i < std::min(size_t(5), joint_names_.size()); ++i)
                {
                    RCLCPP_INFO(this->get_logger(), "   %s: [%.3f, %.3f] vel:%.3f",
                                joint_names_[i].c_str(), joint_lower_limits_[i], joint_upper_limits_[i], joint_velocity_limits_[i]);
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_WARN(this->get_logger(), "WARN, Failed to read joint limits from config: %s. Using hardcoded defaults.", e.what());
            }

            // Load initial joint positions from config
            initial_joint_positions_.resize(joint_names_.size(), 0.0); // Default to zeros

            // Try to read initial positions from config file
            for (size_t i = 0; i < joint_names_.size(); ++i)
            {
                std::string param_name = "robot.initial_position." + joint_names_[i];

                // Declare parameter with default value 0.0
                this->declare_parameter<double>(param_name, 0.0);

                try
                {
                    initial_joint_positions_[i] = this->get_parameter(param_name).as_double();
                    RCLCPP_DEBUG(this->get_logger(), "Debug, %s: %.3f",
                                 joint_names_[i].c_str(), initial_joint_positions_[i]);
                }
                catch (const std::exception &e)
                {
                    RCLCPP_WARN(this->get_logger(), "WARN,Failed to read initial position for %s, using 0.0: %s",
                                joint_names_[i].c_str(), e.what());
                    initial_joint_positions_[i] = 0.0;
                }
            }

            // VR coordinate transformation parameters
            // Use try-catch to read nested parameters from config file
            try
            {
                RCLCPP_INFO(this->get_logger(), "Reading VR coordinate parameters from config file");

                // Declare all required parameters first to avoid crashes
                this->declare_parameter<std::vector<int64_t>>("vr_coordinate_transform.left_hand.position_axis_mapping.x", {0, 1});
                this->declare_parameter<std::vector<int64_t>>("vr_coordinate_transform.left_hand.position_axis_mapping.y", {2, -1});
                this->declare_parameter<std::vector<int64_t>>("vr_coordinate_transform.left_hand.position_axis_mapping.z", {1, -1});
                this->declare_parameter<std::vector<int64_t>>("vr_coordinate_transform.left_hand.orientation_axis_mapping.x", {0, 1});
                this->declare_parameter<std::vector<int64_t>>("vr_coordinate_transform.left_hand.orientation_axis_mapping.y", {2, -1});
                this->declare_parameter<std::vector<int64_t>>("vr_coordinate_transform.left_hand.orientation_axis_mapping.z", {1, -1});

                this->declare_parameter<std::vector<int64_t>>("vr_coordinate_transform.right_hand.position_axis_mapping.x", {0, 1});
                this->declare_parameter<std::vector<int64_t>>("vr_coordinate_transform.right_hand.position_axis_mapping.y", {2, 1});
                this->declare_parameter<std::vector<int64_t>>("vr_coordinate_transform.right_hand.position_axis_mapping.z", {1, -1});
                this->declare_parameter<std::vector<int64_t>>("vr_coordinate_transform.right_hand.orientation_axis_mapping.x", {0, 1});
                this->declare_parameter<std::vector<int64_t>>("vr_coordinate_transform.right_hand.orientation_axis_mapping.y", {2, 1});
                this->declare_parameter<std::vector<int64_t>>("vr_coordinate_transform.right_hand.orientation_axis_mapping.z", {1, -1});

                // NEW: Read separate left and right hand configurations
                // Left hand configuration
                auto left_pos_x = this->get_parameter("vr_coordinate_transform.left_hand.position_axis_mapping.x").as_integer_array();
                auto left_pos_y = this->get_parameter("vr_coordinate_transform.left_hand.position_axis_mapping.y").as_integer_array();
                auto left_pos_z = this->get_parameter("vr_coordinate_transform.left_hand.position_axis_mapping.z").as_integer_array();
                auto left_ori_x = this->get_parameter("vr_coordinate_transform.left_hand.orientation_axis_mapping.x").as_integer_array();
                auto left_ori_y = this->get_parameter("vr_coordinate_transform.left_hand.orientation_axis_mapping.y").as_integer_array();
                auto left_ori_z = this->get_parameter("vr_coordinate_transform.left_hand.orientation_axis_mapping.z").as_integer_array();

                // Right hand configuration
                auto right_pos_x = this->get_parameter("vr_coordinate_transform.right_hand.position_axis_mapping.x").as_integer_array();
                auto right_pos_y = this->get_parameter("vr_coordinate_transform.right_hand.position_axis_mapping.y").as_integer_array();
                auto right_pos_z = this->get_parameter("vr_coordinate_transform.right_hand.position_axis_mapping.z").as_integer_array();
                auto right_ori_x = this->get_parameter("vr_coordinate_transform.right_hand.orientation_axis_mapping.x").as_integer_array();
                auto right_ori_y = this->get_parameter("vr_coordinate_transform.right_hand.orientation_axis_mapping.y").as_integer_array();
                auto right_ori_z = this->get_parameter("vr_coordinate_transform.right_hand.orientation_axis_mapping.z").as_integer_array();

                // Parse left hand mappings
                std::vector<std::vector<int64_t>> left_pos_mappings = {left_pos_x, left_pos_y, left_pos_z};
                for (const auto &axis_config : left_pos_mappings)
                {
                    if (axis_config.size() >= 2)
                    {
                        int axis = static_cast<int>(axis_config[0]);
                        int sign = static_cast<int>(axis_config[1]);
                        left_vr_position_axis_mapping_.push_back({axis, sign});
                    }
                }

                std::vector<std::vector<int64_t>> left_ori_mappings = {left_ori_x, left_ori_y, left_ori_z};
                for (const auto &axis_config : left_ori_mappings)
                {
                    if (axis_config.size() >= 2)
                    {
                        int axis = static_cast<int>(axis_config[0]);
                        int sign = static_cast<int>(axis_config[1]);
                        left_vr_orientation_axis_mapping_.push_back({axis, sign});
                    }
                }

                // Parse right hand mappings
                std::vector<std::vector<int64_t>> right_pos_mappings = {right_pos_x, right_pos_y, right_pos_z};
                for (const auto &axis_config : right_pos_mappings)
                {
                    if (axis_config.size() >= 2)
                    {
                        int axis = static_cast<int>(axis_config[0]);
                        int sign = static_cast<int>(axis_config[1]);
                        right_vr_position_axis_mapping_.push_back({axis, sign});
                    }
                }

                std::vector<std::vector<int64_t>> right_ori_mappings = {right_ori_x, right_ori_y, right_ori_z};
                for (const auto &axis_config : right_ori_mappings)
                {
                    if (axis_config.size() >= 2)
                    {
                        int axis = static_cast<int>(axis_config[0]);
                        int sign = static_cast<int>(axis_config[1]);
                        right_vr_orientation_axis_mapping_.push_back({axis, sign});
                    }
                }

                // Keep old unified mapping for backward compatibility (use left hand as default)
                vr_position_axis_mapping_ = left_vr_position_axis_mapping_;
                vr_orientation_axis_mapping_ = left_vr_orientation_axis_mapping_;

                // Debug: Print configuration that was loaded
                RCLCPP_INFO(this->get_logger(), "Loaded VR coordinate mapping from config file");
            }
            catch (const std::exception &e)
            {
                RCLCPP_WARN(this->get_logger(), "WARN, Failed to read VR coordinate parameters from config: %s", e.what());
                RCLCPP_INFO(this->get_logger(), "Using default coordinate mapping (no transformation)");

                // Use default mapping: no transformation
                vr_position_axis_mapping_ = {{0, 1}, {1, 1}, {2, 1}};
                vr_orientation_axis_mapping_ = {{0, 1}, {1, 1}, {2, 1}};
            }

            // Validate mappings
            if (vr_position_axis_mapping_.size() != 3 || vr_orientation_axis_mapping_.size() != 3)
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid VR coordinate mapping: need exactly 3 axis mappings");
                return false;
            }

            RCLCPP_INFO(this->get_logger(), "VR coordinate transform - Position: [[%d,%d],[%d,%d],[%d,%d]], Orientation: [[%d,%d],[%d,%d],[%d,%d]]",
                        vr_position_axis_mapping_[0].first, vr_position_axis_mapping_[0].second,
                        vr_position_axis_mapping_[1].first, vr_position_axis_mapping_[1].second,
                        vr_position_axis_mapping_[2].first, vr_position_axis_mapping_[2].second,
                        vr_orientation_axis_mapping_[0].first, vr_orientation_axis_mapping_[0].second,
                        vr_orientation_axis_mapping_[1].first, vr_orientation_axis_mapping_[1].second,
                        vr_orientation_axis_mapping_[2].first, vr_orientation_axis_mapping_[2].second);

            RCLCPP_INFO(this->get_logger(), "Parameters initialized successfully");
            RCLCPP_INFO(this->get_logger(), "Initial joint positions loaded from config");
            return true;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "ERROR, Failed to initialize parameters: %s", e.what());
            return false;
        }
    }

    bool IKWrapperNode::initializeController()
    {
        try
        {
            // 使用 RobotKinematicsTool 替代 DualArmIKFKTool
            wbc_controller_ = std::make_unique<RobotKinematicsTool>();

            // 初始化控制器（传递IK solver参数）
            if (!wbc_controller_->robotKinematicsInit(robot_urdf_path_,
                                                       left_end_effector_,
                                                       right_end_effector_,
                                                       true, // enableSelfCollision
                                                       frame_task_weight_,
                                                       kinetic_energy_weight_))
            {
                RCLCPP_ERROR(this->get_logger(), "ERROR, Failed to initialize WBC controller");
                return false;
            }

            // 设置关节限制（分别调用）
            if (!wbc_controller_->setJointPositionLimits(joint_lower_limits_, joint_upper_limits_))
            {
                RCLCPP_ERROR(this->get_logger(), "ERROR, Failed to set joint position limits");
                return false;
            }
            
            if (!wbc_controller_->setJointVelocityLimits(joint_velocity_limits_))
            {
                RCLCPP_ERROR(this->get_logger(), "ERROR, Failed to set joint velocity limits");
                return false;
            }

            // 配置 IK 求解器参数（可选，使用默认值也可以）
            wbc_controller_->configureIKSolver(
                200,      // maxIterations
                5e-6,     // precision
                0.02,     // selfCollisionDistance
                1.2       // regularizationWeight
            );

            // 设置初始关节位置
            if (!wbc_controller_->setInitialJointPositions(initial_joint_positions_))
            {
                RCLCPP_ERROR(this->get_logger(), "ERROR, Failed to set initial joint positions");
                return false;
            }

            RCLCPP_INFO(this->get_logger(), "WBC controller initialized with initial joint positions");
            return true;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "ERROR, Exception initializing controller: %s", e.what());
            return false;
        }
    }

    void IKWrapperNode::initializeROS()
    {
        // Publishers
       // joint_command_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        //    "/joint_states", 100);

        ///model/pinocchio/target_joints

        joint_command_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/tele/pinocchio/target_joints", 100);


        wbc_state_pub_ = this->create_publisher<wbc::msg::WBCState>(
            "/wbc/state", 100);

        // ty add
        // 创建arm ik数据发布器，用于data_collection收集ik数据,14个关节 2025-10-11
        ik_arm_data_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/tele/mink_ik/target_arm_joints", 100);

        // 创建腰部ik数据发布器，用于data_collection收集ik数据,2个关节 2025-10-11
        ik_waist_data_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/mink_ik/target_waist_joints", 100);

        // ty add end

        // TF broadcaster for target poses
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // VR tracker subscribers - we only use VR trackers now
        RCLCPP_INFO(this->get_logger(), "🥽 Using VR tracker input from /hal/tracker/htc/left/pose and /hal/tracker/htc/right/pose");

        left_vr_tracker_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/hal/tracker/htc/left/pose", 100,
            std::bind(&IKWrapperNode::leftVRTrackerPoseCallback, this, std::placeholders::_1));

        right_vr_tracker_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/hal/tracker/htc/right/pose", 100,
            std::bind(&IKWrapperNode::rightVRTrackerPoseCallback, this, std::placeholders::_1));

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states_input", 100,
            std::bind(&IKWrapperNode::jointStateCallback, this, std::placeholders::_1));

        // Subscribe to arm stream realign trigger for resetting tracker initialization flags (F2 only)
        realign_trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/arm_stream_realign_trigger",
            rclcpp::QoS(10),
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data) {
                    RCLCPP_INFO(this->get_logger(), 
                        "[F2] Arm stream realign trigger received - Resetting tracker initialization flags");
                    has_left_tracker_init_ = false;
                    has_right_tracker_init_ = false;
                }
            });

        RCLCPP_INFO(this->get_logger(), "ROS interfaces initialized");
    }

    void IKWrapperNode::initializeJointState()
    {
        current_joint_state_.name = joint_names_;
        current_joint_state_.position.resize(joint_names_.size(), 0.0);
        current_joint_state_.velocity.resize(joint_names_.size(), 0.0);
        current_joint_state_.effort.resize(joint_names_.size(), 0.0);

        // Set initial positions from config file
        if (initial_joint_positions_.size() == joint_names_.size())
        {
            current_joint_state_.position = initial_joint_positions_;

            RCLCPP_INFO(this->get_logger(), "Joint state initialized with configured home pose:");
            for (size_t i = 0; i < joint_names_.size(); ++i)
            {
                RCLCPP_INFO(this->get_logger(), "   🔧 %s: %.3f rad",
                            joint_names_[i].c_str(), current_joint_state_.position[i]);
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "WARN, Initial positions size mismatch, using zeros");
            std::fill(current_joint_state_.position.begin(), current_joint_state_.position.end(), 0.0);
        }
    }

    void IKWrapperNode::controlLoop()
    {
        if (!controller_active_)
        {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                  "Controller not active");
            return;
        }

        if (!wbc_controller_->isInitialized())
        {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                  "WBC controller not initialized");
            return;
        }

        std::lock_guard<std::mutex> lock(state_mutex_);

        // Always publish current joint state for visualization
        current_joint_state_.header.stamp = this->get_clock()->now();
        current_joint_state_.header.frame_id = base_link_;
        joint_command_pub_->publish(current_joint_state_);

        // ty add
        // 取current_joint_state的后14个关节然后发布到/mink_ik/target_arm_joints 2025-10-11
        sensor_msgs::msg::JointState ik_data;
        ik_data.header.stamp = this->get_clock()->now();
        ik_data.header.frame_id = base_link_;
        // 取joint_names_的最后14个关节名称
        ik_data.name = std::vector<std::string>(joint_names_.end() - 14, joint_names_.end());
        ik_data.position = std::vector<double>(current_joint_state_.position.begin() + 2, current_joint_state_.position.end());
        ik_arm_data_pub_->publish(ik_data);

        // 取current_joint_state的前2个关节然后发布到/mink_ik/target_waist_joints 2025-10-11
        ik_data.header.stamp = this->get_clock()->now();
        ik_data.header.frame_id = base_link_;
        // Python版本顺序：[joint_body_pitch, joint_lift_body]
        ik_data.name = {
            joint_names_[1], // "joint_body_pitch"
            joint_names_[0]  // "joint_lift_body"
        };
        ik_data.position = {
            current_joint_state_.position[1], // body_pitch值
            current_joint_state_.position[0]  // lift_body值
        };

        ik_waist_data_pub_->publish(ik_data);
        // ty add end

        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                              "Published joint state: lift_body=%.3f, body_pitch=%.3f",
                              current_joint_state_.position[0], current_joint_state_.position[1]);

        // Update dual-arm control if we have target poses
        if (has_left_target_pose_ && has_right_target_pose_)
        {
            updateDualArmControl();
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                  "Dual-arm control active: L[%.3f,%.3f,%.3f] R[%.3f,%.3f,%.3f]",
                                  left_target_pose_.pose.position.x, left_target_pose_.pose.position.y, left_target_pose_.pose.position.z,
                                  right_target_pose_.pose.position.x, right_target_pose_.pose.position.y, right_target_pose_.pose.position.z);
        }
        else
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                 "⏳ Waiting for tracker data: left=%s, right=%s, robot_init=%s",
                                 has_left_target_pose_ ? "✅" : "❌",
                                 has_right_target_pose_ ? "✅" : "❌",
                                 has_robot_init_poses_ ? "✅" : "❌");
        }

        // Publish WBC state
        current_wbc_state_.header.stamp = this->get_clock()->now();
        current_wbc_state_.is_initialized = wbc_controller_->isInitialized();
        current_wbc_state_.computation_time = last_computation_time_;
        wbc_state_pub_->publish(current_wbc_state_);

        // Publish debug markers if enabled
        if (debug_mode_ && enable_debug_output_)
        {
            publishDebugMarkers();
        }

        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                              "Control loop: L=%s R=%s joints=%zu",
                              has_left_target_pose_ ? "✓" : "✗",
                              has_right_target_pose_ ? "✓" : "✗",
                              current_joint_state_.position.size());
    }

    void IKWrapperNode::leftVRTrackerPoseCallback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {

        std::lock_guard<std::mutex> lock(state_mutex_);

        // Transform VR coordinates to Robot coordinates
        geometry_msgs::msg::PoseStamped robot_coord_pose = transformVRToRobotCoordinates(*msg, "left");

        // Initialize tracker pose on first detection (using robot coordinates)
        if (!has_left_tracker_init_)
        {
            left_tracker_init_pose_ = robot_coord_pose;
            left_tracker_init_pose_.header.frame_id = base_link_; // Use consistent frame
            has_left_tracker_init_ = true;
            return; // Skip first frame, just initialize
        }

        // Compute relative target using SE3 transformations (using robot coordinates)
        if (has_robot_init_poses_)
        {
            left_target_pose_ = computeRelativeTargetPose(robot_coord_pose, left_tracker_init_pose_, left_robot_init_ee_pose_, "left");
            has_left_target_pose_ = true;

            // Publish target pose as TF for visualization
            publishTargetPoseTF(left_target_pose_, "left_target");
        }
        else
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                 "Robot initial poses not ready, skipping VR control");
        }
    }

    void IKWrapperNode::rightVRTrackerPoseCallback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {

        std::lock_guard<std::mutex> lock(state_mutex_);

        // Transform VR coordinates to Robot coordinates
        geometry_msgs::msg::PoseStamped robot_coord_pose = transformVRToRobotCoordinates(*msg, "right");

        // Initialize tracker pose on first detection (using robot coordinates)
        if (!has_right_tracker_init_)
        {
            right_tracker_init_pose_ = robot_coord_pose;
            right_tracker_init_pose_.header.frame_id = base_link_; // Use consistent frame
            has_right_tracker_init_ = true;

            return; // Skip first frame, just initialize
        }

        // Compute relative target using SE3 transformations (using robot coordinates)
        if (has_robot_init_poses_)
        {
            right_target_pose_ = computeRelativeTargetPose(robot_coord_pose, right_tracker_init_pose_, right_robot_init_ee_pose_, "right");
            has_right_target_pose_ = true;

            // Publish target pose as TF for visualization
            publishTargetPoseTF(right_target_pose_, "right_target");
        }
        else
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                 "Robot initial poses not ready, skipping VR control");
        }
    }

    void IKWrapperNode::jointStateCallback(
        const sensor_msgs::msg::JointState::SharedPtr msg)
    {

        return;
        std::lock_guard<std::mutex> lock(state_mutex_);

        // Update joints from external source (if needed)
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            auto it = std::find(current_joint_state_.name.begin(),
                                current_joint_state_.name.end(),
                                msg->name[i]);
            if (it != current_joint_state_.name.end())
            {
                size_t idx = std::distance(current_joint_state_.name.begin(), it);
                if (idx < current_joint_state_.position.size() && i < msg->position.size())
                {
                    current_joint_state_.position[idx] = msg->position[i];
                }
            }
        }
    }

    void IKWrapperNode::updateDualArmControl()
    {
        try
        {
            // Validate workspace limits
            if (!validateWorkspaceLimits(left_target_pose_, "left") ||
                !validateWorkspaceLimits(right_target_pose_, "right"))
            {
                return;
            }

            // 使用 RobotKinematicsTool 的 ikSolve 方法
            sensor_msgs::msg::JointState outputJointState;
            double solveTime;
            
            
            bool success = wbc_controller_->ikSolve(
                current_joint_state_,
                left_target_pose_,
                right_target_pose_,
                outputJointState,
                solveTime,
                1.0 / control_frequency_); // dt

            if (success)
            {
                // Log successful IK solve (non-blocking, asynchronous)
                /*if (ik_solve_logger_)
                {
                    // Get high-precision timestamp (microseconds precision, 0.0001s)
                    double timestamp_us = this->now().nanoseconds();
                    ik_solve_logger_->logIKSolve(
                        timestamp_us,
                        current_joint_state_,
                        left_target_pose_,
                        right_target_pose_,
                        outputJointState,
                        solveTime,
                        1.0 / control_frequency_,
                        true);  // success = true
                }*/

                // Update current joint state with new commands
                if (outputJointState.position.size() == current_joint_state_.position.size())
                {
                    current_joint_state_.position = outputJointState.position;
                    current_joint_state_.velocity = outputJointState.velocity;
                }
                
                // 更新 WBC 状态的计算时间（转换为秒）
                last_computation_time_ = solveTime / 1000.0;
            }
            else
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                    "WARN, IK solve failed, keeping current joint state");
            }

            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                  "Dual-arm WBC updated successfully");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                  "ERROR, Exception in dual-arm control: %s", e.what());
        }
    }

    bool IKWrapperNode::validateWorkspaceLimits(
        const geometry_msgs::msg::PoseStamped &pose,
        const std::string &workspace_name)
    {

        // Workspace validation using configurable limits
        bool valid = (pose.pose.position.x >= workspace_x_min_ && pose.pose.position.x <= workspace_x_max_ &&
                      pose.pose.position.y >= workspace_y_min_ && pose.pose.position.y <= workspace_y_max_ &&
                      pose.pose.position.z >= workspace_z_min_ && pose.pose.position.z <= workspace_z_max_);

        if (!valid)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                 "WARN, %s target outside workspace: [%.3f, %.3f, %.3f]",
                                 workspace_name.c_str(),
                                 pose.pose.position.x,
                                 pose.pose.position.y,
                                 pose.pose.position.z);
        }

        return valid;
    }

    void IKWrapperNode::publishDebugMarkers()
    {
        // TODO: Implement debug visualization markers
        // This would show target poses, current end-effector positions, etc.
    }

    void IKWrapperNode::computeAndLogInitialEEPoses()
    {
        if (!wbc_controller_->isInitialized())
        {
            RCLCPP_WARN(this->get_logger(), "WARN, WBC controller not initialized, cannot compute EE poses");
            return;
        }

        try
        {
            geometry_msgs::msg::Pose left_pose, right_pose;

            // Use initial joint positions (home configuration)
            std::vector<double> home_joints(initial_joint_positions_.begin(), initial_joint_positions_.end());

            double solveTime;
            if (wbc_controller_->fkSolve(home_joints, left_pose, right_pose, solveTime))
            {
                RCLCPP_INFO(this->get_logger(), "Initial End-Effector Poses (Home Configuration):");
                RCLCPP_INFO(this->get_logger(), "   Left EE:  [%.3f, %.3f, %.3f] | Q[%.3f, %.3f, %.3f, %.3f]",
                            left_pose.position.x, left_pose.position.y, left_pose.position.z,
                            left_pose.orientation.w, left_pose.orientation.x,
                            left_pose.orientation.y, left_pose.orientation.z);
                RCLCPP_INFO(this->get_logger(), "   Right EE: [%.3f, %.3f, %.3f] | Q[%.3f, %.3f, %.3f, %.3f]",
                            right_pose.position.x, right_pose.position.y, right_pose.position.z,
                            right_pose.orientation.w, right_pose.orientation.x,
                            right_pose.orientation.y, right_pose.orientation.z);

                // Store for debug tracker use
                initial_left_ee_pose_ = left_pose;
                initial_right_ee_pose_ = right_pose;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "ERROR, Failed to compute initial end-effector poses");
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "ERROR, Exception computing initial EE poses: %s", e.what());
        }
    }

    void IKWrapperNode::initializeRobotInitialPoses()
    {
        RCLCPP_INFO(this->get_logger(), "Starting robot initial poses initialization...");

        if (!wbc_controller_)
        {
            RCLCPP_ERROR(this->get_logger(), "ERROR, WBC controller not initialized, cannot get initial poses");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "WBC controller is initialized");

        // Get current joint positions (home configuration)
        std::vector<double> home_joints(joint_names_.size(), 0.0);
        for (size_t i = 0; i < initial_joint_positions_.size() && i < home_joints.size(); ++i)
        {
            home_joints[i] = initial_joint_positions_[i];
        }

        RCLCPP_INFO(this->get_logger(), "Using %zu home joint positions", home_joints.size());
        for (size_t i = 0; i < std::min(size_t(5), home_joints.size()); ++i)
        {
            RCLCPP_INFO(this->get_logger(), "   Joint[%zu]: %.3f", i, home_joints[i]);
        }

        geometry_msgs::msg::Pose left_pose, right_pose;
        RCLCPP_INFO(this->get_logger(), "Computing initial end-effector poses...");

        double solveTime;
        if (wbc_controller_->fkSolve(home_joints, left_pose, right_pose, solveTime))
        {
            // Convert to PoseStamped and store as initial poses
            left_robot_init_ee_pose_.header.frame_id = base_link_;
            left_robot_init_ee_pose_.header.stamp = this->now();
            left_robot_init_ee_pose_.pose = left_pose;

            right_robot_init_ee_pose_.header.frame_id = base_link_;
            right_robot_init_ee_pose_.header.stamp = this->now();
            right_robot_init_ee_pose_.pose = right_pose;

            has_robot_init_poses_ = true;

            RCLCPP_INFO(this->get_logger(), "Robot initial EE poses saved for relative control:");
            RCLCPP_INFO(this->get_logger(), "   Left EE init:  [%.3f, %.3f, %.3f]",
                        left_pose.position.x, left_pose.position.y, left_pose.position.z);
            RCLCPP_INFO(this->get_logger(), "   Right EE init: [%.3f, %.3f, %.3f]",
                        right_pose.position.x, right_pose.position.y, right_pose.position.z);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "ERROR, Failed to get initial end-effector poses from WBC controller");
            has_robot_init_poses_ = false;
        }
    }

    Eigen::Affine3d IKWrapperNode::poseToAffine3d(const geometry_msgs::msg::Pose &pose)
    {
        // Convert position
        Eigen::Vector3d translation(pose.position.x, pose.position.y, pose.position.z);

        // Convert orientation (quaternion to rotation matrix)
        Eigen::Quaterniond quat(pose.orientation.w, pose.orientation.x,
                                pose.orientation.y, pose.orientation.z);
        quat.normalize(); // Ensure unit quaternion

        Eigen::Affine3d transform = Eigen::Affine3d::Identity();
        transform.translation() = translation;
        transform.linear() = quat.toRotationMatrix();

        return transform;
    }

    geometry_msgs::msg::Pose IKWrapperNode::affine3dToPose(const Eigen::Affine3d &transform, const std::string &tracker_id)
    {
        (void)tracker_id; // Suppress unused parameter warning
        geometry_msgs::msg::Pose pose;

        // Convert position
        pose.position.x = transform.translation()(0);
        pose.position.y = transform.translation()(1);
        pose.position.z = transform.translation()(2);

        // Convert orientation (rotation matrix to quaternion)
        Eigen::Quaterniond quat(transform.rotation());
        quat.normalize(); // Ensure unit quaternion

        pose.orientation.w = quat.w();
        pose.orientation.x = quat.x();
        pose.orientation.y = quat.y();
        pose.orientation.z = quat.z();

        return pose;
    }

    geometry_msgs::msg::PoseStamped IKWrapperNode::computeRelativeTargetPose(
        const geometry_msgs::msg::PoseStamped &current_tracker_pose,
        const geometry_msgs::msg::PoseStamped &init_tracker_pose,
        const geometry_msgs::msg::PoseStamped &init_robot_ee_pose,
        const std::string &tracker_id)
    {

        // Convert to SE3 for proper 6D transformations
        Eigen::Affine3d T_tracker_current = poseToAffine3d(current_tracker_pose.pose);
        Eigen::Affine3d T_tracker_init = poseToAffine3d(init_tracker_pose.pose);
        Eigen::Affine3d T_robot_init = poseToAffine3d(init_robot_ee_pose.pose);

        // 简单的差值计算：target = robot_init + (vive_current - vive_init)

        // 计算vive的变化：current - init
        Eigen::Affine3d T_vive_delta = T_tracker_init.inverse() * T_tracker_current;

        // 将vive的变化应用到robot初始pose上：robot_init * vive_delta
        Eigen::Affine3d T_target = T_robot_init * T_vive_delta;

        // Convert back to PoseStamped
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = base_link_;
        target_pose.header.stamp = current_tracker_pose.header.stamp;
        target_pose.pose = affine3dToPose(T_target, tracker_id);

        // Simple debug output for delta transform
        Eigen::Vector3d vive_delta_pos = T_vive_delta.translation();
        Eigen::Quaterniond vive_delta_quat(T_vive_delta.rotation());

        Eigen::Vector3d robot_target_pos = T_target.translation();

        return target_pose;
    }

    void IKWrapperNode::publishTargetPoseTF(const geometry_msgs::msg::PoseStamped &pose, const std::string &frame_name)
    {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = pose.header.stamp; // Use the pose's timestamp
        tf_msg.header.frame_id = base_link_;     // Use consistent parent frame
        tf_msg.child_frame_id = frame_name;

        // Copy pose to transform
        tf_msg.transform.translation.x = pose.pose.position.x;
        tf_msg.transform.translation.y = pose.pose.position.y;
        tf_msg.transform.translation.z = pose.pose.position.z;
        tf_msg.transform.rotation = pose.pose.orientation;

        // Send transform
        tf_broadcaster_->sendTransform(tf_msg);
    }

    geometry_msgs::msg::PoseStamped IKWrapperNode::transformVRToRobotCoordinates(
        const geometry_msgs::msg::PoseStamped &vr_pose, const std::string &hand_side)
    {

        geometry_msgs::msg::PoseStamped robot_pose = vr_pose; // Copy header and basic structure

        // Choose the correct mapping based on hand_side
        std::vector<std::pair<int, int>> *pos_mapping;
        std::vector<std::pair<int, int>> *ori_mapping;

        if (hand_side == "left")
        {
            pos_mapping = &left_vr_position_axis_mapping_;
            ori_mapping = &left_vr_orientation_axis_mapping_;
        }
        else if (hand_side == "right")
        {
            pos_mapping = &right_vr_position_axis_mapping_;
            ori_mapping = &right_vr_orientation_axis_mapping_;
        }
        else
        {
            // Default to unified mapping for backward compatibility
            pos_mapping = &vr_position_axis_mapping_;
            ori_mapping = &vr_orientation_axis_mapping_;
        }

        // Get VR position as array for easier indexing
        std::array<double, 3> vr_pos = {vr_pose.pose.position.x, vr_pose.pose.position.y, vr_pose.pose.position.z};

        // Transform position according to selected mapping
        // Robot[i] = sign * VR[source_axis]
        robot_pose.pose.position.x = (*pos_mapping)[0].second * vr_pos[(*pos_mapping)[0].first];
        robot_pose.pose.position.y = (*pos_mapping)[1].second * vr_pos[(*pos_mapping)[1].first];
        robot_pose.pose.position.z = (*pos_mapping)[2].second * vr_pos[(*pos_mapping)[2].first];

        // Transform orientation using rotation matrix transformation
        // Convert quaternion to rotation matrix
        Eigen::Quaterniond vr_quat(vr_pose.pose.orientation.w,
                                   vr_pose.pose.orientation.x,
                                   vr_pose.pose.orientation.y,
                                   vr_pose.pose.orientation.z);
        Eigen::Matrix3d vr_rotation = vr_quat.toRotationMatrix();

        // Create complete axis transformation matrix
        // This matrix will reorder and flip axes as specified
        Eigen::Matrix3d axis_transform = Eigen::Matrix3d::Zero();

        // Fill transformation matrix based on selected mapping
        // Row i of axis_transform represents how Robot axis i is computed from VR axes
        for (int robot_axis = 0; robot_axis < 3; ++robot_axis)
        {
            int vr_source_axis = (*ori_mapping)[robot_axis].first;
            int sign = (*ori_mapping)[robot_axis].second;
            axis_transform(robot_axis, vr_source_axis) = sign;
        }

        // Apply transformation: R_robot = axis_transform * R_vr * axis_transform^T
        // This handles both axis reordering and sign flipping for rotations
        Eigen::Matrix3d robot_rotation = axis_transform * vr_rotation * axis_transform.transpose();

        // Convert back to quaternion
        Eigen::Quaterniond robot_quat(robot_rotation);
        robot_quat.normalize();

        // Set transformed orientation
        robot_pose.pose.orientation.w = robot_quat.w();
        robot_pose.pose.orientation.x = robot_quat.x();
        robot_pose.pose.orientation.y = robot_quat.y();
        robot_pose.pose.orientation.z = robot_quat.z();

        return robot_pose;
}

} // namespace wbc

// Main function
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<wbc::IKWrapperNode>();
    rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "❌ Fatal error: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}