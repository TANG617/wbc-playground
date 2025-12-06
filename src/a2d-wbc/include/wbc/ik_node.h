#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "wbc/robot_kinematics_tool.h"
#include "wbc/ik_solve_logger.h"
#include "wbc/msg/wbc_command.hpp"
#include "wbc/msg/wbc_state.hpp"

#include <mutex>
#include <memory>
#include <Eigen/Dense>

namespace wbc
{

    /**
     * @brief ROS2 Node for Dual-Arm Whole Body Control
     *
     * Integrates dual-arm WBC with PSI Glove tracker data for simultaneous
     * bilateral teleoperation control. Publishes complete robot joint states
     * and maintains unified TF tree.
     */
    class IKWrapperNode : public rclcpp::Node
    {
    public:
        /**
         * @brief Constructor
         */
        IKWrapperNode();

        /**
         * @brief Destructor
         */
        ~IKWrapperNode();

    private:
        // Core WBC controller
        std::unique_ptr<RobotKinematicsTool> wbc_controller_;

        // ROS parameters
        std::string robot_urdf_path_;
        std::string base_link_;
        std::string left_end_effector_;
        std::string right_end_effector_;
        double control_frequency_;
        bool debug_mode_;
        bool enable_debug_output_;

        // Control parameters (configurable)
        int task_check_frequency_;
        int target_log_frequency_;
        int debug_log_frequency_;
        int state_log_frequency_;
        int solve_log_frequency_;

        // IK solver parameters (configurable from params_private.yaml)
        double frame_task_weight_;
        double kinetic_energy_weight_;

        // Workspace limits (configurable) - used in validateWorkspaceLimits()
        double workspace_x_min_;
        double workspace_x_max_;
        double workspace_y_min_;
        double workspace_y_max_;
        double workspace_z_min_;
        double workspace_z_max_;

        // Joint configuration
        std::vector<std::string> joint_names_;
        std::vector<double> joint_lower_limits_;
        std::vector<double> joint_upper_limits_;
        std::vector<double> joint_velocity_limits_;
        std::vector<double> initial_joint_positions_; // Initial/home positions from config

        // Control state
        std::mutex state_mutex_;
        bool controller_active_;
        bool has_left_target_pose_;
        bool has_right_target_pose_;

        // Current robot state (all 16 joints)
        sensor_msgs::msg::JointState current_joint_state_;
        geometry_msgs::msg::PoseStamped left_target_pose_;
        geometry_msgs::msg::PoseStamped right_target_pose_;
        wbc::msg::WBCState current_wbc_state_;

        // Initial end-effector poses for debug reference
        geometry_msgs::msg::Pose initial_left_ee_pose_;
        geometry_msgs::msg::Pose initial_right_ee_pose_;

        // VR tracker relative control state
        geometry_msgs::msg::PoseStamped left_tracker_init_pose_;
        geometry_msgs::msg::PoseStamped right_tracker_init_pose_;
        geometry_msgs::msg::PoseStamped left_robot_init_ee_pose_;
        geometry_msgs::msg::PoseStamped right_robot_init_ee_pose_;
        bool has_left_tracker_init_;
        bool has_right_tracker_init_;
        bool has_robot_init_poses_;

        // VR to Robot coordinate system transformation
        // Each pair: [source_axis_index, sign] where source_axis_index ∈ {0,1,2}, sign ∈ {-1,1}
        std::vector<std::pair<int, int>> vr_position_axis_mapping_;    // [[axis,sign], [axis,sign], [axis,sign]] for [X,Y,Z]
        std::vector<std::pair<int, int>> vr_orientation_axis_mapping_; // [[axis,sign], [axis,sign], [axis,sign]] for [X,Y,Z]

        // NEW: Separate coordinate transformations for left and right hands
        std::vector<std::pair<int, int>> left_vr_position_axis_mapping_;
        std::vector<std::pair<int, int>> left_vr_orientation_axis_mapping_;
        std::vector<std::pair<int, int>> right_vr_position_axis_mapping_;
        std::vector<std::pair<int, int>> right_vr_orientation_axis_mapping_;

        // ty add 2025-10-11
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr ik_arm_data_pub_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr ik_waist_data_pub_;
        // ty add end

        // ROS publishers
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_pub_;
        rclcpp::Publisher<wbc::msg::WBCState>::SharedPtr wbc_state_pub_;
        // Publishers
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr target_joint_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_target_pose_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_target_pose_pub_;

        // TF broadcaster for target poses
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        // ROS subscribers
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr left_vr_tracker_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr right_vr_tracker_sub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr realign_trigger_sub_;

        // Control timer
        rclcpp::TimerBase::SharedPtr control_timer_;

        // Last computation time (stored from ikSolve)
        double last_computation_time_;

        // IK solve logger control
        const bool enable_ik_logging_ = false;  // Hardcoded: disable IK logging
        std::unique_ptr<IKSolveLogger> ik_solve_logger_;

        /**
         * @brief Initialize ROS parameters from configuration
         * @return true if successful
         */
        bool initializeParameters();

        /**
         * @brief Initialize WBC controller
         * @return true if successful
         */
        bool initializeController();

        /**
         * @brief Initialize ROS publishers and subscribers
         */
        void initializeROS();

        /**
         * @brief Initialize complete robot joint state
         */
        void initializeJointState();

        /**
         * @brief Main control loop - runs at control_frequency_
         */
        void controlLoop();

        /**
         * @brief Left VR tracker pose callback
         * @param msg VR tracker pose message
         */
        void leftVRTrackerPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        /**
         * @brief Right VR tracker pose callback
         * @param msg VR tracker pose message
         */
        void rightVRTrackerPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        /**
         * @brief Joint state callback (for external joint state updates)
         * @param msg Joint state message
         */
        void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

        /**
         * @brief Update dual-arm control with current targets
         */
        void updateDualArmControl();

        /**
         * @brief Publish debug visualization markers
         */
        void publishDebugMarkers();

        /**
         * @brief Publish target pose as TF for visualization
         * @param pose Target pose to publish
         * @param frame_name TF frame name
         */
        void publishTargetPoseTF(const geometry_msgs::msg::PoseStamped &pose, const std::string &frame_name);

        /**
         * @brief Validate workspace limits for target poses
         * @param pose Target pose to validate
         * @param workspace_name Name for logging
         * @return true if within workspace
         */
        bool validateWorkspaceLimits(const geometry_msgs::msg::PoseStamped &pose,
                                     const std::string &workspace_name);

        /**
         * @brief Compute relative target pose for VR tracker control
         * @param current_tracker_pose Current tracker pose
         * @param init_tracker_pose Initial tracker pose (when first detected)
         * @param init_robot_ee_pose Initial robot end-effector pose
         * @param tracker_id Tracker identifier for quaternion continuity
         * @return Relative target pose for end-effector
         */
        geometry_msgs::msg::PoseStamped computeRelativeTargetPose(
            const geometry_msgs::msg::PoseStamped &current_tracker_pose,
            const geometry_msgs::msg::PoseStamped &init_tracker_pose,
            const geometry_msgs::msg::PoseStamped &init_robot_ee_pose,
            const std::string &tracker_id);

        /**
         * @brief Convert geometry_msgs::msg::Pose to Eigen::Affine3d
         * @param pose Input pose
         * @return Affine3d transformation
         */
        Eigen::Affine3d poseToAffine3d(const geometry_msgs::msg::Pose &pose);

        /**
         * @brief Convert Eigen::Affine3d to geometry_msgs::msg::Pose
         * @param transform Input Affine3d transformation
         * @param tracker_id Optional tracker identifier for quaternion continuity
         * @return Pose message
         */
        geometry_msgs::msg::Pose affine3dToPose(const Eigen::Affine3d &transform, const std::string &tracker_id = "");

        /**
         * @brief Transform VR tracker pose to Robot coordinate system
         * @param vr_pose Input pose in VR coordinate system
         * @return Transformed pose in Robot coordinate system
         */
        geometry_msgs::msg::PoseStamped transformVRToRobotCoordinates(const geometry_msgs::msg::PoseStamped &vr_pose, const std::string &hand_side = "both");

        /**
         * @brief Initialize robot initial poses for relative control
         */
        void initializeRobotInitialPoses();

        /**
         * @brief Compute and log initial end-effector poses using home configuration
         */
        void computeAndLogInitialEEPoses();
    };

} // namespace wbc

