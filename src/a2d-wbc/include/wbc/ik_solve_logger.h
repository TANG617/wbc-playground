#pragma once

#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <fstream>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <memory>
#include <string>

namespace wbc
{

/**
 * @brief Structure to hold IK solve data for logging
 */
struct IKSolveLogEntry
{
    double timestamp;
    bool success;
    double solve_time_ms;
    double dt;
    
    // Input data
    sensor_msgs::msg::JointState input_joint_state;
    geometry_msgs::msg::PoseStamped left_target_pose;
    geometry_msgs::msg::PoseStamped right_target_pose;
    
    // Output data
    sensor_msgs::msg::JointState output_joint_state;
};

/**
 * @brief Asynchronous logger for IK solve data using producer-consumer pattern
 * 
 * This class logs IK solve input/output data without blocking the main control loop.
 * Uses a background thread to write data to file asynchronously.
 */
class IKSolveLogger
{
public:
    /**
     * @brief Constructor
     * @param logger ROS logger for status messages
     * @param max_queue_size Maximum size of log queue (default: 1000)
     */
    explicit IKSolveLogger(const rclcpp::Logger& logger, size_t max_queue_size = 1000);
    
    /**
     * @brief Destructor - stops logging thread and closes file
     */
    ~IKSolveLogger();

    /**
     * @brief Initialize logger with file path
     * @param log_dir Directory to save log files (default: /var/psi/log)
     * @return true if successful
     */
    bool initialize(const std::string& log_dir = "/var/psi/log");

    /**
     * @brief Log an IK solve call (non-blocking, thread-safe)
     * @param timestamp Current ROS time in seconds
     * @param current_joint_state Input joint state
     * @param left_target_pose Input left target pose
     * @param right_target_pose Input right target pose
     * @param output_joint_state Output joint state
     * @param solve_time_ms Solve time in milliseconds
     * @param dt Time step
     * @param success Whether IK solve succeeded
     */
    void logIKSolve(
        double timestamp,
        const sensor_msgs::msg::JointState& current_joint_state,
        const geometry_msgs::msg::PoseStamped& left_target_pose,
        const geometry_msgs::msg::PoseStamped& right_target_pose,
        const sensor_msgs::msg::JointState& output_joint_state,
        double solve_time_ms,
        double dt,
        bool success);

    /**
     * @brief Get number of pending log entries in queue
     * @return Queue size
     */
    size_t getPendingLogCount() const { return log_queue_.size(); }

    /**
     * @brief Check if logger is active
     * @return true if active
     */
    bool isActive() const { return is_active_; }

private:
    /**
     * @brief Background thread function for writing logs
     */
    void logWriterThread();

    /**
     * @brief Write a single log entry to file
     * @param entry Log entry to write
     */
    void writeLogEntry(const IKSolveLogEntry& entry);

    /**
     * @brief Create log file with timestamp
     * @param log_dir Directory to save log file
     * @return true if successful
     */
    bool createLogFile(const std::string& log_dir);

    // Logger
    rclcpp::Logger logger_;

    // File stream
    std::ofstream log_file_;
    std::string log_filename_;

    // Thread-safe queue
    std::queue<IKSolveLogEntry> log_queue_;
    mutable std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    size_t max_queue_size_;

    // Background thread
    std::thread writer_thread_;
    std::atomic<bool> is_active_;
    std::atomic<bool> should_stop_;

    // Statistics
    std::atomic<size_t> total_logged_;
    std::atomic<size_t> total_dropped_;
};

} // namespace wbc

