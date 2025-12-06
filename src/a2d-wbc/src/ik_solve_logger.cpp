#include "wbc/ik_solve_logger.h"
#include <sstream>
#include <iomanip>
#include <chrono>
#include <filesystem>
#include <sys/stat.h>

namespace wbc
{

IKSolveLogger::IKSolveLogger(const rclcpp::Logger& logger, size_t max_queue_size)
    : logger_(logger)
    , max_queue_size_(max_queue_size)
    , is_active_(false)
    , should_stop_(false)
    , total_logged_(0)
    , total_dropped_(0)
{
}

IKSolveLogger::~IKSolveLogger()
{
    // Signal thread to stop
    should_stop_ = true;
    queue_cv_.notify_all();

    // Wait for thread to finish
    if (writer_thread_.joinable())
    {
        writer_thread_.join();
    }

    // Close file
    if (log_file_.is_open())
    {
        log_file_.close();
    }

    if (total_logged_ > 0)
    {
        RCLCPP_INFO(logger_, "IK Logger: Total logged=%zu, dropped=%zu, file=%s",
                    total_logged_.load(), total_dropped_.load(), log_filename_.c_str());
    }
}

bool IKSolveLogger::initialize(const std::string& log_dir)
{
    if (is_active_)
    {
        RCLCPP_WARN(logger_, "IK Logger already initialized");
        return false;
    }

    // Create log file
    if (!createLogFile(log_dir))
    {
        return false;
    }

    // Start background thread
    is_active_ = true;
    writer_thread_ = std::thread(&IKSolveLogger::logWriterThread, this);

    RCLCPP_INFO(logger_, "IK Logger initialized: %s (max_queue=%zu)",
                log_filename_.c_str(), max_queue_size_);

    return true;
}

bool IKSolveLogger::createLogFile(const std::string& log_dir)
{
    // Create directory if it doesn't exist
    try
    {
        std::filesystem::path dir_path(log_dir);
        
        if (!std::filesystem::exists(dir_path))
        {
            RCLCPP_INFO(logger_, "Creating log directory: %s", log_dir.c_str());
            
            // Create directory with parent directories if needed
            if (!std::filesystem::create_directories(dir_path))
            {
                RCLCPP_ERROR(logger_, "ERROR,Failed to create directory: %s", log_dir.c_str());
                return false;
            }
            
            // Set directory permissions to 755 (rwxr-xr-x)
            std::filesystem::permissions(dir_path,
                std::filesystem::perms::owner_all |
                std::filesystem::perms::group_read | std::filesystem::perms::group_exec |
                std::filesystem::perms::others_read | std::filesystem::perms::others_exec,
                std::filesystem::perm_options::replace);
            
            RCLCPP_INFO(logger_, "Directory created successfully: %s", log_dir.c_str());
        }
        else
        {
            // Check if path is actually a directory
            if (!std::filesystem::is_directory(dir_path))
            {
                RCLCPP_ERROR(logger_, "ERROR,Path exists but is not a directory: %s", log_dir.c_str());
                return false;
            }
        }
    }
    catch (const std::filesystem::filesystem_error& e)
    {
        RCLCPP_ERROR(logger_, "ERROR,Filesystem error while creating directory: %s - %s", 
                     log_dir.c_str(), e.what());
        return false;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(logger_, "ERROR,Exception while creating directory: %s - %s", 
                     log_dir.c_str(), e.what());
        return false;
    }

    // Generate filename with timestamp: ik_solve_YYYYMMDD_HHMMSS.txt
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    
    std::stringstream ss;
    ss << log_dir << "/ik_solve_"
       << std::put_time(std::localtime(&time_t_now), "%Y%m%d_%H%M%S")
       << ".txt";
    log_filename_ = ss.str();

    // Open file
    log_file_.open(log_filename_, std::ios::out | std::ios::trunc);
    if (!log_file_.is_open())
    {
        RCLCPP_ERROR(logger_, "ERROR,Failed to create log file: %s", log_filename_.c_str());
        RCLCPP_ERROR(logger_, "   Check permissions for directory: %s", log_dir.c_str());
        return false;
    }

    // Write header
    log_file_ << "# IK Solve Debug Log - CSV Format\n";
    log_file_ << "# Generated: " << std::put_time(std::localtime(&time_t_now), "%Y-%m-%d %H:%M:%S") << "\n";
    log_file_ << "# Log Directory: " << log_dir << "\n";
    log_file_ << "# Format: CSV with 47 columns per line\n";
    log_file_ << "# Column layout: timestamp(1), current_joint_positions(16), left_target_pose(7), right_target_pose(7), output_joint_positions(16)\n";
    log_file_ << "# Total: 1 + 16 + 7 + 7 + 16 = 47 values per line\n";
    log_file_ << "# Timestamp precision: microseconds (0.000001s, 6 decimal places)\n";
    log_file_ << "# Joint order: joint_lift_body, joint_body_pitch, joint1-7_l, joint1-7_r\n";
    log_file_ << "# Pose format: x, y, z, qx, qy, qz, qw\n";
    log_file_ << "timestamp,";
    // Current joint positions (16)
    log_file_ << "curr_j0,curr_j1,curr_j2,curr_j3,curr_j4,curr_j5,curr_j6,curr_j7,curr_j8,curr_j9,curr_j10,curr_j11,curr_j12,curr_j13,curr_j14,curr_j15,";
    // Left target pose (7)
    log_file_ << "left_x,left_y,left_z,left_qx,left_qy,left_qz,left_qw,";
    // Right target pose (7)
    log_file_ << "right_x,right_y,right_z,right_qx,right_qy,right_qz,right_qw,";
    // Output joint positions (16)
    log_file_ << "out_j0,out_j1,out_j2,out_j3,out_j4,out_j5,out_j6,out_j7,out_j8,out_j9,out_j10,out_j11,out_j12,out_j13,out_j14,out_j15\n";
    log_file_ << std::fixed << std::setprecision(6);
    log_file_.flush();

    return true;
}

void IKSolveLogger::logIKSolve(
    double timestamp,
    const sensor_msgs::msg::JointState& current_joint_state,
    const geometry_msgs::msg::PoseStamped& left_target_pose,
    const geometry_msgs::msg::PoseStamped& right_target_pose,
    const sensor_msgs::msg::JointState& output_joint_state,
    double solve_time_ms,
    double dt,
    bool success)
{
    if (!is_active_)
    {
        return;
    }

    // Create log entry
    IKSolveLogEntry entry;
    entry.timestamp = timestamp;
    entry.success = success;
    entry.solve_time_ms = solve_time_ms;
    entry.dt = dt;
    entry.input_joint_state = current_joint_state;
    entry.left_target_pose = left_target_pose;
    entry.right_target_pose = right_target_pose;
    entry.output_joint_state = output_joint_state;

    // Push to queue (non-blocking)
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        
        if (log_queue_.size() >= max_queue_size_)
        {
            // Queue full, drop oldest entry
            log_queue_.pop();
            total_dropped_++;
            
            if (total_dropped_ % 100 == 1)
            {
                RCLCPP_WARN(logger_, "WARN, IK Logger queue full, dropping entries (total dropped: %zu)",
                           total_dropped_.load());
            }
        }
        
        log_queue_.push(std::move(entry));
    }
    
    // Notify writer thread
    queue_cv_.notify_one();
}

void IKSolveLogger::logWriterThread()
{
    RCLCPP_INFO(logger_, "IK Logger background thread started");

    while (!should_stop_)
    {
        IKSolveLogEntry entry;
        
        // Wait for data or stop signal
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [this] {
                return !log_queue_.empty() || should_stop_;
            });

            if (should_stop_ && log_queue_.empty())
            {
                break;
            }

            if (!log_queue_.empty())
            {
                entry = std::move(log_queue_.front());
                log_queue_.pop();
            }
            else
            {
                continue;
            }
        }

        // Write to file (outside of lock)
        writeLogEntry(entry);
        total_logged_++;
    }

    // Flush remaining entries
    while (true)
    {
        IKSolveLogEntry entry;
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (log_queue_.empty())
            {
                break;
            }
            entry = std::move(log_queue_.front());
            log_queue_.pop();
        }
        writeLogEntry(entry);
        total_logged_++;
    }

    RCLCPP_INFO(logger_, "IK Logger background thread stopped");
}

void IKSolveLogger::writeLogEntry(const IKSolveLogEntry& entry)
{
    if (!log_file_.is_open())
    {
        return;
    }

    // Write to file - CSV format (one line per entry)
    // Format: timestamp, current_joint_positions(16), left_target_pose(7), right_target_pose(7), output_joint_positions(16)
    
    // 1. Timestamp
    log_file_ << entry.timestamp << ",";
    
    // 2. Current joint positions (16 values, no velocities)
    for (size_t i = 0; i < entry.input_joint_state.position.size(); ++i)
    {
        log_file_ << entry.input_joint_state.position[i];
        if (i < entry.input_joint_state.position.size() - 1)
            log_file_ << ",";
    }
    log_file_ << ",";
    
    // 3. Left target pose (7 values: x, y, z, qx, qy, qz, qw)
    log_file_ << entry.left_target_pose.pose.position.x << ","
              << entry.left_target_pose.pose.position.y << ","
              << entry.left_target_pose.pose.position.z << ","
              << entry.left_target_pose.pose.orientation.x << ","
              << entry.left_target_pose.pose.orientation.y << ","
              << entry.left_target_pose.pose.orientation.z << ","
              << entry.left_target_pose.pose.orientation.w << ",";
    
    // 4. Right target pose (7 values: x, y, z, qx, qy, qz, qw)
    log_file_ << entry.right_target_pose.pose.position.x << ","
              << entry.right_target_pose.pose.position.y << ","
              << entry.right_target_pose.pose.position.z << ","
              << entry.right_target_pose.pose.orientation.x << ","
              << entry.right_target_pose.pose.orientation.y << ","
              << entry.right_target_pose.pose.orientation.z << ","
              << entry.right_target_pose.pose.orientation.w << ",";
    
    // 5. Output joint positions (16 values, no velocities)
    for (size_t i = 0; i < entry.output_joint_state.position.size(); ++i)
    {
        log_file_ << entry.output_joint_state.position[i];
        if (i < entry.output_joint_state.position.size() - 1)
            log_file_ << ",";
    }
    
    // End of line
    log_file_ << "\n";
    
    // Periodic flush (every 10 entries to balance performance and data safety)
    if (total_logged_ % 10 == 0)
    {
        log_file_.flush();
    }

    // Output to console - concise version
    std::stringstream console_msg;
    console_msg << std::fixed << std::setprecision(3);
    console_msg << "[IK@" << entry.timestamp << "] ";
    console_msg << (entry.success ? "✓" : "✗") << " ";
    console_msg << "Time:" << entry.solve_time_ms << "ms | ";
    
    // Left arm target
    console_msg << "L:[" 
        << entry.left_target_pose.pose.position.x << ","
        << entry.left_target_pose.pose.position.y << ","
        << entry.left_target_pose.pose.position.z << "] ";
    
    // Right arm target
    console_msg << "R:[" 
        << entry.right_target_pose.pose.position.x << ","
        << entry.right_target_pose.pose.position.y << ","
        << entry.right_target_pose.pose.position.z << "]";
    
    if (entry.success && !entry.output_joint_state.position.empty())
    {
        // Show first 3 output joint positions as sample
        console_msg << " | Out:[";
        for (size_t i = 0; i < std::min(size_t(3), entry.output_joint_state.position.size()); ++i)
        {
            if (i > 0) console_msg << ",";
            console_msg << entry.output_joint_state.position[i];
        }
        console_msg << "...]";
    }
    
    RCLCPP_INFO(logger_, "%s", console_msg.str().c_str());
}

} // namespace wbc

