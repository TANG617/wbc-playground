# Whole Body Control state message
std_msgs/Header header

# Initialization status
bool is_initialized

# Current joint state
sensor_msgs/JointState current_joint_state

# Current end-effector pose
geometry_msgs/PoseStamped current_pose

# Task execution status
uint8 IDLE = 0
uint8 EXECUTING = 1
uint8 SUCCESS = 2
uint8 ERROR = 3
uint8 status

# Error information
string error_message
float64 position_error
float64 orientation_error

# Performance metrics
float64 computation_time
float64 solver_residual
bool convergence_achieved

# Joint limits status
bool[] joint_limits_violated
float64[] joint_limit_margins
