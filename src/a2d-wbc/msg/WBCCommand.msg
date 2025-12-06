# Whole Body Control command message
std_msgs/Header header

# Target pose for end-effector (from PSI Glove)
geometry_msgs/PoseStamped target_pose

# Task priority (1=highest, 10=lowest)  
uint8 priority

# Control mode
uint8 POSITION_CONTROL = 1
uint8 VELOCITY_CONTROL = 2
uint8 FORCE_CONTROL = 3
uint8 control_mode

# Joint space targets (optional)
sensor_msgs/JointState joint_targets

# Task-specific parameters
float64 position_tolerance
float64 orientation_tolerance
float64 max_velocity
float64 max_acceleration

# Enable/disable flags
bool enable_position_control
bool enable_orientation_control
bool enable_null_space_control
