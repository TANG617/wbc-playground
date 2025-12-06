# Task-specific command for WBC
std_msgs/Header header

# Task identification
string task_name
uint8 task_id

# Task type
uint8 POSITION_TASK = 1
uint8 ORIENTATION_TASK = 2
uint8 POSE_TASK = 3
uint8 JOINT_TASK = 4
uint8 task_type

# Task priority and weights
uint8 priority
float64[] task_weights

# Target specification
geometry_msgs/Pose target_pose
sensor_msgs/JointState target_joints
geometry_msgs/Twist target_velocity

# Constraints
float64[] position_bounds_lower
float64[] position_bounds_upper
float64[] velocity_limits
float64[] acceleration_limits

# Activation
bool active
float64 activation_time
