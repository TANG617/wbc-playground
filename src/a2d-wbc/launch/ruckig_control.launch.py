from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import yaml
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable
def generate_launch_description():
    ruckig_dof_arg = DeclareLaunchArgument(
    'ruckig_dof', 
    default_value='16',
    description='the number of degrees of freedom of the robot'
    )

    ruckig_hz_arg = DeclareLaunchArgument(
    'ruckig_hz', 
    default_value='100.0',
    description='the frequency of the ruckig control'
    )

    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('wbc'),
            'assets', 'A2D_NoHand', 'A2D_NoHand_Flattened.urdf'
        ]),
        description='Path to robot URDF file'
    )


    #仿照qrcode_transfer.launch.py，从params_common.yaml中读取参数
    config_file = "/var/psi/configuration/params_common.yaml"
    with open(config_file, 'r') as f:
        yaml_data = yaml.safe_load(f)
    yaml_vel_scale = yaml_data['wbc']['arm_control_node']['ros__parameters']['vel_scale']
    yaml_acc_scale = yaml_data['wbc']['arm_control_node']['ros__parameters']['acc_scale']
    yaml_jerk_scale = yaml_data['wbc']['arm_control_node']['ros__parameters']['jerk_scale']

    declare_vel_scale = DeclareLaunchArgument(  #命令行参数
    'vel_scale', 
    default_value=str(yaml_vel_scale),
    description='the velocity scale of the ruckig control'
    )

    declare_acc_scale = DeclareLaunchArgument(
    'acc_scale', 
    default_value=str(yaml_acc_scale),
    description='the acceleration scale of the ruckig control'
    )

    declare_jerk_scale = DeclareLaunchArgument(
    'jerk_scale', 
    default_value=str(yaml_jerk_scale),
    description='the jerk scale of the ruckig control'
    )

    ruckig_control_node = Node(
        package='wbc',
        executable='arm_control_node',
        name='arm_control_node',
        namespace='arm_control_node',
        output='screen',
        parameters=[
        config_file,
        {"vel_scale": LaunchConfiguration('vel_scale'),
        "acc_scale": LaunchConfiguration('acc_scale'),
        "jerk_scale": LaunchConfiguration('jerk_scale'),
        "dof": LaunchConfiguration('ruckig_dof'), 
        'robot_urdf_path': LaunchConfiguration('urdf_path'),
        "control_frequency": LaunchConfiguration('ruckig_hz')}
        ]
    )
  
    return LaunchDescription([
        SetEnvironmentVariable(name="RCUTILS_COLORIZED_OUTPUT", value="1"),#使rclcpp输出彩色日志
        urdf_path_arg,
        ruckig_dof_arg, 
        ruckig_hz_arg, 
        declare_vel_scale,
        declare_acc_scale,
        declare_jerk_scale,
        ruckig_control_node
    ])