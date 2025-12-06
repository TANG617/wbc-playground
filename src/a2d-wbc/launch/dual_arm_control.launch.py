from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable
def generate_launch_description():
    #1.启动Vive追踪器
    vr_tracker_path = get_package_share_directory('vr_tracker')
    vr_tracker_launch_file = PathJoinSubstitution([vr_tracker_path, 'launch', 'trackers_node.launch.py'])
    #2.启动FK
    fk_node_path = get_package_share_directory('wbc')
    fk_node_launch_file = PathJoinSubstitution([fk_node_path, 'launch', 'fk_node.launch.py'])
    #3.启动IK
    ik_node_path = get_package_share_directory('wbc')
    ik_node_launch_file = PathJoinSubstitution([ik_node_path, 'launch', 'ik_node.launch.py'])
    #4.启动Ruckig控制器
    ruckig_control_node_path = get_package_share_directory('wbc')
    ruckig_control_node_launch_file = PathJoinSubstitution([ruckig_control_node_path, 'launch', 'ruckig_control.launch.py'])

    
    return LaunchDescription([
        SetEnvironmentVariable(name="RCUTILS_COLORIZED_OUTPUT", value="1"),#使rclcpp输出彩色日志
        IncludeLaunchDescription(PythonLaunchDescriptionSource(vr_tracker_launch_file)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fk_node_launch_file),
            launch_arguments={
                "enable_rviz": "false",
                "enable_robot_state_publisher": "false"
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ik_node_launch_file),
            launch_arguments={
                "enable_rviz": "true"  # 显式启用 RViz
            }.items()      
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(ruckig_control_node_launch_file)),
    ])