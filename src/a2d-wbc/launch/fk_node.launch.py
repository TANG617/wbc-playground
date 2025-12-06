#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('wbc')

    default_urdf = PathJoinSubstitution([
        pkg_share, 'assets', 'A2D_NoHand', 'A2D_NoHand_Flattened.urdf'
    ])
    
    rviz_config = PathJoinSubstitution([
        pkg_share, 'config', 'fk_visualization.rviz'
    ])
    
    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value=default_urdf,
        description='Path to robot URDF file'
    )
    
    frequency_arg = DeclareLaunchArgument(
        'frequency',
        default_value='100.0',
        description='FK computation frequency in Hz'
    )
    
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='map',
        description='Base frame for TF publishing'
    )
    
    left_ee_frame_arg = DeclareLaunchArgument(
        'left_ee_frame',
        default_value='Link7_l',
        description='Left end effector frame name'
    )
    
    right_ee_frame_arg = DeclareLaunchArgument(
        'right_ee_frame',
        default_value='Link7_r',
        description='Right end effector frame name'
    )
    
    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Enable TF publishing for TCP poses'
    )
    
    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Launch RViz visualization'
    )
    
    enable_rsp_arg = DeclareLaunchArgument(
        'enable_robot_state_publisher',
        default_value='true',
        description='Launch robot state publisher'
    )
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['cat ', LaunchConfiguration('urdf_path')]),
            'use_sim_time': False,
            'publish_frequency': LaunchConfiguration('frequency'),
            'frame_prefix': 'fk/',  
        }],
        remappings=[
    ('/joint_states', '/fk/joint_states'),
],
        condition=IfCondition(LaunchConfiguration('enable_robot_state_publisher'))
    )
    
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_fk_a2d',
        output='screen',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                   '--frame-id', 'map', '--child-frame-id', 'fk/A2D']
    )
    
    # FK Wrapper Node
    fk_node = Node(
        package='wbc',
        executable='fk_node',
        name='fk_wrapper_node',
        output='screen',
        parameters=[{
            'robot_urdf_path': LaunchConfiguration('urdf_path'),
            'base_frame': LaunchConfiguration('base_frame'),
            'left_end_effector': LaunchConfiguration('left_ee_frame'),
            'right_end_effector': LaunchConfiguration('right_ee_frame'),
            'frequency': LaunchConfiguration('frequency'),
            'publish_tf': LaunchConfiguration('publish_tf'),
            'enable_robot_state_publisher': LaunchConfiguration('enable_robot_state_publisher'),  
        }]
    )
    
    # RViz Visualization (使用FK专用的 fk_visualization.rviz, Fixed Frame=map)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_fk_visualization',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('enable_rviz'))
    )
    
    return LaunchDescription([
        urdf_path_arg,
        frequency_arg,
        base_frame_arg,
        left_ee_frame_arg,
        right_ee_frame_arg,
        publish_tf_arg,
        enable_rviz_arg,
        enable_rsp_arg,
        
        # Nodes
        robot_state_publisher_node,
        static_tf_node,
        fk_node,
        rviz_node,
    ])
