#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    control_frequency_arg = DeclareLaunchArgument(
        'control_frequency',
        default_value='100.0',
        description='Control loop frequency in Hz'
    )
    
    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('wbc'),
            'assets', 'A2D_NoHand', 'A2D_NoHand_Flattened.urdf'
        ]),
        description='Path to robot URDF file'
    )
    
    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Enable RViz2 visualization'
    )
    
    control_frequency = LaunchConfiguration('control_frequency')
    urdf_path = LaunchConfiguration('urdf_path')
    enable_rviz = LaunchConfiguration('enable_rviz')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_ik',
        output='screen',
        parameters=[{
            'robot_description': Command(['cat ', urdf_path]),
            'use_sim_time': False,
            'publish_frequency': 100.0
        }]
    )
    
    static_tf_world_to_a2d_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_to_a2d',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'world',
            '--child-frame-id', 'A2D'
        ]
    )
    

    ik_wrapper_node = Node(
        package='wbc',
        executable='ik_node',
        name='ik_wrapper_node',
        output='screen',
        parameters=[
            {
                'robot_urdf_path': urdf_path,
                'base_link': 'base_link',
                'left_end_effector': 'Link7_l',
                'right_end_effector': 'Link7_r',
                'control_frequency': ParameterValue(control_frequency, value_type=float),
                'use_vr_trackers': True
            },
            PathJoinSubstitution([
                FindPackageShare('wbc'),
                'config', 'wbc_config.yaml'
            ]),
            '/var/psi/configuration/params_private.yaml'
        ]
    )

    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('wbc'),
        'config', 'wbc_visualization.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_vr_wbc',
        output='screen',
        condition=IfCondition(enable_rviz),
        arguments=['-d', rviz_config_path]
    )
    
    return LaunchDescription([
        # Launch arguments
        control_frequency_arg,
        urdf_path_arg,
        enable_rviz_arg,
        
        # Nodes
        robot_state_publisher_node,
        static_tf_world_to_a2d_node,
        ik_wrapper_node,
        rviz_node,
    ])
