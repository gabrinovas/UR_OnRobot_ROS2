#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    moveit_config_package = 'ur_onrobot_moveit_config'
    moveit_config_dir = get_package_share_directory(moveit_config_package)
    
    # Parámetros
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    launch_rviz = LaunchConfiguration('launch_rviz', default='true')
    
    # Incluir el launch principal de MoveIt
    main_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(moveit_config_package),
                'launch',
                'ur_onrobot_moveit.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'launch_rviz': 'false',
            'launch_onrobot': 'true',
            'use_fake_hardware': 'true',
            'environment': 'basic',
        }.items()
    )
    
    # Node para RViz con MoveIt (configuración específica para demo)
    rviz_config_path = os.path.join(moveit_config_dir, 'rviz', 'view_robot.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(launch_rviz)
    )
    
    # Node para Joint State Publisher GUI (opcional para demo)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(use_sim_time)
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'launch_rviz', 
            default_value='true',
            description='Launch RViz2 for demo'
        ),
        main_launch,
        rviz_node,
        joint_state_publisher_gui_node,
    ])