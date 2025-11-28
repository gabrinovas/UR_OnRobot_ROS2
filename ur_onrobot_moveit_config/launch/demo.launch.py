#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    moveit_config_package = 'ur_onrobot_moveit_config'
    
    # Parámetros para demo MoveIt puro
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
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
            'use_fake_hardware': 'true',
            'environment': 'basic',
            'launch_rviz': 'false',
            'launch_onrobot': 'true',
        }.items()
    )
    
    # RViz específico para MoveIt (sin controles manuales)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_moveit',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare(moveit_config_package),
            'rviz',
            'view_robot.rviz'
        ])],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(launch_rviz)
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time for MoveIt demo'
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true', 
            description='Launch RViz2 for MoveIt visualization'
        ),
        main_launch,
        rviz_node,
    ])