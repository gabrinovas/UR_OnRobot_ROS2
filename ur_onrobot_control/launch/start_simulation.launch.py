#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
simulation_launch.py - Launch exclusivo para simulación
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def configure_simulation(context):
    sim_env = LaunchConfiguration('sim_env').perform(context)
    
    print(f"\n🎮 CONFIGURANDO SIMULACIÓN")
    print(f"   - Entorno: {sim_env}")
    print(f"   - Robot UR: {LaunchConfiguration('ur_type').perform(context)}")
    print(f"   - Gripper: {LaunchConfiguration('onrobot_type').perform(context)}")
    
    # Determinar URDF según el entorno
    if sim_env == 'left':
        description_package = 'ur_onrobot_control'
        description_file = 'left_robot_with_sim_env.urdf.xacro'
    elif sim_env == 'right':
        description_package = 'ur_onrobot_control'
        description_file = 'right_robot_with_sim_env.urdf.xacro'
    else:  # 'basic' por defecto
        description_package = 'ur_onrobot_description'
        description_file = 'ur_onrobot.urdf.xacro'
    
    print(f"   - URDF: {description_package}/{description_file}")
    
    return [
        SetLaunchConfiguration('description_package', description_package),
        SetLaunchConfiguration('description_file', description_file),
        SetLaunchConfiguration('robot_ip', '127.0.0.1'),
    ]

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('ur_type', default_value='ur5e',
                            description='Tipo de robot UR'),
        DeclareLaunchArgument('onrobot_type', default_value='2fg7',
                            description='Tipo de gripper OnRobot'),
        DeclareLaunchArgument('launch_onrobot', default_value='true',
                            description='Lanzar control del gripper'),
        DeclareLaunchArgument('rviz_config', default_value='view_robot.rviz',
                            description='Configuración de RVIZ'),
        DeclareLaunchArgument('sim_env', default_value='basic',
                            description='Entorno: basic, left, right'),
        # Parámetros internos
        DeclareLaunchArgument('description_package', default_value='ur_onrobot_description'),
        DeclareLaunchArgument('description_file', default_value='ur_onrobot.urdf.xacro'),
        DeclareLaunchArgument('robot_ip', default_value='127.0.0.1'),
    ]

    config_action = OpaqueFunction(function=configure_simulation)

    # Robot description (use_fake_hardware siempre true)
    robot_description_content = Command([
        'xacro ', PathJoinSubstitution([
            FindPackageShare(LaunchConfiguration('description_package')),
            'urdf',
            LaunchConfiguration('description_file')
        ]),
        ' ur_type:=', LaunchConfiguration('ur_type'),
        ' robot_ip:=', LaunchConfiguration('robot_ip'),
        ' onrobot_type:=', LaunchConfiguration('onrobot_type'),
        ' use_fake_hardware:=true'
    ])

    robot_description = {"robot_description": robot_description_content}

    # Nodos
    main_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='main_robot_state_publisher',
        output='both',
        parameters=[robot_description],
        remappings=[('/joint_states', '/merged_joint_states')]
    )

    joint_state_merger = Node(
        package='ur_onrobot_control',
        executable='joint_state_merger.py',
        name='joint_state_merger',
        output='screen'
    )

    ur_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur_onrobot_control'),
                'launch',
                'ur_control.launch.py'
            ])
        ]),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
            'robot_ip': LaunchConfiguration('robot_ip'),
            'use_fake_hardware': 'true',
            'launch_rviz': 'false',
            'headless_mode': 'false',
            'launch_robot_state_publisher': 'false',
            'description_package': LaunchConfiguration('description_package'),
            'description_file': LaunchConfiguration('description_file'),
        }.items()
    )

    onrobot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('onrobot_driver'),
                'launch',
                'onrobot_control.launch.py'
            ])
        ]),
        launch_arguments={
            'onrobot_type': LaunchConfiguration('onrobot_type'),
            'use_fake_hardware': 'true',
            'launch_rviz': 'false',
            'launch_rsp': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_onrobot'))
    )

    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('ur_onrobot_description'),
        'rviz',
        LaunchConfiguration('rviz_config')
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    return LaunchDescription([
        *declared_arguments,
        config_action,
        joint_state_merger,
        main_robot_state_publisher,
        ur_launch,
        onrobot_launch,
        rviz_node
    ])