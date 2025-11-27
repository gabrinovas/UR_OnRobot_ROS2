#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

import subprocess

def detect_robot_only_when_needed(context):
    use_fake = LaunchConfiguration('use_fake_hardware').perform(context)
    
    # Si se fuerza simulación, retornar inmediatamente
    if use_fake == "true":
        print("\nModo simulación forzado → MODO SIMULACIÓN\n")
        return [SetLaunchConfiguration('use_fake_hardware', 'true'),
                SetLaunchConfiguration('robot_ip', '127.0.0.1'),
                SetLaunchConfiguration('robot_detected', 'true')]

    # Verificar solo la IP específica
    target_ip = "192.168.1.101"
    result = subprocess.run(['ping', '-c', '1', '-W', '1', target_ip],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    
    if result.returncode == 0:
        print(f"\nRobot físico detectado en {target_ip} → MODO REAL\n")
        return [SetLaunchConfiguration('use_fake_hardware', 'false'),
                SetLaunchConfiguration('robot_ip', target_ip),
                SetLaunchConfiguration('robot_detected', 'true')]

    print(f"\nNo se detectó robot físico en {target_ip} → NO SE INICIA EL SISTEMA\n")
    return [SetLaunchConfiguration('use_fake_hardware', 'true'),
            SetLaunchConfiguration('robot_ip', '127.0.0.1'),
            SetLaunchConfiguration('robot_detected', 'false')]

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument('ur_type', default_value='ur5e'))
    declared_arguments.append(DeclareLaunchArgument('robot_ip', default_value='192.168.1.101'))
    declared_arguments.append(DeclareLaunchArgument('use_fake_hardware', default_value='false',
                                                    description='Si es true, fuerza simulación'))
    declared_arguments.append(DeclareLaunchArgument('onrobot_type', default_value='2fg7'))
    declared_arguments.append(DeclareLaunchArgument('launch_onrobot', default_value='true'))
    declared_arguments.append(DeclareLaunchArgument('rviz_config', default_value='view_robot.rviz'))
    declared_arguments.append(DeclareLaunchArgument('robot_detected', default_value='false'))

    detection_action = OpaqueFunction(function=detect_robot_only_when_needed)

    # ====== ROBOT DESCRIPTION COMBINADO CON ENTORNO ======
    robot_description_content = Command([
        'xacro ', PathJoinSubstitution([
            FindPackageShare('ur_onrobot_control'),
            'urdf',
            'left_robot_with_environment.urdf.xacro'
        ]),
        ' ur_type:=', LaunchConfiguration('ur_type'),
        ' robot_ip:=', LaunchConfiguration('robot_ip'),
        ' onrobot_type:=', LaunchConfiguration('onrobot_type'),
        ' use_fake_hardware:=', LaunchConfiguration('use_fake_hardware')
    ])

    robot_description = {"robot_description": robot_description_content}

    # ====== MAIN ROBOT STATE PUBLISHER ======
    main_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='main_robot_state_publisher',
        output='both',
        parameters=[robot_description],
        remappings=[
            ('/joint_states', '/merged_joint_states'),
        ],
        condition=IfCondition(LaunchConfiguration('robot_detected'))
    )

    # ====== JOINT STATE MERGER ======
    joint_state_merger = Node(
        package='ur_onrobot_control',
        executable='joint_state_merger.py',
        name='joint_state_merger',
        output='screen',
        condition=IfCondition(LaunchConfiguration('robot_detected'))
    )

    # ====== DRIVER UR MODIFICADO ======
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
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
            'launch_rviz': 'false',
            'headless_mode': 'false',
            'launch_robot_state_publisher': 'false',
            'description_package': 'ur_onrobot_control',
            'description_file': 'left_robot_with_environment.urdf.xacro',
        }.items(),
        condition=IfCondition(LaunchConfiguration('robot_detected'))
    )

    # ====== DRIVER ONROBOT ======
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
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
            'launch_rviz': 'false',
            'launch_rsp': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_onrobot'))
    )

    # ====== RVIZ ======
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
        detection_action,
        joint_state_merger,
        main_robot_state_publisher,
        ur_launch,
        onrobot_launch,
        rviz_node
    ])