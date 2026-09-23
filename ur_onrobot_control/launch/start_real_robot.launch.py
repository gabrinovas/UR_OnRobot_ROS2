#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
start_real_robot.launch.py - Launch unificado para robot físico (UR5e + OnRobot 2FG7/3FG15/VGC10)
Soporta entornos basic, left y right con main_robot_state_publisher único y joint_state_merger reactivo.
Compatible con ROS 2 Humble y Dockerfile.aimen.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def configure_real_robot(context):
    sim_env = LaunchConfiguration('sim_env').perform(context)
    ur_type = LaunchConfiguration('ur_type').perform(context)
    onrobot_type = LaunchConfiguration('onrobot_type').perform(context)
    robot_ip = LaunchConfiguration('robot_ip').perform(context)

    print(f"\n🤖 CONFIGURANDO ROBOT REAL (ROS 2 Humble / Dockerfile.aimen)")
    print(f"   - Entorno de trabajo: {sim_env}")
    print(f"   - Robot UR:           {ur_type} (IP: {robot_ip})")
    print(f"   - Gripper OnRobot:    {onrobot_type}")

    # Selección dinámica del URDF combinado con entorno
    description_package = 'ur_onrobot_description'
    if sim_env == 'left':
        description_file = 'left_robot_with_environment.urdf.xacro'
        print(f"   - URDF: Entorno LEFT ({description_package}/{description_file})")
    elif sim_env == 'right':
        description_file = 'right_robot_with_environment.urdf.xacro'
        print(f"   - URDF: Entorno RIGHT ({description_package}/{description_file})")
    else:  # 'basic' por defecto
        description_file = 'ur_onrobot.urdf.xacro'
        print(f"   - URDF: Básico sin entorno adicional ({description_package}/{description_file})")

    return [
        SetLaunchConfiguration('description_package', description_package),
        SetLaunchConfiguration('description_file', description_file),
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('ur_type', default_value='ur5e',
                              description='Tipo de robot UR (ur3, ur3e, ur5, ur5e, ur10, ur10e, ur16e)'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.101',
                              description='Dirección IP del robot UR físico'),
        DeclareLaunchArgument('onrobot_type', default_value='2fg7',
                              choices=['2fg7', '3fg15', 'vgc10'],
                              description='Tipo de efector OnRobot (2fg7, 3fg15, vgc10)'),
        DeclareLaunchArgument('launch_onrobot', default_value='true',
                              description='Lanzar control y comunicación Modbus del gripper'),
        DeclareLaunchArgument('sim_env', default_value='basic',
                              choices=['basic', 'left', 'right'],
                              description='Entorno de celda: basic (solo robot), left (entorno izquierdo), right (entorno derecho)'),
        DeclareLaunchArgument('launch_rviz', default_value='true',
                              description='Lanzar RViz2'),
        DeclareLaunchArgument('rviz_config', default_value='view_robot.rviz',
                              description='Archivo de configuración de RViz2'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false',
                              description='false para robot físico real'),
        DeclareLaunchArgument('tf_prefix', default_value='',
                              description='Prefijo TF para el robot'),
        # Parámetros internos resueltos dinámicamente
        DeclareLaunchArgument('description_package', default_value='ur_onrobot_description'),
        DeclareLaunchArgument('description_file', default_value='ur_onrobot.urdf.xacro'),
    ]

    config_action = OpaqueFunction(function=configure_real_robot)

    # Robot description combinado mediante xacro para hardware real
    robot_description_content = Command([
        'xacro ', PathJoinSubstitution([
            FindPackageShare(LaunchConfiguration('description_package')),
            'urdf',
            LaunchConfiguration('description_file')
        ]),
        ' ur_type:=', LaunchConfiguration('ur_type'),
        ' robot_ip:=', LaunchConfiguration('robot_ip'),
        ' onrobot_type:=', LaunchConfiguration('onrobot_type'),
        ' use_fake_hardware:=false',
        ' tf_prefix:=', LaunchConfiguration('tf_prefix')
    ])

    robot_description = {"robot_description": robot_description_content}

    # ====== MAIN ROBOT STATE PUBLISHER ======
    # Publica la cinemática completa del robot + entorno + pinza a partir de /merged_joint_states
    main_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='main_robot_state_publisher',
        output='both',
        parameters=[robot_description],
        remappings=[('/joint_states', '/merged_joint_states')]
    )

    # ====== JOINT STATE MERGER (Zero-Latency) ======
    joint_state_merger = Node(
        package='ur_onrobot_control',
        executable='joint_state_merger.py',
        name='joint_state_merger',
        output='screen',
        parameters=[{
            'sim_env': LaunchConfiguration('sim_env'),
            'tf_prefix': LaunchConfiguration('tf_prefix')
        }]
    )

    # ====== SELECTOR DE CONFIGURACIÓN DE CONTROLADORES ======
    controllers_config_file = PythonExpression([
        "'right_scaled_controller.yaml' if '", LaunchConfiguration('sim_env'), "' == 'right' else ",
        "'left_scaled_controller.yaml' if '", LaunchConfiguration('sim_env'), "' == 'left' else ",
        "'base_env_controllers.yaml'"
    ])

    # ====== DRIVER UR FÍSICO ======
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
            'use_fake_hardware': 'false',
            'launch_rviz': 'false',
            'headless_mode': 'true',
            'launch_robot_state_publisher': 'false',
            'tf_prefix': LaunchConfiguration('tf_prefix'),
            'controllers_config_file': PathJoinSubstitution([
                FindPackageShare('ur_onrobot_control'),
                'config',
                controllers_config_file
            ])
        }.items()
    )

    # ====== DRIVER ONROBOT FÍSICO ======
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
            'use_fake_hardware': 'false',
            'launch_rviz': 'false',
            'launch_rsp': 'false',
            'tf_prefix': LaunchConfiguration('tf_prefix'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_onrobot'))
    )

    # ====== RVIZ2 CENTRALIZADO ======
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
        condition=IfCondition(LaunchConfiguration('launch_rviz'))
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