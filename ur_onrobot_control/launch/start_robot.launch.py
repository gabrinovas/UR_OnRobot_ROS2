#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

import subprocess

def detect_robot_only_when_needed(context):
    use_fake = LaunchConfiguration('use_fake_hardware').perform(context)
    robot_side = LaunchConfiguration('robot_side').perform(context)
    
    # Si se fuerza simulación, retornar inmediatamente
    if use_fake == "true":
        print(f"\nModo simulación forzado → MODO SIMULACIÓN ({robot_side})\n")
        return [SetLaunchConfiguration('use_fake_hardware', 'true'),
                SetLaunchConfiguration('robot_ip', '127.0.0.1'),
                SetLaunchConfiguration('robot_detected', 'true')]

    # Determinar IPs objetivo según el lado del robot (CORREGIDO)
    if robot_side == "left":
        target_ips = ["192.168.1.105"]  # Robot izquierdo
    elif robot_side == "right":
        target_ips = ["192.168.1.101"]  # Robot derecho
    else:  # both o cualquier otro valor
        target_ips = ["192.168.1.101", "192.168.1.105"]  # IPs para ambos robots

    # Verificar conectividad con las IPs
    for ip in target_ips:
        result = subprocess.run(['ping', '-c', '1', '-W', '1', ip],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if result.returncode == 0:
            print(f"\nRobot físico detectado en {ip} → MODO REAL ({robot_side})\n")
            return [SetLaunchConfiguration('use_fake_hardware', 'false'),
                    SetLaunchConfiguration('robot_ip', ip),
                    SetLaunchConfiguration('robot_detected', 'true')]

    print(f"\nNo se detectó robot físico en {target_ips} → NO SE INICIA EL SISTEMA ({robot_side})\n")
    return [SetLaunchConfiguration('use_fake_hardware', 'true'),
            SetLaunchConfiguration('robot_ip', '127.0.0.1'),
            SetLaunchConfiguration('robot_detected', 'false')]

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('ur_type', default_value='ur5e',
                            description='Tipo de robot UR (ur3, ur3e, ur5, ur5e, ur10, ur10e, ur16e, ur20)'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.101',
                            description='Dirección IP del robot físico'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false',
                            description='Si es true, fuerza simulación'),
        DeclareLaunchArgument('onrobot_type', default_value='2fg7',
                            description='Tipo de gripper OnRobot (2fg7, rg2, etc.)'),
        DeclareLaunchArgument('launch_onrobot', default_value='true',
                            description='Si se lanza el controlador del gripper'),
        DeclareLaunchArgument('rviz_config', default_value='view_robot.rviz',
                            description='Archivo de configuración de RViz'),
        DeclareLaunchArgument('robot_detected', default_value='false',
                            description='Indica si se detectó el robot físico'),
        DeclareLaunchArgument('robot_side', default_value='both',
                            choices=['left', 'right', 'both'],
                            description='Lado del robot a lanzar: left, right o both'),
        DeclareLaunchArgument('namespace', default_value='',
                            description='Namespace para múltiples robots')
    ]

    detection_action = OpaqueFunction(function=detect_robot_only_when_needed)

    # ====== CONFIGURACIÓN DE ARCHIVOS SEGÚN LADO DEL ROBOT ======
    def get_urdf_filename(robot_side):
        if robot_side == "left":
            return "left_robot_with_environment.urdf.xacro"
        elif robot_side == "right":
            return "right_robot_with_environment.urdf.xacro"
        else:  # both
            return "ur_onrobot.urdf.xacro"

    def get_rviz_config(robot_side):
        if robot_side == "left":
            return "view_left_robot.rviz"
        elif robot_side == "right":
            return "view_right_robot.rviz"
        else:  # both
            return "view_robot.rviz"

    # ====== ROBOT DESCRIPTION ======
    robot_description_content = Command([
        'xacro ', PathJoinSubstitution([
            FindPackageShare('ur_onrobot_control' if LaunchConfiguration('robot_side') != 'both' else 'ur_onrobot_description'),
            'urdf',
            get_urdf_filename(LaunchConfiguration('robot_side').perform)
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
            'description_package': 'ur_onrobot_control' if LaunchConfiguration('robot_side') != 'both' else 'ur_onrobot_description',
            'description_file': get_urdf_filename(LaunchConfiguration('robot_side').perform),
            'runtime_config_package': 'ur_robot_driver',
            'controllers_file': 'ur_controllers.yaml',
            'kinematics_params_file': PathJoinSubstitution([
                FindPackageShare('ur_onrobot_description'),
                'config',
                LaunchConfiguration('ur_type'),
                'default_kinematics.yaml'
            ]) if LaunchConfiguration('robot_side') == 'both' else '',
            'initial_joint_controller': 'scaled_joint_trajectory_controller',
            'activate_joint_controller': 'true',
            'launch_dashboard_client': 'false',
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
        get_rviz_config(LaunchConfiguration('robot_side').perform)
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