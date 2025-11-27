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
    if use_fake == "true":
        return [SetLaunchConfiguration('use_fake_hardware', 'true'),
                SetLaunchConfiguration('robot_ip', '127.0.0.1')]

    ips = ["192.168.1.101", "192.168.1.105"]
    for ip in ips:
        result = subprocess.run(['ping', '-c', '1', '-W', '1', ip],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if result.returncode == 0:
            print(f"\nRobot físico detectado en {ip} → MODO REAL\n")
            return [SetLaunchConfiguration('use_fake_hardware', 'false'),
                    SetLaunchConfiguration('robot_ip', ip)]

    print("\nNo se detectó robot físico → MODO SIMULACIÓN\n")
    return [SetLaunchConfiguration('use_fake_hardware', 'true'),
            SetLaunchConfiguration('robot_ip', '127.0.0.1')]

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument('ur_type', default_value='ur5e'))
    declared_arguments.append(DeclareLaunchArgument('robot_ip', default_value='192.168.1.101'))
    declared_arguments.append(DeclareLaunchArgument('use_fake_hardware', default_value='false',
                                                    description='Si es true, fuerza simulación'))
    declared_arguments.append(DeclareLaunchArgument('onrobot_type', default_value='2fg7'))
    declared_arguments.append(DeclareLaunchArgument('launch_onrobot', default_value='true'))
    declared_arguments.append(DeclareLaunchArgument('rviz_config', default_value='view_robot.rviz'))

    detection_action = OpaqueFunction(function=detect_robot_only_when_needed)

    # ====== ROBOT DESCRIPTION COMBINADO (UR + OnRobot) ======
    robot_description_content = Command([
        'xacro ', PathJoinSubstitution([
            FindPackageShare('ur_onrobot_description'),
            'urdf',
            'ur_onrobot.urdf.xacro'
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
        ]
    )

    # ====== JOINT STATE MERGER ======
    joint_state_merger = Node(
        package='ur_onrobot_control',
        executable='joint_state_merger.py',
        name='joint_state_merger',
        output='screen'
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
            'description_package': 'ur_onrobot_description',
            'description_file': 'ur_onrobot.urdf.xacro',
            'runtime_config_package': 'ur_robot_driver',
            'controllers_file': 'ur_controllers.yaml',
            'kinematics_params_file': PathJoinSubstitution([
                FindPackageShare('ur_onrobot_description'),
                'config',
                LaunchConfiguration('ur_type'),
                'default_kinematics.yaml'
            ]),
            'initial_joint_controller': 'scaled_joint_trajectory_controller',
            'activate_joint_controller': 'true',
            'launch_dashboard_client': 'false',
        }.items()
    )

    # ====== DRIVER ONROBOT (sin robot_state_publisher propio) ======
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