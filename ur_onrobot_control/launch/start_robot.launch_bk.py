#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # ====================== ARGUMENTOS ======================
    declared_arguments = []

    # UR
    declared_arguments.append(DeclareLaunchArgument('ur_type', default_value='ur5e'))
    declared_arguments.append(DeclareLaunchArgument('robot_ip', default_value='192.168.1.105'))

    # OnRobot
    declared_arguments.append(DeclareLaunchArgument('onrobot_type', default_value='rg2',
                                                    description='rg2, rg6, 2fg7, 2fg14, 3fg15'))
    declared_arguments.append(DeclareLaunchArgument('launch_onrobot', default_value='true',
                                                    description='true/false para lanzar el gripper'))

    # ====================== LANZAR UR OFICIAL ======================
    ur_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur_robot_driver'),
                'launch',
                'ur_control.launch.py'
            ])
        ]),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
            'robot_ip': LaunchConfiguration('robot_ip'),
            # Añade aquí cualquier otro parámetro que uses habitualmente
            # 'launch_rviz': 'true',
            'use_fake_hardware': 'false',
        }.items()
    )

    # ====================== LANZAR ONROBOT (solo si launch_onrobot:=true) ======================
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
            # Añade aquí los parámetros que necesites del gripper
            # 'connection_type': 'tcp',
            # 'ip_address': '192.168.1.1',
            # 'ns': 'onrobot',
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_onrobot'))   # CORREGIDO
    )

    return LaunchDescription([
        *declared_arguments,
        ur_launch,
        onrobot_launch
    ])