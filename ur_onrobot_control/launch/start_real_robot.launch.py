#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
real_robot_launch.py - Launch exclusivo para robot físico
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('ur_type', default_value='ur5e',
                            description='Tipo de robot UR'),
        DeclareLaunchArgument('robot_ip', 
                            default_value='192.168.1.101',
                            description='IP del robot físico'),
        DeclareLaunchArgument('onrobot_type', default_value='2fg7',
                            description='Tipo de gripper OnRobot'),
        DeclareLaunchArgument('launch_onrobot', default_value='true',
                            description='Lanzar control del gripper'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false',
                            description='true=simulación, false=robot real'),
    ]

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
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
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
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_onrobot'))
    )

    return LaunchDescription([
        *declared_arguments,
        ur_launch,
        onrobot_launch
    ])