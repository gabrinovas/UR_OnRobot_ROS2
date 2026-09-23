#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ur_onrobot_servo.launch.py - Launch para MoveIt 2 Servo (Teleoperación Cartesiana en Vivo)
Manipulador UR5e con efectores OnRobot (2FG7, 3FG15, VGC10) en entornos basic, left y right.
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except (EnvironmentError, yaml.YAMLError):
        return None


def launch_setup(context, *args, **kwargs):
    ur_type_val = LaunchConfiguration('ur_type').perform(context)
    onrobot_type_val = LaunchConfiguration('onrobot_type').perform(context)
    sim_env_val = LaunchConfiguration('sim_env').perform(context)
    use_fake_hardware_val = LaunchConfiguration('use_fake_hardware').perform(context)
    robot_ip_val = LaunchConfiguration('robot_ip').perform(context)
    tf_prefix_val = LaunchConfiguration('tf_prefix').perform(context)

    # 1. Resolver archivo URDF y nombre de robot según el entorno seleccionado
    urdf_package = 'ur_onrobot_description'
    if sim_env_val == 'left':
        urdf_file = 'left_robot_with_environment.urdf.xacro'
        robot_name_val = 'left_robot_with_environment'
    elif sim_env_val == 'right':
        urdf_file = 'right_robot_with_environment.urdf.xacro'
        robot_name_val = 'right_robot_with_environment'
    else:  # 'basic'
        urdf_file = 'ur_onrobot.urdf.xacro'
        robot_name_val = 'ur_onrobot'

    # Generación dinámica del robot_description mediante xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([FindPackageShare(urdf_package), 'urdf', urdf_file]),
        ' ',
        'ur_type:=', ur_type_val,
        ' ',
        'onrobot_type:=', onrobot_type_val,
        ' ',
        'robot_ip:=', robot_ip_val,
        ' ',
        'use_fake_hardware:=', use_fake_hardware_val,
        ' ',
        'tf_prefix:=', tf_prefix_val,
    ])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # 2. Generación dinámica del SRDF mediante xacro
    srdf_file = PathJoinSubstitution([
        FindPackageShare('ur_onrobot_moveit_config'),
        'srdf',
        'ur_onrobot.srdf.xacro'
    ])
    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        srdf_file,
        ' ',
        'robot_name:=', robot_name_val,
        ' ',
        'onrobot_type:=', onrobot_type_val,
        ' ',
        'sim_env:=', sim_env_val,
        ' ',
        'tf_prefix:=', tf_prefix_val,
    ])
    robot_description_semantic = {'robot_description_semantic': ParameterValue(robot_description_semantic_content, value_type=str)}

    # 3. Cargar cinemática y límites articulares
    kinematics_yaml = load_yaml('ur_onrobot_moveit_config', 'config/kinematics.yaml')
    joint_limits_yaml = load_yaml('ur_onrobot_moveit_config', 'config/joint_limits.yaml')

    # 4. Cargar configuración de MoveIt Servo
    servo_yaml = load_yaml('ur_onrobot_moveit_config', 'config/ur_onrobot_servo.yaml')
    servo_params = {'moveit_servo': servo_yaml}

    # Nodo principal MoveIt Servo
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node',
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits_yaml,
        ],
        output='screen',
    )

    # Nodo opcional de joystick (Joy)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        condition=IfCondition(LaunchConfiguration('launch_joy')),
        output='screen',
    )

    return [
        servo_node,
        joy_node,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'ur_type',
            default_value='ur5e',
            description='Modelo de robot UR (ur3, ur3e, ur5, ur5e, ur10, ur10e, ur16e, ur20, ur30)'
        ),
        DeclareLaunchArgument(
            'onrobot_type',
            default_value='2fg7',
            description='Modelo de pinza OnRobot (2fg7, 3fg15, vgc10)'
        ),
        DeclareLaunchArgument(
            'sim_env',
            default_value='basic',
            description='Entorno de celda (basic, left, right)'
        ),
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='true',
            description='Usar hardware simulado (true) o controlador físico (false)'
        ),
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.1.105',
            description='Dirección IP del robot real'
        ),
        DeclareLaunchArgument(
            'tf_prefix',
            default_value='',
            description='Prefijo TF para articulaciones y enlaces'
        ),
        DeclareLaunchArgument(
            'launch_joy',
            default_value='false',
            description='Lanzar nodo joy para mando físico (Xbox / PS / Logitech)'
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
