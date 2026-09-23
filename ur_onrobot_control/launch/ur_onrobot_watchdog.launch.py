#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ur_onrobot_watchdog.launch.py - Lanzador del Watchdog de Seguridad Industrial UR + OnRobot
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    onrobot_type_val = LaunchConfiguration('onrobot_type').perform(context)
    auto_recover_val = LaunchConfiguration('auto_recover').perform(context).lower() == 'true'
    safety_mode_topic_val = LaunchConfiguration('safety_mode_topic').perform(context)
    robot_mode_topic_val = LaunchConfiguration('robot_mode_topic').perform(context)
    joint_states_topic_val = LaunchConfiguration('joint_states_topic').perform(context)

    # Configuración de esfuerzo de retención según efector
    if onrobot_type_val == 'vgc10':
        fail_safe_effort = 80.0
        gripper_action_topic = '/onrobot/gripper_channel_a_controller/gripper_cmd'
    else:
        fail_safe_effort = 140.0
        gripper_action_topic = '/onrobot/gripper_action_controller/gripper_cmd'

    watchdog_node = Node(
        package='ur_onrobot_control',
        executable='ur_onrobot_safety_watchdog',
        name='ur_onrobot_safety_watchdog',
        output='screen',
        parameters=[{
            'onrobot_type': onrobot_type_val,
            'safety_mode_topic': safety_mode_topic_val,
            'robot_mode_topic': robot_mode_topic_val,
            'joint_states_topic': joint_states_topic_val,
            'gripper_action_topic': gripper_action_topic,
            'auto_recover': auto_recover_val,
            'fail_safe_effort': fail_safe_effort,
            'diagnostic_period': 1.0,
        }]
    )

    return [watchdog_node]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'onrobot_type',
            default_value='2fg7',
            description='Modelo de efector OnRobot (2fg7, 3fg15, vgc10)',
            choices=['2fg7', '3fg15', 'vgc10']
        ),
        DeclareLaunchArgument(
            'auto_recover',
            default_value='false',
            description='Intentar rearme automático al volver a estado NORMAL'
        ),
        DeclareLaunchArgument(
            'safety_mode_topic',
            default_value='/io_and_status_controller/safety_mode',
            description='Tópico de modo de seguridad UR'
        ),
        DeclareLaunchArgument(
            'robot_mode_topic',
            default_value='/io_and_status_controller/robot_mode',
            description='Tópico de modo de robot UR'
        ),
        DeclareLaunchArgument(
            'joint_states_topic',
            default_value='/merged_joint_states',
            description='Tópico sincronizado de estados articulares'
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
