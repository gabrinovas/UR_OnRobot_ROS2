#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ur_onrobot_mtc.launch.py - Launch para Pipeline Autónomo de Pick & Place (MoveIt Task Constructor)
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
    execute_on_startup_val = LaunchConfiguration('execute_on_startup').perform(context).lower() == 'true'

    # 1. Determinar URDF y nombre de robot según el entorno
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

    # 2. Generación dinámica de SRDF
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

    # 3. Cargar cinemática, límites y pipelines de planificación
    kinematics_yaml = load_yaml('ur_onrobot_moveit_config', 'config/kinematics.yaml')
    joint_limits_yaml = load_yaml('ur_onrobot_moveit_config', 'config/joint_limits.yaml')
    ompl_planning_yaml = load_yaml('ur_onrobot_moveit_config', 'config/ompl_planning.yaml')

    planning_pipelines = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': ompl_planning_yaml,
    }

    moveit_controllers_yaml = load_yaml('ur_onrobot_moveit_config', 'config/moveit_controllers.yaml')
    target_dict = moveit_controllers_yaml.get('moveit_simple_controller_manager', moveit_controllers_yaml)
    if use_fake_hardware_val:
        if 'scaled_joint_trajectory_controller' in target_dict:
            target_dict['scaled_joint_trajectory_controller']['default'] = False
        if 'joint_trajectory_controller' in target_dict:
            target_dict['joint_trajectory_controller']['default'] = True
    else:
        if 'scaled_joint_trajectory_controller' in target_dict:
            target_dict['scaled_joint_trajectory_controller']['default'] = True
        if 'joint_trajectory_controller' in target_dict:
            target_dict['joint_trajectory_controller']['default'] = False

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
        'trajectory_execution.execution_duration_monitoring': False,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # Estados de pinza según el modelo
    if onrobot_type_val == 'vgc10':
        open_state = 'release'
        close_state = 'grip'
    else:
        open_state = 'open'
        close_state = 'closed'

    # Frame del mundo según entorno
    world_frame = 'world'

    # Parámetros MTC
    mtc_params = {
        'arm_group_name': 'ur_manipulator',
        'gripper_group_name': 'gripper',
        'gripper_open_state': open_state,
        'gripper_close_state': close_state,
        'hand_frame': 'gripper_tcp',
        'world_frame': world_frame,
        'object_name': 'workpiece_box',
        'object_dimensions': [0.04, 0.04, 0.07],
        'pick_pose': [0.10, -0.45, 0.07, 0.0, 3.14159, 0.0],
        'place_pose': [0.30, -0.45, 0.07, 0.0, 3.14159, 0.0],
        'approach_distance': 0.08,
        'lift_distance': 0.12,
        'retreat_distance': 0.10,
        'execute_on_startup': execute_on_startup_val,
        'spawn_table_if_missing': (sim_env_val == 'basic'),
    }

    # Nodo MTC C++
    mtc_node = Node(
        package='ur_onrobot_control',
        executable='ur_onrobot_mtc_node',
        name='ur_onrobot_mtc_node',
        parameters=[
            robot_description,
            robot_description_semantic,
            {'robot_description_kinematics': kinematics_yaml},
            {'robot_description_planning': joint_limits_yaml},
            {'joint_limits': joint_limits_yaml},
            planning_pipelines,
            moveit_controllers_yaml,
            trajectory_execution,
            planning_scene_monitor_parameters,
            mtc_params,
        ],
        output='screen',
    )

    # Nodo opcional de RViz con MTC Plugin
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('ur_onrobot_moveit_config'),
        'rviz',
        'mtc.rviz'
    ])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_mtc',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
        ],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
        output='screen',
    )

    return [
        mtc_node,
        rviz_node,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'ur_type',
            default_value='ur5e',
            description='Modelo UR (ur5e)'
        ),
        DeclareLaunchArgument(
            'onrobot_type',
            default_value='2fg7',
            description='Efector OnRobot (2fg7, 3fg15, vgc10)'
        ),
        DeclareLaunchArgument(
            'sim_env',
            default_value='basic',
            description='Entorno de celda (basic, left, right)'
        ),
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='true',
            description='Usar simulación ros2_control (true/false)'
        ),
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.1.105',
            description='IP del robot físico'
        ),
        DeclareLaunchArgument(
            'tf_prefix',
            default_value='',
            description='Prefijo TF'
        ),
        DeclareLaunchArgument(
            'execute_on_startup',
            default_value='false',
            description='Ejecutar inmediatamente el pipeline al iniciar'
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='false',
            description='Lanzar RViz2 con display de MTC'
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
