#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ur_onrobot_moveit.launch.py - Launch dinámico de MoveIt 2 para UR5e + OnRobot (2FG7, 3FG15, VGC10)
Compatible con entornos basic, left y right.
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
    launch_rviz_val = LaunchConfiguration('launch_rviz')

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

    # 4. Pipelines de planificación (OMPL + Pilz)
    ompl_planning_yaml = load_yaml('ur_onrobot_moveit_config', 'config/ompl_planning.yaml')
    pilz_planning_yaml = load_yaml('ur_onrobot_moveit_config', 'config/pilz_industrial_motion_planner_planning.yaml')

    planning_pipelines = {
        'planning_pipelines': ['ompl', 'pilz_industrial_motion_planner'],
        'default_planning_pipeline': 'ompl',
        'ompl': ompl_planning_yaml,
        'pilz_industrial_motion_planner': pilz_planning_yaml,
    }

    # 5. Configuración de controladores MoveIt (dinámica según fake_hardware / robot real)
    moveit_controllers_yaml = load_yaml('ur_onrobot_moveit_config', 'config/moveit_controllers.yaml')
    target_dict = moveit_controllers_yaml.get('moveit_simple_controller_manager', moveit_controllers_yaml)
    if use_fake_hardware_val.lower() == 'true':
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

    # 6. Planning Scene Monitor
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    joint_states_topic_val = LaunchConfiguration('joint_states_topic')

    # Nodo move_group
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        remappings=[
            ('joint_states', joint_states_topic_val),
            ('/joint_states', joint_states_topic_val),
        ],
        parameters=[
            robot_description,
            robot_description_semantic,
            {'robot_description_kinematics': kinematics_yaml},
            {'robot_description_planning': joint_limits_yaml},
            {'joint_limits': joint_limits_yaml},
            planning_pipelines,
            trajectory_execution,
            moveit_controllers_yaml,
            planning_scene_monitor_parameters,
        ],
    )

    # Nodo RViz2 con configuración MoveIt
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('ur_onrobot_moveit_config'),
        'rviz',
        'moveit.rviz'
    ])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_moveit',
        output='screen',
        condition=IfCondition(launch_rviz_val),
        arguments=['-d', rviz_config_file],
        remappings=[
            ('joint_states', joint_states_topic_val),
            ('/joint_states', joint_states_topic_val),
        ],
        parameters=[
            robot_description,
            robot_description_semantic,
            {'robot_description_kinematics': kinematics_yaml},
            planning_pipelines,
        ],
    )

    return [
        move_group_node,
        rviz_node,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'ur_type',
            default_value='ur5e',
            description='Modelo de robot UR (ej. ur5e)',
        ),
        DeclareLaunchArgument(
            'onrobot_type',
            default_value='2fg7',
            description='Modelo de pinza OnRobot (2fg7, 3fg15, vgc10)',
            choices=['2fg7', '3fg15', 'vgc10'],
        ),
        DeclareLaunchArgument(
            'sim_env',
            default_value='basic',
            description='Entorno de celda de trabajo: basic (solo robot), left (entorno izquierdo), right (entorno derecho)',
            choices=['basic', 'left', 'right'],
        ),
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='true',
            description='true para simulación, false para robot físico',
        ),
        DeclareLaunchArgument(
            'robot_ip',
            default_value='127.0.0.1',
            description='IP del robot UR (127.0.0.1 para simulación)',
        ),
        DeclareLaunchArgument(
            'tf_prefix',
            default_value='',
            description='Prefijo TF para multi-robot o namespacing',
        ),
        DeclareLaunchArgument(
            'joint_states_topic',
            default_value='/merged_joint_states',
            description='Tópico de estados articulares unificados para MoveIt (por defecto /merged_joint_states)',
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Lanzar RViz2 con el plugin MotionPlanning',
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
