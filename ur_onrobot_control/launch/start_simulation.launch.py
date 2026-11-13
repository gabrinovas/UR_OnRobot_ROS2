#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
start_simulation.launch.py - Launch exclusivo para simulación (hardware simulado).
Declara los argumentos de la simulación e incluye el bringup común (ur_onrobot_bringup.launch.py).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('sim_env', default_value='basic',
                            description='Entorno de simulación (basic, left, right)'),
        DeclareLaunchArgument('ur_type', default_value='ur5e',
                            description='Tipo de robot UR'),
        DeclareLaunchArgument('onrobot_type', default_value='2fg7',
                            description='Tipo de gripper OnRobot'),
        DeclareLaunchArgument('launch_onrobot', default_value='true',
                            description='Lanzar control del gripper'),
        DeclareLaunchArgument('rviz_config', default_value='view_robot.rviz',
                            description='Configuración de RVIZ'),
        DeclareLaunchArgument('launch_rviz', default_value='true',
                            description='Lanzar RViz2'),
        DeclareLaunchArgument('launch_watchdog', default_value='false',
                            description='Lanzar Watchdog de Seguridad Industrial UR + OnRobot'),
        DeclareLaunchArgument('initial_joint_controller', default_value='scaled_joint_trajectory_controller',
                            description='Controlador articular inicial'),
        DeclareLaunchArgument('controller_spawner_timeout', default_value='25',
                            description='Timeout used when spawning controllers.'),
        DeclareLaunchArgument('load_freedrive_controller', default_value='false',
                            description='Cargar el controlador de guiado a mano (freedrive) del UR. Desactivado por '
                                        'defecto: mientras está cargado, el nodo de control puede colgarse al parar'),
        # Parámetros internos
        DeclareLaunchArgument('description_package', default_value='ur_onrobot_description'),
        DeclareLaunchArgument('description_file', default_value='ur_onrobot.urdf.xacro'),
        DeclareLaunchArgument('robot_ip', default_value='127.0.0.1'),
        DeclareLaunchArgument('tf_prefix', default_value='',
                            description='Prefijo TF para el robot'),
    ]

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('ur_onrobot_control'), 'launch', 'ur_onrobot_bringup.launch.py'])
        ]),
        launch_arguments={
            'use_fake_hardware': 'true',
            'robot_ip': '127.0.0.1',  # la simulación no usa la IP del robot
            'ur_type': LaunchConfiguration('ur_type'),
            'onrobot_type': LaunchConfiguration('onrobot_type'),
            'launch_onrobot': LaunchConfiguration('launch_onrobot'),
            'sim_env': LaunchConfiguration('sim_env'),
            'launch_rviz': LaunchConfiguration('launch_rviz'),
            'rviz_config': LaunchConfiguration('rviz_config'),
            'tf_prefix': LaunchConfiguration('tf_prefix'),
            'launch_watchdog': LaunchConfiguration('launch_watchdog'),
            'initial_joint_controller': LaunchConfiguration('initial_joint_controller'),
            'controller_spawner_timeout': LaunchConfiguration('controller_spawner_timeout'),
            'use_gripper_action_controller': 'true',
            'load_freedrive_controller': LaunchConfiguration('load_freedrive_controller'),
        }.items()
    )

    return LaunchDescription([*declared_arguments, bringup])
