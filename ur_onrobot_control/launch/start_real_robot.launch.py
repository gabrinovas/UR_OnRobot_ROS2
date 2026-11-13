#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
start_real_robot.launch.py - Launch unificado para robot físico (UR5e + OnRobot 2FG7/3FG15/VGC10)
Soporta entornos basic, left y right con main_robot_state_publisher único y joint_state_merger reactivo.
Compatible con ROS 2 Humble y Dockerfile.aimen.
Declara los argumentos del robot físico e incluye el bringup común (ur_onrobot_bringup.launch.py).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


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
        DeclareLaunchArgument('load_freedrive_controller', default_value='false',
                              description='Cargar el controlador de guiado a mano (freedrive) del UR. Desactivado por '
                                          'defecto: mientras está cargado, el nodo de control puede colgarse al parar'),
        # Parámetros internos resueltos dinámicamente
        DeclareLaunchArgument('description_package', default_value='ur_onrobot_description'),
        DeclareLaunchArgument('description_file', default_value='ur_onrobot.urdf.xacro'),
    ]

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('ur_onrobot_control'), 'launch', 'ur_onrobot_bringup.launch.py'])
        ]),
        launch_arguments={
            'use_fake_hardware': 'false',
            'robot_ip': LaunchConfiguration('robot_ip'),
            'ur_type': LaunchConfiguration('ur_type'),
            'onrobot_type': LaunchConfiguration('onrobot_type'),
            'launch_onrobot': LaunchConfiguration('launch_onrobot'),
            'sim_env': LaunchConfiguration('sim_env'),
            'launch_rviz': LaunchConfiguration('launch_rviz'),
            'rviz_config': LaunchConfiguration('rviz_config'),
            'tf_prefix': LaunchConfiguration('tf_prefix'),
            # Valores que este lanzador nunca ha expuesto: los de por defecto de los drivers
            'launch_watchdog': 'false',
            'initial_joint_controller': 'scaled_joint_trajectory_controller',
            'controller_spawner_timeout': '10',
            'use_gripper_action_controller': 'true',
            'load_freedrive_controller': LaunchConfiguration('load_freedrive_controller'),
        }.items()
    )

    return LaunchDescription([*declared_arguments, bringup])
