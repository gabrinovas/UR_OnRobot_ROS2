#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
foxglove_bridge.launch.py - Puente WebSocket para conexión de Foxglove Studio
Permite conectar la interfaz gráfica Foxglove Studio (Web o Desktop) a la celda ROS 2.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'port',
            default_value='8765',
            description='Puerto WebSocket de escucha para Foxglove Studio'
        ),
        DeclareLaunchArgument(
            'address',
            default_value='0.0.0.0',
            description='Dirección IP de enlace (0.0.0.0 para acceso de red completo)'
        ),
        DeclareLaunchArgument(
            'send_buffer_limit',
            default_value='10000000',
            description='Límite de buffer de envío en bytes'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Usar tiempo de simulación'
        )
    ]

    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'address': LaunchConfiguration('address'),
            'send_buffer_limit': LaunchConfiguration('send_buffer_limit'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'client_topic_whitelist': ['.*'],
            'min_qos_depth': 1,
            'max_qos_depth': 10,
        }]
    )

    return LaunchDescription(declared_arguments + [foxglove_bridge_node])
