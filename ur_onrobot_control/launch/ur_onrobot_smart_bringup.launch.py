#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Launch definitivo UR + OnRobot con:
- Detección automática por ping (101 y 105)
- 3 ventanas de RViz:
    1. Solo gripper  ← lanzada por onrobot_driver
    2. Solo UR       ← lanzada por ur_robot_driver
    3. UR + gripper  ← lanzada por nosotros
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def check_robot_reachable(context):
    import subprocess
    ips = ["192.168.1.101", "192.168.1.105"]
    for ip in ips:
        result = subprocess.run(['ping', '-c', '1', '-W', '1', ip],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if result.returncode == 0:
            print(f"\nRobot físico detectado en {ip} → MODO REAL\n")
            return "true"
    print(f"\nNo se detectó robot físico → MODO SIMULACIÓN (3 ventanas RViz)\n")
    return "false"


def generate_launch_description():

    # ====================== ARGUMENTOS ======================
    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("ur_type", default_value="ur5e"))
    declared_arguments.append(DeclareLaunchArgument("onrobot_type", default_value="rg2"))
    declared_arguments.append(DeclareLaunchArgument("launch_onrobot", default_value="true"))

    # ====================== DETECCIÓN AUTOMÁTICA ======================
    robot_reachable = OpaqueFunction(function=check_robot_reachable)
    use_fake_hardware = PythonExpression(["'", robot_reachable, "' == 'false'"])
    robot_ip = PythonExpression(["'192.168.1.101' if '", robot_reachable, "' == 'true' else '127.0.0.1'"])

    # ====================== DRIVER UR (siempre lanza su RViz) ======================
    ur_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ur_robot_driver"),
                "launch",
                "ur_control.launch.py"
            ])
        ]),
        launch_arguments={
            "ur_type": LaunchConfiguration("ur_type"),
            "robot_ip": robot_ip,
            "use_fake_hardware": use_fake_hardware,
            "launch_rviz": "true",                                   # Ventana 2: solo UR
            "rviz_config": PathJoinSubstitution([
                FindPackageShare("ur_robot_driver"),
                "config",
                "view_robot.rviz"
            ]),
        }.items()
    )

    # ====================== DRIVER ONROBOT (lanza su propio RViz) ======================
    onrobot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("onrobot_driver"),   # o el paquete que uses
                "launch",
                "onrobot_control.launch.py"
            ])
        ]),
        launch_arguments={
            "onrobot_type": LaunchConfiguration("onrobot_type"),
            "launch_rviz": "true",                                    # Ventana 1: solo gripper
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_onrobot"))
    )

    # ====================== RVIZ 3: ROBOT COMPLETO (UR + GRIPPER) ======================
    rviz_full_config = PathJoinSubstitution([
        FindPackageShare("tu_paquete"),   # CAMBIA "tu_paquete" por el nombre real de tu paquete
        "rviz",
        "ur_with_onrobot.rviz"
    ])

    rviz_full = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_full_robot",
        output="screen",
        arguments=["-d", rviz_full_config],
        parameters=[{"use_sim_time": False}]
    )

    # ====================== LANZAMIENTO FINAL ======================
    return LaunchDescription([
        *declared_arguments,
        robot_reachable,        # necesario para que se ejecute la función
        ur_launch,
        onrobot_launch,
        rviz_full               # Ventana 3: robot + gripper juntos
    ])