#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UR + OnRobot - Launch DEFINITIVO (Noviembre 2025)
- Detección automática del robot (ping) solo si no se fuerza simulación
- UNA SOLA ventana RViz con robot + gripper completos (real y simulación)
"""
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, EqualsSubstitution, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def detect_robot_only_when_needed(context):
    """
    Solo se ejecuta si NO se forzó use_fake_hardware:=true
    """
    use_fake = LaunchConfiguration('use_fake_hardware').perform(context)

    if use_fake == "true":
        print("\nuse_fake_hardware:=true detectado → Forzando modo simulación\n")
        return [
            SetLaunchConfiguration('use_fake_hardware', 'true'),
            SetLaunchConfiguration('robot_ip', '127.0.0.1')
        ]

    # Ping a las IPs posibles
    ips = ["192.168.1.101", "192.168.1.105"]
    for ip in ips:
        result = subprocess.run(['ping', '-c', '1', '-W', '1', ip],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if result.returncode == 0:
            print(f"\nRobot físico detectado en {ip} → MODO REAL\n")
            return [
                SetLaunchConfiguration('use_fake_hardware', 'false'),
                SetLaunchConfiguration('robot_ip', ip)
            ]

    # No encontrado → simulación
    print("\nNo se detectó robot físico → MODO SIMULACIÓN\n")
    return [
        SetLaunchConfiguration('use_fake_hardware', 'true'),
        SetLaunchConfiguration('robot_ip', '127.0.0.1')
    ]


def generate_launch_description():
    return LaunchDescription([

        # ====================== ARGUMENTOS ======================
        DeclareLaunchArgument("ur_type", default_value="ur5e"),
        DeclareLaunchArgument("onrobot_type", default_value="2fg7"),
        DeclareLaunchArgument("launch_onrobot", default_value="true"),
        DeclareLaunchArgument("tf_prefix", default_value=""),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",  # por defecto simulación si no se dice nada
            description="true → simulación forzada (sin ping), false → fuerza real, nada → detección automática"
        ),
        DeclareLaunchArgument("robot_ip", default_value="127.0.0.1"),

        # ====================== DETECCIÓN AUTOMÁTICA ======================
        OpaqueFunction(
            function=detect_robot_only_when_needed,
            condition=UnlessCondition(EqualsSubstitution(LaunchConfiguration("use_fake_hardware"), "true"))
        ),

        # ====================== DRIVER UR ======================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("ur_robot_driver"),
                    "launch",
                    "ur_control.launch.py"
                ])
            ]),
            launch_arguments={
                "ur_type": LaunchConfiguration("ur_type"),
                "robot_ip": LaunchConfiguration("robot_ip"),
                "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
                "headless_mode": "false",
                "start_joint_controller": "true",
                "reverse_interface_enabled": "false",
                "dashboard_client_enabled": "false",
            }.items()
        ),

        # ====================== DRIVER ONROBOT ======================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("onrobot_driver"),
                    "launch",
                    "onrobot_control.launch.py"
                ])
            ]),
            launch_arguments={
                "onrobot_type": LaunchConfiguration("onrobot_type"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("launch_onrobot"))
        ),

        # ====================== DESCRIPCIÓN COMPLETA + UNA SOLA VENTANA RVIZ ======================
        # Siempre se lanza (tanto en real como en simulación)
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher_complete",
            output="screen",
            parameters=[{
                "robot_description": Command([
                    "xacro ",
                    PathJoinSubstitution([
                        FindPackageShare("ur_onrobot_description"),
                        "urdf",
                        "ur_onrobot.urdf.xacro"
                    ]),
                    " ur_type:=", LaunchConfiguration("ur_type"),
                    " onrobot_type:=", LaunchConfiguration("onrobot_type"),
                    " tf_prefix:=", LaunchConfiguration("tf_prefix"),
                    " use_fake_hardware:=", LaunchConfiguration("use_fake_hardware"),
                    " simulation_mode:=true"
                ]),
                "frame_prefix": LaunchConfiguration("tf_prefix")
            }]
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_complete_robot",
            arguments=[
                "-d", PathJoinSubstitution([
                    FindPackageShare("ur_onrobot_description"),
                    "rviz",
                    "view_robot.rviz"
                ]),
                "--fixed-frame", "world"
            ],
            output="screen"
        ),

    ])