#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Launch file universal para Universal Robots + OnRobot (real o simulado)

Uso típico:
  # Robot real + gripper real
  ros2 launch mi_paquete ur_onrobot_bringup.launch.py

  # Simulación completa con RViz
  ros2 launch mi_paquete ur_onrobot_bringup.launch.py use_fake_hardware:=true

  # Solo el brazo simulado (sin gripper)
  ros2 launch mi_paquete ur_onrobot_bringup.launch.py use_fake_hardware:=true launch_onrobot:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ====================== ARGUMENTOS ======================
    declared_arguments = []

    # ---- Universal Robots ----
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
            description="Modelo del UR: ur3, ur3e, ur5, ur5e, ur10, ur10e, ur16e, ur20, ur30"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.1.105",
            description="IP del robot real (solo se usa cuando use_fake_hardware=false)"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="true → modo simulación (fake hardware), false → robot físico"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="true → abre RViz con la configuración del UR"
        )
    )

    # ---- OnRobot Gripper ----
    declared_arguments.append(
        DeclareLaunchArgument(
            "onrobot_type",
            default_value="rg2",
            description="Tipo de gripper: rg2, rg6, 2fg7, 2fg14, 3fg15, etc."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_onrobot",
            default_value="true",
            description="true → lanza el driver del gripper OnRobot, false → solo el brazo"
        )
    )

    # ====================== LANZAR DRIVER UR ======================
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
            "robot_ip": LaunchConfiguration("robot_ip"),
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            # Configuración de RViz recomendada (incluida en ur_robot_driver)
            "rviz_config": PathJoinSubstitution([
                FindPackageShare("ur_robot_driver"),
                "config",
                "view_robot.rviz"
            ]),
            # Opcional: activar controllers por defecto en simulación
            "controllers_file": PathJoinSubstitution([
                FindPackageShare("ur_robot_driver"),
                "config",
                "ur_controllers.yaml"
            ]),
            "initial_joint_controller": "scaled_joint_trajectory_controller",
        }.items()
    )

    # ====================== LANZAR ONROBOT (opcional) ======================
    onrobot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("onrobot_driver"),        # o el paquete que tengas (onrobot_rg2, etc.)
                "launch",
                "onrobot_control.launch.py"
            ])
        ]),
        launch_arguments={
            "onrobot_type": LaunchConfiguration("onrobot_type"),
            # Si tu driver de OnRobot necesita más parámetros, añádelos aquí
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_onrobot"))
    )

    # ====================== DEVOLVER LANZAMIENTO ======================
    return LaunchDescription([
        *declared_arguments,
        ur_launch,
        onrobot_launch
    ])