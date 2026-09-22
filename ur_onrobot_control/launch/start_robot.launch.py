#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
start_robot.launch.py - Launch principal con selector TUI (Textual) y soporte headless/no bloqueante.
Compatible con ROS 2 Humble y Dockerfile.aimen.
"""

import os
import sys
from ament_index_python.packages import get_package_share_directory, get_package_prefix, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def import_selector_tui():
    """Busca e importa selector_tui de forma dinámica independientemente de la forma de instalación."""
    try:
        from ur_onrobot_control.scripts import selector_tui
        return selector_tui
    except ImportError:
        pass

    # Intentar rutas relativas e instaladas
    candidate_paths = [
        os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'scripts')),
    ]
    try:
        share_dir = get_package_share_directory('ur_onrobot_control')
        candidate_paths.append(os.path.join(share_dir, 'scripts'))
    except PackageNotFoundError:
        pass

    try:
        prefix_dir = get_package_prefix('ur_onrobot_control')
        candidate_paths.append(os.path.join(prefix_dir, 'lib', 'ur_onrobot_control'))
    except PackageNotFoundError:
        pass

    for p in candidate_paths:
        if os.path.isdir(p) and p not in sys.path:
            sys.path.insert(0, p)

    try:
        import selector_tui
        return selector_tui
    except ImportError:
        return None


def interactive_selector(context):
    """
    Función de selección:
    - Si interactive=true y hay TTY disponible: lanza la TUI con Textual.
    - Si interactive=false o no hay TTY: modo no bloqueante headless basado en argumentos.
    """
    interactive_str = LaunchConfiguration('interactive').perform(context).lower()
    is_interactive = (interactive_str in ('true', '1', 'yes'))
    has_tty = sys.stdin.isatty() and sys.stdout.isatty()

    ur_type = LaunchConfiguration('ur_type').perform(context)
    onrobot_type = LaunchConfiguration('onrobot_type').perform(context)
    use_simulation = LaunchConfiguration('use_simulation').perform(context)
    sim_env = LaunchConfiguration('sim_env').perform(context)
    robot_ip = LaunchConfiguration('robot_ip').perform(context)

    actions = []

    if is_interactive and has_tty:
        selector_mod = import_selector_tui()
        if selector_mod is not None:
            # Ejecutar interfaz TUI de Textual
            result = selector_mod.run_tui(default_ur=ur_type, default_onrobot=onrobot_type)
            if not result or result.get('cancelled', False):
                print("\n🚫 Operación cancelada por el usuario en la TUI.")
                sys.exit(0)

            # Aplicar configuración seleccionada en la TUI
            context.launch_configurations['use_simulation'] = str(result.get('use_simulation', 'true'))
            context.launch_configurations['sim_env'] = str(result.get('sim_env', 'basic'))
            context.launch_configurations['robot_ip'] = str(result.get('robot_ip', '127.0.0.1'))
            context.launch_configurations['onrobot_type'] = str(result.get('onrobot_type', onrobot_type))
            context.launch_configurations['launch_rviz'] = str(result.get('launch_rviz', 'true'))

            sel_sim = context.launch_configurations['use_simulation']
            sel_env = context.launch_configurations['sim_env']
            sel_ip = context.launch_configurations['robot_ip']
            sel_grp = context.launch_configurations['onrobot_type']
            sel_rviz = context.launch_configurations['launch_rviz']

            print("\n" + "="*60)
            if sel_sim == 'true':
                print(f"🎮 TUI -> SIMULACIÓN: Entorno={sel_env.upper()} | Gripper={sel_grp.upper()} | RViz={sel_rviz}")
            else:
                print(f"🤖 TUI -> ROBOT REAL: IP={sel_ip} | Gripper={sel_grp.upper()} | RViz={sel_rviz}")
            print("="*60 + "\n")

            actions.append(LogInfo(msg=f"Configuración TUI aplicada: sim={sel_sim}, env={sel_env}, ip={sel_ip}, gripper={sel_grp}, rviz={sel_rviz}"))
            return actions
        else:
            print("⚠️ [start_robot] 'textual' o 'selector_tui' no disponible. Continuando con configuración de argumentos...")

    # Modo Headless / No bloqueante
    print("\n" + "="*60)
    print("🤖 APERTA / UR ONROBOT - MODO HEADLESS / NO BLOQUEANTE".center(60))
    print("="*60)
    print(f"   • Simulación:    {use_simulation}")
    print(f"   • Entorno:       {sim_env}")
    print(f"   • Robot IP:      {robot_ip}")
    print(f"   • Tipo Robot UR: {ur_type}")
    print(f"   • Tipo Gripper:  {onrobot_type}")
    print("="*60 + "\n")

    actions.append(LogInfo(msg=f"Modo Headless activo: sim={use_simulation}, env={sim_env}, ip={robot_ip}, gripper={onrobot_type}"))
    return actions


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('ur_type', default_value='ur5e',
                            description='Tipo de robot UR (ur5, ur5e, ur10, etc.)'),
        DeclareLaunchArgument('onrobot_type', default_value='2fg7',
                            choices=['2fg7', '3fg15', 'vgc10'],
                            description='Tipo de gripper OnRobot (2fg7, 3fg15, vgc10)'),
        DeclareLaunchArgument('launch_onrobot', default_value='true',
                            description='Lanzar control del gripper'),
        DeclareLaunchArgument('launch_rviz', default_value='true',
                            description='Lanzar RViz2'),
        DeclareLaunchArgument('rviz_config', default_value='view_robot.rviz',
                            description='Configuración de RVIZ'),
        DeclareLaunchArgument('interactive', default_value='true',
                            description='true=Abrir TUI textual interactiva si hay TTY; false=Modo headless usando argumentos'),
        
        # Parámetros del entorno / ejecución (pueden venir de CLI o ser sobreescritos por TUI)
        DeclareLaunchArgument('use_simulation', default_value='true',
                            description='true=simulación, false=robot real'),
        DeclareLaunchArgument('robot_ip', default_value='127.0.0.1',
                            description='IP del robot físico (127.0.0.1 para simulación)'),
        DeclareLaunchArgument('sim_env', default_value='basic',
                            choices=['basic', 'left', 'right'],
                            description='Entorno: basic, left, right'),
    ]

    # Selector TUI / Headless
    selector_action = OpaqueFunction(function=interactive_selector)

    # ========== LAUNCH DE SIMULACIÓN ==========
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur_onrobot_control'),
                'launch',
                'start_simulation.launch.py'
            ])
        ]),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
            'onrobot_type': LaunchConfiguration('onrobot_type'),
            'launch_onrobot': LaunchConfiguration('launch_onrobot'),
            'launch_rviz': LaunchConfiguration('launch_rviz'),
            'rviz_config': LaunchConfiguration('rviz_config'),
            'sim_env': LaunchConfiguration('sim_env'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_simulation'))
    )

    # ========== LAUNCH DE ROBOT REAL ==========
    real_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur_onrobot_control'),
                'launch',
                'start_real_robot.launch.py'
            ])
        ]),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
            'robot_ip': LaunchConfiguration('robot_ip'),
            'onrobot_type': LaunchConfiguration('onrobot_type'),
            'launch_onrobot': LaunchConfiguration('launch_onrobot'),
            'launch_rviz': LaunchConfiguration('launch_rviz'),
            'use_fake_hardware': 'false',
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('use_simulation'))
    )

    return LaunchDescription([
        *declared_arguments,
        selector_action,
        simulation_launch,
        real_robot_launch,
    ])