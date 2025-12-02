#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
robot_master_selector.launch.py - Launch principal simplificado y optimizado
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

import subprocess
import sys

def interactive_selector(context):
    """
    Función interactiva que detecta robots y pregunta al usuario
    """
    print("\n" + "="*60)
    print("🤖 SELECTOR PRINCIPAL - ROBOT/SIMULACIÓN".center(60))
    print("="*60)
    
    # Parámetros del usuario
    ur_type = LaunchConfiguration('ur_type').perform(context)
    onrobot_type = LaunchConfiguration('onrobot_type').perform(context)
    
    print(f"\n📋 Configuración base:")
    print(f"   - Robot UR: {ur_type}")
    print(f"   - Gripper: {onrobot_type}")
    
    # Definir IPs para cada robot
    ips_by_side = {
        'left': '192.168.1.105',
        'right': '192.168.1.101'
    }
    
    detected_robots = {}
    
    print("\n🔍 Escaneando red...")
    
    # Detectar robots
    for side, ip in ips_by_side.items():
        result = subprocess.run(['ping', '-c', '1', '-W', '1', ip],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if result.returncode == 0:
            detected_robots[side] = ip
            print(f"   ✅ Robot {side.upper()} en {ip}")
        else:
            print(f"   ❌ Robot {side.upper()} no disponible")
    
    print("\n" + "-"*60)
    
    actions = []
    
    # Si no hay robots detectados -> Solo simulación disponible
    if not detected_robots:
        print("\n⚠️  No se detectaron robots físicos.")
        print("\nOpciones:")
        print("   1. Usar SIMULACIÓN")
        print("   2. Salir")
        
        try:
            choice = input("\nSelecciona (1-2): ").strip()
        except KeyboardInterrupt:
            print("\n\n🚫 Cancelado por el usuario")
            sys.exit(0)
        
        if choice == "1":
            print("\n🎮 Activando SIMULACIÓN...")
            
            # Preguntar entorno de simulación
            print("\n🎯 Selecciona entorno de simulación:")
            print("   1. Básico (robot solo)")
            print("   2. LEFT (robot con entorno izquierdo)")
            print("   3. RIGHT (robot con entorno derecho)")
            
            try:
                sim_choice = input("\nSelecciona entorno (1-3): ").strip()
            except KeyboardInterrupt:
                print("\n\n🚫 Cancelado por el usuario")
                sys.exit(0)
            
            if sim_choice == "1":
                sim_env = 'basic'
            elif sim_choice == "2":
                sim_env = 'left'
            elif sim_choice == "3":
                sim_env = 'right'
            else:
                print("⚠️  Opción no válida, usando BÁSICO")
                sim_env = 'basic'
            
            print(f"\n🎮 Configurando simulación: entorno {sim_env.upper()}")
            
            # Configurar para simulación
            context.launch_configurations['use_simulation'] = 'true'
            context.launch_configurations['robot_ip'] = '127.0.0.1'
            context.launch_configurations['sim_env'] = sim_env
            
            actions.append(LogInfo(msg=f"Simulación activada - entorno {sim_env}"))
            
        else:
            print("\n🚫 Saliendo...")
            sys.exit(0)
    
    # Si hay robots detectados
    else:
        print("\n🤖 ROBOTS DISPONIBLES:")
        for i, (side, ip) in enumerate(detected_robots.items(), 1):
            print(f"   {i}. Robot {side.upper()} ({ip})")
        print(f"   {len(detected_robots) + 1}. Usar SIMULACIÓN")
        print(f"   {len(detected_robots) + 2}. Salir")
        
        while True:
            try:
                choice = input(f"\nSelecciona (1-{len(detected_robots) + 2}): ").strip()
                choice_num = int(choice)
                
                if 1 <= choice_num <= len(detected_robots):
                    # Robot físico seleccionado
                    selected_side = list(detected_robots.keys())[choice_num - 1]
                    selected_ip = detected_robots[selected_side]
                    
                    print(f"\n🤖 Robot {selected_side.upper()} seleccionado")
                    print("   Activando modo REAL...")
                    
                    # Configurar para robot real
                    context.launch_configurations['use_simulation'] = 'false'
                    context.launch_configurations['robot_ip'] = selected_ip
                    context.launch_configurations['sim_env'] = selected_side  # left o right
                    
                    actions.append(LogInfo(msg=f"Robot físico {selected_side} activado"))
                    break
                
                elif choice_num == len(detected_robots) + 1:
                    # Simulación seleccionada
                    print("\n🎮 SIMULACIÓN seleccionada")
                    
                    # Preguntar entorno de simulación
                    print("\n🎯 Selecciona entorno de simulación:")
                    print("   1. Básico (robot solo)")
                    print("   2. LEFT (robot con entorno izquierdo)")
                    print("   3. RIGHT (robot con entorno derecho)")
                    
                    try:
                        sim_choice = input("\nSelecciona entorno (1-3): ").strip()
                    except KeyboardInterrupt:
                        print("\n\n🚫 Cancelado por el usuario")
                        sys.exit(0)
                    
                    if sim_choice == "1":
                        sim_env = 'basic'
                    elif sim_choice == "2":
                        sim_env = 'left'
                    elif sim_choice == "3":
                        sim_env = 'right'
                    else:
                        print("⚠️  Opción no válida, usando BÁSICO")
                        sim_env = 'basic'
                    
                    print(f"\n🎮 Configurando simulación: entorno {sim_env.upper()}")
                    
                    # Configurar para simulación
                    context.launch_configurations['use_simulation'] = 'true'
                    context.launch_configurations['robot_ip'] = '127.0.0.1'
                    context.launch_configurations['sim_env'] = sim_env
                    
                    actions.append(LogInfo(msg=f"Simulación activada - entorno {sim_env}"))
                    break
                
                elif choice_num == len(detected_robots) + 2:
                    # Salir
                    print("\n🚫 Saliendo...")
                    sys.exit(0)
                else:
                    print(f"❌ Opción inválida")
                    
            except ValueError:
                print("❌ Entrada inválida")
            except KeyboardInterrupt:
                print("\n\n🚫 Cancelado por el usuario")
                sys.exit(0)
    
    # Mostrar resumen final
    use_simulation = context.launch_configurations.get('use_simulation', 'true')
    sim_env = context.launch_configurations.get('sim_env', 'basic')
    robot_ip = context.launch_configurations.get('robot_ip', '127.0.0.1')
    
    print("\n" + "="*60)
    if use_simulation == 'true':
        print(f"🎮 MODO SIMULACIÓN: entorno {sim_env.upper()}")
    else:
        print(f"🤖 MODO REAL: Robot {sim_env.upper()} en {robot_ip}")
    print("="*60)
    
    return actions

def generate_launch_description():
    # Argumentos configurables por el usuario
    declared_arguments = [
        DeclareLaunchArgument('ur_type', default_value='ur5e',
                            description='Tipo de robot UR (ur5, ur5e, ur10, etc.)'),
        DeclareLaunchArgument('onrobot_type', default_value='2fg7',
                            description='Tipo de gripper OnRobot'),
        DeclareLaunchArgument('launch_onrobot', default_value='true',
                            description='Lanzar control del gripper'),
        DeclareLaunchArgument('rviz_config', default_value='view_robot.rviz',
                            description='Configuración de RVIZ'),
        
        # Parámetros internos (se establecen en interactive_selector)
        DeclareLaunchArgument('use_simulation', default_value='true',
                            description='true=simulación, false=robot real'),
        DeclareLaunchArgument('robot_ip', default_value='127.0.0.1',
                            description='IP del robot (127.0.0.1 para simulación)'),
        DeclareLaunchArgument('sim_env', default_value='basic',
                            description='Entorno: basic, left, right'),
    ]
    
    # Selector interactivo
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
            'rviz_config': LaunchConfiguration('rviz_config'),
            'sim_env': LaunchConfiguration('sim_env'),  # basic, left, right
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