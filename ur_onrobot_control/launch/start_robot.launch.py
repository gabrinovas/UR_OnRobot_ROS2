#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration, IncludeLaunchDescription, LogInfo, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, PythonExpression, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

import subprocess
import sys

def detect_robot_and_configure(context):
    use_fake = LaunchConfiguration('use_fake_hardware').perform(context)
    environment = LaunchConfiguration('environment').perform(context)
    
    # Si se fuerza simulación, retornar inmediatamente
    if use_fake == "true":
        print("\n🔄 Modo simulación forzado → MODO SIMULACIÓN")
        return [SetLaunchConfiguration('use_fake_hardware', 'true'),
                SetLaunchConfiguration('robot_ip', '127.0.0.1'),
                SetLaunchConfiguration('robot_detected', 'true'),
                SetLaunchConfiguration('simulation_mode', 'true'),
                SetLaunchConfiguration('robot_side', 'none')]

    # Definir IPs para cada robot
    ips_by_side = {
        'left': '192.168.1.105',
        'right': '192.168.1.101'
    }
    
    detected_robots = []
    
    # Detectar qué robots están disponibles
    for side, ip in ips_by_side.items():
        result = subprocess.run(['ping', '-c', '1', '-W', '1', ip],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if result.returncode == 0:
            detected_robots.append((side, ip))
            print(f"✅ Robot {side} detectado en {ip}")

    # Lógica de decisión
    if len(detected_robots) == 0:
        print("\n❌ No se detectó ningún robot físico → MODO SIMULACIÓN")
        return [SetLaunchConfiguration('use_fake_hardware', 'true'),
                SetLaunchConfiguration('robot_ip', '127.0.0.1'),
                SetLaunchConfiguration('robot_detected', 'true'),
                SetLaunchConfiguration('simulation_mode', 'true'),
                SetLaunchConfiguration('robot_side', 'none')]
    
    elif len(detected_robots) == 1:
        detected_side, detected_ip = detected_robots[0]
        print(f"\n✅ Robot {detected_side} detectado → MODO REAL")
        
        # Si environment no es básico ni auto y no coincide con el robot detectado
        if (environment != 'basic' and environment != 'auto' and 
            environment != detected_side):
            print(f"\n❌ CONFLICTO DE ENTORNO")
            print(f"Robot físico detectado: '{detected_side}'")
            print(f"Entorno solicitado: '{environment}'")
            print("\n💡 SOLUCIÓN: Ejecuta con uno de los siguientes comandos:")
            print(f"   - Para usar el entorno del robot disponible: 'environment:={detected_side}'")
            print(f"   - Para entorno básico: 'environment:=basic'")
            print(f"   - Para simulación del entorno solicitado: 'use_fake_hardware:=true'")
            print("\n🚫 Cerrando ejecución...")
            
            return [LogInfo(msg=f"Conflicto: entorno '{environment}' no compatible con robot '{detected_side}'"),
                    SetLaunchConfiguration('robot_detected', 'false'),
                    Shutdown(reason='Entorno no compatible con robot físico')]
        
        # Si environment es básico, usar robot físico normalmente
        if environment == 'basic':
            print("🎯 Entorno básico seleccionado → Usando robot físico detectado")
            return [SetLaunchConfiguration('use_fake_hardware', 'false'),
                    SetLaunchConfiguration('robot_ip', detected_ip),
                    SetLaunchConfiguration('robot_detected', 'true'),
                    SetLaunchConfiguration('simulation_mode', 'false'),
                    SetLaunchConfiguration('robot_side', detected_side)]
        
        # Caso normal: usar el robot detectado con entorno correspondiente
        return [SetLaunchConfiguration('use_fake_hardware', 'false'),
                SetLaunchConfiguration('robot_ip', detected_ip),
                SetLaunchConfiguration('robot_detected', 'true'),
                SetLaunchConfiguration('simulation_mode', 'false'),
                SetLaunchConfiguration('robot_side', detected_side)]
    
    else:  # Ambos robots detectados
        print("\n❌ AMBOS ROBOTS DETECTADOS - CONFLICTO")
        print("Se detectaron ambos robots físicos:")
        for side, ip in detected_robots:
            print(f"  - Robot {side}: {ip}")
        print("\n💡 SOLUCIÓN: Desconecta uno de los robots o usa simulación:")
        print("   'use_fake_hardware:=true'")
        print("\n🚫 Cerrando ejecución...")
        
        # Retornar acciones que terminen la ejecución
        return [LogInfo(msg="Conflicto: ambos robots detectados"),
                SetLaunchConfiguration('robot_detected', 'false'),
                Shutdown(reason='Conflicto de robots detectados')]

# Función para determinar el archivo URDF basado en la configuración
def get_urdf_filename_expression():
    """Retorna una expresión Python que determina el archivo URDF"""
    return PythonExpression([
        "'ur_onrobot.urdf.xacro' if ",
        "('", LaunchConfiguration('environment'), "' == 'basic') or ",
        "('", LaunchConfiguration('environment'), "' == 'auto' and '", LaunchConfiguration('simulation_mode'), "' == 'true') ",
        "else ",
        "'left_robot_with_environment.urdf.xacro' if '", LaunchConfiguration('environment'), "' == 'left' ",
        "else ",
        "'right_robot_with_environment.urdf.xacro' if '", LaunchConfiguration('environment'), "' == 'right' ",
        "else ",
        "'left_robot_with_environment.urdf.xacro' if '", LaunchConfiguration('robot_side'), "' == 'left' ",
        "else ",
        "'right_robot_with_environment.urdf.xacro' if '", LaunchConfiguration('robot_side'), "' == 'right' ",
        "else ",
        "'ur_onrobot.urdf.xacro'"
    ])

def get_description_package_expression():
    """Retorna una expresión Python que determina el package de descripción"""
    return PythonExpression([
        "'ur_onrobot_description' if '", LaunchConfiguration('environment'), "' == 'basic' ",
        "else 'ur_onrobot_control'"
    ])

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('ur_type', default_value='ur5e',
                            description='Tipo de robot UR (ur5, ur5e, ur10, etc.)'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.101',
                            description='IP del robot (se autodetecta)'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false',
                            description='Forzar modo simulación (true/false)'),
        DeclareLaunchArgument('onrobot_type', default_value='2fg7',
                            description='Tipo de gripper OnRobot'),
        DeclareLaunchArgument('launch_onrobot', default_value='true',
                            description='Lanzar control del gripper'),
        DeclareLaunchArgument('rviz_config', default_value='view_robot.rviz',
                            description='Configuración de RVIZ'),
        DeclareLaunchArgument('robot_detected', default_value='false',
                            description='Indica si se detectó robot físico'),
        DeclareLaunchArgument('simulation_mode', default_value='false',
                            description='Indica si estamos en modo simulación'),
        DeclareLaunchArgument('environment', default_value='auto',
                            choices=['auto', 'left', 'right', 'basic'],
                            description='Entorno a visualizar (auto=usar robot detectado)'),
        DeclareLaunchArgument('robot_side', default_value='none',
                            description='Lado del robot detectado automáticamente (solo lectura)'),
    ]

    detection_action = OpaqueFunction(function=detect_robot_and_configure)

    # ====== ROBOT DESCRIPTION ======
    robot_description_content = Command([
        'xacro ', PathJoinSubstitution([
            get_description_package_expression(),
            'urdf',
            get_urdf_filename_expression()
        ]),
        ' ur_type:=', LaunchConfiguration('ur_type'),
        ' robot_ip:=', LaunchConfiguration('robot_ip'),
        ' onrobot_type:=', LaunchConfiguration('onrobot_type'),
        ' use_fake_hardware:=', LaunchConfiguration('use_fake_hardware')
    ])

    robot_description = {"robot_description": robot_description_content}

    # ====== MAIN ROBOT STATE PUBLISHER ======
    main_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='main_robot_state_publisher',
        output='both',
        parameters=[robot_description],
        remappings=[
            ('/joint_states', '/merged_joint_states'),
        ],
        condition=IfCondition(LaunchConfiguration('robot_detected'))
    )

    # ====== JOINT STATE MERGER ======
    joint_state_merger = Node(
        package='ur_onrobot_control',
        executable='joint_state_merger.py',
        name='joint_state_merger',
        output='screen',
        condition=IfCondition(LaunchConfiguration('robot_detected'))
    )

    # ====== DRIVER UR MODIFICADO ======
    ur_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur_onrobot_control'),
                'launch',
                'ur_control.launch.py'
            ])
        ]),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
            'robot_ip': LaunchConfiguration('robot_ip'),
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
            'launch_rviz': 'false',
            'headless_mode': 'false',
            'launch_robot_state_publisher': 'false',
            'description_package': get_description_package_expression(),
            'description_file': get_urdf_filename_expression(),
        }.items(),
        condition=IfCondition(LaunchConfiguration('robot_detected'))
    )

    # ====== DRIVER ONROBOT ======
    onrobot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('onrobot_driver'),
                'launch',
                'onrobot_control.launch.py'
            ])
        ]),
        launch_arguments={
            'onrobot_type': LaunchConfiguration('onrobot_type'),
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
            'launch_rviz': 'false',
            'launch_rsp': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_onrobot'))
    )

    # ====== RVIZ ======
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('ur_onrobot_description'),
        'rviz',
        LaunchConfiguration('rviz_config')
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    return LaunchDescription([
        *declared_arguments,
        detection_action,
        joint_state_merger,
        main_robot_state_publisher,
        ur_launch,
        onrobot_launch,
        rviz_node
    ])