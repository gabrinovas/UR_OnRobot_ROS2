#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration, IncludeLaunchDescription, LogInfo, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

import subprocess
import sys

def detect_robot_and_configure(context):
    use_fake = LaunchConfiguration('use_fake_hardware').perform(context)
    robot_side = LaunchConfiguration('robot_side').perform(context)
    
    # Si se fuerza simulación, retornar inmediatamente
    if use_fake == "true":
        print("\n🔄 Modo simulación forzado → MODO SIMULACIÓN")
        return [SetLaunchConfiguration('use_fake_hardware', 'true'),
                SetLaunchConfiguration('robot_ip', '127.0.0.1'),
                SetLaunchConfiguration('robot_detected', 'true'),
                SetLaunchConfiguration('simulation_mode', 'true')]

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
                SetLaunchConfiguration('simulation_mode', 'true')]
    
    elif len(detected_robots) == 1:
        detected_side, detected_ip = detected_robots[0]
        print(f"\n✅ Robot {detected_side} detectado → MODO REAL")
        
        # Si el usuario especificó un lado diferente al detectado, usar simulación
        if robot_side != 'auto' and robot_side != detected_side:
            print(f"⚠️  Usuario solicitó robot '{robot_side}' pero solo está disponible '{detected_side}'")
            print("🔁 Cambiando a modo simulación para el escenario solicitado")
            return [SetLaunchConfiguration('use_fake_hardware', 'true'),
                    SetLaunchConfiguration('robot_ip', '127.0.0.1'),
                    SetLaunchConfiguration('robot_detected', 'true'),
                    SetLaunchConfiguration('simulation_mode', 'true')]
        
        return [SetLaunchConfiguration('use_fake_hardware', 'false'),
                SetLaunchConfiguration('robot_ip', detected_ip),
                SetLaunchConfiguration('robot_detected', 'true'),
                SetLaunchConfiguration('simulation_mode', 'false'),
                SetLaunchConfiguration('detected_side', detected_side)]
    
    else:  # Ambos robots detectados
        print("\n❌ AMBOS ROBOTS DETECTADOS - CONFLICTO")
        print("Se detectaron ambos robots físicos:")
        for side, ip in detected_robots:
            print(f"  - Robot {side}: {ip}")
        print("\n💡 SOLUCIÓN: Especifica qué robot usar con el argumento:")
        print("   'robot_side:=left' o 'robot_side:=right'")
        print("\n🚫 Cerrando ejecución...")
        
        # Retornar acciones que terminen la ejecución
        return [LogInfo(msg="Conflicto: ambos robots detectados"),
                SetLaunchConfiguration('robot_detected', 'false'),
                Shutdown(reason='Conflicto de robots detectados')]

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
        DeclareLaunchArgument('robot_side', default_value='auto',
                            choices=['auto', 'left', 'right', 'base'],
                            description='Lado del robot (auto=autodetectar, base=escenario base)'),
        DeclareLaunchArgument('environment', default_value='with_environment',
                            choices=['with_environment', 'basic'],
                            description='Tipo de entorno a visualizar'),
        DeclareLaunchArgument('detected_side', default_value='none',
                            description='Lado detectado automáticamente'),
    ]

    detection_action = OpaqueFunction(function=detect_robot_and_configure)

    # ====== SELECCIÓN DE ARCHIVOS URDF SEGÚN CONFIGURACIÓN ======
    def get_urdf_filename(context):
        robot_side = LaunchConfiguration('robot_side').perform(context)
        environment = LaunchConfiguration('environment').perform(context)
        simulation_mode = LaunchConfiguration('simulation_mode').perform(context)
        detected_side = LaunchConfiguration('detected_side').perform(context)
        
        # Determinar el lado real a usar
        actual_side = detected_side if detected_side != 'none' else robot_side
        if actual_side == 'auto' and simulation_mode == 'false':
            actual_side = detected_side
        
        print(f"\n🎯 Configuración:")
        print(f"   - Lado solicitado: {robot_side}")
        print(f"   - Lado detectado: {detected_side}")
        print(f"   - Lado real: {actual_side}")
        print(f"   - Entorno: {environment}")
        print(f"   - Modo simulación: {simulation_mode}")
        
        # Seleccionar archivo URDF
        if environment == 'basic':
            return 'ur_onrobot.urdf.xacro'
        elif actual_side == 'left':
            return 'left_robot_with_environment.urdf.xacro'
        elif actual_side == 'right':
            return 'right_robot_with_environment.urdf.xacro'
        else:  # base o cualquier otro caso
            return 'ur_onrobot.urdf.xacro'

    # ====== ROBOT DESCRIPTION ======
    robot_description_content = Command([
        'xacro ', PathJoinSubstitution([
            FindPackageShare('ur_onrobot_control' if LaunchConfiguration('environment').perform != 'basic' else 'ur_onrobot_description'),
            'urdf',
            PythonExpression(['"', get_urdf_filename, '"'])
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
            'description_package': 'ur_onrobot_control' if PythonExpression([
                LaunchConfiguration('environment'), ' != "basic"'
            ]) else 'ur_onrobot_description',
            'description_file': PythonExpression(['"', get_urdf_filename, '"']),
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