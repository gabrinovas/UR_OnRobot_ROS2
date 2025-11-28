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
        
        # Logs específicos por tipo de environment
        if environment == 'left':
            print("\n⚠️  AVISO: Se solicitó environment=left con simulation_mode=false,")
            print("   pero no se detectó el robot left (192.168.1.105).")
            print("   Cambiando automáticamente a MODO SIMULACIÓN")
            
        elif environment == 'right':
            print("\n⚠️  AVISO: Se solicitó environment=right con simulation_mode=false,")
            print("   pero no se detectó el robot right (192.168.1.101).")
            print("   Cambiando automáticamente a MODO SIMULACIÓN")
        
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
            
            # Log específico del conflicto
            if environment == 'left':
                print(f"   - Se solicitó entorno LEFT pero solo está disponible robot {detected_side.upper()}")
            elif environment == 'right':
                print(f"   - Se solicitó entorno RIGHT pero solo está disponible robot {detected_side.upper()}")
                
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
            print(f"   - Robot físico {detected_side} disponible, usando entorno básico")
            return [SetLaunchConfiguration('use_fake_hardware', 'false'),
                    SetLaunchConfiguration('robot_ip', detected_ip),
                    SetLaunchConfiguration('robot_detected', 'true'),
                    SetLaunchConfiguration('simulation_mode', 'false'),
                    SetLaunchConfiguration('robot_side', detected_side)]
        
        # Caso normal: usar el robot detectado con entorno correspondiente
        print(f"🎯 Entorno {environment} seleccionado → Usando robot físico {detected_side}")
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
        
        # Log específico según el environment solicitado
        if environment == 'auto':
            print("\n💡 SOLUCIÓN: Para usar modo automático, desconecta uno de los robots")
        elif environment == 'left':
            print(f"\n💡 SOLUCIÓN: Desconecta el robot RIGHT o usa: 'use_fake_hardware:=true'")
        elif environment == 'right':
            print(f"\n💡 SOLUCIÓN: Desconecta el robot LEFT o usa: 'use_fake_hardware:=true'")
        elif environment == 'basic':
            print(f"\n💡 SOLUCIÓN: Desconecta uno de los robots o usa: 'use_fake_hardware:=true'")
            
        print("\n🚫 Cerrando ejecución...")
        
        # Retornar acciones que terminen la ejecución
        return [LogInfo(msg="Conflicto: ambos robots detectados"),
                SetLaunchConfiguration('robot_detected', 'false'),
                Shutdown(reason='Conflicto de robots detectados')]

def configure_urdf_settings(context):
    environment = LaunchConfiguration('environment').perform(context)
    simulation_mode = LaunchConfiguration('simulation_mode').perform(context)
    robot_side = LaunchConfiguration('robot_side').perform(context)
    
    # Determinar el entorno real a usar
    if environment == 'auto' and simulation_mode == 'false' and robot_side != 'none':
        actual_environment = robot_side
    elif environment == 'auto' and simulation_mode == 'true':
        actual_environment = 'basic'
    else:
        actual_environment = environment
    
    print(f"\n🎯 Configuración final:")
    print(f"   - Robot detectado: {robot_side}")
    print(f"   - Entorno solicitado: {environment}")
    print(f"   - Entorno real: {actual_environment}")
    print(f"   - Modo simulación: {simulation_mode}")
    
    # Log específico del entorno final
    if simulation_mode == 'true':
        if actual_environment == 'left':
            print("   - 🎮 SIMULACIÓN: Entorno LEFT")
        elif actual_environment == 'right':
            print("   - 🎮 SIMULACIÓN: Entorno RIGHT")
        elif actual_environment == 'basic':
            print("   - 🎮 SIMULACIÓN: Entorno BÁSICO")
    else:
        if actual_environment == 'left':
            print("   - 🤖 MODO REAL: Robot LEFT físico")
        elif actual_environment == 'right':
            print("   - 🤖 MODO REAL: Robot RIGHT físico")
        elif actual_environment == 'basic':
            print("   - 🤖 MODO REAL: Robot físico con entorno básico")
    
    # Determinar package y archivo URDF
    if actual_environment == 'basic':
        description_package = 'ur_onrobot_description'
        description_file = 'ur_onrobot.urdf.xacro'
    elif actual_environment == 'left':
        description_package = 'ur_onrobot_control'
        description_file = 'left_robot_with_environment.urdf.xacro'
    elif actual_environment == 'right':
        description_package = 'ur_onrobot_control'
        description_file = 'right_robot_with_environment.urdf.xacro'
    else:
        # Por defecto
        description_package = 'ur_onrobot_description'
        description_file = 'ur_onrobot.urdf.xacro'
    
    print(f"   - Package URDF: {description_package}")
    print(f"   - Archivo URDF: {description_file}")
    
    return [
        SetLaunchConfiguration('description_package', description_package),
        SetLaunchConfiguration('description_file', description_file)
    ]

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
        DeclareLaunchArgument('launch_moveit', default_value='true',
                            description='Lanzar MoveIt2'),
        DeclareLaunchArgument('moveit_config_package', default_value='ur_onrobot_moveit_config',
                            description='Paquete de configuración de MoveIt'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                            description='Usar tiempo de simulación'),
        DeclareLaunchArgument('launch_rviz', default_value='true',
                            description='Lanzar RViz2'),
        DeclareLaunchArgument('launch_servo', default_value='true',
                            description='Lanzar MoveIt Servo'),
    ]

    detection_action = OpaqueFunction(function=detect_robot_and_configure)
    urdf_config_action = OpaqueFunction(function=configure_urdf_settings)

    # ====== ROBOT DESCRIPTION ======
    robot_description_content = Command([
        'xacro ', PathJoinSubstitution([
            FindPackageShare(LaunchConfiguration('description_package')),
            'urdf',
            LaunchConfiguration('description_file')
        ]),
        ' ur_type:=', LaunchConfiguration('ur_type'),
        ' robot_ip:=', LaunchConfiguration('robot_ip'),
        ' onrobot_type:=', LaunchConfiguration('onrobot_type'),
        ' use_fake_hardware:=', LaunchConfiguration('use_fake_hardware')
    ])

    robot_description = {"robot_description": robot_description_content}

    # ====== ROBOT DESCRIPTION SEMANTIC ======
    robot_description_semantic_content = Command([
        'xacro ', PathJoinSubstitution([
            FindPackageShare(LaunchConfiguration('moveit_config_package')),
            'srdf',
            'ur_onrobot.srdf.xacro'
        ]),
        ' name:=ur_onrobot',
        ' prefix:=',
        ' onrobot_type:=', LaunchConfiguration('onrobot_type')
    ])

    # Configuración semántica como acción
    semantic_config_action = SetLaunchConfiguration(
        'robot_description_semantic',
        robot_description_semantic_content
    )

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
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('robot_detected'), "' == 'true'"]))
    )

    # ====== JOINT STATE MERGER ======
    joint_state_merger = Node(
        package='ur_onrobot_control',
        executable='joint_state_merger.py',
        name='joint_state_merger',
        output='screen',
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('robot_detected'), "' == 'true'"]))
    )

    # ====== STATIC TRANSFORM PUBLISHER ======
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base_link'],
        output='screen',
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('robot_detected'), "' == 'true'"]))
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
            'description_package': LaunchConfiguration('description_package'),
            'description_file': LaunchConfiguration('description_file'),
        }.items(),
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('robot_detected'), "' == 'true'"]))
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
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('launch_onrobot'), "' == 'true'"]))
    )

    # ====== MOVEIT LAUNCH ======
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(LaunchConfiguration('moveit_config_package')),
                'launch',
                'move_group.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_description': robot_description_content,
            'robot_description_semantic': LaunchConfiguration('robot_description_semantic'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'publish_monitored_planning_scene': 'true',
            'publish_static_transform': 'false',  # Ya lo publicamos arriba
        }.items(),
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('launch_moveit'), "' == 'true' and ",
            "'", LaunchConfiguration('robot_detected'), "' == 'true'"
        ]))
    )

    # ====== MOVEIT SERVO ======
    moveit_servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(LaunchConfiguration('moveit_config_package')),
                'launch',
                'servo.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': robot_description_content,
            'launch_joy': 'false',
        }.items(),
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('launch_moveit'), "' == 'true' and ",
            "'", LaunchConfiguration('launch_servo'), "' == 'true' and ",
            "'", LaunchConfiguration('simulation_mode'), "' == 'false'"
        ]))
    )

    # ====== RVIZ CON MOVEIT ======
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('ur_onrobot_moveit_config'),
        'rviz',
        'view_robot.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[
            robot_description,
            {'robot_description_semantic': LaunchConfiguration('robot_description_semantic')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('launch_rviz'), "' == 'true' and ",
            "'", LaunchConfiguration('launch_moveit'), "' == 'true' and ",
            "'", LaunchConfiguration('robot_detected'), "' == 'true'"
        ]))
    )

    # ====== RVIZ SIN MOVEIT (fallback) ======
    rviz_fallback_config_path = PathJoinSubstitution([
        FindPackageShare('ur_onrobot_description'),
        'rviz',
        LaunchConfiguration('rviz_config')
    ])

    rviz_fallback_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_fallback_config_path],
        parameters=[robot_description],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('launch_rviz'), "' == 'true' and ",
            "'", LaunchConfiguration('launch_moveit'), "' != 'true' and ",
            "'", LaunchConfiguration('robot_detected'), "' == 'true'"
        ]))
    )

    return LaunchDescription([
        *declared_arguments,
        detection_action,
        urdf_config_action,
        semantic_config_action,
        joint_state_merger,
        main_robot_state_publisher,
        static_transform_publisher,
        ur_launch,
        onrobot_launch,
        moveit_launch,
        moveit_servo_launch,
        rviz_node,
        rviz_fallback_node
    ])