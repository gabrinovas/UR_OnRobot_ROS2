#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ur_onrobot_bringup.launch.py - Bringup común de la celda UR + OnRobot, en simulación o con el robot físico.

Lo incluyen start_simulation.launch.py y start_real_robot.launch.py, que declaran sus argumentos con sus valores
por defecto y fijan aquí use_fake_hardware. Lanza:
  - joint_state_merger y el robot_state_publisher único (robot + pinza + entorno);
  - el driver del UR (ur_control.launch.py) con los controladores del entorno;
  - el driver de la pinza OnRobot en su propio controller manager (/onrobot), si launch_onrobot;
  - el watchdog de seguridad, si launch_watchdog;
  - RViz2, si launch_rviz.
Todos los argumentos son obligatorios: los valores por defecto viven en los dos lanzadores que lo incluyen.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

ARGUMENTS = (
    'ur_type', 'robot_ip', 'onrobot_type', 'launch_onrobot', 'sim_env', 'launch_rviz', 'rviz_config', 'tf_prefix',
    'use_fake_hardware', 'launch_watchdog', 'initial_joint_controller', 'controller_spawner_timeout',
    'use_gripper_action_controller', 'load_freedrive_controller',
)
DESCRIPTION_PACKAGE = 'ur_onrobot_description'
DESCRIPTION_FILES = {  # URDF de cada entorno (sim_env); cualquier otro valor usa el básico
    'left': 'left_robot_with_environment.urdf.xacro',
    'right': 'right_robot_with_environment.urdf.xacro',
}
BASIC_DESCRIPTION_FILE = 'ur_onrobot.urdf.xacro'


def select_description(context):
    """Elige el URDF del entorno y lo anuncia por consola, con el texto de cada modo."""
    sim_env = LaunchConfiguration('sim_env').perform(context)
    ur_type = LaunchConfiguration('ur_type').perform(context)
    onrobot_type = LaunchConfiguration('onrobot_type').perform(context)
    launch_rviz = LaunchConfiguration('launch_rviz').perform(context)
    simulation = LaunchConfiguration('use_fake_hardware').perform(context).lower() == 'true'
    description_file = DESCRIPTION_FILES.get(sim_env, BASIC_DESCRIPTION_FILE)

    if simulation:
        print("\n🎮 CONFIGURANDO SIMULACIÓN")
        print(f"   - Entorno de simulación: {sim_env}")
        print(f"   - Robot UR: {ur_type}")
        print(f"   - Gripper: {onrobot_type}")
        print(f"   - Launch RViz: {launch_rviz}")
        basic = f"   - URDF: Básico ({DESCRIPTION_PACKAGE}/{description_file})"
    else:
        print("\n🤖 CONFIGURANDO ROBOT REAL (ROS 2 Humble / Dockerfile.aimen)")
        print(f"   - Entorno de trabajo: {sim_env}")
        print(f"   - Robot UR:           {ur_type} (IP: {LaunchConfiguration('robot_ip').perform(context)})")
        print(f"   - Gripper OnRobot:    {onrobot_type}")
        print(f"   - Launch RViz:        {launch_rviz}")
        basic = f"   - URDF: Básico sin entorno adicional ({DESCRIPTION_PACKAGE}/{description_file})"
    if sim_env in DESCRIPTION_FILES:
        print(f"   - URDF: Entorno {sim_env.upper()} ({DESCRIPTION_PACKAGE}/{description_file})")
    else:
        print(basic)

    return [
        SetLaunchConfiguration('description_package', DESCRIPTION_PACKAGE),
        SetLaunchConfiguration('description_file', description_file),
        SetLaunchConfiguration('bringup_launch_rviz', launch_rviz),
    ]


def include(package, launch_file, arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare(package), 'launch', launch_file])]),
        launch_arguments=arguments.items())


def generate_launch_description():
    declared_arguments = [DeclareLaunchArgument(name) for name in ARGUMENTS]

    # Robot description (URDF del entorno ya elegido)
    robot_description_content = Command([
        'xacro ', PathJoinSubstitution([
            FindPackageShare(LaunchConfiguration('description_package')),
            'urdf',
            LaunchConfiguration('description_file')
        ]),
        ' ur_type:=', LaunchConfiguration('ur_type'),
        ' robot_ip:=', LaunchConfiguration('robot_ip'),
        ' onrobot_type:=', LaunchConfiguration('onrobot_type'),
        ' use_fake_hardware:=', LaunchConfiguration('use_fake_hardware'),
        ' tf_prefix:=', LaunchConfiguration('tf_prefix')
    ])

    # ====== MAIN ROBOT STATE PUBLISHER ======
    # Publica la cinemática completa del robot + entorno + pinza a partir de /merged_joint_states
    main_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='main_robot_state_publisher',
        output='both',
        parameters=[{"robot_description": robot_description_content}],
        remappings=[('/joint_states', '/merged_joint_states')]
    )

    # ====== JOINT STATE MERGER ======
    joint_state_merger = Node(
        package='ur_onrobot_control',
        executable='joint_state_merger.py',
        name='joint_state_merger',
        output='screen',
        parameters=[{
            'sim_env': LaunchConfiguration('sim_env'),
            'tf_prefix': LaunchConfiguration('tf_prefix')
        }]
    )

    # ====== DRIVER UR (GroupAction para aislar launch configurations) ======
    ur_launch_group = GroupAction([include('ur_onrobot_control', 'ur_control.launch.py', {
        'ur_type': LaunchConfiguration('ur_type'),
        'robot_ip': LaunchConfiguration('robot_ip'),
        'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
        'initial_joint_controller': LaunchConfiguration('initial_joint_controller'),
        'launch_rviz': 'false',
        'headless_mode': 'true',
        'launch_robot_state_publisher': 'false',
        'tf_prefix': LaunchConfiguration('tf_prefix'),
        'runtime_config_package': 'ur_onrobot_control',
        'controllers_file': 'ur_onrobot_controllers.yaml',  # los mismos controladores en todos los entornos
        'controller_spawner_timeout': LaunchConfiguration('controller_spawner_timeout'),
        'load_freedrive_controller': LaunchConfiguration('load_freedrive_controller'),
        # Gripper runs in its own controller manager (/onrobot) when launch_onrobot is true:
        # keep it out of the UR controller manager so a gripper failure cannot kill the UR driver.
        'include_gripper_ros2_control': PythonExpression(
            ["'false' if '", LaunchConfiguration('launch_onrobot'), "'.lower() == 'true' else 'true'"]),
    })])

    # ====== DRIVER ONROBOT (GroupAction para aislar launch configurations) ======
    onrobot_launch_group = GroupAction(
        [include('onrobot_driver', 'onrobot_control.launch.py', {
            'onrobot_type': LaunchConfiguration('onrobot_type'),
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
            'launch_rviz': 'false',
            'launch_rsp': 'false',
            'tf_prefix': LaunchConfiguration('tf_prefix'),
            'use_gripper_action_controller': LaunchConfiguration('use_gripper_action_controller'),
        })],
        condition=IfCondition(LaunchConfiguration('launch_onrobot'))
    )

    # ====== WATCHDOG DE SEGURIDAD INDUSTRIAL ======
    watchdog_launch_group = GroupAction(
        [include('ur_onrobot_control', 'ur_onrobot_watchdog.launch.py', {
            'onrobot_type': LaunchConfiguration('onrobot_type'),
        })],
        condition=IfCondition(LaunchConfiguration('launch_watchdog'))
    )

    # ====== RVIZ ======
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('ur_onrobot_description'), 'rviz', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('bringup_launch_rviz'))
    )

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=select_description),
        joint_state_merger,
        main_robot_state_publisher,
        ur_launch_group,
        onrobot_launch_group,
        watchdog_launch_group,
        rviz_node
    ])
