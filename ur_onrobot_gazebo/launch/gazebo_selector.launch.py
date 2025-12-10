#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def gazebo_selector(context):
    """
    Selector para entornos de Gazebo
    """
    sim_env = LaunchConfiguration('sim_env').perform(context)
    ur_type = LaunchConfiguration('ur_type').perform(context)
    
    print(f"\n🎮 Configurando Gazebo para entorno: {sim_env.upper()}")
    
    actions = []
    
    # Incluir el launch file apropiado
    if sim_env == 'left':
        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ur_onrobot_gazebo'),
                    'launch',
                    'gazebo_left.launch.py'
                ])
            ]),
            launch_arguments={
                'ur_type': ur_type,
                'use_sim_time': 'true',
            }.items()
        )
        actions.append(gazebo_launch)
    elif sim_env == 'right':
        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ur_onrobot_gazebo'),
                    'launch',
                    'gazebo_right.launch.py'
                ])
            ]),
            launch_arguments={
                'ur_type': ur_type,
                'use_sim_time': 'true',
            }.items()
        )
        actions.append(gazebo_launch)
    else:
        # Basic environment - solo robot
        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ur_onrobot_gazebo'),
                    'launch',
                    'gazebo_basic.launch.py'
                ])
            ]),
            launch_arguments={
                'ur_type': ur_type,
                'use_sim_time': 'true',
            }.items()
        )
        actions.append(gazebo_launch)
    
    actions.append(LogInfo(msg=f"Entorno Gazebo '{sim_env}' iniciado"))
    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim_env', default_value='basic',
                            description='Entorno de simulación: basic, left, right'),
        DeclareLaunchArgument('ur_type', default_value='ur5e',
                            description='Tipo de robot UR'),
        DeclareLaunchArgument('onrobot_type', default_value='2fg7',
                            description='Tipo de gripper OnRobot'),
        OpaqueFunction(function=gazebo_selector)
    ])