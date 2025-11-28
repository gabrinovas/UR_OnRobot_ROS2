#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    moveit_config_package = 'ur_onrobot_moveit_config'
    moveit_config_dir = get_package_share_directory(moveit_config_package)
    
    # Parámetros
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_gui = LaunchConfiguration('use_gui', default='false')
    
    # Descripción del robot
    robot_description_content = Command([
        'xacro ', PathJoinSubstitution([
            FindPackageShare('ur_onrobot_description'),
            'urdf',
            'ur_onrobot.urdf.xacro'
        ]),
        ' use_fake_hardware:=true',  # Para contexto de planificación
        ' onrobot_type:=2fg7'  # Valor por defecto
    ])
    
    robot_description = {'robot_description': robot_description_content}
    
    # Node de Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Node de Joint State Publisher (para simulación con GUI)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(PythonExpression(["'", use_gui, "' == 'true'"]))
    )
    
    # Node de Joint State Publisher (sin GUI)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=IfCondition(PythonExpression(["'", use_gui, "' != 'true'"]))
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'use_gui',
            default_value='false',
            description='Use joint state publisher GUI'
        ),
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        joint_state_publisher_node,
    ])