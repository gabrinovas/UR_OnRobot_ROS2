#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
    launch_joy = LaunchConfiguration('launch_joy', default='false')
    servo_yaml = os.path.join(moveit_config_dir, 'config', 'ur_onrobot_servo.yaml')
    
    # Parámetros para servo
    servo_params = [
        servo_yaml,
        {
            'use_sim_time': use_sim_time,
            'robot_description': LaunchConfiguration('robot_description'),
        },
    ]
    
    # Node de Servo
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        output='screen',
        parameters=servo_params,
        remappings=[
            ('/joint_states', '/merged_joint_states'),
            ('/servo_node/delta_twist_cmds', '/delta_twist_cmds'),
            ('/servo_node/delta_joint_cmds', '/delta_joint_cmds'),
        ]
    )
    
    # Node para convertir comandos de joystick (opcional)
    joy_to_twist_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        condition=IfCondition(launch_joy)
    )
    
    # Node para convertir joystick a comandos twist
    joy_to_servo_node = Node(
        package='moveit_servo',
        executable='joy_to_twist',
        name='joy_to_twist_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'scale_linear': 0.1,
            'scale_angular': 0.2,
            'scale_joint': 0.1,
        }],
        condition=IfCondition(launch_joy)
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'launch_joy',
            default_value='false',
            description='Launch joystick control nodes'
        ),
        servo_node,
        joy_to_twist_node,
        joy_to_servo_node,
    ])