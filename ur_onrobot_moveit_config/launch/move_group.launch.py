#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Configuración de MoveIt
    moveit_config_package = 'ur_onrobot_moveit_config'
    moveit_config_dir = get_package_share_directory(moveit_config_package)
    
    # Parámetros
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    publish_monitored_planning_scene = LaunchConfiguration('publish_monitored_planning_scene', default='true')
    
    # Archivos de configuración
    kinematics_yaml = os.path.join(moveit_config_dir, 'config', 'kinematics.yaml')
    joint_limits_yaml = os.path.join(moveit_config_dir, 'config', 'joint_limits.yaml')
    ompl_planning_yaml = os.path.join(moveit_config_dir, 'config', 'ompl_planning.yaml')
    moveit_controllers_yaml = os.path.join(moveit_config_dir, 'config', 'moveit_controllers.yaml')
    sensors_yaml = os.path.join(moveit_config_dir, 'config', 'sensors_3d.yaml')
    
    # Robot description para standalone
    robot_description_content = Command([
        'xacro ', PathJoinSubstitution([
            FindPackageShare('ur_onrobot_description'),
            'urdf',
            'ur_onrobot.urdf.xacro'
        ]),
        ' use_fake_hardware:=true',
        ' onrobot_type:=2fg7'
    ])
    
    robot_description = {'robot_description': robot_description_content}
    
    # Robot description semantic
    robot_description_semantic_content = Command([
        'xacro ', PathJoinSubstitution([
            FindPackageShare(moveit_config_package),
            'srdf',
            'ur_onrobot.srdf.xacro'
        ]),
        ' name:=ur_onrobot',
        ' prefix:=',
        ' onrobot_type:=2fg7'
    ])
    
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_content}
    
    # Parámetros para move_group
    move_group_params = [
        robot_description,
        robot_description_semantic,
        {
            'use_sim_time': use_sim_time,
            'publish_monitored_planning_scene': publish_monitored_planning_scene,
            'allow_trajectory_execution': True,
            'max_safe_path_cost': 1,
            'jiggle_fraction': 0.05,
            'publish_planning_scene': True,
            'publish_geometry_updates': True,
            'publish_state_updates': True,
            'publish_transforms_updates': True,
            'monitor_dynamics': False,
        },
        kinematics_yaml,
        joint_limits_yaml, 
        ompl_planning_yaml,
        moveit_controllers_yaml,
        sensors_yaml,
    ]
    
    # Node de MoveGroup
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=move_group_params,
        remappings=[
            ('/joint_states', '/merged_joint_states'),
        ]
    )
    
    # Node de servidor de estado del robot
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            robot_description
        ],
        remappings=[
            ('/joint_states', '/merged_joint_states'),
        ]
    )
    
    # Node para publicar transformaciones estáticas
    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base_link']
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'publish_monitored_planning_scene',
            default_value='true',
            description='Publish monitored planning scene'
        ),
        move_group_node,
        robot_state_publisher_node,
        static_transform_publisher_node,
    ])