#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Arguments
    world_name = LaunchConfiguration('world', default='empty.world')
    ur_type = LaunchConfiguration('ur_type', default='ur5e')
    onrobot_type = LaunchConfiguration('onrobot_type', default='2fg7')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                FindPackageShare('ur_onrobot_gazebo'),
                'worlds',
                world_name
            ]),
            'verbose': 'false',
            'pause': 'false',
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'right_ur_robot',
            '-file', PathJoinSubstitution([
                FindPackageShare('ur_onrobot_gazebo'),
                'description',
                'urdf',
                'right_robot_gazebo.urdf.xacro'
            ]),
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ],
        output='screen'
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': PathJoinSubstitution([
                FindPackageShare('ur_onrobot_gazebo'),
                'description',
                'urdf',
                'right_robot_gazebo.urdf.xacro'
            ]),
            'use_sim_time': use_sim_time,
        }]
    )
    
    # Joint State Publisher GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Spawn controllers
    spawn_controllers = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            'scaled_joint_trajectory_controller'
        ],
        output='screen'
    )
    
    # Delay controller spawning after spawn_robot
    delayed_spawn_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[spawn_controllers],
        )
    )
    
    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher,
        joint_state_publisher_gui,
        spawn_robot,
        delayed_spawn_controllers,
    ])