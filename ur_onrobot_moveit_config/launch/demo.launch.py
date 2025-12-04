#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Demo launch file for testing
    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur_onrobot_moveit_config'),
                'launch',
                'ur_onrobot_moveit.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'launch_rviz': 'true',
            'launch_servo': 'false',
        }.items()
    )

    return LaunchDescription([demo_launch])