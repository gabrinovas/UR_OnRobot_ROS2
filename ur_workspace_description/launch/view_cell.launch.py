#!/usr/bin/env python3
# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Show the furniture of one cell in RViz, without the robot."""

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from onrobot_description.config_loader import load_config, package_file, require
from onrobot_description.launch_arguments import declare_arguments
from ur_workspace_description.cells import cell_names

PACKAGE = 'ur_workspace_description'


def generate_launch_description():
    config = load_config(package_file(PACKAGE, 'config', 'ur_workspace_description.yaml'))
    arguments = declare_arguments(
        config, 'view_launch_arguments', choices={'cell': cell_names(with_environment=True)})
    description_file = package_file(PACKAGE, require(config, 'view', 'description_file'))
    rviz_config = package_file(PACKAGE, require(config, 'view', 'rviz_config'))
    cell = LaunchConfiguration('cell')
    command = Command([FindExecutable(name='xacro'), ' ', description_file, ' cell:=', cell])
    robot_description = {'robot_description': ParameterValue(command, value_type=str)}
    return LaunchDescription(arguments + [
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='both',
             parameters=[robot_description]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             name='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
             arguments=['-d', rviz_config]),
    ])
