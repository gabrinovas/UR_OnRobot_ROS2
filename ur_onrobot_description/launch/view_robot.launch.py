#!/usr/bin/env python3
# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Show a UR arm with an OnRobot gripper in RViz, with its joints driven from a small window."""

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from onrobot_description.config_loader import package_file, require
from onrobot_description.grippers import gripper_names
from onrobot_description.launch_arguments import as_launch_text, declare_arguments
from ur_onrobot_description.description_command import description_config
from ur_onrobot_description.description_command import robot_description_command
from ur_workspace_description.cells import cell_names
from ur_workspace_description.robot_addresses import settings

PACKAGE = 'ur_onrobot_description'


def launch_setup(context, *args, **kwargs):
    cell = LaunchConfiguration('cell').perform(context)
    view = dict(require(description_config(), 'view'), __file__=description_config()['__file__'])
    command = robot_description_command(
        cell=cell,
        ur_type=LaunchConfiguration('ur_type').perform(context),
        onrobot_type=LaunchConfiguration('onrobot_type'),
        robot_ip=str(require(settings(), 'simulation_address')),  # the viewer talks to no robot
        use_fake_hardware=as_launch_text(require(view, 'use_fake_hardware')),
        tf_prefix=LaunchConfiguration('tf_prefix'),
        include_gripper_ros2_control=as_launch_text(require(view, 'include_gripper_ros2_control')))
    robot_description = {'robot_description': ParameterValue(Command(command), value_type=str)}
    rviz_config = package_file(PACKAGE, require(view, 'rviz_config'))
    return [
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='both',
             parameters=[robot_description]),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
             arguments=['-d', rviz_config]),
    ]


def generate_launch_description():
    arguments = declare_arguments(
        description_config(), 'view_launch_arguments',
        choices={'onrobot_type': gripper_names(), 'cell': cell_names()})
    return LaunchDescription(arguments + [OpaqueFunction(function=launch_setup)])
