#!/usr/bin/env python3
# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Run MoveIt Servo (Cartesian jogging) for a UR arm with an OnRobot gripper in a cell."""

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from onrobot_description.config_loader import require
from onrobot_description.grippers import gripper_names
from onrobot_description.launch_arguments import declare_arguments
from ur_onrobot_moveit_config.parameters import load_file, moveit_config
from ur_onrobot_moveit_config.parameters import robot_description, semantic_description
from ur_workspace_description.cells import cell_names


def launch_setup(context, *args, **kwargs):
    config = moveit_config()
    values = {name: LaunchConfiguration(name).perform(context)
              for name in require(config, 'servo_launch_arguments')}
    return [
        Node(package='moveit_servo', executable='servo_node_main', name='servo_node',
             output='screen',
             parameters=[
                 {'moveit_servo': load_file(config, 'servo')},
                 robot_description(values), semantic_description(config, values),
                 load_file(config, 'kinematics'), load_file(config, 'joint_limits')]),
        Node(package='joy', executable='joy_node', name='joy_node', output='screen',
             condition=IfCondition(LaunchConfiguration('launch_joy'))),
    ]


def generate_launch_description():
    arguments = declare_arguments(
        moveit_config(), 'servo_launch_arguments',
        choices={'onrobot_type': gripper_names(), 'cell': cell_names()})
    return LaunchDescription(arguments + [OpaqueFunction(function=launch_setup)])
