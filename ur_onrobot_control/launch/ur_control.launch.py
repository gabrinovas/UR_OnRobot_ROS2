#!/usr/bin/env python3
# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Run the driver of a UR arm with an OnRobot gripper: control node, controllers, helper nodes."""

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from onrobot_description.grippers import gripper_names
from onrobot_description.launch_arguments import declare_arguments
from ur_onrobot_control.config import control_config, ur_types
from ur_onrobot_control.launch_parts.spawners import controller_spawners
from ur_onrobot_control.launch_parts.ur_driver import ur_driver_nodes
from ur_workspace_description.cells import cell_names


def launch_setup(context, *args, **kwargs):
    config = control_config('ur_control')
    return ur_driver_nodes(context, config) + controller_spawners(context, config)


def generate_launch_description():
    arguments = declare_arguments(
        control_config('ur_control'), 'launch_arguments',
        choices={'ur_type': ur_types(), 'onrobot_type': gripper_names(), 'cell': cell_names()})
    return LaunchDescription(arguments + [OpaqueFunction(function=launch_setup)])
