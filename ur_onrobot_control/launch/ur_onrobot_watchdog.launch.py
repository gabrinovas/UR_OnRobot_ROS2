#!/usr/bin/env python3
# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Run the safety watchdog of a UR arm with an OnRobot gripper."""

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from onrobot_description.config_loader import require
from onrobot_description.grippers import gripper_names, load_gripper
from onrobot_description.launch_arguments import declare_arguments
import onrobot_driver
from ur_onrobot_control.config import control_config, is_true, launch_values, PACKAGE, section


def launch_setup(context, *args, **kwargs):
    config = control_config('safety_watchdog')
    values = launch_values(context, config)
    gripper = values['onrobot_type']
    kind = require(load_gripper(gripper), 'kind')
    driver_gripper = onrobot_driver.load_driver_gripper(gripper)
    names = {
        'namespace': require(onrobot_driver.driver_config(), 'launch_arguments', 'ns', 'default'),
        'controller': require(driver_gripper, 'controllers', 'action')[0],
    }
    parameters = dict(require(config, 'node'))
    parameters.update(section(config, 'holds', kind))
    parameters.pop('__file__')
    parameters.update({
        'onrobot_type': gripper,
        'gripper_kind': kind,
        'joints': onrobot_driver.gripper_joints(gripper, values['tf_prefix']),
        'gripper_action': require(config, 'gripper_action').format(**names),
        'reset_power_service': require(config, 'reset_power_service').format(**names),
        'auto_recover': is_true(values['auto_recover']),
        'safety_mode_topic': values['safety_mode_topic'],
        'robot_mode_topic': values['robot_mode_topic'],
        'joint_states_topic': values['joint_states_topic'],
    })
    return [Node(package=PACKAGE, executable='ur_onrobot_safety_watchdog',
                 name='ur_onrobot_safety_watchdog', output='screen', parameters=[parameters])]


def generate_launch_description():
    arguments = declare_arguments(
        control_config('safety_watchdog'), 'launch_arguments',
        choices={'onrobot_type': gripper_names()})
    return LaunchDescription(arguments + [OpaqueFunction(function=launch_setup)])
