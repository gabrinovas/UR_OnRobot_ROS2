#!/usr/bin/env python3
# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Run a whole cell, simulated or real.

Starts the joint state merger, the single robot_state_publisher (arm, gripper and furniture), the
driver of the arm, the driver of the gripper in a controller manager of its own, the safety
watchdog and RViz. Every value comes from config/bringup.yaml.
"""

from launch import LaunchDescription
from launch.actions import GroupAction, LogInfo, OpaqueFunction
from launch.logging import get_logger
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from onrobot_description.config_loader import package_file, require
from onrobot_description.grippers import gripper_names
from onrobot_description.launch_arguments import as_launch_text, declare_arguments
from ur_onrobot_control.config import control_config, include, is_true, launch_values
from ur_onrobot_control.config import PACKAGE, section, ur_types
from ur_onrobot_description.description_command import robot_description_command
from ur_workspace_description.cells import cell_names
from ur_workspace_description.robot_choice import report, resolve_robot


def resolved_values(context, config):
    """
    Return the launch values of the cell.

    The mode of the cell fixes the values it owns, and the robot addresses file gives the address
    and the kinematics calibration of the real robot and the Compute Box of its gripper.
    """
    values = launch_values(context, config)
    simulated = is_true(values['use_fake_hardware'])
    mode = require(config, 'modes', 'simulation' if simulated else 'real')
    values.update({name: as_launch_text(value) for name, value in mode.items()})
    choice = resolve_robot(values['cell'], values['robot_ip'], simulated)
    report(choice, get_logger('launch.user'))
    values.update(robot_ip=choice.address, kinematics_params_file=choice.calibration,
                  gripper_ip_address=choice.compute_box)
    return values


def launch_setup(context, *args, **kwargs):
    config = control_config('bringup')
    values = resolved_values(context, config)
    simulated = is_true(values['use_fake_hardware'])
    gripper_alone = is_true(values['launch_onrobot'])
    topics = section(config, 'joint_states')
    ur_driver = section(config, 'ur_driver')
    gripper_driver = section(config, 'gripper_driver')
    rviz = section(config, 'rviz_config_directory')

    description = Command(robot_description_command(
        cell=values['cell'], ur_type=values['ur_type'], onrobot_type=values['onrobot_type'],
        robot_ip=values['robot_ip'], use_fake_hardware=values['use_fake_hardware'],
        tf_prefix=values['tf_prefix'], include_gripper_ros2_control='true',
        kinematics_parameters_file=values['kinematics_params_file'] or None,
        gripper_ip_address=values['gripper_ip_address'] or None))

    actions = [
        LogInfo(msg=(
            f"Cell '{values['cell']}': {values['ur_type']} with {values['onrobot_type']}, "
            + ('simulated' if simulated else f"real robot at {values['robot_ip']}"))),
        Node(package=PACKAGE, executable='joint_state_merger.py', name='joint_state_merger',
             output='screen',
             parameters=[package_file(PACKAGE, require(config, 'merger_config')),
                         {'tf_prefix': values['tf_prefix']}]),
        # Kinematics of the arm, the gripper and the furniture, from the merged joint states.
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='main_robot_state_publisher', output='both',
             parameters=[{'robot_description': ParameterValue(description, value_type=str)}],
             remappings=[(require(topics, 'arm_topic'), require(topics, 'merged_topic'))]),
        # The included launch files run in a group so that their arguments stay their own.
        GroupAction([include(PACKAGE, 'ur_control.launch.py', {
            'ur_type': values['ur_type'],
            'robot_ip': values['robot_ip'],
            'kinematics_params_file': values['kinematics_params_file'],
            'gripper_ip_address': values['gripper_ip_address'],
            'cell': values['cell'],
            'onrobot_type': values['onrobot_type'],
            'use_fake_hardware': values['use_fake_hardware'],
            'tf_prefix': values['tf_prefix'],
            'initial_joint_controller': values['initial_joint_controller'],
            'controller_spawner_timeout': values['controller_spawner_timeout'],
            'load_freedrive_controller': values['load_freedrive_controller'],
            'launch_rviz': as_launch_text(require(ur_driver, 'launch_rviz')),
            'headless_mode': as_launch_text(require(ur_driver, 'headless_mode')),
            'launch_robot_state_publisher':
                as_launch_text(require(ur_driver, 'launch_robot_state_publisher')),
            # A gripper with a controller manager of its own stays out of the one of the arm.
            'include_gripper_ros2_control': as_launch_text(not gripper_alone),
        })]),
    ]
    if gripper_alone:
        actions.append(GroupAction([include('onrobot_driver', 'onrobot_control.launch.py', {
            'onrobot_type': values['onrobot_type'],
            'use_fake_hardware': values['use_fake_hardware'],
            'prefix': values['tf_prefix'],
            'ip_address': values['gripper_ip_address'],
            'launch_rviz': as_launch_text(require(gripper_driver, 'launch_rviz')),
            'launch_rsp': as_launch_text(require(gripper_driver, 'launch_rsp')),
            'use_gripper_action_controller':
                as_launch_text(require(gripper_driver, 'use_gripper_action_controller')),
        })]))
    if is_true(values['launch_watchdog']):
        actions.append(GroupAction([include(PACKAGE, 'ur_onrobot_watchdog.launch.py', {
            'onrobot_type': values['onrobot_type'], 'tf_prefix': values['tf_prefix']})]))
    if is_true(values['launch_rviz']):
        layout = package_file(
            require(rviz, 'package'), require(rviz, 'directory'), values['rviz_config'])
        actions.append(Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
                            arguments=['-d', layout]))
    return actions


def generate_launch_description():
    arguments = declare_arguments(
        control_config('bringup'), 'launch_arguments',
        choices={'ur_type': ur_types(), 'onrobot_type': gripper_names(), 'cell': cell_names()})
    return LaunchDescription(arguments + [OpaqueFunction(function=launch_setup)])
