#!/usr/bin/env python3
# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Run MoveIt (move_group and RViz) for a UR arm with an OnRobot gripper in a cell."""

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from onrobot_description.config_loader import package_file, require
from onrobot_description.grippers import gripper_names
from onrobot_description.launch_arguments import declare_arguments
from ur_onrobot_moveit_config.parameters import load_file, moveit_config, PACKAGE
from ur_onrobot_moveit_config.parameters import robot_description, semantic_description
from ur_workspace_description.cells import cell_names


def launch_setup(context, *args, **kwargs):
    config = moveit_config()
    values = {name: LaunchConfiguration(name).perform(context)
              for name in require(config, 'launch_arguments')}
    description = robot_description(values)
    semantic = semantic_description(config, values)
    kinematics = {'robot_description_kinematics': load_file(config, 'kinematics')}
    joint_limits = load_file(config, 'joint_limits')

    pipelines = require(config, 'planning', 'pipelines')
    planning = {'planning_pipelines': list(pipelines),
                'default_planning_pipeline': require(config, 'planning', 'default_pipeline')}
    for pipeline, key in pipelines.items():
        planning[pipeline] = load_file(config, key)

    # MoveIt sends its trajectories to the controller marked as default: the one asked for.
    controllers = load_file(config, 'moveit_controllers')
    section = controllers[require(config, 'controller_manager_section')]
    for controller in require(config, 'arm_controllers'):
        section[controller]['default'] = controller == values['trajectory_controller']

    joint_states = [('joint_states', values['joint_states_topic']),
                    ('/joint_states', values['joint_states_topic'])]
    return [
        Node(package='moveit_ros_move_group', executable='move_group', output='screen',
             remappings=joint_states,
             parameters=[
                 description, semantic, kinematics,
                 {'robot_description_planning': joint_limits}, {'joint_limits': joint_limits},
                 planning, dict(require(config, 'trajectory_execution')), controllers,
                 dict(require(config, 'planning_scene_monitor'))]),
        Node(package='rviz2', executable='rviz2', name='rviz2_moveit', output='screen',
             arguments=['-d', package_file(PACKAGE, require(config, 'files', 'rviz'))],
             remappings=joint_states,
             parameters=[description, semantic, kinematics, planning],
             condition=IfCondition(LaunchConfiguration('launch_rviz'))),
    ]


def generate_launch_description():
    arguments = declare_arguments(
        moveit_config(), 'launch_arguments',
        choices={'onrobot_type': gripper_names(), 'cell': cell_names()})
    return LaunchDescription(arguments + [OpaqueFunction(function=launch_setup)])
