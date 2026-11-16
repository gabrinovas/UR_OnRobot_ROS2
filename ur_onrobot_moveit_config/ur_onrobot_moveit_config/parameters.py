# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Parameters shared by the MoveIt nodes: the two descriptions and the files of this package."""

from launch.logging import get_logger
from launch.substitutions import Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue
from onrobot_description.config_loader import load_config, package_file, require
from ur_onrobot_description.description_command import description_name
from ur_onrobot_description.description_command import robot_description_command
from ur_workspace_description.robot_choice import report, resolve_robot

PACKAGE = 'ur_onrobot_moveit_config'


def moveit_config():
    """Return the content of config/moveit_launch.yaml of this package."""
    return load_config(package_file(PACKAGE, 'config', 'moveit_launch.yaml'))


def load_file(config, key):
    """Return the content of one of the YAML files listed under `files`."""
    content = load_config(package_file(PACKAGE, require(config, 'files', key)))
    content.pop('__file__')
    return content


def robot_description(values):
    """
    Return the robot_description parameter; values: the launch values as text.

    The address and the kinematics calibration of the real robot are chosen as the bringup does,
    so that MoveIt plans with the same kinematics the driver publishes.
    """
    simulated = values['use_fake_hardware'].lower() == 'true'
    choice = resolve_robot(values['cell'], values['robot_ip'], simulated)
    report(choice, get_logger('launch.user'))
    command = robot_description_command(
        cell=values['cell'], ur_type=values['ur_type'], onrobot_type=values['onrobot_type'],
        robot_ip=choice.address, use_fake_hardware=values['use_fake_hardware'],
        tf_prefix=values['tf_prefix'], include_gripper_ros2_control='true',
        kinematics_parameters_file=choice.calibration or None,
        gripper_ip_address=choice.compute_box or None)
    return {'robot_description': ParameterValue(Command(command), value_type=str)}


def semantic_description(config, values):
    """Return the robot_description_semantic parameter (the SRDF of the cell and the gripper)."""
    srdf = package_file(PACKAGE, require(config, 'files', 'srdf'))
    command = [FindExecutable(name='xacro'), ' ', srdf]
    arguments = {
        'robot_name': description_name(values['cell']),
        'onrobot_type': values['onrobot_type'],
        'cell': values['cell'],
        'tf_prefix': values['tf_prefix'],
    }
    for name, value in arguments.items():
        command += [f' {name}:=', value]
    return {'robot_description_semantic': ParameterValue(Command(command), value_type=str)}
