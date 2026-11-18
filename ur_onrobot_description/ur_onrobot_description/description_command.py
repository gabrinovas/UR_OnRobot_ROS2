# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Build the xacro command that generates the description of a UR arm with an OnRobot gripper.

Every launch file that needs the description calls robot_description_command(), so the list of
xacro arguments and the files behind them exist in one place.
"""

from launch.substitutions import FindExecutable
from onrobot_description.config_loader import load_config, package_file, require
from onrobot_description.launch_arguments import as_launch_text
import onrobot_driver
from ur_workspace_description.cells import load_cell

PACKAGE = 'ur_onrobot_description'
# Values of the gripper connection taken from the configuration of the gripper driver. The
# address of its Compute Box is not one of them: it belongs to the installation.
GRIPPER_CONNECTION = ('connection_type', 'device', 'port', 'device_address')


def description_config():
    """Return the content of config/ur_onrobot_description.yaml of this package."""
    return load_config(package_file(PACKAGE, 'config', 'ur_onrobot_description.yaml'))


def _file(entry, ur_type):
    """Return the absolute path of a {package, file} entry; {ur_type} in the path is replaced."""
    return package_file(require(entry, 'package'), require(entry, 'file').format(ur_type=ur_type))


def description_name(cell):
    """Return the name of the robot in the URDF and the SRDF of a cell."""
    return require(load_cell(cell), 'description_name')


def robot_description_command(*, cell, ur_type, onrobot_type, robot_ip, use_fake_hardware,
                              tf_prefix, include_gripper_ros2_control,
                              kinematics_parameters_file=None, gripper_ip_address=None):
    """
    Return the pieces of the xacro command, to be given to launch.substitutions.Command.

    cell and ur_type are plain text (they select files); the other values may be text or launch
    substitutions. kinematics_parameters_file replaces the kinematics of the configuration, for a
    robot with its own calibration. gripper_ip_address is the Compute Box of the gripper, which
    only a real gripper over tcp needs.
    """
    config = description_config()
    files = dict(require(config, 'description'), __file__=config['__file__'])
    robot = dict(require(load_cell(cell), 'robot'), __file__=load_cell(cell)['__file__'])
    gripper = require(onrobot_driver.driver_config(), 'launch_arguments')
    arguments = {
        'name': description_name(cell),
        'cell': cell,
        'ur_type': ur_type,
        'onrobot_type': onrobot_type,
        'robot_ip': robot_ip,
        'use_fake_hardware': use_fake_hardware,
        'tf_prefix': tf_prefix,
        'include_gripper_ros2_control': include_gripper_ros2_control,
        'joint_limits_parameters_file': _file(require(files, 'joint_limits'), ur_type),
        'kinematics_parameters_file':
            kinematics_parameters_file or _file(require(files, 'kinematics'), ur_type),
        'physical_parameters_file': _file(require(files, 'physical_parameters'), ur_type),
        'visual_parameters_file': _file(require(files, 'visual_parameters'), ur_type),
        'script_filename': _file(require(robot, 'script'), ur_type),
        'input_recipe_filename': _file(require(files, 'input_recipe'), ur_type),
        'output_recipe_filename': _file(require(files, 'output_recipe'), ur_type),
    }
    for name in GRIPPER_CONNECTION:
        arguments[name] = as_launch_text(require(gripper, name, 'default'))
    if gripper_ip_address:
        arguments['ip_address'] = gripper_ip_address

    command = [FindExecutable(name='xacro'), ' ', package_file(PACKAGE, require(files, 'file'))]
    for name, value in arguments.items():
        command += [f' {name}:=', value]
    return command
