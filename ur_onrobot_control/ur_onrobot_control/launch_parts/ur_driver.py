# Copyright (c) 2021 PickNik, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#
#
# Derived from ur_robot_driver/launch/ur_control.launch.py. Changes: the description is the UR
# arm with an OnRobot gripper in a cell, the freedrive controller is loaded only on request, the
# nodes that need the real robot are not started in simulation, and every value comes from
# config/ur_control.yaml.
"""The nodes of the UR arm driver: control node, helper nodes of ur_robot_driver and RViz."""

from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import AndSubstitution, Command, LaunchConfiguration, NotSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from onrobot_description.config_loader import package_file, require
from ur_onrobot_control.config import is_true, launch_values, PACKAGE, section
from ur_onrobot_description.description_command import robot_description_command


def freedrive_requested(config, values):
    """Return whether the freedrive controller has to be loaded."""
    freedrive = require(config, 'controllers', 'freedrive')
    return is_true(values['load_freedrive_controller']) or \
        values['initial_joint_controller'] == freedrive


def robot_description(values):
    """Return the robot_description parameter of the arm with its gripper in its cell."""
    command = robot_description_command(
        cell=values['cell'],
        ur_type=values['ur_type'],
        onrobot_type=values['onrobot_type'],
        robot_ip=values['robot_ip'],
        use_fake_hardware=values['use_fake_hardware'],
        tf_prefix=values['tf_prefix'],
        include_gripper_ros2_control=values['include_gripper_ros2_control'],
        kinematics_parameters_file=values['kinematics_params_file'] or None,
        gripper_ip_address=values['gripper_ip_address'] or None)
    return {'robot_description': ParameterValue(Command(command), value_type=str)}


def control_parameters(config, values):
    """Return the parameters of the control node: description, update rate and controllers."""
    files = section(config, 'parameter_files')
    update_rate = package_file(
        PACKAGE, require(files, 'update_rate').format(ur_type=values['ur_type']))
    controllers = [ParameterFile(package_file(PACKAGE, path), allow_substs=True)
                   for path in require(files, 'controllers')]
    return [robot_description(values), update_rate, *controllers]


def ur_driver_nodes(context, config):
    """Return the nodes of the arm driver for the launch arguments of the context."""
    values = launch_values(context, config)
    fake = LaunchConfiguration('use_fake_hardware')
    headless = LaunchConfiguration('headless_mode')
    robot_ip = LaunchConfiguration('robot_ip')
    parameters = control_parameters(config, values)

    # FreedriveModeController of ur_controllers starts a logging thread when it is configured and
    # stops it only on cleanup: at shutdown its destructor blocks on that thread and the control
    # node hangs. While it is loaded the launch terminates the control node after a short wait; by
    # then the hardware has been deactivated. The wait is resolved here because the launch
    # evaluates it when it stops, outside the scope of the arguments of this file.
    stop = {'sigterm_timeout': values['freedrive_stop_timeout']} \
        if freedrive_requested(config, values) else {}

    def driver_node(executable, **options):
        return Node(package='ur_robot_driver', executable=executable, output='screen', **options)

    motion_action = require(config, 'trajectory_until', 'motion_action').format(
        controller=values['initial_joint_controller'])
    rviz = section(config, 'rviz_config')
    return [
        Node(package='controller_manager', executable='ros2_control_node', output='screen',
             parameters=parameters, condition=IfCondition(fake), **stop),
        driver_node('ur_ros2_control_node', parameters=parameters,
                    condition=UnlessCondition(fake), **stop),
        driver_node('dashboard_client', name='dashboard_client', emulate_tty=True,
                    parameters=[{'robot_ip': robot_ip}],
                    condition=IfCondition(AndSubstitution(
                        LaunchConfiguration('launch_dashboard_client'), NotSubstitution(fake)))),
        driver_node('robot_state_helper', name='ur_robot_state_helper',
                    parameters=[{'headless_mode': headless}, {'robot_ip': robot_ip}],
                    condition=UnlessCondition(fake)),
        driver_node('tool_communication.py', name='ur_tool_comm',
                    parameters=[{'robot_ip': robot_ip,
                                 'tcp_port': LaunchConfiguration('tool_tcp_port'),
                                 'device_name': LaunchConfiguration('tool_device_name')}],
                    condition=IfCondition(LaunchConfiguration('use_tool_communication'))),
        driver_node('controller_stopper_node', name='controller_stopper', emulate_tty=True,
                    parameters=[
                        {'headless_mode': headless},
                        {'joint_controller_active':
                            LaunchConfiguration('activate_joint_controller')},
                        {'consistent_controllers': require(config, 'controllers', 'consistent')}],
                    condition=UnlessCondition(fake)),
        driver_node('urscript_interface', parameters=[{'robot_ip': robot_ip}],
                    condition=UnlessCondition(fake)),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='both',
             parameters=[robot_description(values)],
             condition=IfCondition(LaunchConfiguration('launch_robot_state_publisher'))),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
             arguments=['-d', package_file(require(rviz, 'package'), require(rviz, 'file'))],
             condition=IfCondition(LaunchConfiguration('launch_rviz'))),
        driver_node('trajectory_until_node', name='trajectory_until_node',
                    parameters=[{
                        'motion_controller_uri': motion_action,
                        'until_action_uri': require(config, 'trajectory_until', 'until_action')}]),
    ]
