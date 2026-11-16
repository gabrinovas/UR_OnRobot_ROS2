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
# Derived from ur_robot_driver/launch/ur_control.launch.py. Changes: the lists of controllers
# come from config/ur_control.yaml and the freedrive controller is spawned only on request.
"""The controller spawners of the UR arm driver."""

from launch_ros.actions import Node
from onrobot_description.config_loader import require
from ur_onrobot_control.config import is_true, launch_values
from ur_onrobot_control.launch_parts.ur_driver import freedrive_requested


def controller_lists(config, values):
    """Return the controllers to activate and the ones to load inactive."""
    active = list(require(config, 'controllers', 'active'))
    inactive = list(require(config, 'controllers', 'inactive'))
    if not freedrive_requested(config, values):
        inactive.remove(require(config, 'controllers', 'freedrive'))
    if values['activate_joint_controller'] == 'true':
        active.append(values['initial_joint_controller'])
        inactive.remove(values['initial_joint_controller'])
    if is_true(values['use_fake_hardware']):
        for controller in require(config, 'controllers', 'real_robot_only'):
            active.remove(controller)
    return active, inactive


def controller_spawners(context, config):
    """Return one spawner for the active controllers and one for the inactive ones."""
    values = launch_values(context, config)
    active, inactive = controller_lists(config, values)
    common = ['--controller-manager', require(config, 'controller_manager'),
              '--controller-manager-timeout', values['controller_spawner_timeout']]
    return [
        Node(package='controller_manager', executable='spawner', arguments=common + active),
        Node(package='controller_manager', executable='spawner',
             arguments=common + ['--inactive'] + inactive),
    ]
