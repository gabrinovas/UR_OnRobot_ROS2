#!/usr/bin/env python3
# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Run the WebSocket bridge that lets Foxglove Studio (web or desktop) look at the cell."""

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from onrobot_description.config_loader import require
from onrobot_description.launch_arguments import declare_arguments
from ur_onrobot_control.config import control_config


def generate_launch_description():
    config = control_config('foxglove')
    parameters = dict(require(config, 'bridge'))
    for name in require(config, 'launch_arguments'):
        parameters[name] = LaunchConfiguration(name)
    return LaunchDescription(declare_arguments(config, 'launch_arguments') + [
        Node(package='foxglove_bridge', executable='foxglove_bridge', name='foxglove_bridge',
             output='screen', parameters=[parameters]),
    ])
