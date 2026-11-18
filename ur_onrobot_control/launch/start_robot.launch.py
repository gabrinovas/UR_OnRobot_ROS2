#!/usr/bin/env python3
# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Run a whole cell after asking how in a terminal window.

With interactive:=false, or without a terminal, it is ur_onrobot_bringup.launch.py with the
arguments given.
"""

import sys

from launch import LaunchDescription
from launch.actions import LogInfo, OpaqueFunction, Shutdown
from onrobot_description.config_loader import ConfigError
from onrobot_description.grippers import gripper_names
from onrobot_description.launch_arguments import declare_arguments
from ur_onrobot_control.config import control_config, include, is_true, launch_values, PACKAGE
from ur_onrobot_control.config import ur_types
from ur_workspace_description.cells import cell_names


def launch_setup(context, *args, **kwargs):
    config = control_config('bringup')
    values = launch_values(context, config)
    asked = launch_values(context, config, 'selector_launch_arguments')
    if is_true(asked['interactive']) and sys.stdin.isatty() and sys.stdout.isatty():
        try:
            from ur_onrobot_control.selector import RESULT_KEYS, run_selector
        except ImportError as error:
            raise ConfigError(
                f'[Fail-Fast] The selector cannot be loaded ({error}). Install textual or launch '
                'with interactive:=false') from error
        chosen = run_selector({key: values[key] for key in RESULT_KEYS})
        if chosen is None:
            return [LogInfo(msg='Launch cancelled in the selector'),
                    Shutdown(reason='cancelled in the selector')]
        values.update(chosen)
    return [
        LogInfo(msg='Launching with: ' + ', '.join(f'{k}={v}' for k, v in values.items() if v)),
        include(PACKAGE, 'ur_onrobot_bringup.launch.py', values),
    ]


def generate_launch_description():
    config = control_config('bringup')
    choices = {'ur_type': ur_types(), 'onrobot_type': gripper_names(), 'cell': cell_names()}
    arguments = declare_arguments(config, 'launch_arguments', choices=choices)
    arguments += declare_arguments(config, 'selector_launch_arguments')
    return LaunchDescription(arguments + [OpaqueFunction(function=launch_setup)])
